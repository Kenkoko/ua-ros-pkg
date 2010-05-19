/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Antons Rebguns.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <background_filters/GetBgStats.h>

#include <sys/types.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include "background_filters/common.h"

using namespace std;

class BackgroundAverager
{
private:
    double delay;
    int num_samples;
    double scale;
    string colorspace;

    int img_width;
    int img_height;
    int img_depth;
    int img_n_chan;

    IplImage **samples;
    IplImage *avg_img;
    vector<float> std_dev;
    vector<float> cov_mat;
    vector<float> cov_mat_inv;
    vector<float> dets;
    int sample_counter;
    bool have_avg_img;

    ros::ServiceServer avg_img_srv;
    ros::Subscriber image_sub;

public:
    BackgroundAverager(ros::NodeHandle& nh)
    {
        ros::NodeHandle local_nh("~");
        local_nh.param("scale", scale, 1.0);
        local_nh.param("colorspace", colorspace, string("rgb"));
        local_nh.param("number_of_samples", num_samples, 10);
        local_nh.param("sampling_delay", delay, 0.0);

        samples = (IplImage **) calloc(num_samples, sizeof(IplImage *));
        sample_counter = 0;
        have_avg_img = false;

        image_sub = nh.subscribe("image", 1, &BackgroundAverager::handle_image, this);
        avg_img_srv = nh.advertiseService("get_background_stats", &BackgroundAverager::get_background_stats, this);
    }

    void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        img_width = (int) (msg_ptr->width / scale);
        img_height = (int) (msg_ptr->height / scale);

        if (sample_counter < num_samples)
        {
            sensor_msgs::CvBridge bridge;
            IplImage *sample = NULL;

            try
            {
                if (colorspace == "rgb")
                {
                    img_n_chan = 3;
                    img_depth = IPL_DEPTH_8U;
                    sample = cvCreateImage(cvSize(img_width, img_height), img_depth, img_n_chan);
                    cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), sample);
                }
                else if (colorspace == "hsv")
                {
                    img_n_chan = 3;
                    img_depth = IPL_DEPTH_8U;
                    sample = cvCreateImage(cvSize(img_width, img_height), img_depth, img_n_chan);
                    cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), sample);
                    cvCvtColor(sample, sample, CV_BGR2HSV);
                }
                else if (colorspace == "rgchroma")
                {
                    img_n_chan = 2;
                    img_depth = IPL_DEPTH_32F;
                    IplImage *img = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_8U, 3);
                    cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), img);
                    sample = cvCreateImage(cvSize(img_width, img_height), img_depth, img_n_chan);
                    convertToChroma(img, sample);
                    cvReleaseImage(&img);
                }

                samples[sample_counter++] = sample;

                if (delay > 0.0) { ros::Duration(delay).sleep(); }
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("CvBridgeError");
            }

            return;
        }

        if (!have_avg_img)
        {
            ROS_INFO("Collected samples for background averaging");
            ROS_INFO("Image: %dx%d with %d channels (depth is %d)", img_width, img_height, img_n_chan, img_depth);

            if (colorspace == "rgb") { process_bgr_images(); }
            else if (colorspace == "hsv") { process_hsv_images(); }
            else if (colorspace == "rgchroma") { process_rgchroma_images(); }

            have_avg_img = true;
            ROS_INFO("Computed average background from samples");
        }
        else
        {
            image_sub.shutdown();
        }
    }

    void calculate_avg_img(float alpha)
    {
        avg_img = cvCreateImage(cvSize(img_width, img_height), img_depth, img_n_chan);
        uchar* avg_data = (uchar *) avg_img->imageData;

        int pixel_num = img_width * img_height;

        // a img_n_chan x img_n_chan covariance matrix for each pixel in the image
        cov_mat.resize(pixel_num * (img_n_chan * img_n_chan));
        cov_mat_inv.resize(pixel_num * (img_n_chan * img_n_chan));
        dets.resize(pixel_num);
        std_dev.resize(pixel_num * img_n_chan);

        CvMat *ave = cvCreateMat(1, img_n_chan, CV_32FC1);
        CvMat *covMat = cvCreateMat(img_n_chan, img_n_chan, CV_32FC1);
        CvMat *covMatInv = cvCreateMat(img_n_chan, img_n_chan, CV_32FC1);
        CvMat **vects = (CvMat **) calloc(num_samples, sizeof(CvMat *));

        for (int i = 0; i < num_samples; ++i)
        {
            if (colorspace == "rgb" || colorspace == "hsv") { vects[i] = cvCreateMat(1, img_n_chan, CV_8UC1); }
            else if (colorspace == "rgchroma") { vects[i] = cvCreateMat(1, img_n_chan, CV_32FC1); }
        }

        for (int row = 0; row < img_height; ++row)
        {
            for (int col = 0; col < img_width; ++col)
            {
                for (int i = 0; i < num_samples; ++i)
                {
                    if (colorspace == "rgb" || colorspace == "hsv")
                    {
                        uchar* ptr = (uchar *) (samples[i]->imageData + row*(samples[i]->widthStep));

                        for (int ch = 0; ch < img_n_chan; ++ch)
                        {
                            cvSet1D(vects[i], ch, cvScalar(ptr[img_n_chan*col + ch]));
                        }
                    }
                    else if (colorspace == "rgchroma")
                    {
                        float* ptr = (float *) (samples[i]->imageData + row*(samples[i]->widthStep));

                        for (int ch = 0; ch < img_n_chan; ++ch)
                        {
                            cvSet1D(vects[i], ch, cvScalar(ptr[img_n_chan*col + ch]));
                        }
                    }
                }

                int pixel = row * img_width + col;

                if (pixel == 0)
                {
                    for (int i = 0; i < num_samples; ++i)
                    {
                        print_mat(vects[i]);
                    }

                    cout << "Printing covar mat" << endl;
                }

                cvCalcCovarMatrix((const CvArr**) vects, num_samples, covMat, ave, CV_COVAR_NORMAL);
                cvConvertScale(covMat, covMat, 1.0 / num_samples);

                if (pixel == 0)
                {
                    print_mat(covMat);
                    cout << endl;

                    cout << "Printing averages of input vectors" << endl;
                    print_mat(ave);
                    cout << endl;
                }

                // pad the diagonal of the covariance matrix to avoid 0's
                for (int ch = 0; ch < img_n_chan; ++ch)
                {
                    cvSet2D(covMat, ch, ch, cvScalar(cvGet2D(covMat, ch, ch).val[0] + alpha));
                }

                if (pixel == 0)
                {
                    cout << "Printing covar mat after adding alpha" << endl;
                    print_mat(covMat);
                    cout << endl;
                }

                dets[pixel] = cvInvert(covMat, covMatInv, CV_LU);

                if (pixel == 0)
                {
                    cout << "Printing covar mat after copying" << endl;
                }

                // copy over the covariance matrix
                for (int row = 0; row < covMat->rows; ++row)
                {
                    const float* ptr = (const float*) ((covMat->data.ptr) + row * (covMat->step));

                    for (int col = 0; col < covMat->cols; ++col)
                    {
                        cov_mat[(pixel*(img_n_chan*img_n_chan)) + (row*(covMat->cols)) + col] = *ptr++;

                        if (pixel == 0)
                        {
                            cout << cov_mat[(pixel*(img_n_chan*img_n_chan)) + (row*covMat->cols) + col] << " ";
                        }
                    }
                }

                if (pixel == 0)
                {
                    cout << endl;
                }

                // copy over the inverse covariance matrix
                for (int row = 0; row < covMatInv->rows; ++row)
                {
                    const float* ptr = (const float*) ((covMatInv->data.ptr) + row * (covMatInv->step));

                    for (int col = 0; col < covMatInv->cols; ++col)
                    {
                        cov_mat_inv[(pixel*(img_n_chan*img_n_chan)) + (row*(covMatInv->cols)) + col] = *ptr++;
                    }
                }

                // copy over the average data
                for (int row = 0; row < ave->rows; ++row)
                {
                    const float* ptr = (const float*) ave->data.ptr + row * ave->step;

                    for (int col = 0; col < ave->cols; ++col)
                    {
                        if (colorspace == "rgb" || colorspace == "hsv")
                        {
                            float t = *ptr++;
                            avg_data[(pixel*img_n_chan) + (row*(ave->cols)) + col] = t;
                            cout << "[" << pixel << "] " << t << " ?==? " << (float) avg_data[pixel*img_n_chan + row*ave->cols + col] << endl;
                        }
                        else if (colorspace == "rgchroma")
                        {
                            ((float *) avg_img->imageData)[(pixel*img_n_chan) + (row*(ave->cols)) + col] = *ptr++;
                        }
                    }
                }

                // get standard deviations from the diagonal of covariance matrix
                for (int ch = 0; ch < img_n_chan; ++ch)
                {
                    std_dev[pixel*img_n_chan + ch] = sqrt(cvGet2D(covMat, ch, ch).val[0]);
                }
            }
        }

        cout << "Right after copy" << endl;
        print_img(avg_img);
//         CvMat *temp = cvCreateMat(avg_img->height, avg_img->width, CV_8UC3);
//         cvGetMat(avg_img, temp);
// 
//         for (int row = 0; row < temp->rows; ++row)
//         {
//             uchar* ptr = (uchar*) temp->data.ptr + row * temp->step;
// 
//             for (int col = 0; col < temp->cols; ++col)
//             {
//                 uchar t = *ptr++;
//                 cout << "[" << (row*(temp->cols) + col) << "] = {r: " << row << ", c: " << col << "} " << (float) t << endl;
//             }
//         }
// 
//         cout << "h: " << temp->rows << ", w: " << temp->cols << endl;


        for (int i = 0; i < num_samples; ++i)
        {
            cvReleaseMat(&vects[i]);
            cvReleaseImage(&samples[i]);
        }

        cvReleaseMat(&ave); ave = NULL;
        cvReleaseMat(&covMat); covMat = NULL;
        cvReleaseMat(&covMatInv); covMatInv = NULL;

        if (vects) free(vects);
        if (samples) free(samples);
    }

    void process_bgr_images()
    {
        cout << "colorspace = bgr" << endl;
        calculate_avg_img(1.0);
    }

    void process_hsv_images()
    {
        cout << "colorspace = hsv" << endl;
        calculate_avg_img(50.0);
    }

    void process_rgchroma_images()
    {
        cout << "colorspace = rgchroma" << endl;
        calculate_avg_img(0.001);
    }

    bool get_background_stats(background_filters::GetBgStats::Request& request, background_filters::GetBgStats::Response& response)
    {
        cout << "In service" << endl;
        print_img(avg_img);

//         CvMat *temp = cvCreateMat(avg_img->height, avg_img->width, CV_8UC3);
//         cvGetMat(avg_img, temp);
// 
//         for (int row = 0; row < temp->rows; ++row)
//         {
//             uchar* ptr = (uchar*) temp->data.ptr + row * temp->step;
// 
//             for (int col = 0; col < temp->cols; ++col)
//             {
//                 uchar t = *ptr++;
//                 cout << "[" << (row*(temp->cols) + col) << "] = {r: " << row << ", c: " << col << "} " << (float) t << endl;
//             }
//         }
// 
//         cout << "h: " << temp->rows << ", w: " << temp->cols << endl;

        response.colorspace = colorspace;
        sensor_msgs::CvBridge::fromIpltoRosImage(avg_img, response.average_background);
        response.covariance_matrix = cov_mat;
        response.covariance_matrix_inv = cov_mat_inv;
        response.covariance_matrix_dets = dets;
        response.standard_deviations = std_dev;

        return true;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_sampler");
    ros::NodeHandle n;

    BackgroundAverager ba(n);
    ros::spin();

    return 0;
}

