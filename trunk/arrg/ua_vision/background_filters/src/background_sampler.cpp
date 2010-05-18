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

    IplImage **bgs;
    IplImage *ave_bg;
    vector<float> std_dev;
    vector<float> cov_mat;
    vector<float> cov_mat_inv;
    vector<float> dets;
    int bg_counter;
    bool have_ave_bg;

    ros::ServiceServer service;
    ros::Subscriber image_sub;
    ros::Publisher ave_bg_pub;

public:
    BackgroundAverager(ros::NodeHandle& nh)
    {
        ros::NodeHandle ln("~");
        ln.param("scale", scale, 1.0);
        ln.param("colorspace", colorspace, string("rgb"));
        ln.param("number_of_samples", num_samples, 10);
        ln.param("sampling_delay", delay, 0.0);

        bgs = (IplImage **) calloc(num_samples, sizeof(IplImage *));
        bg_counter = 0;
        have_ave_bg = false;

        image_sub = nh.subscribe("image", 1, &BackgroundAverager::handle_image, this);
        ave_bg_pub = nh.advertise<sensor_msgs::Image>("average_bg", 1);
        service = nh.advertiseService("get_background_stats", &BackgroundAverager::get_background_stats, this);
    }

    void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        img_width = (int) (msg_ptr->width / scale);
        img_height = (int) (msg_ptr->height / scale);

        if (bg_counter < num_samples)
        {
            sensor_msgs::CvBridge bridge;
            IplImage *bg = NULL;

            try
            {
                if (colorspace == "rgb")
                {
                    img_n_chan = 3;
                    img_depth = IPL_DEPTH_8U;
                    bg = cvCreateImage(cvSize(img_width, img_height), img_depth, img_n_chan);
                    cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), bg);
                }
                else if (colorspace == "hsv")
                {
                    img_n_chan = 3;
                    img_depth = IPL_DEPTH_8U;
                    bg = cvCreateImage(cvSize(img_width, img_height), img_depth, img_n_chan);
                    cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), bg);
                    cvCvtColor(bg, bg, CV_BGR2HSV);
                }
                else if (colorspace == "rgchroma")
                {
                    img_n_chan = 2;
                    img_depth = IPL_DEPTH_32F;
                    IplImage *img = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_8U, 3);
                    cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), img);
                    bg = cvCreateImage(cvSize(img_width, img_height), img_depth, img_n_chan);
                    convertToChroma(img, bg);
                    cvReleaseImage(&img);
                }

                bgs[bg_counter++] = bg;

                if (delay > 0.0) { ros::Duration(delay).sleep(); }
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("CvBridgeError");
            }

            return;
        }

        if (!have_ave_bg)
        {
            ROS_INFO("Collected samples for background avergaing");
            ROS_INFO("Image: %dx%d with %d channels (depth is %d)", img_width, img_height, img_n_chan, img_depth);

            if (colorspace == "rgb") { process_bgr_images(); }
            else if (colorspace == "hsv") { process_hsv_images(); }
            else if (colorspace == "rgchroma") { process_rgchroma_images(); }

            have_ave_bg = true;
            ROS_INFO("Computed average background from samples");
        }
        else
        {
            image_sub.shutdown();
        }
    }

    void process_image(float alpha)
    {
        ave_bg = cvCreateImage(cvSize(img_width, img_height), img_depth, img_n_chan);

        // figure out the actual size of image array
        int size = img_width * img_height * img_n_chan;

        // a img_n_chan x img_n_chan covariance matrix for each pixel in the image
        cov_mat.resize(img_width * img_height * (img_n_chan * img_n_chan));
        cov_mat_inv.resize(img_width * img_height * (img_n_chan * img_n_chan));
        dets.resize(img_width * img_height);
        std_dev.resize(size);

        CvMat *ave = cvCreateMat(1, img_n_chan, CV_32FC1);
        CvMat *covMat = cvCreateMat(img_n_chan, img_n_chan, CV_32FC1);
        CvMat *covMatInv = cvCreateMat(img_n_chan, img_n_chan, CV_32FC1);
        CvMat **vects = (CvMat **) calloc(num_samples, sizeof(CvMat *));

        for (int i = 0; i < num_samples; ++i)
        {
            if (colorspace == "rgb" || colorspace == "hsv") { vects[i] = cvCreateMat(1, img_n_chan, CV_8UC1); }
            else if (colorspace == "rgchroma") { vects[i] = cvCreateMat(1, img_n_chan, CV_32FC1); }
        }

        for (int y = 0; y < img_height; ++y)
        {
            for (int x = 0; x < img_width; ++x)
            {
                for (int j = 0; j < num_samples; ++j)
                {
                    if (colorspace == "rgb" || colorspace == "hsv")
                    {
                        uchar* ptr = (uchar *) (bgs[j]->imageData + y * bgs[j]->widthStep);

                        for (int k = 0; k < img_n_chan; ++k)
                        {
                            cvSet1D(vects[j], k, cvScalar(ptr[img_n_chan*x + k]));
                        }
                    }
                    else if (colorspace == "rgchroma")
                    {
                        float* ptr = (float *) (bgs[j]->imageData + y * bgs[j]->widthStep);

                        for (int k = 0; k < img_n_chan; ++k)
                        {
                            cvSet1D(vects[j], k, cvScalar(ptr[img_n_chan*x + k]));
                        }
                    }
                }

                int pixel = y * img_width + x;

                if (pixel == 0)
                {
                    for (int j = 0; j < num_samples; ++j)
                    {
                        print_mat(vects[j]);
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
                for (int k = 0; k < img_n_chan; ++k)
                {
                    cvSet2D(covMat, k, k, cvScalar(cvGet2D(covMat, k, k).val[0] + alpha));
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
                    const float* ptr = (const float*) (covMat->data.ptr + row * covMat->step);

                    for (int col = 0; col < covMat->cols; ++col)
                    {
                        cov_mat[pixel*(img_n_chan*img_n_chan) + row*covMat->cols + col] = *ptr++;

                        if (pixel == 0)
                        {
                            cout << cov_mat[pixel*(img_n_chan*img_n_chan) + row*covMat->cols + col] << " ";
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
                    const float* ptr = (const float*) (covMatInv->data.ptr + row * covMatInv->step);

                    for (int col = 0; col < covMatInv->cols; ++col)
                    {
                        cov_mat_inv[pixel*(img_n_chan*img_n_chan) + row*covMatInv->cols + col] = *ptr++;
                    }
                }

                // copy over the average data
                for (int row = 0; row < ave->rows; ++row)
                {
                    const float* ptr = (const float*) (ave->data.ptr + row * ave->step);

                    for (int col = 0; col < ave->cols; ++col)
                    {
                        if (colorspace == "rgb" || colorspace == "hsv")
                        {
                            ((uchar *) ave_bg->imageData)[pixel*img_n_chan + row*ave->cols + col] = *ptr++;
                        }
                        else if (colorspace == "rgchroma")
                        {
                            ((float *) ave_bg->imageData)[pixel*img_n_chan + row*ave->cols + col] = *ptr++;
                        }
                    }
                }

                // get standard deviations from the diagonal of covariance matrix
                for (int k = 0; k < img_n_chan; ++k)
                {
                    std_dev[pixel*img_n_chan + k] = sqrt(cvGet2D(covMat, k, k).val[0]);
                }
            }
        }

        for (int i = 0; i < num_samples; ++i)
        {
            cvReleaseMat(&vects[i]);
            cvReleaseImage(&bgs[i]);
        }

        cvReleaseMat(&ave);
        cvReleaseMat(&covMat);
        cvReleaseMat(&covMatInv);
        ave = NULL;
        covMat = NULL;
        covMatInv = NULL;
        if (vects) free(vects);
        if (bgs) free(bgs);
    }

    void process_bgr_images()
    {
        cout << "colorspace = bgr" << endl;
        process_image(1.0);
    }

    void process_hsv_images()
    {
        cout << "colorspace = hsv" << endl;
        process_image(50.0);
    }

    void process_rgchroma_images()
    {
        cout << "colorspace = rgchroma" << endl;
        process_image(0.001);
    }

    bool get_background_stats(background_filters::GetBgStats::Request& request, background_filters::GetBgStats::Response& response)
    {
        print_img(ave_bg);

        response.colorspace = colorspace;
        sensor_msgs::CvBridge::fromIpltoRosImage(ave_bg, response.average_background);
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

