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

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include "background_filters/common.h"

#define TWO_PI 6.28318531

using namespace std;

IplImage *ave_bg;

vector<float> cov_mats;
vector<float> cov_mats_inv;
vector<float> dets;
vector<float> std_dev;

ros::Subscriber image_sub;
ros::Publisher prob_img_pub;

string colorspace;

bool first = true;

void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    sensor_msgs::CvBridge bridge;
    IplImage *bg = NULL;

    try
    {
        if (colorspace == "rgb")
        {
            bg = cvCreateImage(cvGetSize(ave_bg), IPL_DEPTH_8U, 3);
            cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), bg);
        }
        else if (colorspace == "hsv")
        {
            bg = cvCreateImage(cvGetSize(ave_bg), IPL_DEPTH_8U, 3);
            cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), bg);
            cvCvtColor(bg, bg, CV_BGR2HSV);
        }
        else if (colorspace == "rgchroma")
        {
            IplImage *img = cvCreateImage(cvGetSize(ave_bg), IPL_DEPTH_8U, 3);
            cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), img);
            bg = cvCreateImage(cvGetSize(ave_bg), IPL_DEPTH_32F, 2);
            convertToChroma(img, bg);
            cvReleaseImage(&img);
        }
    }
    catch (sensor_msgs::CvBridgeException error)
    {
        ROS_ERROR("CvBridgeError");
    }

    IplImage *prob_img = cvCreateImage(cvGetSize(bg), IPL_DEPTH_32F, 1);
    float *prob_data = (float *) prob_img->imageData;

    int width = bg->width;
    int height = bg->height;

    CvMat *bgr_new = NULL;
    CvMat *bgr_ave = NULL;
    CvMat *inv_cov = NULL;

    if (colorspace == "rgb" || colorspace == "hsv")
    {
        bgr_new = cvCreateMat(1, 3, CV_32FC1);
        bgr_ave = cvCreateMat(1, 3, CV_32FC1);
        inv_cov = cvCreateMatHeader(3, 3, CV_32FC1);
    }
    else if (colorspace == "rgchroma")
    {
        bgr_new = cvCreateMat(1, 2, CV_32FC1);
        bgr_ave = cvCreateMat(1, 2, CV_32FC1);
        inv_cov = cvCreateMatHeader(2, 2, CV_32FC1);
    }

    for (int y = 0; y < height; ++y)
    {
        if (colorspace == "rgb" || colorspace == "hsv")
        {
            uchar* ptr_bg = (uchar *) (bg->imageData + y * bg->widthStep);
            uchar* ptr_ave = (uchar *) (ave_bg->imageData + y * ave_bg->widthStep);

            for (int x = 0; x < width; ++x)
            {
                cvSet1D(bgr_new, 0, cvScalar(ptr_bg[3*x+0]));
                cvSet1D(bgr_new, 1, cvScalar(ptr_bg[3*x+1]));
                cvSet1D(bgr_new, 2, cvScalar(ptr_bg[3*x+2]));

                cvSet1D(bgr_ave, 0, cvScalar(ptr_ave[3*x+0]));
                cvSet1D(bgr_ave, 1, cvScalar(ptr_ave[3*x+1]));
                cvSet1D(bgr_ave, 2, cvScalar(ptr_ave[3*x+2]));

                cvInitMatHeader(inv_cov, 3, 3, CV_32FC1, &cov_mats_inv[(y*width+x)*9]);

                double mah_dist = cvMahalanobis(bgr_new, bgr_ave, inv_cov);
                double unnorm_gaussian = exp(-0.5 * mah_dist);
                double partition = 1.0 / (pow(TWO_PI, 1.5) * sqrt(dets[y*width+x]));
                float p = partition * unnorm_gaussian;

                prob_data[y*width+x] = p;
            }
        }
        else if (colorspace == "rgchroma")
        {
            float* ptr_bg = (float *) (bg->imageData + y * bg->widthStep);
            float* ptr_ave = (float *) (ave_bg->imageData + y * ave_bg->widthStep);

            for (int x = 0; x < width; ++x)
            {
                cvSet1D(bgr_new, 0, cvScalar(ptr_bg[2*x+0]));
                cvSet1D(bgr_new, 1, cvScalar(ptr_bg[2*x+1]));

                cvSet1D(bgr_ave, 0, cvScalar(ptr_ave[2*x+0]));
                cvSet1D(bgr_ave, 1, cvScalar(ptr_ave[2*x+1]));

                cvInitMatHeader(inv_cov, 2, 2, CV_32FC1, &cov_mats_inv[(y*width+x)*4]);

                double mah_dist = cvMahalanobis(bgr_new, bgr_ave, inv_cov);
                double unnorm_gaussian = exp(-0.5 * mah_dist);
                double partition = 1.0 / (TWO_PI * sqrt(dets[y*width+x]));
                float p = partition * unnorm_gaussian;

                //if (y*width+x < 100) cout << mah_dist << ", " << p << endl;
                prob_data[y*width+x] = p;

                if ((y*width+x) < 100)
                {
                    cout << "[" << y*width+x << "] = " << p << endl;
                    cout << ptr_ave[2*x+0] << endl;
                    cout << ptr_ave[2*x+1] << endl;
                }

            }
        }
    }

    cvReleaseMat(&bgr_new);
    cvReleaseMat(&bgr_ave);
    cvReleaseMat(&inv_cov);

    double min, max;
    cvLog(prob_img, prob_img);
    cvMinMaxLoc(prob_img, &min, &max);
    cvConvertScale(prob_img, prob_img, 1.0 / (max - min), -min / (max - min));

    cvShowImage("prob_img", prob_img);

    prob_img_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(prob_img));

    cvReleaseImage(&bg);
    prob_data = NULL;
    cvReleaseImage(&prob_img);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_sub");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<background_filters::GetBgStats>("get_background_stats");

    background_filters::GetBgStats srv;
    sensor_msgs::CvBridge b;

    cvNamedWindow("prob_img");

    if (client.call(srv))
    {
        cout << "calling service" << endl;

        b.fromImage(srv.response.average_background);
        ave_bg = b.toIpl();

        //print_img(ave_bg);
//         CvMat *temp = cvCreateMat(ave_bg->height, ave_bg->width, CV_8UC3);
//         cvGetMat(ave_bg, temp);
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
/*
        cout << "h: " << temp->rows << ", w: " << temp->cols << endl;*/

        colorspace = srv.response.colorspace;
        cov_mats = srv.response.covariance_matrix;
        cov_mats_inv = srv.response.covariance_matrix_inv;
        dets = srv.response.covariance_matrix_dets;
        std_dev = srv.response.standard_deviations;

        cout << "done calling service" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service get_bg_stats");
        return 1;
    }

    cout << "subscribing" << endl;
    image_sub = n.subscribe("image", 1, handle_image);
    ros::Publisher ave_bg_pub = n.advertise<sensor_msgs::Image>("average_bg_service", 1);
    prob_img_pub = n.advertise<sensor_msgs::Image>("prob_img", 1);
    cout << "done subscribing" << endl;

//    ros::Rate r(1.0);
//    while (ros::ok())
//    {
//        ave_bg_pub.publish(b.cvToImgMsg(ave_bg, "bgr8"));
//        ros::spinOnce();
//        r.sleep();
//    }

    cvStartWindowThread();
    ros::spin();

    return 0;
}
