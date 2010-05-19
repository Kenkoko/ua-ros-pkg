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

#include <iostream>
#include <cstdio>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

void print_mat(CvMat *mat)
{
    for (int i = 0; i < mat->rows; i++)
    {
        printf("\n");

        switch (CV_MAT_DEPTH(mat->type))
        {
            case CV_32F:
            case CV_64F:
                for (int j = 0; j < mat->cols; j++)
                printf ("%8.6f ", (float) cvGetReal2D(mat, i, j));
                break;
            case CV_8U:
            case CV_16U:
                for(int j = 0; j < mat->cols; j++)
                printf ("%6d", (int) cvGetReal2D(mat, i, j));
                break;
            default:
            break;
        }
    }

    printf("\n");
}

void print_img(IplImage *img)
{
    for (int y = 0; y < img->height; ++y)
    {
        float* ptr = (float*) (img->imageData + y * img->widthStep);

        for (int x = 0; x < img->width; ++x)
        {
            int pixel = y * img->width + x;
            std::cout << "[" << pixel << "] = " << "(" << ptr[2*x+0] << ", " << ptr[2*x+1] << ")" << std::endl;
        }
    }
}

void convertToChroma(IplImage *in_bgr, IplImage *out_rgchroma)
{
    int img_height = in_bgr->height;
    int img_width = in_bgr->width;

    for (int y = 0; y < img_height; ++y)
    {
        uchar* ptr = (uchar *) (in_bgr->imageData + y * in_bgr->widthStep);
        float* out_ptr = (float *) (out_rgchroma->imageData + y * out_rgchroma->widthStep);

        for (int x = 0; x < img_width; ++x)
        {
            float b = ptr[3*x+0];
            float g = ptr[3*x+1];
            float r = ptr[3*x+2];

            out_ptr[2*x+0] = r / (1 + b + g + r);
            out_ptr[2*x+1] = g / (1 + b + g + r);
        }
    }
}
