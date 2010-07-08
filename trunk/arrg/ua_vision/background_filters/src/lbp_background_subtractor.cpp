/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Ian Fasel.
*  Copyright (C) 2002, Maenpaa.
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


// Implementation of algorithm from
// Heikkila, M., & Pietikainen, M. (2006). A Texture-Based Method for Modeling the
// Background and Detecting Moving Objects. IEEE Transactions on Pattern Analysis and
// Machine Intelligence, 28(4). http://portal.acm.org/citation.cfm?id=1115789.
//
// LBP code is modified from http://www.ee.oulu.fi/~topiolli/cpplibs/files/


#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <set>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <background_filters/lbp_background_subtractor.h>

using namespace cv;
using namespace std;

LBPModel::LBPModel(int nframes) : frame_num(0), num_model_frames(nframes) {}

void LBPModel::setFrameNum( int fr )
{
    frame_num = fr;
}

void LBPModel::initialize(Mat& frame )
{
    getGrayIntFrame(frame);
    last_lbp = Mat(int_frame.rows, int_frame.cols, CV_8U);
    ComputeLBP(int_frame, last_lbp);
}

void LBPModel::update( Mat& frame )
{
    getGrayIntFrame(frame);
    Mat current_lbp = Mat(int_frame.rows, int_frame.cols, CV_8U);
    ComputeLBP(int_frame, current_lbp, false);
    //simpleNeighborhood(current_lbp);
    //imshow("LBP", foreground);
    regionHistogram(current_lbp);
}

void LBPModel::getGrayIntFrame(Mat& mat_frame)
{
    cvtColor(mat_frame, gray_frame, CV_BGR2GRAY);
    resize(gray_frame, mat_frame, Size(gray_frame.size().width/framescale, gray_frame.size().height/framescale));
    GaussianBlur(mat_frame, mat_frame, Size(3,3), .95, .95);
    mat_frame.convertTo(int_frame, CV_32S);
}

void LBPModel::regionHistogram( Mat &current_lbp )
{
    // Parameters from paper
    unsigned int max_num_histograms = 4;  // In paper this was K, num histograms
    double rate_a = 0.01;  // In paper this was a_b, learning rate to adapt histograms
    double rate_b = 0.01;  // In paper this was a_w, learning rate to adapt weights of histograms
    // Below: Lower = more background (and holes), Higher = more noise
    double Tb = 0.65; //0.4-.8 // threshold for match to background model. Lower = easier to be bg = more black
    double Tp = 0.7; //0.65 // threshold for any histogram match. Lower = more variation is ok = black.
    int region_out = 3;    // In paper R_region = region_out*2+1.  Paper was 9, so region_out = 4

    // You can pass in a number to tell it to stop updating frames
    bool do_updates = true;
    if(num_model_frames > 0 && frame_num >= num_model_frames){
        do_updates = false;
    }

    //int region_out = 2; // 2 = (2 + 1 + 2) x (2 + 1 + 2) = 5 x 5 = 25 elements
    int region_step = region_out; // could take step sizes different from region half-size
    int region_size = (region_out*2+1)*(region_out*2+1);
    int num_regions = (current_lbp.rows / region_step) * (current_lbp.cols / region_step);

    // Mark everything as foreground
    foreground = Mat::zeros(current_lbp.rows, current_lbp.cols, CV_8U);
    Mat fg2 = Mat(foreground, Rect(region_out*2, region_out*2, current_lbp.cols-region_out*4, current_lbp.rows-region_out*4));
    fg2 = 255;
    Mat masker = Mat(region_out*2, region_out*2, CV_8U);
    masker = 80;

    // Pre-allocate region histogram sets if they don't already exist
    if(region_histogram_sets.size() == 0){
        region_histogram_sets.resize( num_regions );
        region_histogram_weights.resize( num_regions );
        region_histogram_weight_inds.resize( num_regions );
    }

    // Now step through the image regions and collect & compare histograms
    list< vector<Mat> >::iterator region_hists = region_histogram_sets.begin();
    list< vector<double> >::iterator region_weights = region_histogram_weights.begin();
    list< set<int> >::iterator region_weight_inds = region_histogram_weight_inds.begin();
    for( unsigned int r = region_out; r <= current_lbp.rows-region_out; r+= region_step){
        for( unsigned int c = region_out; c <= current_lbp.cols-region_out; c+= region_step){
            // Calculate the current region histogram
            Mat cur_hist = Mat::zeros(1, 256, CV_8U);
            uint8_t *h = cur_hist.ptr<uint8_t>(0);
            for(unsigned int region_row = r-region_out; region_row < r+region_out+1; ++region_row){
                uint8_t *rowptr = current_lbp.ptr<uint8_t>(region_row);
                for(unsigned int region_col = c-region_out; region_col < c+region_out+1; ++region_col){
                    uint8_t val = rowptr[region_col];
                    //if(val > 0){ // 0 is magic!
                    h[val]++;
                    //}
                }
            }

            // Calculate distances between cur_hist and stored histograms for this region
            vector<int> distances(region_hists->size());
            for(unsigned int i=0; i < region_hists->size(); ++i){
                Mat mins = min(cur_hist, (*region_hists)[i]); // element-wise minimum
                distances[i] = sum(mins)[0]; // add up across elements
            }

            //OpenCV minmaxind crashes me hard, so do it by hand
            int minIdx=0, maxIdx=0;
            int minVal=1000, maxVal=0;
            for(unsigned int i = 0; i < distances.size(); ++i){
                int d = distances[i];
                if(d > maxVal){
                    maxIdx = i;
                    maxVal = d;
                } else if(d <= minVal){
                    minIdx = i;
                    minVal = d;
                }
            }

            // Check if any of them are above threshold.  If so, update stats.
            int hist_distance_thresh = region_size * Tp;
            if(maxVal >= hist_distance_thresh){

                // Is best matching histogram a background hist?
                if( region_weight_inds->find(maxIdx) != region_weight_inds->end() ){
                    // If found in background set, paint image as background
                    Mat R = Mat(foreground, Rect(std::max(0,(int)c-region_out-1), r-region_out, region_out*2, region_out*2));
                    R -= masker;
                }

                if(do_updates){
                    // Update the best matching histogram
                    (*region_hists)[maxIdx] =  (*region_hists)[maxIdx] * (1-rate_a) + cur_hist * rate_a;

                    // Update the weights for each histogram
                    for(int i=0; i < region_hists->size(); ++i){
                        (*region_weights)[i] = (*region_weights)[i] * (1-rate_b) + ((i==maxIdx) ? rate_b : 0);
                    }
                }

            } else if(region_hists->size() < max_num_histograms && do_updates){
                // If we didn't match and there are fewer than max, just add it to list
                region_hists->push_back( cur_hist );
                region_weights->push_back( 0.01 );
            } else if ( do_updates) {
                // otherwise, delete the lowest scoring one and replace with the new one
                (*region_hists)[minIdx] = cur_hist;
                (*region_weights)[minIdx] = 0.01;
            }

            if (do_updates){
                // Select which histograms are frequent enough to be called "background"
                Mat inds;
                Mat region_weights_mat(1, (*region_weights).size(), CV_64F, (*region_weights).data());
                sortIdx(region_weights_mat, inds, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);
                int *indp = inds.ptr<int>(0);

                set< int > winds;
                double s = 0.0;
                //cout << "s: ";
                for(unsigned int i = 0; i < region_hists->size(); ++i){
                    int j = indp[i];
                    s += (*region_weights)[j];
                    winds.insert(j);
                    if(s > Tb){
                        break;
                    }
                }
                if(s > Tb)
                    *region_weight_inds = winds;
                else{
                    set< int > nothing;
                    *region_weight_inds = nothing;
                }
            }

            // Finally, increment region iterators
            region_hists++;
            region_weights++;
            region_weight_inds++;
        }
    }
}

void LBPModel::simpleNeighborhood( Mat &current_lbp )
{
    // Compare last LBP to nearby shifted LBP (currently the 3x3 region surrounding)
    // First get center region of last_lbp
    Mat L = Mat(last_lbp, Rect(1,1,last_lbp.cols-2,last_lbp.rows-2));

    // Now for each shifted rgion of result_lbp, compare
    Mat diffs[9];
    int i = 0;
    for (int r = 0; r < 3; ++r){
        for(int c = 0; c < 3; ++c){
            Mat R = Mat(current_lbp, Rect(c,r,current_lbp.cols-2, current_lbp.rows-2));
            absdiff(L, R, diffs[i]);
            i++;
        }
    }
    foreground = diffs[0];
    for (int j = 1; j < 9; ++j){
        foreground = min(foreground, diffs[j]);
    }
    threshold(foreground, foreground, 1, 255, THRESH_BINARY);
    last_lbp = current_lbp;
}

// This version allows you to use 5, 6, 7, or 8 points around the center pixel.
// However it does not do interpolation or allow you to have a radius greater than 1.
// I've only tested it with 6 and 8, which work fine.  Code for the bilinear interpolation
// and other variations is available from the same place the original LPB code came from.
void LBPModel::ComputeLBP(Mat &input, Mat &result, bool use59){
    int rows = input.rows;
    int columns = input.cols;
    int pred2 = predicate * 2;
    int r,c;
    unsigned int X[8], Y[8];
    unsigned int numbits = NUMBITS;

    // Start from top left corner
    //cout << "predicate: " << predicate << endl;
    for(unsigned int p = 0; p < NUMBITS; ++p){
        X[p] = (unsigned int)( predicate * cos(2.0 * PI * (p / (float)NUMBITS) ) + predicate + 0.5);
        Y[p] = (unsigned int)( predicate * sin(2.0 * PI * (p / (float)NUMBITS) ) + predicate + 0.5);
        //cout << "X, Y: " << X[p] << ", " << Y[p] << endl;
    }
    for (r=0;r<rows-pred2;r++)
    {
        //Set up a circularly indexed neighborhood using nine pointers.
        int
        #if NUMBITS > 7
        *p7 = input.ptr<int>(Y[7]+r) + X[7],
        #endif
        #if NUMBITS > 6
        *p6 = input.ptr<int>(Y[6]+r) + X[6],
        #endif
        #if NUMBITS > 5
        *p5 = input.ptr<int>(Y[5]+r) + X[5],
        #endif
        #if NUMBITS > 4
        *p4 = input.ptr<int>(Y[4]+r) + X[4],
        #endif
        *p3 = input.ptr<int>(Y[3]+r) + X[3],
        *p2 = input.ptr<int>(Y[2]+r) + X[2],
        *p1 = input.ptr<int>(Y[1]+r) + X[1],
        *p0 = input.ptr<int>(Y[0]+r) + X[0],
        *center = input.ptr<int>(r+predicate)+predicate;
        unsigned int value;
        uint8_t *rptr = result.ptr<uint8_t>(r);

        for (c=0;c<columns-pred2;c++)
        {
            value = 0;

            /* Unrolled loop */
            #if NUMBITS > 7
            compab_mask_inc(p7,7);
            #endif
            #if NUMBITS > 6
            compab_mask_inc(p6,6);
            #endif
            #if NUMBITS > 5
            compab_mask_inc(p5,5);
            #endif
            #if NUMBITS > 4
            compab_mask_inc(p4,4);
            #endif
            compab_mask_inc(p3,3);
            compab_mask_inc(p2,2);
            compab_mask_inc(p1,1);
            compab_mask_inc(p0,0);
            center++;

            if(use59){
                *rptr = (uint8_t)UniformPattern59[value];
            } else {
                *rptr = (uint8_t)value;
            }
            rptr++;
        }
    }
}
