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


#ifndef LBP_BACKGROUND_SUBTRACTOR_H
#define LBP_BACKGROUND_SUBTRACTOR_H

#include <vector>
#include <list>
#include <set>

#include <opencv2/core/core.hpp>

// Compare a value pointed to by 'ptr' to the 'center' value and
// increment pointer. Comparison is made by masking the most
// significant bit of an integer (the sign) and shifting it to an
// appropriate position.

// Higher threshold makes more resistant to pixel noise, but loses information.
// #define pix_thr 1
#define pix_thr 3
#define compab_mask_inc(ptr,shift) { value |= ((unsigned int)(*center - *ptr - pix_thr) & 0x80000000) >> (31-shift); ptr++; }
/* Compare a value 'val' to the 'center' value. */
#define compab_mask(val,shift) { value |= ((unsigned int)(*center - (val) - pix_thr) & 0x80000000) >> (31-shift); }

/* Predicate 1 for the 3x3 neighborhood */
#define predicate 1
/* The number of bits */
#define bits 8

#define framescale 1.0

#define DIST 1
#define NUMBITS 6
#define PI 3.1415926535897932


// uniform pattern for LBP for 8 neighboring points
uint8_t  UniformPattern59[256]={
    1,   2,   3,   4,   5,   0,   6,   7,   8,   0,   0,   0,   9,   0,  10,  11,
    12,   0,   0,   0,   0,   0,   0,   0,  13,   0,   0,   0,  14,   0,  15,  16,
    17,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    18,   0,   0,   0,   0,   0,   0,   0,  19,   0,   0,   0,  20,   0,  21,  22,
    23,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    24,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    25,   0,   0,   0,   0,   0,   0,   0,  26,   0,   0,   0,  27,   0,  28,  29,
    30,  31,   0,  32,   0,   0,   0,  33,   0,   0,   0,   0,   0,   0,   0,  34,
    0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  35,
    0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  36,
    37,  38,   0,  39,   0,   0,   0,  40,   0,   0,   0,   0,   0,   0,   0,  41,
    0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  42,
    43,  44,   0,  45,   0,   0,   0,  46,   0,   0,   0,   0,   0,   0,   0,  47,
    48,  49,   0,  50,   0,   0,   0,  51,  52,  53,   0,  54,  55,  56,  57,  58
};

class LBPModel
{
public:
    cv::Mat foreground;
    bool do_updates;

    LBPModel();
    void initialize( cv::Mat& frame );
    void update( cv::Mat& frame );

private:
    cv::Mat int_frame;
    cv::Mat gray_frame;
    cv::Mat last_lbp;
    std::list< std::vector<cv::Mat> > region_histogram_sets;
    std::list< std::vector<double> > region_histogram_weights;
    std::list< std::set<int> > region_histogram_weight_inds;

    void getGrayIntFrame( cv::Mat& mat_frame );
    void regionHistogram( cv::Mat &current_lbp );
    void simpleNeighborhood( cv::Mat &current_lbp );

    // This version allows you to use 5, 6, 7, or 8 points around the center pixel.
    // However it does not do interpolation or allow you to have a radius greater than 1.
    // I've only tested it with 6 and 8, which work fine.  Code for the bilinear interpolation
    // and other variations is available from the same place the original LPB code came from.
    void ComputeLBP(cv::Mat &input, cv::Mat &result, bool use59=false);
};

#endif
