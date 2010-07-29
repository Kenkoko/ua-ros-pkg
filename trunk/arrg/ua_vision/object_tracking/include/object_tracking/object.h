/*
    Copyright (c) <year>, <copyright holder>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY <copyright holder> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef OBJECT_H
#define OBJECT_H

#include <fstream>

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Object
{
public:
    int id;
    double area;
    double min;
    double max;

    int missed_frames;

    std::vector<cv::Point> tracks;
    std::vector<ros::Time> timestamps;

    cv::SparseMat histogram;
    cv::RotatedRect tight_bounding_box;

    //cv::Mat tr_img;
    //std::vector<cv::KeyPoint> keypoints;

    Object() : missed_frames(0)
    {
    }

    void dump_to_file()
    {
        std::string t = boost::lexical_cast<std::string>(ros::Time::now().sec) + "." + boost::lexical_cast<std::string>(ros::Time::now().nsec);
        std::string fname = "tracks_" + boost::lexical_cast<std::string>(id) + "_" + t + ".dat";
        std::ofstream f(fname.c_str());

        for (size_t i = 0; i < tracks.size(); ++i)
        {
            f << id << "," << timestamps[i] << "," << tracks[i].x << "," << tracks[i].y << std::endl;
        }
    }
};

#endif // OBJECT_H
