#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

using namespace std;

// class ContourObjectFinder
// {
// private:
//     image_transport::Subscriber* image_sub_;
//     image_transport::Subscriber* prob_image_sub_;
//
//
// public:
//     ContourObjectFinder(ros::NodeHandle& nh, const std::string& transport)
//     {
//         image_sub_ = new image_transport::Subscriber(nh, )
//     }
// };


class ContourObjectFinder
{
private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Image> fg_objects_sub;
    message_filters::Synchronizer<SyncPolicy>* sync;

    sensor_msgs::CvBridge image_bridge;
    sensor_msgs::CvBridge fg_objects_bridge;

public:
    ContourObjectFinder(ros::NodeHandle& nh)
    {
        image_sub.subscribe(nh, "camera/image_color", 1);
        fg_objects_sub.subscribe(nh, "probability_image", 1);

        // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), fg_objects_sub, image_sub);
        sync->registerCallback(boost::bind(&ContourObjectFinder::process_images, this, _1, _2));
    }

    ~ContourObjectFinder()
    {
        delete sync;
    }

    void process_images(const sensor_msgs::ImageConstPtr& fg_objects_msg, const sensor_msgs::ImageConstPtr& saliency_msg)
    {
        //ROS_INFO_STREAM("Foreground objects time: " << fg_objects_msg->header.stamp);
        //ROS_INFO_STREAM("Original time:           " << saliency_msg->header.stamp);

        cv::Mat fg_objects_img(fg_objects_bridge.imgMsgToCv(fg_objects_msg));
        cv::Mat original(image_bridge.imgMsgToCv(saliency_msg));

        // Compute the (foreground) probability image under the logistic model
        double w = -1/5.0, b = 4.0;
        cv::Mat fg_prob_img = fg_objects_img;
        fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), w, b);
        cv::exp(fg_prob_img, fg_prob_img);
        fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), 1, 1);
        cv::divide(1.0, fg_prob_img, fg_prob_img);

        cv::namedWindow("fg_prob_img");
        cv::imshow("fg_prob_img", fg_prob_img);

        double sum = cv::sum(fg_prob_img)[0];

        std::vector<std::vector<cv::Point> > contours;
        cv::Mat bin_image = (fg_prob_img > 0.6);
        cv::imshow("binary", bin_image);
        cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // Make an HSV image
        cv::Mat hsv_img = original.clone();
        cv::cvtColor(original, hsv_img, CV_BGR2HSV);

        cv::namedWindow("hsv_img");
        cv::imshow("hsv_img", hsv_img);

        //             cv::drawContours(new_img, contours, -1, cv::Scalar(0, 255, 0));

        srand ( time(NULL) );

        std::vector<cv::Rect> fg_rects;
        std::vector<cv::MatND> histograms;

        int h_bins = 30, s_bins = 32;

        for (int i = 0; i < contours.size(); ++i)
        {
            cout << "INSIDE" << endl;
            std::vector<cv::Point> con = contours[i];

            int r, g, b;
            r = rand() % 255;
            g = rand() % 255;
            b = rand() % 255;

            std::vector<std::vector<cv::Point> > one_contour;
            one_contour.push_back(con);

            cv::drawContours(original, one_contour, -1, cv::Scalar(r, g, b));
            cv::Rect bounder = cv::boundingRect(cv::Mat(con));
            cv::rectangle(original, bounder, cv::Scalar(r, g, b));
//             if (cv::contourArea(cv::Mat(con)) > 10)
//             {
//                 fg_rects.push_back(bounder);
//                 cv::Mat mask = bin_image(bounder);
//                 cv::Mat hsv_roi = hsv_img(bounder);
//
//                 cv::MatND hist;
//
//                 int hist_size[] = {h_bins, s_bins};
//                 // hue varies from 0 to 179, see cvtColor
//                 float hranges[] = { 0, 180 };
//                 // saturation varies from 0 (black-gray-white) to
//                 // 255 (pure spectrum color)
//                 float sranges[] = { 0, 256 };
//                 const float* ranges[] = { hranges, sranges };
//                 //                     int hist_size[] = {num_bins, num_bins};
//                 int channels[] = {0, 1};
//                 //                     float range[] = {0, 256};
//                 //                     const float* ranges[] = {range, range};
//                 cv::calcHist(&hsv_roi, 1, channels, mask, hist, 2, hist_size, ranges);
//                 histograms.push_back(hist);
//
//                 cv::Mat back_project;
//                 cv::calcBackProject(&hsv_img, 1, channels, hist, back_project, ranges);
//
//                 std::string window_name = boost::lexical_cast<string>(i) + "back_project";
//                 cv::namedWindow( window_name, 1 );
//                 //                     cv::imshow( window_name, back_project );
//                 cv::Mat bp_prob;
//                 back_project.convertTo(bp_prob, CV_32FC1, 1.0/255.0);
//                 cv::imshow( window_name, bp_prob );
//
//                 double max;
//                 cv::minMaxLoc(bp_prob, 0, &max);
//                 std::cout << window_name << " " << max << std::endl;
//
//                 cv::Mat fg_combined_prob;
//                 cv::multiply(fg_prob_img, bp_prob, fg_combined_prob);
//                 cv::normalize(fg_combined_prob, fg_combined_prob, 0, 1, cv::NORM_MINMAX);
//                 window_name = boost::lexical_cast<string>(i) + "fg_combined_prob";
//                 cv::namedWindow(window_name);
//                 cv::imshow( window_name, fg_combined_prob );
//             }
        }

        cout << "HERE" << endl;
        cv::namedWindow("contours");
        cv::imshow("contours", original);
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "contour_object_finder", ros::init_options::AnonymousName);
    ros::NodeHandle n;

//     if (n.resolveName("image") == "/image")
//     {
//         ROS_WARN("background_subtractor: image has not been remapped! Typical command-line usage:\n"
//         "\t$ ./background_subtractor image:=<image topic> [transport]");
//     }

    cout << "WE'RE IN\n";

    ContourObjectFinder finder(n);
    ros::spin();

    return 0;
}