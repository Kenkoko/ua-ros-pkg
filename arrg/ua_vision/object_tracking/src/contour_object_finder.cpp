#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <object_tracking/AddObject.h>

using namespace std;

class Object
{
public:
    double area;
    vector<cv::Point> tracks;
    cv::SparseMat histogram;
    ros::Time timestamp;

// private:
//     int h_bins = 30;
//     int s_bins = 32;
//     int hist_size[] = {h_bins, s_bins};
//     float hranges[] = {0, 180};
//     float sranges[] = {0, 256};
//     const float* ranges[] = {hranges, sranges};
//     int channels[] = {0, 1};

public:
    void subtract_self(const cv::Mat& fg_prob_img, const cv::Mat& orig_img, const cv::Mat& bin_img, const cv::Mat& hsv_img, vector<vector<cv::Point> > contours)
    {
        BOOST_FOREACH(vector<cv::Point> contour, contours)
        {
            cv::Rect bounder = cv::boundingRect(cv::Mat(contour));
            int cx = (bounder.x + bounder.width) / 2;
            int cy = (bounder.y + bounder.height) / 2;
            cv::Point center(cx, cy);

            if (tracks.back().inside(bounder))
            {
//                 cv::Mat mask = bin_img(bounder);
//                 cv::Mat hsv_roi = hsv_img(bounder);
//                 cv::MatND hist;
//
//                 cv::calcHist(&hsv_roi, 1, channels, mask, hist, 2, hist_size, ranges);
//
//                 cv::Mat back_project;
//                 cv::calcBackProject(&hsv_img, 1, channels, hist, back_project, ranges);
//
//                 cv::Mat bp_prob;
//                 back_project.convertTo(bp_prob, CV_32FC1, 1.0/255.0);
//
//                 double max;
//                 cv::minMaxLoc(bp_prob, 0, &max);
//
//                 cv::Mat fg_combined_prob;
//                 cv::multiply(fg_prob_img, bp_prob, fg_combined_prob);
//                 cv::normalize(fg_combined_prob, fg_combined_prob, 0, 1, cv::NORM_MINMAX);
            }
        }
    }
};

class ContourObjectFinder
{
private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Image> fg_objects_sub;
    message_filters::Synchronizer<SyncPolicy>* sync;

    sensor_msgs::CvBridge image_bridge;
    sensor_msgs::CvBridge fg_objects_bridge;
    sensor_msgs::CvBridge new_obj_bridge;

    ros::ServiceServer add_object_service;
    ros::Publisher occluded_fg_objects_pub;

    boost::mutex obj_mutex;
    vector<Object> objects;

    double fg_prob_threshold;

public:
    ContourObjectFinder(ros::NodeHandle& nh)
    {
        fg_prob_threshold = 0.6;

        image_sub.subscribe(nh, "camera/image_color", 1);
        fg_objects_sub.subscribe(nh, "probability_image", 1);

        // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), fg_objects_sub, image_sub);
        sync->registerCallback(boost::bind(&ContourObjectFinder::process_images, this, _1, _2));

        add_object_service = nh.advertiseService("add_object", &ContourObjectFinder::add_object, this);
        occluded_fg_objects_pub = nh.advertise<sensor_msgs::Image>("occluded_fg_objects", 1);
    }

    ~ContourObjectFinder()
    {
        delete sync;
    }

    bool add_object(object_tracking::AddObject::Request& request, object_tracking::AddObject::Response& response)
    {
        boost::mutex::scoped_lock lock(obj_mutex);
        new_obj_bridge.fromImage(request.histogram);

        Object obj;
        obj.histogram = cv::Mat(new_obj_bridge.toIpl()).clone();
        obj.timestamp = request.histogram.header.stamp;
        obj.area = request.area;
        cv::Point center;
        center.x = request.center.x;
        center.y = request.center.y;
        obj.tracks.push_back(center);

        objects.push_back(obj);
        return true;
    }

    void process_images(const sensor_msgs::ImageConstPtr& fg_objects_msg, const sensor_msgs::ImageConstPtr& image_msg)
    {
        //ROS_INFO_STREAM("Foreground objects time: " << fg_objects_msg->header.stamp);
        //ROS_INFO_STREAM("Original time:           " << saliency_msg->header.stamp);

        if (!objects.empty())
        {
            cv::Mat fg_objects_img(fg_objects_bridge.imgMsgToCv(fg_objects_msg));
            cv::Mat original(image_bridge.imgMsgToCv(image_msg));

            // Compute the (foreground) probability image under the logistic model
            double w = -1/5.0, b = 4.0;
            cv::Mat fg_prob_img = fg_objects_img;
            fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), w, b);
            cv::exp(fg_prob_img, fg_prob_img);
            fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), 1, 1);
            cv::divide(1.0, fg_prob_img, fg_prob_img);

            cv::namedWindow("fg_prob_img");
            cv::imshow("fg_prob_img", fg_prob_img);

            // Find contours for all the blobs found by background subtraction
            std::vector<std::vector<cv::Point> > contours;
            cv::Mat bin_image = (fg_prob_img > fg_prob_threshold);
            cv::imshow("binary", bin_image);
            cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

            // Make an HSV image
            cv::Mat hsv_img = original.clone();
            cv::cvtColor(original, hsv_img, CV_BGR2HSV);

            cv::namedWindow("hsv_img");
            cv::imshow("hsv_img", hsv_img);

            BOOST_FOREACH(Object obj, objects)
            {
                obj.subtract_self(fg_prob_img, original, bin_image, hsv_img, contours);
            }

            IplImage img_out = fg_prob_img;
            occluded_fg_objects_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(&img_out));
        }
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "contour_object_finder", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    ContourObjectFinder finder(n);
    ros::spin();

    return 0;
}