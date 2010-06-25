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

#include <background_filters/common.h>

using namespace std;

class Object
{
public:
    int id;
    double area;
    double alpha;
    double beta;
    vector<cv::Point> tracks;
    cv::SparseMat histogram;
    ros::Time timestamp;
    bool wasFound;

// private:
//     int h_bins = 30;
//     int s_bins = 32;
//     int hist_size[] = {h_bins, s_bins};
//     float hranges[] = {0, 180};
//     float sranges[] = {0, 256};
//     const float* ranges[] = {hranges, sranges};
//     int channels[] = {0, 1};

public:
    void subtract_self(const cv::Mat& fg_prob_img, const cv::Mat& orig_img, const cv::Mat& bin_img,
                       const cv::Mat& hsv_img, vector<vector<cv::Point> > contours, cv::Mat& fg_loglike_img)
    {
      int h_bins = 30;
      int s_bins = 32;
//      int hist_size[] = {h_bins, s_bins};
      float hranges[] = {0, 180};
      float sranges[] = {0, 256};
      const float* ranges[] = {hranges, sranges};
      int channels[] = {0, 1};


      // Back project the histogram
//      cv::Mat back_project;
//      cv::calcBackProject(&hsv_img, 1, channels, histogram, back_project, ranges);

      // Convert the 0-255 "probabilities" to float probs (might be able to hack this away later)
//      cv::Mat bp_prob;
//      back_project.convertTo(bp_prob, CV_32FC1, 1.0/255.0);

//      cv::namedWindow("TEST");
//      cv::imshow("TEST", bp_prob);

      // Multiply the back-projected probabilities with the foreground probabilities
//      cv::Mat fg_combined_prob;
//      cv::multiply(fg_prob_img, bp_prob, fg_combined_prob);

        wasFound = false;

      BOOST_FOREACH(vector<cv::Point> contour, contours)
      {
        cv::Rect bounder = cv::boundingRect(cv::Mat(contour));

        cv::Point new_center(0, 0);
        cv::Point last = tracks.back();

        if (tracks.size() > 1)
        {
            cv::Point last2 = tracks[tracks.size() - 2];
            new_center.x = last.x - last2.x;
            new_center.y = last.y - last2.y;
        }

        new_center += last;
        cv::Rect new_bounder = bounder + cv::Size(bounder.width / 2, bounder.height / 2);

        if (new_center.inside(new_bounder))
        {
          cv::Mat roi = hsv_img(bounder);

          // Back project the histogram
          cv::Mat back_project;
          cv::calcBackProject(&roi, 1, channels, histogram, back_project, ranges);

          // Convert the 0-255 "probabilities" to float probs (might be able to hack this away later)
          cv::Mat bp_prob;
          back_project.convertTo(bp_prob, CV_32FC1, 1.0/255.0, 1.0/255.0);

          cv::Mat real_roi = fg_loglike_img(bounder);

          cv::log(bp_prob, bp_prob);
          cv::add(bp_prob, real_roi, bp_prob);

          double min, max;
          cv::minMaxLoc(bp_prob, &min, &max);
          ROS_INFO("min = %f, max = %f", min, max);

          bp_prob.convertTo(bp_prob, bp_prob.type(), -beta, -alpha);
          cv::exp(bp_prob, bp_prob);
          bp_prob.convertTo(bp_prob, bp_prob.type(), 1, 1);
          cv::divide(1.0, bp_prob, bp_prob);

          cv::Mat bin_image;
          cv::threshold(bp_prob, bin_image, 0.6, 255, cv::THRESH_BINARY);
          bin_image.convertTo(bin_image, CV_8UC1);

          std::vector<std::vector<cv::Point> > contours;
          cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
          ROS_INFO("Found %Zu contours", contours.size());

          for (uint i = 0; i < contours.size(); ++i)
          {
              std::vector<cv::Point> con = contours[i];

              // If we have a reasonably large contour, we need to inform the tracker that it
              // is missing an object
              double area = cv::contourArea(cv::Mat(con));
              ROS_INFO("Contour %u has area %f", i, area);
              if (area > 20)
              {
                bin_image.convertTo(bin_image, CV_32FC1);
                bin_image.convertTo(bin_image, CV_32FC1, -1, 1);
                cv::blur(bin_image, bin_image, cv::Size(3, 3));
                cv::multiply(bin_image, real_roi, real_roi);

                cv::namedWindow("obj_" + boost::lexical_cast<string>(id));
                cv::imshow("obj_" + boost::lexical_cast<string>(id), bp_prob);

                ROS_INFO("Object %d found itself at location (%d, %d)", id, tracks.back().x, tracks.back().y);
                wasFound = true;

                cv::Rect b = cv::boundingRect(cv::Mat(con));
                int cx = b.x + b.width / 2 + bounder.x;
                int cy = b.y + b.height / 2 + bounder.y;
                cv::Point center(cx, cy);

                tracks.push_back(center);
              }
          }

          if (contours.size() > 0) break;
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
    CvFont font;

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

        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .33, .33);

        cv::startWindowThread();
    }

    ~ContourObjectFinder()
    {
        delete sync;
    }

    bool add_object(object_tracking::AddObject::Request& request, object_tracking::AddObject::Response& response)
    {
        boost::mutex::scoped_lock lock(obj_mutex);
//        new_obj_bridge.fromImage(request.histogram);
        sensor_msgs::CvBridge new_bridge;
        new_bridge.fromImage(request.histogram);

        Object obj;
        obj.id = objects.size();
        obj.histogram = cv::Mat(new_bridge.toIpl()).clone();
//        displayHist(obj.histogram, "PLEASE");
        obj.timestamp = request.histogram.header.stamp;
        obj.area = request.area;
        cv::Point center;
        center.x = request.center.x;
        center.y = request.center.y;
        obj.alpha = request.alpha;
        obj.beta = request.beta;
        obj.tracks.push_back(center);

        objects.push_back(obj);
        return true;
    }

    void process_images(const sensor_msgs::ImageConstPtr& fg_objects_msg, const sensor_msgs::ImageConstPtr& image_msg)
    {
        //ROS_INFO_STREAM("Foreground objects time: " << fg_objects_msg->header.stamp);
        //ROS_INFO_STREAM("Original time:           " << saliency_msg->header.stamp);

        if (objects.empty()) { // If no objects, just pass along the image unchanged - otherwise the other node has no input
          ROS_INFO_STREAM("No objects, doing nothing.");
          occluded_fg_objects_pub.publish(fg_objects_msg);


        } else {
          ROS_INFO_STREAM("BEGIN");

            cv::Mat fg_loglike_img(fg_objects_bridge.imgMsgToCv(fg_objects_msg));
            cv::Mat original_big(image_bridge.imgMsgToCv(image_msg));
            cv::Mat original;
            cv::resize(original_big, original, fg_loglike_img.size());

            // Compute the (foreground) probability image under the logistic model
            double w = -1/5.0, b = 4.0;
            cv::Mat fg_prob_img = fg_loglike_img.clone();
            fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), w, b);
            cv::exp(fg_prob_img, fg_prob_img);
            fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), 1, 1);
            cv::divide(1.0, fg_prob_img, fg_prob_img);

            cv::namedWindow("fg_prob_img");
            cv::imshow("fg_prob_img", fg_prob_img);

            // Find contours for all the blobs found by background subtraction
            std::vector<std::vector<cv::Point> > contours;
            cv::Mat bin_image = (fg_prob_img > fg_prob_threshold);
//            cv::imshow("binary", bin_image);
            cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

            // Make an HSV image
            cv::Mat hsv_img = original.clone();
            cv::cvtColor(original, hsv_img, CV_BGR2HSV);

//            cv::namedWindow("hsv_img");
//            cv::imshow("hsv_img", hsv_img);

            IplImage orig_ipl = original;

            BOOST_FOREACH(Object obj, objects)
            {
                obj.subtract_self(fg_prob_img, original, bin_image, hsv_img, contours, fg_loglike_img);

                if (obj.wasFound)
                {
                    cvPutText(&orig_ipl, boost::lexical_cast<std::string>(obj.id).c_str(), cvPoint(obj.tracks.back().x, obj.tracks.back().y), &font, CV_RGB(255, 0, 0));

                    if (obj.tracks.size() > 1)
                    {
                        for (int i = 1; i < obj.tracks.size(); ++i)
                        {
                            cv::line(original, obj.tracks[i-1], obj.tracks[i], CV_RGB(255, 0, 0));
                        }
                    }
                }
            }

            IplImage img_out = fg_loglike_img;
            sensor_msgs::Image::Ptr img_msg_out = sensor_msgs::CvBridge::cvToImgMsg(&img_out);
            img_msg_out->header.stamp = fg_objects_msg->header.stamp;
            occluded_fg_objects_pub.publish(img_msg_out);

            cv::namedWindow("objects");
            cv::imshow("objects", original);

            ROS_INFO_STREAM("END");
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
