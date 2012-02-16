#include <opencv/cv.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <cxcore.h>
#include <highgui.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/smart_ptr.hpp>

#include <iostream>
#include <sys/types.h>
#include <string>

#include "ultraspeech/Control.h"
#include "ultraspeech/CurrentStim.h"
#include "ultraspeech/SaveFile.h"


char *convertStringToChar(const std::string &str)
{
  char *retPtr = new char[str.length() + 1];
  std::strcpy(retPtr, str.c_str());
  return retPtr;
}

class ImageWrite
{
private:
  image_transport::Subscriber sub_;
  
  ros::Subscriber control_sub_;
  ros::Publisher savefile_pub_;
  
  sensor_msgs::ImageConstPtr last_msg_;
  sensor_msgs::CvBridge img_bridge_;
  boost::mutex image_mutex_;

  std::string window_name_;
  boost::format filename_format_;
  int count_;
  std::string filename_chunk_;
  std::string subdir_;
  std::string image_name_;
  int run_status_;
  
  CvVideoWriter *writer_;
  double fps_;
  CvSize imgSize_;
  
public:
  ImageWrite(const ros::NodeHandle& nh, 
	     const std::string& fname_chunk, 
	     const std::string& transport)
    : filename_format_(""), 
      count_(0), 
      filename_chunk_(fname_chunk),
      run_status_(0),
      fps_(30)
  {
    std::string topic = nh.resolveName("image");
    ros::NodeHandle local_nh("~");

    image_transport::ImageTransport it(nh);
    sub_ = it.subscribe(topic, 1, &ImageWrite::image_cb, this, transport);

    control_sub_ = local_nh.subscribe("/control", 1, &ImageWrite::control_cb, this);
    savefile_pub_ = local_nh.advertise<ultraspeech::SaveFile>("save_name", 1);
    

  }
  
  void control_cb(const ultraspeech::ControlConstPtr& msg)
  {
    using std::cout;
    using std::endl;
    //char num[10];
    
    run_status_ = msg->run;
    subdir_ = (std::string)(msg->directory + "/" + filename_chunk_ + "/");
    std::cout << "run_status changed: " << run_status_ << std::endl;

    // Commenting out code for saving data to compressed video for now -IF
/*    if (run_status_ == 1) {
      std::sprintf(num, "%.06d", msg->header.seq);
      std::string filename(subdir_ + filename_chunk_ + "_" + num +".avi");
      char *fnamebuf(convertStringToChar(filename));
      writer_ = cvCreateVideoWriter(fnamebuf,
                                    //CV_FOURCC('H', 'F', 'Y', 'U'),  // works on ubuntu hd (ffenc_huffyuv) (lossless)
                                    //CV_FOURCC('M', 'P', '1', 'V'), // (mpeg2enc)
                                    CV_FOURCC('X', 'V', 'I', 'D'),  // works on N's linux and ubuntu hd
                                    //CV_FOURCC('M', 'J', 'P', 'G'),  // works on a mac
                                    fps_,imgSize_);
      if( !writer_ ){
        std::cout << "--- Could not start video writing, saving to images instead" << std::endl;
      }
      else 
	std::cout << "created video writer " << fnamebuf << std::endl;

      
    }
    else {
      cvReleaseVideoWriter(&writer_);
      std::cout << "released video writer" << std::endl;
    }
*/  }
  
  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    using std::cout;
    using std::endl;
    using boost::lexical_cast;
    char num[10];
    //int frame;
    ultraspeech::SaveFile sv_msg;

    boost::lock_guard<boost::mutex> guard(image_mutex_);

    // Hang on to message pointer for sake of mouse_cb
    last_msg_ = msg;

    // May want to view raw bayer data
    // NB: This is hacky, but should be OK since we have only one image CB.
    if (msg->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";
    IplImage* img = img_bridge_.imgMsgToCv(msg, "bgr8");
    imgSize_.height = img->height;
    imgSize_.width = img->width;
    //std::sprintf(num, "%.06d", msg->header.seq);
    //std::string filename(subdir_ + filename_chunk_ + "_" + num +".png");
    //image_name_ = filename;
    if (run_status_ == 1)
    {
      // Not saving to video but to png
 /*     if (writer_) {
	//sv_msg.header.stamp = ros::Time::now();
	sv_msg.header.stamp = msg->header.stamp;
	sv_msg.filepath = "video_writing_started";
	savefile_pub_.publish(sv_msg);
	cvWriteFrame(writer_, img);
	//std::cout << "wrote frame" << std::endl;
      }
      else {
*/	std::sprintf(num, "%.06d", msg->header.seq);
	std::string filename(subdir_ + filename_chunk_ + "_" + num +".png");
	char *fnamebuf(convertStringToChar(filename));
	sv_msg.header.stamp = msg->header.stamp;
	sv_msg.filepath = filename;
	savefile_pub_.publish(sv_msg);
	//std::cout << "wrote file" << std::endl;
	if(!cvSaveImage(fnamebuf,img)) 
	  printf("Could not save: %s\n",fnamebuf);
  //    }
    } 
    //std::cout << filename << std::endl;
    //std::cout << "run_status: " << run_status_ << std::endl;
    
    count_++;
  }

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_collect", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  if (n.resolveName("image") == "/image") {
    ROS_WARN("image_write: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./image_write image:=<image topic> [transport]");
  }

  ImageWrite view(n, argv[1], (argc > 2) ? argv[2] : "raw");

  ros::spin();

  return 0;
}
