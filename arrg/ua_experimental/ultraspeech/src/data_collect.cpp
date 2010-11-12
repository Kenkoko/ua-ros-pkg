#include <opencv/cv.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/smart_ptr.hpp>

#include "threadpool/boost/threadpool.hpp"
#include <iostream>
#include "png.h"
#include <sys/types.h>
#include <string>

#include "ultraspeech/Control.h"
#include "ultraspeech/CurrentStim.h"
#include "ultraspeech/SaveFile.h"

#define ERROR 1
#ifndef png_jmpbuf
#  define png_jmpbuf(png_ptr) ((png_ptr)->jmpbuf)
#endif

class ImageWrite
{
private:
  image_transport::Subscriber sub_;
  
  ros::Subscriber control_sub_;
  ros::Publisher savefile_pub_;
  
  sensor_msgs::ImageConstPtr last_msg_;
  sensor_msgs::CvBridge img_bridge_;
  boost::mutex image_mutex_;
  boost::threadpool::pool png_threadpool;

  std::string window_name_;
  boost::format filename_format_;
  int count_;
  std::string filename_chunk_;
  std::string subdir_;
  std::string image_name_;
  int run_status_;
  
public:
  ImageWrite(const ros::NodeHandle& nh, 
	     const std::string& fname_chunk, 
	     const std::string& transport)
    : filename_format_(""), 
      count_(0), 
      png_threadpool(15),
      filename_chunk_(fname_chunk),
      run_status_(0)
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
    
    run_status_ = msg->run;
    subdir_ = (std::string)(msg->directory + "/" + filename_chunk_ + "/");
    std::cout << "run_status changed: " << run_status_ << std::endl;
  }
  
  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    using std::cout;
    using std::endl;
    using boost::lexical_cast;
    char num[10];
    ultraspeech::SaveFile sv_msg;

    boost::lock_guard<boost::mutex> guard(image_mutex_);

    // Hang on to message pointer for sake of mouse_cb
    last_msg_ = msg;

    // May want to view raw bayer data
    // NB: This is hacky, but should be OK since we have only one image CB.
    if (msg->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";
    cv::Mat img = img_bridge_.imgMsgToCv(msg, "rgb8");
    std::sprintf(num, "%.06d", msg->header.seq);
    std::string filename(subdir_ + filename_chunk_ + "_" + num +".png");
    image_name_ = filename;
    if (run_status_ == 1)
    {
      //sv_msg.header.stamp = ros::Time::now();
      sv_msg.header.stamp = msg->header.stamp;
      sv_msg.filepath = filename;
      savefile_pub_.publish(sv_msg);
    } 
    std::cout << filename << std::endl;
    std::cout << "run_status: " << run_status_ << std::endl;

    if (run_status_ == 1)
      png_thread(img, filename);

    count_++;
  }

  void png_thread(cv::Mat &img, std::string &filename){
    using boost::shared_ptr;
    using std::cout;
    using std::endl;
    using boost::lexical_cast;

    shared_ptr<cv::Mat> shared_img(new cv::Mat);
    *shared_img = img.clone();
    //cout << "shared_img->rows: " << shared_img->rows << endl;
	
    png_threadpool.schedule( boost::bind(&ImageWrite::save_to_png, this, filename, shared_img) );
    //std::cout << count_ << std::endl;

  }

  void save_to_png(std::string filename, boost::shared_ptr<cv::Mat> img){

    png_bytep pix = img->data;
    png_uint_32 width = img->cols;
    png_uint_32 height = img->rows;
    size_t bit_depth = 8;//img->depth();
    size_t bytes_per_pixel = 3;//bit_depth*3;
    
    //img.data, img.cols, img.rows,8, 3
    /* Open the file */
       FILE *fp;
       png_structp png_ptr;
       png_infop info_ptr;
       fp = fopen(filename.c_str(), "wb");
       if (fp == NULL)
               return ;//(ERROR);

       /* Create and initialize the png_struct with the desired error handler
        * functions.  If you want to use the default stderr and longjump method,
        * you can supply NULL for the last three parameters.  We also check that
        * the library version is compatible with the one used at compile time,
        * in case we are using dynamically linked libraries.  REQUIRED.
        */
       png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);

       if (png_ptr == NULL)
       {
               fclose(fp);
               return ;//(ERROR);
       }

       /* Allocate/initialize the image information data.  REQUIRED */
       info_ptr = png_create_info_struct(png_ptr);
       if (info_ptr == NULL)
       {
               fclose(fp);
               png_destroy_write_struct(&png_ptr,  NULL);
               return ;//(ERROR);
       }

       /* Set error handling.  REQUIRED if you aren't supplying your own
        * error handling functions in the png_create_write_struct() call.
        */
       if (setjmp(png_jmpbuf(png_ptr)))
       {
               /* If we get here, we had a problem writing the file */
               fclose(fp);
               png_destroy_write_struct(&png_ptr, &info_ptr);
               return ;//(ERROR);
       }

       /* One of the following I/O initialization functions is REQUIRED */

       /* Set up the output control if you are using standard C streams */
       png_init_io(png_ptr, fp);

       /* Set the image information here.  Width and height are up to 2^31,
        * bit_depth is one of 1, 2, 4, 8, or 16, but valid values also depend on
        * the color_type selected. color_type is one of PNG_COLOR_TYPE_GRAY,
        * PNG_COLOR_TYPE_GRAY_ALPHA, PNG_COLOR_TYPE_PALETTE, PNG_COLOR_TYPE_RGB,
        * or PNG_COLOR_TYPE_RGB_ALPHA.  interlace is either PNG_INTERLACE_NONE or
        * PNG_INTERLACE_ADAM7, and the compression_type and filter_type MUST
        * currently be PNG_COMPRESSION_TYPE_BASE and PNG_FILTER_TYPE_BASE. REQUIRED
        */
       png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, PNG_COLOR_TYPE_RGB,
                                PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);


       /* Write the file header information.  REQUIRED */
       png_write_info(png_ptr, info_ptr);

       /* The easiest way to write the image (you may have a different memory
        * layout, however, so choose what fits your needs best).  You need to
        * use the first method if you aren't handling interlacing yourself.
        */
       png_bytep row_pointers[height];

       if (height > PNG_UINT_32_MAX/png_sizeof(png_bytep))
               png_error (png_ptr, "Image is too tall to process in memory");

       for (png_uint_32 k = 0; k < height; k++)
               row_pointers[k] = pix + k*width*bytes_per_pixel;

       /* One of the following output methods is REQUIRED */

       png_write_image(png_ptr, row_pointers);


       /* It is REQUIRED to call this to finish writing the rest of the file */
       png_write_end(png_ptr, info_ptr);

       /* Clean up after the write, and free any memory allocated */
       png_destroy_write_struct(&png_ptr, &info_ptr);

       /* Close the file */
       fclose(fp);

       //return(0);
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
