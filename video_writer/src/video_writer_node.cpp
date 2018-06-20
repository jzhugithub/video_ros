#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>  


using namespace cv;
using namespace std;

class MyVideoWriter
{
public:
  //node
  ros::NodeHandle nh_;
  ros::NodeHandle nh_image_param;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  string subscribed_topic;
  //video
  string VIDEO_WINDOW_NAME;
  bool show_video_flag;
  bool save_video_flag;
  double video_rate;
  double image_hight;
  double image_width;
  double video_delay;
  VideoWriter video;
  string video_file_name;
  //frame
  int frame_num;
  Mat src_3;
  
  MyVideoWriter():
  it_(nh_),//intial it_
  nh_image_param("~")
  {
    //node
    if(!nh_image_param.getParam("subscribed_topic", subscribed_topic))subscribed_topic = "/dji_sdk/image_raw";
    // Subscrive to input video feed from "/dji_sdk/image_raw" topic, imageCb is the callback function
    image_sub_ = it_.subscribe(subscribed_topic, 1, &MyVideoWriter::imageCb, this);
    //video
    if(!nh_image_param.getParam("show_video_flag", show_video_flag))show_video_flag = true;
    if(show_video_flag)
    {
    VIDEO_WINDOW_NAME="video";
    namedWindow(VIDEO_WINDOW_NAME);
    }
    if(!nh_image_param.getParam("save_video_flag", save_video_flag))save_video_flag = false;
    if(!nh_image_param.getParam("rate", video_rate))video_rate = 30.0;
    video_delay = 1000/video_rate;
    if(!nh_image_param.getParam("video_file_name", video_file_name))video_file_name = "/home/ubuntu/ros_my_workspace/src/video_writer/video/a544.avi";
    //frame
    frame_num = 1;
  }
  
  ~MyVideoWriter()
  {
    destroyAllWindows();
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr->image.copyTo(src_3);
    //cout<<"rows"<<src_3.rows<<endl;
    //cout<<"cols"<<src_3.cols<<endl;
    
    if(frame_num == 1)
    {
      image_hight = src_3.rows;
      image_width = src_3.cols;
      video = VideoWriter(video_file_name, CV_FOURCC('M', 'J', 'P', 'G'), video_rate, Size(image_width, image_hight));
    }
    frame_num++;
    
    //save and show video
    if(save_video_flag)
    {
      video<<src_3;
    }
    if(show_video_flag)
    {
      imshow(VIDEO_WINDOW_NAME, src_3);
      waitKey(1);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_writer_node");//node name
  double loop_rate;
  MyVideoWriter mrd;//class initializing
  ros::NodeHandle nh_loop_param("~");
  if(!nh_loop_param.getParam("rate", loop_rate))loop_rate = 30.0;//video
  ros::Rate loop_rate_class(loop_rate);//frequency: n Hz
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate_class.sleep();
  }
  ros::spin();
  return 0;
}

