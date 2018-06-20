#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>  

using namespace std;
using namespace cv;

class VideoPublisher
{
public:
  //node
  ros::NodeHandle nh_;
  ros::NodeHandle nh_param;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  //video
  string WINDOW_NAME;
  string published_video_file_name;
  bool show_video_flag;
  VideoCapture my_video;
  bool video_open_flag;
  double video_rate;
  double video_hight;
  double video_width;
  double video_delay;
  //trans
  ros::Time time;
  cv_bridge::CvImage cvi;
  sensor_msgs::Image im;
  Mat src;
  
  VideoPublisher():
  it_(nh_),
  nh_param("~")
  {
    //node
    image_pub_ = it_.advertise("/my_video", 1);
    if(!nh_param.getParam("published_video_file_name", published_video_file_name))published_video_file_name = "/home/ubuntu/ros_my_workspace/src/video_publisher/video/a544.avi";
    //video
    if(!nh_param.getParam("show_video_flag", show_video_flag))show_video_flag = true;
    if(show_video_flag)
    {
      WINDOW_NAME = "Image window";
      namedWindow(WINDOW_NAME);
    }
    my_video = VideoCapture(published_video_file_name);
    if(!my_video.isOpened())
    {
      video_open_flag = false;
      cout<<"video open error"<<endl;
    }
    else
    {
      video_open_flag = true;
      cout<<"video is open"<<endl;
      video_rate = my_video.get(CV_CAP_PROP_FPS);
      if(isnan(video_rate))
      {
	if(!nh_param.getParam("given_video_rate", video_rate))video_rate = 30.0;
	cout<<"video_rate is nan, using given_video_rate: "<<video_rate<<endl;
      }
      else
      {
	cout<<"video_rate: "<<video_rate<<endl;
      }
      video_width = my_video.get(CV_CAP_PROP_FRAME_WIDTH);
      cout<<"video_width: "<<video_width<<endl;
      video_hight = my_video.get(CV_CAP_PROP_FRAME_HEIGHT);
      cout<<"video_hight: "<<video_hight<<endl;
      video_delay = 1000/video_rate;
    }
  }

  ~VideoPublisher()
  {
    destroyAllWindows();
  }

  void runPublish()
  {
    ros::Rate rate(video_rate);
    while(ros::ok())
    {
      //trans
      if (!my_video.read(src)){cout<<"video end"<<endl; break;}
      time=ros::Time::now();
      cvi.header.stamp = time;
      cvi.header.frame_id = "my video";
      cvi.encoding = "bgr8";
      cvi.image = src;
      cvi.toImageMsg(im);
      if(show_video_flag)
      {
	imshow(WINDOW_NAME, src);
	waitKey(1);
      }
      image_pub_.publish(im);
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_publisher_node");
  VideoPublisher vp;
  if(vp.video_open_flag)
    vp.runPublish();
  ros::spin();
  return 0;
}