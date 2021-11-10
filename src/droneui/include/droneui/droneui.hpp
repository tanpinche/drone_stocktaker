#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include<opencv4/opencv2/objdetect.hpp>
#include<opencv4/opencv2/imgcodecs.hpp>
#include<opencv4/opencv2/imgproc/imgproc.hpp>
#include<opencv4/opencv2/highgui/highgui.hpp>

#include <std_msgs/Duration.h>
#include <cstdlib>
 
#include "espdrone_msgs/GoTo.h"
#include "espdrone_msgs/Takeoff.h"
#include "espdrone_msgs/Hover.h"
#include "espdrone_msgs/Stop.h"
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>


#include "implot/implot.h"
#include <string.h>
#include <thread>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot/implot.h"

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <atomic>
#include <map>
#include <chrono>


/*using namespace rdu;
struct ImDraw : public ImCanvas {
  void Draw() override {
    // do nothing
  }
};*/


class DroneGui {
 public:
  DroneGui(uint32_t width = 640, uint32_t height = 480,
         std::string title = "DroneGui");

  ~DroneGui();

  // virtual functions
  virtual double CalculateLinearVeloctiy();
  virtual double CalculateAngularVeloctiy();
  virtual void AdditionalFunctions();

  void Draw();
  bool send_inputs = false;

  

  

 private:
  // variables for window size settings
  bool initialized_ = false;
  uint32_t width_ = 500, height_ = 700;
  GLFWwindow *window_;
  ImVec4 background_color_;
  bool resize_after_connect_ = false;

  // variables for widget use
  int window_width_, window_height_;
  int current_height_ = 100, current_width_ = 100;
  ImVec2 cursor_pos_;

  //variables for imgui images
  GLuint texture;

 //control variables
 std::thread hover_thread_, waypoint_thread_;
 std::atomic<bool> tohover_ , checkforqr_, stocktake_, threadsrunning_;
  


 //data variables
 std::string last_data_;
 bool found_qr_, firstdetection_, isflying_;
 int currentwaypoint_;
 std::vector<std::string> stockfound_;
 bool servicecallsuccess_;
 double waitduration_;
//  std::map<std::string, std::string>stocklist_;

 //ros variables

ros::NodeHandle nh_;
ros::Subscriber image_sub_, pose_sub_;
ros::Publisher drone_hover_;
ros::ServiceClient waypointclient_;
ros::ServiceClient estopclient_;
ros::ServiceClient takeoffclient_;
ros::ServiceClient emergencyclient_;
ros::ServiceClient isflyingclient_;

espdrone_msgs::Hover hovercmd_;
espdrone_msgs::GoTo waypointcmd_;
std::vector<espdrone_msgs::GoTo> waypointlist_;
espdrone_msgs::Stop estop_;
espdrone_msgs::Takeoff takeoff_;
geometry_msgs::PoseStamped currentpose_;
std_srvs::SetBool setemergency_;
std_srvs::Trigger setflying_;





 //ros function
 void QRCallback(const sensor_msgs::Image::ConstPtr& msg);
 void QRDisplay (cv::Mat &im, cv::Mat &bbox);
 void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);

 //opencv variables
cv::QRCodeDetector qrDecoder_;
cv::Mat bbox_, rectifiedImage_, image_;
cv_bridge::CvImagePtr cv_ptr_;

 
  
};


