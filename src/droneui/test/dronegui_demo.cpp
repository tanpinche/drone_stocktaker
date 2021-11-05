#include <string.h>
#include <thread>
#include "droneui/droneui.hpp"


#include <ctime>


class DroneDraw : public DroneGui {
  // bool log_history_ = false;
  // std::vector<double> linear_history_;
  // std::vector<double> angular_history_;
  // int count_ = 0;
  // bool reverse_ = false;
  // double linear_velocity_, angular_velocity_;
  
  // list the ports you want to select from here.
  // const char *can_port_[2] = {"vcan0", "can0"};

//   double CalculateLinearVeloctiy() override {
//     linear_velocity_ = 0.5 * (joyctl.GetAxisValue(2) - joyctl.GetAxisValue(5));
//     if (joyctl.GetAxisValue(7) != 0) linear_velocity_ = joyctl.GetAxisValue(7);
//     // double velocity = 0.5;
//     if (log_history_)
//       linear_history_.resize(linear_history_.size() + 1, linear_velocity_);
//     if (reverse_) {
//       linear_velocity_ = -1 * linear_history_[count_];
//     }
//     return linear_velocity_;
//     // return 1;
//   };

//   double CalculateAngularVeloctiy() override {
   
//     if (linear_velocity_ < 0)
//       angular_velocity_ = -1 * joyctl.GetAxisValue(0);
//     else
//       angular_velocity_ = joyctl.GetAxisValue(0);
//     if (joyctl.GetAxisValue(6) != 0) {
//       angular_velocity_ = 0.6 * joyctl.GetAxisValue(6);
//     }
//     if (log_history_){
//       angular_history_.resize(linear_history_.size() + 1, angular_velocity_);
//       count_++;
//       }
//     if (reverse_) {
//       angular_velocity_ = -1 * angular_history_[count_];
      
//       if (count_ == 0) {
//         reverse_ = false;
//         linear_history_.clear();
//         angular_history_.clear();
//         count_ = 1;
//       }
//       count_--;
//     }
//     // velocity = 0.5;
//     return angular_velocity_;
//     // return 1;
//   };

//   void AdditionalFunctions() override {
//     if (ImGui::Button("log history")) {
//       log_history_ = true;
//     }
//     if (ImGui::Button("reverse movement")) {
//       log_history_ = false;
//       reverse_ = true;
//     }
//   }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "basicreader");
  DroneDraw dronedraw;
  dronedraw.Draw();
}
