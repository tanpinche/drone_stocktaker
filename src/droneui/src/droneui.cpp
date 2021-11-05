#include "droneui/droneui.hpp"



static void glfw_error_callback(int error, const char* description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}


DroneGui::DroneGui(uint32_t width, uint32_t height, std::string title) {
  // selectedjs_ = 0;
  // selectedcan_ = 0;
  // selectedrobot_ = 0;

  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) return;

  // Decide GL+GLSL versions
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
  // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only

  // Create window with graphics context
  window_ = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
  if (window_ == NULL) return;
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);  // Enable vsync

  // Initialize OpenGL loader
  if (gl3wInit() != 0) return;

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  // Enable Keyboard Controls
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  //   io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  // Enable Gamepad Controls

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Set default background color to be white
  background_color_ = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);

  // Everything is okay if reached here
  initialized_ = true;

  //set variable values
  checkforqr_ =true;
  tohover_ =  false;
  stocktake_ = false;
  found_qr_ = false;
  currentwaypoint_ = 0;
  stockfound_.resize(1);
  last_data_ = "Not found any stock yet";
  stockfound_[0] =  last_data_;

  //initialise cmd values
  hovercmd_.vx = 0;
  hovercmd_.vy = 0;
  hovercmd_.yawrate =0;
  hovercmd_.zDistance = 1;


 waypointlist_[0].request.duration = ros::Duration(5.0);
 waypointlist_[0].request.relative = false;
 waypointlist_[0].request.goal.x=0.09;
 waypointlist_[0].request.goal.y=0.07;
 waypointlist_[0].request.goal.z= 1.5;
 waypointlist_[0].request.yaw = 0;



  //set thread
  hover_thread_ = std::thread([this]() {
    if(tohover_){
      hovercmd_.zDistance = currentpose_.pose.position.z;
      drone_hover_.publish(hovercmd_);
    }
    
});

  waypoint_thread_ = std::thread([this]() {
    if(stocktake_){
    waypointcmd_ = waypointlist_[currentwaypoint_];
    tohover_ =  false;
    if(waypointclient_.call(waypointcmd_)){
      ROS_INFO("moving to point %d\n",currentwaypoint_);
      waypointcmd_.request.duration.sleep();
    }
    tohover_ =  true;

    }
    
});


 // initialise ros subsriber and publisher
 image_sub_ = nh_.subscribe<sensor_msgs::Image>("/espdrone/image_rect_color", 10, &DroneGui::QRCallback, this);
 pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/espdrone/pose", 10, &DroneGui::PoseCallback, this);
 drone_hover_ = nh_.advertise<espdrone_msgs::Hover>("/espdrone/cmd_hover", 1000);
 waypointclient_ = nh_.serviceClient<espdrone_msgs::GoTo>("/espdrone178/go_to");


 // initialise gluint textures
 glGenTextures( 1, &texture );
 glBindTexture( GL_TEXTURE_2D, texture );
 glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
 glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
 glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );

}

void DroneGui::Draw() {
  

  while (!glfwWindowShouldClose(window_)) {
    // Poll and handle events (inputs, window resize, etc.)
    glfwPollEvents();

    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) break;

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Begining of ImGui drawing

    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.2, 0.2, 0.35, 1));
    // set size of imgui window to match glfwWindow
    glfwGetWindowSize(window_, &window_width_, &window_height_);
    ImGui::SetWindowSize(ImVec2(window_width_, window_height_));
    ImGui::SetWindowPos(ImVec2(0, 0));

    // // resize window to fit all widgets after connecting to a joystick
    // if (resize_after_connect_) {
    //   glfwSetWindowSize(window_, current_width_ + 150, current_height_ + 150);
    //   resize_after_connect_ = false;
    // }
    // // button to manually resize window to fit all existing widgets
    // if (ImGui::Button("Fit Window")) {
    //   glfwSetWindowSize(window_, current_width_ + 150, current_height_ + 150);
    // }
    // // interface for connecting to joystick
    // ImGui::Text("Joystick Settings");
    // ImGui::Text("Connect to:");
    // ImGui::SameLine();
    // ImGui::PushItemWidth(50);
    // // ImGui::Combo("", &selectedjs_, js, joyctl.list_of_joystick_.size());
    // ImGui::SameLine();
    
    ImGui::Columns(2, "UI");
    ImGui::Image( reinterpret_cast<void*>( static_cast<intptr_t>( texture ) ), ImVec2( image_.cols, image_.rows ) );
    ImGui::NextColumn();
    if (ImGui::Button("Start stocktake")) {
      stocktake_ = true;
    }
    ImGui::SameLine();
     if (ImGui::Button("Emergency Stop")) {
      tohover_ = false;
      stocktake_ = false;
    }
    ImGui::Text("Pose\nX:%f\nY:%f\nZ:%f", currentpose_.pose.position.x,currentpose_.pose.position.y,currentpose_.pose.position.z);
    if(checkforqr_){
    ImGui::Text("Status: checking for qrcode");
    }
    if(found_qr_){
      ImGui::Text("QR code detected");

    }
    else {
      ImGui::Text("QR code not detected");


    }
    ImGui::Text(" \nStock Found:");
    for(int i =0; i<stockfound_.size(); i++){
      ImGui::Text("%s", stockfound_[i].c_str());
      


    }
    

    // // display joystick info
    // auto joystick_name = joyctl.GetCurrentDeviceName();
    // ImGui::SameLine();
    // ImGui::Text("Connected to");
    // ImGui::Text(device_name_.c_str());
    // ImGui::Text("Has %d Axes", joyctl.GetNumberOfAxis());
    // ImGui::SameLine();
    // ImGui::Text("and %d buttons", joyctl.GetNumberOfButton());
    // ImGui::Text("Thread Status:");
    // ImGui::SameLine();
    // // display ReadThread status
    // if (joyctl.IsThreadActive())
    //   ImGui::TextColored(ImVec4(0, 1, 0, 1), "ACTIVE");
    // else
    //   ImGui::TextColored(ImVec4(1, 0, 0, 1), "INACTIVE");
    // if (ImGui::Button("Start Reading Thread")) joyctl.StartReadThread();
    // ImGui::SameLine();
    // if (ImGui::Button("Stop Reading Thread")) joyctl.StopRead();

    // // interface for connecting to CAN
    // const char* can_port[can_port_.size()];
    // for (int x = 0; x < can_port_.size(); x++) {
    //   can_port[x] = can_port_[x].c_str();
    // }
    // ImGui::Text("CAN Settings");
    // ImGui::PushItemWidth(65);
    // ImGui::Combo(" ", &selectedcan_, can_port, IM_ARRAYSIZE(can_port));
    // ImGui::SameLine();
    // if (ImGui::Button("Connect to CAN")) {
    //   device_name_ = can_port_[selectedcan_];
    // }
    // ImGui::SameLine();
    // char user_input[20]="";
    // ImGui::InputText("", user_input, IM_ARRAYSIZE(user_input));
    // ImGui::SameLine();
    // if (ImGui::Button("add port")) {
    //   can_port_.push_back(user_input);
      
    // }
    
    // ImGui::Text("Connected to");
    // ImGui::SameLine();
    // ImGui::Text(device_name_.c_str());
    // ImGui::Text(" ");

    // // Interface for selecting robot type
    // ImGui::Text("Robot Settings");
    // ImGui::Combo("  ", &selectedrobot_, robot_type_, IM_ARRAYSIZE(robot_type_));

    // ImGui::SameLine();
    // if (ImGui::Button("Select Robot")) {
    //   switch (selectedrobot_) {
    //     case 0:
    //       linked_robot_ = selectedrobot_;
    //       scout.Connect(device_name_);
    //       break;

    //     case 1:
    //       linked_robot_ = selectedrobot_;
    //       // TracerBase tracer;
    //       tracer.Connect(device_name_);
    //       break;
    //   }
    // }
    // ImGui::Text("Connected to");
    // ImGui::SameLine();
    // ImGui::Text(robot_type_[linked_robot_]);
    // ImGui::Text(" ");

    // // PublishThread status and control
    // ImGui::Text("Robot Type");
    // ImGui::Text("Sending Inputs:");
    // ImGui::SameLine();
    // if (send_inputs)
    //   ImGui::TextColored(ImVec4(0, 1, 0, 1), "ACTIVE");
    // else
    //   ImGui::TextColored(ImVec4(1, 0, 0, 1), "INACTIVE");
    // if (ImGui::Button("Start Sending Joystick Data")) send_inputs = true;
    // ImGui::SameLine();
    // if (ImGui::Button("Stop Sending Joystick Data")) send_inputs = false;

    // // diplaying values of joystick axes and buttons
    // ImGui::Columns(2, "Values");
    // ImGui::SetColumnWidth(0, 420);
    // for (int i = 0; i < joyctl.GetNumberOfAxis(); i++) {
    //   ImGui::Text("Axis %d", i);
    //   cursor_pos_ = ImGui::GetCursorScreenPos();

    //   // creating of the progress bar to display negative axis values
    //   ImGui::GetWindowDrawList()->AddRectFilled(
    //       cursor_pos_, ImVec2(cursor_pos_.x + 199, cursor_pos_.y + 19),
    //       ImGui::GetColorU32(ImGuiCol_PlotHistogram));
    //   ImGui::GetWindowDrawList()->AddRectFilled(
    //       cursor_pos_,
    //       ImVec2(cursor_pos_.x + 199 - joyctl.GetAxisValue(i) * -199,
    //              cursor_pos_.y + 19),
    //       ImGui::GetColorU32(ImGuiCol_FrameBg));

    //   ImGui::SetCursorPos(
    //       ImVec2(cursor_pos_.x + 176 - joyctl.GetAxisValue(i) * -199,
    //              cursor_pos_.y + 3));
    //   if (joyctl.GetAxisValue(i) < 0 && joyctl.GetAxisValue(i) > -0.85)
    //     ImGui::Text("%.0f%%", -100 * joyctl.GetAxisValue(i));
    //   else if (joyctl.GetAxisValue(i) < 0) {
    //     ImGui::SetCursorPos(ImVec2(cursor_pos_.x + 2, cursor_pos_.y + 2));
    //     ImGui::Text("%.0f%%", -100 * joyctl.GetAxisValue(i));
    //   }

    //   cursor_pos_.x += 200;
    //   ImGui::SetCursorPos(cursor_pos_);
    //   ImGui::ProgressBar(joyctl.GetAxisValue(i), ImVec2(199.0f, 0.0f));
    // }
    // ImGui::NextColumn();
    // for (int i = 0; i < joyctl.GetNumberOfButton(); i++) {
    //   if (joyctl.GetButtonValue(i) == 1) {
    //     ImGui::Text("Button %d:", i);
    //     ImGui::SameLine();
    //     ImGui::TextColored(ImVec4(0, 1, 0, 1), "ON");
    //   } else
    //     ImGui::Text("Button %d: OFF", i);

      // cursor_pos_ = ImGui::GetCursorPos();
      // if (current_width_ < cursor_pos_.x) current_width_ = cursor_pos_.x;
      // if (current_height_ < cursor_pos_.y) current_height_ = cursor_pos_.y;
    

    // AdditionalFunctions();
    // // send commands to robot
    // if (joyctl.IsThreadActive() && send_inputs) {
    //   switch (linked_robot_) {
    //     case 0:
    //       scout.SetMotionCommand(CalculateLinearVeloctiy(),
    //                              CalculateAngularVeloctiy());
    //       break;

    //     case 1:
    //       tracer.SetMotionCommand(CalculateLinearVeloctiy(),
    //                               CalculateAngularVeloctiy());
    //       break;
    //   }
    // }

    // ImGui::Columns();
    

    //end with ros::spin
    ros::spinOnce();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(background_color_.x, background_color_.y, background_color_.z,
                 background_color_.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window_);
  
};
}

DroneGui::~DroneGui() {
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window_);
  glfwTerminate();
}

double DroneGui::CalculateLinearVeloctiy() { return 0; }

double DroneGui::CalculateAngularVeloctiy() { return 0; };

void DroneGui::AdditionalFunctions() { int x = 1; }

void DroneGui::QRCallback(const sensor_msgs::Image::ConstPtr& msg){

    cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     
    image_ = cv_ptr_->image;

    if(checkforqr_){
  
     
     std::string data = qrDecoder_.detectAndDecode(image_, bbox_, rectifiedImage_);

     if(data.length()>0){
    
    if(!last_data_.compare(data)){
    found_qr_ = true;
    last_data_ = data;
    if(stockfound_.size() == 1){
      stockfound_[0] = last_data_;
    }
    else{
    stockfound_.push_back(last_data_);}

    std::cout << "Decoded Data : " << data << std::endl;

    //  Display(image, bbox);
    //  rectifiedImage_.convertTo(rectifiedImage_, CV_8UC3);
    //  cv::imshow("Rectified QRCode", rectifiedImage_);
    //  cv::waitKey(0);
     }


     }
     else {
       if(found_qr_){
         found_qr_ = false;
         std::cout<<"No QR code detected"<<std::endl;
       }
     }
    }
    cv::cvtColor( image_, image_, cv::COLOR_BGR2RGBA );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, image_.cols, image_.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_.data );


}

void DroneGui::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
currentpose_.pose = pose->pose;

}
