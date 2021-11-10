#include "droneui/droneui.hpp"



static void glfw_error_callback(int error, const char* description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}


DroneGui::DroneGui(uint32_t width, uint32_t height, std::string title) {
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
  threadsrunning_ = true;
  firstdetection_ =  true;
  isflying_ = false;
  servicecallsuccess_ = false;
  setemergency_.request.data = true;
  currentwaypoint_ = 0;
  stockfound_.clear();
  last_data_ = "Not found any stock yet";

  //initialise cmd values
  hovercmd_.vx = 0;
  hovercmd_.vy = 0;
  hovercmd_.yawrate =0;
  hovercmd_.zDistance = 1;



// 
espdrone_msgs::GoTo waypoints[5];
waypoints[0].request.duration = ros::Duration(3.0);
waypoints[0].request.relative = false;
waypoints[0].request.goal.x=-0.522;
waypoints[0].request.goal.y=0.716;
waypoints[0].request.goal.z= 0.089;
waypoints[0].request.yaw = -11.591; 

waypoints[1].request.duration = ros::Duration(3.0);
waypoints[1].request.relative = false;
waypoints[1].request.goal.x=0.24;
waypoints[1].request.goal.y=1.265;
waypoints[1].request.goal.z= -0.064;
waypoints[1].request.yaw = -53.523;

waypoints[2].request.duration = ros::Duration(3.0);
waypoints[2].request.relative = false;
waypoints[2].request.goal.x=0.895;
waypoints[2].request.goal.y=1.428;
waypoints[2].request.goal.z= 0.121;
waypoints[2].request.yaw = -50.237;



waypoints[3].request.duration = ros::Duration(3.0);
waypoints[3].request.relative = false;
waypoints[3].request.goal.x=1.248;
waypoints[3].request.goal.y=1.621;
waypoints[3].request.goal.z= 0.321;
waypoints[3].request.yaw = -64.17;

waypoints[4].request.duration = ros::Duration(3.0);
waypoints[4].request.relative = false;
waypoints[4].request.goal.x=-1.646;
waypoints[4].request.goal.y=1.451;
waypoints[4].request.goal.z= 0.292;
waypoints[4].request.yaw = 246.505;





waypointlist_.clear();
for(int x = 0; x<5; x++){
  waypointlist_.push_back(waypoints[x]);


}

 takeoff_.request.height = 0.5;
 takeoff_.request.duration = ros::Duration(3);

//  emergency.data = true;



  //set thread
  hover_thread_ = std::thread([this]() {
    while(threadsrunning_){
    if(tohover_){
      if(!isflying_){
        hovercmd_.zDistance = takeoff_.request.height;
      }
      else{
      hovercmd_.zDistance = currentpose_.pose.position.z;
      drone_hover_.publish(hovercmd_);
      }
    }
    }
    
  });

  waypoint_thread_ = std::thread([this]() {
    while(threadsrunning_){
    if(stocktake_){
    waypointcmd_ = waypointlist_[currentwaypoint_];
    tohover_ =  false;
    if(waypointclient_.call(waypointcmd_)){
      ROS_INFO("moving to point %d\n",currentwaypoint_);
      std::this_thread::sleep_for(std::chrono::seconds(3));
      servicecallsuccess_ = true;
    }
    else {
      servicecallsuccess_ = false;
    }
    if(waypointlist_.size()>currentwaypoint_ +1 ){
      std::cout<<"incrementing waypoint index";
      currentwaypoint_ ++;
    }
    else{
      std::cout<<"end stocktake"<<"/n";
      stocktake_ = false;
    }

    tohover_ =  true;
    
    }
    }
  }); 


 // initialise ros subsriber and publisher
 image_sub_ = nh_.subscribe<sensor_msgs::Image>("/espdrone/image_rect_color", 10, &DroneGui::QRCallback, this);
 pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/espdrone/pose", 10, &DroneGui::PoseCallback, this);
 drone_hover_ = nh_.advertise<espdrone_msgs::Hover>("/espdrone/cmd_hover", 1000);
 waypointclient_ = nh_.serviceClient<espdrone_msgs::GoTo>("/espdrone/go_to");
 estopclient_ = nh_.serviceClient<espdrone_msgs::Stop>("/espdrone/stop");
 takeoffclient_ = nh_.serviceClient<espdrone_msgs::Takeoff>("/espdrone/takeoff");
 emergencyclient_ = nh_.serviceClient<std_srvs::SetBool>("/espdrone/emergency");
 isflyingclient_ = nh_.serviceClient<std_srvs::SetBool>("/espdrone/is_flying");


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
    //ImGui::SetWindowFontScale(1.2);
    
    ImGui::Columns(2, "UI");
    ImGui::Image( reinterpret_cast<void*>( static_cast<intptr_t>( texture ) ), ImVec2( image_.cols, image_.rows ) );
    ImGui::NextColumn();
    if (ImGui::Button("Take off")) {
      if(!isflying_){
        isflyingclient_.call(setflying_);
        isflying_ = true;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Start stocktake")) {
      stocktake_ = true;
    }
    ImGui::SameLine();
     if (ImGui::Button("Emergency Stop")) {
       emergencyclient_.call(setemergency_);
      threadsrunning_ = false;
      tohover_ = false;
      stocktake_ = false;
      isflying_ = false;
    }
    
    ImGui::Text("Number of Waypoints in route: %d", waypointlist_.size());

    ImGui::Text("\n\nPose\nX:%f\nY:%f\nZ:%f", currentpose_.pose.position.x,currentpose_.pose.position.y,currentpose_.pose.position.z);
    ImGui::SetWindowFontScale(1.2f); 
    ImGui::Text("\nDrone Status\n");
    ImGui::SetWindowFontScale(1);


    if(isflying_){
    ImGui::Text("Flight Status: Flying");}

    else 
    ImGui::Text("Flight Status: NOT Flying");
    if(stocktake_){
    ImGui::Text("Status: checking for qrcode");
    }
    if(found_qr_){
      ImGui::Text("QR code detected");

    }
    else {
      ImGui::Text("QR code not detected");


    }
    ImGui::Text(" \nStock Found:");
    if(stockfound_.size()>0)
    for(int i =0; i<stockfound_.size(); i++){
      if(stockfound_[i].compare("Mango")==0)
      ImGui::Text("B02: %s", stockfound_[i].c_str());

      else if(stockfound_[i].compare("Banana")==0)
      ImGui::Text("C01: %s", stockfound_[i].c_str());

      
      else if(stockfound_[i].compare("Orange")==0)
      ImGui::Text("F02: Orange");


      else if(stockfound_[i].compare("Durian")==0)
      ImGui::Text("G02: Durian");


      else if(stockfound_[i].compare("Apple")==0)
      ImGui::Text("G01: Apple");
      


    }
    else {
        ImGui::Text("No Stock Detected Yet");

    }

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
  threadsrunning_ = false;
  hover_thread_.join();
  waypoint_thread_.join();
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
    
    //std::cout<<"checking for qr"<<std::endl;
    
     
     std::string qrdata = qrDecoder_.detectAndDecode(image_, bbox_, rectifiedImage_);
    // std::cout<<qrdata.length()<<std::endl;

     if(qrdata.length()>0){

     //  std::cout<<"data detected"<<std::endl;
       
    
    if(last_data_.compare(qrdata)!=0){
    //std::cout<<"data is new"<<std::endl;
    found_qr_ = true;
    last_data_ = qrdata;
    if(stockfound_.size() == 0){
      //std::cout<<"updating first entry"<<std::endl;
      stockfound_.resize(1);
      stockfound_[0] = last_data_;
    }
    else{
      //std::cout<<"adding new entry"<<std::endl;
    stockfound_.push_back(last_data_);}

    //std::cout << "Decoded Data : " << qrdata << std::endl;

    //  QRDisplay(image_, bbox_);
    //  rectifiedImage_.convertTo(rectifiedImage_, CV_8UC3);
    //  cv::imshow("Rectified QRCode", rectifiedImage_);
    //  cv::waitKey(0);
     }


     }
     else {
       if(found_qr_){
         found_qr_ = false;
         //std::cout<<"No QR code detected"<<std::endl;
       }
     }
    }
    cv::cvtColor( image_, image_, cv::COLOR_BGR2RGBA );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, image_.cols, image_.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_.data );


}

void DroneGui::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
currentpose_.pose = pose->pose;
}

void DroneGui::QRDisplay(cv::Mat &im, cv::Mat &bbox){
   int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    line(im, cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), cv::Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), cv::Scalar(255,0,0), 3);
  }
  imshow("Result", im);

}
