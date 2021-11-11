# drone_stocktaker

Repository has 2 submodules, use recursive when cloning.

submodule branches:

imgui -> branch 58075c4414

implot -> branch e64df657b2


depencies:

opencv version 4.2.0 - build and install from souce. Make sure to included quirc library (it is usually included by default)

glfw3

sudo apt-get install libglfw3-dev


go to catkin root folder "catkin_ws"

catkin build

rosrun drone_stocktaker droneguidemo 
