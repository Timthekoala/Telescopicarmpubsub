# Telescopicarmpubsub

source foxy/
source workspace

cd /<workspace>/
rosdep install -i --from-path src --rosdistro foxy -y
colcon build

ros2 run thirdcam_pubsub my_publisher
or 
ros2 run thirdcam_pubsub my_subscriber
