# ZenithLocalization
Zenith Localization ROS Module Code

## Topics

### Inputs

#### /zed/odom
This takes a message of type nav_msgs/Odometry.msg for dead reckoning between detected April Tags

*note: should probably be changed to /zenith/odom and remaped to /zed/odom during launch*

#### /zenith/aprilTagsWithId
This takes in a custom msg type defined by (to be defined) and sets the robot position based off known april tags

### Outputs

#### /zenith/pose2D
Outpust a message of type geometry_msgs/Pose2D.msg (Still to be implemented)

