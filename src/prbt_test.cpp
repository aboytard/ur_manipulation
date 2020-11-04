#include "ur_manipulation/moveit_custom_api.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prbt_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveitCustomApi prbt_obj(5,"false");
    prbt_obj.initialiseMoveit(nh);
    prbt_obj.printBasicInfo();
    ROS_INFO("---------------------------");
    ROS_INFO("Moving to ready pose");
    prbt_obj.moveToNamedTarget("ready");

    int seq = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    /// The square waypoint traj code does not work properly, as the robot does some weird movements in between,
    /// and some trajectories are aborted because starting point deviates too much from expected point
    /// My guess is that this is because of settling time of PRBT controller after finishing trajectory.
    /// Maybe a sleep to settle feature can fix this?

    //    geometry_msgs::Pose wp_pose1 = prbt_obj.move_group->getCurrentPose().pose;
    //    waypoints.push_back(wp_pose1);

    //    wp_pose1.position.x += 0.1;
    //    waypoints.push_back(wp_pose1);
    //    wp_pose1.position.x += 0.1;
    //    waypoints.push_back(wp_pose1);

    //    wp_pose1.position.z -= 0.1;
    //    waypoints.push_back(wp_pose1);
    //    wp_pose1.position.z -= 0.1;
    //    waypoints.push_back(wp_pose1);

    //    wp_pose1.position.x -= 0.1;
    //    waypoints.push_back(wp_pose1);
    //    wp_pose1.position.x -= 0.1;
    //    waypoints.push_back(wp_pose1);

    //    wp_pose1.position.z += 0.1;
    //    waypoints.push_back(wp_pose1);
    //    wp_pose1.position.z += 0.1;
    //    waypoints.push_back(wp_pose1);

    //    while(ros::ok())
    //    {
    //        ROS_INFO_STREAM("----------------------SEQ " << seq << "-------------------------------------");
    //        prbt_obj.executeCartesianTrajForWaypoints(waypoints,0.1);
    //        prbt_obj.sleepSafeFor(0.5);
    //        ROS_INFO_STREAM("----------------------SEQ " << seq++ << "-------------------------------------");
    //    }

    ROS_INFO("Starting PnP");
    ROS_INFO("---------------------------");

    geometry_msgs::Pose target_pose1;

    target_pose1.position.x = 0.3;
    target_pose1.position.y = 0.4;
    target_pose1.position.z = 0.012;
    geometry_msgs::Quaternion quat_msg;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(180),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
    target_pose1.orientation = quat_msg;

    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose1.position.x = 0.05;


    bool switcher=false;
    while(ros::ok())
    {
        ROS_INFO_STREAM("----------------------SEQ " << seq << "-------------------------------------");
        prbt_obj.pickAtPoseFromHeight((switcher)?target_pose1:target_pose2, 0.03, nh, false);
        prbt_obj.placeAtPoseFromHeight((switcher)?target_pose2:target_pose1, 0.03, nh, false);
        switcher = !switcher;
        ROS_INFO_STREAM("----------------------SEQ " << seq++ << "-------------------------------------");
    }


    std::cout << "Hello world " << std::endl;

}
