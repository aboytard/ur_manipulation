#include "ur_manipulation/moveit_custom_api.hpp"

void loggedButtonMash( MoveitCustomApi obj,
                       geometry_msgs::Pose target_pose,
                       double height,
                       int button_no,
                       EnduranceTestLogger logger,
                       std::string log_msg)
/** Assuming  the provided @param{target_pose}is the pose of the target object itself,
 * the end effector first goes to the same XY lcoation, but at a height of @param{height} above it,
 * then moves down to it,
 * and then moves back up,
 * thereby pressing the button
 */
{
    // Go to a set height above given pose---------------
    target_pose.position.z+=height;
    geometry_msgs::Pose current_pose = obj.move_group->getCurrentPose().pose;

    if(obj.comparePoses(current_pose, target_pose))
    {
        ROS_INFO_STREAM("Poses same, skipping Pre Mash Pose");
    }
    else
    {
        obj.executeCartesianTrajtoPose(target_pose,"Pre Mash Pose");
    }
    ROS_INFO("---------------------------");
    obj.sleepSafeFor(0.5);

    //Check if pose is reached, and log it
    current_pose = obj.move_group->getCurrentPose().pose;
    std::string state = button_no==1?"A":"C";
    if(obj.comparePoses(current_pose, target_pose))
    {
        ROS_INFO_STREAM("State "<< state << " succesfully achieved");
        logger.send_data_timestamped(log_msg+state, true, 0.1);
    }
//    else
//    {
//        ROS_INFO_STREAM("State "<< state << " failed");
//        logger.send_data_timestamped("State "+state+" failed.", true, 0.1);
//    }

    // Go down to press the button---------------------------
    target_pose.position.z-=height;
    obj.executeCartesianTrajtoPose(target_pose,"Mash Pose");
    obj.sleepSafeFor(0.5);
    ROS_INFO("---------------------------");

//    //Check if pose is reached, and log it
//    current_pose = move_group->getCurrentPose().pose;
//    if(comparePoses(current_pose, target_pose))
//    {
//        std::string state= button_no==1?"B":"D";
//        ROS_INFO_STREAM("State "<< state << " succesfully achieved");
//        logger.send_data_timestamped(log_msg+state, true, 0.1);
//    }
//    else
//    {
//        ROS_INFO_STREAM("State "<< state << " failed");
//        logger.send_data_timestamped("State "+state+" failed.", true, 0.1);
//    }

    // Go back up---------------------------------------------
    target_pose.position.z+=height;
    obj.executeCartesianTrajtoPose(target_pose,"Post Mash Pose");
    ROS_INFO("---------------------------");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "seher_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Logger test. Remove when no longer necessary.
  //Test server: https://github.com/ipa-kut/socket_learning/blob/master/3.multi_client/multi-client-server-threaded.py
  //Enable the print statement in the server code to print received messages to console
  EnduranceTestLogger logger1("127.0.0.1",65432);;
  logger1.connect_to_server();
//  for (int i=0; i<10; i++)
//  {
//      logger1.send_data_timestamped("Hello", true, 0.1 );
//  }

  ///TODO: Create a button masher program.
  /// 1. Send meaningful log messages
  /// 2. Perhaps logging should be added inside the pick/placeAtPoseFromHeight functions?

  //-------------Pick and place program-------------------

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Publisher pub_seq = nh.advertise<std_msgs::Header>("/ur_manipulation/sequence",1);
  ros::Publisher pub_fail = nh.advertise<std_msgs::Header>("/ur_manipulation/failure_counter",1);

  MoveitCustomApi ret_obj(atoi(argv[2]),argv[1]);
  ret_obj.initialiseMoveit(nh);
  ret_obj.printBasicInfo();
  ROS_INFO("---------------------------");
  ret_obj.addCollissionObjects();
  ROS_INFO("Moving to home pose");
  ret_obj.moveToNamedTarget("home");
  unsigned int seq = 0;

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
//    ret_obj.pickAtPoseFromHeight((switcher)?target_pose1:target_pose2, 0.03, nh, false);
//    ret_obj.placeAtPoseFromHeight((switcher)?target_pose2:target_pose1, 0.03, nh, false);
    loggedButtonMash(ret_obj,
                     (switcher)?target_pose2:target_pose1,
                     0.03,
                     (switcher)?1:2,
                     logger1,
                     "ROS");
    switcher = !switcher;
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    msg.seq = seq;
    pub_seq.publish(msg);
    msg.seq =ret_obj.failure_counter_;
    pub_fail.publish(msg);
    ROS_INFO_STREAM("----------------------SEQ " << seq++ << "-------------------------------------");
  }

  //-------------Pick and place program-------------------


  return 0;
}
