#!/usr/bin/env python

import rospy
import rosbag
import actionlib

import argo_move_group_msgs.msg

from sensor_msgs.msg import JointState
from robot_calibration_msgs.msg import CaptureConfig

bag_name = 'calibration_poses.bag'
action_topic = '/combined_planner'
frame_ids = ['chilitag{}_link'.format(i) for i in [28, 30, 31, 32]]
poses_per_frame = 10

client_timeout = 5
request_timeout = 30


class CapturePoses:
    def __init__(self):
        self.last_state_ = CaptureConfig()
        self.joint_states = []
        rospy.Subscriber("/combined_planner/joint_positions", JointState, self.state_cb)

        self.count = 0

    def __enter__(self):
        # bag to write data to
        self.bag = rosbag.Bag(bag_name, 'w')
        return self

    def capture_pose(self, num):
        if len(self.joint_states) == 0:
            print('Joint state is empty. Can\'t save it.')
        else:
            desired_count = self.count + num
            for state in self.joint_states:
                if self.count < desired_count:
                    capture_config = CaptureConfig()
                    capture_config.joint_states = state
                    self.bag.write('calibration_joint_states', capture_config)
                    print("-- Saving pose %d" % self.count)
                    self.count += 1
                else:
                    break

            self.joint_states = []  # clear after saving desired poses

    def __exit__(self, exc_type, exc_value, traceback):
        self.bag.close()

    def state_cb(self, msg):
        """ Callback for joint_states messages """
        self.joint_states.append(msg)


def create_action_goal(frame):
    goal = argo_move_group_msgs.msg.ArgoCombinedPlanGoal()
    goal.object_type.data = "chilitag"
    goal.action_type.val = argo_move_group_msgs.msg.ActionCodes.SAMPLE
    goal.target.header.frame_id = frame
    goal.target.pose.position.z = -0.0
    # by default, the observation planner looks at the z-axis
    # since the chilitag z-axis points inward, we need to rotate the target pose
    goal.target.pose.orientation.x = 1
    goal.target.pose.orientation.w = 0
    return goal


if __name__ == '__main__':
    rospy.init_node('generate_calibration_poses')
    if poses_per_frame > 50:
        rospy.logerr("Can only generate a maximum number of 50 poses per frame.")
        exit(0)
    # create action client
    client = actionlib.SimpleActionClient(action_topic, argo_move_group_msgs.msg.ArgoCombinedPlanAction)
    print("Waiting", client_timeout, "seconds for action server on topic", action_topic)
    if not client.wait_for_server(rospy.Duration.from_sec(client_timeout)):
      rospy.logerr("No connection to argo combined plan action server")
      exit(0)

    # create bag manager
    with CapturePoses() as bag:
        for frame in frame_ids:
            print("## Generating", poses_per_frame, "poses for frame", frame)
            # send action
            client.send_goal(create_action_goal(frame))
            print("Waiting", request_timeout, "seconds for the result.")
            client.wait_for_result(rospy.Duration.from_sec(request_timeout))
            result = client.get_result() #TODO catch none
            if result is None:
                print("-- No result received in time.")
            else:
                success = (result.success.val == argo_move_group_msgs.msg.ErrorCodes.SUCCESS)
                if success:
                    bag.capture_pose(poses_per_frame)
                else:
                    print("-- Observation planning failed with error code", result.success.val)
                    # TODO repeat planning x times?
                    continue
