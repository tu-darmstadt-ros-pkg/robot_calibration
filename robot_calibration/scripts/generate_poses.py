#!/usr/bin/env python

import rospy
import rosbag
import actionlib

import argo_move_group_msgs.msg

from sensor_msgs.msg import JointState
from robot_calibration_msgs.msg import CaptureConfig

bag_name = 'calibration_poses.bag'
action_topic = '/combined_planner'
frame_ids = ['chilitag{}_link'.format(i) for i in [4, 5, 7]]
poses_per_frame = 10

client_timeout = 5
request_timeout = 30


class CapturePoses:
    def __init__(self):
        self.last_state_ = CaptureConfig()
        rospy.Subscriber("joint_states", JointState, self.state_cb)

        self.count = 0

    def __enter__(self):
        # bag to write data to
        self.bag = rosbag.Bag(bag_name, 'w')
        return self

    def capture_pose(self):
        if len(self.last_state_.joint_states.name) == 0:
            print('Joint state is empty. Can\'t save it.')
        else:
            self.bag.write('calibration_joint_states', self.last_state_)
            self.count += 1
            print("-- Saving pose %d" % self.count)

    def __exit__(self, exc_type, exc_value, traceback):
        self.bag.close()

    def state_cb(self, msg):
        """ Callback for joint_states messages """
        for joint, position in zip(msg.name, msg.position):
            try:
                idx = self.last_state_.joint_states.name.index(joint)
                self.last_state_.joint_states.position[idx] = position
            except ValueError:
                self.last_state_.joint_states.name.append(joint)
                self.last_state_.joint_states.position.append(position)


def create_action_goal():
    goal = argo_move_group_msgs.msg.ArgoCombinedPlanGoal()
    goal.object_type.data = "chilitag"
    goal.action_type.val = argo_move_group_msgs.msg.ActionCodes.SAMPLE_MOVE_ARM
    goal.target.header.frame_id = frame
    goal.target.pose.position.z = 0.001
    return goal


if __name__ == '__main__':
    rospy.init_node('generate_calibration_poses')
    # create action client
    client = actionlib.SimpleActionClient(action_topic, argo_move_group_msgs.msg.ArgoCombinedPlanAction)
    print("Waiting", client_timeout, "seconds for action server on topic", action_topic)
    client.wait_for_server(rospy.Duration.from_sec(client_timeout)) #TODO warning if no connection

    # create bag manager
    with CapturePoses() as bag:
        for frame in frame_ids:
            print("## Generating", poses_per_frame, "poses for frame", frame)
            for i in range(poses_per_frame):
                # send action
                client.send_goal(create_action_goal())
                print("Planning pose", i, ". Waiting", request_timeout, "seconds for the result.")
                client.wait_for_result(rospy.Duration.from_sec(request_timeout))
                result = client.get_result() #TODO catch none
                if result is None:
                    print("-- No result received in time.")
                else:
                    success = (result.success.val == argo_move_group_msgs.msg.ErrorCodes.SUCCESS)
                    if success:
                        bag.capture_pose()
                    else:
                        print("-- Observation planning failed with error code", result.success.val)
                        # TODO repeat planning x times?
                        continue