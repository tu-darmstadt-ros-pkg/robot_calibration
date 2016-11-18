import rospy
import rosbag
import actionlib

import argo_move_group_msgs.msg

from sensor_msgs.msg import JointState
from robot_calibration_msgs.msg import CaptureConfig

bag_name = 'calibration_poses.bag'
action_topic = '/combined_planner'
frame_ids = ['chilitag{}_link'.format(i) for i in range(2)]
poses_per_frame = 10


class CapturePoses:
    def __init__(self):
        self.last_state_ = CaptureConfig()

        rospy.init_node('generate_calibration_poses')
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
            print("Saving pose %d" % self.count)
            self.count += 1

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
    goal = argo_move_group_msgs.msg.ArgoCombinedPlanActionGoal()
    goal.goal.object_type = 4
    goal.goal.action_type = 2
    goal.goal.target.header.frame_id = frame
    goal.goal.target.pose.position.z = 0.001
    return goal


if __name__ == '__main__':
    # create action client
    client = actionlib.SimpleActionClient(action_topic, argo_move_group_msgs.msg.ArgoCombinedPlanAction)
    print("Waiting 5 seconds for action server on topic", action_topic)
    client.wait_for_server(rospy.Duration.from_sec(5))

    # create bag manager
    with CapturePoses() as bag:
        for frame in frame_ids:
            for i in range(poses_per_frame):
                # send action
                client.send_goal(create_action_goal())
                print("Send goal. Waiting 5s for the result.")
                client.wait_for_result(rospy.Duration.from_sec(5))
                result = client.get_result()

                success = result.val == argo_move_group_msgs.msg.ErrorCodes.SUCCESS
                if success:
                    bag.capture_pose()
                else:
                    print("Observation planning failed with error code", result.val)
                    continue