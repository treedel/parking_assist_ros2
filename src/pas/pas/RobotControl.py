from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node

from time import sleep

class RobotControl(Node):
    # Reference for euler to quart conversion
    preset_ref = {
        -1.57: [-0.707, 0.707],
        0: [1.0, 0.0],
        1.57: [0.707, 0.707],
        3.14: [0.0, 1.0]
    }

    def __init__(self, set_initial_pose=False, initial_pose=(0, 0, 0)):
        rclpy.init()
        super().__init__('coordinator_client')

        self.navigator = BasicNavigator()
        if set_initial_pose: 
            self.initial_pose = initial_pose
            self.navigator.setInitialPose(self.eulerToMapPose(self.initial_pose))
        self.navigator.waitUntilNav2Active()

    # Basic rad to quartion conversion function
    # If the angle is not present in the dict, returns a default value to avoid crashing
    def simpleRadToQuart(self, rad):
        res = self.preset_ref.get(rad)
        if res: return res
        return [0.707, 0.707]

    # Converts (x, y, theta) into map pose
    def eulerToMapPose(self, map_pose):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = map_pose[0]
        pose.pose.position.y = map_pose[1]

        quart = self.simpleRadToQuart(map_pose[2])

        pose.pose.orientation.z = quart[0]
        pose.pose.orientation.w = quart[1]

        return pose

    # Makes NAV2 navigate to the given pose tuple
    def goToEulerPose(self, pose_tuple):      
        has_completed = False

        while not has_completed:
            self.navigator.goToPose(self.eulerToMapPose(pose_tuple))
            while not self.navigator.isTaskComplete(): sleep(1)
            if (self.navigator.getResult() == TaskResult.SUCCEEDED): has_completed = True
            sleep(3)

def main():
    robot = RobotControl(True, (0.0, 0.0, 0.0))
    robot.goToEulerPose((10.0, 0.0, 3.14))
    
if __name__ == "__main__":
    main()