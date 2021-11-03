# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros import Buffer
from tf2_ros import TransformListener

class FrameListener(Node):
    def __init__(self):
        super().__init__('tf2_test_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        
        # print('\nSubscribe to tf messages')
        # self.tfBuffer = tf2_ros.Buffer(cache_time= CustomDuration(5.0, 0))
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer,
        #                                             self,
        #                                             spin_thread=True,
        #                                             qos=10,
        #                                             static_qos=10)

        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'velodyne'
        to_frame_rel = 'odom'

        # rate = self.create_rate(2)

        try:
            sec1, nsec1 = self.get_clock().now().seconds_nanoseconds()

            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())

            print("#### tf listner")

            sec2 = trans.header.stamp.sec
            nsec2 = trans.header.stamp.nanosec
            timediff = sec2 - sec1 + int((nsec2 - nsec1) * 1e-6) * 1e-3

            self.get_logger().warn('Current time {sec1}.{nsec1}, got tf at {sec2}.{nsec2}')
            
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            
            # rate.sleep()

def main(args=None):
    rclpy.init(args=args)

    tf2_listner = FrameListener()

    rclpy.spin(tf2_listner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tf2_listner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()