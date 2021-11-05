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
import time

from tf2_ros import TransformException
from tf2_ros import Buffer
from tf2_ros import TransformListener

class FrameListener(Node):
    def __init__(self):
        super().__init__('tf2_test_listener')

        print('\nSubscribe to tf messages')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, 
                                             self, 
                                             spin_thread=True, 
                                             qos=10,
                                             static_qos=10)
        
        self.call_tf2_listener()

    def call_tf2_listener(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'velodyne'
        to_frame_rel = 'odom'
        mean_timediff = None

        while(True):
            try:
                sec1, nsec1 = self.get_clock().now().seconds_nanoseconds()

                trans = self.tf_buffer.lookup_transform(to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())

                # print("#### tf listner")

                sec2 = trans.header.stamp.sec
                nsec2 = trans.header.stamp.nanosec
                timediff = sec1 - sec2 + (nsec1 - nsec2) * 1e-9

                if mean_timediff is None:
                    mean_timediff = timediff
                else:
                    mean_timediff = 0.98 * mean_timediff + 0.02 * timediff

                # self.get_logger().warn(f'Current time {sec1}.{nsec1}, got tf at {sec2}.{nsec2}, time difference {timediff}')

                self.get_logger().warn('timediff {:.3f} - mean {:.3f}'.format(timediff, mean_timediff))
                
            except TransformException as ex:
                self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            
            time.sleep(0.001)

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