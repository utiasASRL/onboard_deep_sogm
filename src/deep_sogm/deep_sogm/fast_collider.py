#!/usr/bin/env python3

#
#
#      0=================================0
#      |    Kernel Point Convolutions    |
#      0=================================0
#
#
# ----------------------------------------------------------------------------------------------------------------------
#
#      Callable script to start a training on ModelNet40 dataset
#
# ----------------------------------------------------------------------------------------------------------------------
#
#      Hugues THOMAS - 06/03/2020
#


# ----------------------------------------------------------------------------------------------------------------------
#
#           Imports and global variables
#       \**********************************/
#


# Path setup
import os
from os.path import exists, join
import sys
from numpy.core.numeric import False_
ENV_USER = os.getenv('USER')
ENV_HOME = os.getenv('HOME')
ENV_USER = 'administrator'
ENV_HOME = '/home/administrator'
sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm"))
sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm/utils"))
sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm/models"))
sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm/kernels"))
sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm/cpp_wrappers"))

# Common libs
import sklearn
import torch
# import pickle
import time
os.environ.update(OMP_NUM_THREADS='1',
                  OPENBLAS_NUM_THREADS='1',
                  NUMEXPR_NUM_THREADS='1',
                  MKL_NUM_THREADS='1',)
import numpy as np
# import torch.multiprocessing as max_in_p
from utils.mayavi_visu import zoom_collisions, fast_save_future_anim

# Useful classes
from utils.config import Config
from utils.ply import read_ply, write_ply
from models.architectures import KPCollider
from kernels.kernel_points import create_3D_rotations
from torch.utils.data import DataLoader, Sampler
from fast_collider_data import MyhalCollisionCollate, OnlineDataset, OnlineSampler
from scipy import ndimage
import scipy.ndimage.filters as filters
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as scipyR
import threading
import imageio

# ROS
# import rospy
# import ros_numpy
# from ros_numpy import point_cloud2 as pc2
from utils.pc2_numpy import pointcloud2_to_array, get_xyz_points, array_to_pointcloud2


import rclpy
from rclpy.node import Node
from rclpy.time import Time as rclTime

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PolygonStamped, Point32


from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from msg_interfaces.msg import VoxGrid

import tf2_ros
from tf2_msgs.msg import TFMessage

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.duration import Duration
# from rclpy.time import Time

# for pausing gazebo during computation:
from std_srvs.srv import Empty
# from sensor_msgs.msg import LaserScan

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class CustomDuration():

    def __init__(self, sec0=0, nanosec0=0):
        self.sec = sec0
        self.nanosec = nanosec0

    def __eq__(self, other):
        if other is None:
            return False
        else:
            return (self.sec == other.sec) and (self.nanosec == other.nanosec)

class OnlineCollider(Node):

    def __init__(self, training_path, chkp_name):
        super().__init__('online_collider')

        ####################
        # Init environment #
        ####################

        # Set which gpu is going to be used (auto for automatic choice)
        on_gpu = True
        GPU_ID = '0'

        # Automatic choice (need pynvml to be installed)
        if GPU_ID == 'auto':
            print('\nSearching a free GPU:')
            for i in range(torch.cuda.device_count()):
                a = torch.cuda.list_gpu_processes(i)
                print(torch.cuda.list_gpu_processes(i))
                a = a.split()
                if a[1] == 'no':
                    GPU_ID = a[0][-1:]

        # Safe check no free GPU
        if GPU_ID == 'auto':
            print('\nNo free GPU found!\n')
            a = 1/0

        else:
            print('\nUsing GPU:', GPU_ID, '\n')

        # Get the GPU for PyTorch
        if on_gpu and torch.cuda.is_available():
            self.device = torch.device("cuda:{:d}".format(int(GPU_ID)))
        else:
            self.device = torch.device("cpu")
            
        ############
        # Init ROS #
        ############

        self.obstacle_range = 1.2
        self.norm_p = 4
        self.norm_invp = 1 / self.norm_p

        #rclpy.init(args=sys.argv)
        #self.node = rclpy.create_node('fast_collider')
        #self.node.get_logger().info('Created node: fast_collider')

        ######################
        # Load trained model #
        ######################

        print('\nModel Preparation')
        print('*****************')
        t1 = time.time()

        # Choose which training checkpoints to use
        chkp_path = os.path.join(training_path, 'checkpoints', chkp_name)

        # Load configuration class used at training
        self.config = Config()
        self.config.load(training_path)

        # Init data class
        self.online_dataset = OnlineDataset(self.config)
        self.online_sampler = OnlineSampler(self.online_dataset)
        self.online_loader = DataLoader(self.online_dataset,
                                        batch_size=1,
                                        sampler=self.online_sampler,
                                        collate_fn=MyhalCollisionCollate,
                                        num_workers=1,
                                        pin_memory=True)

        # Define network model
        self.net = KPCollider(self.config, self.online_dataset.label_values, self.online_dataset.ignored_labels)
        self.net.to(self.device)
        self.softmax = torch.nn.Softmax(1)
        self.sigmoid_2D = torch.nn.Sigmoid()

        # Collision risk diffusion
        k_range = int(np.ceil(self.obstacle_range / self.config.dl_2D))
        k = 2 * k_range + 1
        dist_kernel = np.zeros((k, k))
        for i, vv in enumerate(dist_kernel):
            for j, v in enumerate(vv):
                dist_kernel[i, j] = np.sqrt((i - k_range) ** 2 + (j - k_range) ** 2)
        dist_kernel = np.clip(1.0 - dist_kernel * self.config.dl_2D / self.obstacle_range, 0, 1) ** self.norm_p
        self.fixed_conv = torch.nn.Conv2d(1, 1, k, stride=1, padding=k_range, bias=False)
        self.fixed_conv.weight.requires_grad = False
        self.fixed_conv.weight *= 0
        self.fixed_conv.weight += torch.from_numpy(dist_kernel)
        self.fixed_conv.to(self.device)

        # Load the pretrained weigths
        if on_gpu and torch.cuda.is_available():
            checkpoint = torch.load(chkp_path, map_location=self.device)
        else:
            checkpoint = torch.load(chkp_path, map_location=torch.device('cpu'))
        self.net.load_state_dict(checkpoint['model_state_dict'])

        # Switch network from training to evaluation mode
        self.net.eval()

        print("\nModel and training state restored from " + chkp_path)
        print('Done in {:.1f}s\n'.format(time.time() - t1))

        ###############
        # ROS sub/pub #
        ###############
        
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to the lidar topic
        print('\nSubscribe to /velodyne_points')
        self.velo_frame_id = 'velodyne'
        self.velo_subscriber = self.create_subscription(PointCloud2,
                                                      '/velodyne_points',
                                                      self.lidar_callback,
                                                      10,
                                                      callback_group=self.callback_group)

        print('OK\n')

        # # Subsrcibe
        # self.tfBuffer = tf2_ros.Buffer(cache_time= CustomDuration(5.0, 0))
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer,
        #                                             self,
        #                                             spin_thread=True,
        #                                             qos=10,
        #                                             static_qos=10)

            
        # Subscribe to tf in the same call_back group as the lidar callback
        print('\nSubscribe to tf messages') 
        self.tfBuffer = tf2_ros.Buffer()                                       
        self.tf_sub = self.create_subscription(TFMessage,
                                          '/tf',
                                          self.tf_callback,
                                          10,
                                          callback_group=self.callback_group)
                                                    
        self.tf_static_sub = self.create_subscription(TFMessage,
                                          '/tf_static',
                                          self.tf_static_callback,
                                          10,
                                          callback_group=self.callback_group)

        print('OK\n')

        # Init collision publisher
        # self.collision_pub = rospy.Publisher('/plan_costmap_3D', VoxGrid, queue_size=10)
        # self.visu_pub = rospy.Publisher('/collision_visu', OccupancyGrid, queue_size=10)
        self.collision_pub = self.create_publisher(VoxGrid, '/plan_costmap_3D', 10)
        self.visu_pub = self.create_publisher(OccupancyGrid, '/collision_visu', 10)
        self.time_resolution = self.config.T_2D / self.config.n_2D_layers

        # Init obstacle publisher
        # self.obstacle_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=10)
        self.obstacle_pub = self.create_publisher(ObstacleArrayMsg, '/move_base/TebLocalPlannerROS/obstacles', 10)

        # Init point cloud publisher
        # self.pointcloud_pub = rospy.Publisher('/classified_points', PointCloud2, queue_size=10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/classified_points', 10)

        # # obtaining/defining the parameters for the output of the laserscan data
        # try:
        #     self.gmapping_status = rospy.get_param('gmapping_status')
        # except KeyError:
        #     self.gmapping_status = True
        
        # self.template_scan = LaserScan()

        # self.template_scan.angle_max = np.pi
        # self.template_scan.angle_min = -np.pi
        # self.template_scan.angle_increment = 0.01
        # self.template_scan.time_increment = 0.0
        # self.template_scan.scan_time = 0.01
        # self.template_scan.range_min = 0.0
        # self.template_scan.range_max = 30.0
        # self.min_height = 0.01
        # self.max_height = 1
        # self.ranges_size = int(np.ceil((self.template_scan.angle_max - self.template_scan.angle_min)/self.template_scan.angle_increment))

                                                                                    
        # self.pub_funcs = []
        # if (PUBLISH_POINTCLOUD):
        #     self.pub = rospy.Publisher('/classified_points', PointCloud2, queue_size=10)
        #     self.pub_funcs.append(self.publish_as_pointcloud)
        # if (PUBLISH_LASERSCAN):
        #     # scan for local planner (the first element of the tuple denotes the classes alloted to that scan)
        #     self.pub_list = [([0, 1, 2, 3, 4], rospy.Publisher('/local_planner_points2', LaserScan, queue_size=10))]
        #     self.pub_funcs.append(self.publish_as_laserscan)
        #     if (self.gmapping_status):
        #         self.pub_list.append(([0, 2, 3], rospy.Publisher('/gmapping_points2', LaserScan, queue_size=10)))  # scan for gmapping
        #     else:
        #         self.pub_list.append(([2], rospy.Publisher('/amcl_points2', LaserScan, queue_size=10)))  # scan for amcl localization
        #         self.pub_list.append(([0, 2, 3], rospy.Publisher('/global_planner_points2', LaserScan, queue_size=10)))  # scan for global planner

        self.last_t = time.time()
        self.last_t_tf = time.time()
        self.sec0, self.nsec0 = self.get_clock().now().seconds_nanoseconds()

        return

    def tf_callback(self, data):

        
        t1 = time.time()
        got_map = False
        who = 'default_authority'
        for transform in data.transforms:
            # print(transform.header.frame_id, transform.child_frame_id)
            self.tfBuffer.set_transform(transform, who)
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                got_map = True
        
        t2 = time.time()
        if got_map:
            print('--------- {:.1f} / {:.1f} --- new map pose OK'.format(1000*(t1 - self.last_t_tf), 1000*(t2 - t1)))
        else:
            print('--------- {:.1f} / {:.1f} --- new map pose NO'.format(1000*(t1 - self.last_t_tf), 1000*(t2 - t1)))

        

        self.last_t_tf = t2
        
        # TODO: DEBUG HEEEEEEEEEEEEEEEEREEEEEEEEEEEEEEE
        #       IDEA: print delay btween current time and latest lidar frame timestamp, we do not compute frame queue fast enough

    def tf_static_callback(self, data):
        who = 'default_authority'
        for transform in data.transforms:
            # print(transform.header.frame_id, transform.child_frame_id)
            self.tfBuffer.set_transform_static(transform, who)

        
    def lidar_callback(self, cloud):
        
        ########################
        # Update the frame poses
        ########################

        t1 = time.time()
        # print('\nGetting poses with lock')

        # Get the time stamps for all frames
        
        with self.online_dataset.worker_lock:
            for f_i, data in enumerate(self.online_dataset.frame_queue):

                if self.online_dataset.pose_queue[f_i] is None:

                    try:

                        # tsec1, tnsec1 = self.get_clock().now().seconds_nanoseconds()
                        # trans = self.tfBuffer.lookup_transform(
                        # 'map',
                        # 'velodyne',
                        # rclpy.time.Time())

                        # # print("#### tf listner")
                        # tsec2 = trans.header.stamp.sec
                        # tnsec2 = trans.header.stamp.nanosec
                        # timediff = tsec2 - tsec1 + int((tnsec2 - tnsec1) * 1e-6) * 1e-3

                        # self.get_logger().warn(f'Current time {tsec1}.{tnsec1}, got tf at {tsec2}.{tnsec2}, time difference {timediff}')
                        
                        pose = self.tfBuffer.lookup_transform('map', 'velodyne', data[0])
                        #pose = self.tfBuffer.lookup_transform('odom', 'base_link', data[0])

                        # sec1 = pose.header.stamp.sec
                        # nsec1 = pose.header.stamp.nanosec
                        # sec2, nsec2 = self.get_clock().now().seconds_nanoseconds()
                        # stamp1 = sec1 - self.sec0 + int((nsec1 - self.nsec0) * 1e-6) * 1e-3
                        # stamp2 = sec2 - self.sec0 + int((nsec2 - self.nsec0) * 1e-6) * 1e-3
                        # logstr = 'pose {:.3f} read at {:.3f}'.format(stamp1, stamp2)
                        # print('{:^35s}'.format(logstr), 35*' ', 35*' ')

                    except (tf2_ros.InvalidArgumentException, tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                        # self.get_logger().error("##### Not working")
                        # print("Iteration: {} error: {}".format(look_i, e))
                        # print(data[0])
                        pose = None
                        pass

                    self.online_dataset.pose_queue[f_i] = pose

        t2 = time.time()
        # print('Done in {:.1f}ms'.format(1000* (t2 - t1)))
        # print('\nGetting new frame')

        ###############
        # Add new frame
        ###############

        # t1 = time.time()
        # print('Starting lidar_callback after {:.1f}'.format(1000*(t1 - self.last_t)))
        self.velo_frame_id = cloud.header.frame_id

        # Check if we already know this frame
        if len(self.online_dataset.frame_queue) > 0:
            #print(cloud.header.stamp, self.online_dataset.frame_queue[-1][0])
            if (cloud.header.stamp == self.online_dataset.frame_queue[-1][0]):
                self.get_logger().warn('Same timestamp, pass')
                return


        # convert PointCloud2 message to structured numpy array
        labeled_points = pointcloud2_to_array(cloud)

        # convert numpy array to Nx3 sized numpy array of float32
        xyz_points = get_xyz_points(labeled_points, remove_nans=True, dtype=np.float32)

        # Safe check
        if xyz_points.shape[0] < 100:
            print('{:^35s}'.format('CPU 0 : Corrupted frame not added'), 35*' ', 35*' ')
            return

        # # Convert to torch tensor and share memory
        # xyz_tensor = torch.from_numpy(xyz_points)
        # xyz_tensor.share_memory_()

        # # Convert header info to shared tensor
        # frame_t = torch.from_numpy(np.array([cloud.header.stamp.to_sec()], dtype=np.float64))
        # frame_t.share_memory_()

        # print('/-------------\\')
        # print([data[1].shape[0] for f_i, data in enumerate(self.online_dataset.frame_queue)])
        # print([pp is not None for pp in self.online_dataset.pose_queue])
        # print('\\-------------/')

        t3 = time.time()
        #print('Done in {:.1f}ms'.format(1000* (t3 - t2)))
        #print('\n frame FIFO')
        
        pose_fifo = [pp is not None for pp in self.online_dataset.pose_queue]

        logstr = ''
        for pp_bool in pose_fifo:
            logstr += ' '
            if pp_bool:
                logstr += bcolors.OKGREEN + 'X' + bcolors.ENDC
            else:
                logstr += bcolors.FAIL + '_' + bcolors.ENDC
        logstr += ' '
        print('{:^35s}'.format(logstr), 35*' ', 35*' ')
            
        # Update the frame list
        with self.online_dataset.worker_lock:

            if len(self.online_dataset.frame_queue) >= (10 + self.config.n_frames):
                self.online_dataset.frame_queue.pop(0)
                self.online_dataset.pose_queue.pop(0)
            self.online_dataset.frame_queue.append((cloud.header.stamp, xyz_points))
            self.online_dataset.pose_queue.append(None)
        
        # logstr = 'CPU 0 : New frame {:d}'.format(xyz_points.shape[0])
        # print('{:^35s}'.format(logstr), 35*' ', 35*' ')

        # t2 = time.time()
        # print('Finished lidar_callback in {:.1f}'.format(1000*(t2 - t1)))
        # self.last_t = time.time()

        # self.get_logger().warn("##############Ended one Lidar callback")
        # print("")

        t4 = time.time()
        #print('Done in {:.1f}ms'.format(1000* (t4 - t3)))

        return

    def inference_loop(self):

        # No gradient computation here
        with torch.no_grad():

            # When starting this for loop, one thread will be spawned and creating network
            # input while the loop is performing GPU operations.
            for i, batch in enumerate(self.online_loader):

                #####################
                # Input preparation #
                #####################

                t = [time.time()]

                # # Check that ros master is still up
                # try:
                #     topics = rospy.get_published_topics()
                # except ConnectionRefusedError as e:
                #     print('Lost connection to master. Terminate collider')
                #     break

                # Check if batch is a dummy
                if len(batch.points) < 1:
                    print(35*' ', 35*' ', '{:^35s}'.format('GPU : Corrupted batch skipped'))
                    time.sleep(0.5)
                    continue
                
                print(35*' ', 35*' ', '{:^35s}'.format('GPU : Got batch, start inference'))

                # Convert batch to a cuda tensors
                if 'cuda' in self.device.type:
                    batch.to(self.device)
                torch.cuda.synchronize(self.device)

                t += [time.time()]
                    
                #####################
                # Network inference #
                #####################

                # Forward pass
                outputs_3D, preds_init, preds_future = self.net(batch, self.config)
                torch.cuda.synchronize(self.device)
                
                t += [time.time()]

                ###########
                # Outputs #
                ###########

                # Get collision predictions [1, T, W, H, 3] -> [T, W, H, 3]
                collision_preds = self.sigmoid_2D(preds_future)[0]


                # Get the diffused risk
                diffused_risk, obst_pos = self.get_diffused_risk(collision_preds)

                # Convert stamp to float
                sec1 = batch.t0.sec
                nsec1 = batch.t0.nanosec
                stamp0 = float(sec1) + float(int((nsec1) * 1e-6)) * 1e-3

                # ##########################################################################################################
                # Save predictions in files for debug

                # stamp1 = sec1 - self.sec0 + int((nsec1 - self.nsec0) * 1e-6) * 1e-3
                # im_name = join(ENV_HOME, 'results/debug_1_{:.3f}.gif'.format(stamp1))
                # debug_preds = collision_preds.cpu().detach().numpy()
                # print('\n\n', debug_preds.dtype, debug_preds.shape, '\n\n')

                # # imageio.imwrite(im_name, zoom_collisions(debug_preds, 5))
                # fast_save_future_anim(im_name, debug_preds, zoom=5, correction=True)

                # im_name2 = join(ENV_HOME, 'results/debug_2_{:.3f}.gif'.format(stamp1))
                # cm = plt.get_cmap('viridis')
                # debug_preds2 = cm(diffused_risk)
                # print('\n\n', debug_preds2.dtype, debug_preds2.shape, '\n\n')
                
                # # imageio.imwrite(im_name2, zoom_collisions(debug_preds2, 5))
                # fast_save_future_anim(im_name2, debug_preds2, zoom=5, correction=False)
                # ############################################################################################################

                # Get obstacles in world coordinates
                origin0 = batch.p0 - self.config.in_radius / np.sqrt(2)

                world_obst = []
                for obst_i, pos in enumerate(obst_pos):
                    world_obst.append(origin0[:2] + pos * self.config.dl_2D)

                # Publish collision risk in a custom message
                # self.publish_collisions(diffused_risk, stamp0, batch.p0, batch.q0)
                self.publish_collisions_visu(diffused_risk, batch.t0, batch.p0, batch.q0, visu_T=15)

                # Publish obstacles
                #self.publish_obstacles(world_obst, batch.t0, batch.p0, batch.q0)

                ##############
                # Outputs 3D #
                ##############

                # Get predictions
                predicted_probs = self.softmax(outputs_3D).cpu().detach().numpy()
                for l_ind, label_value in enumerate(self.online_dataset.label_values):
                    if label_value in self.online_dataset.ignored_labels:
                        predicted_probs = np.insert(predicted_probs, l_ind, 0, axis=1)
                predictions = self.online_dataset.label_values[np.argmax(predicted_probs, axis=1)].astype(np.int32)

                # Get frame points re-aligned in the velodyne coordinates
                pred_points = batch.points[0].cpu().detach().numpy() + batch.p0

                R0 = scipyR.from_quat(batch.q0).as_matrix()
                pred_points = np.dot(pred_points - batch.p0, R0)
                
                # ############################################################################################################
                # # DEBUG: Save input frames
                # plyname = join(ENV_HOME, 'results/ptpreds_{:.3f}.gif'.format(stamp1))
                # write_ply(plyname,
                #           [pred_points.astype(np.float32), predictions],
                #           ['x', 'y', 'z', 'classif'])
                # ############################################################################################################

                # Publish pointcloud
                self.publish_pointcloud(pred_points, predictions, batch.t0)

                # Fake slowing pause
                #time.sleep(2.5)
                
                t += [time.time()]

                print(35 * ' ', 35 * ' ', '{:^35s}'.format('GPU : Inference Done in {:.0f} + {:.0f} + {:.0f} ms'.format(1000 * (t[1] - t[0]),
                                                                                                                        1000 * (t[2] - t[1]),
                                                                                                                        1000 * (t[3] - t[2]))))


        return

    def get_diffused_risk(self, collision_preds):
                                    
        # # Remove residual preds (hard hysteresis)
        # collision_risk *= (collision_risk > 0.06).type(collision_risk.dtype)
                    
        # Remove residual preds (soft hysteresis)
        lim1 = 0.06
        lim2 = 0.09
        dlim = lim2 - lim1
        mask0 = collision_preds <= lim1
        mask1 = torch.logical_and(collision_preds < lim2, collision_preds > lim1)
        collision_preds[mask0] *= 0
        collision_preds[mask1] *= (1 - ((collision_preds[mask1] - lim2) / dlim) ** 2) ** 2

        # Get risk from static objects, [1, 1, W, H]
        static_preds = torch.unsqueeze(torch.max(collision_preds[:1, :, :, :2], dim=-1)[0], 1)
        #static_preds = (static_risk > 0.3).type(collision_preds.dtype)

        # Normalize risk values between 0 and 1 depending on density
        static_risk = static_preds / (self.fixed_conv(static_preds) + 1e-6)

        # Diffuse the risk from normalized static objects
        diffused_0 = self.fixed_conv(static_risk).cpu().detach().numpy()

        # Repeat for all the future steps [1, 1, W, H] -> [T, W, H]
        diffused_0 = np.squeeze(np.tile(diffused_0, (collision_preds.shape[0], 1, 1, 1)))

        # Diffuse the risk from moving obstacles , [T, 1, W, H] -> [T, W, H]
        moving_risk = torch.unsqueeze(collision_preds[..., 2], 1)
        diffused_1 = np.squeeze(self.fixed_conv(moving_risk).cpu().detach().numpy())
        
        # Inverse power for p-norm
        diffused_0 = np.power(np.maximum(0, diffused_0), self.norm_invp)
        diffused_1 = np.power(np.maximum(0, diffused_1), self.norm_invp)

        # Merge the two risk after rescaling
        diffused_0 *= 1.0 / (np.max(diffused_0) + 1e-6)
        diffused_1 *= 1.0 / (np.max(diffused_1) + 1e-6)
        diffused_risk = np.maximum(diffused_0, diffused_1)

        # Convert to uint8 for message 0-254 = prob, 255 = fixed obstacle
        diffused_risk = np.minimum(diffused_risk * 255, 255).astype(np.uint8)
        
        # # Save walls for debug
        # debug_walls = np.minimum(diffused_risk[10] * 255, 255).astype(np.uint8)
        # cm = plt.get_cmap('viridis')
        # print(batch.t0)
        # print(type(batch.t0))
        # im_name = join(ENV_HOME, 'catkin_ws/src/collision_trainer/results/debug_walls_{:.3f}.png'.format(batch.t0))
        # imageio.imwrite(im_name, zoom_collisions(cm(debug_walls), 5))

        # Get local maxima in moving obstacles
        obst_mask = self.get_local_maxima(diffused_1[15])

        # Create obstacles in walls (one cell over 2 to have arround 1 obstacle every 25 cm)
        static_mask = np.squeeze(static_preds.cpu().detach().numpy() > 0.3)
        static_mask[::2, :] = 0
        static_mask[:, ::2] = 0

        # Merge obstacles
        obst_mask[static_mask] = 1

        # Convert to pixel positions
        obst_pos = self.mask_to_pix(obst_mask)

        return diffused_risk, obst_pos

    def get_local_maxima(self, data, neighborhood_size=5, threshold=0.1):
        
        # Get maxima positions as a mask
        data_max = filters.maximum_filter(data, neighborhood_size)
        max_mask = (data == data_max)

        # Remove maxima if their peak is not higher than threshold in the neighborhood
        data_min = filters.minimum_filter(data, neighborhood_size)
        diff = ((data_max - data_min) > threshold)
        max_mask[diff == 0] = 0

        return max_mask

    def mask_to_pix(self, mask):
        
        # Get positions in world coordinates
        labeled, num_objects = ndimage.label(mask)
        slices = ndimage.find_objects(labeled)
        x, y = [], []

        mask_pos = []
        for dy, dx in slices:

            x_center = (dx.start + dx.stop - 1) / 2
            y_center = (dy.start + dy.stop - 1) / 2
            mask_pos.append(np.array([x_center, y_center], dtype=np.float32))

        return mask_pos

    def publish_collisions(self, collision_preds, stamp0, p0, q0):
        
        #self.get_logger().warn("Started Publisher COLLISION callback")

        # Get origin and orientation
        origin0 = p0 - self.config.in_radius / np.sqrt(2)

        # Define header
        msg = VoxGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Define message
        msg.depth = collision_preds.shape[0]
        msg.width = collision_preds.shape[1]
        msg.height = collision_preds.shape[2]
        msg.dl = self.config.dl_2D
        msg.dt = self.time_resolution
        msg.origin.x = origin0[0]
        msg.origin.y = origin0[1]
        msg.origin.z = stamp0  # This is already the converted float value (message type is float64)

        #msg.theta = q0[0]
        msg.theta = 0.0

        msg.data = collision_preds.ravel().tolist()


        # Publish
        self.collision_pub.publish(msg)

        return

    def publish_collisions_visu(self, collision_preds, t0, p0, q0, visu_T=15):
        '''
        0 = invisible
        1 -> 98 = blue to red
        99 = cyan
        100 = yellow
        101 -> 127 = green
        128 -> 254 = red to yellow
        255 = vert/gris
        '''

        # Get origin and orientation
        origin0 = p0 - self.config.in_radius / np.sqrt(2)

        # Define header
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'


        # Define message meta data
        msg.info.map_load_time = rclTime(seconds=t0.sec, nanoseconds=t0.nanosec).to_msg()
        msg.info.resolution = self.config.dl_2D
        msg.info.width = collision_preds.shape[1]
        msg.info.height = collision_preds.shape[2]
        msg.info.origin.position.x = origin0[0]
        msg.info.origin.position.y = origin0[1]
        msg.info.origin.position.z = -0.01
        #msg.info.origin.orientation.x = q0[0]
        #msg.info.origin.orientation.y = q0[1]
        #msg.info.origin.orientation.z = q0[2]
        #msg.info.origin.orientation.w = q0[3]


        # Define message data
        data_array = collision_preds[visu_T, :, :].astype(np.float32)
        mask = collision_preds[visu_T, :, :] > 253
        mask2 = np.logical_not(mask)
        data_array[mask2] = data_array[mask2] * 98 / 253
        data_array[mask2] = np.maximum(1, np.minimum(98, data_array[mask2] * 1.0))
        data_array[mask] = 98  # 101
        data_array = data_array.astype(np.int8)
        msg.data = data_array.ravel().tolist()

        # Publish
        self.visu_pub.publish(msg)

        return

    def publish_obstacles(self, obstacle_list):


        msg = ObstacleArrayMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Add point obstacles
        for obst_i, pos in enumerate(obstacle_list):

            obstacle_msg = ObstacleMsg()
            obstacle_msg.id = obst_i
            obstacle_msg.polygon.points = [Point32(x=pos[0], y=pos[1], z=0)]

            # obstacle_msg.polygon.points[0].x = 1.5
            # obstacle_msg.polygon.points[0].y = 0
            # obstacle_msg.polygon.points[0].z = 0

            msg.obstacles.append(obstacle_msg)

        self.obstacle_pub.publish(msg)

        return

    def publish_pointcloud(self, new_points, predictions, t0):

        # data structure of binary blob output for PointCloud2 data type
        output_dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring'],
                                 'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
                                 'offsets': [0, 4, 8, 16, 20],
                                 'itemsize': 32})

        # fill structured numpy array with points and classes (in the intensity field). Fill ring with zeros to maintain Pointcloud2 structure
        c_points = np.c_[new_points, predictions, np.zeros(len(predictions))]
        c_points = np.core.records.fromarrays(c_points.transpose(), output_dtype)

        # convert to Pointcloud2 message and publish
        msg = array_to_pointcloud2(c_points,
                                   rclTime(seconds=t0.sec, nanoseconds=t0.nanosec).to_msg(),
                                   self.velo_frame_id)

        self.pointcloud_pub.publish(msg)

# ----------------------------------------------------------------------------------------------------------------------
#
#           Main Call
#       \***************/
#

def main(args=None):

    # Parameters
    log_name = 'Log_2021-05-24_13-59-19'
    chkp_name = 'chkp_0380.tar'
    training_path = join(ENV_HOME, 'results/pretrained_logs/', log_name)

    # Setup the collider Class
    print('\n\n\n\n        ------ Init Collider ------')
    rclpy.init(args=args)
    tester = OnlineCollider(training_path, chkp_name)


    # Spin in a separate thread
    executor = SingleThreadedExecutor()
    def run_func():
        executor.add_node(tester)
        executor.spin()
        executor.remove_node(tester)
    dedicated_listener_thread = threading.Thread(target=run_func)
    dedicated_listener_thread.start()
    print('OK')

    # # Spin in a separate thread
    # thread = threading.Thread(target=rclpy.spin, args=(tester, ), daemon=True)
    # thread.start()
    # print('OK')


    try:
        # Start network process
        print('Start inference loop')
        tester.inference_loop()
        print('OK')
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    dedicated_listener_thread.join()



if __name__ == '__main__':
    main()
