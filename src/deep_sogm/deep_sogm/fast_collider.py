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
from multiprocessing import Lock
from numpy.core.numeric import False_
ENV_USER = os.getenv('USER')
ENV_HOME = os.getenv('HOME')
# ENV_USER = 'asrl'
# ENV_HOME = '/home/asrl'
# sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm"))
# sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm/utils"))
# sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm/models"))
# sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm/kernels"))
# sys.path.insert(0, join(ENV_HOME, "eloquent_ws/src/deep_sogm/deep_sogm/cpp_wrappers"))

print('\n\n', ENV_HOME, '\n\n', ENV_USER, '\n\n')

ROBOT_ROOT = join(ENV_HOME, 'eloquent_ws/src/deep_sogm/deep_sogm')
SIMU_ROOT = join(ENV_HOME, 'Deep-Collison-Checker/Myhal_Simulator/onboard_deep_sogm/src/deep_sogm/deep_sogm')
for ROOT_DIR in [ROBOT_ROOT, SIMU_ROOT]:
    sys.path.insert(0, ROOT_DIR)
    sys.path.insert(0, join(ROOT_DIR, "utils"))
    sys.path.insert(0, join(ROOT_DIR, "models"))
    sys.path.insert(0, join(ROOT_DIR, "kernels"))
    sys.path.insert(0, join(ROOT_DIR, "cpp_wrappers"))


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
from utils.pc2_numpy import array_to_pointcloud2_fast, pointcloud2_to_array, get_xyz_points, array_to_pointcloud2


import rclpy
from rclpy.node import Node
from rclpy.time import Time as rclTime

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PolygonStamped, Point32
from rosgraph_msgs.msg import Clock

from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from vox_msgs.msg import VoxGrid

import tf2_ros
from tf2_msgs.msg import TFMessage

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.duration import Duration
# from rclpy.time import Time


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

    def __init__(self):
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

        self.static_range = 0.7
        self.dynamic_range = 1.2  # TODO 1.2
        self.dynamic_t_range = 1.0  # TODO 1.0
        self.norm_p = 3  # TODO 3
        self.norm_invp = 1 / self.norm_p

        # self.maxima_layers = [18, 38]
        self.maxima_layers = [15]

        self.visu_T = 29

        self.declare_parameter('nav_without_sogm', False)
        self.declare_parameter('model_path', '')
        
        self.nav_without_sogm = self.get_parameter('nav_without_sogm').get_parameter_value().bool_value
        self.simu = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value

        #rclpy.init(args=sys.argv)
        #self.node = rclpy.create_node('fast_collider')
        #self.node.get_logger().info('Created node: fast_collider')

        ######################
        # Load trained model #
        ######################

        print('\nModel Preparation')
        print('*****************')
        t1 = time.time()

        # Get training path from session
        training_path = '/'.join(self.model_path.split('/')[:-2])

        print(self.model_path)
        print(training_path)

        # Load configuration class used at training
        self.config = Config()
        self.config.load(training_path)

        # Init data class
        self.queue_length = 5
        self.online_dataset = OnlineDataset(self.config, self.queue_length, self.simu)
        self.online_sampler = OnlineSampler(self.online_dataset)
        self.online_loader = DataLoader(self.online_dataset,
                                        batch_size=1,
                                        sampler=self.online_sampler,
                                        collate_fn=MyhalCollisionCollate,
                                        num_workers=0,
                                        pin_memory=True)

        # Define network model
        self.net = KPCollider(self.config, self.online_dataset.label_values, self.online_dataset.ignored_labels)
        self.net.to(self.device)
        self.softmax = torch.nn.Softmax(1)
        self.sigmoid_2D = torch.nn.Sigmoid()

        # Convolution for Collision risk diffusion
        self.static_conv = self.diffusing_convolution(self.static_range, self.config.dl_2D, self.norm_p)
        self.static_conv.to(self.device)
        self.dynamic_conv = self.diffusing_convolution(self.dynamic_range, self.config.dl_2D, self.norm_p)
        self.dynamic_conv.to(self.device)

        # Convolution for time diffusion
        self.dt = self.config.T_2D / self.config.n_2D_layers
        self.time_conv = self.diffusing_convolution(self.dynamic_t_range, self.dt, self.norm_p, dim1D=True)
        self.time_conv.to(self.device)

        # Load the pretrained weigths
        if on_gpu and torch.cuda.is_available():
            checkpoint = torch.load(self.model_path, map_location=self.device)
        else:
            checkpoint = torch.load(self.model_path, map_location=torch.device('cpu'))

        self.net.load_state_dict(checkpoint['model_state_dict'])

        # Switch network from training to evaluation mode
        self.net.eval()

        print("\nModel and training state restored from " + self.model_path)
        print('Done in {:.1f}s\n'.format(time.time() - t1))

        ###############
        # ROS sub/pub #
        ###############

        self.callback_group1 = MutuallyExclusiveCallbackGroup()
        self.callback_group2 = MutuallyExclusiveCallbackGroup()
        self.callback_group3 = MutuallyExclusiveCallbackGroup()

        # Subscribe to the lidar topic
        print('\nSubscribe to /sub_points')
        self.map_frame_id = 'map'
        self.velo_subscriber = self.create_subscription(PointCloud2,
                                                        '/sub_points',
                                                        self.lidar_callback,
                                                        10,
                                                        callback_group=self.callback_group1)

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
                                               callback_group=self.callback_group2)

        self.tf_static_sub = self.create_subscription(TFMessage,
                                                      '/tf_static',
                                                      self.tf_static_callback,
                                                      10,
                                                      callback_group=self.callback_group2)

        print('OK\n')

        # Subscribe to clock topic for simulated runs
        if self.simu:
            self.clock_subscriber = self.create_subscription(Clock,
                                                             '/clock',
                                                             self.clock_callback,
                                                             10,
                                                             callback_group=self.callback_group3)

        if self.nav_without_sogm:
            # Init dummy publishers
            self.collision_pub = self.create_publisher(VoxGrid, '/dummy_plan_costmap_3D', 10)
            self.obstacle_pub = self.create_publisher(ObstacleArrayMsg, '/dummy_obstacles', 10)

        else:
            # Init collision and obstacle publishers
            self.collision_pub = self.create_publisher(VoxGrid, '/plan_costmap_3D', 10)
            self.obstacle_pub = self.create_publisher(ObstacleArrayMsg, '/move_base/TebLocalPlannerROS/obstacles', 10)

        # Init other visu  publisher
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/classified_points', 10)
        self.visu_pub = self.create_publisher(OccupancyGrid, '/dynamic_visu', 10)
        self.visu_pub_static = self.create_publisher(OccupancyGrid, '/static_visu', 10)

        self.time_resolution = self.config.T_2D / self.config.n_2D_layers

        self.last_t = time.time()
        self.last_t_tf = time.time()
        self.sec0, self.nsec0 = self.get_clock().now().seconds_nanoseconds()

        # Lock for visu publisher
        self.visu_lock = Lock()
        self.msg_static = None
        self.msg_dyn = None
        self.current_SRM = None
        self.visu_mode = "v2"
        self.current_T = 0
        self.last_layer_i = -1

        return

    def diffusing_convolution(self, obstacle_range, dl, norm_p, dim1D=False):
        
        k_range = int(np.ceil(obstacle_range / dl))
        k = 2 * k_range + 1

        if dim1D:
            dist_kernel = np.zeros((k, 1, 1))
            for i in range(k):
                dist_kernel[i, 0, 0] = abs(i - k_range)

        else:
            dist_kernel = np.zeros((k, k))
            for i, vv in enumerate(dist_kernel):
                for j, v in enumerate(vv):
                    dist_kernel[i, j] = np.sqrt((i - k_range) ** 2 + (j - k_range) ** 2)


        dist_kernel = np.clip(1.0 - dist_kernel * (dl / obstacle_range), 0, 1) ** norm_p

        if dim1D:
            fixed_conv = torch.nn.Conv3d(1, 1, (k, 1, 1),
                                         stride=1,
                                         padding=(k_range, 0, 0),
                                         padding_mode='replicate',
                                         bias=False)

        else:
            fixed_conv = torch.nn.Conv2d(1, 1, k, stride=1, padding=k_range, bias=False)

        fixed_conv.weight.requires_grad = False
        fixed_conv.weight *= 0

        fixed_conv.weight += torch.from_numpy(dist_kernel)

        return fixed_conv

    def tf_callback(self, data):

        t1 = time.time()
        difft = 1000*(t1 - self.last_t_tf)
        if not self.simu and difft > 200:
            print(bcolors.WARNING + '{:.1f} --- TF DELAY'.format(difft) + bcolors.ENDC)
        # else:
        #     print('{:.1f} --- OK'.format(difft))

        # Parameters
        got_map = False
        who = 'default_authority'

        # Add new message info to the tf buffer
        for transform in data.transforms:
            # print(transform.header.frame_id, transform.child_frame_id)
            self.tfBuffer.set_transform(transform, who)
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                got_map = True

        # # Just got a new map pose, update fifo
        # if got_map:
        #     self.online_dataset.shared_fifo.update_poses(self.tfBuffer)

        self.last_t_tf = time.time()

        return

    def tf_static_callback(self, data):
        who = 'default_authority'
        for transform in data.transforms:
            # print(transform.header.frame_id, transform.child_frame_id)
            self.tfBuffer.set_transform_static(transform, who)

        return

    def clock_callback(self, data):

        # We do not need the clock data but we use this functio nas publication

        verif = False
        if verif:
            print(data)
            
        self.publish_collisions_visu(None, None, None, None)

        return

    def lidar_callback(self, cloud):

        ###############
        # Add new frame
        ###############
        

        t1 = time.time()
        difft = 1000*(t1 - self.last_t)
        if not self.simu and difft > 200:
            print(bcolors.WARNING + '{:.1f} --- VELO DELAY'.format(difft) + bcolors.ENDC)
            

        # self.velo_frame_id = cloud.header.frame_id

        # # Check if we already know this frame
        # if self.online_dataset.shared_fifo.len() > 0:
        #     #print(cloud.header.stamp, self.online_dataset.stamp_queue[-1])
        #     if (cloud.header.stamp == self.online_dataset.stamp_queue[-1]):
        #         self.get_logger().warn('Same timestamp, pass')
        #         return

        # convert PointCloud2 message to structured numpy array
        frame_data = pointcloud2_to_array(cloud)
        
        # Remove nan values
        mask = np.isfinite(frame_data['x']) & \
            np.isfinite(frame_data['y']) & \
            np.isfinite(frame_data['z'])
        frame_data = frame_data[mask]

        # Safe check
        if frame_data.shape[0] < 100:
            print('{:^35s}'.format('CPU 0 : Corrupted frame not added'), 35*' ', 35*' ')
            return

        #################
        # Update the fifo
        #################
        
        # Update the frame list
        self.online_dataset.shared_fifo.add_points(frame_data, cloud.header.stamp)

        # We should have just published the pose of this frame
        self.online_dataset.shared_fifo.update_poses(self.tfBuffer)

        # Display queue (with current frame delay)
        logstr = self.online_dataset.shared_fifo.to_str()
        sec1, nsec1 = self.get_clock().now().seconds_nanoseconds()
        sec2 = cloud.header.stamp.sec
        nsec2 = cloud.header.stamp.nanosec

        timediff = (sec1 - sec2) * 1e3 + (nsec1 - nsec2) * 1e-6
        logstr += '  {:6.1f}ms '.format(-timediff)
        print('{:^35s}'.format(logstr), 35*' ', 35*' ')

        self.last_t = time.time()

        return

    def get_diffused_risk(self, collision_preds):
                                    
        # # Remove residual preds (hard hysteresis)
        # collision_risk *= (collision_risk > 0.06).type(collision_risk.dtype)
                    
        # Remove residual preds (soft hysteresis)
        # lim1 = 0.06
        # lim2 = 0.09
        lim1 = 0.15
        lim2 = 0.2
        dlim = lim2 - lim1
        mask0 = collision_preds <= lim1
        mask1 = torch.logical_and(collision_preds < lim2, collision_preds > lim1)
        collision_preds[mask0] *= 0
        collision_preds[mask1] *= (1 - ((collision_preds[mask1] - lim2) / dlim) ** 2) ** 2

        # Static risk
        # ***********

        # Get risk from static objects, [1, 1, W, H]
        static_preds = torch.unsqueeze(torch.max(collision_preds[:1, :, :, :2], dim=-1)[0], 1)

        # Normalize risk values between 0 and 1 depending on density
        static_risk = static_preds / (self.static_conv(static_preds) + 1e-6)

        # Diffuse the risk from normalized static objects
        diffused_0 = self.static_conv(static_risk).cpu().detach().numpy()

        # Do not repeat we only keep it for the first layer: [1, 1, W, H] -> [W, H]
        diffused_0 = np.squeeze(diffused_0)
        
        # Inverse power for p-norm
        diffused_0 = np.power(np.maximum(0, diffused_0), self.norm_invp)

        # Dynamic risk
        # ************

        # Get dynamic risk [T, W, H]
        dynamic_risk = collision_preds[..., 2]

        # Get high risk area
        high_risk_threshold = 0.4
        high_risk_mask = dynamic_risk > high_risk_threshold
        high_risk = torch.zeros_like(dynamic_risk)
        # high_risk[high_risk_mask] = dynamic_risk[high_risk_mask]
        high_risk[high_risk_mask] = 1

        # On the whole dynamic_risk, convolution
        # Higher value for larger area of risk even if low risk
        dynamic_risk = torch.unsqueeze(dynamic_risk, 1)
        diffused_1 = torch.squeeze(self.dynamic_conv(dynamic_risk))

        # Inverse power for p-norm
        diffused_1 = torch.pow(torch.clamp(diffused_1, min=0), self.norm_invp)

        # Rescale this low_risk at smaller value
        low_risk_value = 0.4
        diffused_1 = low_risk_value * diffused_1 / (torch.max(diffused_1) + 1e-6)

        # On the high risk, we normalize to have similar value of highest risk (around 1.0)
        high_risk_norm = torch.squeeze(self.dynamic_conv(torch.unsqueeze(high_risk, 1)))
        high_risk_norm = torch.unsqueeze(torch.unsqueeze(high_risk_norm, 0), 0)
        high_risk_norm = torch.squeeze(self.time_conv(high_risk_norm))
        high_risk_normalized = high_risk / (high_risk_norm + 1e-6)

        # We only diffuse time for high risk (as this is useful for the beginning of predictions)
        diffused_2 = torch.squeeze(self.dynamic_conv(torch.unsqueeze(high_risk_normalized, 1)))
        diffused_2 = torch.unsqueeze(torch.unsqueeze(diffused_2, 0), 0)
        diffused_2 = torch.squeeze(self.time_conv(diffused_2))

        # Inverse power for p-norm
        diffused_2 = torch.pow(torch.clamp(diffused_2, min=0), self.norm_invp)

        # Rescale and combine risk
        # ************************
        
        # Combine dynamic risks
        diffused_1 = torch.maximum(diffused_1, diffused_2).detach().cpu().numpy()

        # Rescale risk with a fixed value, because thx to normalization, the mx should be close to one, 
        # There are peak at border so we divide by 1.1 to take it into consideration
        diffused_1 *= 1.0 / 1.1
        diffused_0 *= 1.0 / 1.1

        # merge the static risk as the first layer of the vox grid (with the delay this layer is useless for dynamic)
        diffused_1[0, :, :] = diffused_0

        # Convert to uint8 for message 0-254 = prob, 255 = fixed obstacle
        diffused_risk = np.minimum(diffused_1 * 255, 255).astype(np.uint8)
        
        # # Save walls for debug
        # debug_walls = np.minimum(diffused_risk[10] * 255, 255).astype(np.uint8)
        # cm = plt.get_cmap('viridis')
        # print(batch.t0)
        # print(type(batch.t0))
        # im_name = join(ENV_HOME, 'catkin_ws/src/collision_trainer/results/debug_walls_{:.3f}.png'.format(batch.t0))
        # imageio.imwrite(im_name, zoom_collisions(cm(debug_walls), 5))

        # Get local maxima in moving obstacles
        obst_mask = None
        for layer_i in self.maxima_layers:
            if obst_mask is None:
                obst_mask = self.get_local_maxima(diffused_1[layer_i])
            else:
                obst_mask = np.logical_or(obst_mask, self.get_local_maxima(diffused_1[layer_i]))

        # Use max pool to get obstacles in one cell over two [H, W] => [H//2, W//2]
        stride = 2
        pool = torch.nn.MaxPool2d(stride, stride=stride, return_indices=True)
        unpool = torch.nn.MaxUnpool2d(stride, stride=stride)
        output, indices = pool(static_preds.detach())
        static_preds_2 = unpool(output, indices, output_size=static_preds.shape)

        # Merge obstacles
        obst_mask[np.squeeze(static_preds_2.cpu().numpy()) > 0.3] = 1

        # Convert to pixel positions
        obst_pos = self.mask_to_pix(obst_mask)

        # Get mask of static obstacles for visu
        static_mask = np.squeeze(static_preds.detach().cpu().numpy()) > 0.3
        
        return diffused_risk, obst_pos, static_mask

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

        # Get origin and orientation
        origin0 = p0 - self.config.radius_2D / np.sqrt(2)

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

    def publish_collisions_visu(self, collision_preds, static_mask, t0, p0):
        '''
        0 = invisible
        1 -> 98 = blue to red
        99 = cyan
        100 = yellow
        101 -> 127 = green
        128 -> 254 = red to yellow
        255 = vert/gris
        '''
        update_data = collision_preds is not None

        if update_data:

            # Get origin and orientation
            origin0 = p0 - self.config.radius_2D / np.sqrt(2)

            # Define header
            msg_dyn = OccupancyGrid()
            msg_dyn.header.stamp = self.get_clock().now().to_msg()
            msg_dyn.header.frame_id = 'map'
            msg_static = OccupancyGrid()
            msg_static.header.stamp = self.get_clock().now().to_msg()
            msg_static.header.frame_id = 'map'

            # Define message meta data
            msg_dyn.info.map_load_time = rclTime(seconds=t0.sec, nanoseconds=t0.nanosec).to_msg()
            msg_dyn.info.resolution = self.config.dl_2D
            msg_dyn.info.width = collision_preds.shape[1]
            msg_dyn.info.height = collision_preds.shape[2]
            msg_dyn.info.origin.position.x = origin0[0]
            msg_dyn.info.origin.position.y = origin0[1]
            msg_dyn.info.origin.position.z = 0.011
            #msg_dyn.info.origin.orientation.x = q0[0]
            #msg_dyn.info.origin.orientation.y = q0[1]
            #msg_dyn.info.origin.orientation.z = q0[2]
            #msg_dyn.info.origin.orientation.w = q0[3]

            msg_static.info.map_load_time = rclTime(seconds=t0.sec, nanoseconds=t0.nanosec).to_msg()
            msg_static.info.resolution = self.config.dl_2D
            msg_static.info.width = collision_preds.shape[1]
            msg_static.info.height = collision_preds.shape[2]
            msg_static.info.origin.position.x = origin0[0]
            msg_static.info.origin.position.y = origin0[1]
            msg_static.info.origin.position.z = 0.01


            # Define message data
            #   > static risk: yellow to red
            #   > dynamic risk: blue to red
            #   > invisible for the rest of the map
            # Actually separate them in two different costmaps

            visu_SRM = np.zeros((1, 1))
            if self.visu_mode == "v1":
                visu_SRM = collision_preds[1:, :, :].astype(np.float32)
                visu_SRM *= 1 / 255
                visu_SRM *= 126
                visu_SRM = np.maximum(0, np.minimum(126, visu_SRM.astype(np.int8)))
                mask = visu_SRM > 0
                visu_SRM[mask] += 128

            elif self.visu_mode == "v2":
                for iso_i, iso in enumerate([220, 150]):

                    dynamic_mask = collision_preds[1:, :, :] > iso
                    dynamic_data = dynamic_mask.astype(np.float32) * np.expand_dims(np.arange(dynamic_mask.shape[0]), (1, 2))
                    dynamic_data = np.max(dynamic_data, axis=0)
                    if iso_i > 0:
                        erode_mask = dynamic_data > 0
                        close_struct = np.ones((5, 5))
                        erode_struct = np.ones((3, 3))
                        erode_mask = ndimage.binary_closing(erode_mask, structure=close_struct, iterations=2)
                        erode_mask = ndimage.binary_erosion(erode_mask, structure=erode_struct)
                        dynamic_data[erode_mask] = 0
                    visu_SRM = np.maximum(visu_SRM, dynamic_data)

                visu_SRM *= (1 / np.max(visu_SRM) + 1e-6)
                visu_SRM *= 126
                visu_SRM = np.maximum(0, np.minimum(126, visu_SRM.astype(np.int8)))
                mask = visu_SRM > 0
                visu_SRM[mask] += 128

            # Define message data
            #   > static risk: yellow to red
            
            static_data0 = collision_preds[0, :, :].astype(np.float32)
            static_data = static_data0 * 98 / 255
            static_data = static_data * 1.06 - 3
            static_data = np.maximum(0, np.minimum(98, static_data.astype(np.int8)))
            static_data[static_mask] = 99

            # Static message does not change
            msg_static.data = static_data.ravel().tolist()

            
            with self.visu_lock:

                # Save current data
                self.msg_static = msg_static
                self.msg_dyn = msg_dyn
                self.current_SRM = visu_SRM

        else:

            # Ignore publication if we do not have data yet
            with self.visu_lock:
                if self.current_SRM is not None:
                    if self.visu_mode == "v1":

                        # Time we want to show ( now - SOGM_start )
                        stamp0 = self.msg_dyn.info.map_load_time.sec + self.msg_dyn.info.map_load_time.nanosec * 1e-9
                        sec1, nsec1 = self.get_clock().now().seconds_nanoseconds()
                        stamp1 = sec1 + nsec1 * 1e-9
                        t_now = stamp1 - stamp0

                        # Corresponding layer
                        layer_i = int(np.round(t_now + self.current_T / self.dt))

                        if layer_i > self.current_SRM.shape[0]:
                            layer_i = int(np.round(t_now / self.dt))
                            self.current_T = 0.005
                        else:
                            self.current_T += 0.005

                        if layer_i != self.last_layer_i:

                            self.last_layer_i = layer_i
                            visu_data = self.current_SRM[layer_i]

                            # Publish
                            self.msg_dyn.data = visu_data.ravel().tolist()
                            self.visu_pub.publish(self.msg_dyn)
                            self.visu_pub_static.publish(self.msg_static)
                            
                    elif self.visu_mode == "v2":
                        
                        # Only publish when new data arrives
                        stamp0 = self.msg_dyn.info.map_load_time.sec + self.msg_dyn.info.map_load_time.nanosec * 1e-9
                        stamp0 = int(stamp0 * 1e3)
                        if stamp0 > self.last_layer_i:

                            self.last_layer_i = stamp0
                            visu_data = self.current_SRM

                            # Publish
                            self.msg_dyn.data = visu_data.ravel().tolist()
                            self.visu_pub.publish(self.msg_dyn)
                            self.visu_pub_static.publish(self.msg_static)

        return

    def publish_obstacles(self, obstacle_list):


        msg = ObstacleArrayMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Add point obstacles
        for obst_i, pos in enumerate(obstacle_list):

            obstacle_msg = ObstacleMsg()
            obstacle_msg.id = obst_i
            obstacle_msg.polygon.points = [Point32(x=pos[0], y=pos[1], z=0.0)]

            msg.obstacles.append(obstacle_msg)

        self.obstacle_pub.publish(msg)

        return

    def publish_pointcloud(self, new_points, predictions, f_times, f_rings, t0):

        t = [time.time()]

        # data structure of binary blob output for PointCloud2 data type
        output_dtype = np.dtype({'names': ['x', 'y', 'z', 'classif', 'time', 'ring'],
                                 'formats': ['<f4', '<f4', '<f4', '<i4', '<f4', '<u2']})

        # new_points = np.hstack((new_points, predictions))
        structured_pc2_array = np.empty([new_points.shape[0]], dtype=output_dtype)

        t += [time.time()]

        structured_pc2_array['x'] = new_points[:, 0]
        structured_pc2_array['y'] = new_points[:, 1]
        structured_pc2_array['z'] = new_points[:, 2]
        structured_pc2_array['classif'] = predictions.astype(np.int32)
        structured_pc2_array['time'] = f_times.astype(np.float32)
        structured_pc2_array['ring'] = f_rings.astype(np.uint16)

        # structured_pc2_array['frame_id'] = (features[:, -1] > 0.01).astype(np.uint8)
        
        t += [time.time()]

        # convert to Pointcloud2 message and publish
        msg = array_to_pointcloud2_fast(structured_pc2_array,
                                        rclTime(seconds=t0.sec, nanoseconds=t0.nanosec).to_msg(),
                                        self.map_frame_id,
                                        True)


        t += [time.time()]

        self.pointcloud_pub.publish(msg)
        

        t += [time.time()]

        # print(35 * ' ',
        #       35 * ' ',
        #       '{:^35s}'.format('Publish pointcloud {:.0f} + {:.0f} + {:.0f} + {:.0f} ms'.format(1000 * (t[1] - t[0]),
        #                                                                                         1000 * (t[2] - t[1]),
        #                                                                                         1000 * (t[3] - t[2]),
        #                                                                                         1000 * (t[4] - t[3]))))



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

                ######################
                # Publish collisions #
                ######################

                # Get collision predictions [1, T, W, H, 3] -> [T, W, H, 3]
                collision_preds = self.sigmoid_2D(preds_future)[0]


                # Get the diffused risk
                diffused_risk, obst_pos, static_mask = self.get_diffused_risk(collision_preds)

                # Convert stamp to float
                sec1 = batch.t0.sec
                nsec1 = batch.t0.nanosec
                stamp_sec = float(sec1) + float(int((nsec1) * 1e-6)) * 1e-3

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

                # Wait until current ros time reached desired value
                if self.simu:
                    simu_delay = 0.25
                else:
                    simu_delay = 0.0
                now_stamp = self.get_clock().now().to_msg()
                now_sec = float(now_stamp.sec) + float(int((now_stamp.nanosec) * 1e-6)) * 1e-3
                while (now_sec < stamp_sec + simu_delay):
                    now_stamp = self.get_clock().now().to_msg()
                    now_sec = float(now_stamp.sec) + float(int((now_stamp.nanosec) * 1e-6)) * 1e-3

                # Publish collision risk in a custom message
                print(35 * ' ', 35 * ' ', 'Publishing {:.3f} with a delay of {:.3f}s'.format(stamp_sec, now_sec - stamp_sec))
                self.publish_collisions(diffused_risk, stamp_sec, batch.p0, batch.q0)

                # This call dos not publish but just updates the data. Pubications are done in aother thread at a much higher rate
                self.publish_collisions_visu(diffused_risk, static_mask, batch.t0, batch.p0)
                if not self.simu:
                    self.publish_collisions_visu(None, None, None, None)
                
                t += [time.time()]

                #####################
                # Publish obstacles #
                #####################

                # Get obstacles in world coordinates
                origin0 = batch.p0 - self.config.radius_2D / np.sqrt(2)

                world_obst = []
                for obst_i, pos in enumerate(obst_pos):
                    world_obst.append(origin0[:2] + pos * self.config.dl_2D)

                # Publish obstacles
                self.publish_obstacles(world_obst)

                #####################
                # Publish 3D points #
                #####################

                # Get predictions
                predicted_probs = self.softmax(outputs_3D).cpu().detach().numpy()
                for l_ind, label_value in enumerate(self.online_dataset.label_values):
                    if label_value in self.online_dataset.ignored_labels:
                        predicted_probs = np.insert(predicted_probs, l_ind, 0, axis=1)
                predictions = self.online_dataset.label_values[np.argmax(predicted_probs, axis=1)].astype(np.uint8)

                # Get frame points re-aligned in the velodyne coordinates
                pred_points = batch.points[0].cpu().detach().numpy() + batch.p0

                # R0 = scipyR.from_quat(batch.q0).as_matrix()
                # pred_points = np.dot(pred_points - batch.p0, R0)
                
                t += [time.time()]
                
                # ############################################################################################################
                # # DEBUG: Save input frames
                # plyname = join(ENV_HOME, 'results/ptpreds_{:.3f}.gif'.format(stamp1))
                # write_ply(plyname,
                #           [pred_points.astype(np.float32), predictions],
                #           ['x', 'y', 'z', 'classif'])
                # ############################################################################################################


                # # Remove static point for better visu in rviz (publish as a separate topic)
                # mask_pred = predictions > 2.5
                # pred_points = pred_points[mask_pred]
                # features = batch.features[mask_pred]
                # predictions = predictions[mask_pred]

                # Only keep points from last frame
                last_frame_mask = batch.features.cpu().detach().numpy()[:, -1] > 0.1
                pred_points = pred_points[last_frame_mask]
                predictions = predictions[last_frame_mask]
                f_times = batch.f_times[last_frame_mask]
                f_rings = batch.f_rings[last_frame_mask]

                # Publish pointcloud
                self.publish_pointcloud(pred_points, predictions, f_times, f_rings, batch.t0)
                
                t += [time.time()]

                print(35 * ' ',
                      35 * ' ',
                      '{:^35s}'.format('GPU : Inference Done in {:.0f} + {:.0f} + {:.0f} + {:.0f} + {:.0f} ms'.format(1000 * (t[1] - t[0]),
                                                                                                                      1000 * (t[2] - t[1]),
                                                                                                                      1000 * (t[3] - t[2]),
                                                                                                                      1000 * (t[4] - t[3]),
                                                                                                                      1000 * (t[5] - t[4]))))


        return


# ----------------------------------------------------------------------------------------------------------------------
#
#           Main Call
#       \***************/
#


def main(args=None):

    # Setup the collider Class
    print('\n\n\n\n        ------ Init Collider ------')
    rclpy.init(args=args)
    tester = OnlineCollider()

    # Spin in a separate thread
    executor = MultiThreadedExecutor()

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
