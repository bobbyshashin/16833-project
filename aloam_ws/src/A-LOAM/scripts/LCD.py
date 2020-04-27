import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import ros_numpy

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pickle
from scipy.spatial.transform import Rotation as R

import icp

class LCD():
    def __init__(self):
        self.PC_sub = rospy.Subscriber('/laser_cloud_sharp', PointCloud2, self.PCCallback)
        # self.PC_sub = rospy.Subscriber('/laser_cloud_corner_last', PointCloud2, self.PCCallback)
        self.PC_frame = 0
        self.past_key_frame = {} # key: frame id, value: point cloud

        self.path_sub = rospy.Subscriber('/aft_mapped_path', Path, self.pathCallback)
        self.path_frame = 0

        self.PC_list = []
        self.path_list = [] 

        self.LC_pose_idx_pub = rospy.Publisher('/loop_closure_pose_idx', Odometry, queue_size=1)
        self.publish_flag = True 

    def PCCallback(self, PC):
        self.PC_list.append(PC)
        # pickle.dump(self.PC_list, open('pc.p', 'wb'))
        self.PC_frame += 1
        # print("PC frame number: ", self.PC_frame)
        
        PC = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(PC) # N*3
        # print("PC shape: ", PC.shape)
        
        if self.PC_frame % 20 == 0:
            if len(self.past_key_frame) > 0:
                icp_transform_ls = []
                icp_distances_ls = []
                frame_id_ls = []
                for key_frame_id in self.past_key_frame:
                    if self.PC_frame - key_frame_id > 700: # two frames should not be close, need to define the threshold
                        source, target = PC.copy(), self.past_key_frame[key_frame_id].copy()
                        if source.shape[0] > target.shape[0]:
                            source = source[ np.random.choice(source.shape[0], target.shape[0], replace=False) ]
                        elif source.shape[0] < target.shape[0]:
                            target = target[ np.random.choice(target.shape[0], source.shape[0], replace=False) ]
                        T, distances, itr = icp.icp(source, target)
                        # print("input frame id: {}, past frame id: {}".format(self.PC_frame, key_frame_id))
                        icp_transform_ls.append(T)
                        icp_distances_ls.append(np.mean(distances))
                        frame_id_ls.append(key_frame_id) 
                if len(icp_distances_ls) > 0:
                    min_distance = min(icp_distances_ls)
                    if min_distance < 1.0: # the error should be smaller than a threshold, need to define the threshold
                        closest_id = frame_id_ls[ np.argmin(icp_distances_ls) ]
                        closest_T = icp_transform_ls[ np.argmin(icp_distances_ls) ]
                        # print("input frame id: {}, closest past frame id: {}, distance: {:.4f}"
                        #       .format(self.PC_frame, closest_id, min_distance))
                        # print("{} {} {}".format(self.PC_frame, closest_id, min_distance))
                        if self.publish_flag == True:
                            self.publishLoopClosurePoseIndex(closest_id, self.PC_frame, closest_T)
                            self.publish_flag = False
            
            self.past_key_frame[self.PC_frame] = PC
    
    def pathCallback(self, path):
        self.path_list.append(path.poses[-1])
        # pickle.dump(self.path_list, open('path.p', 'wb'))
        self.path_frame += 1
        # print("path frame number: ", self.path_frame)
        # print(len(path.poses))
    
    def publishLoopClosurePoseIndex(self, pc_frame1, pc_frame2, T):
        print("pc frame 1: {}, pc frame 2: {}".format(pc_frame1, pc_frame2))
        time1 = self.PC_list[pc_frame1-1].header.stamp
        time2 = self.PC_list[pc_frame2-1].header.stamp
        for i in range(len(self.path_list)):
            if self.path_list[i].header.stamp > time1:
                pose_frame1 = i
                break
        pose_frame2 = len(self.path_list)-1
        print("pose frame 1: {}, pose frame 2: {}".format(pose_frame1, pose_frame2))
        
        pub_msg = Odometry()
        pub_msg.header.frame_id = str(pose_frame2)
        pub_msg.child_frame_id = str(pose_frame1)
        # print("transformation: ", T)
        # import pdb; pdb.set_trace()
        pub_msg.pose.pose.position.x = T[0,3]
        pub_msg.pose.pose.position.y = T[1,3]
        pub_msg.pose.pose.position.z = T[2,3]
        q = R.from_matrix(T[0:3,0:3]).as_quat()
        pub_msg.pose.pose.orientation.x = q[0]
        pub_msg.pose.pose.orientation.y = q[1]
        pub_msg.pose.pose.orientation.z = q[2]
        pub_msg.pose.pose.orientation.w = q[3]

        self.LC_pose_idx_pub.publish(pub_msg)


def runLCD():
    rospy.init_node('loopClosureDetection')
    my_LCD = LCD()
    rospy.spin()

if __name__ == '__main__':
    try:
        runLCD()
    except rospy.ROSInterruptException:
        pass