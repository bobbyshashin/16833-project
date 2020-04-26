import ros_numpy
import pickle
import numpy as np
import matplotlib.pyplot as plt

import icp

id = '00_corner_last'

pc_list = pickle.load(open('pc'+id+'.p', 'rb'))
print(len(pc_list))
print(type(pc_list[0]))

path_list = pickle.load(open('path'+id+'.p', 'rb'))
print(len(path_list))
print(type(path_list[0]))

fig = plt.figure(figsize=(20,10))
x_list = []
y_list = []
for pose in path_list:
    x_list.append(pose.pose.position.x)
    y_list.append(pose.pose.position.y)
# plt.scatter(x_list, y_list)
plt.plot(x_list, y_list, '.', markersize=1)

past_key_frame = {}
for i in range(len(pc_list)):
    PC = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_list[i])
    if i % 20 == 0:
        if len(past_key_frame) > 0:
            icp_distances_ls = []
            frame_id_ls = []
            for key_frame_id in past_key_frame:
                if i - key_frame_id > 700:
                    source, target = PC.copy(), past_key_frame[key_frame_id].copy()
                    if source.shape[0] > target.shape[0]:
                        source = source[ np.random.choice(source.shape[0], target.shape[0], replace=False) ]
                    elif source.shape[0] < target.shape[0]:
                        target = target[ np.random.choice(target.shape[0], source.shape[0], replace=False) ]
                    T, distances, itr = icp.icp(source, target, max_iterations=50)
                    icp_distances_ls.append(np.mean(distances))
                    frame_id_ls.append(key_frame_id) 
            if len(icp_distances_ls) > 0:
                min_distance = min(icp_distances_ls)
                if min_distance < 0.6:
                    closest_id = frame_id_ls[ np.argmin(icp_distances_ls) ]
                    print("input frame id: {}, closest past frame id: {}, distance: {:.4f}"
                          .format(i, closest_id, min_distance))
                    pc_frame1, pc_frame2 = closest_id, i
                    time1 = pc_list[pc_frame1].header.stamp
                    time2 = pc_list[pc_frame2].header.stamp
                    for pose in path_list:
                        if pose.header.stamp > time1:
                            x1, y1 = pose.pose.position.x, pose.pose.position.y
                            break
                    for pose in path_list:
                        if pose.header.stamp > time2:
                            x2, y2 = pose.pose.position.x, pose.pose.position.y
                            break
                    plt.plot([x1, x2],[y1, y2], 'r-', linewidth=1)

        past_key_frame[i] = PC

plt.savefig('/home/boshenniu/Downloads/'+id+'.png', dpi=400)
# plt.show()


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.scatter(cornerPC[:, 0], cornerPC[:, 1], cornerPC[:, 2], c='r', marker='.')
# self.plot_flag = False
# plt.show()