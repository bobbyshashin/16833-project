import pickle
import numpy as np
import matplotlib.pyplot as plt

id = '00'

pc_list = pickle.load(open('pc'+id+'.p', 'rb'))
print(len(pc_list))
print(type(pc_list[0]))

path_list = pickle.load(open('path'+id+'.p', 'rb'))
print(len(path_list))
print(type(path_list[0]))

x_list = []
y_list = []
for pose in path_list:
    x_list.append(pose.pose.position.x)
    y_list.append(pose.pose.position.y)
plt.scatter(x_list, y_list)


file_path = id+'.txt'
with open(file_path) as fp:
    line = fp.readline()
    while line:
        # print(line[:-1].split(' '))
        line = line[:-1].split(' ')
        distance = float(line[2])
        if distance < 1.05:
            pc_frame1, pc_frame2 = int(line[1]), int(line[0])
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
            plt.plot([x1, x2],[y1, y2], color='red')

        line = fp.readline()

plt.show()

