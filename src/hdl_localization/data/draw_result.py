import matplotlib.pyplot as plt

gps_x = []
gps_y = []

file_name = 'gps_odom.txt'
with open(file_name) as f_obj:
    for line in f_obj:
        gps_data = line.split()
        gps_x.append(float(gps_data[0]))
        gps_y.append(float(gps_data[1]))

slam_x = []
slam_y = []

file_name = 'slam_odom.txt'
with open(file_name) as f_obj:
    for line in f_obj:
        slam_data = line.split()
        slam_x.append(float(slam_data[0]))
        slam_y.append(float(slam_data[1]))

#print(slam_data)
plt.scatter(gps_x, gps_y, c='black', edgecolor='none',s=5)
plt.scatter(slam_x, slam_y, c='red', edgecolor='none',s=5)
plt.title("GPS(black) and SLAM(red)")
plt.axis([-10, 120, -10, 120])

plt.show()
