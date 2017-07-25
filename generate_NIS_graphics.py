import numpy as np
import matplotlib.pyplot as plt

lidar_threshold = 5.991
radar_threshold = 7.815

NIS_lidar = [line.rstrip('\n') for line in open('NIS_lidar_data_file.txt')]
NIS_radar = [line.rstrip('\n') for line in open('NIS_radar_data_file.txt')]

float_values_lidar = [float(i) for i in NIS_lidar]
float_values_radar = [float(i) for i in NIS_radar]
lidar_percentage = sum(i > lidar_threshold for i in float_values_lidar) / sum(float_values_lidar)
radar_percentage = sum(i > radar_threshold for i in float_values_radar) / sum(float_values_radar)

fig = plt.figure()

f, (a1, a2) = plt.subplots(2,1)
f.tight_layout(pad=3.0, w_pad=1.0, h_pad=3.0)

a1.set_title("Lidar NIS")    
a1.set_xlabel('Number of measurement')
a1.set_ylabel('NIS vaule')
a1.plot(NIS_lidar, c='b', label='NIS vaule')
a1.axhline(y=lidar_threshold, c='b')
a1.text(0, 13, 'Percentage of values above threshold: ' + str(format(lidar_threshold, '.2f')) +'%')
a1.legend()

a2.set_title("Radar NIS")    
a2.set_xlabel('Number of measurement')
a2.set_ylabel('NIS vaule')
a2.plot(NIS_radar, c='b', label='NIS vaule')
a2.axhline(y=radar_threshold, c='b')
a2.text(50, 60, 'Percentage of values above threshold: ' + str(format(radar_percentage, '.2f')) +'%')
a2.legend()




plt.savefig("NIS_plot.png")