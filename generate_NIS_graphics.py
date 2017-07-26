import numpy as np
import matplotlib.pyplot as plt

radar_threshold = 7.815

NIS_radar = [line.rstrip('\n') for line in open('NIS_radar_data_file.txt')]

float_values_radar = [float(i) for i in NIS_radar]
radar_percentage = sum(i > radar_threshold for i in float_values_radar) / sum(float_values_radar)

plt.title("Radar NIS")    
plt.xlabel('Number of measurement')
plt.ylabel('NIS vaule')
plt.plot(NIS_radar, c='b', label='NIS vaule')
plt.axhline(y=radar_threshold, c='b')
plt.text(5, 60, 'Percentage of values above threshold: ' + str(format(radar_percentage, '.2f')) +'%')
plt.legend()

plt.savefig("NIS_plot.png")