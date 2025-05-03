
# https://github.com/OSUrobotics/laser_to_image/tree/master

import numpy as np
import math
import json
import time
import os
from sensor_msgs.msg import LaserScan
# Discretization Size
disc_size = .05
# Discretization Factor
disc_factor = 1/disc_size
# Max Lidar Range
# Create Image Size Using Range and Discretization Factor

class LaserToImage():
	def __init__(self):
		self.file_saved = False

	def scan_to_image(self,scan:LaserScan):
		# Store maxAngle of lidar
		self.save_scan(scan)
		maxAngle = scan.angle_max
		# Store minAngle of lidar
		self.minAngle = scan.angle_min
		# Store angleInc of lidar
		self.angleInc = scan.angle_increment
		# Store maxLength in lidar distances
		maxLength = scan.range_max
		# Store array of ranges
		ranges = scan.ranges
		index_pts = [j for j in range(len(ranges))]
		# Calculate the number of points in array of ranges
		num_pts = len(ranges)
		# Create Array for extracting X,Y points of each data point
		xy_scan = np.zeros((num_pts,2))
		# Create 3 Channel Blank Image
		image_size = int(maxLength*2*disc_factor)
		blank_image = np.zeros((image_size,image_size,4),dtype=np.uint8)
		# Loop through all points converting distance and angle to X,Y point
		for i in range(num_pts):
			# Check that distance is not longer than it should be
			if (ranges[i] > maxLength) or (math.isnan(ranges[i])):
				pass
			else:
				# Calculate angle of point and calculate X,Y position
				angle = self.minAngle + float(i)*self.angleInc
				xy_scan[i][0] = float(ranges[i]*math.cos(angle))
				xy_scan[i][1] = float(ranges[i]*math.sin(angle))

		# Loop through all points plot in blank_image
		for i in range(num_pts):
			pt_x = xy_scan[i,0]
			pt_y = xy_scan[i,1]
			if (pt_x < maxLength) or (pt_x > -1 * (maxLength-disc_size)) or (pt_y < maxLength) or (pt_y > -1 * (maxLength-disc_size)):
				pix_x = int(math.floor((pt_x + maxLength) * disc_factor))
				pix_y = int(math.floor((maxLength - pt_y) * disc_factor))
				index_pts[i] = [pix_x, pix_y]
				if (pix_x > image_size) or (pix_y > image_size):
					print ("Error")
				else:
					blank_image[pix_y - 1,pix_x + 1] = [255,0,0,255]
					blank_image[pix_y - 1,pix_x] = [255,0,0,255]
					blank_image[pix_y - 1,pix_x - 1] = [255,0,0,255]
					blank_image[pix_y,pix_x  + 1] = [255,0,0,255]
					blank_image[pix_y,pix_x] = [255,0,0,255]
					blank_image[pix_y,pix_x - 1] = [255,0,0,255]
					blank_image[pix_y + 1,pix_x + 1] = [255,0,0,255]
					blank_image[pix_y + 1,pix_x] = [255,0,0,255]
					blank_image[pix_y + 1,pix_x - 1] = [255,0,0,255]
		return blank_image

	
	def polar_to_cartesian(self, i, r):
		angle = self.minAngle + float(i)*self.angleInc
		x = float(r*math.cos(angle))
		y = float(r*math.sin(angle))
		return x, y
	
	def save_scan(self, scan:LaserScan):
		if not os.path.isfile("/home/ovali/scan.json"):
			ranges = list(scan.ranges)
			obj = {
				"angle_min": scan.angle_min,
				"angle_max": scan.angle_max,
				"angle_increment": scan.angle_increment,
				"ranges": ranges
			}
			with open("/home/ovali/scan.json", "w") as output:
				json.dump(obj, output)
			self.file_saved = True

def main(args=None):
    print("Please run this script only as a module")

if __name__ == '__main__':
    main()