# -*- coding: utf-8 -*-
"""
Created on Wed Dec 22 2021

@author: Max-Ole von Waldow

Helper functions to use grid map with Python
"""
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib import cm
import time
import rospy
import geometry_msgs.msg
from grid_map_msgs.msg import GridMap # type: grid_map_msgs/GridMap



class GridMapHelper:

	def __init__(self):
		self.mapSizeCells  = None
		self.mapSizeMeters = None # From Parameter server: /elevation_mapping/length_in_x, /elevation_mapping/length_in_y
		self.mapRes = None        # From Parameter server: /elevation_mapping/resolution
		self.gmap = {}
		
		self.sub = rospy.Subscriber("/elevation_mapping/elevation_map_raw", GridMap, self.recieveMap, queue_size=1)
		
	def recieveMap(self, msg):
		# flip map
		print("\ngot map:\n")
		self.mapRes = msg.info.resolution
		self.mapSizeMeters = (msg.info.length_x, msg.info.length_y)
		self.mapSizeCells  = ( int(msg.info.length_x / msg.info.resolution), \
							   int(msg.info.length_y / msg.info.resolution) )
		
		# save all layers
		for idx,l in enumerate(msg.layers):
			elevationMap = np.reshape(msg.data[idx].data, (self.mapSizeCells[0], self.mapSizeCells[1]), order='F')
			self.gmap[l] = np.flipud(np.fliplr(np.transpose(elevationMap)))
		
		####
		a = np.count_nonzero(~np.isnan(self.gmap['limit_slope']))
		print a
		print 100.0 * float(a) / (self.mapSizeCells[0]*self.mapSizeCells[1])

		print "at (0,0):  " + str(self.getCallValue('limit_slope', 80,80))
		print "at (20,0): " + str(self.getCallValue('limit_slope', 20,0))
		
		print "cellpos at (0,0):  " + str(self.pos2Cell(0,0))
		print "cellpos at (1,0):  " + str(self.pos2Cell(1,0))
		print "cellpos at (0,1):  " + str(self.pos2Cell(0,1))
		
		#self.plotGridMap()
		
	def getCallValue(self, layer, cell_x, cell_y):
		""" read value of a given cell in a given layer"""
		assert layer in self.gmap.keys(), "ERROR: The layer name given to given getCallValue does not exist. Correct spelling or verify initialization of map."
		return self.gmap[layer][cell_x, cell_y]
	
	
	# returns the cell the robot is in to its corresponding position
	def pos2Cell(self, inX, inY):
	
		"""
		vectorToOrigin = (0.5 * mapLength).matrix();
		
		
		positionOfOrigin = position + vectorToOrigin;


		inline Index getIndexFromIndexVector(
			const Vector& indexVector,
			const Size& bufferSize,
			const Index& bufferStartIndex)
		{
		  Index index = transformMapFrameToBufferOrder(indexVector);
		  return getBufferIndexFromIndex(index, bufferSize, bufferStartIndex);
		}
		
		
		Index getBufferIndexFromIndex(const Index& index, const Size& bufferSize, const Index& bufferStartIndex)
		{
		  if (checkIfStartIndexAtDefaultPosition(bufferStartIndex)) return index;

		  Index bufferIndex = index + bufferStartIndex;
		  wrapIndexToRange(bufferIndex, bufferSize);
		  return bufferIndex;
		}
		
		
		getVectorToOrigin(vectorToOrigin, mapLength);
		Vector indexVector = ((position - vectorToOrigin - mapPosition).array() / resolution).matrix();
		index = getIndexFromIndexVector(indexVector, bufferSize, bufferStartIndex);
		  
		"""
		x = int(round((inY) / self.mapRes + self.mapSizeCells[1]/2)) # why here y in size for x?
		y = int(round((inX) / self.mapRes + self.mapSizeCells[0]/2)) # same here

		# clip so we do not get index out of bound errors
		if x < 0:
			x = 0
		elif x >= self.mapSizeCells[0]:
			x = self.mapSizeCells[0] - 1
		if y < 0:
			y = 0
		elif y >= self.mapSizeCells[1]:
			y = self.mapSizeCells[1] - 1

		return x, y

	# returns the position of the robot from the corresponding cell
	def cell2Pos(self, cellX, cellY):
		return ((cellY - self.mapSizeCells[1]/2) * self.mapRes), ((cellX - self.mapSizeCells[0]/2) * self.mapRes)
		
	def clipVal(self, val, min_val, max_val):
		""" Keep value inside given bounds. Set value to nearest bound if it is outside. """
		return  min_val if val < min_val  else max_val if val > max_val  else val
		
	def plotGridMap(self, ):
		ax = plt.axes(projection ="3d")
		 
		X, Y = np.meshgrid(list(range(self.mapSizeCells[0])), list(range(self.mapSizeCells[1])))
		ax.plot_surface(X, Y, self.gmap['limit_slope'], cmap=cm.coolwarm)
		plt.title("Grid Map Surface plot")

		ax.set_xlabel('X Label')
		ax.set_ylabel('Y Label')
		ax.set_zlabel('Z Label')
		plt.show()
		time.sleep(1)
