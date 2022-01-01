# -*- coding: utf-8 -*-
"""
Created on Wed Dec 22 2021

@author: Max-Ole von Waldow

Helper functions to use grid map with Python
"""
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
from mpl_toolkits import mplot3d
import time
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from grid_map_msgs.msg import GridMap # type: grid_map_msgs/GridMap
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



class GridMapHelper:

	def __init__(self):
		self.mapSizeCells  = None
		self.mapSizeMeters = None # From Message or Parameter server: /elevation_mapping/length_in_x, /elevation_mapping/length_in_y
		self.mapRes = None		# From Message or Parameter server: /elevation_mapping/resolution
		self.mapPosition = None   # Position in [m] of map center in the frame from the header
		self.mapOrientation = None# Orientation as Quaternion in the frame
		self.frame = None		 # frame_id
		self.gmap = {}			# map data ordered as dict with layers as keys
		
		self.goal = None
		rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalpose_callback)
		rospy.Subscriber("/elevation_mapping/elevation_map_raw", GridMap, self.recieveMap, queue_size=1)
			
		topic = 'visualization_marker_array'
		self.markerPublisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
	
	def goalpose_callback(self, goal_msg):		
		""" Gets Position from Rviz """
		self.goal = (goal_msg.pose.position.x, goal_msg.pose.position.y)
		print "\nGot goal position [m]: " + str(self.goal)
		print "Corresponds to cell [idx]: " + str(self.pos2Cell(*self.goal))
		print "Elevation value is: " + str(self.getCellValue('elevation', *self.pos2Cell(*self.goal)))
		print "limits_merged value is: " + str(self.getCellValue('limits_merged', *self.pos2Cell(*self.goal)))
		print " "
	
	def recieveMap(self, msg):
		# flip map
		#print("\ngot map:\n")
		self.mapRes = msg.info.resolution
		self.mapSizeMeters = (msg.info.length_x, msg.info.length_y)
		self.mapSizeCells  = ( int(msg.info.length_x / msg.info.resolution), \
							   int(msg.info.length_y / msg.info.resolution) )
		self.mapPosition   = (msg.info.pose.position.x, msg.info.pose.position.y, msg.info.pose.position.z)
		self.mapOrientation= (msg.info.pose.orientation.x, msg.info.pose.orientation.y, msg.info.pose.orientation.z, msg.info.pose.orientation.w)
		self.frame		 = msg.info.header.frame_id
		
		# save all layers
		for idx,l in enumerate(msg.layers):
			elevationMap = np.reshape(msg.data[idx].data, (self.mapSizeCells[0], self.mapSizeCells[1]), order='F')
			self.gmap[l] = np.flipud(np.fliplr(np.transpose(elevationMap)))
		
		#### none nan value percentage of map layer
		a = np.count_nonzero(~np.isnan(self.gmap['limit_slope']))
		"""print a
		print 100.0 * float(a) / (self.mapSizeCells[0]*self.mapSizeCells[1])

		print "at (0,0):  " + str(self.getCellValue('elevation', 80,80))
		print "at (20,0): " + str(self.getCellValue('elevation', 20,0))
		
		print "cellpos at (0,0):  " + str(self.pos2Cell(0,0)) + "  with value: " + str(self.getCellValue('elevation', *self.pos2Cell(0,0)))
		print "cellpos at (1,0):  " + str(self.pos2Cell(1,0))
		print "cellpos at (0,1):  " + str(self.pos2Cell(0,1))"""
		
		#self.plotGridMap()
		
	def getCellValue(self, layer, cell_x, cell_y):
		""" read value of a given cell in a given layer"""
		assert layer in self.gmap.keys(), "ERROR: The layer name given to given getCellValue does not exist. Correct spelling or verify initialization of map."
		return self.gmap[layer][cell_x, cell_y]
	
	
	# returns the cell the robot is in to its corresponding position
	def pos2Cell(self, x_meter, y_meter):
		"""
		cell[1] index increases as x position increases
		cell[0] index increases as y position increases """
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
		#x = int(round((y_meter) / self.mapRes + self.mapSizeCells[1]/2)) # why here y in size for x?
		#y = int(round((x_meter) / self.mapRes + self.mapSizeCells[0]/2)) # same here
		
		# turning needed?
		# mapPosition could be negative?
		x_cell = int(round( (y_meter - self.mapPosition[1]) / self.mapRes + self.mapSizeCells[1]/2) ) # why here y in size for x?
		y_cell = int(round( (x_meter - self.mapPosition[0]) / self.mapRes + self.mapSizeCells[0]/2) ) # same here

		# clip so we do not get index out of bound errors
		x_cell = self.clipVal(x_cell, 0, self.mapSizeCells[0] - 1)
		y_cell = self.clipVal(y_cell, 0, self.mapSizeCells[1] - 1)

		return x_cell, y_cell

	# returns the position of the robot from the corresponding cell
	def cell2Pos(self, cellX, cellY):
		return ((cellY - self.mapSizeCells[1]/2) * self.mapRes), ((cellX - self.mapSizeCells[0]/2) * self.mapRes)
		
	def clipVal(self, val, min_val, max_val):
		""" Keep value inside given bounds. Set value to nearest bound if it is outside. """
		return  min_val if val < min_val  else max_val if val > max_val  else val
	
	def visualizeMarkers(self, x, y):
		"""
		visualize the nodes as markers
		"""
		markerArray = MarkerArray()
		marker = Marker()
		marker.header.frame_id = "/map"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = 0
		markerArray.markers.append(marker)

		# Renumber the marker IDs
		id = 0
		for m in markerArray.markers:
			m.id = id
			id += 1

		# Publish the MarkerArray
		self.markerPublisher.publish(markerArray)
	
	"""
		Plotting
	"""
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

	def plotGridMap_live_init(self, ):
		ax = plt.axes(projection ="3d")
		 
		X, Y = np.meshgrid(list(range(self.mapSizeCells[0])), list(range(self.mapSizeCells[1])))
		ax.plot_surface(X, Y, self.gmap['limit_slope'], cmap=cm.coolwarm)
		plt.title("Grid Map Surface plot")

		ax.set_xlabel('X Label')
		ax.set_ylabel('Y Label')
		ax.set_zlabel('Z Label')
		
		# plt.ion() equals setting interactive to True: matplotlib.interactive(True)
		matplotlib.interactive(True)
		
		plt.draw()
		# might need fig.canvas.flush_events() here
		plt.pause(0.01)


class SurfacePlotLive:
	""" manages a surface plot that is updated live. Source: https://stackoverflow.com/questions/5179589/continuous-3d-plotting-i-e-figure-update-using-python-matplotlib """

	def __init__( self, mapHelperObj, systemSideLength=8, lowerCutoffLength=-10 ):
		""" setup surface plot """
		matplotlib.interactive(True)
		
		self.mapHelperObj = mapHelperObj
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot( 111, projection='3d' )
		self.ax.set_zlim3d( -10e-9, 10e9 )

		rng = np.arange( 0, systemSideLength, lowerCutoffLength )
		while(self.mapHelperObj.mapSizeCells is None):
			print("waiting for grid map")
			pass
		self.X, self.Y = np.meshgrid(list(range(self.mapHelperObj.mapSizeCells[0])), list(range(self.mapHelperObj.mapSizeCells[1])))

		self.ax.w_zaxis.set_major_locator( LinearLocator( 10 ) )
		self.ax.w_zaxis.set_major_formatter( FormatStrFormatter( '%.03f' ) )

		heightR = np.zeros( self.X.shape )
		self.surf = self.ax.plot_surface( 
			self.X, self.Y, heightR, rstride=1, cstride=1, 
			cmap=cm.jet, linewidth=0, antialiased=False )

	def updateMap(self, *args):#msg, mapHelperObj, layer):
		""" gets data from grid map and plots it """
		#print(type(msg), type(mapHelperObj), type(layer))
		#print msg
		print "updateMap\n\n"
		self.drawNow(self.mapHelperObj.gmap["elevation"])

	def drawNow( self, heightR ):
		""" update surface plot with new data """
		print "\n\ndrawNow\n\n"
		self.surf.remove()
		self.surf = self.ax.plot_surface( 
			self.X, self.Y, heightR, rstride=1, cstride=1, 
			cmap=cm.jet, linewidth=0, antialiased=False )
		plt.draw() # redraw the canvas
		# might need fig.canvas.flush_events() here
		#self.fig.canvas.flush_events()
		print "drawing"
		plt.pause(0.1)
