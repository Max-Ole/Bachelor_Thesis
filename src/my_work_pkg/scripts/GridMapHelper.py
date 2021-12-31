# -*- coding: utf-8 -*-
"""
Created on Wed Dec 22 2021

@author: Max-Ole von Waldow

Helper functions to use grid map with Python
"""
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
import time
import rospy
import geometry_msgs.msg
from grid_map_msgs.msg import GridMap # type: grid_map_msgs/GridMap



class GridMapHelper:

	def __init__(self):
		self.mapSizeCells  = None
		self.mapSizeMeters = None # From Message or Parameter server: /elevation_mapping/length_in_x, /elevation_mapping/length_in_y
		self.mapRes = None        # From Message or Parameter server: /elevation_mapping/resolution
		self.mapPosition = None   # Position in [m] of map center in the frame from the header
		self.mapOrientation = None# Orientation as Quaternion in the frame
		self.frame = None         # frame_id
		self.gmap = {}            # map data ordered as dict with layers as keys
		
		self.sub = rospy.Subscriber("/elevation_mapping/elevation_map_raw", GridMap, self.recieveMap, queue_size=1)
		
	def recieveMap(self, msg):
		# flip map
		print("\ngot map:\n")
		self.mapRes = msg.info.resolution
		self.mapSizeMeters = (msg.info.length_x, msg.info.length_y)
		self.mapSizeCells  = ( int(msg.info.length_x / msg.info.resolution), \
							   int(msg.info.length_y / msg.info.resolution) )
		self.mapPosition   = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
		self.mapOrientation= (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
		self.frame         = msg.header.frame_id
		
		# save all layers
		for idx,l in enumerate(msg.layers):
			elevationMap = np.reshape(msg.data[idx].data, (self.mapSizeCells[0], self.mapSizeCells[1]), order='F')
			self.gmap[l] = np.flipud(np.fliplr(np.transpose(elevationMap)))
		
		#### none nan value percentage of map layer
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
	def pos2Cell(self, x_meter, y_meter):
	
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
		x_cell = int(round( (y_meter + self.mapPosition[1]) / self.mapRes + self.mapSizeCells[1]/2) ) # why here y in size for x?
		y_cell = int(round( (x_meter + self.mapPosition[0]) / self.mapRes + self.mapSizeCells[0]/2) ) # same here

		# clip so we do not get index out of bound errors
		x_cell = self.clipVal(x, 0, self.mapSizeCells[0] - 1)
		y_cell = self.clipVal(y, 0, self.mapSizeCells[1] - 1)

		return x_cell, y_cell

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

    def __init__( self, systemSideLength, lowerCutoffLength ):
		""" setup surface plot """
		matplotlib.interactive(True)
		
        self.systemSideLength = systemSideLength
        self.lowerCutoffLength = lowerCutoffLength
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot( 111, projection='3d' )
        self.ax.set_zlim3d( -10e-9, 10e9 )

        rng = np.arange( 0, self.systemSideLength, self.lowerCutoffLength )
        self.X, self.Y = np.meshgrid(rng,rng)

        self.ax.w_zaxis.set_major_locator( LinearLocator( 10 ) )
        self.ax.w_zaxis.set_major_formatter( FormatStrFormatter( '%.03f' ) )

        heightR = np.zeros( self.X.shape )
        self.surf = self.ax.plot_surface( 
            self.X, self.Y, heightR, rstride=1, cstride=1, 
            cmap=cm.jet, linewidth=0, antialiased=False )

	def updateMap(self, mapHelperObj, layer)
		""" gets data from grid map and plots it """
		self.drawNow(mapHelperObj.gmap[layer])

    def drawNow( self, heightR ):
		""" update surface plot with new data """
        self.surf.remove()
        self.surf = self.ax.plot_surface( 
            self.X, self.Y, heightR, rstride=1, cstride=1, 
            cmap=cm.jet, linewidth=0, antialiased=False )
        plt.draw() # redraw the canvas
		# might need fig.canvas.flush_events() here
        plt.pause(0.01)