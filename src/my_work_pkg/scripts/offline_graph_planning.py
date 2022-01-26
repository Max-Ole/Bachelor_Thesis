import matplotlib.pyplot as plt
import networkx as nx
import time
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

import GridMapHelper



rospy.init_node("map_limit_graph", anonymous=True)
topic = 'visualization_marker_array'
markerPublisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
pathPublisher = rospy.Publisher("/astar_path", Path, queue_size=10)

mapManager = GridMapHelper.GridMapHelper()
# wait for map
while not mapManager.gmap:
	time.sleep(1)
print("got map")

# create graph
(x_dim, y_dim) = mapManager.mapSizeCells
print(x_dim, y_dim)
#G = nx.grid_2d_graph(x_dim,y_dim)
G = nx.Graph()
print(nx.info(G))

def weight_from_limit(alimit):
	""" gets acceleration limit and transforms it into edge weight. """
	if alimit!=alimit: 
		# is nan
		weight = 100
	elif alimit <= 0:
		weight = float('inf')
	else:
		weight = 100 - limit**2
	
	return weight

# set traversability cost as edge weights
# G[(0,0)][(0,-1)]['weight'] = 5
for x in range(x_dim):
	for y in range(y_dim):
	
		limit = mapManager.getCellValue('a', x, y)
		w = weight_from_limit(limit)
		G.add_edge((x,y), (x,y+1), weight=w)
		G.add_edge((x,y), (x+1,y), weight=w)
		G.add_edge((x,y), (x+1,y+1), weight=w)
		#G[(x,y)][(x,y+1)]['weight'] = limit
		#G[(x,y)][(x+1,y)]['weight'] = limit
		if x%10==0 and y%10==0:
			print("({:3d}, {:3d}) with limit {:6.1f} --> weight {:6.1f}".format(x, y, limit, w))
		

print("made graph")
print(nx.info(G))

def dist(a, b):
	(x1, y1) = a
	(x2, y2) = b
	return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
	
def visualizeMarkers( points):
	"""
	visualize the nodes as markers
	"""
	print(type(points))
	print(type(points[0]))
	print(points[0])
	markerArray = MarkerArray()
	x = float(points[0])
	y = float(points[1])
	marker = Marker()
	marker.header.frame_id = "/odom"
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

def publish_path(path):
	"""
	This method publishes a Path msg with the path positions to the goal
	Args:
		path ([]): node path to the goal
	Returns:
	"""
	print("publishing path...")
	msg = Path()
	msg.header.frame_id = "odom"
	#add node positions from path to the msg
	msg.header.stamp = rospy.Time.now()
	msg.poses = [PoseStamped(pose=Pose(position=Point(x=n[0], y=n[1]))) for n in path]

	#publish the path
	pathPublisher.publish(msg)

start = (195, 159)
goal = (107, 191)
# path_cells = nx.astar_path(G,start,goal,heuristic=dist,weight='weight')
path_cells = nx.astar_path(G,start,goal,weight='weight')
path_meters = []
for point in path_cells:
	path_meters.append( mapManager.cell2Pos(*point) )

print("Cells on path: {}".format(len(path_meters)))
costs = []
for idx in range(len(path_cells)-1):
	p1 = path_cells[idx]
	p2 = path_cells[idx+1]
	costs.append( G[p1][p2]['weight'] )
	print G[p1][p2]['weight']

print("total cost: {:6.2f}".format(sum(costs)))
publish_path(path_meters)
#mapManager.plotGridMap(layer='a')

#time.sleep(15)
#nx.draw(G)
#plt.draw()
#plt.show()
