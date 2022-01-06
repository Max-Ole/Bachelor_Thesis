from math import acos, tan, radians, degrees, pi

def law_of_cosines(a,b,c):
	"""Get angle from known lengths in triangle with
Law of cosines. The triangle does not need a right angle:
c**2 = a**2 + b**2 - 2*a*b*cos(gamma)
<=>
gamma = acos( (a**2+b**2-c**2) / (2*a*b) )
"""
	return acos( (a**2+b**2-c**2) / (2*a*b) ) # =gamma



# measurements
height_front_resting = 0.366
height_back_resting  = 0.375
height_left_resting  = 0.254
height_right_resting = 0.258

# width and length: distance between the edges the robot was tipped around
chassis_width = 0.310 # chassis_width from urdf
chassis_length = 0.420 # chassis_length from urdf
dist_measurement = 0.2 # distance of edge to base of height measurement

alpha = pi/2 - law_of_cosines(dist_measurement, chassis_width, height_left_resting)
beta  = pi/2 - law_of_cosines(dist_measurement, chassis_width, height_right_resting)
gamma = pi/2 - law_of_cosines(dist_measurement, chassis_length, height_back_resting)
gamma_alt = pi/2 - law_of_cosines(dist_measurement, chassis_length, height_front_resting)
print("angles from law of cosines: \nalpha    = {:1.4f} \nbeta     = {:1.4f} \ngamma    = {:1.4f} \ngamma_alt= {:1.4f} \n".format(degrees(alpha), degrees(beta), degrees(gamma), degrees(gamma_alt)))

# calc sidewards coord of CoG
# dist from right side
b2 = (tan(alpha) * chassis_width) / (tan(alpha) + tan(beta))
# dist from left side
b1 = chassis_width - b2
assert abs(b1-b2) < 0.01, "b1 and b2 do not add up to the chassis_width by a difference of " + str(abs(b1-b2)) + " meters.\n The measurements seem inaccurate."

# calc height of CoG
h = tan(beta) * b2

# calc front/back coord of CoG
# dist from back side
x_back = h / tan(gamma)
# dist from front side
x_front = h / tan(gamma_alt)

x_diff = abs(x_back + x_front - chassis_length)

print("Distances of center of gravity to the edges it was tipped around:")
print("b1:  {:1.4f} \nb2:  {:1.4f} \nh:   {:1.4f} \nx_back:\n     {:1.4f} \nx_front:\n     {:1.4f}".format(b1,b2,h,x_back,x_front))
print("Difference of redundant front/back measurement: {:1.4f}".format(x_diff))

# CoG from center of robot. = base_link of Jackal. With average of front and back measurement
CoG_x = chassis_length/2 - (x_front + chassis_length-x_back)/2
CoG_y = chassis_width/2 - b2
CoG_z = h

print("\nCenter of gravity as expressed in frame in center of robot. With average of front and back measurement.")
print("CoG_x:  {:+1.4f} \nCoG_y:  {:+1.4f} \nCoG_z:  {:+1.4f}".format(CoG_x, CoG_y, CoG_z))

"""
Output for Jackal Robot with Nvidia Jetson, Lidar and ZED camera installed:

angles from law of cosines:
alpha    = 35.2602
beta     = 34.1094
gamma    = 26.8105
gamma_alt= 29.3891

Distances of center of gravity to the edges it was tipped around:
b1:  0.1517
b2:  0.1583
h:   0.1072
x_back:
     0.2122
x_front:
     0.1904
Difference of redundant front/back measurement: 0.0174

Center of gravity as expressed in frame in center of robot. With average of front and back measurement.
CoG_x:  +0.0109
CoG_y:  -0.0033
CoG_z:  +0.1072
"""