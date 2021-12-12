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
b2 = (tan(alpha) * chassis_width) / (tan(alpha) + tan(beta))
b1 = chassis_width - b2
assert abs(b1-b2) < 0.01, "b1 and b2 do not add up to the chassis_width by a difference of " + str(abs(b1-b2)) + " meters.\n The measurements seem inaccurate."

# calc height of CoG
h = tan(beta) * b2

# calc front/back coord of CoG
x = h / tan(gamma)
x_alt = h / tan(gamma_alt)

print("b1:  {:1.4f} \nb2:  {:1.4f} \nh:   {:1.4f} \nx:   {:1.4f} \n\nalternative x:\n     {:1.4f} \ndifference:\n     {:1.4f}".format(b1,b2,h,x,x_alt, abs(x-x_alt)))

