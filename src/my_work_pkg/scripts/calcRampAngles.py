from math import tan, radians

heights = [1003, 843, 703, 560, 422, 284, 146] # in mm

for angle in range(5,70,5): # in degrees
	print("\nangle:", angle)
	for h in heights:
			l = h/tan(radians(angle))
			if l<2000 and l>500:
					print("\t with height {:4d}. Use length {:6.1f}".format( h, l) )
					
					
# results in the following output:
"""
angle: 5
         with height  146. Use length 1668.8

angle: 10
         with height  284. Use length 1610.6	# 230, 230
         with height  146. Use length  828.0

angle: 15
         with height  422. Use length 1574.9	# 85, 125
         with height  284. Use length 1059.9
         with height  146. Use length  544.9

angle: 20
         with height  703. Use length 1931.5
         with height  560. Use length 1538.6
         with height  422. Use length 1159.4	# 185, 200; 95, 115
         with height  284. Use length  780.3

angle: 25
         with height  843. Use length 1807.8	# -100; 105
         with height  703. Use length 1507.6
         with height  560. Use length 1200.9
         with height  422. Use length  905.0
         with height  284. Use length  609.0

angle: 30
         with height 1003. Use length 1737.2
         with height  843. Use length 1460.1	# 110, 110
         with height  703. Use length 1217.6
         with height  560. Use length  969.9
         with height  422. Use length  730.9

angle: 35
         with height 1003. Use length 1432.4	# 0.02 pass, 0.2 pass, 0.5 pass, 0.75 pass, 1 fail, 0.9 fail/pass, 0.85 pass, 0.95 fail
         with height  843. Use length 1203.9	# 100, 105
         with height  703. Use length 1004.0
         with height  560. Use length  799.8
         with height  422. Use length  602.7

angle: 40
         with height 1003. Use length 1195.3	# tip over at start, even with accelx = 0.02
         with height  843. Use length 1004.6
         with height  703. Use length  837.8
         with height  560. Use length  667.4
         with height  422. Use length  502.9

angle: 45
         with height 1003. Use length 1003.0
         with height  843. Use length  843.0
         with height  703. Use length  703.0
         with height  560. Use length  560.0

angle: 50
         with height 1003. Use length  841.6
         with height  843. Use length  707.4
         with height  703. Use length  589.9

angle: 55
         with height 1003. Use length  702.3
         with height  843. Use length  590.3

angle: 60
         with height 1003. Use length  579.1

angle: 65
"""
