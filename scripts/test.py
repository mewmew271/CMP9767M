#/launch 
#roslaunch uol_cmp9767m_base thorvald-sim.launch
#rviz #visulization of data requires map and robot model 
#roslaunch uol_cmp9767m_tutorial move_base.launch
#roslaunch uol_cmp9767m_tutorial amcl.launch

coordsTop = [[5.484, -3.784], [5.484, -2.785], [5.484, -0.761], [5.484, 0.233], [5.484, 2.188], [5.484, 3.231]] #add 
coordsBottom = [[-5.484, -3.784], [-5.484, -2.785], [-5.484, -0.761], [-5.484, 0.233], [-5.484, 2.188], [-5.484, 3.231]]
currentLocation = [-6, -6]#get from odom 
if abs((coordsTop[0][0] - currentLocation[0]) + (coordsTop[0][1] - currentLocation[1])) < abs((coordsBottom[0][0] - currentLocation[0]) + (coordsBottom[0][1] - currentLocation[1])):
	get_to  = coordsTop[0]
	
else:
	get_to  = coordsBottom[0]

coordsTop.pop(0)
coordsBottom.pop(0)
print 'get_to', get_to






#coordsFound = [[8.6, 8], [-8, -8]]
#coordsBeen = []
#print(coordsFound)

#objectCoords = [8.9, 8]#found coords 

#print(all(map(coordsStack.__contains__, objectCoords)))

#if objectCoords not in coordsFound or coordsBeen:
	#coordsFound.append(objectCoords)
	
	#print(coordsFound)
	##when found/moved to 
	#coordsBeen.append(coordsFound[0])
	#coordsFound.pop(0)
	#print(coordsFound)
	#print(coordsBeen)