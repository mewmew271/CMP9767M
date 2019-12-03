#/launch 
#roslaunch uol_cmp9767m_base thorvald-sim.launch
#rviz #visulization of data requires map and robot model 
#roslaunch uol_cmp9767m_tutorial move_base.launch
#roslaunch uol_cmp9767m_tutorial amcl.launch



coordsFound = [[8.6, 8], [-8, -8]]
coordsBeen = []
print(coordsFound)

objectCoords = [8.9, 8]#found coords 

#print(all(map(coordsStack.__contains__, objectCoords)))

if objectCoords not in coordsFound or coordsBeen:
	coordsFound.append(objectCoords)
	

	print(coordsFound)

	#when found/moved to 
	coordsBeen.append(coordsFound[0])
	coordsFound.pop(0)
	print(coordsFound)
	print(coordsBeen)
#for c in coordsStack:
	#if c == objectCoords:
		#lock = 1
		#break
#if lock == 0: 
	#coordsStack.append(objectCoords)





