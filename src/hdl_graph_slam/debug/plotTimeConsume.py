import matplotlib.pyplot as plt 
import numpy as np
import sys

def main(argv):
	points = []
	with open(argv[1],'r') as f:
		line = f.readline()
		while True:
			line = f.readline()
			if not line:
				break
			data = line.split()
			point = [float(i) for i in data]
			points.append(point)
	points = np.array(points)
	print(points.shape)

	print np.sum(points,axis=0)

	
if __name__=='__main__':
	if(len(sys.argv) <2):
		print("please input file name!!!")
		exit()
	main(sys.argv)
