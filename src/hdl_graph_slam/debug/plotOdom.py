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
			point = [str(i) for i in data]
			points.append(point)
	points = np.array(points)

	print(points.shape)

	plt.plot(points[:,0],points[:,1],'r',label='register_odom')
	plt.plot(points[0,0],points[0,1],'r*')

	plt.plot(points[:,3],points[:,4],'b--',label='optimize_odom')
	plt.plot(points[0,3],points[0,4],'b*')
	
	plt.plot(points[:,6],points[:,7],'k',label='gps_odom')
	plt.plot(points[0,6],points[0,7],'k*')
	
	plt.axis('equal')
	plt.grid('on')
	
	plt.legend()
	plt.show()
	
if __name__=='__main__':
	if(len(sys.argv) <2):
		print("please input file name!!!")
		exit()
	main(sys.argv)
