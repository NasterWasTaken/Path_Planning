import copy
import math
import numpy as np
import sys
from numba import njit

class ConnectComponent(object):
	
	def compactLabeling(self, m, yNodes, xNodes,  zeroAsBg):
		image = m.ravel()

		self.label2d = np.zeros((xNodes, yNodes))
		self.MAX_LABELS = xNodes * yNodes
		self.rows = xNodes
		self.columns = yNodes
		# label first
		label, self.next_label = self.labeling(image, xNodes, yNodes, zeroAsBg,self.MAX_LABELS,self.uf_union,self.uf_find)
		stat = [0 for _ in range(self.next_label + 1)]
		y = np.bincount(label)
		stat+=y
		stat[0] = 0  # label 0 will be mapped to 0
		# whether 0 is background or not
		j = 1
		for i in range(len(stat)):
			if stat[i] != 0:
				stat[i] = j
				j += 1

		self.next_label = j - 1

		# return self.label2d
		label=stat[np.array(label).astype(int)]
		self.label2d=label.reshape(self.rows,self.columns) # type: ignore
  
	def getMaxLabel(self):
		return self.next_label

	@staticmethod
	@njit
	def labeling(image, xNodes, yNodes, zeroAsBg, MAX_LABELS, uf_union, uf_find):
		w = yNodes
		h = xNodes
		rst = [0 for _ in range(w * h)]
		parent = [0 for _ in range(MAX_LABELS)]
		labels = [0 for _ in range(MAX_LABELS)]
		# region label starts from 1
		# this is required as union-find data structure
		next_region = 1
		for y in range(h):
			for x in range(w):
				if image[y * w + x] == 0 and zeroAsBg: # type: ignore
					continue
				k = 0
				connected = False
				# if connected to the left
				if x > 0 and image[y * w + x - 1] == image[y * w + x]: # type: ignore
					k = rst[y * w + x - 1]
					connected = True
				# if connected to the up
				if y > 0 and image[(y - 1) * w + x] == image[y * w + x] and ( # type: ignore
						(not connected) or image[(y - 1) * w + x] < k): # type: ignore
					k = rst[(y - 1) * w + x]
					connected = True
				if not connected:
					k = next_region
					next_region += 1

				rst[y * w + x] = k
				# if connected, but with different label, then do union
				if x > 0 and image[y * w + x - 1] == image[y * w + x] and rst[y * w + x - 1] != k: # type: ignore
					uf_union(k, rst[y * w + x - 1], parent) # type: ignore
				if y > 0 and image[(y - 1) * w + x] == image[y * w + x] and rst[(y - 1) * w + x] != k: # type: ignore
					uf_union(k, rst[(y - 1) * w + x], parent)


		# Begin the second pass.  Assign the new labels
		# if 0 is reserved for background, then the first available label is 1
		next_label = 1
		for i in range(w*h):
			if image[i] != 0 or not zeroAsBg: # type: ignore
				rst[i] , next_label = uf_find(rst[i], parent, labels,next_label)
				# The labels are from 1, if label 0 should be considered, then
				# all the label should minus 1
				if not zeroAsBg:
					rst[i] -= 1

		next_label -= 1  # next_label records the max label
		if not zeroAsBg:
			next_label -= 1

		# System.out.println(next_label+" regions")

		return rst ,next_label

	@staticmethod
	@njit
	def uf_union(x, y, parent):
		
		while parent[x] > 0:
			
			x = parent[x]
			
		while parent[y] > 0:
			
			y = parent[y]
			
		if x != y:
			if x < y:
				parent[x] = y
			
			else:
				parent[y] = x
				
	@staticmethod
	@njit
	def uf_find(x, parent, label, next_label):
		
		while parent[x] > 0:
			
			x = parent[x]
			
		if label[x] == 0:
			
			label[x] = next_label
			next_label += 1
		
		return label[x], next_label

	def constructBinaryImages(self, robotsLabel):

		self.BinaryRobot = self.deepCopyMatrix(self.label2d)
		self.BinaryNonRobot = self.deepCopyMatrix(self.label2d)

		for i in range(self.rows):
			for j in range(self.columns):
				if self.label2d[i][j] == robotsLabel:
					self.BinaryRobot[i][j] = 1 # type: ignore
					self.BinaryNonRobot[i][j] = 0 # type: ignore
				elif self.label2d[i][j] != 0:
					self.BinaryRobot[i][j] = 0 # type: ignore
					self.BinaryNonRobot[i][j] = 1 # type: ignore
 
   
	def deepCopyMatrix(self, input):

		if input is None:
			
			return None

		result = [[] for _ in range(len(input))]
		
		for r in range(len(input)):
			
			result[r] = copy.copy(input[r])
		
		return result
	
	def NormalizedEuclideanDistanceBinary(self, RobotR):

			Region = np.zeros((self.rows, self.columns))

			f = [0.0 for _ in range(max(self.rows, self.columns))]
			d = [0 for _ in range(len(f))]
			v = [0 for _ in range(len(f))]
			z = [0 for _ in range(len(f) + 1)]

			for x in range(self.columns):
				for y in range(self.rows):
					if RobotR:
						f[y] = sys.float_info.max if self.BinaryRobot[y][x] == 0 else 0.0 # type: ignore
					else:
						f[y] = sys.float_info.max if self.BinaryNonRobot[y][x] == 0 else 0.0 # type: ignore


				self.DT1D(f, d, v, z)
				for y in range(self.rows):
					Region[y][x] = d[y]


			maxV = 0
			minV = sys.float_info.max
			for y in range(self.rows):
				self.DT1D(self.getVector(Region, y), d, v, z)

				for x in range(self.columns):
					Region[y][x] = float(math.sqrt(d[x]))
					if maxV < Region[y][x]:
						maxV = Region[y][x]
					if minV > Region[y][x]:
						minV = Region[y][x]


			# Normalization
			for y in range(self.rows):
				for x in range(self.columns):
					if RobotR:
						Region[y][x] = (Region[y][x] - minV) * (1 / (maxV - minV)) + 1
					else:
						Region[y][x] = (Region[y][x] - minV) * (1 / (maxV - minV))

			return Region
		
	def DT1D(self, f, d, v, z):
		k = 0
		v[0] = 0
		z[0] = -sys.float_info.max
		z[1] = sys.float_info.max

		for q in range(1, len(f)):
			s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])

			while s <= z[k]:
				k -= 1
				s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
			k += 1
			v[k] = q
			z[k] = s
			z[k + 1] = sys.float_info.max


		k = 0
		for q in range(len(f)):
			while z[k + 1] < q:
				k += 1

			d[q] = (q - v[k]) * (q - v[k]) + f[v[k]]

	def getVector(self, A, row):
		ret = [0 for _ in range(self.columns)]

		for i in range(self.columns):
			ret[i] = A[row][i]


		return ret
		
