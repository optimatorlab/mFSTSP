'''
Created on April 2, 2014

@author: Jacob Conaway
@author: David Jones 
'''

import csv

def parseCSVstring(filename, returnJagged=False, fillerValue=-1, delimiter=',', commentChar='%'):
	with open(filename, "U") as csvfile:
		csvFile = csvfile.readlines()
		matrix = []
		maxSize = 0

		for line in csvFile:
			if(line.startswith(commentChar)):
				# Got to next line
				continue

			row = map(str, filter(None, line.rstrip().split(delimiter)))
			matrix.append(row)
			if (len(row) > maxSize):
				maxSize = len(row)

		if(not(returnJagged)):
			for row in matrix:    
				row += [fillerValue] * (maxSize - len(row))

		#if (len(matrix) == 1):
			# This is a vector, just return a 1-D vector
			#matrix = matrix[0]			

		return matrix
        

def printMatrix(matrix):
	for row in matrix:
		for cell in row:
			print cell,