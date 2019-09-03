#!/usr/bin/env python

#################################################################
# System:   Collaborative Robot Programming Interface           #
# File:     buildCRPI.py                                        #
# Revision: 1.0 1/10/2018                                       #
# Author:   M. Zimmerman                                        #
#                                                               #
# Description                                                   #
# ===========                                                   #
# This python file calls the necessary operations to make CRPI  #
# on a linux platform.                                          #
#                                                               #
# *save space for options*                                      #
#################################################################

import sys
import os


#################################################################
# Wrapper for os.system
# input: command to execute
# output: true if executed, else false
def bash(command):
	return os.system(command)

	
#################################################################
# Wrapper for os.chdir()
# input: directory to navigate to
# output: true if executed, else false
def cd(command):
	return os.chdir(command)

	
#################################################################
# Wrapper
# input: file to check
# output: true if file exists, else false
def exists(fileName):
	return os.path.isfile(fileName)
	

#################################################################
# Wrapper
# input: directory to check
# output: true if directory exists, else false
def directoryExists(path):
		return os.path.isdir(path)
		

		
def build(lib):
	cmd = "make " + lib
	return bash(cmd)
	
#fileSame
#Input: two strings of file names
#Output: Boolean True if file is the same, False if not
#Helper function for determining if two files match
#Didn't use linux commands out of convenience
#If something went wrong in the build a file is made "ulapiBuild.txt"
#that shows the lines where the test differs from the sample test
def fileSame(file1,file2):
	f1 = open(file1,'r')
	f2 = open(file2,'r')
	output = open("ulapiBuild.txt","w")
	flag = True
	for line1 in f1:
		line2 = f2.readline()
		if line1 != line2:
			output.write("Diff: "+ line1+ " -- " + line2)
			flag = False
			
	fl.close()
	f2.close()
	f3.close()
	return flag
	
#Following instructions taken from ulapi/README.Linux
def makeUlapi():
	cd("../ulapi")
	cmd = "tar xzvf ulapi-1.X.tar.gz"
	bash(cmd)
	cd("ulapi-1.X")
	cmd = "./configure"
	bash(cmd)
	cmd = "make"
	bash(cmd)
	cmd = "make install prefix=/usr/local/ulapi"
	
	#run test, forwarding output to installog.txt
	cmd = "bin/ultest>installog.txt"
	bash(cmd)
	cd("../")
	if not fileSame("installlog.txt","install_compare.txt","ulapiBuild.txt"):
		print("Something may have gone wrong with ULAPI, please refer to ULAPI's documentation")
		print("A log of where the tests differ is in ulapiBuild.txt")
		return
	return
	
#still in progess	
def makeAll():
	#cmd = "make all"
	#First we are making all of the standalone libraries, ULAPI should already be built
	
	#make math
	cd("Libraries/Math")
	cmd = "make"
	print("math")
	bash(cmd)
	
	#make Serial
	cd("../Serial")
	cmd = "make"
	bash(cmd)
	
	#make machine learning libraries-------
	#clustering
	cd("../../Clustering")
	cd("Patterns")
	cmd = "make"
	bash(cmd)
	cd("../Cluster")
	bash(cmd)
	cd("../kMeans")
	bash(cmd)
	
	#Neural
	cd("../../Neural/Neuron")
	bash(cmd)
	cd("../NeuralNet")
	bash(cmd)

	#Genetic Algorithm
	cd("../../GeneticAlgorithms")
	bash(cmd)
	#End machine learning libraries--------
	cd("CRPI/Libraries")
	
	#if ULAPI is not built, do this now.
	#makeUlapi()
	
	
	#make crpi
	cd("CRPI")
	cmd = "make"
	bash(cmd)
	
	#make motion prim libarary
	cd("../MotionPrims")
	cmd = "make"
	bash(cmd)
	
	#make mocap libary
	
	
	return bash(cmd)

def clean():
	cmd = "make clean"
	return bash(cmd)

def makePlus(dirs):
	flag = False
	cmd = "make all"
	flag = bash(cmd)
	for dir in dirs:
		cmd = "make " + dir
		flag = bash(cmd)
	return flag

def makeIndividual(dir):
	return build(lib)
	
def usage():
	print("Please refer to readme for instructions on usage")


def main(argv):
	
	#checking if any tags exist
	if len(argv) > 1 :
		if argv[1] == '-a':
			makeAll()
		elif argv[1] ==  '-c':
			clean()
		elif argv[1] == '-plus':
			if len(argv) > 2:
				for i in argv:
					if i != "buildCRPI.py" and i != "-plus":
						dirs += i
				makePlus(dirs)
			else:
				usage()
		else:
			usage()
	print("done")
	
	

if __name__ == "__main__":
    main(sys.argv)
