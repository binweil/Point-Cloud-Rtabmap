import os
import csv
def write_csv_file(fname):
	text_file = open(fname, "r")
	out_file = open("out_file.csv","w")
	with open(fname, 'r') as f:
		lines = f.readlines()
		#print(lines)
		lines_red = lines[11:]
		for lin in lines_red:
			mystring = lin.replace(" ", ",")
			print(mystring)
			out_file.write(mystring)

write_csv_file("rtabmap_cloud.pcd")

