# -*- coding: utf-8 -*-
"""
Created on Fri Mar  5 01:15:53 2021

@author: Alice
"""

infile = "./RobotAssemblyTemp_description/RobotAssemblyTempTemp.urdf"
outfile = "./RobotAssemblyTemp_description/RobotAssemblyTemp.urdf"

delete = "package://" + infile.split("/")[1] +  "/"
with open(infile) as fin, open(outfile, "w+") as fout:
    for line in fin:
        line = line.replace(delete, "")
        fout.write(line)
    fout.close()