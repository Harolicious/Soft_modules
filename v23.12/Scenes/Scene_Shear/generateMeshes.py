#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 26 14:02:25 2023

@author: lab
"""

import gmsh
import numpy as np
import Constants
from defineMeshSizes import defineMeshSizes
from meshembed import meshembed

gmsh.initialize()
 
LadoCubo = Constants.LadoCubo
AlturaCilindro_s = Constants.AlturaCilindro_s_s
RadioCilindro_s = Constants.RadioCilindro_s_s


BoxTag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox = (3, BoxTag)

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro_s)/2,0,0, AlturaCilindro_s, 0 , RadioCilindro_s, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
DimTagcylinder = gmsh.model.occ.rotate([DimTagCylinder], 0, LadoCubo/2, 0, 0, 0, 1, -np.pi/4)

Cutout = gmsh.model.occ.cut([DimTagBox], [DimTagCylinder])

meshembed(LadoCubo, 1, 0.01, BoxTag)

gmsh.model.occ.synchronize()

defineMeshSizes(2.5)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("CubitoShear.vtk")
gmsh.fltk.run()

gmsh.clear()

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro_s)/2,0, 0, AlturaCilindro_s,0, RadioCilindro_s, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
DimTagcylinder = gmsh.model.occ.rotate([DimTagCylinder], 0, LadoCubo/2, 0, 0, 0, 1, -np.pi/4)


gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("CubitoShear_Cavity.stl")
gmsh.fltk.run()

# Visual model

Box1Tag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox1 = (3, Box1Tag)


gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()

gmsh.write("Cubito_Shear_visu.stl")

gmsh.clear()
gmsh.finalize()