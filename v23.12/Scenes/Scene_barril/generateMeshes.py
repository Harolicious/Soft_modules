#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 26 14:02:25 2023


@author: tu_sc
"""

import Constants
import numpy as np
import gmsh
from defineMeshSizes import defineMeshSizes
from meshembed import meshembed

gmsh.initialize()
    

LadoCubo = Constants.LadoCubo
AlturaCilindro = Constants.AlturaCilindro
RadioCilindro = Constants.RadioCilindro

BoxTag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox = (3, BoxTag)

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0,0, AlturaCilindro, 0 , RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

Cutout = gmsh.model.occ.cut([DimTagBox], [DimTagCylinder])


meshembed(LadoCubo, 1, 0.1, BoxTag)


gmsh.model.occ.synchronize()

defineMeshSizes(2.5)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("Cubitobarril.vtk")
gmsh.fltk.run()

gmsh.clear()

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("Cubitobarril_Cavity.stl")
gmsh.fltk.run()

# Visual model

Box1Tag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox1 = (3, Box1Tag)


gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()


gmsh.write("Cubito_barril_visu.stl")

gmsh.clear()
gmsh.finalize()