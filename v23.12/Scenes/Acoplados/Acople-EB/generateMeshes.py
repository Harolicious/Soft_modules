#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 26 14:02:25 2023

@author: lab_Harold
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

#Cubo con cavidad

Box1Tag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox1 = (3, Box1Tag)
meshembed(LadoCubo, 1, 0.01, Box1Tag, 0)

Cylinder1Tag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0,0, AlturaCilindro, 0 , RadioCilindro, angle= 2*np.pi)
DimTagCylinder1 = (3, Cylinder1Tag)


Box2Tag = gmsh.model.occ.addBox(-LadoCubo/2, LadoCubo,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox2 = (3, Box2Tag)
meshembed(LadoCubo, 1, 0.01, Box2Tag, LadoCubo)

Cylinder2Tag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2 + LadoCubo,0,0, AlturaCilindro, 0 , RadioCilindro, angle= 2*np.pi)
DimTagCylinder2 = (3, Cylinder2Tag)

FusionOut = gmsh.model.occ.fuse([(DimTagBox1)], [(DimTagBox2)])
EnsambleDimTag = FusionOut[0][0]
EnsambleDimTags = FusionOut[0]


Cutout1 = gmsh.model.occ.cut(EnsambleDimTags, [DimTagCylinder1,DimTagCylinder2])


#Cutout1 = gmsh.model.occ.cut([DimTagBox1], [DimTagCylinder1])
#Cutout2 = gmsh.model.occ.cut([DimTagBox2], [DimTagCylinder2])

gmsh.model.occ.synchronize()
defineMeshSizes(3)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("CubitoRotadorx2.vtk")
gmsh.fltk.run()

gmsh.clear()


CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("Cubito_Cavity01.stl")
gmsh.fltk.run()

gmsh.clear()

Cylinder2Tag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2 + LadoCubo,0,0, AlturaCilindro, 0 , RadioCilindro, angle= 2*np.pi)
DimTagCylinder2 = (3, Cylinder2Tag)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("Cubito_Cavity02.stl")
gmsh.fltk.run()

gmsh.clear()

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
Cylinder2Tag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2 + LadoCubo,0,0, AlturaCilindro, 0 , RadioCilindro, angle= 2*np.pi)
DimTagCylinder2 = (3, Cylinder2Tag)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("CubitoEB_Cavityx2.stl")
gmsh.fltk.run()

gmsh.clear()

# Visual model

Box1Tag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox1 = (3, Box1Tag)


Box2Tag = gmsh.model.occ.addBox(-LadoCubo/2, LadoCubo,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox2 = (3, Box2Tag)

FusionOut = gmsh.model.occ.fuse([(DimTagBox1)], [(DimTagBox2)])

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()

gmsh.write("CubitoVisual.stl")
