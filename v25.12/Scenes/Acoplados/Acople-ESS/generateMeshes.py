#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 15:18:54 2024

@author: lab_Harold
"""

import Constants
import numpy as np
import gmsh
from defineMeshSizes import *
from meshembed import *

gmsh.initialize()

LadoCubo = Constants.LadoCubo
AlturaCilindro = Constants.AlturaCilindro
RadioCilindro = Constants.RadioCilindro
AlturaCilindroShear = Constants.AlturaCilindroShear
RadioCilindroShear = Constants.RadioCilindroShear

#Estirar 1

BoxTag1 = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox1 = (3, BoxTag1)

Cylinder1Tag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0,0, AlturaCilindro, 0 , RadioCilindro, angle= 2*np.pi)
DimTagCylinder1 = (3, Cylinder1Tag)


# Cutout1 = gmsh.model.occ.cut([DimTagBox1], [DimTagCylinder1])

# Shear 2
BoxTag2 = gmsh.model.occ.addBox(-LadoCubo/2,LadoCubo,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox2 = (3, BoxTag2)

CylinderTag2 = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindroShear)/2 + LadoCubo,0,0, AlturaCilindroShear, 0 , RadioCilindroShear, angle= 2*np.pi)
DimTagCylinder2 = (3, CylinderTag2)
DimTagcylinder2 = gmsh.model.occ.rotate([DimTagCylinder2], 0, 3*LadoCubo/2, 0, 0, 0, 1, -np.pi/4)


# Cutout2 = gmsh.model.occ.cut([DimTagBox2], [DimTagCylinder2])

# Shear 3
BoxTag3 = gmsh.model.occ.addBox(-LadoCubo/2,2*LadoCubo,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox3 = (3, BoxTag3)

CylinderTag3 = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindroShear)/2 + 2*LadoCubo,0,0, AlturaCilindroShear, 0 , RadioCilindroShear, angle= 2*np.pi)
DimTagCylinder3 = (3, CylinderTag3)
DimTagcylinder3 = gmsh.model.occ.rotate([DimTagCylinder3], 0, 5*LadoCubo/2, 0, 1, 0, 0, -np.pi/4)



# Cutout3 = gmsh.model.occ.cut([DimTagBox3], [DimTagCylinder3])


FusionOut= gmsh.model.occ.fuse([(DimTagBox1),(DimTagBox3)], [(DimTagBox2)])
# EnsambleDimTag = FusionOut[0][0]
EnsambleDimTags = FusionOut[0]

Cutout = gmsh.model.occ.cut(EnsambleDimTags, [DimTagCylinder1,DimTagCylinder2,DimTagCylinder3])

# meshembed(LadoCubo, 1, 1, 0.01, Cutout[0][0][1])
# meshembed(LadoCubo, 1, 2, 0.01, Cutout[0][0][1])
# meshembed(LadoCubo, 1, 3, 0.01, Cutout[0][0][1])

#Cutout1 = gmsh.model.occ.cut([DimTagBox1], [DimTagCylinder1])
#Cutout2 = gmsh.model.occ.cut([DimTagBox2], [DimTagCylinder2])

gmsh.model.occ.synchronize()
defineMeshSizes(3)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("CubitoAcoplex3.vtk")
gmsh.fltk.run()
gmsh.clear()

### Cavidad

# Estirar 1
CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
# Shear 2
CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindroShear)/2 + 1*LadoCubo,0,0, AlturaCilindroShear, 0 , RadioCilindroShear, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
DimTagcylinder = gmsh.model.occ.rotate([DimTagCylinder], 0, 3*LadoCubo/2, 0, 0, 0, 1, -np.pi/4)
# Shear 3
CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindroShear)/2 + 2*LadoCubo,0,0, AlturaCilindroShear, 0 , RadioCilindroShear, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
DimTagcylinder = gmsh.model.occ.rotate([DimTagCylinder], 0, 5*LadoCubo/2, 0, 1, 0, 0, -np.pi/4)


gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("CubitoESS_Cavityx3.stl")
gmsh.fltk.run()

gmsh.clear()

### Cavidad separado

# Estirar 1
CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("CubitoESS_Cavity_1.stl")
gmsh.fltk.run()

gmsh.clear()

### Cavidad separado

# Shear 2
CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindroShear)/2 + 1*LadoCubo,0,0, AlturaCilindroShear, 0 , RadioCilindroShear, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
DimTagcylinder = gmsh.model.occ.rotate([DimTagCylinder], 0, 3*LadoCubo/2, 0, 0, 0, 1, -np.pi/4)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("CubitoESS_Cavity_2.stl")
gmsh.fltk.run()

gmsh.clear()

### Cavidad separado

# Shear 3
CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindroShear)/2 + 2*LadoCubo,0,0, AlturaCilindroShear, 0 , RadioCilindroShear, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
DimTagcylinder = gmsh.model.occ.rotate([DimTagCylinder], 0, 5*LadoCubo/2, 0, 1, 0, 0, -np.pi/4)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("CubitoESS_Cavity_3.stl")
gmsh.fltk.run()

gmsh.clear()

# Visual model

BoxTag1 = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox1 = (3, BoxTag1)

BoxTag2 = gmsh.model.occ.addBox(-LadoCubo/2,LadoCubo,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox2 = (3, BoxTag2)

BoxTag3 = gmsh.model.occ.addBox(-LadoCubo/2,2*LadoCubo,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox3 = (3, BoxTag3)

FusionOut = gmsh.model.occ.fuse([(DimTagBox1),(DimTagBox3)], [(DimTagBox2)])


gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()

gmsh.write("CubitoVisualx3.stl")

gmsh.clear()
gmsh.finalize()