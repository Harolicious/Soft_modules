#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Harold
"""
import Constants
import math
import numpy as np
import gmsh
from defineMeshSizes import defineMeshSizes
from meshembed import meshembed

gmsh.initialize()

LadoCubo = Constants.LadoCubo
AlturaCilindro = Constants.AlturaCilindro
RadioCilindro = Constants.RadioCilindro
AlturaCilindro_sensor = Constants.AlturaCilindro_sensor
RadioCilindro_sensor = Constants.RadioCilindro_sensor


################ Sensor stl ##################

gmsh.merge("Sensor_Galinstan_-45_y.STEP")

gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(3)

egmsh.write("Sensor_galinstan_-45_y.vtk")
gmsh.fltk.run()
gmsh.clear()

################ Sensor stl ##################

gmsh.merge("Sensor_Galinstan_-45_cavity_y.STEP")

gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(3)

gmsh.write("Sensor_galinstan_-45_cavity_y.stl")
gmsh.fltk.run()
gmsh.clear()

################3 cubito Rotador #################

BoxTag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox = (3, BoxTag)

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro_sensor)/2,0,0, AlturaCilindro_sensor, 0 , RadioCilindro_sensor, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

Cutout = gmsh.model.occ.cut([DimTagBox], [DimTagCylinder])

meshembed(LadoCubo, 1, 0.001, BoxTag)

gmsh.model.occ.synchronize()

defineMeshSizes(4)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("Cubitorotador.vtk")
gmsh.fltk.run()
gmsh.clear()

############# Cavidad filamentos ###########

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2) 
gmsh.model.mesh.refine()
gmsh.write("Cubitorotador_Cavity.stl")
gmsh.fltk.run()
gmsh.clear()

############# Cavidad ###########

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro_sensor)/2,0, 0, AlturaCilindro_sensor,0, RadioCilindro_sensor, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2) 
gmsh.model.mesh.refine()
gmsh.write("Cubitorotador_Cavity_sensor.stl")
gmsh.fltk.run()
gmsh.clear()

############# Visual model

Box1Tag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox1 = (3, Box1Tag)

gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()

gmsh.write("Cubito_rotador_visu.stl")
gmsh.fltk.run()
gmsh.clear()


gmsh.finalize()