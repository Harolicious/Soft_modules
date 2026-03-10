#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 26 14:02:25 2023

@author: stefan
"""

import gmsh

gmsh.initialize()

gmsh.merge("cubito_estirar_limpio_ansys.STEP")

gmsh.model.mesh.generate(3)
gmsh.model.mesh.refine()
gmsh.write("CubitoEstirar.vtk")
gmsh.fltk.run()

gmsh.clear()

gmsh.merge("Cavity.step")

gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
# gmsh.model.mesh.refine()
gmsh.write("CubitoEstirar_Cavity.stl")
gmsh.fltk.run()
