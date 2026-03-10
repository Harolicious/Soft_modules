#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 30 14:52:11 2024

@author: lab
"""

import gmsh

def meshembed(a, b, tol, vol, y_offset=0):

    # ----- Plano inferior -----
    y1 = b + y_offset

    p1 = gmsh.model.occ.addPoint(-a/2 + tol, y1, a/2 - tol)
    p2 = gmsh.model.occ.addPoint(a/2 - tol, y1, a/2 - tol)
    p3 = gmsh.model.occ.addPoint(a/2 - tol, y1, -a/2 + tol)
    p4 = gmsh.model.occ.addPoint(-a/2 + tol, y1, -a/2 + tol)

    l1 = gmsh.model.occ.addLine(p1, p2)
    l2 = gmsh.model.occ.addLine(p2, p3)
    l3 = gmsh.model.occ.addLine(p3, p4)
    l4 = gmsh.model.occ.addLine(p4, p1)
    loop1 = gmsh.model.occ.addCurveLoop([l1, l2, l3, l4])
    plane1 = gmsh.model.occ.addPlaneSurface([loop1])

    # ----- Plano superior -----
    y2 = (a - b) + y_offset

    p5 = gmsh.model.occ.addPoint(-a/2 + tol, y2, a/2 - tol)
    p6 = gmsh.model.occ.addPoint(a/2 - tol, y2, a/2 - tol)
    p7 = gmsh.model.occ.addPoint(a/2 - tol, y2, -a/2 + tol)
    p8 = gmsh.model.occ.addPoint(-a/2 + tol, y2, -a/2 + tol)

    l5 = gmsh.model.occ.addLine(p5, p6)
    l6 = gmsh.model.occ.addLine(p6, p7)
    l7 = gmsh.model.occ.addLine(p7, p8)
    l8 = gmsh.model.occ.addLine(p8, p5)
    loop2 = gmsh.model.occ.addCurveLoop([l5, l6, l7, l8])
    plane2 = gmsh.model.occ.addPlaneSurface([loop2])

    gmsh.model.occ.synchronize()

    gmsh.model.mesh.embed(2, [plane1, plane2], 3, vol)

    return
