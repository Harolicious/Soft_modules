# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 16:16:01 2023

@author: tu_sc
"""

# ---------- GenerateMeshes ----------
LadoCubo = 20 #mm
AlturaCilindro = 15
RadioCilindro = 7

# ---------- SPC ---------- 

Density = 20
LevelHeight = AlturaCilindro-2 #13.5
Repeat = 8

Diff = (AlturaCilindro - LevelHeight)/2
H = (LadoCubo-AlturaCilindro)/2 + Diff
