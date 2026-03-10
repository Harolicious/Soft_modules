#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 15:39:45 2024

@author: lab-Harold
"""

import Sofa.Core
import Constants
import os
import csv
import numpy as np

PSI1 = 6
PSI2 = 4.5  
despla = 4.5 #desplazamiento deseado 

LadoCubo = Constants.LadoCubo

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        # Control de animación
        self.animation_finished = False 
        self.Decreasing = False
        
        # Nodos
        self.RootNode = kwargs['RootNode']
        self.SPC1 = kwargs['SPC1']
        self.SPC2 = kwargs['SPC2']
        self.EndEffectorMO = kwargs['EndEffectorMO']

        # PSI independientes
        self.PSI1 = kwargs['PSI1']
        self.PSI2 = kwargs['PSI2']

        # Presión máxima (Pa)
        self.Maxpressure1 = 6.89 * self.PSI1
        self.Maxpressure2 = 6.89 * self.PSI2

        # Incrementos independientes
        self.Increment1 = self.Maxpressure1 / 500
        self.Increment2 = self.Maxpressure2 / 500

        # Presiones actuales
        self.Pressure1 = 0
        self.Pressure2 = 0

        # Archivo CSV
        self.csv_file_path = "end_effector_data_ER_YMA.csv"

        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time","Pressure1","Pressure2",
                                 "Position_X","Position_Y","Position_Z","Angle"])
        
        print('Finished Init')
        

    def save_end_effector_data(self, time):
        position = self.EndEffectorMO.position.value
        
        PointA = position[0]
        PointB = position[1]
        
        delta_x = PointB[0] - PointA[0]
        delta_y = PointB[1] - PointA[1]
        delta_z = PointB[2] - PointA[2]
        
        Angle = np.rad2deg(np.arctan2(delta_y, delta_x))
        
        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    time,
                    self.Pressure1,
                    self.Pressure2,
                    PointA[0],
                    PointA[1],
                    PointA[2],
                    Angle
                ])
        except Exception as e:
            print(f"Error al escribir en archivo csv: {e}")
            

    def update_pressure_increase(self):
        self.Pressure1 += self.Increment1
        self.Pressure2 += self.Increment2

        if self.Pressure1 > self.Maxpressure1:
            self.Pressure1 = self.Maxpressure1

        if self.Pressure2 > self.Maxpressure2:
            self.Pressure2 = self.Maxpressure2

        self.SPC1.value.value = [self.Pressure1]
        self.SPC2.value.value = [self.Pressure2]


    def update_pressure_decrease(self):
        self.Pressure1 -= self.Increment1
        self.Pressure2 -= self.Increment2

        if self.Pressure1 < 0:
            self.Pressure1 = 0

        if self.Pressure2 < 0:
            self.Pressure2 = 0

        self.SPC1.value.value = [self.Pressure1]
        self.SPC2.value.value = [self.Pressure2]



    def onAnimateBeginEvent(self, eventType):

        current_time = self.RootNode.time.value

        # Terminar animación
        if self.animation_finished:
            self.RootNode.dt = 0
            return
        
        # Guardar datos
        self.save_end_effector_data(current_time)

        # Fase de incremento
        if not self.Decreasing:
            self.update_pressure_increase()

            if (self.Pressure1 >= self.Maxpressure1 and
                self.Pressure2 >= self.Maxpressure2):
                self.Decreasing = True
        
        # Fase de decremento
        else:
            self.update_pressure_decrease()

            if (self.Pressure1 <= 0 and
                self.Pressure2 <= 0):
                self.animation_finished = True



def createScene(rootNode):

                rootNode.addObject(
                    "RequiredPlugin",
                    pluginName="""SofaPython3
                    SoftRobots
                    SoftRobots.Inverse
                    Sofa.Component.AnimationLoop
                    Sofa.Component.Constraint.Lagrangian.Correction
                    Sofa.Component.Constraint.Lagrangian.Solver
                    Sofa.Component.Engine.Select
                    Sofa.Component.IO.Mesh
                    Sofa.Component.LinearSolver.Direct
                    Sofa.Component.LinearSolver.Iterative
                    Sofa.Component.Mapping.Linear
                    Sofa.Component.Mapping.MappedMatrix
                    Sofa.Component.Mapping.NonLinear
                    Sofa.Component.Mass
                    Sofa.Component.ODESolver.Backward
                    Sofa.Component.Setting
                    Sofa.Component.SolidMechanics.FEM.Elastic
                    Sofa.Component.SolidMechanics.Spring
                    Sofa.Component.StateContainer
                    Sofa.Component.Topology.Container.Constant
                    Sofa.Component.Collision.Geometry
                    Sofa.Component.Topology.Container.Dynamic
                    Sofa.Component.Visual
                    Sofa.GL.Component.Rendering3D
                    Sofa.GL.Component.Shader"""
                )
                
                rootNode.addObject(
                    "VisualStyle",
                    displayFlags="""
                        hideWireframe
                        showBehaviorModels
                        hideCollisionModels
                        hideBoundingCollisionModels
                        showForceFields
                        showInteractionForceFields""",
                )
                
                # rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
                rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Mapping') # Needed to use components [Tetra2TriangleTopologicalMapping]
                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.00001)
                rootNode.dt = 0.01 
                              
		#cubito
        
                cubito = rootNode.addChild('cubito')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner')
                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioners='preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='CubitoRotadorx2.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', translation="0 0 0" ,showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                              
                boxROIStiffness1 = cubito.addObject('BoxROI', name='boxROIStiffness1', box=[-14, 17, -14,  14, 23, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIStiffness2 = cubito.addObject('BoxROI', name='boxROIStiffness2', box=[-14, LadoCubo+17, -14,  14, LadoCubo+22, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
         
                boxROIMain1 = cubito.addObject('BoxROI', name='boxROIMain1', box=[-13, 0, -13,  13, 22, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain2 = cubito.addObject('BoxROI', name='boxROIMain2', box=[-13, 18, -13,  13, 41, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")

                Loader.init()
                Container.init()
                MO.init()
                boxROIStiffness1.init()
                boxROIMain1.init()
                boxROIStiffness2.init()
                boxROIMain2.init()
                
                
                YM_base1 = 11186 #7648.08
                YM_stiffROI1 = 11000 * 100
                YM_base2 = 5420 #5419.85
                YM_stiffROI2 = 6000 * 100
               
                modelStiff1 = cubito.addChild('modelStiff1')
                modelStiff1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness1.tetrahedraInROI", name='container')
                modelStiff1.addObject('TetrahedronFEMForceField', template = 'Vec3d', name='FEM_stiff1', method='large', poissonRatio=0.45, youngModulus=YM_stiffROI1) 
                
                modelSubTopo1 = cubito.addChild('modelSubTopo1')
                modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain1.tetrahedraInROI", name='container')
                modelSubTopo1.addObject('TetrahedronFEMForceField', template='Vec3d',  name='FEM_main1', method='large', poissonRatio=0.45, youngModulus=YM_base1)
                             
                modelStiff2 = cubito.addChild('modelStiff2')
                modelStiff2.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness2.tetrahedraInROI", name='container')
                modelStiff2.addObject('TetrahedronFEMForceField', template = 'Vec3d', name='FEM_stiff2', method='large', poissonRatio=0.45, youngModulus=YM_stiffROI2) 
                
                modelSubTopo2 = cubito.addChild('modelSubTopo2')
                modelSubTopo2.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain2.tetrahedraInROI", name='container')
                modelSubTopo2.addObject('TetrahedronFEMForceField', template='Vec3d',  name='FEM_main2', method='large', poissonRatio=0.45, youngModulus=YM_base2)

                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-14,  -2, -14,  14, 2, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')   
                
                
        #cubito/fibers
      
        #Estirar

                FiberNode = cubito.addChild("FiberReinforcementNode")    
                Density = 30
                IncrementAngle = 2*np.pi/Density
                Radius = 8
                NLevels = 7
                LevelHeight = 2
                Points = []
                Edges = []
                for i in range(NLevels):
                    for j in range(0,30): 
                        Angle = j*IncrementAngle
                        Coords = [Radius*np.cos(Angle), 4+i*LevelHeight, Radius*np.sin(Angle)]
                        Points.append(Coords)
                        if j>=1:
                            Edges.append([i*Density+j-1,i*Density+j])
                            if j==Density-1:
                                Edges.append([i*Density+j, i*Density+j-Density+1])
                
                FiberNode.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
       
        #Rotador                

                FiberNode = cubito.addChild("FiberReinforcement")
               
                Radius = Constants.RadioCilindro
                Displacement = Constants.LadoCubo
                BaseHeight = 2.5
                CylinderHeight = Constants.AlturaCilindro
               
                # Ajustes solicitados
                HelixStartOffset = 2.0
                HelixEndOffset = 2.0
               
                HelixStartY = Displacement+ BaseHeight + HelixStartOffset
                HelixEndY = Displacement + BaseHeight + CylinderHeight - HelixEndOffset
                HelixHeight = HelixEndY - HelixStartY
               
                Density = 12        # Discretización vertical
                Repeat = 8           # Número de fibras
                Turns = 0.3        # Número de vueltas helicoidales
               
                PointsHelix = []
                EdgesHelix = []
               
                dY = HelixHeight / (Density - 1)
                dTheta = 2 * np.pi * Turns / (Density - 1)
               
                for i in range(Density):
                    y = HelixStartY + i * dY
                    theta_i = i * dTheta
               
                    for j in range(Repeat):
                        idx = i * Repeat + j
                        theta = theta_i + j * (2 * np.pi / Repeat)
               
                        x = Radius * np.cos(theta)
                        z = Radius * np.sin(theta)
               
                        PointsHelix.append([x, y, z])
               
                        # Conexión vertical (helicoidal)
                        if i < Density - 1:
                            EdgesHelix.append([idx, idx + Repeat])
               
                FiberNode.addObject("Mesh", name="HelicalMesh", position=PointsHelix, edges=EdgesHelix)                
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=7e5)
                FiberNode.addObject("BarycentricMapping")
               
                RingNode = cubito.addChild("RingReinforcement")
               
                DensityRing =16
                dAngle = 2 * np.pi / DensityRing
               
                RingHeights = [HelixStartY, HelixEndY]
               
                PointsRing = []
                EdgesRing = []
               
                for level, y in enumerate(RingHeights):
                    for j in range(DensityRing):
                        idx = level * DensityRing + j
                        angle = j * dAngle
               
                        x = Radius * np.cos(angle)
                        z = Radius * np.sin(angle)
               
                        PointsRing.append([x, y, z])
                
                        # Cierre circular
                        next_j = (j + 1) % DensityRing
                        EdgesRing.append([idx, level * DensityRing + next_j])
               
                RingNode.addObject("Mesh", name="RingMesh", position=PointsRing, edges=EdgesRing)                
                RingNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                RingNode.addObject("MeshSpringForceField", linesStiffness=7e6)
                RingNode.addObject("BarycentricMapping")


  
                
                
        # Punto "End-effector"
                
                EndEffectorNode = cubito.addChild("EndEffectorNode")
                EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,2*LadoCubo,0], [10,2*LadoCubo,0], [-10,2*LadoCubo,0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")
                
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[4, 2*LadoCubo+despla , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)                
                
                
		#cubito/cavity
                cavity = cubito.addChild('cavity1')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubito_Cavity01.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPC1 = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

	
                cavity = cubito.addChild('cavity2')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubito_Cavity02.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')           
                SPC2 = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)



		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="CubitoVisual.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC1=SPC1,SPC2=SPC2, 
                                              EndEffectorMO=EndEffectorMO, PSI1=PSI1, PSI2=PSI2))
               

                return rootNode