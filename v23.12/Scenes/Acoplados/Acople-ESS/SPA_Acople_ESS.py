#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 15:39:45 2024

@author: lab_Harold
"""
import Sofa.Core
# import Sofa
import Constants
import os
import csv
import numpy as np

LadoCubo = Constants.LadoCubo
AlturaCilindro = Constants.AlturaCilindro
RadioCilindro = Constants.RadioCilindro
AlturaCilindroShear = Constants.AlturaCilindroShear
RadioCilindroShear = Constants.RadioCilindroShear

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'


# class Controller(Sofa.Core.Controller):   
    
#     def __init__(self, *args, **kwargs):
#         super().__init__(*args, **kwargs)
#         # Sofa.Core.Controller.__init__(self, *args, **kwargs)
#         print(" Python::__init__::" + str(self.name.value))
#         # Bandera para controlar la animacion
#         self.animation_finished = False 
        
#         self.RootNode = kwargs['RootNode']
#         self.SPA1 = kwargs['SPA1']
#         self.SPA2 = kwargs['SPA2']
#         self.SPA3 = kwargs['SPA3']
#         self.EndEffectorMO = kwargs['EndEffectorMO']
#         self.Increment = 0.5
#         self.Pressure1 = 0
#         self.Pressure2 = 0
#         self.Pressure3 = 0
#         self.Maxpressure = 60
#         self.Decreasing = False
#         self.csv_file_path = "end_effector_data.csv"

#         # Crear archivo CSV y escribir encabezados si no existe
#         if not os.path.exists(self.csv_file_path):
#             with open(self.csv_file_path, mode='w', newline='') as file:
#                 writer = csv.writer(file)
#                 writer.writerow(["Time", "Pressure_1","Pressure_2","Pressure_3","Position_X", "Position_Y" ,"Position_Z"])
                
#         print(kwargs['RootNode'])       
#         print('Finished Init')
        
#     def save_end_effector_data(self, time):
#         position = self.EndEffectorMO.position.value
#         # orientation = self.EndEffectorMO.orientation.value
        
#         with open(self.csv_file_path, mode='a', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow([time,self.Pressure1,self.Pressure2,self.Pressure3,position[0][0],position[0][1],position[0][2]])

#     def update_pressure_increase(self, pressure, SPA):
#         pressure += self.Increment
#         if pressure > self.Maxpressure and self.animation_finished==False:
#             pressure = self.Maxpressure
#         SPA.value.value = [pressure]
#         return pressure

#     def update_pressure_decrease(self, pressure, SPA):
#         pressure -= self.Increment
#         if pressure < 0 and self.animation_finished==False:
#             pressure = 0
#         SPA.value.value = [pressure]
#         return pressure

    
#     def onAnimateBeginEvent(self, eventType):
#         # print(f"Current End-Effector position: {self.EndEffectorMO.position.value}")        
#         # print(f"pressure 1: {self.Pressure1}")
#         # print(f"pressure 2: {self.Pressure2}")
#         # print(f"pressure 3: {self.Pressure3}")
        
#         # Aquí obtienes el tiempo actual de la simulación
#         current_time = self.RootNode.time.value
        
#         if not self.animation_finished:
#             # Guardar datos del EndEffector
#             self.save_end_effector_data(current_time)
    
#             if not self.Decreasing:
#                 # Incrementar presiones
#                 self.Pressure1 = self.update_pressure_increase(self.Pressure1, self.SPA1)
                
#                 if self.Pressure1 >= self.Maxpressure:
#                     self.Pressure2 = self.update_pressure_increase(self.Pressure2, self.SPA2)
                    
#                     if self.Pressure2 >= self.Maxpressure:
#                         self.Pressure3 = self.update_pressure_increase(self.Pressure3, self.SPA3)
                        
#                         if self.Pressure3 >= self.Maxpressure:
#                             # Cambiar a modo decremento
#                             self.Decreasing = True  
#             else:
#                 # Decrementar presiones
#                 self.Pressure1 = self.update_pressure_decrease(self.Pressure1, self.SPA1)
                
#                 if self.Pressure1 <= 0:
#                     self.Pressure2 = self.update_pressure_decrease(self.Pressure2, self.SPA2)
                    
#                     if self.Pressure2 <= 0:
#                         self.Pressure3 = self.update_pressure_decrease(self.Pressure3, self.SPA3)
                        
#                         if self.Pressure3 <= 0:
#                             # Cambiar a modo incremento
#                             self.Decreasing = False  
#                             self.animation_finished = True
#         else:
#             print("Fin animacion")

#         pass

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
                rootNode.addObject("QPInverseProblemSolver", printLog=1, epsilon=0.1, maxIterations=1000,tolerance=1e-5)
                rootNode.dt = 0.001 
                
		#cubito1
                cubito = rootNode.addChild('cubito')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner')
                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioners='preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='CubitoAcoplex3.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', translation="0 0 0" ,showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                              
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-13, 37, -13,  13, 41, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIStiffness2 = cubito.addObject('BoxROI', name='boxROIStiffness2', box=[-13, 18, -13,  13, 22, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIStiffness3 = cubito.addObject('BoxROI', name='boxROIStiffness3', box=[-13, 56, -13,  13, 61, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                
                Container.init()
                MO.init()
                boxROIStiffness.init()                
                boxROIStiffness2.init()
                boxROIStiffness3.init()
                YM1 = 180000
                YM2 = YM1*100
                YMArray = np.ones(len(Loader.tetras))*YM1
                IdxElementsInROI = np.array(boxROIStiffness.tetrahedronIndices.value)
                IdxElementsInROI2 = np.array(boxROIStiffness2.tetrahedronIndices.value)
                IdxElementsInROI3 = np.array(boxROIStiffness3.tetrahedronIndices.value)
                YMArray[IdxElementsInROI] = YM2
                YMArray[IdxElementsInROI2] = YM2
                YMArray[IdxElementsInROI3] = YM2
                
                print(f"len IdxElementsInROI: {len(IdxElementsInROI)}")
                print(f"Largo de YMArray1:{len(YMArray)}")
                print(f"len IdxElementsInROI2: {len(IdxElementsInROI2)}")
                print(f"Largo de YMArray2:{len(YMArray)}")
                print(f"len IdxElementsInROI3: {len(IdxElementsInROI3)}")
                print(f"Largo de YMArray3:{len(YMArray)}")
                #cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=180000)
                cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=YMArray.flatten().tolist())
                #cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM2', method='large', poissonRatio=0.3,  youngModulus=180000)
                
                #cubito.addObject('TetrahedronHyperelasticityFEMForceField', name="HyperElasticMaterial", materialName="MooneyRivlin", ParameterSet="48000 -1.5e5 3000")

                cubito.addObject('BoxROI', name='boxROI1', box=[-13, -1, -13,  13, 2, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")       
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI1.indices', stiffness=1e12)       
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                

		                
       
#################cubito/fibers
      
##############Estirar


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
                        Coords = [Radius*np.cos(Angle), 4 + i*LevelHeight, Radius*np.sin(Angle)]
                        Points.append(Coords)
                        if j>=1:
                            Edges.append([i*Density+j-1,i*Density+j])
                
                FiberNode.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e7)
                FiberNode.addObject("BarycentricMapping")
                
#############Shear 

                FiberNode = cubito.addChild("FiberReinforcementNode")    
                
                angle_rad = np.radians(315)
                rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad),0],
                                            [np.sin(angle_rad), np.cos(angle_rad), 0],
                                            [0, 0, 1]])
                
                
                Density = 20
                IncrementAngle = 2*np.pi/Density
                Radius = 6.5
                NLevels = 6
                LevelHeight = 2
                Rotated_points = []
                Edges = []
                
                for i in range(NLevels):
                    for j in range(Density): 
                        Angle = j*IncrementAngle
                        Coords = [Radius*np.cos(Angle), 3*LadoCubo/2 + 5 + i*LevelHeight, Radius*np.sin(Angle)]
                        # Points.append(Coords)
                        # Rotar las coordenadas
                        Rotated_coords = np.dot(rotation_matrix, Coords)
                        # Desplazamiento
                        Rotated_coords[0] += -(LadoCubo+RadioCilindro)
                        Rotated_coords[1] += 2
                        Rotated_coords[2] += 0 
                        # Agregar las coordenadas rotadas
                        Rotated_points.append(Rotated_coords.tolist())
                        if j>=1:
                            Edges.append([i*Density+j-1,i*Density+j])
                            
        
                        
                
                FiberNode.addObject("Mesh", position=Rotated_points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
                
#############Shear 

                FiberNode = cubito.addChild("FiberReinforcementNode")    
                
                angle_rad = np.radians(315)
                rotation_matrix = np.array([[1, 0, 0],
                                            [0, np.cos(angle_rad), -np.sin(angle_rad)],
                                            [0, np.sin(angle_rad), np.cos(angle_rad)]])
                
                
                Density = 20
                IncrementAngle = 2*np.pi/Density
                Radius = 6.5
                NLevels = 6
                LevelHeight = 2
                Rotated_points = []
                Edges = []
                
                for i in range(NLevels):
                    for j in range(Density): 
                        Angle = j*IncrementAngle
                        Coords = [Radius*np.cos(Angle), 5*LadoCubo/2 + 5+i*LevelHeight, Radius*np.sin(Angle)]
                        # Points.append(Coords)
                        # Rotar las coordenadas
                        Rotated_coords = np.dot(rotation_matrix, Coords)
                        # Desplazamieto 
                        Rotated_coords[0] += 0
                        Rotated_coords[1] += RadioCilindro
                        Rotated_coords[2] += 3*LadoCubo/2 + AlturaCilindroShear
                        # Agregar las coordenadas rotadas
                        Rotated_points.append(Rotated_coords.tolist())
                        if j>=1:
                            Edges.append([i*Density+j-1,i*Density+j])
                            
        
                        
                
                FiberNode.addObject("Mesh", position=Rotated_points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
		#cubito/cavity

        #cavidad 1
        
                cavity = cubito.addChild('cavity1')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoESS_Cavity_1.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                #cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=600, valueType=0)
                
                SPA1 = cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles')                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

        #cavidad 2
        
                cavity = cubito.addChild('cavity2')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoESS_Cavity_2.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                #cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=600, valueType=0)
                
                SPA2 = cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles')                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

        #cavidad 3
        
                cavity = cubito.addChild('cavity3')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoESS_Cavity_3.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                #cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=600, valueType=0)
                
                SPA3 = cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles')                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)


        # Effector
        # bunny/effector
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[0, 62, 2], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)
                goal.addObject('UncoupledConstraintCorrection')
                
        # Punto "End-effector"         
                effector = cubito.addChild('EffectorNode')
                effector.addObject('MechanicalObject', position=[0, 60, 0], showObject=True, showObjectScale=10)
                PositionEffector = effector.addObject('PositionEffector', indices=0, effectorGoal=goal.goalMO.position.linkpath)
                effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)


		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="CubitoVisualx3.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPA1=SPA1, SPA2=SPA2, SPA3=SPA3, EndEffectorMO=EndEffectorMO))


                return rootNode