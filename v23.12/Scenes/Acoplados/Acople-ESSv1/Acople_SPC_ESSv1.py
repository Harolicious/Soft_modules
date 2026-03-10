#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 15:39:45 2024

@author: lab_Harold
"""

import Sofa.Core
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

PSI1 = 4
PSI2 = 4
PSI3 = 6.5
Despla = 4        

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
        self.SPC3 = kwargs['SPC3']
        self.EndEffectorMO = kwargs['EndEffectorMO']

        # PSI independientes
        self.PSI1 = kwargs['PSI1']
        self.PSI2 = kwargs['PSI2']
        self.PSI3 = kwargs['PSI3']

        # Presión máxima (Pa)
        self.Maxpressure1 = 6.89 * self.PSI1
        self.Maxpressure2 = 6.89 * self.PSI2
        self.Maxpressure3 = 6.89 * self.PSI3

        # Incrementos independientes
        self.Increment1 = self.Maxpressure1 / 500
        self.Increment2 = self.Maxpressure2 / 500
        self.Increment3 = self.Maxpressure3 / 500

        # Presiones actuales
        self.Pressure1 = 0
        self.Pressure2 = 0
        self.Pressure3 = 0

        # Archivo CSV
        self.csv_file_path = "end_effector_data_ESS_YMA.csv"

        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time","Pressure1","Pressure2", "Pressure3", 
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
                    self.Pressure3,
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
        self.Pressure3 += self.Increment3

        if self.Pressure1 > self.Maxpressure1:
            self.Pressure1 = self.Maxpressure1

        if self.Pressure2 > self.Maxpressure2:
            self.Pressure2 = self.Maxpressure2
            
        if self.Pressure3 > self.Maxpressure3:
            self.Pressure3 = self.Maxpressure3

        self.SPC1.value.value = [self.Pressure1]
        self.SPC2.value.value = [self.Pressure2]
        self.SPC3.value.value = [self.Pressure3]

    def update_pressure_decrease(self):
        self.Pressure1 -= self.Increment1
        self.Pressure2 -= self.Increment2
        self.Pressure3 -= self.Increment3
        
        if self.Pressure1 < 0:
            self.Pressure1 = 0

        if self.Pressure2 < 0:
            self.Pressure2 = 0

        if self.Pressure3 < 0:
            self.Pressure3 = 0
            
        self.SPC1.value.value = [self.Pressure1]
        self.SPC2.value.value = [self.Pressure2]
        self.SPC3.value.value = [self.Pressure3]


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
                self.Pressure2 >= self.Maxpressure2 and
                self.Pressure3 >= self.Maxpressure3):
                self.Decreasing = True
        
        # Fase de decremento
        else:
            self.update_pressure_decrease()

            if (self.Pressure1 <= 0 and
                self.Pressure2 <= 0 and
                self.Pressure3 <= 0):
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
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.0000001)
                
              
                
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
                              
                boxROIStiffness1 = cubito.addObject('BoxROI', name='boxROIStiffness1', box=[-14, 37, -14,  14, 41, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIStiffness2 = cubito.addObject('BoxROI', name='boxROIStiffness2', box=[-14, 18, -14,  14, 22, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIStiffness3 = cubito.addObject('BoxROI', name='boxROIStiffness3', box=[-14, 56, -14,  14, 61, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                
                boxROIMain1 = cubito.addObject('BoxROI', name='boxROIMain1', box=[-13, 0, -13,  13, 22, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain2 = cubito.addObject('BoxROI', name='boxROIMain2', box=[-13, 18, -13,  13, 41, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain3 = cubito.addObject('BoxROI', name='boxROIMain3', box=[-13, 36, -13,  13, 60, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")

                
                Loader.init()
                Container.init()
                MO.init()
                boxROIStiffness1.init()
                boxROIMain1.init()
                boxROIStiffness2.init()
                boxROIMain2.init()
                boxROIStiffness3.init()
                boxROIMain3.init()
                
                
                YM_base1 = 11186 
                YM_stiffROI1 = 11000 * 100
                YM_base2 = 4620 
                YM_stiffROI2 = 8000 * 100
                YM_base3 = 4620 
                YM_stiffROI3 = 8000 * 100
               
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

                modelStiff3 = cubito.addChild('modelStiff3')
                modelStiff3.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness3.tetrahedraInROI", name='container')
                modelStiff3.addObject('TetrahedronFEMForceField', template = 'Vec3d', name='FEM_stiff3', method='large', poissonRatio=0.45, youngModulus=YM_stiffROI3) 
                
                modelSubTopo3 = cubito.addChild('modelSubTopo3')
                modelSubTopo3.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain3.tetrahedraInROI", name='container')
                modelSubTopo3.addObject('TetrahedronFEMForceField', template='Vec3d',  name='FEM_main3', method='large', poissonRatio=0.45, youngModulus=YM_base3)

                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-14,  -2, -14,  14, 2, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
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
                
#############Shear 
           
                def create_ring(points, edges, center, radius, y, density, R):
                
                    offset = len(points)
                    dtheta = 2 * np.pi / density
                
                    for i in range(density):
                        theta = i * dtheta
                
                        p = np.array([
                            center[0] + radius * np.cos(theta),
                            y,
                            center[2] + radius * np.sin(theta)
                        ])
                
                        p_rot = R @ (p - center) + center
                        points.append(p_rot.tolist())
                
                        edges.append([
                            offset + i,
                            offset + (i + 1) % density
                        ])
                
                
                def create_ring_with_diameters(points, edges, center, radius, y, ring_density,
                                               diameter_density, R):
                    offset_ring = len(points)
                    dtheta_ring = 2 * np.pi / ring_density
                
                    for i in range(ring_density):
                        theta = i * dtheta_ring
                
                        p = np.array([
                            center[0] + radius * np.cos(theta),
                            y,
                            center[2] + radius * np.sin(theta)
                        ])
                
                        p_rot = R @ (p - center) + center
                        points.append(p_rot.tolist())
                
                        edges.append([
                            offset_ring + i,
                            offset_ring + (i + 1) % ring_density
                        ])
                
                    dtheta = np.pi / diameter_density
                    ds = 2 * radius / diameter_density
                
                    for j in range(diameter_density):
                        theta = j * dtheta
                
                        prev_idx = None
                
                        for k in range(diameter_density + 1):
                            s = -radius + k * ds
                
                            p = np.array([
                                center[0] + s * np.cos(theta),
                                y,
                                center[2] + s * np.sin(theta)
                            ])
                
                            p_rot = R @ (p - center) + center
                            idx = len(points)
                            points.append(p_rot.tolist())
                
                            if prev_idx is not None:
                                edges.append([prev_idx, idx])
                
                            prev_idx = idx

#############Shear 

                Radius = RadioCilindroShear
                
                ring_density = 16          # nodos por anillo
                diameter_density = 8       # fibras diametrales en tapas
                n_rings = 6                # anillos a lo largo del cilindro
                
                y_start = 3
                y_end = 11
                
                stiffness_ring = 5e5
                stiffness_diameter = 5e5
                
                cube_center = np.array([55.5, 85.5, 0])
                
                # Rotación -45° z
                angle_rad = np.radians(-45)
                R = np.array([
                    [np.cos(angle_rad), -np.sin(angle_rad), 0],
                    [np.sin(angle_rad),  np.cos(angle_rad), 0],
                    [0,                  0,                 1]
                ])
                
                
                FiberBody = cubito.addChild("FiberBody1")
                
                PointsBody = []
                EdgesBody = []
                
                height = y_end - y_start
                
                for i in range(n_rings):
                    y = y_start + i * height / (n_rings - 1)
                
                    create_ring(points=PointsBody, edges=EdgesBody, center=cube_center,radius=Radius,
                                y=y, density=ring_density, R=R)
                
                FiberBody.addObject("Mesh", position=PointsBody, edges=EdgesBody)
                FiberBody.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                FiberBody.addObject("MeshSpringForceField",linesStiffness=stiffness_ring)
                FiberBody.addObject("BarycentricMapping")
                
                
                CapBottom = cubito.addChild("CapBottom1")
                
                PointsB = []
                EdgesB = []
                
                create_ring_with_diameters(points=PointsB, edges=EdgesB, center=cube_center,radius=Radius,
                                           y=y_start-2, ring_density=ring_density,diameter_density=diameter_density, R=R)
                
                CapBottom.addObject("Mesh", position=PointsB, edges=EdgesB)
                CapBottom.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                CapBottom.addObject("MeshSpringForceField", linesStiffness=stiffness_diameter)
                CapBottom.addObject("BarycentricMapping")
                
                CapTop = cubito.addChild("CapTop1")
                
                PointsT = []
                EdgesT = []
                
                create_ring_with_diameters(points=PointsT,edges=EdgesT,center=cube_center,radius=Radius,
                                           y=y_end+2, ring_density=ring_density, diameter_density=diameter_density,R=R)
                
                CapTop.addObject("Mesh", position=PointsT, edges=EdgesT)
                CapTop.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                CapTop.addObject("MeshSpringForceField",linesStiffness=stiffness_diameter)
                CapTop.addObject("BarycentricMapping")
                
#############Shear 

                Radius = RadioCilindroShear
                
                ring_density = 16          # nodos por anillo
                diameter_density = 8       # fibras diametrales en tapas
                n_rings = 6                # anillos a lo largo del cilindro
                
                y_start = 3
                y_end = 11
                
                stiffness_ring = 5e5
                stiffness_diameter = 5e5
                
                cube_center = np.array([0, 153.78, -103.78])        
                
                # Rotación -45° x
                angle_rad = np.radians(-45)
                R = np.array([
                    [1, 0                   , 0                 ],
                    [0, np.cos(angle_rad)   , -np.sin(angle_rad)],
                    [0, np.sin(angle_rad)   , np.cos(angle_rad) ]
                ])
                
               

                FiberBody = cubito.addChild("FiberBody2")
                
                PointsBody = []
                EdgesBody = []
                
                height = y_end - y_start
                
                for i in range(n_rings):
                    y = y_start + i * height / (n_rings - 1)
                
                    create_ring(points=PointsBody, edges=EdgesBody, center=cube_center,radius=Radius,
                                y=y, density=ring_density, R=R)
                
                FiberBody.addObject("Mesh", position=PointsBody, edges=EdgesBody)
                FiberBody.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                FiberBody.addObject("MeshSpringForceField",linesStiffness=stiffness_ring)
                FiberBody.addObject("BarycentricMapping")
                
                
                CapBottom = cubito.addChild("CapBottom2")
                
                PointsB = []
                EdgesB = []
                
                create_ring_with_diameters(points=PointsB, edges=EdgesB, center=cube_center,radius=Radius,
                                           y=y_start-2, ring_density=ring_density,diameter_density=diameter_density, R=R)
                
                CapBottom.addObject("Mesh", position=PointsB, edges=EdgesB)
                CapBottom.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                CapBottom.addObject("MeshSpringForceField", linesStiffness=stiffness_diameter)
                CapBottom.addObject("BarycentricMapping")
                
                CapTop = cubito.addChild("CapTop2")
                
                PointsT = []
                EdgesT = []
                
                create_ring_with_diameters(points=PointsT,edges=EdgesT,center=cube_center,radius=Radius,
                                           y=y_end+2, ring_density=ring_density, diameter_density=diameter_density,R=R)
                
                CapTop.addObject("Mesh", position=PointsT, edges=EdgesT)
                CapTop.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                CapTop.addObject("MeshSpringForceField",linesStiffness=stiffness_diameter)
                CapTop.addObject("BarycentricMapping")



		#cubito/cavity

        #cavidad 1
        
                cavity = cubito.addChild('cavity1')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoESS_Cavity_1.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                
                SPC1 = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

        #cavidad 2
        
                cavity = cubito.addChild('cavity2')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoESS_Cavity_2.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                
                SPC2 = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

        #cavidad 3
        
                cavity = cubito.addChild('cavity3')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoESS_Cavity_3.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
               
                SPC3 = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

        # Punto "End-effector"
                
                EndEffectorNode = cubito.addChild("EndEffectorNode")
                EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,3*LadoCubo,0], [10,3*LadoCubo,0], [-10,3*LadoCubo,0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")
                
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[4, 3*LadoCubo+Despla , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)                
                


		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="CubitoVisualx3.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC1=SPC1, SPC2=SPC2, SPC3=SPC3,
                                              EndEffectorMO=EndEffectorMO, PSI1=PSI1, PSI2=PSI2, PSI3=PSI3))


                return rootNode