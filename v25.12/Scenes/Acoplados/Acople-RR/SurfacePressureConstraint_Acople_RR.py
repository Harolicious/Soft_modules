#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 15:39:45 2024

@author: lab-Harold
"""

import Sofa

import os
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'


class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        self.RootNode = kwargs['RootNode']
        self.SPC1 = kwargs['SPC1']
        self.SPC2 = kwargs['SPC2']
        self.Increment = 50
        self.Pressure = 0
        print(kwargs['RootNode'])
    
        
        print('Finished Init')
        
    def onAnimateBeginEvent(self, eventType):
        self.Pressure = self.Pressure + self.Increment
        if self.Pressure > 550 or self.Pressure < 0:
            # self.Pressure = 550
            self.Increment = -self.Increment
        self.SPC1.value.value = [self.Pressure]
        self.SPC2.value.value = [self.Pressure]
        
        pass
  

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
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.0000001)
                
              
                
		#cubito1
                cubito = rootNode.addChild('cubito')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner')
                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioners='preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='CubitoRotadorx2.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', translation="0 0 0" ,showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                              
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-13, 37, -13,  13, 41, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIStiffness2 = cubito.addObject('BoxROI', name='boxROIStiffness2', box=[-13, 18, -13,  13, 22, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                Container.init()
                MO.init()
                boxROIStiffness.init()                
                boxROIStiffness2.init()
                YM1 = 180000
                YM2 = YM1*100
                YMArray = np.ones(len(Loader.tetras))*YM1
                IdxElementsInROI = np.array(boxROIStiffness.tetrahedronIndices.value)
                IdxElementsInROI2 = np.array(boxROIStiffness2.tetrahedronIndices.value)
                YMArray[IdxElementsInROI] = YM2
                YMArray[IdxElementsInROI2] = YM2
                
                print(f"len IdxElementsInROI1: {len(IdxElementsInROI)}")
                print(f"Largo de YMArray1:{len(YMArray)}")
                print(f"len IdxElementsInROI2: {len(IdxElementsInROI2)}")
                #cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=180000)
                cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=YMArray.flatten().tolist())
                #cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM2', method='large', poissonRatio=0.3,  youngModulus=180000)
                
                #cubito.addObject('TetrahedronHyperelasticityFEMForceField', name="HyperElasticMaterial", materialName="MooneyRivlin", ParameterSet="48000 -1.5e5 3000")

                cubito.addObject('BoxROI', name='boxROI1', box=[-13, -1, -13,  13, 2, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")       
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI1.indices', stiffness=1e12)       
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                #cubito.addObject('UncoupledConstraintCorrection')
                
       #         cubito.addObject('AttachProjectiveConstraint', object1="@M1", object2="@M2", indices1="0 1 2", indices2="10 11 12", constraintFactor="1 1 1")
		                
       
#################cubito/fibers
       


                FiberNode = cubito.addChild("FiberReinforcementNode1")    
                Density = 20
                IncrementAngle = np.pi/Density
                Radius = 8
                Repeat = 5
                Deg = 2*np.pi/Repeat
                LevelHeight = 13.725
                Points = []
                Edges = []
                for i in range(Density):
                    for j in range (0,Repeat):
                        
                        Angle = i*IncrementAngle
                        Coords = [Radius*np.cos(Angle+Deg*j), 3.5+i/Density*LevelHeight, Radius*np.sin(Angle+Deg*j)]
                        Points.append(Coords)
                       
                        if i<=Density-2:
                            Edges.append([i*Repeat+j,i*Repeat+Repeat+j])
                           
                FiberNode.addObject("Mesh", position=Points, name="Mesh1", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e8)
                FiberNode.addObject("BarycentricMapping")



                FiberNode = cubito.addChild("FiberReinforcementNode1.1")  
                Density = 20
                IncrementAngle = 2*np.pi/Density
                Radius = 8
                NLevels = 2
                LevelHeight = 13
                Points = []
                Edges = []
                for i in range(NLevels):
                    for j in range(0,Density): 
                        Angle = j*IncrementAngle
                        Coords = [Radius*np.cos(Angle), 3.5+i*LevelHeight, Radius*np.sin(Angle)]
                        Points.append(Coords)
                        if j>=1:
                            Edges.append([i*Density+j-1,i*Density+j]) 
                            if j==Density-1:
                                for k in range(0,NLevels):
                                    Edges.append([k*Density,(1+k)*Density-1])
                                   
                                   
                   
               
                FiberNode.addObject("Mesh", position=Points, name="Mesh2", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
                
                
               
                FiberNode = cubito.addChild("FiberReinforcementNode2.1")    
                Density = 20
                IncrementAngle = np.pi/Density
                Radius = 8
                Repeat = 5
                Deg = 2*np.pi/Repeat
                LevelHeight = 13.725
                Points = []
                Edges = []
                for i in range(Density):
                    for j in range (0,Repeat):
                                
                        Angle = i*IncrementAngle
                        Coords = [Radius*np.cos(Angle+Deg*j), 20 + 3.5+i/Density*LevelHeight, Radius*np.sin(Angle+Deg*j)]
                        Points.append(Coords)
                               
                        if i<=Density-2:
                            Edges.append([i*Repeat+j,i*Repeat+Repeat+j])
                                   
                FiberNode.addObject("Mesh", position=Points, name="Mesh3", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e8)
                FiberNode.addObject("BarycentricMapping")



                FiberNode = cubito.addChild("FiberReinforcementNode2.2")  
                Density = 20
                IncrementAngle = 2*np.pi/Density
                Radius = 8
                NLevels = 2
                LevelHeight = 13
                Points = []
                Edges = []
                for i in range(NLevels):
                    for j in range(0,Density): 
                        Angle = j*IncrementAngle
                        Coords = [Radius*np.cos(Angle), 20 + 3.5+i*LevelHeight, Radius*np.sin(Angle)]
                        Points.append(Coords)
                        if j>=1:
                            Edges.append([i*Density+j-1,i*Density+j]) 
                            if j==Density-1:
                                for k in range(0,NLevels):
                                    Edges.append([k*Density,(1+k)*Density-1])
                                           
                                           
                           
                       
                FiberNode.addObject("Mesh", position=Points, name="Mesh4", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
              
       #
                
		#cubito/cavity
                cavity = cubito.addChild('cavity1')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoRotador_Cavity01.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                #cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=600, valueType=0)

                SPC1 = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

	
                cavity = cubito.addChild('cavity2')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoRotador_Cavity02.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                #cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=600, valueType=0)
                
                SPC2 = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)                
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)



		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="CubitoVisual.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC1=SPC1,SPC2=SPC2))


                return rootNode