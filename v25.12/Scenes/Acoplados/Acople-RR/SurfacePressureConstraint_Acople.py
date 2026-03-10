#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 15:39:45 2024

@author: lab-Harold
"""

import Sofa

import os
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))

class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        self.RootNode = kwargs['RootNode']
        self.SPC = kwargs['SPC']
        self.Increment = 50
        self.Pressure = 0
        print(kwargs['RootNode'])
    
        
        print('Finished Init')
        
    def onAnimateBeginEvent(self, eventType):
        self.Pressure = self.Pressure + self.Increment
        if self.Pressure > 650 or self.Pressure < 0:
            # self.Pressure = 550
            self.Increment = -self.Increment
        self.SPC.value.value = [self.Pressure]
        
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
                cubito1 = rootNode.addChild('cubito1')
                cubito1.addObject('EulerImplicitSolver', name='odesolver')
                
                cubito1.addObject('SparseLDLSolver', name='preconditioner')

                cubito1.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioners='preconditioner', use_precond=True, update_step=1)

                Loader1 = cubito1.addObject('MeshVTKLoader', name='loader', filename='Cubitorotador.vtk')
                Container1 = cubito1.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito1.addObject('TetrahedronSetTopologyModifier')

                MO1 = cubito1.addObject('MechanicalObject', name='tetras', template='Vec3', translation="0 0 0" ,showIndices=False)
                cubito1.addObject('UniformMass', totalMass=0.5)
                              
                boxROIStiffness1 = cubito1.addObject('BoxROI', name='boxROIStiffness', box=[-13, 21, -13,  13, 24, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                Container1.init()
                MO1.init()
                boxROIStiffness1.init()
                YM1 = 180000
                YM21 = YM1*100
                YMArray1 = np.ones(len(Loader1.tetras))*YM1
                IdxElementsInROI1 = np.array(boxROIStiffness1.tetrahedronIndices.value)
                YMArray1[IdxElementsInROI1] = YM21
                print(f"len IdxElementsInROI1: {len(IdxElementsInROI1)}")
                

                
                print(f"Largo de YMArray1:{len(YMArray1)}")
                #cubito1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=180000)
                cubito1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=YMArray1.flatten().tolist())
                #cubito1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM2', method='large', poissonRatio=0.3,  youngModulus=180000)
                
                #cubito1.addObject('TetrahedronHyperelasticityFEMForceField', name="HyperElasticMaterial", materialName="MooneyRivlin", ParameterSet="48000 -1.5e5 3000")

                cubito1.addObject('BoxROI', name='boxROI', box=[-13, -1, -13,  13, 2, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito1.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito1.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                #cubito1.addObject('UncoupledConstraintCorrection')
                
		#cubito2
                
                cubito2 = rootNode.addChild('cubito2')
                cubito2.addObject('EulerImplicitSolver', name='odesolver')
                
                cubito2.addObject('SparseLDLSolver', name='preconditioner')

                cubito2.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioners='preconditioner', use_precond=True, update_step=1)

                Loader2 = cubito2.addObject('MeshVTKLoader', name='loader', filename='Cubitorotador.vtk')
                Container2 = cubito2.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito2.addObject('TetrahedronSetTopologyModifier')

                MO2 = cubito2.addObject('MechanicalObject', name='tetras', template='Vec3', translation="0 15 0" , showIndices=False) #11.4
                cubito2.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness2 = cubito2.addObject('BoxROI', name='boxROIStiffness', box=[-13, 50, -13,  13, 54, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                Container2.init()
                MO2.init()
                boxROIStiffness2.init()
                YM1 = 180000
                YM22 = YM1*100
                YMArray2 = np.ones(len(Loader2.tetras))*YM1
                IdxElementsInROI2 = np.array(boxROIStiffness2.tetrahedronIndices.value)
                YMArray2[IdxElementsInROI2] = YM22
                print(f"len IdxElementsInROI2: {len(IdxElementsInROI2)}")
                
                print(f"Largo de YMArray2:{len(YMArray2)}")
                #cubito2.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=180000)
                cubito2.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=YMArray2.flatten().tolist())
                #cubito2.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM2', method='large', poissonRatio=0.3,  youngModulus=180000)
                
                #cubito2.addObject('TetrahedronHyperelasticityFEMForceField', name="HyperElasticMaterial", materialName="MooneyRivlin", ParameterSet="48000 -1.5e5 3000")

                cubito2.addObject('BoxROI', name='boxROI', box=[-13, 29, -13,  13, 32, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito2.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito2.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                #cubito2.addObject('UncoupledConstraintCorrection')
                
        #cubito/fibers
       
                FiberNode1 = cubito1.addChild("FiberReinforcementNode1")    
               
                Density = 20
                IncrementAngle = np.pi/Density
                Radius = 8
                Repeat = 5
                Deg = 2*np.pi/Repeat
                LevelHeight = 13.5
                Points = []
                Edges = []
                for i in range(Density):
                    for j in range (0,Repeat):
                        
                        Angle = i*IncrementAngle
                        Coords = [Radius*np.cos(Angle+Deg*j), 5+i/Density*LevelHeight, Radius*np.sin(Angle+Deg*j)]
                        Points.append(Coords)
                       
                        if i<=Density-2:
                            Edges.append([i*Repeat+j,i*Repeat+Repeat+j])
                           
                FiberNode1.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode1.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode1.addObject("MeshSpringForceField", linesStiffness=1e8)
                FiberNode1.addObject("BarycentricMapping")

                FiberNode1 = cubito1.addChild("FiberReinforcementNode1.1")  
               
                Density2 = 20
                IncrementAngle2 = 2*np.pi/Density2
                Radius2 = 8
                NLevels2 = 2
                LevelHeight2 = 13
                Points2 = []
                Edges2 = []
                for i in range(NLevels2):
                    for j in range(0,Density2): 
                        Angle2 = j*IncrementAngle2
                        Coords2 = [Radius2*np.cos(Angle2), 5+i*LevelHeight2, Radius2*np.sin(Angle2)]
                        Points2.append(Coords2)
                        if j>=1:
                            Edges2.append([i*Density2+j-1,i*Density2+j]) 
                            if j==Density2-1:
                                for k in range(0,NLevels2):
                                    Edges2.append([k*Density2,(1+k)*Density2-1])
                                   
                                   
                   
               
                FiberNode1.addObject("Mesh", position=Points2, name="Mesh", edges=Edges2)
                FiberNode1.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode1.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode1.addObject("BarycentricMapping")
               
       #cubito/fibers
       
                FiberNode2 = cubito2.addChild("FiberReinforcementNode2")    
               
                Density = 20
                IncrementAngle = np.pi/Density
                Radius = 8
                Repeat = 5
                Deg = 2*np.pi/Repeat
                LevelHeight = 13.5
                Points = []
                Edges = []
                for i in range(Density):
                    for j in range (0,Repeat):
                        
                        Angle = i*IncrementAngle
                        Coords = [Radius*np.cos(Angle+Deg*j), 5+i/Density*LevelHeight, Radius*np.sin(Angle+Deg*j)]
                        Points.append(Coords)
                       
                        if i<=Density-2:
                            Edges.append([i*Repeat+j,i*Repeat+Repeat+j])
                           
                FiberNode2.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode2.addObject("MechanicalObject", showObject=True, showObjectScale=10, translation="0 30 0" ) #23
                FiberNode2.addObject("MeshSpringForceField", linesStiffness=1e8)
                FiberNode2.addObject("BarycentricMapping")

                FiberNode2 = cubito2.addChild("FiberReinforcementNode2.1")  
               
                Density2 = 20
                IncrementAngle2 = 2*np.pi/Density2
                Radius2 = 8
                NLevels2 = 2
                LevelHeight2 = 13
                Points2 = []
                Edges2 = []
                for i in range(NLevels2):
                    for j in range(0,Density2): 
                        Angle2 = j*IncrementAngle2
                        Coords2 = [Radius2*np.cos(Angle2), 5+i*LevelHeight2, Radius2*np.sin(Angle2)]
                        Points2.append(Coords2)
                        if j>=1:
                            Edges2.append([i*Density2+j-1,i*Density2+j]) 
                            if j==Density2-1:
                                for k in range(0,NLevels2):
                                    Edges2.append([k*Density2,(1+k)*Density2-1])
                                   
                                   
                   
               
                FiberNode2.addObject("Mesh", position=Points2, name="Mesh", edges=Edges2)
                FiberNode2.addObject("MechanicalObject", showObject=True, showObjectScale=10, translation="0 30 0" ) #23        
                FiberNode2.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode2.addObject("BarycentricMapping")
                
                
		#cubito1/cavity
                cavity1 = cubito1.addChild('cavity')
                cavity1.addObject('MeshSTLLoader', name='loader', filename='Cubitorotador_Cavity.stl')
                cavity1.addObject('MeshTopology', src='@loader', name='topo')
                cavity1.addObject('MechanicalObject', name='cavity')
                cavity1.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=600, valueType=0)
                #SPC1=cavity1.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                cavity1.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

		#cubito/cavity
                cavity2 = cubito2.addChild('cavity')
                cavity2.addObject('MeshSTLLoader', name='loader', filename='Cubitorotador_Cavity.stl')
                cavity2.addObject('MeshTopology', src='@loader', name='topo')
                cavity2.addObject('MechanicalObject', name='cavity', translation="0 30 0" ) #23
                cavity2.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=600, valueType=0)
                #SPC2=cavity2.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                cavity2.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)


		#cubito/cubitoVisu
                cubitoVisu1 = cubito1.addChild('visu')
                cubitoVisu1.addObject('TriangleSetTopologyContainer', name='container')
                cubitoVisu1.addObject('TriangleSetTopologyModifier')
                cubitoVisu1.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container", output="@container")

                cubitoVisu1.addObject('OglModel', color=[0.3, 0.2, 0.2, 0.6])
                cubitoVisu1.addObject('IdentityMapping')


		#cubito/cubitoVisu
                cubitoVisu2 = cubito2.addChild('visu')
                cubitoVisu2.addObject('TriangleSetTopologyContainer', name='container')
                cubitoVisu2.addObject('TriangleSetTopologyModifier')
                cubitoVisu2.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container", output="@container")

                cubitoVisu2.addObject('OglModel', color=[0.3, 0.2, 0.2, 0.6])
                cubitoVisu2.addObject('IdentityMapping')

          #      rootNode.addObject(Controller(name="ActuationController1", RootNode=rootNode, SPC=SPC2))
          #      rootNode.addObject(Controller(name="ActuationController2", RootNode=rootNode, SPC=SPC1))
                
                return rootNode
               