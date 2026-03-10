# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 15:39:45 2024

@author: lab_Harold
"""
# import Sofa
import Sofa.Core
import Constants
import os
import csv
import numpy as np

LadoCubo = Constants.LadoCubo
PSI = 4
Displa = 9

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'
  

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
                    Sofa.Component.Topology.Mapping
                    Sofa.Component.Collision.Geometry
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
                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject("QPInverseProblemSolver", printLog=1, epsilon=0.1, maxIterations=100,tolerance=0.0000001)
                rootNode.dt = 0.01 

		#cubito
                cubito = rootNode.addChild('cubito')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner', template='CompressedRowSparseMatrixMat3x3d')
                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='Cubitorotador.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-14, 18, -14,  14, 22, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain = cubito.addObject('BoxROI', name='boxROIMain', box=[-13, 0.5, -13,  13, 19, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
             
                Container.init()
                MO.init()
                boxROIStiffness.init()
                boxROIMain.init()

                YM_base  = 10000     #5419.85
                YM_stiffROI = 4000 * 100
                
                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-14,  -2, -14,  14, 2, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')     
                
                modelStiff = cubito.addChild('modelStiff')
                modelStiff.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness.tetrahedraInROI", name='container')
                modelStiff.addObject('TetrahedronFEMForceField', template = 'Vec3d', name='FEM_stiff', method='large', poissonRatio=0.45, youngModulus=YM_stiffROI) 
                
                modelSubTopo1 = cubito.addChild('modelSubTopo1')
                modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain.tetrahedraInROI", name='container')
                modelSubTopo1.addObject('TetrahedronFEMForceField', template='Vec3d',  name='FEM_main', method='large', poissonRatio=0.45, youngModulus=YM_base)
                modelSubTopo1.addObject('YoungModulusActuator', template='Vec3d', name='YMActuator', maxYoungVariationRatio=0.1, minYoung=10, maxYoung=YM_base)  

            
        
        #cubito/fibers

                FiberNode = cubito.addChild("FiberReinforcement")
                
                Radius = Constants.RadioCilindro
                BaseHeight = 2.5
                CylinderHeight = Constants.AlturaCilindro
                
                # Ajustes solicitados
                HelixStartOffset = 2.0
                HelixEndOffset = 2.0
                
                HelixStartY = BaseHeight + HelixStartOffset
                HelixEndY = BaseHeight + CylinderHeight - HelixEndOffset
                HelixHeight = HelixEndY - HelixStartY
                
                Density = 30         # Discretización vertical
                Repeat = 20            # Número de fibras
                Turns = 0.4           # Número de vueltas helicoidales
                
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
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e6)
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
                RingNode.addObject("MeshSpringForceField", linesStiffness=1e6)
                RingNode.addObject("BarycentricMapping")
               
                
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitorotador_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                cavity.addObject('SurfacePressureEquality', triangles='@topo.triangles', eqPressure=6.89 * PSI) #38000 Pa or 5.5 PSI
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                                  
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[0, LadoCubo+Displa , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)

                
        # Punto "End-effector"         
                effector = cubito.addChild('EffectorNode')
                effector.addObject('MechanicalObject', position=[0, LadoCubo, 0], showObject=True, showObjectScale=10)
                PositionEffector = effector.addObject('PositionEffector', indices=0, effectorGoal=goal.goalMO.position.linkpath)
                effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)


		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_rotador_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
              
                return rootNode
