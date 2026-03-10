
import Sofa.Core
import Constants

import os
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

LadoCubo = Constants.LadoCubo
PSI = 6.6
Displa = 7.8



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

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='CubitoEstirar.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container' , edges=False)
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-14, 18, -14,  14, 22, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain = cubito.addObject('BoxROI', name='boxROIMain', box=[-13, -1, -13,  13, 19, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                
                Container.init()
                MO.init()
                boxROIStiffness.init()
                boxROIMain.init()
                
                YM_base  = 15000 #14850
                YM_stiffROI = YM_base*100
                
                n_tetra_total = len(Loader.tetras)
                print("Número total de tetraedros:", n_tetra_total)
                
                idx_tetra_stiff = boxROIStiffness.tetrahedronIndices.value
                n_stiff = len(idx_tetra_stiff)
                print("Número de tetraedros en boxROIStiffness:", n_stiff)
                
                idx_tetra_base = boxROIMain.tetrahedronIndices.value
                n_base = len(idx_tetra_base)
                print("Número de tetraedros en boxROIMain:", n_base)

                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-14,  -2, -14,  14, 2, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')    
                
                idx_tetra_BC = boxROI.tetrahedronIndices.value
                n_BC = len(idx_tetra_BC)
                print("Número de tetraedros en boxROIBC:", n_BC)

                modelStiff = cubito.addChild('modelStiff')
                modelStiff.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness.tetrahedraInROI", name='container')
                modelStiff.addObject('TetrahedronFEMForceField', template = 'Vec3d', name='FEM_stiff', method='large', poissonRatio=0.45, youngModulus=YM_stiffROI) 
                
                modelSubTopo1 = cubito.addChild('modelSubTopo1')
                modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain.tetrahedraInROI", name='container')
                modelSubTopo1.addObject('TetrahedronFEMForceField', template='Vec3d',  name='FEM_main', method='large', poissonRatio=0.45, youngModulus=YM_base)
                modelSubTopo1.addObject('YoungModulusActuator', template='Vec3d', name='YMActuator', maxYoungVariationRatio=0.1, minYoung=10, maxYoung=YM_base)   
            

            
                
        #cubito/fibers
        
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
                            if j==29:
                                Edges.append([i*Density+j, i*Density+j-Density+1])
                                
                
                FiberNode.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e7)
                FiberNode.addObject("BarycentricMapping")
                
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoEstirar_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                cavity.addObject('SurfacePressureEquality', triangles='@topo.triangles', eqPressure=6.89 * PSI) # PSI to KPa
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[0, LadoCubo + Displa , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)   
  
        # Punto "End-effector"   
                effector = cubito.addChild('EffectorNode')
                effector.addObject('MechanicalObject', position=[0, LadoCubo, 0], showObject=True, showObjectScale=10)
                PositionEffector = effector.addObject('PositionEffector', indices=0, effectorGoal=goal.goalMO.position.linkpath)
                effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_Estirar_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPA=SPA))
                
                return rootNode
            
