
import Sofa.Core
import Constants

import os
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

LadoCubo = Constants.LadoCubo
PSI = 7.4
Displa = 3.4

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

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='Cubitobarril.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container' , edges=False)
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-14, 17, -14,  14, 22, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain = cubito.addObject('BoxROI', name='boxROIMain', box=[-13, 0.5, -13,  13, 19.5, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                
                Container.init()
                MO.init()
                boxROIStiffness.init()
                boxROIMain.init()

                YM_base  = 12000 #14850
                YM_stiffROI = YM_base*100
                
                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-14,  -2, -14,  14, 3, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
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
        
                FiberNode = cubito.addChild("FiberReinforcementNode")  
                
                Density = 6              # discretización vertical
                CapDensity = 6            # discretización SOLO en tapas
                Radius = Constants.RadioCilindro
                Repeat = 16             # potencia de 2
                LevelHeight = Constants.AlturaCilindro
                
                Ymin = 2.5
                Ymax = Ymin + LevelHeight
                dy = (Ymax - Ymin) / (Density - 1)
                
                half = Repeat // 2
                
                # avance angular entre fibras vecinas
                dTheta_fiber = 2 * np.pi / Repeat
                
                # 🔥 giro por nivel: fibra j → j+2
                dTheta_level = (2 * dTheta_fiber) / (Density - 1)
                
                Points = []
                Edges = []
                
                offset_pos = 0  # para indexado posterior
                
                for j in range(Repeat):
                
                    theta0 = j * dTheta_fiber
                
                    for i in range(Density):
                
                        theta = theta0 + i * dTheta_level
                        y = Ymin + i * dy
                
                        idx = len(Points)
                
                        x = Radius * np.cos(theta)
                        z = Radius * np.sin(theta)
                
                        Points.append([x, y, z])
                
                        if i < Density - 1:
                            Edges.append([idx, idx + 1])
                
                offset_neg = len(Points)
                
                for j in range(Repeat):
                
                    theta0 = j * dTheta_fiber
                
                    for i in range(Density):
                
                        theta = theta0 - i * dTheta_level
                        y = Ymin + i * dy
                
                        idx = len(Points)
                
                        x = Radius * np.cos(theta)
                        z = Radius * np.sin(theta)
                
                        Points.append([x, y, z])
                
                        if i < Density - 1:
                            Edges.append([idx, idx + 1])
                
                def add_cap_connections(offset, level_i):
                
                    for j in range(half):
                
                        # nodos extremos de fibras opuestas
                        idx1 = offset + j * Density + level_i
                        idx2 = offset + (j + half) * Density + level_i
                
                        p1 = np.array(Points[idx1])
                        p2 = np.array(Points[idx2])
                
                        cap_idx = []
                
                        for k in range(CapDensity + 1):
                            alpha = k / CapDensity
                            p = (1 - alpha) * p1 + alpha * p2
                            cap_idx.append(len(Points))
                            Points.append(p.tolist())
                
                        for k in range(CapDensity):
                            Edges.append([cap_idx[k], cap_idx[k + 1]])
                
                
                # hélice positiva
                add_cap_connections(offset_pos, 0)               # tapa inferior
                add_cap_connections(offset_pos, Density - 1)     # tapa superior
    

                
                FiberNode.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=5e5)
                FiberNode.addObject("BarycentricMapping")
                
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitobarril_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                cavity.addObject('SurfacePressureEquality', triangles='@topo.triangles', eqPressure=6.89 * PSI) # PSI to KPa
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                
        # Effector
        # bunny/effector
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[0, LadoCubo/2, LadoCubo/2 + Displa ], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)   
  
        # Punto "End-effector"   
                effector = cubito.addChild('EffectorNode')
                effector.addObject('MechanicalObject', position=[0, LadoCubo/2, LadoCubo/2], showObject=True, showObjectScale=10)
                PositionEffector = effector.addObject('PositionEffector', indices=0, effectorGoal=goal.goalMO.position.linkpath)
                effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_barril_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPA=SPA))
                
                return rootNode
            
