"""
Created on Wed Apr 28 18:25:43 2024

@author: lab_Harold
"""

# import Sofa
import Sofa.Core
import Constants

import os
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

LadoCubo = Constants.LadoCubo
PSI = 4
Displa = 4.5


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
                rootNode.addObject("QPInverseProblemSolver", printLog=1, epsilon=0.1, maxIterations=1000,tolerance=1e-5)
                rootNode.dt = 0.01

		#cubito
                cubito = rootNode.addChild('CubitoBarril')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner')
                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='Cubitoacordeon.vtk')
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

                YM_base  = 30000     #5419.85
                YM_stiffROI = 9000 * 100
                
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
        
                CylinderHeight = 15
                y_start = 3.5
                y_end = CylinderHeight + 1.5
                fiber_height = y_end - y_start     # = 7.0
                
                Radius = Constants.RadioCilindro
                
                def create_rings(parent, name, radius,
                                         n_rings, ring_density,
                                         y_start, y_end, stiffness):
                
                    node = parent.addChild(name)
                
                    Points = []
                    Edges = []
                
                    dTheta = 2 * np.pi / ring_density
                    dy = (y_end - y_start) / (n_rings - 1)
                
                    for i in range(n_rings):
                        y = y_start + i * dy
                
                        ring_offset = i * ring_density
                
                        for j in range(ring_density):
                            theta = j * dTheta
                            x = radius * np.cos(theta)
                            z = radius * np.sin(theta)
                
                            Points.append([x, y, z])
                
                            # Cerrar anillo
                            Edges.append([
                                ring_offset + j,
                                ring_offset + (j + 1) % ring_density
                            ])
                
                    node.addObject("Mesh", position=Points, edges=Edges)
                    node.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                    node.addObject("MeshSpringForceField", linesStiffness=stiffness)
                    node.addObject("BarycentricMapping")
 
                    
 
                def create_helices_sector(parent, name, radius, n_fibers, density,
                                          y_start, height, total_angle,
                                          sector_angle, stiffness, angle_offset=0.0):
                
                    node = parent.addChild(name)
                
                    Points = []
                    Edges = []
                
                    dY = height / (density - 1)
                    dTheta = total_angle / (density - 1)
                    fiber_angle = sector_angle / (n_fibers - 1)
                
                    for i in range(density):
                        y = y_start + i * dY
                        theta_i = i * dTheta + angle_offset
                
                        for j in range(n_fibers):
                            idx = i * n_fibers + j
                            theta = theta_i + j * fiber_angle
                
                            x = radius * np.cos(theta)
                            z = radius * np.sin(theta)
                
                            Points.append([x, y, z])
                
                            if i < density - 1:
                                Edges.append([idx, idx + n_fibers])
                
                    node.addObject("Mesh", position=Points, edges=Edges)
                    node.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                    node.addObject("MeshSpringForceField", linesStiffness=stiffness)
                    node.addObject("BarycentricMapping")
                
                
                create_rings(parent=cubito,name="FiberReinforcementNode1",radius=Radius,
                             n_rings=6, ring_density=15, y_start=y_start, y_end = y_end, stiffness=5e9)                
                
                create_helices_sector(parent=cubito, name="FiberReinforcementNode_Positive", radius=Radius,
                                      n_fibers=4, density=10, y_start=y_start, height=fiber_height, total_angle=np.pi/2,
                                      sector_angle=np.pi/2, angle_offset=-np.pi/2, stiffness=1e9)
                
 
                create_helices_sector(parent=cubito, name="FiberReinforcementNode_Negative", radius=Radius,
                                      n_fibers=4, density=10, y_start=y_start, height=fiber_height, total_angle=-np.pi/2,
                                      sector_angle=np.pi/2, angle_offset=0.0, stiffness=1e9)
               
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitoacordeon_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                cavity.addObject('SurfacePressureEquality', triangles='@topo.triangles', eqPressure=6.89 * PSI) #38000 Pa or 5.5 PSI
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                
                  
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[4, LadoCubo+Displa , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)

                
        # Punto "End-effector"         
                effector = cubito.addChild('EffectorNode')
                effector.addObject('MechanicalObject', position=[0, LadoCubo, 0], showObject=True, showObjectScale=10)
                PositionEffector = effector.addObject('PositionEffector', indices=0, effectorGoal=goal.goalMO.position.linkpath)
                effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)


		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_acordeon_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
              
                return rootNode