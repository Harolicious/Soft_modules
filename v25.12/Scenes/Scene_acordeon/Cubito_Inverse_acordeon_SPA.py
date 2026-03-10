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
PSI = 5
Displa = 3

# class Controller(Sofa.Core.Controller):   
    
#     def __init__(self, *args, **kwargs):
#         Sofa.Core.Controller.__init__(self, *args, **kwargs)
#         print(" Python::__init__::" + str(self.name.value))
        
#         self.RootNode = kwargs['RootNode']
#         self.SPA = kwargs['SPA']
#         self.Increment = 40
#         self.Pressure = 0
#         print(kwargs['RootNode'])
    
        
#         print('Finished Init')
        
#     def onAnimateBeginEvent(self, eventType):
#         self.Pressure = self.Pressure + self.Increment
#         if self.Pressure > 550 or self.Pressure < 0:
#             # self.Pressure = 550
#             self.Increment = -self.Increment
#         self.SPA.value.value = [self.Pressure]
        
#         pass
  
        
    # def onKeypressedEvent(self, c):
    #     key = c['key']        
            
        ##########################################
        # Cable                                  #
        ##########################################                
        
        # CurrentCableLength = np.array(self.CableConstraint.value.value[0])
        # print("CurrentCableLength", CurrentCableLength)
        # Increment = 1
        
        # if (key == "0"):
        #     InitialCavityVolume = self.ModelNode.Cavity01.SurfacePressureConstraint.initialCavityVolume.value
        #     Cavity01VolumeGrowth = self.SurfacePressureConstraint1.volumeGrowth.value
        #     GrowthPercent = np.abs(Cavity01VolumeGrowth)/InitialCavityVolume * 100
        #     GrowthPerDisplacement = GrowthPercent/CurrentCableLength
        #     print("GrowthPercent: ", GrowthPercent)
        #     print("GrowthPerDisplacement: ", GrowthPerDisplacement)
        
        # if (key == "6"):
        #     pass
        #     CurrentCableLength = CurrentCableLength + Increment
        #     self.CableConstraint.value = [CurrentCableLength.tolist()]
        #     #self.SerialObj.writelines(CurrentCableLength)

        # if (key == "4"):
        #     pass
        #     CurrentCableLength = CurrentCableLength - Increment
        #     self.CableConstraint.value = [CurrentCableLength.tolist()]            
            
#        ##########################################
#        # ReferenceMO                            #
#        ##########################################                
#        
#        CurrentPosition = np.array(self.ReferenceMO.position.value[0])
#        
#        self.DistanceFromBase
#        
#        print("CurrentPosition", CurrentPosition)
#        Increment = np.deg2rad(5)
#        
#        if (key == "2"):
#            pass
#            self.CurrentAngle +=  Increment
#            # Axes are inverted with respect to standard references
#            X = self.DistanceFromBase * np.sin(self.CurrentAngle)
#            Z = -self.DistanceFromBase * np.cos(self.CurrentAngle)
#            NewPosition = np.array([X+self.StartPosition[0],self.StartPosition[1],Z]) 
#            self.ReferenceMO.position.value = [NewPosition.tolist()]            
#        
#        if (key == "8"):
#            pass
#            self.CurrentAngle -=  Increment
#            # Axes are inverted with respect to standard references
#            X = self.DistanceFromBase * np.sin(self.CurrentAngle)
#            Z = -self.DistanceFromBase * np.cos(self.CurrentAngle)
#            NewPosition = np.array([X+self.StartPosition[0],self.StartPosition[1],Z]) 
#            self.ReferenceMO.position.value = [NewPosition.tolist()]            

        
        


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
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-13, 17, -13,  13, 21, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                Container.init()
                MO.init()
                boxROIStiffness.init()
                YM1 = 180000
                YM2 = YM1*100
                YMArray = np.ones(len(Loader.tetras))*YM1
                IdxElementsInROI = np.array(boxROIStiffness.tetrahedronIndices.value)
                YMArray[IdxElementsInROI] = YM2
                print(f"len IdxElementsInROI: {len(IdxElementsInROI)}")
                
                print(f"Largo de YMArray:{len(YMArray)}")
                #cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=180000)
                cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=YMArray.flatten().tolist())
                #cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM2', method='large', poissonRatio=0.3,  youngModulus=180000)
                
                #cubito.addObject('TetrahedronHyperelasticityFEMForceField', name="HyperElasticMaterial", materialName="MooneyRivlin", ParameterSet="48000 -1.5e5 3000")

                cubito.addObject('BoxROI', name='boxROI', box=[-13, -1, -13,  13, 2, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                #cubito.addObject('UncoupledConstraintCorrection')
                

        #cubito/fibers
        

                FiberNode = cubito.addChild("FiberReinforcementNode1")    
                Density2 = 15
                IncrementAngle2 = 2*np.pi/Density2
                Radius2 = 8
                NLevels2 = 7
                LevelHeight2 = 2.1
                Points2 = []
                Edges2 = []
                for i in range(NLevels2):
                    for j in range(0,Density2): 
                        Angle2 = j*IncrementAngle2
                        Coords2 = [Radius2*np.cos(Angle2), 3.5+i*LevelHeight2, Radius2*np.sin(Angle2)]
                        Points2.append(Coords2)
                        if j>=1:
                            Edges2.append([(i*Density2+j-1),(i*Density2+j)])

                            if j==Density2-1:
                                for k in range(0,NLevels2):
                                    Edges2.append([k*Density2,(1+k)*Density2-1])
                                
                             
                                    
                FiberNode.addObject("Mesh", position=Points2, name="Mesh", edges=Edges2)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
                
                             
                FiberNode = cubito.addChild("FiberReinforcementNode3")    
                Density = 10
                IncrementAngle = (np.pi/2)/Density
                Radius = 8
                Repeat = 6
                Deg = (np.pi/2)/Repeat
                LevelHeight = 14
                Points = []
                Edges = []
                for i in range(Density):
                    for j in range (0,Repeat):
                        
                        Angle = i*IncrementAngle+Deg*19
                        Coords = [Radius*np.cos(Angle+Deg*j), 3.5+i/Density*LevelHeight, Radius*np.sin(Angle+Deg*j)]
                        Points.append(Coords)
                        
                        if i<=Density-2:
                            Edges.append([i*Repeat+j,i*Repeat+Repeat+j])
                            
                FiberNode.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")


                FiberNode = cubito.addChild("FiberReinforcementNode4")    
                Density4 = 10
                IncrementAngle4 = (-np.pi/2)/Density4
                Radius4 = 8
                Repeat4 = 6
                Deg4 = (np.pi/2)/Repeat4
                LevelHeight4 = 14
                Points4 = []
                Edges4 = []
                for i in range(Density4):
                    for j in range (0,Repeat4):
                        
                        Angle4 = i*IncrementAngle4
                        Coords4 = [Radius4*np.cos(Angle4+Deg4*j), 3.5+i/Density4*LevelHeight4, Radius4*np.sin(Angle4+Deg4*j)]
                        Points4.append(Coords4)
                        
                        if i<=Density4-2:
                            Edges4.append([i*Repeat4+j,i*Repeat4+Repeat4+j])
                            
                FiberNode.addObject("Mesh", position=Points4, name="Mesh", edges=Edges4)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
               
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitoacordeon_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPA = cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles')
                #cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=False)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                
        
        # Effector
        # bunny/effector
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[LadoCubo/5 , LadoCubo + 3 , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)
                goal.addObject('UncoupledConstraintCorrection')
                
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
                
                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPA=SPA))

                return rootNode