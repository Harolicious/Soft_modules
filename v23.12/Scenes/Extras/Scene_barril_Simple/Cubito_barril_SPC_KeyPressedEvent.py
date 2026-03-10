import Sofa

import os
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        self.RootNode = kwargs['RootNode']
        self.SPC = kwargs['SPC']
        self.Increment = 40
        self.Pressure = 0
        print(kwargs['RootNode'])
    
        
        print('Finished Init')
        
    def onAnimateBeginEvent(self, eventType):
        self.Pressure = self.Pressure + self.Increment
        if self.Pressure > 550 or self.Pressure < 0:
            # self.Pressure = 550
            self.Increment = -self.Increment
        self.SPC.value.value = [self.Pressure]
        
        pass
  
        
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
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.0000001)

		#cubito
                cubito = rootNode.addChild('CubitoBarril')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                
                cubito.addObject('SparseLDLSolver', name='preconditioner')

                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioners='preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='Cubitobarril.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-13, 17 , -13,  13, 21, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
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

                cubito.addObject('BoxROI', name='boxROI', box=[-13, -1, -13,  13, 3, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                #cubito.addObject('UncoupledConstraintCorrection')
                

        #cubito/fibers
        
                FiberNode = cubito.addChild("FiberReinforcementNode")  
                
                Density = 15
                # IncrementAngle = np.pi/Density
                Radius = 8
                Repeat = 20
                Deg = 2*np.pi/Repeat
                LevelHeight = 13.5
                Points = []
                Edges = []
                for i in range(Density):
                    for j in range (0,Repeat):
                        Angle = 0
                        Coords = [Radius*np.cos(Angle+Deg*j), 3.5+i/Density*LevelHeight, Radius*np.sin(Angle+Deg*j)]
                        Points.append(Coords)
                        if i*Repeat+j+Repeat+1<=Repeat*Density:
                            Edges.append([i*Repeat+j,i*Repeat+j+Repeat])
                        
                
                FiberNode.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
                
               
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitobarril_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPC = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                #cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=False)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)


		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_barril_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC=SPC))
                
                return rootNode