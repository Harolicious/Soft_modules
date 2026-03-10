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

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def rotate(degrees):
    theta = np.radians(degrees)
    c,s =   np.cos(theta), np.sin(theta)
    R = np.array(((c, -s),(c, s)))
    return R

class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        # Bandera para controlar la animacion
        self.animation_finished = False 
        
        # Inicializar atributos con valores de kwargs
        self.RootNode = kwargs['RootNode']
        self.SPA = kwargs['SPA']
        self.Increment = 0.5
        self.Pressure = 0        
        self.Decreasing = False
        self.Maxpressure = 50
        self.EndEffectorMO = kwargs['EndEffectorMO']
        
        # Definir ruta de archivo csv 
        self.csv_file_path = "end_effector_data_Rotador.csv"

        # Crear archivo CSV y escribir encabezados si no existe
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Pressure","Position_X", "Position_Y" ,"Position_Z", "Yaw", "Pitch", "Roll"])
        
        print('Finished Init')
        
    def save_end_effector_data(self, time):
        position = self.EndEffectorMO.position.value
        
        SecondPointCoord = self.EndEffectorMO.position.value[1]
        Angle = np.rad2deg(np.arctan2(SecondPointCoord[2],SecondPointCoord[0]))
        
        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time,self.Pressure,position[0][0],position[0][1],position[0][2],Angle])
        except Exception as e:
            print(f"Error al escribir en archivo csv:{e}")
            
    def update_pressure_increase(self, pressure, SPA):
        pressure += self.Increment
        if pressure > self.Maxpressure and self.animation_finished==False:
            pressure = self.Maxpressure
        SPA.value.value = [pressure]
        return pressure

    def update_pressure_decrease(self, pressure, SPA):
        pressure -= self.Increment
        if pressure < 0 and not self.animation_finished:
            pressure = 0
        SPA.value.value = [pressure]
        return pressure

        
    def onAnimateBeginEvent(self, eventType):
        # print(f"Current End-Effector position: {self.EndEffectorMO.position.value}")        
        print(f"pressure: {self.Pressure}")
    
        # Aquí obtienes el tiempo actual de la simulación
        current_time = self.RootNode.time.value
        
        # Imprimir mensaje si la animación ha terminado
        if self.animation_finished:
            print("animación terminada")
            self.RootNode.dt = 0 
            self.Pressure = 0
            return

        # Guardar datos del EndEffector
        self.save_end_effector_data(current_time)

        if not self.Decreasing:
            # Incrementar presiones
            self.Pressure = self.update_pressure_increase(self.Pressure, self.SPA)
            
            if self.Pressure >= self.Maxpressure:
                self.Decreasing = True  
        else:
            # Decrementar presiones
            self.Pressure = self.update_pressure_decrease(self.Pressure, self.SPA)
            
            if self.Pressure <= 0:
                self.Decreasing = False  
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
                cubito = rootNode.addChild('cubito')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                
                cubito.addObject('SparseLDLSolver', name='preconditioner')

                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='Cubitorotador.vtk')
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
        
                FiberNode = cubito.addChild("FiberReinforcementNode")    
                
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
                            
                FiberNode.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e8)
                FiberNode.addObject("BarycentricMapping")

                FiberNode = cubito.addChild("FiberReinforcementNode")  
                
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
                                    
                                    
                    
                
                FiberNode.addObject("Mesh", position=Points2, name="Mesh", edges=Edges2)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
                
                
                
                
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitorotador_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPA=cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles')
                #cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=False)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                

        
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[0, LadoCubo+5, 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)
                goal.addObject('UncoupledConstraintCorrection')
                
        # Punto "End-effector"         
                effector = cubito.addChild('EffectorNode')
                effector.addObject('MechanicalObject', position=[0, LadoCubo, 0], showObject=True, showObjectScale=10)
                PositionEffector = effector.addObject('PositionEffector', indices=0, effectorGoal=goal.goalMO.position.linkpath)
                effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

        # # Punto "End-effector"
                
        #         EndEffectorNode = cubito.addChild("EndEffectorNode")
        #         EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,LadoCubo,0], [LadoCubo/2,LadoCubo,0], [0,LadoCubo/2+LadoCubo,0],[0,LadoCubo,LadoCubo/2]], showObject=True, showObjectScale=10)
        #         EndEffectorNode.addObject("BarycentricMapping")        


		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_rotador_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPA=SPA, EndEffectorMO=EndEffectorMO))
                
                return rootNode
