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
RadioCilindro = Constants.RadioCilindro
PSI = 6

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
        self.SPC = kwargs['SPC']
        self.Maxpressure = 6.89 * PSI #Pa
        self.Increment = self.Maxpressure/500 #Pa
        self.Pressure = 0        
        self.Decreasing = False
        self.EndEffectorMO = kwargs['EndEffectorMO']
        
        # Definir ruta de archivo csv 
        self.csv_file_path = "end_effector_data_Rotador.csv"

        # Crear archivo CSV y escribir encabezados si no existe
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Pressure","Position_X", "Position_Y" ,"Position_Z", "Angle"])
        
        print('Finished Init')
        
    def save_end_effector_data(self, time):
        position = self.EndEffectorMO.position.value
        # rotation = self.EndEffectorMO.rotation.value
        
        SecondPointCoord = self.EndEffectorMO.position.value[1]
        Angle = np.rad2deg(np.arctan2(SecondPointCoord[2],SecondPointCoord[0]))
        
        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time,self.Pressure,position[0][0],position[0][1],position[0][2],Angle])
        except Exception as e:
            print(f"Error al escribir en archivo csv:{e}")
            
    def update_pressure_increase(self, pressure, spc):
        pressure += self.Increment
        if pressure > self.Maxpressure and self.animation_finished==False:
            pressure = self.Maxpressure
        spc.value.value = [pressure]
        return pressure

    def update_pressure_decrease(self, pressure, spc):
        pressure -= self.Increment
        if pressure < 0 and not self.animation_finished:
            pressure = 0
        spc.value.value = [pressure]
        return pressure

        
    def onAnimateBeginEvent(self, eventType):
        # print(f"Current End-Effector position: {self.EndEffectorMO.position.value}")        
        # print(f"pressure: {self.Pressure}")
    
        # Aquí obtienes el tiempo actual de la simulación
        current_time = self.RootNode.time.value
        
        # Imprimir mensaje si la animación ha terminado
        if self.animation_finished:
            # print("animación terminada")
            self.RootNode.dt = 0 
            self.Pressure = 0
            return

        # Guardar datos del EndEffector
        self.save_end_effector_data(current_time)

        if not self.Decreasing:
            # Incrementar presiones
            self.Pressure = self.update_pressure_increase(self.Pressure, self.SPC)
            
            if self.Pressure >= self.Maxpressure:
                self.Decreasing = True  
        else:
            # Decrementar presiones
            self.Pressure = self.update_pressure_decrease(self.Pressure, self.SPC)
            
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
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.0000001)
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
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-LadoCubo/2 - 3 , LadoCubo - 3, -LadoCubo/2 - 3,  LadoCubo/2 + 3, LadoCubo + 3, LadoCubo/2 + 3], drawBoxes=False, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                Container.init()
                MO.init()
                boxROIStiffness.init()
                YM1 = 125000 #YMd del Ecoflex0030
                YM2 = YM1*1000
                YMArray = np.ones(len(Loader.tetras))*YM1
                IdxElementsInROI = np.array(boxROIStiffness.tetrahedronIndices.value)
                YMArray[IdxElementsInROI] = YM2
           #    print(f"len IdxElementsInROI: {len(IdxElementsInROI)}")
                
           #    print(f"Largo de YMArray:{len(YMArray)}")
                #cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=180000)
                cubito.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.45,  youngModulus=YMArray.flatten().tolist())
                #cubito.addObject('TetrahedronHyperelasticityFEMForceField', name="HyperElasticMaterial", materialName="MooneyRivlin", ParameterSet="48000 -1.5e5 3000")

                cubito.addObject('BoxROI', name='boxROI', box=[-13, -0.5, -13,  13, 2, 13], drawBoxes=False, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                #cubito.addObject('UncoupledConstraintCorrection')

		#cubito/cavity_sensor
                sensor = cubito.addChild('sensor')
                sensor.addObject('MeshVTKLoader', name='loader', filename='Sensor_galinstan_45_y.vtk')
                # sensor.addObject('MeshVTKLoader', name='loader', filename='Sensor_galinstan_-45_y.vtk')
                sensor.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                sensor.addObject('TetrahedronSetTopologyModifier')
                sensor.addObject('MechanicalObject', name='tetras',  translation=[0,LadoCubo/2,0]) # 45
                # sensor.addObject('MechanicalObject', name='tetras',  translation=[0,15,22.5]) # -45
                sensor.addObject('UniformMass', totalMass=0.5)
                sensor.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.45,  youngModulus=100)
                sensor.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')

        # Punto "End-effector"
                
                EndEffectorNode = cubito.addChild("EndEffectorNode")
                EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,LadoCubo,0], [10,LadoCubo,0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")
                

        #cubito/fibers
        
                FiberNode1 = cubito.addChild("FiberReinforcementNode")    
                
                Density = 20
                IncrementAngle = np.pi/Density
                Radius = RadioCilindro
                Repeat = 5
                Deg = 2*np.pi/Repeat
                LevelHeight = LadoCubo/2 
                Points = []
                Edges = []
                for i in range(Density):
                    for j in range (0,Repeat):
                        
                        Angle = i*IncrementAngle
                        Coords = [Radius*np.cos(Angle+Deg*j), 7.5+i/Density*LevelHeight, Radius*np.sin(Angle+Deg*j)]
                        Points.append(Coords)
                        
                        if i<=Density-2:
                            Edges.append([i*Repeat+j,i*Repeat+Repeat+j])
                            
                FiberNode1.addObject("Mesh", position=Points, name="Mesh", edges=Edges)
                FiberNode1.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode1.addObject("MeshSpringForceField", linesStiffness=1e8)
                FiberNode1.addObject("BarycentricMapping")

                FiberNode2 = cubito.addChild("FiberReinforcementNode_2")  
                
                Density2 = 20
                IncrementAngle2 = 2*np.pi/Density2
                Radius2 = RadioCilindro
                NLevels2 = 2
                LevelHeight2 = LadoCubo/2
                Points2 = []
                Edges2 = []
                for i in range(NLevels2):
                    for j in range(0,Density2): 
                        Angle2 = j*IncrementAngle2
                        Coords2 = [Radius2*np.cos(Angle2), 7.5+i*LevelHeight2, Radius2*np.sin(Angle2)]
                        Points2.append(Coords2)
                        if j>=1:
                            Edges2.append([i*Density2+j-1,i*Density2+j]) 
                            if j==Density2-1:
                                for k in range(0,NLevels2):
                                    Edges2.append([k*Density2,(1+k)*Density2-1])
                                    
                                    
                    
                
                FiberNode2.addObject("Mesh", position=Points2, name="Mesh", edges=Edges2)
                FiberNode2.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode2.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode2.addObject("BarycentricMapping")
                
                                
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitorotador_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPC=cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                #cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=False)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                
		#cubito/cavity_sensor
                cavity_sensor = cubito.addChild('cavity_sensor')
                
                cavity_sensor.addObject('MeshSTLLoader', name='loader', filename='Sensor_galinstan_45_cavity_y.stl')
                # cavity_sensor.addObject('MeshSTLLoader', name='loader', filename='Sensor_galinstan_-45_cavity_y.stl')
           
                cavity_sensor.addObject('MeshTopology', src='@loader', name='topo')
                
                cavity_sensor.addObject('MechanicalObject', name='cavity_sensor', translation=[0,LadoCubo/2, 0.5], rotation=[0, 90, 0]  )  # 45
                # cavity_sensor.addObject('MechanicalObject', name='cavity_sensor', translation=[22.5, 14.5 , 0.5], rotation=[0, 90, 0]  )  #-45
                
                cavity_sensor.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                cavity_sensor.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                

# 		#cubito/cubitoVisu
#                 cubitoVisu = cubito.addChild('visu')
#                 cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_rotador_visu.stl", name="loader")
#                 cubitoVisu.addObject("OglModel", src="@loader", color=[1,1,1,0.7])
#                 cubitoVisu.addObject("BarycentricMapping")
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC=SPC, EndEffectorMO=EndEffectorMO))
                
                
                return rootNode
