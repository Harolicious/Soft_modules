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
PSI = 4.5
despla = 11 #desplazamiento deseado 

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
        self.csv_file_path = "end_effector_data_Rotador_YMA.csv"

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
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.00001)
                rootNode.dt = 0.01 

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
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-14, 18, -14,  14, 22, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain = cubito.addObject('BoxROI', name='boxROIMain', box=[-13, 1, -13,  13, 19, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")

                Container.init()
                MO.init()
                boxROIStiffness.init()
                boxROIMain.init()
                
                YM_base = 5420 #5419.85
                YM_stiffROI = 6000 * 100
                
                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-14,  -2, -14,  14, 2, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')   
                
                modelStiff = cubito.addChild('modelStiff')
                modelStiff.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness.tetrahedraInROI", name='container')
                modelStiff.addObject('TetrahedronFEMForceField', template = 'Vec3d', name='FEM_stiff', method='large', poissonRatio=0.45, youngModulus=YM_stiffROI) 
                
                modelSubTopo1 = cubito.addChild('modelSubTopo1')
                modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain.tetrahedraInROI", name='container')
                modelSubTopo1.addObject('TetrahedronFEMForceField', template='Vec3d',  name='FEM_main', method='large', poissonRatio=0.45, youngModulus=YM_base)

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
               
                Density = 12        # Discretización vertical
                Repeat = 8           # Número de fibras
                Turns = 0.3        # Número de vueltas helicoidales
               
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
                FiberNode.addObject("MeshSpringForceField", linesStiffness=7e5)
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
                RingNode.addObject("MeshSpringForceField", linesStiffness=7e6)
                RingNode.addObject("BarycentricMapping")
              
        # Punto "End-effector"
                
                EndEffectorNode = cubito.addChild("EndEffectorNode")
                EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,LadoCubo,0], [10,LadoCubo,0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")
                
                
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitorotador_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPC=cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
        
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[0, LadoCubo+despla , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)
                
		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_rotador_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC=SPC, EndEffectorMO=EndEffectorMO))
                
                return rootNode