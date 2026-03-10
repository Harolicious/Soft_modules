# -*- coding: utf-8 -*-
"""
Created on Wed Apr 10 13:37:13 2024

@author: lab_Harold
"""

# import Sofa
import Sofa.Core
import Constants
import os
import csv
import numpy as np

LadoCubo = Constants.LadoCubo
PSI = 7.4

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

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
        self.Increment = self.Maxpressure/500 #KPa
        self.Pressure = 0        
        self.Decreasing = False
        self.EndEffectorMO = kwargs['EndEffectorMO']
        self.EndEffectorMO2 = kwargs['EndEffectorMO2']        

        # Definir ruta de archivo csv 
        self.csv_file_path = "end_effector_data_Barril_YMA.csv"

        # Crear archivo CSV y escribir encabezados si no existe
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Pressure","P1_Position_X", "P1_Position_Y" ,"P1_Position_Z","P2_Position_X", "P2_Position_Y" ,"P2_Position_Z"])
        
        print('Finished Init')
        
    def save_end_effector_data(self, time):
        position = self.EndEffectorMO.position.value
        position2 = self.EndEffectorMO2.position.value
              
        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time,self.Pressure,position[0][0],position[0][1],position[0][2],position2[0][0],position2[0][1],position2[0][2]])
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
        # print(f"Current End-Effector position_2: {self.EndEffectorMO2.position.value}")        
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
                rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Mapping') # Needed to use components [Tetra2TriangleTopologicalMapping]
                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.0000001)
                rootNode.dt = 0.01

		#cubito
                cubito = rootNode.addChild('CubitoBarril')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner')
                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='Cubitobarril.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-14, 17, -14,  14, 22, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain = cubito.addObject('BoxROI', name='boxROIMain', box=[-13, 0.5, -13,  13, 19.5, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                
                Container.init()
                MO.init()
                boxROIStiffness.init()
                boxROIMain.init()
                
                YM_base = 4396.22
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

        # Punto "End-effector"
                
                EndEffectorNode = cubito.addChild("EndEffectorNode")
                EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,LadoCubo,0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")

        # Punto "End-effector"
                
                EndEffectorNode2 = cubito.addChild("EndEffectorNode_2")
                EndEffectorMO2 = EndEffectorNode2.addObject("MechanicalObject", position=[[0,LadoCubo/2,LadoCubo/2]], showObject=True, showObjectScale=10)
                EndEffectorNode2.addObject("BarycentricMapping")
                

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
                
                # giro por nivel: fibra j → j+2
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
                SPC = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                #cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=False)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)


		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_barril_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC=SPC, EndEffectorMO=EndEffectorMO, EndEffectorMO2=EndEffectorMO2 ))
                
                return rootNode