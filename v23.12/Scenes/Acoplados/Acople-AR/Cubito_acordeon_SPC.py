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

PSI = 4
despla = 4.5 #desplazamiento deseado 

LadoCubo = Constants.LadoCubo
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
        self.Maxpressure = 6.89 * PSI  #Pa 
        self.Increment = self.Maxpressure/500
        self.Pressure = 0        
        self.Decreasing = False
        self.EndEffectorMO = kwargs['EndEffectorMO']      

        # Definir ruta de archivo csv 
        self.csv_file_path = "end_effector_data_Acordeon_YMA.csv"

        # Crear archivo CSV y escribir encabezados si no existe
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Pressure","Position_X", "Position_Y" ,"Position_Z","Angle"])
        
        print('Finished Init')
        
    def save_end_effector_data(self, time):
        position = self.EndEffectorMO.position.value
        
        PointA = position[0]
        PointB = position[1]
        
        delta_x = PointB[0] - PointA[0]
        delta_y = PointB[1] - PointA[1]
        delta_z = PointB[2] - PointA[2]
        
        Angle = np.rad2deg(np.arctan2(delta_y, delta_x))
        
        # SecondPointCoord = self.EndEffectorMO.position.value[1]
        # Angle = np.rad2deg(np.arctan2(SecondPointCoord[1],SecondPointCoord[0]))
        
        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time,self.Pressure,PointA[0],PointA[1],PointA[2],Angle])
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
                cubito = rootNode.addChild('CubitoAcordeon')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner')
                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='Cubitoacordeon.vtk')
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
                
                YM_base = 6560 #7648.08
                YM_stiffROI = 12000 * 100
                
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

                CylinderHeight = Constants.AlturaCilindro
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
                
                
                create_rings(
                    parent=cubito,
                    name="FiberReinforcementNode1",
                    radius=Radius,
                    n_rings=6,
                    ring_density=15,
                    y_start=y_start,
                    y_end = y_end,
                    stiffness=5e9
                )                
                
                create_helices_sector(
                    parent=cubito,
                    name="FiberReinforcementNode_Positive",
                    radius=Radius,
                    n_fibers=4,
                    density=10,
                    y_start=y_start,
                    height=fiber_height,
                    total_angle=np.pi/2,
                    sector_angle=np.pi/2,
                    angle_offset=-np.pi/2,
                    stiffness=1e9
                )
                

                create_helices_sector(
                    parent=cubito,
                    name="FiberReinforcementNode_Negative",
                    radius=Radius,
                    n_fibers=4,
                    density=10,
                    y_start=y_start,
                    height=fiber_height,
                    total_angle=-np.pi/2,
                    sector_angle=np.pi/2,
                    angle_offset=0.0,
                    stiffness=1e9
                )

                
        # Punto "End-effector"
                
                EndEffectorNode = cubito.addChild("EndEffectorNode")
                EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,LadoCubo,0], [10,LadoCubo,0], [-10,LadoCubo,0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")
           
               
		#cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Cubitoacordeon_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPC = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[4, LadoCubo+despla , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)

		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_acordeon_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")

                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC=SPC, EndEffectorMO=EndEffectorMO))

                return rootNode