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
PSI = 5.5
Displa_z = 2.5
Displa_x = 6


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
        self.Increment = self.Maxpressure/500
        self.Pressure = 0        
        self.Decreasing = False
 
        self.EndEffectorMO = kwargs['EndEffectorMO']
        
        # Definir ruta de archivo csv 
        self.csv_file_path = "end_effector_data_Shear_YMA.csv"


        # Crear archivo CSV y escribir encabezados si no existe
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Pressure","Position_X", "Position_Y" ,"Position_Z"])
        
        print('Finished Init')
        
    def save_end_effector_data(self, time):
        position = self.EndEffectorMO.position.value
        
        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time,self.Pressure,position[0][0],position[0][1],position[0][2]])
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
                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.00001)
                rootNode.dt = 0.01

		#cubito
                cubito = rootNode.addChild('cubito')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner')
                cubito.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True, update_step=1)

                Loader = cubito.addObject('MeshVTKLoader', name='loader', filename='CubitoShear.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-14, 20.5, -14,  14, 24, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain = cubito.addObject('BoxROI', name='boxROIMain', box=[-13, -2, -13,  13, 22, 13], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
             
                Container.init()
                MO.init()
                boxROIStiffness.init()
                boxROIMain.init()
            
                YM_base  = 4620.03     #5419.85
                YM_stiffROI = 4000 * 100
                
                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-14, 0, -14,  14, 2, 14], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')  
                
                modelStiff = cubito.addChild('modelStiff')
                modelStiff.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness.tetrahedraInROI", name='container')
                modelStiff.addObject('TetrahedronFEMForceField', template = 'Vec3d', name='FEM_stiff', method='large', poissonRatio=0.45, youngModulus=YM_stiffROI) 
                
                modelSubTopo1 = cubito.addChild('modelSubTopo1')
                modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain.tetrahedraInROI", name='container')
                modelSubTopo1.addObject('TetrahedronFEMForceField', template='Vec3d',  name='FEM_main', method='large', poissonRatio=0.45, youngModulus=YM_base)

                
        #cubito/fibers

                Radius = Constants.RadioCilindro
                
                ring_density = 16          # nodos por anillo
                diameter_density = 8       # fibras diametrales en tapas
                n_rings = 6                # anillos a lo largo del cilindro
                
                y_start = 3
                y_end = 11
                
                stiffness_ring = 5e5
                stiffness_diameter = 5e5
                
                cube_center = np.array([10.8, 22.3, 0.0])
                
                # Rotación -45°
                angle_rad = np.radians(-45)
                R = np.array([
                    [np.cos(angle_rad), -np.sin(angle_rad), 0],
                    [np.sin(angle_rad),  np.cos(angle_rad), 0],
                    [0,                  0,                 1]
                ])
                
                 
                def create_ring(points, edges, center, radius, y, density, R):
                
                    offset = len(points)
                    dtheta = 2 * np.pi / density
                
                    for i in range(density):
                        theta = i * dtheta
                
                        p = np.array([
                            center[0] + radius * np.cos(theta),
                            y,
                            center[2] + radius * np.sin(theta)
                        ])
                
                        p_rot = R @ (p - center) + center
                        points.append(p_rot.tolist())
                
                        edges.append([
                            offset + i,
                            offset + (i + 1) % density
                        ])
                
                
                def create_ring_with_diameters(points, edges, center, radius, y, ring_density,
                                               diameter_density, R):
                    offset_ring = len(points)
                    dtheta_ring = 2 * np.pi / ring_density
                
                    for i in range(ring_density):
                        theta = i * dtheta_ring
                
                        p = np.array([
                            center[0] + radius * np.cos(theta),
                            y,
                            center[2] + radius * np.sin(theta)
                        ])
                
                        p_rot = R @ (p - center) + center
                        points.append(p_rot.tolist())
                
                        edges.append([
                            offset_ring + i,
                            offset_ring + (i + 1) % ring_density
                        ])
                
                    dtheta = np.pi / diameter_density
                    ds = 2 * radius / diameter_density
                
                    for j in range(diameter_density):
                        theta = j * dtheta
                
                        prev_idx = None
                
                        for k in range(diameter_density + 1):
                            s = -radius + k * ds
                
                            p = np.array([
                                center[0] + s * np.cos(theta),
                                y,
                                center[2] + s * np.sin(theta)
                            ])
                
                            p_rot = R @ (p - center) + center
                            idx = len(points)
                            points.append(p_rot.tolist())
                
                            if prev_idx is not None:
                                edges.append([prev_idx, idx])
                
                            prev_idx = idx
                 
                FiberBody = cubito.addChild("FiberBody")
                
                PointsBody = []
                EdgesBody = []
                
                height = y_end - y_start
                
                for i in range(n_rings):
                    y = y_start + i * height / (n_rings - 1)
                
                    create_ring(points=PointsBody, edges=EdgesBody, center=cube_center,radius=Radius,
                                y=y, density=ring_density, R=R)
                
                FiberBody.addObject("Mesh", position=PointsBody, edges=EdgesBody)
                FiberBody.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                FiberBody.addObject("MeshSpringForceField",linesStiffness=stiffness_ring)
                FiberBody.addObject("BarycentricMapping")
                
                
                CapBottom = cubito.addChild("CapBottom")
                
                PointsB = []
                EdgesB = []
                
                create_ring_with_diameters(points=PointsB, edges=EdgesB, center=cube_center,radius=Radius,
                                           y=y_start-2, ring_density=ring_density,diameter_density=diameter_density, R=R)
                
                CapBottom.addObject("Mesh", position=PointsB, edges=EdgesB)
                CapBottom.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                CapBottom.addObject("MeshSpringForceField", linesStiffness=stiffness_diameter)
                CapBottom.addObject("BarycentricMapping")
                
                CapTop = cubito.addChild("CapTop")
                
                PointsT = []
                EdgesT = []
                
                create_ring_with_diameters(points=PointsT,edges=EdgesT,center=cube_center,radius=Radius,
                                           y=y_end+2, ring_density=ring_density, diameter_density=diameter_density,R=R)
                
                CapTop.addObject("Mesh", position=PointsT, edges=EdgesT)
                CapTop.addObject("MechanicalObject", showObject=True, showObjectScale=10)
                CapTop.addObject("MeshSpringForceField",linesStiffness=stiffness_diameter)
                CapTop.addObject("BarycentricMapping")


        # Punto "End-effector"
                
                EndEffectorNode = cubito.addChild("EndEffectorNode")
                EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,LadoCubo,0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")
                
		#cubito/cavity          
        
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoShear_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPC = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[Displa_x, LadoCubo+Displa_z , 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)

		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_Shear_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC=SPC,  EndEffectorMO=EndEffectorMO))
                
                return rootNode
