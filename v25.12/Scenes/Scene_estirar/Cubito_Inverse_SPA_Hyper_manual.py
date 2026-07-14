import Sofa 
import os 
import csv 
import Constants 
import numpy as np 
 
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/' 
 
LadoCubo = Constants.LadoCubo 
 
 
class Controller(Sofa.Core.Controller): 
    def __init__(self, *args, **kwargs): 
        super().__init__(*args, **kwargs) 
        print(" Python::__init__::" + str(self.name.value)) 
        
        self.animation_finished = False 
        
        self.RootNode = kwargs['RootNode'] 
        self.SPA = kwargs['SPA'] 
        self.FEM_main = kwargs['FEM_main']
        
        self.Maxpressure = 50
        self.Increment = self.Maxpressure/500
        self.Pressure = 0 
        self.Decreasing = False 
        
        self.EndEffectorMO  = kwargs['EndEffectorMO'] 
        self.EndEffectorMO2 = kwargs['EndEffectorMO2'] 
 
        # Rangos de presión compartidos por YM y maxPressure
        self.pressure_ranges = [0, 15, 30, 40, 50]
 
        # YM ajustados desde datos FEM (regresión lineal por tramos)
        # Tramo 1 (0-15):  pendiente 0.353  E = 10075 Pa
        # Tramo 2 (15-30): pendiente 0.920  E =  3867 Pa
        # Tramo 3 (30-40): pendiente 1.540  E =  2309 Pa
        # Tramo 4 (40-50): pendiente 2.292  E =  1552 Pa
        self.YM_values = [1007, 387, 231, 155]   # factor /10 para diagnóstico
 
        # maxPressure del SurfacePressureActuator por tramo
        # Un valor por cada intervalo: [0→15, 15→30, 30→40, 40→50]
        self.SPA_maxPressure_values = [15, 30, 40, 50]
        
        self.csv_file_path = "end_effector_data_Estirar_SPA_hyper_manual.csv" 
        
        if not os.path.exists(self.csv_file_path): 
            with open(self.csv_file_path, mode='w', newline='') as file: 
                writer = csv.writer(file) 
                writer.writerow(["Time", "Pressure",
                                  "P1_Position_X", "P1_Position_Y", "P1_Position_Z",
                                  "P2_Position_X", "P2_Position_Y", "P2_Position_Z"]) 
                
        print('Finished Init') 
 
    def save_end_effector_data(self, time): 
        position  = self.EndEffectorMO.position.value 
        position2 = self.EndEffectorMO2.position.value 
        
        try: 
            with open(self.csv_file_path, mode='a', newline='') as file: 
                writer = csv.writer(file) 
                writer.writerow([time, self.Pressure,
                                  position[0][0],  position[0][1],  position[0][2],
                                  position2[0][0], position2[0][1], position2[0][2]]) 
        except Exception as e: 
            print(f"Error al escribir en archivo csv: {e}") 
 
    def update_pressure_increase(self, pressure, SPA): 
        pressure += self.Increment 
        if pressure > self.Maxpressure and not self.animation_finished: 
            pressure = self.Maxpressure 
        SPA.value = [pressure] 
        return pressure 
    
    def update_pressure_decrease(self, pressure, SPA): 
        pressure -= self.Increment 
        if pressure < 0 and not self.animation_finished: 
            pressure = 0 
        SPA.value = [pressure] 
        return pressure 
 
    def update_young_modulus(self):
        for i in range(len(self.pressure_ranges) - 1):
            if self.pressure_ranges[i] <= self.Pressure < self.pressure_ranges[i + 1]:
                target_YM  = self.YM_values[i]
                current_YM = self.FEM_main.youngModulus.value
                alpha      = 0.1
                self.FEM_main.youngModulus.value = current_YM * (1 - alpha) + target_YM * alpha
                return
 
    def update_max_pressure(self):
        for i in range(len(self.pressure_ranges) - 1):
            if self.pressure_ranges[i] <= self.Pressure < self.pressure_ranges[i + 1]:
                self.SPA.maxPressure.value = float(self.SPA_maxPressure_values[i])
                return
 
    def onAnimateBeginEvent(self, eventType): 
        
        current_time = self.RootNode.time.value 
        print("Pressure:", self.Pressure)
        print("YM:", self.FEM_main.youngModulus.value)
        print("maxPressure:", self.SPA.maxPressure.value)
        
        if self.animation_finished: 
            self.RootNode.dt = 0 
            self.Pressure = 0 
            return 
        
        self.save_end_effector_data(current_time) 
        
        self.update_young_modulus()
        self.update_max_pressure()
        
        if not self.Decreasing: 
            self.Pressure = self.update_pressure_increase(self.Pressure, self.SPA) 
            if self.Pressure >= self.Maxpressure: 
                self.Decreasing = True 
        else: 
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
                    Sofa.Component.Collision.Geometry
                    Sofa.Component.Topology.Mapping
                    Sofa.GL.Component.Rendering3D
                    Sofa.GL.Component.Shader
                    MultiThreading"""
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
             
                rootNode.addObject('InteractiveCamera', name='cam', position=[0,0,1], projectionType=1)
                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject("QPInverseProblemSolver", printLog=0, epsilon=0.1, maxIterations=1000, tolerance=1e-5)
                rootNode.dt = 0.001
                
        #cubito
                cubito = rootNode.addChild('cubito')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner', template='CompressedRowSparseMatrixMat3x3d')
                cubito.addObject('PreconditionedMatrixFreeSystem', template='GraphScattered')

                cubito.addObject('PCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True)
                loader    = cubito.addObject('MeshVTKLoader', name='loader', filename='CubitoEstirar.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@loader.tetrahedra', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')
 
                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-12, 19.75, -12, 12, 20.25, 12], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain      = cubito.addObject('BoxROI', name='boxROIMain',      box=[-11,  0.25, -11, 11, 19.75, 11], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
           
                Container.init()
                MO.init()
                boxROIStiffness.init()
                boxROIMain.init()
                
                YM_base     = 10075      
                YM_stiffROI = 9000 * 100
 
                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-12, -0.25, -12, 12, 0.25, 12], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')   
                
                modelStiff = cubito.addChild('modelStiff')
                modelStiff.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness.tetrahedraInROI", name='container')
                modelStiff.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', name='FEM_stiff', method='large', poissonRatio=0.49, youngModulus=YM_stiffROI) 
                
                modelSubTopo1 = cubito.addChild('modelSubTopo1')
                modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain.tetrahedraInROI", name='container')
                FEM_main = modelSubTopo1.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', name='FEM_main', method='large', poissonRatio=0.49, youngModulus=YM_base)
 
        #cubito/fibers
                FiberNode     = cubito.addChild("FiberReinforcementNode")    
                Density       = 30
                IncrementAngle = 2 * np.pi / Density
                Radius        = 8
                NLevels       = 7
                LevelHeight   = 2
                Points        = []
                Edges         = []
                for i in range(NLevels):
                    for j in range(0, 30): 
                        Angle  = j * IncrementAngle
                        Coords = [Radius * np.cos(Angle), 4 + i * LevelHeight, Radius * np.sin(Angle)]
                        Points.append(Coords)
                        if j >= 1:
                            Edges.append([i * Density + j - 1, i * Density + j])
                            if j == 29:
                                Edges.append([i * Density + j, i * Density + j - Density + 1])
                                
                FiberNode.addObject("MeshTopology", position=Points, name="Mesh", edges=Edges)
                FiberNode.addObject("MechanicalObject", showObject=True, showObjectScale=10)                
                FiberNode.addObject("MeshSpringForceField", linesStiffness=1e9)
                FiberNode.addObject("BarycentricMapping")
                
        #cubito/cavity
                cavity = cubito.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoEstirar_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                SPA = cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles')
                cavity.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=True)
                
        #goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[0, 22, 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)
                goal.addObject('UncoupledConstraintCorrection', defaultCompliance=1e-3)
                
        #end-effectors
                EndEffectorNode  = cubito.addChild("EndEffectorNode")
                EndEffectorMO    = EndEffectorNode.addObject("MechanicalObject", position=[[0, LadoCubo, 0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")
                
                EndEffectorNode2 = cubito.addChild("EndEffectorNode_2")
                EndEffectorMO2   = EndEffectorNode2.addObject("MechanicalObject", position=[[0, LadoCubo/2, LadoCubo/2]], showObject=True, showObjectScale=10)
                EndEffectorNode2.addObject("BarycentricMapping")
 
        #cubito/visu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_Estirar_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                rootNode.addObject(Controller(
                    name="ActuationController",
                    RootNode=rootNode,
                    SPA=SPA,
                    EndEffectorMO=EndEffectorMO,
                    EndEffectorMO2=EndEffectorMO2,
                    FEM_main=FEM_main
                ))
                
                return rootNode
