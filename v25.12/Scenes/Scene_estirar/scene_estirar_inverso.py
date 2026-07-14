import Sofa 
import os 
import csv 
import Constants 
import numpy as np 
from scipy.interpolate import interp1d

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/' 

LadoCubo = Constants.LadoCubo 


class Controller(Sofa.Core.Controller): 
    def __init__(self, *args, **kwargs): 
        super().__init__(*args, **kwargs) 
        print(" Python::__init__::" + str(self.name.value)) 
        
        self.animation_finished = False 
        
        self.RootNode  = kwargs['RootNode'] 
        self.SPA       = kwargs['SPA'] 
        self.FEM_main  = kwargs['FEM_main']
        self.GoalMO    = kwargs['GoalMO']

        self.goal_y0    = 20.0
        self.MaxDesp    = 7.67
        self.Increment  = self.MaxDesp / 5000
        self.GoalDesp   = 0.0
        self.Decreasing = False

        # 10 tramos de 5 kPa — ajustar estos valores para aproximar la curva FEM
        self.pressure_ranges = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50]
        self.YM_values       = [12000, 1, 1, 1, 1, 1, 1, 1, 1, 1]

        # Interpolador inverso FEM: desplazamiento → presión
        # Dado que P1 se movió X mm, devuelve la presión que le correspondería en el FEM
        # Es la única forma de inyectar la no linealidad hiperelástica en modo inverso
        _fem_P = np.array([0,0.51,1.52,2.53,3.54,4.55,5.55,6.56,7.57,8.58,9.59,10.6,
                            11.6,12.61,13.62,14.63,15.64,16.65,17.65,18.66,19.67,20.68,
                            21.69,22.7,23.7,24.71,25.72,26.73,27.74,28.74,29.75,30.76,
                            31.77,32.78,33.79,34.79,35.8,36.81,37.82,38.83,39.83,40.84,
                            41.85,42.86,43.87,44.87,45.88,46.89,47.9,48.91,49.99])
        _fem_D = np.array([0,0.003,0.011,0.024,0.042,0.065,0.093,0.127,0.167,0.214,
                            0.267,0.328,0.396,0.472,0.556,0.648,0.749,0.858,0.977,1.105,
                            1.243,1.39,1.547,1.714,1.891,2.078,2.276,2.484,2.702,2.931,
                            3.171,3.421,3.682,3.954,4.236,4.529,4.832,5.146,5.471,5.806,
                            6.152,6.508,6.875,7.252,7.472,7.524,7.57,7.608,7.638,7.658,7.665])
        self.fem_interp = interp1d(_fem_D, _fem_P, bounds_error=False, fill_value=(0.0, 50.0))

        # maxPressure inicial
        self.SPA.maxPressure.value = np.float64(0.1)

        self.csv_file_path = "end_effector_data_Estirar_inverso.csv" 
        if not os.path.exists(self.csv_file_path): 
            with open(self.csv_file_path, mode='w', newline='') as file: 
                writer = csv.writer(file) 
                writer.writerow(["Time", "Goal_Desp", "SPA_Pressure_calculated",
                                  "P1_Position_X", "P1_Position_Y", "P1_Position_Z",
                                  "P2_Position_X", "P2_Position_Y", "P2_Position_Z"])

        print('Finished Init')

    def get_spa_pressure(self):
        try:
            lambda_val = float(np.array(self.SPA.pressure.value).flat[0])
            return lambda_val * 10.0  # P_real (kPa) = lambda × 10
        except Exception:
            return 0.0

    def get_ym(self):
        return float(np.array(self.FEM_main.youngModulus.value).flat[0])

    def save_data(self, time):
        try:
            p1 = self.EndEffectorMO.position.value
            p2 = self.EndEffectorMO2.position.value
        except Exception:
            p1 = [[0, 0, 0]]
            p2 = [[0, 0, 0]]
        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time, self.GoalDesp, self.get_spa_pressure(),
                                  p1[0][0], p1[0][1], p1[0][2],
                                  p2[0][0], p2[0][1], p2[0][2]])
        except Exception as e:
            print(f"Error al escribir CSV: {e}")

    def update_young_modulus(self):
        # Seleccionar tramo según desplazamiento real de P1
        p1_desp = float(self.EndEffectorMO.position.value[0][1]) - self.goal_y0
        p1_desp = max(0.0, p1_desp)
        peq = (p1_desp / self.MaxDesp) * 50.0
        for i in range(len(self.pressure_ranges) - 1):
            if self.pressure_ranges[i] <= peq < self.pressure_ranges[i + 1]:
                target_YM  = self.YM_values[i]
                current_YM = self.get_ym()
                alpha      = 0.1
                new_YM     = float(current_YM * (1 - alpha) + target_YM * alpha)
                self.FEM_main.youngModulus.value = [new_YM]
                return

    def update_max_pressure(self):
        # Usar el máximo entre desplazamiento real P1 y GoalDesp
        # Evita el círculo vicioso: P1 no se mueve → maxP bajo → P1 no se mueve
        p1_desp   = max(0.0, float(self.EndEffectorMO.position.value[0][1]) - self.goal_y0)
        desp_ref  = max(p1_desp, self.GoalDesp)
        p_fem     = float(self.fem_interp(desp_ref))
        p_fem     = max(1.0, p_fem)
        # P_real = lambda × 10  →  maxPressure = p_fem / 10
        self.SPA.maxPressure.value = np.float64(p_fem / 10.0)

    def move_goal_up(self):
        self.GoalDesp += self.Increment
        if self.GoalDesp > self.MaxDesp:
            self.GoalDesp = self.MaxDesp
        self.GoalMO.position.value = [[0, self.goal_y0 + self.GoalDesp, 0]]

    def move_goal_down(self):
        self.GoalDesp -= self.Increment
        if self.GoalDesp < 0:
            self.GoalDesp = 0.0
        self.GoalMO.position.value = [[0, self.goal_y0 + self.GoalDesp, 0]]

    def onAnimateBeginEvent(self, eventType): 
        
        current_time = self.RootNode.time.value

        if self.animation_finished: 
            self.RootNode.dt = 0
            return

        lambda_val = float(np.array(self.SPA.pressure.value).flat[0])
        max_p      = float(np.array(self.SPA.maxPressure.value).flat[0])
        p1_y       = float(self.EndEffectorMO.position.value[0][1])
        goal_y     = self.goal_y0 + self.GoalDesp
        print(f"GoalDesp: {self.GoalDesp:.4f} mm | "
              f"lambda: {lambda_val:.4f} | "
              f"maxP: {max_p:.4f} | "
              f"P_real: {lambda_val * 10.0:.4f} kPa | "
              f"YM: {self.get_ym():.2f} | "
              f"P1_Y: {p1_y:.4f} | "
              f"Goal_Y: {goal_y:.4f} | "
              f"Error: {goal_y - p1_y:.4f}")

        self.save_data(current_time)
        self.update_max_pressure()

        if not self.Decreasing:
            self.move_goal_up()
            if self.GoalDesp >= self.MaxDesp:
                self.Decreasing = True
        else:
            self.move_goal_down()
            if self.GoalDesp <= 0:
                self.Decreasing = False
                self.animation_finished = True

    def onAnimateEndEvent(self, eventType):
        # Actualizar YM al final del step para que el solver lo use en el siguiente
        # Así el solver siempre tiene el YM correcto para el estado actual
        if not self.animation_finished:
            self.update_young_modulus()


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
                cubito.addObject('PCGLinearSolver', iterations=15, name='linearsolver',
                                  tolerance=1e-5, preconditioner='@preconditioner', use_precond=True)

                loader    = cubito.addObject('MeshVTKLoader', name='loader', filename='CubitoEstirar.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@loader.tetrahedra', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)

                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-12, 19.75, -12, 12, 20.25, 12], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")

                Container.init()
                MO.init()
                boxROIStiffness.init()

                YM_base     = 12000
                YM_stiffROI = 12000 * 100

                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-12, -0.25, -12, 12, 0.25, 12], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')

                modelStiff = cubito.addChild('modelStiff')
                modelStiff.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness.tetrahedraInROI", name='container')
                modelStiff.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', name='FEM_stiff', method='large', poissonRatio=0.49, youngModulus=YM_stiffROI)

                boxROIMain = cubito.addObject('BoxROI', name='boxROIMain', box=[-11, 0.25, -11, 11, 19.75, 11], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain.init()

                modelSubTopo1 = cubito.addChild('modelSubTopo1')
                modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain.tetrahedraInROI", name='container')
                FEM_main = modelSubTopo1.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', name='FEM_main', method='large', poissonRatio=0.49, youngModulus=YM_base)

        #cubito/fibers
                FiberNode      = cubito.addChild("FiberReinforcementNode")
                Density        = 30
                IncrementAngle = 2 * np.pi / Density
                Radius         = 8
                NLevels        = 7
                LevelHeight    = 2
                Points         = []
                Edges          = []
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
                SPA = cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles',
                                        maxPressureVariation=0.5)
                cavity.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=True)

        #goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                GoalMO = goal.addObject('MechanicalObject', name='goalMO',
                                         position=[0, 20, 0],
                                         showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)
                goal.addObject('UncoupledConstraintCorrection', defaultCompliance=1e-3)

        #end-effectors
                EndEffectorNode  = cubito.addChild("EndEffectorNode")
                EndEffectorMO    = EndEffectorNode.addObject("MechanicalObject", position=[[0, LadoCubo, 0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")

                EndEffectorNode2 = cubito.addChild("EndEffectorNode_2")
                EndEffectorMO2   = EndEffectorNode2.addObject("MechanicalObject", position=[[0, LadoCubo/2, LadoCubo/2]], showObject=True, showObjectScale=10)
                EndEffectorNode2.addObject("BarycentricMapping")

        #effector
                effector = EndEffectorNode.addChild("effector")
                effector.addObject('PositionEffector', template='Vec3',
                                    indices=[0],
                                    effectorGoal='@../../../goal/goalMO.position')

        #cubito/visu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_Estirar_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")

                ctrl = Controller(
                    name="ActuationController",
                    RootNode=rootNode,
                    SPA=SPA,
                    FEM_main=FEM_main,
                    GoalMO=GoalMO,
                )
                ctrl.EndEffectorMO  = EndEffectorMO
                ctrl.EndEffectorMO2 = EndEffectorMO2
                rootNode.addObject(ctrl)

                return rootNode
