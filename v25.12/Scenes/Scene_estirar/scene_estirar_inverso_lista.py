import Sofa
import os
import csv
import Constants
import numpy as np

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'

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

        self.goal_y0 = 20.0
        self.MaxDesp = 7.67

        # ---- Lista de deformaciones (INPUT) — cargada desde datos experimentales ----
        # Archivo generado a partir de estirardsm10_c.xlsx (columna E = desplazamiento,
        # columna presion_kPa = presión experimental correspondiente, ya convertida
        # de PSI a kPa). Debe estar en la misma carpeta que esta escena.
        datos_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'presion_desplazamiento_kpa.csv')
        self.desp_list         = []
        self.presion_exp_list  = []
        with open(datos_path, mode='r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.desp_list.append(float(row['desplazamiento_mm']))
                self.presion_exp_list.append(float(row['presion_kPa']))

        self.step_index = 0
        self.GoalDesp   = 0.0

        # ---- Calibración YM(presion) — 20 tramos de 2.5 kPa ----
        # YM_values interpolado a partir de los 10 valores originales (5 kPa c/u),
        # ajustando un decaimiento exponencial con asíntota: YM(p) = A*exp(-k*p) + C
        # (A=22490.0, k=0.08453, C=7520.4, R²=0.997) y evaluándolo en los puntos
        # medios de los 20 nuevos tramos de 2.5 kPa, preservando la misma forma
        # de decaimiento que los 10 valores medidos originalmente.
        self.pressure_ranges = [0, 2.5, 5, 7.5, 10, 12.5, 15, 17.5, 20, 22.5, 25,
                                 27.5, 30, 32.5, 35, 37.5, 40, 42.5, 45, 47.5, 50]
        self.YM_values = [32000, 25800, 20600, 17500, 15500, 14200, 13050, 12150,
                          11300, 10800, 10200, 9800, 9350, 8950, 8600, 8300, 8050,
                          7800, 7600, 7400]
        self.SPA_maxPressure_values = [2.5, 5, 7.5, 10, 12.5, 15, 17.5, 20, 22.5, 25,
                                        27.5, 30, 32.5, 35, 37.5, 40, 42.5, 45, 47.5, 50]

        # maxPressure inicial — primer tramo
        self.SPA.maxPressure.value = np.float64(self.SPA_maxPressure_values[-1])

        self.csv_file_path = "end_effector_data_Estirar_inverso_lista.csv"
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Goal_Desp", "SPA_Pressure_calculated",
                                  "Presion_experimental_kPa", "YM",
                                  "P1_Position_X", "P1_Position_Y", "P1_Position_Z",
                                  "P2_Position_X", "P2_Position_Y", "P2_Position_Z"])

        print(f"YM_values: {self.YM_values}")
        print(f"Cantidad de pasos en desp_list: {len(self.desp_list)}")
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
        # Presión experimental correspondiente al paso actual (para comparar vs. la
        # calculada por el QP inverso). Si ya se agotó la lista, se usa el último valor.
        idx = min(self.step_index, len(self.presion_exp_list) - 1)
        presion_exp = self.presion_exp_list[idx]

        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time, self.GoalDesp, self.get_spa_pressure(),
                                  presion_exp, self.get_ym(),
                                  p1[0][0], p1[0][1], p1[0][2],
                                  p2[0][0], p2[0][1], p2[0][2]])
        except Exception as e:
            print(f"Error al escribir CSV: {e}")

    def update_young_modulus(self):
        # Se indexa según la deformación actual (GoalDesp), igual que antes
        peq = (self.GoalDesp / self.MaxDesp) * 50.0
        for i in range(len(self.pressure_ranges) - 1):
            if self.pressure_ranges[i] <= peq < self.pressure_ranges[i + 1]:
                target_YM  = self.YM_values[i]
                current_YM = self.get_ym()
                alpha      = 0.1
                new_YM     = float(current_YM * (1 - alpha) + target_YM * alpha)
                self.FEM_main.youngModulus.value = [new_YM]
                self.FEM_main.reinit()  # fuerza re-ensamblaje de K antes del próximo solve
                return

    def update_max_pressure(self):
        # maxPressure sube de forma uniforme con el GoalDesp actual
        p_equiv = (self.GoalDesp / self.MaxDesp) * 50.0
        p_equiv = max(1.0, p_equiv)
        self.SPA.maxPressure.value = np.float64(p_equiv / 10.0)

    def move_goal_from_list(self):
        # Toma el siguiente valor de deformación de la lista y lo aplica al goal
        self.GoalDesp = self.desp_list[self.step_index]
        self.GoalMO.position.value = [[0, self.goal_y0 + self.GoalDesp, 0]]
        self.step_index += 1

    def onAnimateBeginEvent(self, eventType):

        current_time = self.RootNode.time.value

        if self.animation_finished:
            self.RootNode.dt = 0
            return

        lambda_val   = float(np.array(self.SPA.pressure.value).flat[0])
        max_p        = float(np.array(self.SPA.maxPressure.value).flat[0])
        p1_y         = float(self.EndEffectorMO.position.value[0][1])
        goal_y       = self.goal_y0 + self.GoalDesp
        p_real       = lambda_val * 10.0
        idx          = min(self.step_index, len(self.presion_exp_list) - 1)
        p_exp        = self.presion_exp_list[idx]
        print(f"Paso: {self.step_index}/{len(self.desp_list)} | "
              f"GoalDesp: {self.GoalDesp:.4f} mm | "
              f"P_sim: {p_real:.4f} kPa | "
              f"P_exp: {p_exp:.4f} kPa | "
              f"Error_P: {p_real - p_exp:.4f} kPa | "
              f"YM: {self.get_ym():.2f} | "
              f"P1_Y: {p1_y:.4f} | "
              f"Goal_Y: {goal_y:.4f} | "
              f"Error_pos: {goal_y - p1_y:.4f}")

        self.save_data(current_time)
        self.update_max_pressure()

        if self.step_index < len(self.desp_list):
            self.move_goal_from_list()
        else:
            self.animation_finished = True

    def onAnimateEndEvent(self, eventType):
        # Actualizar YM al final del step para que el solver lo use en el siguiente
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

    rootNode.addObject('InteractiveCamera', name='cam', position=[0, 0, 1], projectionType=1)
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject("QPInverseProblemSolver", printLog=0, epsilon=0.1, maxIterations=1000, tolerance=1e-5)
    rootNode.dt = 0.001

    # cubito
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
    boxROIMain      = cubito.addObject('BoxROI', name='boxROIMain',      box=[-11, 0.25, -11, 11, 19.75, 11], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")

    Container.init()
    MO.init()
    boxROIStiffness.init()
    boxROIMain.init()

    YM_base     = 32000
    YM_stiffROI = 32000 * 100

    boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-12, -0.25, -12, 12, 0.25, 12], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
    cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
    cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')

    modelStiff = cubito.addChild('modelStiff')
    modelStiff.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness.tetrahedraInROI", name='container')
    modelStiff.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', name='FEM_stiff', method='large', poissonRatio=0.49, youngModulus=YM_stiffROI)

    modelSubTopo1 = cubito.addChild('modelSubTopo1')
    modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain.tetrahedraInROI", name='container')
    FEM_main = modelSubTopo1.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', name='FEM_main', method='large', poissonRatio=0.49, youngModulus=YM_base)
    rootNode.FEM_main = FEM_main  # referencia global para acceso desde Controller

    # cubito/fibers
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

    # cubito/cavity
    cavity = cubito.addChild('cavity')
    cavity.addObject('MeshSTLLoader', name='loader', filename='CubitoEstirar_Cavity.stl')
    cavity.addObject('MeshTopology', src='@loader', name='topo')
    cavity.addObject('MechanicalObject', name='cavity')
    SPA = cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles',
                            maxPressureVariation=0.5)
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=True)

    # goal
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    GoalMO = goal.addObject('MechanicalObject', name='goalMO',
                             position=[0, 20, 0],
                             showObject=True, showObjectScale=15)
    goal.addObject('SphereCollisionModel', radius=2.5, group=1)
    goal.addObject('UncoupledConstraintCorrection', defaultCompliance=1e-3)

    # end-effectors
    EndEffectorNode  = cubito.addChild("EndEffectorNode")
    EndEffectorMO    = EndEffectorNode.addObject("MechanicalObject", position=[[0, LadoCubo, 0]], showObject=True, showObjectScale=10)
    EndEffectorNode.addObject("BarycentricMapping")

    EndEffectorNode2 = cubito.addChild("EndEffectorNode_2")
    EndEffectorMO2   = EndEffectorNode2.addObject("MechanicalObject", position=[[0, LadoCubo / 2, LadoCubo / 2]], showObject=True, showObjectScale=10)
    EndEffectorNode2.addObject("BarycentricMapping")

    # effector
    effector = EndEffectorNode.addChild("effector")
    effector.addObject('PositionEffector', template='Vec3',
                        indices=[0],
                        effectorGoal='@../../../goal/goalMO.position')

    # cubito/visu
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
