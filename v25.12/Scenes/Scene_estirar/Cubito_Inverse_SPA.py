import Sofa
import os
import csv
import Constants
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'


LadoCubo = Constants.LadoCubo

PSI = 5

class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.RootNode = kwargs['RootNode']
        self.SPA = kwargs['SPA']
        self.FEM_main = kwargs['FEM_main']  # 👈 NUEVO
        
        self.EndEffectorMO = kwargs['EndEffectorMO']
        self.EndEffectorMO2 = kwargs['EndEffectorMO2']       

        self.Maxpressure = 6890 * PSI   # ✔ Pa
        self.Increment = self.Maxpressure / 500
        self.Pressure = 0        
        self.Decreasing = False

        self.YM_values = [10000, 9000, 8000, 7000]
        self.current_YM_index = 0

        self.disp_max_list = []
        self.current_disp_max = 0

        self.initial_pos = self.EndEffectorMO.position.value[0].copy()

        self.FEM_main.youngModulus.value = self.YM_values[0]
        print(f"YM inicial: {self.YM_values[0]}")

        # CSV
        self.csv_file_path = "YM_vs_disp.csv"
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["YM", "Max_Displacement"])

    def update_pressure_increase(self):
        self.Pressure += self.Increment
        if self.Pressure > self.Maxpressure:
            self.Pressure = self.Maxpressure
        self.SPA.value = [self.Pressure]

    def update_pressure_decrease(self):
        self.Pressure -= self.Increment
        if self.Pressure < 0:
            self.Pressure = 0
        self.SPA.value = [self.Pressure]


    def onAnimateBeginEvent(self, eventType):

        pos = self.EndEffectorMO.position.value[0]
        disp = pos[1] - self.initial_pos[1]

        if disp > self.current_disp_max:
            self.current_disp_max = disp

        if not self.Decreasing:
            self.update_pressure_increase()
            if self.Pressure >= self.Maxpressure:
                self.Decreasing = True
        else:
            self.update_pressure_decrease()

            if self.Pressure <= 0:

                # Guardar resultado
                YM_actual = self.YM_values[self.current_YM_index]
                self.disp_max_list.append(self.current_disp_max)

                print(f"YM: {YM_actual} -> Disp max: {self.current_disp_max}")

                with open(self.csv_file_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([YM_actual, self.current_disp_max])

                # Reset
                self.current_disp_max = 0
                self.Decreasing = False
                self.Pressure = 0

                # Siguiente YM
                self.current_YM_index += 1

                if self.current_YM_index >= len(self.YM_values):
                    print("Simulación completa")
                    print(self.disp_max_list)
                    self.RootNode.animate = False
                    return

                # Cambiar YM
                new_YM = self.YM_values[self.current_YM_index]
                self.FEM_main.youngModulus.value = new_YM

                print(f"Cambiando a YM: {new_YM}")

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
                # rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
             
                rootNode.addObject('InteractiveCamera',name='cam', position=[0,0,1], projectionType=1)
                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject("QPInverseProblemSolver", printLog=1, epsilon=0.1, maxIterations=100, tolerance = 0.0000001)
                rootNode.dt = 0.01
                
        #cubito
                cubito = rootNode.addChild('cubito')
                cubito.addObject('EulerImplicitSolver', name='odesolver')
                cubito.addObject('SparseLDLSolver', name='preconditioner', template='CompressedRowSparseMatrixMat3x3d')
                cubito.addObject('PCGLinearSolver', iterations=50, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner')

                loader = cubito.addObject('MeshVTKLoader', name='loader', filename='CubitoEstirar.vtk')
                Container = cubito.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@loader.tetrahedra', name='container')
                cubito.addObject('TetrahedronSetTopologyModifier')

                MO = cubito.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                cubito.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = cubito.addObject('BoxROI', name='boxROIStiffness', box=[-12, 19.75, -12,  12, 20.25, 12], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                boxROIMain = cubito.addObject('BoxROI', name='boxROIMain', box=[-11, 0.25, -11,  11, 19.75, 11], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
           
                Container.init()
                MO.init()
                boxROIStiffness.init()
                boxROIMain.init()
                
                YM_base = 17775
                YM_stiffROI = 9000 *100

                boxROI = cubito.addObject('BoxROI', name='boxROI', box=[-12,  -0.25, -12,  12, 0.25, 12], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                cubito.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                cubito.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')   
                
                modelStiff = cubito.addChild('modelStiff')
                modelStiff.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIStiffness.tetrahedraInROI", name='container')
                modelStiff.addObject('ParallelTetrahedronFEMForceField', template = 'Vec3d', name='FEM_stiff', method='large', poissonRatio=0.49, youngModulus=YM_stiffROI) 
                
                modelSubTopo1 = cubito.addChild('modelSubTopo1')
                modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position', tetrahedra="@../boxROIMain.tetrahedraInROI", name='container')
                FEM_main = modelSubTopo1.addObject('ParallelTetrahedronFEMForceField', template='Vec3d',  name='FEM_main', method='large', poissonRatio=0.49, youngModulus=YM_base)


        #cubito/fibers
        
                FiberNode = cubito.addChild("FiberReinforcementNode")    
                Density = 30
                IncrementAngle = 2*np.pi/Density
                Radius = 8
                NLevels = 7
                LevelHeight = 2
                Points = []
                Edges = []
                for i in range(NLevels):
                    for j in range(0,30): 
                        Angle = j*IncrementAngle
                        Coords = [Radius*np.cos(Angle), 4+i*LevelHeight, Radius*np.sin(Angle)]
                        Points.append(Coords)
                        if j>=1:
                            Edges.append([i*Density+j-1,i*Density+j])
                            if j==29:
                                Edges.append([i*Density+j, i*Density+j-Density+1])
                                
                                
                            
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
                #cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=False)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)
                
        # Effector
        # bunny/effector
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[0, 22, 0], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)
                goal.addObject('UncoupledConstraintCorrection')
                
         # Punto "End-effector"
                 
                EndEffectorNode = cubito.addChild("EndEffectorNode")
                EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[0,LadoCubo,0]], showObject=True, showObjectScale=10)
                EndEffectorNode.addObject("BarycentricMapping")
                
                EndEffectorNode2 = cubito.addChild("EndEffectorNode_2")
                EndEffectorMO2 = EndEffectorNode2.addObject("MechanicalObject", position=[[0,LadoCubo/2,LadoCubo/2]], showObject=True, showObjectScale=10)
                EndEffectorNode2.addObject("BarycentricMapping")

		#cubito/cubitoVisu
                cubitoVisu = cubito.addChild('visu')
                cubitoVisu.addObject("MeshSTLLoader", filename="Cubito_Estirar_visu.stl", name="loader")
                cubitoVisu.addObject("OglModel", src="@loader")
                cubitoVisu.addObject("BarycentricMapping")
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPA=SPA, EndEffectorMO=EndEffectorMO , EndEffectorMO2=EndEffectorMO2, FEM_main=FEM_main))
                
                return rootNode