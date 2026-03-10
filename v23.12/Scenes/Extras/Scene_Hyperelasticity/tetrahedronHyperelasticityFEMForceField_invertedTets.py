#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  1 19:13:20 2024

@author: lab
"""

def createScene(root_node):

   root = root_node.addChild('root', dt="0.00005",  gravity="0 0 0")

   plugins = root.addChild('plugins')

   plugins.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Projective")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Direct")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.LinearSystem")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.Mass")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.HyperElastic")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.StateContainer")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Dynamic")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Grid")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.Topology.Mapping")
   plugins.addObject('RequiredPlugin', name="Sofa.Component.Visual")

   root.addObject('VisualStyle', displayFlags="showForceFields showBehaviorModels")
   root.addObject('DefaultAnimationLoop', )

   stable_neo_hookean = root.addChild('StableNeoHookean')

   stable_neo_hookean.addObject('EulerImplicitSolver', name="odesolver")
   stable_neo_hookean.addObject('ConstantSparsityPatternSystem', template="CompressedRowSparseMatrixd", name="A")
   stable_neo_hookean.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
   stable_neo_hookean.addObject('RegularGridTopology', name="hexaGrid", min="0 0 0", max="1 1 2.7", n="6 6 16", p0="0 0 0")
   stable_neo_hookean.addObject('RegularGridTopology', name="hexaGridRest", min="0 0 0", max="1 1 -2.7", n="6 6 16", p0="0 0 0")
   stable_neo_hookean.addObject('MechanicalObject', name="mechObj", rest_position="@hexaGrid.position", position="@hexaGridRest.position")
   stable_neo_hookean.addObject('MeshMatrixMass', totalMass="1.0")

   tetras = StableNeoHookean.addChild('tetras')

   tetras.addObject('TetrahedronSetTopologyContainer', name="Container")
   tetras.addObject('TetrahedronSetTopologyModifier', name="Modifier")
   tetras.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3", name="GeomAlgo")
   tetras.addObject('Hexa2TetraTopologicalMapping', name="default28", input="@../hexaGrid", output="@Container", printLog="0")
   tetras.addObject('TetrahedronHyperelasticityFEMForceField', name="FEM", ParameterSet="1644295.30201342 33557.0469798658", materialName="StableNeoHookean")

   stable_neo_hookean.addObject('BoxROI', drawBoxes="1", box="0 0 0 1 1 0.05", name="box")
   stable_neo_hookean.addObject('FixedProjectiveConstraint', indices="@box.indices")