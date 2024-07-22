import Sofa
import Sofa.Gui

def main():
    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 800)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')

    rootNode.addObject("VisualGrid", nbSubdiv=10, size=1000)

    rootNode.gravity = [0.0, -9.81, 0.0]
    rootNode.dt = 0.01

    # Load necessary plugins individually
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    # Collision pipeline
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('DefaultAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('RuleBasedContactManager', responseParams="mu=" + str(0.0), name='Response', response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)

    # Define the deformable object with a simple sphere tetrahedral mesh
    deformable = rootNode.addChild('DeformableObject')
    deformable.addObject('EulerImplicitSolver', name='odesolver')
    deformable.addObject('CGLinearSolver', name='linearSolver', iterations=25, tolerance=1e-5, threshold=1e-5)

    # Simple sphere tetrahedral mesh (example positions and tetrahedrons)
    positions = [
        [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 1], [1, -1, -1],
        [-1, 1, -1], [-1, -1, 1], [-1, 1, 1], [1, -1, 1], [-1, -1, -1], [1, 1, -1]
    ]
    tetrahedrons = [
        [0, 1, 2, 3], [1, 4, 2, 3], [2, 4, 5, 3], [0, 5, 6, 3],
        [7, 0, 5, 3], [8, 1, 2, 4], [9, 0, 1, 4], [10, 6, 7, 5],
        [11, 2, 3, 8], [0, 3, 4, 1], [5, 4, 3, 6], [2, 4, 3, 5]
    ]

    deformable.addObject('MechanicalObject', name='mechObject', template='Vec3d', position=positions)
    deformable.addObject('TetrahedronSetTopologyContainer', tetrahedra=tetrahedrons)
    deformable.addObject('TetrahedronSetGeometryAlgorithms')
    deformable.addObject('UniformMass', totalMass=1000.0)
    deformable.addObject('TetrahedronFEMForceField', template='Vec3d', method='large', poissonRatio=0.3, youngModulus=1000)

    # Add the pressure constraint
    pressureConstraint = deformable.addChild('PressureConstraint')
    pressureConstraint.addObject('SurfacePressureConstraint', template='Vec3d', value=0.0, triangles='@../TetrahedronSetTopologyContainer.triangles')

    # Visual model for deformable object
    visual = deformable.addChild('VisualModel')
    visual.addObject('OglModel', src='@../mechObject', color=[0.0, 1.0, 0.0])
    visual.addObject('IdentityMapping', input='@../mechObject', output='@.')

    # Add a camera to the scene
    rootNode.addObject('InteractiveCamera', position=[0, 0, 5])

    # Animation to inflate the object
    rootNode.addObject(PressureAnimation(deformable.PressureConstraint.SurfacePressureConstraint))

    return rootNode

class PressureAnimation(Sofa.Core.Controller):
    def __init__(self, pressureConstraint):
        Sofa.Core.Controller.__init__(self)
        self.pressureConstraint = pressureConstraint
        self.time = 0.0

    def onAnimateBeginEvent(self, event):
        self.time += 0.01
        self.pressureConstraint.findData('value').value = '-8' # Linear increase over time

if __name__ == '__main__':
    main()