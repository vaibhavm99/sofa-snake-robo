import Sofa
import numpy as np

def createScene(rootNode):
    # Root Node
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels showForceFields showInteractionForceFields')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='SofaSimpleFem')
    # rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')

    rootNode.dt = 0.01
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceDetection')
    rootNode.addObject('DefaultContactManager')
    rootNode.addObject('DiscreteIntersection')

    # Camera
    rootNode.addObject('InteractiveCamera', position=[0, 5, 20], lookAt=[0, 0, 0])

    # Snake parameters
    num_segments = 7
    segment_length = 30.0 # cm
    radius = 0.1

    # Add gravity
    rootNode.gravity = [0, -9.81, 0]

    # Define the snake base
    snakeBase = rootNode.addChild('SnakeBase')

    # Create snake segments
    segments = []
    for i in range(num_segments):
        segment = snakeBase.addChild(f'Segment_{i}')
        # Mechanical model
        segment.addObject('EulerImplicitSolver', name='cg_odesolver')
        segment.addObject('CGLinearSolver', name='linear_solver')
        segment.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[[i * segment_length, 0, 0, 0, 0, 0, 1]])
        segment.addObject('UniformMass', totalMass=1.0)
        segment.addObject('UncoupledConstraintCorrection')

        # Gravity is set here
        segment.createObject('ConstantForceField', name='gravity', force=[0, -9.81, 0])

        # Visual model
        visualNode = segment.addChild('Visual')
        visualNode.addObject('MeshSTLLoader', name='loader', filename='mesh/sphere.stl', translation=[i * segment_length, 0, 0])
        visualNode.addObject('OglModel', src='@loader')
        visualNode.addObject('RigidMapping', input=segment.dofs.getLinkPath(), output=visualNode.getLinkPath())

        # Collision model
        collisionNode = segment.addChild('Collision')
        collisionNode.addObject('MechanicalObject', template='Vec3d', position=[[i * segment_length, 0, 0]])
        collisionNode.addObject('SphereCollisionModel', radius=radius)
        collisionNode.addObject('RigidMapping', input=segment.dofs.getLinkPath(), output=collisionNode.getLinkPath())

        segments.append(segment)

        # Connect segments with springs
        if i > 0:
            previous_segment = segments[i - 1]
            segment.addObject('StiffSpringForceField', name=f'spring_{i}', object1=previous_segment.dofs.getLinkPath(), object2=segment.dofs.getLinkPath(), spring=[0, 0, 0, segment_length, 0, 0, 1, 0, 0, 1, 100, 0.5])

    # Controller
    rootNode.addObject(SnakeController(segments))

    return rootNode


class SnakeController(Sofa.Core.Controller):
    def __init__(self, segments):
        super().__init__()
        self.segments = segments
        self.time = 0

    def onAnimateBeginEvent(self, event):
        self.time += 0.01
        wave_amplitude = 0.5
        wave_length = 2
        wave_speed = 2
        for i, segment in enumerate(self.segments):
            angle = wave_amplitude * np.sin(wave_length * i - wave_speed * self.time)
            segment.dofs.position.value = [[i * 1.0, 0, 0, 0, np.sin(angle / 2), 0, np.cos(angle / 2)]]


if __name__ == '__main__':
    import SofaRuntime
    import SofaGui
    import SofaSimulation

    # Register the scene
    SofaRuntime.importPlugin("SofaPython3")
    SofaRuntime.importPlugin("SofaComponentAll")
    
    root = Sofa.Core.Node("root")
    createScene(root)

    # Run the simulation
    SofaSimulation.Simulation.init(root)
    SofaGui.GUIManager.Init("myGUI", root)
    SofaGui.GUIManager.createGUI(root, __file__)
    SofaSimulation.Simulation.animate(root, True)
    SofaGui.GUIManager.MainLoop(root)
    SofaGui.GUIManager.closeGUI()