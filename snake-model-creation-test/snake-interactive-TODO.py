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
    camera = rootNode.addObject('InteractiveCamera', position=[0, 5, 20], lookAt=[0, 0, 0])

    # Snake parameters
    num_segments = 7
    segment_length = 30.0
    radius = 0.1

    # Add gravity
    rootNode.gravity = [0, -9.81, 0]

    # Define the snake base
    snakeBase = rootNode.addChild('SnakeBase')

    # Create snake segments
    segments = []
    for i in range(num_segments):
        segment = snakeBase.addChild(f'Segment_{i}')
        segment.addObject('EulerImplicitSolver', name='cg_odesolver')
        segment.addObject('CGLinearSolver', name='linear_solver')
        segment.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[[i * segment_length, 0, 0, 0, 0, 0, 1]])
        segment.addObject('UniformMass', totalMass=1.0)
        segment.addObject('UncoupledConstraintCorrection')

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
    controller = SnakeController(segments)
    rootNode.addObject(controller)

    return rootNode


class SnakeController(Sofa.Core.Controller):
    def __init__(self, segments):
        super().__init__()
        self.segments = segments
        self.target_position = None

    def onAnimateBeginEvent(self, event):
        if self.target_position:
            self.move_towards_target()

    def move_towards_target(self):
        head = self.segments[0]
        head_position = np.array(head.dofs.position[0][:3])
        direction = np.array(self.target_position) - head_position
        distance = np.linalg.norm(direction)

        if distance > 0.1:  # Only move if the target is far enough
            direction /= distance  # Normalize the direction
            speed = 0.1  # Movement speed
            new_position = head_position + direction * speed
            head.dofs.position[0][:3] = new_position

            # Move the rest of the segments
            for i in range(1, len(self.segments)):
                previous_segment = self.segments[i - 1]
                segment = self.segments[i]
                previous_position = np.array(previous_segment.dofs.position[0][:3])
                segment_position = np.array(segment.dofs.position[0][:3])

                direction = previous_position - segment_position
                distance = np.linalg.norm(direction)

                if distance > 0.1:  # Only move if the distance is far enough
                    direction /= distance  # Normalize the direction
                    new_position = segment_position + direction * speed

                    segment.dofs.position[0][:3] = new_position

    def onMouseButtonLeft(self, mouseX, mouseY, isPressed):
        if isPressed:
            # Convert mouse coordinates to scene coordinates
            self.target_position = self.convertMouseToScenePosition(mouseX, mouseY)
            print(f"Mouse pressed at: {self.target_position}")

    def convertMouseToScenePosition(self, mouseX, mouseY):
        # Placeholder function to convert mouse coordinates to scene coordinates
        # Implement the appropriate conversion logic based on your scene setup
        sceneX = (mouseX - 0.5) * 20  # Assuming the window width is normalized [0,1]
        sceneY = (mouseY - 0.5) * -20  # Assuming the window height is normalized [0,1]
        return [sceneX, 0, sceneY]


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