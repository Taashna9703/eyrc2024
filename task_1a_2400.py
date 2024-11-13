import rclpy  # ROS2 Python library
from rclpy.node import Node
import turtle

# Define scale to adjust coordinates and size
scale = 100  # This will scale (2.0, 2.0) to (200, 200)

# List to store trajectory points
trajectory = []

# Function to log the current turtle position
def log_position():
    trajectory.append(turtle.pos())  # Log the current position of the turtle

def draw_circle(center, diameter):
    radius = (diameter / 2) * scale
    turtle.penup()
    turtle.goto(center[0] * scale, (center[1] * scale) - radius)  # Adjust based on scale
    log_position()
    turtle.pendown()
    turtle.circle(radius)
    log_position()

def draw_square_with_fixed_vertices(vertices):
    turtle.penup()
    turtle.goto(vertices[0][0] * scale, vertices[0][1] * scale)
    log_position()
    turtle.pendown()
    
    for vertex in vertices:
        turtle.goto(vertex[0] * scale, vertex[1] * scale)
        log_position()
    turtle.goto(vertices[0][0] * scale, vertices[0][1] * scale)  # Close the square
    log_position()

def draw_line(x1, y1, x2, y2):
    turtle.penup()
    turtle.goto(x1 * scale, y1 * scale)
    log_position()
    turtle.pendown()
    turtle.goto(x2 * scale, y2 * scale)
    log_position()

def calculate_midpoints(vertices):
    midpoints = []
    for i in range(len(vertices)):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % len(vertices)]
        midpoint = ((x1 + x2) / 2, (y1 + y2) / 2)
        midpoints.append(midpoint)
    return midpoints

def draw_drone():
    turtle.speed(3)  # Set the speed of drawing
    turtle.bgcolor("navy")
    turtle.pensize(2)
    turtle.color("white")

    # Store all important coordinates
    important_points = {
        "propeller_centers": [],
        "square_vertices": [],
        "square_midpoints": [],
    }

    # Draw propellers (circles)
    propeller_centers = [(2.0, 2.0), (2.0, 8.0), (8.0, 8.0), (8.0, 2.0)]
    for center in propeller_centers:
        draw_circle(center, 2.0)
        important_points["propeller_centers"].append(center)
    
    # Define the square vertices as per the instructions
    square_vertices = [(3.0, 5.0), (5.0, 7.0), (7.0, 5.0), (5.0, 3.0) ]
    square_vertices1 = [(5.0, 3.0),(3.0, 5.0), (5.0, 7.0), (7.0, 5.0) ]
    important_points["square_vertices"] = square_vertices

    # Draw the drone frame (non-rotated square)
    draw_square_with_fixed_vertices(square_vertices)

    # Calculate midpoints of the square's sides
    midpoints = calculate_midpoints(square_vertices1)
    important_points["square_midpoints"] = midpoints

    # Draw lines from each circle to the midpoints of the square's sides
    for i, center in enumerate(propeller_centers):
        print(i, center)
        draw_line(center[0], center[1], midpoints[i][0], midpoints[i][1])

    # Move to the center of the square
    turtle.penup()
    turtle.goto(5.0 * scale, 5.0 * scale)
    log_position()
    turtle.pendown()
    turtle.done()  # Finish the drawing
    
    return important_points  # Return the important points

# Define ROS2 Node to handle the turtle drawing
class DroneDrawer(Node):
    def __init__(self):
        super().__init__('drone_drawer')
        self.get_logger().info('Drone Drawer Node Started')
        # Execute the drawing when the node starts
        self.draw_drone_task()

    def draw_drone_task(self):
        # Initialize the turtle setup
        turtle.setup(800, 800)  # Set the window size
        turtle.setworldcoordinates(0, 0, 10 * scale, 10 * scale)  # Set coordinates so (5.0, 5.0) is centered
        draw_drone()  # Call the function to draw the drone


def main(args=None):
    rclpy.init(args=args)
    
    # Create the ROS2 node
    drone_drawer = DroneDrawer()
    
    # Keep the node spinning to ensure it runs continuously
    rclpy.spin(drone_drawer)
    
    # Clean up when the node shuts down
    drone_drawer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
