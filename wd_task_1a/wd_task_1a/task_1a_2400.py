import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
import time
import math

class TurtleManualLineDrawer(Node):
    def __init__(self):
        super().__init__('turtle_manual_line_drawer')

        # Create a client to teleport the turtle
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # Wait for the teleport and pen services to be available
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting...')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetPen service not available, waiting...')

        # Start by drawing all circles first
        self.draw_all_circles_manually()

        # After circles are drawn, draw the square
        self.draw_square_manually()

        self.draw_lines()

    def set_pen(self, r, g, b, width, off):
        """ Control the pen (off = 1 means pen up, off = 0 means pen down) """
        pen_request = SetPen.Request()
        pen_request.r = r
        pen_request.g = g
        pen_request.b = b
        pen_request.width = width
        pen_request.off = off
        self.pen_client.call_async(pen_request)

    def teleport_turtle(self, x, y, theta=0.0):
        """ Teleport turtle to the given (x, y) without drawing """
        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = x
        teleport_request.y = y
        teleport_request.theta = theta
        self.teleport_client.call_async(teleport_request)
        time.sleep(1)

    def deg2rad(self, degrees):
        """ Convert degrees to radians """
        return degrees * 3.14159 / 180.0

    def draw_circle(self, radius, coordinates: tuple):
        """ Draw a circle with the given radius """

        # Set the pen up before teleporting
        self.set_pen(0, 0, 0, 2, 1)  # Pen up

        # Teleport to the given coordinates
        self.teleport_turtle(coordinates[0], coordinates[1])

        # Move to the start point 
        self.teleport_turtle(coordinates[0] + radius, coordinates[1])

        # Set the pen down at the start point
        self.set_pen(255, 255, 255, 2, 0) # Pen down

        # Draw the circle by moving the turtle in a circular path
        step_size = 25  # Increase step size to speed up drawing
        for i in range(0, 360, step_size):
            x = coordinates[0] + radius * math.cos(math.radians(i))
            y = coordinates[1] + radius * math.sin(math.radians(i))
            self.teleport_turtle(x, y)
        self.teleport_turtle(coordinates[0] + radius, coordinates[1])  # Move back to the start point
        # Set the pen up after drawing the circle
        self.set_pen(0, 0, 0, 2, 1)  # Pen up

    def connect_points(self, start, end):
        """ Teleport to the start point and draw a line to the end point """
        self.teleport_turtle(start[0], start[1])  # Teleport to the start point
        self.set_pen(255, 255, 255, 2, 0)  # Pen down (start drawing)
        time.sleep(1)
        self.teleport_turtle(end[0], end[1])  # Teleport to the end point to draw the line
        time.sleep(1)
        self.set_pen(255, 255, 255, 2, 1)  # Pen up (stop drawing)

    def draw_square_manually(self):
        """ Manually connect all the given points to form the rotated square """
        self.get_logger().info('Drawing the square manually by connecting points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)  # Pen up (no drawing)
        self.teleport_turtle(3.0, 5.0)
        time.sleep(1)

        # Step 2: Now connect the points to form the square
        # Connect (3.0, 5.0) to (5.0, 7.0)
        self.connect_points((3.0, 5.0), (5.0, 7.0))

        # Connect (5.0, 7.0) to (7.0, 5.0)
        self.connect_points((5.0, 7.0), (7.0, 5.0))

        # Connect (7.0, 5.0) to (5.0, 3.0)
        self.connect_points((7.0, 5.0), (5.0, 3.0))

        # Connect (5.0, 3.0) to (3.0, 5.0)
        self.connect_points((5.0, 3.0), (3.0, 5.0))

    def draw_all_circles_manually(self):
        """ Manually draw all circles """

        # Circle 1: Draw at (2.0, 2.0)
        self.get_logger().info('Teleporting to (2.0, 2.0) and drawing a circle.')
        self.draw_circle(1.0, (2.0, 2.0))  # Draw the circle with a radius of 1.0

        # Circle 2: Draw at (2.0, 8.0)
        self.get_logger().info('Teleporting to (2.0, 8.0) and drawing a circle.')
        self.draw_circle(1.0, (2.0, 8.0))  # Draw the circle with a radius of 1.0

        # Circle 3: Draw at (8.0, 8.0)
        self.get_logger().info('Teleporting to (8.0, 8.0) and drawing a circle.')
        self.draw_circle(1.0, (8.0, 8.0))  # Draw the circle with a radius of 1.0

        # Circle 4: Draw at (8.0, 2.0)
        self.get_logger().info('Teleporting to (8.0, 2.0) and drawing a circle.')
        self.draw_circle(1.0, (8.0, 2.0))

    def draw_lines(self):
        """ Draw lines to connect the given points """
        self.get_logger().info('Drawing lines to connect the given points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)
        self.get_logger().info('Connecting points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)  # Pen up (no drawing)
        self.teleport_turtle(2.0, 2.0)
        time.sleep(1)

        # Step 2: Now connect the points to form the square
        # Connect (3.0, 5.0) to (5.0, 7.0)
        self.connect_points((2.0, 2.0), (4.0, 4.0))


        self.get_logger().info('Drawing lines to connect the given points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)
        self.get_logger().info('Connecting points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)  # Pen up (no drawing)
        self.teleport_turtle(2.0, 8.0)
        time.sleep(1)

        # Step 2: Now connect the points to form the square
        # Connect (3.0, 5.0) to (5.0, 7.0)
        self.connect_points((2.0, 8.0), (4.0, 6.0))


        self.get_logger().info('Drawing lines to connect the given points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)
        self.get_logger().info('Connecting points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)  # Pen up (no drawing)
        self.teleport_turtle(8.0, 2.0)
        time.sleep(1)

        # Step 2: Now connect the points to form the square
        # Connect (3.0, 5.0) to (5.0, 7.0)
        self.connect_points((8.0, 2.0), (6.0, 4.0))

        self.get_logger().info('Drawing lines to connect the given points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)
        self.get_logger().info('Connecting points')

        # Step 1: Teleport to the first point (3.0, 5.0) with pen up (no drawing)
        self.set_pen(255, 255, 255, 2, 1)  # Pen up (no drawing)
        self.teleport_turtle(8.0, 8.0)
        time.sleep(1)

        # Step 2: Now connect the points to form the square
        # Connect (3.0, 5.0) to (5.0, 7.0)
        self.connect_points((8.0, 8.0), (6.0, 6.0))

        self.set_pen(255, 255, 255, 2, 1)  # Pen up (no drawing)
        self.teleport_turtle(5.0, 5.0)


def main(args=None):
    rclpy.init(args=args)
    turtle_drawer = TurtleManualLineDrawer()
    rclpy.spin(turtle_drawer)
    turtle_drawer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
