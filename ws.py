#!/usr/bin/python
import os, sys
import kipr as kp
import SocketServer
import math
import threading
import time

# TODO: Implement sensor logic
# TODO: Implement low-level action logic
# TODO: Integrate sensor and action logic
# TODO: Integrate low-level action logic into higher-level actions
# TODO: Add and document user level actions (high-level actions) and add logic to server/client
# TODO: Add UI for client to send actions to robot.



def clamp(x, lower, upper):
    """
    return lower or upper if x is out of those bounds otherwise return x.
    """
    if x < lower:
        return lower
    elif x > upper:
        return upper
    else:
        return x

def map(x, in_min, in_max, out_min, out_max):
    """
    Linearly map x from its original range in_min, in_max to a new range out_min, out_max.
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def sign(x):
    if x < 0:
        return -1
    else:
        return 1

def meters2ticks(dist):
    """
    Convert the meters the robot's wheel travelled to wheel encoder ticks.
    @param dist (float) - Distance in meters.
    @return a float containing the wheel rotational displacement in ticks.
    """
    return dist * Robot.TICKS_PER_ROTATION / (2 * Robot.WHEEL_RADIUS)

def ticks2meters(ticks):
    """
    Convert the meters travelled by the robot's wheel given the displacement of ticks.
    @param ticks
    @returns a float containing the meters travelled.
    """
    return 2 * Robot.WHEEL_RADIUS * ticks / Robot.TICKS_PER_ROTATION

def radians2ticks(radians):
    """
    Convert from radians to wheel encoder ticks.
    @param radians
    @returns a float containing the ticks corresponding to the given radian displacement.
    """
    revolutions = radians / (2 * math.pi)
    return revolutions * Robot.TICKS_PER_ROTATION

def polar2cartesian(theta, magnitude):
    """
    Convert from polar coordinates to cartesian coordinates.
    @param theta (float) - The angle.
    @param magnitude (float) - The magnitu brug....
    @returns A Vector2 containing the corresponding cartesian coordinates.
    """
    return Vector2(magnitude * math.cos(theta), magnitude * math.sin(theta))

def cartesian2polar(x, y):
    """
    Not doing this again...
    @param x
    @param y
    @returns a Vector2 containing the corresponding polar coordinates.
    """
    return Vector2(math.atan2(y, x), math.sqrt(x*x + y * y))

def max_theoretical_vf(omega):
    """
    Returns the max forward velocity given the turning radius also being applied.
    @param omega (float) - The turning velocity in radians.
    @returns The max forward velocity we can go given omega as the turning velocity.
    """
    omega = abs(omega)
    a1 = -0.0850104
    b1 = 0.185136
    return a1 * omega + b1

def max_theoretical_omega(vf):
    """
    Returns the max turning radius given that the forward velocity in meters/second is also being applied.
    @param vf (float) - The forward velocity in meters/second.
    @returns the max turning velocity we can go given vf as the forward velocity.
    """
    vf = abs(vf)
    a1 = -11.7633
    b1 = 2.17781
    return a1 * vf + b1

class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def add(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class IRSensor:
    def __init__(self, ir_port, white_thresh):
        """
        Manage the IRSensor and act as an interface to tell if the sensor has been triggered or not.
        @param ir_port (int) - the analog port for this sensor.
        @param white_thresh (int) - The sensor value when the line is on top of a piece of black tape.
        """
        self.ir_port = ir_port
        self.white_thresh = white_thresh

    def above_threshold(self):
        return kp.analog(self.ir_port) > self.white_thresh

class IRArray:
    def __init__(self, s_ll, s_l, s_r, s_rr):
        """
        Store the IRSensor objects used along with names indicating their position.
        @param s_ll through s_rr are IRSensor objects with unique ports.
        """
        self.s_ll = s_ll
        self.s_l = s_l
        self.s_r = s_r
        self.s_rr = s_rr

class Gripper:
    # 0 - 2048 but getting to close to the limits drains battery
    servo_min_limit = 60
    servo_max_limit = 2048 - servo_min_limit

    def __init__(self, servo_port, is_open, open_dist, closed_dist):
        """
        Control the gripper actuator on the robot. Initially set the start state of the gripper.
        @param servo_port (int) - The servo port in use.
        @param is_open (Bolean) - True if the servo should start off as open otherwise it starts off closed.
        @param open_dist (float) - A value indicating the servo positon of a open gripper.
        @param closed_dist (float) - A value indicating the servo position of a closed gripper.
        """
        assert open_dist < Gripper.servo_max_limit
        assert closed_dist > Gripper.servo_min_limit
        self.is_open = is_open
        self.servo_port = servo_port
        self.open_dist = open_dist
        self.closed_dist = closed_dist
        # Enable servo and set state to either open_dist or closed_dist
        kp.enable_servo(self.servo_port)
        dist = self.open_dist if self.is_open else self.closed_dist
        kp.set_servo_position(self.servo_port, dist)
        kp.msleep(1000)
        kp.disable_servo(self.servo_port)

    def _set_servo_dist(self, dist):
        """
        Private method for setting the servo distance.
        @param dist (int) - The servo position to set the servo to.
        """
        kp.enable_servo(self.servo_port)
        kp.set_servo_position(self.servo_port, dist)
        kp.msleep(1000)
        kp.disable_servo(self.servo_port)

    def open(self):
        """
        Open the gripper and set the state to open.
        """
        if self.is_open:
            return
        self.is_open = True
        self._set_servo_dist(self.open_dist)

    def close(self):
        """
        Close the gripper and set the state to closed.
        """
        if not self.is_open:
            return
        self.is_open = False
        self._set_servo_dist(self.closed_dist)

class DifferentialDrive:
    def __init__(self, left_motor_port, right_motor_port):
        self.left_motor_port = left_motor_port
        self.right_motor_port = right_motor_port
        self.left_last_ticks = kp.get_motor_position_counter(self.left_motor_port)
        self.right_last_ticks = kp.get_motor_position_counter(self.right_motor_port)
        self.last_timestamp = kp.seconds()
        self.previous_states = [(0,0,0)] # Store commands and dt here in order to deal with backtracking.
        self.record = False
        self.resolution = 100
        # reset motor encoders.

    def rotate_x_deg(self, current_estimate, deg):
        """
        Rotate a fixed degree count using a PID controller and deadreckoning.
        @param current_estimate (Vector3) - An estimate of where the current body frame is.
        @param deg (float) - Amount to turn in radians.
        """
        assert abs(deg) < 2 * math.pi

        theta_init = current_estimate.z
        theta_desired = theta_init + deg
        theta_diff = current_estimate.z - theta_desired

        self.left_last_ticks = kp.get_motor_position_counter(self.left_motor_port)
        self.right_last_ticks = kp.get_motor_position_counter(self.right_motor_port)
        deg_sign = sign(deg)

        previous_error = 0
        integral = 0
        self.last_timestamp = kp.seconds()

        kpr = 3
        ki = 0.6
        kd = 0.3

        while abs(theta_diff) > 0.001:
            current_time = kp.seconds()
            dt = current_time - self.last_timestamp
            self.last_timestamp = current_time

            integral += theta_diff * dt
            
            if dt == 0:
                dt = 0.000000001

            derivative = (theta_diff - previous_error) / dt
            previous_error = theta_diff

            output = kpr * theta_diff + ki * integral + kd * derivative
            raw_error = map(output, -deg, deg, deg_sign * 1500, deg_sign * -1500)
            error_sign = sign(raw_error)
            raw_error = error_sign * clamp(abs(raw_error), 0, 1500)
            power = int(raw_error)
            kp.mav(self.left_motor_port, -power)
            kp.mav(self.right_motor_port, power)

            current_estimate = self.dead_reckon_state_change(current_estimate, dt)
            theta_diff = current_estimate.z - theta_desired
        kp.mav(self.left_motor_port, 0)
        kp.mav(self.right_motor_port, 0)
        return current_estimate

    def drive(self, vf, omega):
        """
        Change the forward speed and turning speed of the robot.
        @param vf - The forward velocity of the robot (constrained by omega)
        @param omeaga - The angular velocity of the robot (constrained by vf)
        """
        vr = (2 * vf + omega * Robot.AXIS_LENGTH) / (2 * Robot.WHEEL_RADIUS)
        vl = (2 * vf - omega * Robot.AXIS_LENGTH) / (2 * Robot.WHEEL_RADIUS)

        vl = int(radians2ticks(vl))
        vr = int(radians2ticks(vr))

        if self.record:
            if len(self.previous_states) > self.resolution:
                self.previous_states.pop(0)
            new_timestamp = kp.seconds()
            self.previous_states.append((vl, vr, new_timestamp - self.last_timestamp))
            self.last_timestamp = new_timestamp

        kp.mav(self.left_motor_port, vl)
        kp.mav(self.right_motor_port, vr)

    def dead_reckon_state_change(self, current_estimate, dt):
        """
        Use wheel encoders to estimate the change in position from the current last estimate.
        When a global position estimate is applied, we will ahve to handle
        resetting the motor encoder ticks and then record from there.
        @param current_estimate (Vector3) an estimate of the current robot's body frame.
        @param dt (float) the time elapsed since the last measurement.
        @returns a Vector3 containing the new estimate.
        """
        left_dx = kp.get_motor_position_counter(self.left_motor_port) - self.left_last_ticks
        right_dx = kp.get_motor_position_counter(self.right_motor_port) - self.right_last_ticks

        self.left_last_ticks += left_dx
        self.right_last_ticks += right_dx

        # Convert to radians
        left_dx = 2 * math.pi * left_dx / Robot.TICKS_PER_ROTATION
        right_dx = 2 * math.pi * right_dx / Robot.TICKS_PER_ROTATION

        forward_velocity = (Robot.WHEEL_RADIUS / 2) * (left_dx + right_dx) # Might need to fact check this as it was written early in the semester.
        current_estimate.x += forward_velocity * math.cos(current_estimate.z)
        current_estimate.y += forward_velocity * math.sin(current_estimate.z)
        current_estimate.z += (Robot.WHEEL_RADIUS / Robot.AXIS_LENGTH) * (right_dx - left_dx)
        return current_estimate

    def dead_reckon_state_estimate(self, current_estimate, dt):
        """
        Reduced variable version of below. Keep the version below until you test both.
        Source: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html but with less variables.
        @param current_estimate (Vector3) - An estimate of the current robot's body frame.
        @param dt (float) - The time elapsed since the last measurement.
        @returns a Vector3 containing the new estimate.
        """
        left_dx = kp.get_motor_position_counter(self.left_motor_port)
        right_dx = kp.get_motor_position_counter(self.right_motor_port)

        if dt == 0:
            dt = 0.0000001

        vl = (left_dx - self.left_last_ticks) / dt
        vl = vl * Robot.WHEEL_RADIUS * math.pi / 180.0
        vr = (right_dx - self.right_last_ticks) / dt        
        vr = vr * Robot.WHEEL_RADIUS * math.pi / 180.0
        self.left_last_ticks = left_dx
        self.right_last_ticks = right_dx
        
        # Calculate forward velocity and turning velocity.
        vf = (vr + vl) / 2.0
        omega = (vr - vl) / Robot.AXIS_LENGTH

        s_x = vf * math.cos(current_estimate.z)
        s_y = vf * math.sin(current_estimate.z)
        s_x1 = vf * math.cos(current_estimate.z + dt * omega / 2)
        s_y1 = vf * math.sin(current_estimate.z + dt * omega / 2)
        s_x2 = vf * math.cos(current_estimate.z + dt * omega)
        s_y2 = vf * math.sin(current_estimate.z + dt * omega)

        current_estimate.x += (dt / 6.0) * (s_x + 4 * s_x1 + s_x2)
        current_estimate.y += (dt / 6.0) * (s_y + 4 * s_y1 + s_y2)
        current_estimate.z += omega * dt

        return current_estimate

class Camera:
    qr_code_size = 0.045 # meters
    def __init__(self, camera_offset):
        """
        Initialize the wallaby camera and manage the camera offset from the
        center of the robot's body frame.
        @param camera_offset (Vector3) - Describes the camera position offset from the center of the robot.
        """
        self.image_height = 120
        self.image_width = 160
        kp.camera_open_at_res(self.image_height)
        kp.camera_update()
        self.offset = camera_offset
        self.focal_length = 0.13 # Approximated with ruler and measurements
        self.sensor_height = 0.0878310065654

    def get_scaled_distance(self, object_real_height, object_pixel_height):
        """
        Gets the scaled height using the unscaled distance and a ratio of focal length and sensor height.
        """
        return self.focal_length * self.unscaled_distance(object_real_height, self.image_height, object_pixel_height) / (self.sensor_height * 100) # convert to meters

    def unscaled_distance(self, object_real_height, image_height, object_pixel_height):
        """
        Gets the unscaled distance between the camera and a given object.
        @param object_real_height (float) - Height of object in mm.
        @param image_height (int) - Height of image in pixels.
        @param object_pixel_height (int) - Height of object in pixels.
        @returns the unscaled distance between the camera and a given object.
        """
        return object_real_height * image_height / object_pixel_height

    def search_for_item(self, item_id):
        """
        Search for a box with a QR code with the given item_id and return the bearing towards it.
        Perform exhaustive search before deciding that the box does not exist.
        When returning the bearing towards the box, ensure that you apply the camera offset.
        @param item_id (string) - The box QR code to search for.
        """
        kp.camera_update()
        if kp.get_object_count(0) > 0:
            if kp.get_object_data(0,0) == item_id:
                # Calculate bearing vector towards target
                area = kp.get_object_area(0,0)
                height = math.sqrt(area)
                center = kp.get_object_center(0,0)
                dx = 160 / 2 - center.x
                dy = 120 / 2 - center.y
                dz = self.get_scaled_distance(Camera.qr_code_size, height)
                return self.offset.add(Vector3(dx, dy, dz))
                
        return None

class BaseAction:
    def __init__(self):
        pass
    def execute(self, robot):
        pass

class ActionChangeSpeeds(BaseAction):
    def __init__(self, vf, omega):
        """
        Change the forward velocity (vf) and turning speed (omega)
        @param vf (float) - The forward velocity.
        @param omega (float) - The turning velocity.
        """
        self.vf = vf
        self.omega = omega
    
    def execute(self, robot):
        robot.controller.drive(self.vf, self.omega)

        return True, []

class ActionRotateXDeg(BaseAction):
    def __init__(self, deg):
        self.deg = deg
    
    def execute(self, robot):
        robot.frame = robot.controller.rotate_x_deg(robot.frame, self.deg)
        return True, []


class ActionGrabQRBin(BaseAction):
    def __init__(self, item_id):
        """
        A subtask that looks for the bin with the qr code at the current position,
        navigates towards it, grabs it, then goes back to where it started using
        either odometry, signs, or a grid-based-ir-based particle filter algorithm (last ditch effort).
        As I am writing this at 3am, the grid-based-ir-based particle filter algorithm (gbib) uses a
        black and white checkerboard pattern, potentially a gradient of various shades, in order to
        define a map that the ir sensors can sense and thus run a basic particle filtering algorithm.
        This will let us use assumptions to navigate back to the starting position. Otherwise we may
        be able to use a crosshatching pattern instead to navigate to the corner of the allowed area - damn this is really complicated for a drop off area nvm.
        """
        self.item_id = item_id
        self.state = "search"
    
    def execute(self, robot):
        """
        Record path/velocities.
        Use camera to find QR code with value item_id.
        Navigate to QR code with item_id so that it lines up with gripper.
        Use distance sensor/QR code size to determine if QR code is within gripper reach.
        Apply reverse motor instructions to get close to original path.
        Get back on the line tracks.
        """
        bearing = None
        robot.controller.drive(0, 0)
        results = [robot.camera.search_for_item(self.item_id) for _ in range(15)]
        for result in results:
            if result is not None:
                bearing = result
        if bearing is None:
            robot.controller.drive(0, Robot.OMEGA_MAX / 3.0)
            kp.msleep(200)
            return False, []
        # Otherwise move towards bearing, close gripper and backup
        theta = math.atan2(bearing.x, bearing.y)
        old_theta = robot.frame.z
        robot.frame = robot.controller.rotate_x_deg(robot.frame, theta)

        print("dist: " + str(bearing.z))

        # Open gripper if need be.
        if not robot.gripper.is_open:
            robot.gripper.open()
            kp.msleep(500)

        time_moving = kp.seconds()

        # Move bearing.z distance.
        speed = Robot.VF_MAX / 2
        while kp.analog(0) < 2920:
            robot.controller.drive(speed, 0)

        while kp.analog(0) > 2920:
            robot.controller.drive(speed, 0)

        # passed barrier
        dt_moved = kp.seconds() - time_moving

        robot.gripper.close()
        kp.msleep(500)

        # Move -bearing.z distance.
        speed = Robot.VF_MAX / 2
        robot.controller.drive(-speed, 0)
        kp.msleep(int(dt_moved * 1000))

        robot.frame = robot.controller.rotate_x_deg(robot.frame, old_theta)

        return True, []

class ActionGoToZone(BaseAction):
    def __init__(self, target_zone):
        """
        A subtask that brings the robot assuming it is in the current_zone to the target_zone.
        """
        self.target_zone = target_zone
    
    def execute(self, robot):
        """
        Use IRArray to follow line until junctions, use current zone to determine where to turn at junctions.
        Update state position and orientation to absolute values when a junction is reached to lower accumulative error.
        """
        if robot.current_zone == self.target_zone:
            return True, []
        elif robot.current_zone in ("Dropoff", "ZoneA", "ZoneB", "ZoneC") and self.target_zone in ("Dropoff", "ZoneA", "ZoneB", "ZoneC"):
            # multiple steps, solve current to dropoff first.
            print("Inserting step")
            return True, [ActionGoToZone("Terminal"), self]
        elif robot.current_zone == "Terminal":
            print("From Terminal")
            print("Turning towards " + self.target_zone)
            # Turn towards target zone and follow line.
            if self.target_zone == "Dropoff":
                robot.turn_to_direction("backward")
            elif self.target_zone == "ZoneA":
                robot.turn_to_direction("left")
            elif self.target_zone == "ZoneB":
                robot.turn_to_direction("forward")
            elif self.target_zone == "ZoneC":
                robot.turn_to_direction("right")
            robot.follow_line_until_junction()
            robot.current_zone = self.target_zone
            return True, []
        elif robot.current_zone == "Dropoff":
            print("From Dropoff")
            # Follow line until terminal, check that bearings are right (robot.get_forward() and (robot.position - terminal_zone) are in the same direction, otherwise correct)
            robot.turn_to_direction("forward")
            robot.follow_line_until_junction()
            robot.current_zone = "Terminal"
            return True, []
        elif robot.current_zone == "ZoneA":
            # Follow line until terminal, check that bearings are right (robot.get_forward() and (robot.position - terminal_zone) are in the same direction, otherwise correct)
            robot.turn_to_direction("backward")
            robot.follow_line_until_junction()
            robot.current_zone = "Terminal"
            return True, []
        elif robot.current_zone == "ZoneB":
            # Follow line until terminal, check that bearings are right (robot.get_forward() and (robot.position - terminal_zone) are in the same direction, otherwise correct)
            robot.turn_to_direction("backward")
            robot.follow_line_until_junction()
            robot.current_zone = "Terminal"
            return True, []
        elif robot.current_zone == "ZoneC":
            # Follow line until terminal, check that bearings are right (robot.get_forward() and (robot.position - terminal_zone) are in the same direction, otherwise correct)
            robot.turn_to_direction("backward")
            robot.follow_line_until_junction()
            robot.current_zone = "Terminal"
            return True, []
        else:
            print("Error: Cannot move from invalid zone...")
            return True, []
        

class Robot:
    zones = {
        "Dropoff": Vector2(0,0),
        "Terminal": Vector2(0,1),
        "ZoneA": Vector2(-1,2),
        "ZoneB": Vector2(0,2),
        "ZoneC": Vector2(1,2)
    }

    # measurements done in m
    TICKS_PER_ROTATION = 1400
    WHEEL_SPEED_IN_TICKS = 1500
    WHEEL_RADIUS = 0.0275
    AXIS_LENGTH = 0.17

    OMEGA_MAX = 2.177
    VF_MAX = 0.185

    def __init__(self, motors, ir_array, gripper, camera):
        # let x, y be their cartesian coordinates and let z be the orientation theta.
        self.frame = Vector3(0,0,0) # theta should be converted to global orientation each time we turn or sense a black line with IR since this prevents error accumulation.
        self.controller = motors
        self.ir_array = ir_array
        self.gripper = gripper
        self.camera = camera
        self.object_held = None
        self.current_zone = "Dropoff"
        self.target_zone = None
        self.action_queue = []
        self.last_timestamp = kp.seconds()
        self.last_debug = kp.seconds()
        self.is_forward = True
        self.is_left = False
        self.is_right = False
        self.is_backward = False

    def get_forward(self):
        return Vector2(math.cos(self.frame.z), math.sin(self.frame.z))

    def follow_line_until_junction(self):
        found_junction = False
        while not found_junction:
            sll = self.ir_array.s_ll.above_threshold()
            sl = self.ir_array.s_l.above_threshold()
            sr = self.ir_array.s_r.above_threshold()
            srr = self.ir_array.s_rr.above_threshold()
            
            passed_turn = False

            if not sll and not srr:
                if not sl and not sr:
                    self.controller.drive(Robot.VF_MAX / 2, 0)
                elif sr and not sl:
                    self.controller.drive(Robot.VF_MAX / 2, max_theoretical_omega(Robot.VF_MAX / 2))
                elif not sr and  sl:
                    self.controller.drive(Robot.VF_MAX / 2, -max_theoretical_omega(Robot.VF_MAX / 2))
            elif sll and srr:
                found_junction = True
            elif sll or srr:
                if self.current_zone == "Terminal" or passed_turn:
                    # turn right
                    kp.msleep(300)
                    self.turn_to_direction("forward")
                    passed_turn = True
                else:
                    found_junction = True
        self.controller.drive(0,0)


    def turn_to_direction(self, direction):
        if direction == "forward":
            if self.is_forward:
                return
            elif self.is_left:
                self.controller.rotate_x_deg(robot.frame, math.pi / 2.0)
                self.is_left = False
            elif self.is_right:
                self.controller.rotate_x_deg(robot.frame, -math.pi / 2.0)
                self.is_right = False
            elif self.is_backward:
                self.controller.rotate_x_deg(robot.frame, math.pi)
                self.is_backward = False
            self.is_forward = True
        elif direction == "left":
            if self.is_left:
                return
            elif self.is_forward:
                self.controller.rotate_x_deg(robot.frame, -math.pi / 2.0)
                self.is_forward = False
            elif self.is_right:
                self.controller.rotate_x_deg(robot.frame, -math.pi)
                self.is_right = False
            elif self.is_backward:
                self.controller.rotate_x_deg(robot.frame, math.pi / 2.0)
                self.is_backward = False
            self.is_left = True
        elif direction == "right":
            if self.is_right:
                return
            elif self.is_forward:
                self.controller.rotate_x_deg(robot.frame, math.pi / 2.0)
                self.is_forward = False
            elif self.is_left:
                self.controller.rotate_x_deg(robot.frame, math.pi)
                self.is_left = False
            elif self.is_backward:
                self.controller.rotate_x_deg(robot.frame, -math.pi / 2.0)
                self.is_backward = False
            self.is_right = True
        elif direction == "backward":
            if self.is_backward:
                return
            elif self.is_forward:
                self.controller.rotate_x_deg(robot.frame, -math.pi)
                self.is_forward = False
            elif self.is_right:
                self.controller.rotate_x_deg(robot.frame, math.pi / 2.0)
                self.is_right = False
            elif self.is_left:
                self.controller.rotate_x_deg(robot.frame, -math.pi / 2.0)
                self.is_left = False
            self.is_backward = True

    def add_action(self, event):
        """
        Adds the action to the event queue.
        """
        self.action_queue.append(event)

    def _insert_events(self, events, position):
        """
        Adds the events at the given position. ([p-1, <events>, p+1])
        """
        self.action_queue = self.action_queue[:position] + events + self.action_queue[position:]

    def log_info(self):
        if kp.seconds() - self.last_debug > 3.0:
            self.last_debug = kp.seconds()
            print("DEBUG:\n" + "\tFrame: (" + str(self.frame.x) + ", " + str(self.frame.y) + ", " + str(self.frame.z) + ")\n")

    def update_state(self):
        """
        Update the state with both local and global methods.
        Local methods include wheel encoders/deadreckoning.
        Global methods include using IR sensors to adjust robot frame when we cross terminals/turn onto the line.
        """
        # Calculate time elapsed
        timestamp = kp.seconds()
        dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        # Apply deadreckoning
        self.frame = self.controller.dead_reckon_state_change(self.frame, dt)

    def process_action(self):
        """
        Run the action loop if there are any queued up.
        """
        if len(self.action_queue) <= 0:
            return
        
        action = self.action_queue[0]
        finished, new_events = action.execute(self)
        if finished:
            print("Finished Action!")
            self.action_queue.pop(0)
        if len(new_events) > 0:
            self._insert_events(new_events, 1)

class RobotTCPServer(SocketServer.TCPServer):
	def __init__(self, server_address, RequestHandlerClass, robot, robot_lock, bind_and_activate=True):
		self.robot = robot
		self.robot_lock = robot_lock
		SocketServer.TCPServer.__init__(self, server_address, RequestHandlerClass, bind_and_activate=True)

class MyTCPHandler(SocketServer.BaseRequestHandler):
    """
    The request handler class for our server.
    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """
    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print("{} wrote:".format(self.client_address[0]))
        print(self.data)
        with self.server.robot_lock:
            if "Action" in self.data:
                self.server.robot.add_action(eval(self.data)) # Horrendous, never do this.
                self.request.sendall("ack")

        # just send back the same data, but upper-cased
        self.request.sendall("fail")

def robot_thread(robot, robot_lock):
    while True:
        with robot_lock:
            robot.update_state()
            robot.log_info()
            robot.process_action()

if __name__ == "__main__":
    sys.stdout = os.fdopen(sys.stdout.fileno(),"w",0)
    sys.stderr = sys.stdout
    print("here " + str(sys.version_info))
    robot_lock = threading.RLock()
    robot = Robot(
        DifferentialDrive(0, 3),
        IRArray(
            IRSensor(5, 2000),
            IRSensor(4, 2000),
            IRSensor(3, 2000),
            IRSensor(2, 2000)
        ),
        Gripper(0, True, Gripper.servo_max_limit - 10, Gripper.servo_min_limit + 10),
        Camera(Vector3(0,0,7))
    )
    HOST, PORT = "0.0.0.0", 8100
    server = RobotTCPServer((HOST, PORT), MyTCPHandler, robot, robot_lock)
    r_thread = threading.Thread(target=robot_thread, args=(robot, robot_lock))
    r_thread.setDaemon(True)
    r_thread.start()
    server.serve_forever()
    r_thread.join()