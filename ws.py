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

# 0 - 2048 but getting to close to the limits drains battery
servo_min_limit = 60
servo_max_limit = 2048 - servo_min_limit

class Utilities:
    def clamp(x, lower, upper):
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
        v = abs(v)
        a1 = -11.7633
        b1 = 2.17781
        return a1 * v + b1

    


class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class IRSensor:
    def __init__(self, ir_name, ir_port, white_thresh, position_on_frame):
        self.ir_name = ir_name
        self.ir_port = ir_port
        self.white_thresh = white_thresh
        self.position_on_frame = position_on_frame

    def above_threshold(self):
        return kp.analog(self.ir_port) > self.white_thresh

class IRArray:
    def __init__(self, s_ll, s_l, s_r, s_rr):
        self.s_ll = s_ll
        self.s_l = s_l
        self.s_r = s_r
        self.s_rr = s_rr

class Gripper:
    def __init__(self, is_open, open_dist, closed_dist):
        assert open_dist < servo_max_limit
        assert closed_dist > servo_min_limit
        self.is_open = is_open
        # Enable servo and set state to either open_dist or closed_dist

    def open(self):
        if self.is_open:
            return
        self.is_open = True
        # Enable servo and set state to open_dist

    def close(self):
        if not self.is_open:
            return
        self.is_open = False
        # Enable servo and set state to closed_dist

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

    def drive(self, vf, omega):
        """
        Change the forward speed and turning speed of the robot.
        @param vf - The forward velocity of the robot (constrained by omega)
        @param omeaga - The angular velocity of the robot (constrained by vf)
        """
        vr = (2 * vf + omega * Robot.AXIS_LENGTH) / (2 * Robot.WHEEL_RADIUS)
        vl = (2 * vf - omega * Robot.AXIS_LENGTH) / (2 * Robot.WHEEL_RADIUS)

        vl = Utilities.radians2ticks(vl)
        vr = Utilities.radians2ticks(vr)

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

        # Convert to radians
        left_dx = 2 * math.pi * left_dx / Robot.TICKS_PER_ROTATION
        right_dx = 2 * math.pi * right_dx / Robot.TICKS_PER_ROTATION

        forward_velocity = (Robot.WHEEL_RADIUS / 2) * (left_dx + right_dx) # Might need to fact check this as it was written early in the semester.
        current_estimate.x += forward_velocity * math.cos(current_estimate.z)
        current_estimate.y += forward_velocity * math.sin(current_estimate.z)
        current_estimate.z += (Robot.WHEEL_RADIUS / Robot.AXIS_LENGTH) * (right_dx - left_dx)
        return current_estimate

    def dead_reckon_state_change_better(self, current_estimate, dt):
        """
        Use wheel encoders to estimate the change in position from the current last estimate.
        When a global position estimate is applied, we will ahve to handle
        resetting the motor encoder ticks and then record from there.
        @param current_estimate (Vector3) an estimate of the current robot's body frame.
        @param dt (float) the time elapsed since the last measurement.
        @returns a Vector3 containing the new estimate.
        """
        vl = (kp.get_motor_position_counter(self.left_motor_port) - self.left_last_ticks) / dt
        vl = vl * Robot.WHEEL_RADIUS * math.pi / 180.0
        vr = (kp.get_motor_position_counter(self.right_motor_port) - self.right_last_ticks) / dt        
        vr = vr * Robot.WHEEL_RADIUS * math.pi / 180.0
        
        # Calculate forward velocity and turning velocity.
        vf = (vr + vl) / 2.0
        omega = (vr - vl) / Robot.AXIS_LENGTH

        # Im pretty sure this literally reduces to the first method lmao
        k00 = vf * math.cos(current_estimate.z)
        k01 = vf * math.sin(current_estimate.z)
        k02 = omega

        k10 = vf * math.cos(current_estimate.z + dt * k02 / 2)
        k11 = vf * math.sin(current_estimate.z + dt * k02 / 2)
        k12 = omega

        k20 = vf * math.cos(current_estimate.z + dt * k12 / 2)
        k21 = vf * math.sin(current_estimate.z + dt * k12 / 2)
        k22 = omega

        k30 = vf * math.cos(current_estimate.z + dt * k22 / 2)
        k31 = vf * math.sin(current_estimate.z + dt * k22 / 2)
        k32 = omega

        row1 = k00 + 2 * (k10 + k20) + k30
        row2 = k01 + 2 * (k11 + k21) + k31
        row3 = k02 + 2 * (k12 + k22) + k32

        current_estimate.x = current_estimate.x + (dt / 6.0) * row1
        current_estimate.y = current_estimate.y + (dt / 6.0) * row2
        current_estimate.z = current_estimate.z + (dt / 6.0) * row3
        return current_estimate

class Camera:
    def __init__(self):
        kp.camera_open()
        kp.camera_update()

    def update(self):
        kp.camera_update()

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
    
    def execute(self, robot):
        """
        Record path/velocities.
        Use camera to find QR code with value item_id.
        Navigate to QR code with item_id so that it lines up with gripper.
        Use distance sensor/QR code size to determine if QR code is within gripper reach.
        Apply reverse motor instructions to get close to original path.
        Get back on the line tracks.
        """
        pass

class ActionGoToZone(BaseAction):
    def __init__(self, current_zone, target_zone):
        """
        A subtask that brings the robot assuming it is in the current_zone to the target_zone.
        """
        self.current_zone = current_zone
        self.target_zone = target_zone
    
    def execute(self, robot):
        """
        Use IRArray to follow line until junctions, use current zone to determine where to turn at junctions.
        Update state position and orientation to absolute values when a junction is reached to lower accumulative error.
        """
        if robot.current_zone == self.target_zone:
            return True, []
        elif robot.current_zone == "Terminal":
            # Turn towards target zone and follow line.
            pass
        elif robot.current_zone == "Dropoff":
            # Follow line until terminal, check that bearings are right (robot.get_forward() and (robot.position - terminal_zone) are in the same direction, otherwise correct)
            pass
        elif robot.current_zone == "ZoneA":
            # Follow line until terminal, check that bearings are right (robot.get_forward() and (robot.position - terminal_zone) are in the same direction, otherwise correct)
            pass
        elif robot.current_zone == "ZoneB":
            # Follow line until terminal, check that bearings are right (robot.get_forward() and (robot.position - terminal_zone) are in the same direction, otherwise correct)
            pass
        elif robot.current_zone == "ZoneC":
            # Follow line until terminal, check that bearings are right (robot.get_forward() and (robot.position - terminal_zone) are in the same direction, otherwise correct)
            pass
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

    def __init__(self, motors: DifferentialDrive, ir_array, gripper, camera):
        # let x, y be their cartesian coordinates and let z be the orientation theta.
        self.frame = Vector3(0,0,0) # theta should be converted to global orientation each time we turn or sense a black line with IR since this prevents error accumulation.
        self.controller = motors
        self.ir_array = ir_array
        self.gripper = gripper
        self.camera_qr_codes = []
        self.object_held = None
        self.current_zone = None
        self.target_zone = None
        self.action_queue = []
        self.last_timestamp = kp.seconds()

    def get_forward(self):
        return Vector2(math.cos(self.frame.z), math.sin(self.frame.z))

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
        self.frame = self.controller.dead_reckon_state_change_better(self.frame, dt)

    def process_action(self):
        """
        Run the action loop if there are any queued up.
        """
        if len(self.action_queue) <= 0:
            return
        
        action = self.action_queue[0]
        finished, new_events = action.execute(self)
        if finished:
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
			self.server.robot.append(self.data)
        # just send back the same data, but upper-cased
		self.request.sendall(self.data.upper())

def robot_thread(robot, robot_lock):
	print("test")
	while True:
		time.sleep(2)
		with robot_lock:
			if len(robot) > 0:
				if robot[-1] == "forward":
					kp.mav(0, 200)
				else:
					kp.mav(0, 0)
			print(robot)
                
def server_thread(robot, robot_lock):
	HOST, PORT = "0.0.0.0", 8100
	server = RobotTCPServer((HOST, PORT), MyTCPHandler, robot, robot_lock)
	server.serve_forever()
	r_thread.join() # yikes!

if __name__ == "__main__":
	sys.stdout = os.fdopen(sys.stdout.fileno(),"w",0)
	print("here " + str(sys.version_info))
	robot_lock = threading.RLock()
	robot = []
	HOST, PORT = "0.0.0.0", 8101
	server = RobotTCPServer((HOST, PORT), MyTCPHandler, robot, robot_lock)
	r_thread = threading.Thread(target=robot_thread, args=(robot, robot_lock))
	r_thread.setDaemon(True)
	r_thread.start()
	server.serve_forever()
	r_thread.join()