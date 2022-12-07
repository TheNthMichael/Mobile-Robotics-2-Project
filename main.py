#!/usr/bin/python
import os, sys
import kipr as kp
import SocketServer
import math

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
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def sign(x):
        if x < 0:
            return -1
        else:
            return 1

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
        self.left_last_ticks = 0
        self.right_last_ticks = 0
        # reset motor encoders.

    def drive(vf, omega):
        """
        Change the forward speed and turning speed of the robot.
        @param vf - The forward velocity of the robot (constrained by omega)
        @param omeaga - The angular velocity of the robot (constrained by vf)
        """
        pass

    def dead_reckon_state_change(self, current_estimate, dt):
        """
        Use wheel encoders to estimate the change in position from the current last estimate.
        When a global position estimate is applied, we will ahve to handle
        resetting the motor encoder ticks and then record from there.
        @param current_estimate (Vector3) an estimate of the current robot's body frame.
        @param dt (float) the time elapsed since the last measurement.
        @returns a Vector3 containing the new estimate.
        """
        pass

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

    def __init__(self, motors, ir_array, gripper, camera):
        # let x, y be their cartesian coordinates and let z be the orientation theta.
        self.frame = Vector3(0,0,0) # theta should be converted to global orientation each time we turn or sense a black line with IR since this prevents error accumulation.
        self.ir_array = ir_array
        self.gripper = gripper
        self.camera_qr_codes = []
        self.object_held = None
        self.current_zone = None
        self.target_zone = None
        self.action_queue = []

    def get_forward(self):
        return Vector2(math.cos(self.frame.z), math.sin(self.frame.z))

    def update_state(self):
        """
        Update the state with both local and global methods.
        Local methods include wheel encoders/deadreckoning.
        Global methods include using IR sensors to adjust robot frame when we cross terminals/turn onto the line.
        """
        pass



class MyTCPHandler(SocketServer.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """
    def handle(self):
        """
        Handle connections and data from requests.
        We will need to define events that let us send both low level and high level
        commands to the robot. We can use poll requests to also send back state information
        about the robot to be visualized.
        """
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print("{} wrote:".format(self.client_address[0]))
        print(self.data)
        # just send back the same data, but upper-cased
        self.request.sendall(self.data.upper())

if __name__ == "__main__":
    sys.stdout = os.fdopen(sys.stdout.fileno(),"w",0)
    print("here " + str(sys.version_info))
    HOST, PORT = "localhost", 8100
    server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)
    server.serve_forever()