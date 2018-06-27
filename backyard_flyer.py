import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone): 

    def __init__(self, connection): 
        super().__init__(connection) #calling the super-class initializer 
        self.target_position = np.array([0.0, 0.0, 0.0]) #initializing the target_position array
        self.all_waypoints = [] #creating empty waypoint array 
        self.in_mission = True
        self.check_state = {} #creating empty state list

        # initial state
        self.flight_state = States.MANUAL

   
        #registering LOCAL_POSITION to the callback: local_position_callback

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        # [register_callback(name, fn)] fn is defined in the super-class: Drone
    
    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]
            if altitude > 0.95 * self.target_position[2]:
                if np.linalg.norm(self.local_velocity[:]) < 0.2:
                    self.all_waypoints = self.calculate_box()
                    self.waypoint_transition()
        
        elif self.flight_state == States.WAYPOINT:
            # code below: if the diff. bet. NE target and NE local is < 1 meter
            # AND if the waypoint array is empty, start landing_transiition  
            if np.linalg.norm(self.target_position[:2] - self.local_position[:2]) < 1.0:
                if len(self.all_waypoints) == 0:
                    if np.linalg.norm(self.local_velocity[:2]) < 1.0:
                        self.landing_transition()
                else: #slow down the drone upon hitting each waypoint                     
                    if np.linalg.norm(self.local_velocity[:2]) < 0.5:
                        self.waypoint_transition()

    def velocity_callback(self):
        # Disarming Transition Criterion:
        # Drone must be in LANDING state
        # the diff. bet. current altitude and home altitude must be < 0.05 meters
        # AND the abs value of the local/current altitude be < 0.06 meters
        if self.flight_state == States.LANDING:
            print('Global Altitude:', self.global_position[2])
            print('Home Altitude:', self.global_home[2])
            print('Local Altitude:', self.local_position[2])
            if abs(self.global_position[2] - self.global_home[2]) < 0.05:
                    if abs(self.local_position[2]) < 0.06:
                        self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                    print('Global Position:', self.global_position)
                    print('Home Position:', self.global_home)
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                    self.manual_transition()

    def calculate_box(self):
        print("Setting Home")
        local_waypoints = [[5.0, 0.0, 3.0], [5.0, 5.0, 3.0], [0.0, 5.0, 3.0], [0.0, 0.0, 3.0]]
        return local_waypoints

    def arming_transition(self):
         # Check if the global position is still equal to zero
        if self.global_position[0] == 0.0 and self.global_position[1] == 0.0: 
            print("Gathering Global Position Data...")
            return
            
        print("Arming Transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1],
                               self.global_position[2])
        # sets the current location to be the home position
        # can't write to global_home, hence using set_home_position
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("Takeoff Transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("Waypoint Transition")
        self.target_position = self.all_waypoints.pop(0) 
        print('Target Position:', self.target_position)
        #cmd_position[ north(m), east(m), altitude(m), heading(rad) ] 
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("Landing Transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("Disarm Transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("Manual Transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("Starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://127.0.0.1:5760')
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
