

#importing libs

import argparse
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
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = [[10,0,3,0], [10,10,3,0], [0,10,3,0], [0,0,3,0]]
        self.in_mission = True
        self.check_state = {}

        # initial reference square edge (four edges)
        self.edge = 0

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if self.target_position[2] - (-1 * self.local_position[2] )< 0.01:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if self.count < 4:
                if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                    self.waypoint_transition() 
        else:
            if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                    self.landing_transition()

    pass

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()
    pass

    def state_callback(self):
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()
        pass

    def calculate_box(self):
        if self.edge <= 3: 
            i = self.edge
            self.edge += 1
        return self.all_waypoints[i]

        pass

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                                self.global_position[1],
                                self.global_position[2])
        self.flight_state = States.ARMING


    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        time.sleep(2)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        print("waypoint transition")
        nwp = self.calculate_box()
        self.target_position[0] = nwp[0]
        self.target_position[1] = nwp[1]
        self.target_position[2] = nwp[2]
        self.cmd_position(nwp[0],nwp[1],nwp[2],nwp[3])
        #time.sleep(4)
        print("Waypoint Target ["+str(nwp[0])+","+str(nwp[1])+","+str(nwp[2])+","+str(nwp[3])+"]")
        self.flight_state = States.WAYPOINT


    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    drone.start()