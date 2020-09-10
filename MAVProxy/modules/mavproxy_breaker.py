from MAVProxy.modules.lib import mp_module, mp_util
from pymavlink import mavutil


class BreakerModule(mp_module.MPModule):
    """A 'breaker' or 'fuse' that drops connections when certain conditions are detected.

    If you have a connection to an external program that's issuing mavlink commands to a drone, you often want to stop
    listening to those programs when certain overriding conditions are reached. For example, if the autopilot has
    determined that an emergency landing or RTL is required, then it (generally) no longer makes sense to let external
    programs try to issue commands.

    A 'breaker' keeps track of connections to programs like this, and when these overriding conditions arise it
    disconnects them.
    """

    def __init__(self, mpstate):
        super().__init__(
            mpstate, "breaker", "breaker control", public=True
        )
        self.add_command(
            "breaker",
            self.handle_command,
            "breaker control",
            ["<list|add|remove|sysid>"],
        )
        self._devices = set()

    def handle_command(self, args):
        if not args or args[0] == "list":
            self.handle_list()
        elif args[0] == "add":
            if len(args) != 2:
                print("Usage: breaker add DEVICE")
                return
            self.handle_add(*args[1:])
        elif args[0] == "remove":
            if len(args) != 2:
                print("Usage: breaker remove DEVICE")
                return
            self.handle_remove(*args[1:])
        elif args[0] == "sysid":
            if len(args) != 3:
                print("Usage: breaker sysid SYSID DEVICE")
                return
            self.handle_breaker(*args[1:])
        else:
            print("usage: breaker <list|add|remove|sysid>")

    def handle_list(self):
        pass

    def handle_add(self, device):
        print(f"Adding breaker {device}")
        try:
            conn = mavutil.mavlink_connection(
                device, input=False, source_system=self.settings.source_system
            )
            conn.mav.srcComponent = self.settings.source_component
        except Exception:
            print(f"Breaker failed to connect to {device}")
            return
        self.mpstate.mav_outputs.append(conn)
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            # TODO: Huh? Why is this ignored?
            pass

        self._devices.add(device)

    def handle_remove(self, device):
        for i, conn in enumerate(self.mpstate.mav_outputs):
            if str(i) == device or conn.address == device:
                print(f"Removing breaker {conn.address}")
                try:
                    mp_util.child_fd_list_add(conn.port.fileno())
                except Exception:
                    pass
                conn.close()
                self.mpstate.mav_outputs.pop(i)

                self._devices.discard(device)

    def handle_sysid(self, sysid, device):
        print(f"Adding breaker {device} for sysid {sysid}")
        try:
            conn = mavutil.mavlink_connection(
                device, input=False, source_system=self.settings.source_system
            )
            conn.mav.srcComponent = self.settings.source_component
        except Exception:
            print(f"Breaker failed to connect to {device}")
            return
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass
        if sysid in self.mpstate.sysid_outputs:
            self.mpstate.sysid_outputs[sysid].close()
        self.mpstate.sysid_outputs[sysid] = conn

        self._devices.add(device)

    def mavlink_packet(self, m):
        # This looks for fence violations, tripping the breaker if one is detected. 
        if m.get_type() == 'FENCE_STATUS':
            if m.breach_status == 1:
                print('Fence breach detected. Disconnecting breaker.')
                for device in self._devices:
                    self.handle_remove(device)

        # TODO: Is there a more direct way of determining that we've entered RTL?


BORING = {
    "NAV_CONTROLLER_OUTPUT",
    "RAW_IMU",
    "TERRAIN_REPORT",
    "SERVO_OUTPUT_RAW",
    "GPS_RAW_INT",
    "TIMESYNC",
    "GLOBAL_POSITION_INT",
    "MEMINFO",
    "MISSION_ITEM_REACHED",
    "RC_CHANNELS",
    "EKF_STATUS_REPORT",
    "SCALED_PRESSURE",
    "VIBRATION",
    "HOME_POSITION",
    "SIMSTATE",
    "BATTERY_STATUS",
    "AHRS2",
    "POSITION_TARGET_GLOBAL_INT",
    "PARAM_VALUE",
    "HWSTATUS",
    "SCALED_IMU2",
    "GPS_GLOBAL_ORIGIN",
    "MISSION_CURRENT",
    "VFR_HUD",
    # "COMMAND_ACK",
    "LOCAL_POSITION_NED",
    "STATUSTEXT",
    "ATTITUDE",
    "SCALED_IMU3",
    "SENSOR_OFFSETS",
    "SYS_STATUS",
    "AHRS",
    "SYSTEM_TIME",
    "HEARTBEAT",
    "POWER_STATUS",
    "FENCE_STATUS"
}


def init(mpstate):
    return BreakerModule(mpstate)
