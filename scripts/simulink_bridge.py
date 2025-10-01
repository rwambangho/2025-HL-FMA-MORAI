#!/usr/bin/env python3
import rospy, math
from std_msgs.msg import Float64MultiArray, Int16
from morai_msgs.msg import CtrlCmd, GetTrafficLightStatus, EgoVehicleStatus
from geometry_msgs.msg import Twist  # in case you want to accept Twist directly

class SimulinkBridge:
    def __init__(self):
        rospy.init_node('simulink_bridge', anonymous=True)

        # Subscribers from Simulink (we expect Float64MultiArray: [speed_m_s, steer_rad])
        self.sub_sim = rospy.Subscriber('/simulink_cmd', Float64MultiArray, self.cb_sim_cmd, queue_size=1)

        # Publisher to Morai
        self.pub_ctrl = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        # MORAI -> Simulink publications (converted to simple standard msgs)
        self.sub_ego = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.cb_ego, queue_size=1)
        self.pub_ego_s = rospy.Publisher('/simulink_ego', Float64MultiArray, queue_size=1)

        self.sub_tl = rospy.Subscriber('/GetTrafficLightStatus', GetTrafficLightStatus, self.cb_tl, queue_size=1)
        self.pub_tl_s = rospy.Publisher('/simulink_traffic', Int16, queue_size=1)

        # Safety limits
        self.max_speed_kph = 40.0
        self.max_steer_rad = math.radians(40)

        rospy.loginfo("SimulinkBridge initialized")

    def cb_sim_cmd(self, msg: Float64MultiArray):
        # Expect msg.data = [speed_m_s, steer_rad]
        data = msg.data
        if not data or len(data) < 2:
            rospy.logwarn("simulink_cmd: unexpected message length")
            return

        speed_m_s = float(data[0])
        steer_rad = float(data[1])

        # Clip / sanitize
        speed_m_s = max(0.0, speed_m_s)            # no reverse in this simple mapping
        speed_kph = min(self.max_speed_kph, speed_m_s * 3.6)
        steer_rad = max(-self.max_steer_rad, min(self.max_steer_rad, steer_rad))

        # Build CtrlCmd message robustly
        cmd = CtrlCmd()
        # Try velocity-control mode if available
        cmd.longlCmdType = 2  # try velocity control if simulator supports
        # Many morai versions expose a 'speed' field for km/h. Try common names:
        if hasattr(cmd, 'speed'):
            try:
                cmd.speed = float(speed_kph)
            except Exception:
                pass
        elif hasattr(cmd, 'vel'):
            try:
                cmd.vel = float(speed_kph)
            except Exception:
                pass
        else:
            # fallback: use accel/brake (simple)
            cmd.longlCmdType = 1
            if speed_m_s > 0.1:
                cmd.accel = 0.3
                cmd.brake = 0.0
            else:
                cmd.accel = 0.0
                cmd.brake = 1.0

        # Steering field: try common names
        for name in ('steer', 'steering', 'steer_angle', 'wheel_angle'):
            if hasattr(cmd, name):
                try:
                    setattr(cmd, name, steer_rad)
                    break
                except Exception:
                    continue

        self.pub_ctrl.publish(cmd)
        rospy.logdebug(f"Published CtrlCmd: speed_kph={speed_kph:.2f} steer={steer_rad:.3f}")

    def cb_ego(self, msg: EgoVehicleStatus):
        arr = Float64MultiArray()
        try:
            x = float(msg.position.x)
            y = float(msg.position.y)
            heading = float(getattr(msg, 'heading', 0.0))  # deg
            vel = float(msg.velocity.x)  # m/s
            arr.data = [x, y, heading, vel]
            self.pub_ego_s.publish(arr)
        except Exception as e:
            rospy.logwarn(f"cb_ego exception: {e}")

    def cb_tl(self, msg: GetTrafficLightStatus):
        st = int(getattr(msg, 'trafficLightStatus', -1))
        out = Int16()
        out.data = st
        self.pub_tl_s.publish(out)

if __name__ == '__main__':
    try:
        b = SimulinkBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
