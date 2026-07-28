#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import EntityWrench, Entity
from ardrone_autonomy.msg import Navdata
import math
import random

# Drone states
Emergency = 0
Inited = 1
LANDED_MODEL = 2
FLYING_MODEL = 3
HOVERING_MODEL = 4
TEST_MODEL = 5
TAKINGOFF_MODEL = 6
GOTOHOVER_MODEL = 7
LANDING_MODEL = 8
LOOPING_MODEL = 9

def rotate_vector(q, v):
    # q is [w, x, y, z], v is [x, y, z]
    w, x, y, z = q
    vx, vy, vz = v
    # temp = q * v
    tw = -x*vx - y*vy - z*vz
    tx =  w*vx + y*vz - z*vy
    ty =  w*vy - x*vz + z*vx
    tz =  w*vz + x*vy - y*vx
    # result = temp * q_conj
    rx = tx*w - tw*x - ty*z + tz*y
    ry = ty*w - tw*y - tz*x + tx*z
    rz = tz*w - tw*z - tx*y + ty*x
    return [rx, ry, rz]

def rotate_vector_inverse(q, v):
    w, x, y, z = q
    return rotate_vector([w, -x, -y, -z], v)

def quaternion_to_euler(q):
    w, x, y, z = q
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]

class PIDController:
    def __init__(self, gain_p=5.0, gain_d=1.0, gain_i=0.0, time_constant=0.0, limit=-1.0):
        self.gain_p = gain_p
        self.gain_d = gain_d
        self.gain_i = gain_i
        self.time_constant = time_constant
        self.limit = limit
        
        self.input = 0.0
        self.dinput = 0.0
        self.output = 0.0
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def update(self, new_input, x, dx, dt):
        if self.limit > 0.0 and abs(new_input) > self.limit:
            new_input = (-1.0 if new_input < 0 else 1.0) * self.limit

        if dt + self.time_constant > 0.0:
            self.input = (dt * new_input + self.time_constant * self.input) / (dt + self.time_constant)
            self.dinput = (new_input - self.input) / (dt + self.time_constant)

        self.p = self.input - x
        self.d = self.dinput - dx
        self.i = self.i + dt * self.p

        self.output = self.gain_p * self.p + self.gain_d * self.d + self.gain_i * self.i
        return self.output

    def reset(self):
        self.input = 0.0
        self.dinput = 0.0
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0
        self.output = 0.0

class ARDroneSimulationDriver(Node):
    def __init__(self):
        super().__init__('ardrone_simulation_driver')
        
        # Multi-drone support: model name parameter
        self.declare_parameter('model_name', 'ardrone_gazebo')
        self.model_name = self.get_parameter('model_name').value
        
        # Parameters
        self.state = LANDED_MODEL
        self.pos_ctrl = False
        self.vel_mode = False
        self.cmd_val = Twist()
        
        self.battery_percent = 100.0
        self.max_flight_time = 20 * 60.0
        self.current_flight_time = 0.0
        self.time_after_cmd = 0.0
        
        self.last_time = None
        self.last_velocity = [0.0, 0.0, 0.0]
        self.last_velocity_world = [0.0, 0.0, 0.0]
        self.last_pos = [0.0, 0.0, 0.0]
        self.filtered_acc = [0.0, 0.0, 0.0]
        self.filtered_acc_world = [0.0, 0.0, 0.0]
        
        # Noise settings
        self.motion_small_noise = 0.05
        self.motion_drift_noise = 0.03
        self.motion_drift_noise_time = 5.0
        self.drift_noise_counter = 0.0
        self.drift_noise = [0.0, 0.0, 0.0, 0.0]
        
        # Drone physical parameters
        self.mass = 1.477
        self.inertia = [0.1152, 0.1152, 0.218]
        self.max_force = 30.0
        
        # Initialize PID Controllers
        self.pid_roll = PIDController(gain_p=10.0, gain_d=5.0, gain_i=0.0, limit=0.5)
        self.pid_pitch = PIDController(gain_p=10.0, gain_d=5.0, gain_i=0.0, limit=0.5)
        self.pid_yaw = PIDController(gain_p=2.0, gain_d=1.0, gain_i=0.0, limit=1.5)
        
        self.pid_vel_x = PIDController(gain_p=5.0, gain_d=2.3, gain_i=0.0, limit=2.0)
        self.pid_vel_y = PIDController(gain_p=5.0, gain_d=2.3, gain_i=0.0, limit=2.0)
        self.pid_vel_z = PIDController(gain_p=5.0, gain_d=1.0, gain_i=0.0, limit=-1.0)
        
        self.pid_pos_x = PIDController(gain_p=1.1, gain_d=0.0, gain_i=0.0, limit=5.0)
        self.pid_pos_y = PIDController(gain_p=1.1, gain_d=0.0, gain_i=0.0, limit=5.0)
        self.pid_pos_z = PIDController(gain_p=1.0, gain_d=0.2, gain_i=0.0, limit=-1.0)

        # Subscriptions (relative topics for namespace support)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.takeoff_sub = self.create_subscription(Empty, 'ardrone/takeoff', self.takeoff_callback, 10)
        self.land_sub = self.create_subscription(Empty, 'ardrone/land', self.land_callback, 10)
        self.reset_sub = self.create_subscription(Empty, 'ardrone/reset', self.reset_callback, 10)
        self.posctrl_sub = self.create_subscription(Bool, 'ardrone/posctrl', self.posctrl_callback, 10)
        self.velmode_sub = self.create_subscription(Bool, 'ardrone/vel_mode', self.velmode_callback, 10)
        
        # Gazebo state subscription (topic set via remapping in launch file)
        self.odom_sub = self.create_subscription(Odometry, 'gz_odom', self.odom_callback, 10)
        
        # Publishers
        self.navdata_pub = self.create_publisher(Navdata, 'ardrone/navdata', 10)
        self.wrench_pub = self.create_publisher(EntityWrench, '/world/empty/wrench', 10)
        
        self.get_logger().info(f"AR.Drone driver initialized for model: {self.model_name}")

    def cmd_callback(self, msg):
        self.cmd_val = msg

    def takeoff_callback(self, msg):
        if self.state == LANDED_MODEL:
            self.state = TAKINGOFF_MODEL
            self.time_after_cmd = 0.0
            self.get_logger().info("The drone will now take off.")

    def land_callback(self, msg):
        if self.state in [FLYING_MODEL, TAKINGOFF_MODEL]:
            self.state = LANDING_MODEL
            self.time_after_cmd = 0.0
            self.get_logger().info("The drone will now land.")

    def reset_callback(self, msg):
        self.get_logger().info("Reset requested (reset simulation driver state).")
        self.reset_state()

    def posctrl_callback(self, msg):
        self.pos_ctrl = msg.data

    def velmode_callback(self, msg):
        self.vel_mode = msg.data

    def reset_state(self):
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()
        self.pid_vel_x.reset()
        self.pid_vel_y.reset()
        self.pid_vel_z.reset()
        self.pid_pos_x.reset()
        self.pid_pos_y.reset()
        self.pid_pos_z.reset()
        self.state = LANDED_MODEL
        self.current_flight_time = 0.0
        self.battery_percent = 100.0

    def update_state(self, pos, vel, dt):
        if self.state == TAKINGOFF_MODEL:
            self.time_after_cmd += dt
            if self.time_after_cmd > 1.5:
                self.state = FLYING_MODEL
                self.get_logger().info("Entering flying model!")
        elif self.state == LANDING_MODEL:
            self.time_after_cmd += dt
            # Smoothly transition to landed state when height is below 0.08m and descent rate is low, or safety timeout of 10s is reached
            if pos[2] <= 0.08 or self.time_after_cmd > 10.0:
                self.state = LANDED_MODEL
                self.get_logger().info("Landed!")
        else:
            self.time_after_cmd = 0.0

    def odom_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        q = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        ang_vel = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
        
        # Convert vel (body frame) to world frame
        vel_world = rotate_vector(q, vel)
        
        if self.last_time is None:
            self.last_time = t
            self.last_velocity = vel
            self.last_velocity_world = vel_world
            self.last_pos = pos
            return
        
        dt = t - self.last_time
        if dt < 0.001:
            return
        
        # Calculate acceleration in world frame and filter it
        raw_acc_world = [(vel_world[i] - self.last_velocity_world[i]) / dt for i in range(3)]
        alpha = 0.2
        acc_world = [alpha * raw_acc_world[i] + (1.0 - alpha) * self.filtered_acc_world[i] for i in range(3)]
        self.filtered_acc_world = acc_world
        
        # Calculate acceleration in body frame and filter it
        raw_acc_body = [(vel[i] - self.last_velocity[i]) / dt for i in range(3)]
        acc_body = [alpha * raw_acc_body[i] + (1.0 - alpha) * self.filtered_acc[i] for i in range(3)]
        self.filtered_acc = acc_body
        
        # Convert quaternion to euler
        euler = quaternion_to_euler(q)
        
        # Apply noise to commands if flying
        cmd = Twist()
        cmd.linear.x = self.cmd_val.linear.x
        cmd.linear.y = self.cmd_val.linear.y
        cmd.linear.z = self.cmd_val.linear.z
        cmd.angular.x = self.cmd_val.angular.x
        cmd.angular.y = self.cmd_val.angular.y
        cmd.angular.z = self.cmd_val.angular.z
        
        if self.state in [FLYING_MODEL, TAKINGOFF_MODEL, LANDING_MODEL]:
            self.drift_noise_counter += dt
            if self.drift_noise_counter > self.motion_drift_noise_time:
                self.drift_noise = [2.0 * self.motion_drift_noise * (random.random() - 0.5) for _ in range(4)]
                self.drift_noise_counter = 0.0
                
            cmd.angular.x += self.drift_noise[0] + 2.0 * self.motion_small_noise * (random.random() - 0.5)
            cmd.angular.y += self.drift_noise[1] + 2.0 * self.motion_small_noise * (random.random() - 0.5)
            cmd.linear.z  += self.drift_noise[2] + 2.0 * self.motion_small_noise * (random.random() - 0.5)
            cmd.angular.z += self.drift_noise[3] + 2.0 * self.motion_small_noise * (random.random() - 0.5)
            
        # Update flight states
        self.update_state(pos, vel, dt)
        
        # Calculate force and torque
        force, torque = self.compute_control(pos, q, vel_world, vel, ang_vel, acc_world, acc_body, euler, cmd, dt)
        
        # Scale force and torque by dt / 0.001 to preserve impulse on the non-persistent topic
        scale = dt / 0.001
        
        # Clamp scaled values to prevent physics simulator divergence under temporary dt lag
        force_scaled = [max(0.0, min(150.0, f * scale)) for f in force]
        # Allow negative/positive scaled torques, clamp to +/- 30.0 N-m
        torque_scaled = [max(-30.0, min(30.0, t * scale)) for t in torque]
        
        # Convert scaled force and torque to world frame
        force_world = rotate_vector(q, force_scaled)
        torque_world = rotate_vector(q, torque_scaled)
        
        # Apply wrench to link 'base_link' of model (dynamic for multi-drone)
        wrench_msg = EntityWrench()
        wrench_msg.header.stamp = msg.header.stamp
        wrench_msg.header.frame_id = 'world'
        wrench_msg.entity.name = f'{self.model_name}::base_link'
        wrench_msg.entity.type = Entity.LINK
        wrench_msg.wrench.force.x = force_world[0]
        wrench_msg.wrench.force.y = force_world[1]
        wrench_msg.wrench.force.z = force_world[2]
        wrench_msg.wrench.torque.x = torque_world[0]
        wrench_msg.wrench.torque.y = torque_world[1]
        wrench_msg.wrench.torque.z = torque_world[2]
        
        # Only publish wrench when airborne - publishing zero wrenches while landed
        # causes Gazebo to spam "Entity not found" errors at every physics step
        if self.state != LANDED_MODEL:
            self.wrench_pub.publish(wrench_msg)
        
        # Simulate battery drain
        if self.state != LANDED_MODEL:
            self.current_flight_time += dt
            self.battery_percent = max(0.0, 100.0 - (100.0 / self.max_flight_time) * self.current_flight_time)
        else:
            self.current_flight_time = 0.0
            
        # Publish Navdata
        self.publish_navdata(pos, vel, acc_world, euler, msg.header.stamp)
        
        # Save variables
        self.last_time = t
        self.last_velocity = vel
        self.last_velocity_world = vel_world
        self.last_pos = pos

    def compute_control(self, pos, q, vel_world, vel_body, ang_vel, acc_world, acc_body, euler, cmd, dt):
        roll, pitch, yaw = euler
        
        # Rotate vectors to heading frame (yaw only)
        heading_q = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        vel_xy = rotate_vector_inverse(heading_q, vel_world)
        acc_xy = rotate_vector_inverse(heading_q, acc_world)
        
        # Angular velocity from ROS twist is already in body frame
        ang_vel_body = ang_vel
        
        gravity = 9.81
        # Gravity in body frame
        gravity_body = rotate_vector_inverse(q, [0.0, 0.0, -9.81])
        gravity_len = math.sqrt(gravity_body[0]**2 + gravity_body[1]**2 + gravity_body[2]**2)
        
        dot_prod = gravity_body[2] * -9.81
        if abs(dot_prod) > 1e-6:
            load_factor = (gravity_len * gravity_len) / dot_prod
        else:
            load_factor = 1.0
            
        force = [0.0, 0.0, 0.0]
        torque = [0.0, 0.0, 0.0]
        
        if self.state in [TAKINGOFF_MODEL, LANDING_MODEL]:
            torque[2] = self.inertia[2] * self.pid_yaw.update(cmd.angular.z, ang_vel[2], 0.0, dt)
            vz_target = 0.8 if self.state == TAKINGOFF_MODEL else -0.4
            force[2] = self.mass * (self.pid_vel_z.update(vz_target, vel_body[2], acc_body[2], dt) + load_factor * gravity)
        elif self.pos_ctrl:
            if self.state == FLYING_MODEL:
                vx = self.pid_pos_x.update(cmd.linear.x, pos[0], pos[0] - self.last_pos[0], dt)
                vy = self.pid_pos_y.update(cmd.linear.y, pos[1], pos[1] - self.last_pos[1], dt)
                vz = self.pid_pos_z.update(cmd.linear.z, pos[2], pos[2] - self.last_pos[2], dt)
                
                vb = rotate_vector_inverse(heading_q, [vx, vy, vz])
                
                pitch_command = self.pid_vel_x.update(vb[0], vel_xy[0], acc_xy[0], dt) / gravity
                roll_command = -self.pid_vel_y.update(vb[1], vel_xy[1], acc_xy[1], dt) / gravity
                
                torque[0] = self.inertia[0] * self.pid_roll.update(roll_command, roll, ang_vel_body[0], dt)
                torque[1] = self.inertia[1] * self.pid_pitch.update(pitch_command, pitch, ang_vel_body[1], dt)
                force[2] = self.mass * (self.pid_vel_z.update(vz, vel_body[2], acc_body[2], dt) + load_factor * gravity)
        else:
            if self.state == FLYING_MODEL:
                pitch_command = self.pid_vel_x.update(cmd.linear.x, vel_xy[0], acc_xy[0], dt) / gravity
                roll_command = -self.pid_vel_y.update(cmd.linear.y, vel_xy[1], acc_xy[1], dt) / gravity
                
                torque[0] = self.inertia[0] * self.pid_roll.update(roll_command, roll, ang_vel_body[0], dt)
                torque[1] = self.inertia[1] * self.pid_pitch.update(pitch_command, pitch, ang_vel_body[1], dt)
            else:
                if self.vel_mode:
                    pitch_command = self.pid_vel_x.update(cmd.angular.x, vel_xy[0], acc_xy[0], dt) / gravity
                    roll_command = -self.pid_vel_y.update(cmd.angular.y, vel_xy[1], acc_xy[1], dt) / gravity
                    
                    torque[0] = self.inertia[0] * self.pid_roll.update(roll_command, roll, ang_vel_body[0], dt)
                    torque[1] = self.inertia[1] * self.pid_pitch.update(pitch_command, pitch, ang_vel_body[1], dt)
                else:
                    torque[0] = self.inertia[0] * self.pid_roll.update(cmd.angular.x, roll, ang_vel_body[0], dt)
                    torque[1] = self.inertia[1] * self.pid_pitch.update(cmd.angular.y, pitch, ang_vel_body[1], dt)
                    
            torque[2] = self.inertia[2] * self.pid_yaw.update(cmd.angular.z, ang_vel[2], 0.0, dt)
            force[2] = self.mass * (self.pid_vel_z.update(cmd.linear.z, vel_body[2], acc_body[2], dt) + load_factor * gravity)
            
        if self.max_force > 0.0 and force[2] > self.max_force:
            force[2] = self.max_force
        if force[2] < 0.0:
            force[2] = 0.0
            
        if self.state == LANDED_MODEL:
            force = [0.0, 0.0, 0.0]
            torque = [0.0, 0.0, 0.0]
        elif self.state == TAKINGOFF_MODEL:
            force[2] *= 1.5
            torque[0] *= 1.5
            torque[1] *= 1.5
        # No modification of forces during landing model to let PID stabilize the descent
            
        # Limit torques to prevent solver explosion (e.g. max 2.0 N-m)
        max_torque = 2.0
        for i in range(3):
            if torque[i] > max_torque:
                torque[i] = max_torque
            elif torque[i] < -max_torque:
                torque[i] = -max_torque
            
        return force, torque

    def publish_navdata(self, pos, vel, acc, euler, stamp):
        navdata = Navdata()
        navdata.header.stamp = stamp
        navdata.header.frame_id = 'base_link'
        navdata.battery_percent = float(self.battery_percent)
        navdata.state = int(self.state)
        
        navdata.rot_x = float(euler[0] * 180.0 / math.pi)
        navdata.rot_y = float(euler[1] * 180.0 / math.pi)
        navdata.rot_z = float(euler[2] * 180.0 / math.pi)
        
        navdata.altd = int(pos[2] * 100.0) # estimated altitude in cm
        
        navdata.vx = float(vel[0] * 1000.0) # linear velocity in mm/s
        navdata.vy = float(vel[1] * 1000.0)
        navdata.vz = float(vel[2] * 1000.0)
        
        navdata.ax = float(acc[0] / 9.81) # linear acceleration in g
        navdata.ay = float(acc[1] / 9.81)
        navdata.az = float(acc[2] / 9.81)
        
        self.navdata_pub.publish(navdata)

def main(args=None):
    rclpy.init(args=args)
    node = ARDroneSimulationDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
