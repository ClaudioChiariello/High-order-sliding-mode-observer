#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from scipy.spatial.transform import Rotation

from .gather_sensors import Callbacks
from urdf_parser_py.urdf import URDF
from . import robot
from geometry_msgs.msg import Twist


"""dot_x1 = x2                      x = [x1] -> [x,y,roll,yaw]
    dot_x2 = f(x,u)                     [x2] -> [vx,vy,dot_roll,dot_yaw]"""
class TruckStateObserver(Node):

    def __init__(self):
        super().__init__('truck_state_observer')

        self.declare_parameter('scale_tanh', 10.0)
        self.scale_tanh = self.get_parameter('scale_tanh').value

        self.declare_parameter("des_vel_x", 10.0)
        self.des_vel_x = self.get_parameter('des_vel_x').value

        self.declare_parameter("des_omega_z", 0.5) #giving a desired angular vel prevent the velocity to reach a steady state
        self.des_omega_z = self.get_parameter('des_omega_z').value

        self.total_time = 0.0
        # self.x = np.float32(0.0)
        # self.y = np.float32(0.0)
        # self.roll = np.float32(0.0)
        # self.yaw = np.float32(0.0)

        self.state_x1 = np.zeros(4, dtype="float32")
        self.observed_state_x1 = np.zeros(4, dtype="float32")
        self.estimate_error_x1 = np.zeros(4, dtype="float32")

        self.state_x2 = np.zeros(4, dtype="float32")
        self.observed_state_x2= np.zeros(4, dtype="float32")
        self.estimate_error_x2 = np.zeros(4, dtype="float32")
    
        self.triggered = False
        self.state = np.concatenate(
            (self.state_x1, self.state_x2)
        )
        self.observed_state = np.concatenate(
            (self.observed_state_x1, self.observed_state_x2)
        )

        self.truck = robot.robot()
        self.cb = Callbacks(self)

        self.previous_e = np.zeros(3,dtype="float32")

        """The gz integration time is 1e-3.
            - If use_sim_time:=True and 
                self.fixed_dt = 1e-2 
                Gazebo_dt = 1e-3
                Il get_now() si aggiorna con i tempi di gazebo, ma tra un get_now() e il successivo passeranno sempre
                10ms (per via della timer callback) e quindi la frequenza di controllo resta 10ms. Nonostante la get_now 
                si aggiorni con frequenza di 1e-3, leggerai la differenza tra una get_now e la successiva dopo 10 iterazioni
                di gazebo (perché la callback è 10 volte più lenta), e quindi sempre 10 ms

            - if use_sim_time:=True                 
                self.fixed_dt = 1e-4 
                Gazebo_dt = 1e-3 (LO vedi nel world)
                il get_now() prenderà i tempi di gazebo, e tra un get_now e l'altro passeranno 1e-3, perché
                il get_now() si aggiornerà con la frequenza di Gazebo, che ha una risoluzione di 1e-3 (da world file).
                Credo che comunque la callback venga chiamata ogni 1e-4 secondi, ma il get_now() si è aggiornato di 1e-3, 
                che sono i tempi di gazebo

            - if use_sim_time:=False Gazebo is ignored, e la get_now si aggiorna con i tempi del sistema"""
        
        if self.get_parameter("use_sim_time").value:
            self.fixed_dt = 1e-4 #lo rendi abbastanza basso da usare il tempo di gazebo
        else:
            self.fixed_dt = 1e-2 #The observer should run at 10ms

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data', #50Hz
            self.cb.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry', #20Hz
            self.cb.odom_callback,
            10
        )
 
        # The controller frequency will always depend on this, and so the observer if it runs in the callback
        self.timer = self.create_timer(
            self.fixed_dt, 
            self.GazeboSim
        )

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.counter = 0
        self.t0 = self.get_clock().now()
        self.early = None


    def GazeboControl(self):
        msg = Twist()

        # Linear velocity (m/s)
        msg.linear.x = self.des_vel_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Angular velocity (rad/s)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.des_omega_z

        self.publisher_.publish(msg)

    def GazeboSim(self):
         
        """Use the Gazebo timestamp if use_sim_time is true. This code will use the Gazebo sim time, because
        the GazeboSim ha una frequenza minore di quella a cui viene chiamata la callback del publisher, quindi
        il get_clock().now() - previous_clock() restituirà il tempo trascorso in Gazebo
        
        When the use_sim_time is False, this will return 10ms, because the fixed_dt is greater than the 
        GazeboSim dt (che poi non verrà più cagato visto che use_sim_time è falso),
        e quindi questo codice runnerà sempre a 10ms."""
        
        now = self.get_clock().now()
        if self.early is None:
            self.early = now
            return
        dt = (now - self.early).nanoseconds * 1e-9
        self.early = now

        u = self.PdController(
            self.des_vel_x,
            self.des_omega_z,
            dt = dt
        )

        Fx, Mz = u
        
        self.estimate_error_x1 = self.state[:4] - self.observed_state[:4]

        self.state += self.truck.dynamics(self.state, Fx, Mz, add_disturb = True)*dt

        #self.state = self.rk4_step(self.state, Fx, Mz, dt, dist = True)

        dot_observed_state = self.truck.dynamics(self.observed_state, Fx, Mz, add_disturb = False)

        self.HighOrderObserver(dot_observed_state, dt)
 

    def HighOrderObserver(self, dot_observed_state, dt):
        # Questo ti farà aspettare i dati che arrivano ogni 50ms
        # if not self.triggered:
        #     return
        # self.triggered = False 

        p = 0.001
        alpha = 2*np.abs(dot_observed_state[:4]) + 250.0
        lambdaa = np.sqrt(2/(alpha - 2*np.abs(dot_observed_state[:4])))* ( (alpha + 2*np.abs(dot_observed_state[:4]))*(1+p))/(1-p) + 10
        delta = np.sqrt(np.abs(self.estimate_error_x1))
        correction_term = lambdaa*delta*np.tanh(self.scale_tanh*self.estimate_error_x1)

        dot_observed_state[:4] = dot_observed_state[:4] + correction_term
        dot_observed_state[4:] = dot_observed_state[4:] + alpha*np.tanh(self.scale_tanh*self.estimate_error_x1)
        
        self.observed_state += dot_observed_state*dt

        if self.counter % 100 == 0:
            self.get_logger().info(
                f"real (estimated) \n"
                f"Pos : x={self.state[0]:7.3f} ({self.observed_state[0]:7.3f})  "
                f"y={self.state[1]:7.3f} ({self.observed_state[1]:7.3f})\n"
                f"Ang : r={self.state[2]:7.3f} ({self.observed_state[2]:7.3f})  "
                f"ψ={self.state[3]:7.3f} ({self.observed_state[3]:7.3f})\n"
                f"Vel : vx={self.state[4]:7.3f} ({self.observed_state[4]:7.3f})  "
                f"vy={self.state[5]:7.3f} ({self.observed_state[5]:7.3f})\n"
                f"Rate: wx={self.state[6]:7.3f} ({self.observed_state[6]:7.3f})  "
                f"wz={self.state[7]:7.3f} ({self.observed_state[7]:7.3f})\n"
                f"dt={dt:.4f}"
            )

    def PdController(self, vx_des, des_omega_z, dt):
        Kp = [1000, 10]
        Kd = [10, 0.1]
        e_x = vx_des - self.state[4]
        #e_y = self.state[5] - vy_des
        e_wz = des_omega_z - self.cb.wz
        
        Fx = Kp[0]*e_x + Kd[0]*((e_x - self.previous_e[0])/dt)
        Mz = Kp[1]*e_wz + Kd[1]*((e_wz - self.previous_e[2])/dt)

        self.previous_e[:] = [e_x, 0, e_wz]
        return [Fx, Mz]
    

    """be carefull in calling this every self.dt steps, otherwise you are not integrating 
    with the correct timestep. """
    def rk4_step(self, state, Fx, Mz, dt, dist):

        k1 = self.truck.dynamics(state, Fx, Mz, dist)
        k2 = self.truck.dynamics(state + 0.5*dt*k1, Fx, Mz, dist)
        k3 = self.truck.dynamics(state + 0.5*dt*k2, Fx, Mz, dist)
        k4 = self.truck.dynamics(state + dt*k3, Fx, Mz, dist)

        return state + dt/6.0 * (k1 + 2*k2 + 2*k3 + k4)


def main(args=None):

    rclpy.init(args=args)

    node = TruckStateObserver()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()