#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_matrix

from .gazebo_model import Callbacks
from . import robot
from observer.utils.data_plotter import DataPlotter
from .states import state as s

from geometry_msgs.msg import Twist


from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import ReentrantCallbackGroup

 


"""dot_x1 = x2                      x = [x1] -> [x,y,roll,yaw]
    dot_x2 = f(x,u)                     [x2] -> [vx,vy,dot_roll,dot_yaw]"""
class TruckStateObserver(Node):

    def __init__(self):
        super().__init__('truck_state_observer')

        self.group = ReentrantCallbackGroup()

        self.declare_parameter('scale_tanh', 10.0)
        self.scale_tanh = self.get_parameter('scale_tanh').value

        self.declare_parameter("des_vel_x", 5.0)
        self.des_vel_x = self.get_parameter('des_vel_x').value

        self.declare_parameter("des_omega_z", 0.5) #giving a desired angular vel prevent the velocity to reach a steady state
        self.des_omega_z = self.get_parameter('des_omega_z').value
        
        self.UseGazeboSim = False
        
        self.rotation_body2world = np.zeros((3,3), dtype='float32')
        self.ex_int = 0.0
        self.e_wz_int = 0.0

        num_states = 8

        self.state = np.zeros(num_states, dtype="float32")
        self.observed_state = np.zeros(num_states, dtype="float32")

        self.estimate_error_x1 = np.zeros(4, dtype="float32")

        self.truck = robot.robot()
        self.cb = Callbacks(self)
        self.plotter = DataPlotter("results")

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
            self.fixed_dt = 1e-3 #lo rendi abbastanza basso da usare il tempo di gazebo
            self.UseGazeboSim = True

            self.timer_gazebo = self.create_timer(
                self.fixed_dt, 
                self.GazeboControl,
                callback_group=self.group
            )

            self.gz_pub_ = self.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )

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
        self.timer_observer = self.create_timer(
            self.fixed_dt, 
            self.ModelSimulator,
            callback_group=self.group
        )

        self.counter = 0
        self.t0 = self.get_clock().now()
        self.early = None

        self.time_data = []
        self.state_data = []
        self.observed_data = []
        self.sim_time = 0.0


    def GazeboControl(self):
        msg = Twist()
        # Linear velocity (m/s)
        msg.linear.x = self.des_vel_x
        msg.linear.y = 0.0
        msg.angular.z = self.des_omega_z

        self.gz_pub_.publish(msg)


    def ModelSimulator(self):
        
        # now = self.get_clock().now()
        # if self.early is None:
        #     self.early = now
        #     return
        # dt = (now - self.early).nanoseconds * 1e-9
        # self.early = now
        dt = self.fixed_dt

        if(not self.UseGazeboSim):
            u = self.PdController(dt = dt)
            Fx, Mz = u
            self.state += self.truck.dynamics(self.state, Fx, Mz, add_disturb = True)*dt
        else:
            self.state = self.cb.gazebo_state

        self.estimate_error_x1 = self.state[:4] - self.observed_state[:4]

        #self.state = self.rk4_step(self.state, Fx, Mz, dt, dist = True)

        dot_observed_state = self.truck.dynamics(self.observed_state, Fx, Mz, add_disturb = False)

        self.HighOrderObserver(dot_observed_state, dt)
 


    def HighOrderObserver(self, dot_observed_state, dt):

        p = 0.01
        alpha = 2*np.abs(dot_observed_state[:4]) + 550.0
        lambdaa = np.sqrt(2/(alpha - 2*np.abs(dot_observed_state[:4])))* ( (alpha + 2*np.abs(dot_observed_state[:4]))*(1+p))/(1-p) + 1
        delta = np.sqrt(np.abs(self.estimate_error_x1))
        correction_term = lambdaa*delta*np.tanh(self.scale_tanh*self.estimate_error_x1)

        dot_observed_state[:4] = dot_observed_state[:4] + correction_term
        dot_observed_state[4:] = dot_observed_state[4:] + alpha*np.tanh(self.scale_tanh*self.estimate_error_x1)
        
        self.observed_state += dot_observed_state*dt
        
        # Save variables for Plotting
        self.sim_time += dt
        self.time_data.append(self.sim_time)
        self.state_data.append(
            self.state.copy()
        )
        self.observed_data.append(
            self.observed_state.copy()
        )

        st = self.state
        obs = self.observed_state
        self.counter+=1
        print(self.counter)
        if self.counter % 100 == 0:
            self.get_logger().info(
                f"real (estimated)\n"
                f"Pos : x={st[s.X]:7.3f} ({obs[s.X]:7.3f})  "
                f"y={st[s.Y]:7.3f} ({obs[s.Y]:7.3f})\n"
                f"Ang : r={st[s.ROLL]:7.3f} ({obs[s.ROLL]:7.3f})  "
                f"ψ={st[s.YAW]:7.3f} ({obs[s.YAW]:7.3f})\n"
                f"Vel : vx={st[s.VX]:7.3f} ({obs[s.VX]:7.3f})  "
                f"vy={st[s.VY]:7.3f} ({obs[s.VY]:7.3f})\n"
                f"Rate: wx={st[s.WX]:7.3f} ({obs[s.WX]:7.3f})  "
                f"wz={st[s.WZ]:7.3f} ({obs[s.WZ]:7.3f})\n"
                f"dt={dt:.4f}"
            )


    def PdController(self, dt):

        if(not self.UseGazeboSim):
            roll = self.state[s.ROLL]
            pitch = 0.0
            yaw = self.state[s.YAW]

            self.rotation_body2world = R.from_euler(
                'xyz',
                [roll, pitch, yaw]
            ).as_matrix().T

        Kp = [1000, 100]
        KI = [10, 1.0]
        vx_des, _, des_omega_z = self.rotation_body2world @ np.array([self.des_vel_x, 0, self.des_omega_z])
        
        e_x = vx_des - self.state[s.VX]
        e_wz = des_omega_z - self.state[s.WZ]

        derivative_term_fx = 100*((e_x - self.previous_e[0])/dt)
        derivative_term_mz = 10*((e_wz - self.previous_e[2])/dt)

        integral_term_fx = KI[0]*self.ex_int
        integral_term_fx = KI[0]*self.e_wz_int

        Fx = Kp[0]*e_x + derivative_term_fx  
        Mz = Kp[1]*e_wz + derivative_term_mz
 
        self.previous_e[:] = [e_x, 0, e_wz]

        self.ex_int += e_x*dt
        self.e_wz_int += e_wz*dt
        
        return [Fx, Mz]
    

    def rk4_step(self, state, Fx, Mz, dt, dist):

        k1 = self.truck.dynamics(state, Fx, Mz, dist)
        k2 = self.truck.dynamics(state + 0.5*dt*k1, Fx, Mz, dist)
        k3 = self.truck.dynamics(state + 0.5*dt*k2, Fx, Mz, dist)
        k4 = self.truck.dynamics(state + dt*k3, Fx, Mz, dist)

        return state + dt/6.0 * (k1 + 2*k2 + 2*k3 + k4)



    def AtEnd(self):

        state = np.array(self.state_data)
        observed = np.array(self.observed_data)
        time = np.array(self.time_data)
        
        self.plotter.plot(
            time,
            state[:, s.VX],
            observed[:, s.VX],
            ylabel="vx [m/s]",
            filename="velocities.png"
        )

        self.plotter.plot(
            time,
            state[:, s.WZ],
            observed[:, s.WZ],
            ylabel="wz [rad/s]",
            filename="angular_vel_z.png"
        )

    # World frame doesn't exist    
    # def World2Robot(self):

    #     tf_buffer = tf2_ros.Buffer()
    #     # Subscribe to /tf and populate the tf_buffer
    #     listener = tf2_ros.TransformListener(tf_buffer, self)
        
    #     trans = tf_buffer.lookup_transform(
    #         "base_link",   # target frame (robot)
    #         "world",       # source frame
    #         rclpy.time.Time()
    #     )
        
    #     t = trans.transform.translation
    #     q = trans.transform.rotation

    #     # Rotation matrix from quaternion
    #     rotation = quaternion_matrix(
    #         [q.x, q.y, q.z, q.w]
    #     )

    #     return rotation*np.array([self.des_vel_x, 0.0, self.des_omega_z])


def main(args=None):

    rclpy.init(args=args)

    node = TruckStateObserver()
    try:
        if node.UseGazeboSim:
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(node)

            try:
                executor.spin()
            finally:
                executor.shutdown()

        else:
            rclpy.spin(node)

    finally:
        node.AtEnd()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()