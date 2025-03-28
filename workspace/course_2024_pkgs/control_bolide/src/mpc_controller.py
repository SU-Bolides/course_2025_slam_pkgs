#!/usr/bin/env python3

__author__ = "Nicolas HAMMJE"
__status__ = "In Developpement"

#%% IMPORTS
import rospy
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from tt02 import AckermannCar, getTrack
import math
from std_msgs.msg import Float32MultiArray
import math
from nav_msgs.msg import Odometry
from control_bolide.msg import SpeedDirection
from inputimeout import inputimeout, TimeoutOccurred
from geometry_msgs.msg import Pose, Point





class AcadosMPC:

    def __init__(self, track="su_bigmap_splined.txt", dt=1/20.):
        track_name = track.split(".txt")[0].split("_splined")[0]

        self.track = track_name + "_splined.txt"
        self.dt = dt
        self.tt02 = AckermannCar(0.257, self.track, dt)
        _,_,_,_,_,_, self.x_ref_s, self.y_ref_s = self.tt02.model
        pass

    def create_ocp_solver(self, Tf=0.75):

        N=int(Tf/self.dt)

        #state = x y theta vx vy omega thetaA delta D vtheta
        #control = delta_dot D_dot vtheta_dot

        model, cost_expr, J, cost_func, Je, lh_constraints, self.x_ref_s, self.y_ref_s = self.tt02.model

        ocp = AcadosOcp()
        ocp.model = model


        ocp.constraints.lbu = np.array([-5.,-10.,-4.]) #DeltaDot, DDot, VthetaDot
        ocp.constraints.ubu = np.array([5.,10.,40.])
        ocp.constraints.idxbu = np.array([0,1,2]) #Indices of the corresponding variables

        ocp.constraints.lbx = np.array([-10,-10,-10, -0.5, -3.0, -5.0, -1.0, -0.23, -1.0, 0.5])
        ocp.constraints.ubx = np.array([10,  10, 10,  2.0,  3.0, 5.0, 100, 0.23, 1.0, 3.0])
        ocp.constraints.idxbx = np.array(range(10))


        # ocp.constraints.lbx = np.array([-10,-10,-10,0.0,-0.5,-1.0,-10])
        # ocp.constraints.ubx = np.array([10, 10, 10, 100, 0.5, 1.0, 10 ])
        # ocp.constraints.idxbx = np.array([3,4,5,6,7,8,9])


        # ocp.constraints.lbx = np.array([-0.5])
        # ocp.constraints.ubx = np.array([0.5])
        # ocp.constraints.idxbx = np.array([7])



        ocp.constraints.lsbx = np.ones(2)*0.1
        ocp.constraints.usbx = np.ones(2)*0.1
        ocp.constraints.idxsbx = np.array([3,9])


        # #Respect track width
        ocp.model.con_h_expr = lh_constraints
        # ocp.model.con_h_expr_e = lh_constraints


        # ocp.constraints.lh = np.array([-0.6, -60, -1e3])
        # ocp.constraints.uh = np.array([0.6, 60, 1e3])

        # ocp.constraints.lsh = np.array([-0.1, -10., -10.])
        # ocp.constraints.ush = np.array([0.1, 10., 10.])
        # ocp.constraints.idxsh = np.array(range(3))


        # ocp.cost.zl = 100 * np.ones((3))
        # ocp.cost.zu = 100 * np.ones((3))
        # ocp.cost.Zl = 0.1 * np.ones((3))
        # ocp.cost.Zu = 0.1 * np.ones((3))


        ocp.constraints.lh = np.array([-0.25,-0.7, 0.2])
        ocp.constraints.uh = np.array([0.25, 0.7, 1000])

        # ocp.constraints.lh_e = np.array([-0.45, -2])
        # ocp.constraints.uh_e = np.array([0.45, 2])

        ocp.constraints.lsh = np.array([-0.4, -0.5, -0.1])
        ocp.constraints.ush = np.array([0.4, 0.5, 0])
        ocp.constraints.idxsh = np.array(range(3))

        # ocp.constraints.lsh_e = np.array([-0.05, -0.05])
        # ocp.constraints.ush_e = np.array([0.05, 0.05])
        # ocp.constraints.idxsh_e = np.array(range(2))


        ocp.cost.zl = np.array([2,2,40,2,2])
        ocp.cost.zu = np.array([2,2,40,2,2])
        ocp.cost.Zl = np.array([0.2,0.2,4,0.2,0.2])
        ocp.cost.Zu = np.array([0.2,0.2,4,0.2,0.2])

        # ocp.cost.zl_e = 20 * np.ones((2))
        # ocp.cost.zu_e = 20 * np.ones((2))
        # ocp.cost.Zl_e = 0.2 * np.ones((2))
        # ocp.cost.Zu_e = 0.2 * np.ones((2))

        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"

        ocp.model.cost_expr_ext_cost = cost_expr # Control rate weighing
        ocp.model.cost_expr_ext_cost_e = Je # Contouring cost

        ocp.constraints.x0 = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])


        #OBSTACLE POSITION
        ocp.parameter_values = np.array([-10, -10])
        

        # set QP solver and integration
        ocp.solver_options.tf = Tf
        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'

        ocp.solver_options.N_horizon = N # N = Tf * 12 because we want 12 Hz

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.hessian_approx = "EXACT"
        ocp.solver_options.regularize_method = "CONVEXIFY"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 3
        ocp.solver_options.nlp_solver_max_iter = 50
        ocp.solver_options.levenberg_marquardt = 1.0
        ocp.solver_options.qp_warm_start = 2 # Enable warm-starting
        ocp.solver_options.tol = 1e-4
        # ocp.solver_options.nlp_solver_tol_comp = 1e-1
        
        # create solver
        acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
        return acados_solver

class MPCController:

    def __init__(self):
        rospy.init_node("mpc_controller_node")

        self.WAYPOINTS_PATH = str(rospy.get_param("~centerline_path", "~/bolide_ws/course_2024_pkgs/control_bolide/racelines/saintcyr_centerline.txt"))
        self.ODOM_TOPIC = str(rospy.get_param("~odom_topic", "/pf/pos/odom"))
        self.CMD_TOPIC = str(rospy.get_param("~cmd_topic", "cmd_vel"))


        self.started = False

        # init publisher
        self.drive_pub = rospy.Publisher(self.CMD_TOPIC, SpeedDirection, queue_size=10)
        self.diag_pub = rospy.Publisher("/stanley_diagnostics", Float32MultiArray, queue_size=1)

        dt = 1/12.

        self.acados_ocp = AcadosMPC(dt = dt, track="saintcyr_centerline.txt")
        self.solver = self.acados_ocp.create_ocp_solver()

        self.state = np.array([self.acados_ocp.tt02.start_pos[0], self.acados_ocp.tt02.start_pos[1], self.acados_ocp.tt02.start_pos[2]+0.1, 0.1, 0., 0., 0.0, 0., 0., 0.2])

        self.u0 = np.array([0.,0.,0.])

        # self.state, _, _ = self.acados_ocp.tt02.integration(self.state,self.u0)

        self.stm32_sub = rospy.Subscriber("stm32_sensors", Float32MultiArray, self.stmCB, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odomCB, queue_size=10)
        self.car_state_sub = rospy.Subscriber("car_state", SpeedDirection, self.car_stateCB, queue_size=1) # Publish the car's current state (velocity and steering angle) for odometry. 

        self.delta = None
        self.vx = None

        self.update = rospy.Timer(rospy.Duration(0.08), self.solveMPC)


    def quaternion_to_yaw(self, q_w, q_x, q_y, q_z):
        yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
        return yaw  # In radians

    def odomCB(self, msg:Odometry):
        #12 Hz (pf/pos/odom)
        #state = x y theta vx vy omega thetaA delta D vtheta

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y     
        
        theta = self.quaternion_to_yaw(msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z)
        
        # vx = msg.twist.twist.linear.x
        # vy = msg.twist.twist.linear.y
        # omega = msg.twist.twist.linear.z 

        self.state = np.array([x, y, theta, self.vx, 0, self.omega, self.state[6], self.delta, self.state[8], self.state[9]])

        
    def solveMPC(self,_):

        print(self.state)
        #set lower and upper bounds for initial state
        self.solver.set(0, 'lbx', self.state)
        self.solver.set(0, 'ubx', self.state)

        status = self.solver.solve()

        if status != 4:
            self.state = self.solver.get(1, "x")
            self.state2 = self.solver.get(2, "x")
        else:
            rospy.logwarn(f"ACADOS RETURNED STATUS {status}")
            self.state = self.state2
            self.state2 = self.state2

        self.apply_control()

    def stmCB(self, data:Float32MultiArray):
        #self.sensor_data.data = [self.yaw, self.speed, self.ir_gauche, self.ir_droit, self.distance_US, self.acc_x, self.yaw_rate]

        self.vx = data.data[1]
        self.omega = data.data[6]


    def apply_control(self):
        #Apply control

        msg = SpeedDirection()


        duty_cycle = self.state[8]
        delta = -self.state[7]


        print("\n Delta, D: ", delta, duty_cycle)

        msg.speed = max(min(duty_cycle, 0.05), -0.05)
        msg.direction = max(min(delta/0.15, 1.0), -1.0) #(rad/max_rad)

        self.drive_pub.publish(msg)


    def destroy_node(self):
        kill = SpeedDirection()
        kill.speed = 0
        kill.direction = 0
        self.drive_pub.publish(kill)


    def car_stateCB(self, msg:SpeedDirection):
        #65 Hz      
        self.delta = math.radians(msg.direction)





    def publish_cmd_vel(self, velocity, angle):
        if not rospy.is_shutdown():
            drive_msg = SpeedDirection()


            ## If the speed difference is bigger than the brake_threshold, we want to apply the brakes
            # rospy.logwarn((self.target_velocity - self.old_target_vel))
            if ((self.target_velocity - self.old_target_vel)) < -self.BRAKE_THRESHOLD_MS:
                rospy.logwarn("BRAKING BRAKING BRAKING BRAKING BRAKING")
                velocity = 2.0
                

            #Max Speed is 7 m/s
            if not velocity == 2.0:
                velocity_normalized = velocity / 10.0
                velocity_normalized = min(max(velocity_normalized, -1), 1.0)
            else:
                velocity_normalized = 2.0

            drive_msg.speed = velocity_normalized

            self.commanded_velocity = velocity
        
            angle_normalized = angle / np.radians(self.STEERING_LIMIT_DEG)

            drive_msg.direction = -angle_normalized
            self.drive_pub.publish(drive_msg)





#%% MAIN
if __name__ == '__main__':
    controller = MPCController()
    print("MPC Controller Initialized")

    try: 
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received. Shutting down...")
    finally:
        # Clean up and unregister keyboard events
        controller.destroy_node()