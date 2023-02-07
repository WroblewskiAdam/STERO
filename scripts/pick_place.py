#!/usr/bin/env python
import rospy
import math
import time
import numpy as np
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
from rcprg_ros_utils import exitError
from velma_common import *
from rcprg_planner.rcprg_planner import *
from rcprg_ros_utils.scripting import exitError
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma, KinematicsSolverLWR4
from velma_common.velma_interface import VelmaInterface, isConfigurationClose
from moveit_msgs.msg import PositionConstraint, Constraints, OrientationConstraint, BoundingVolume, JointConstraint 
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive


def init():
    rospy.init_node("velma_node")
    velma = VelmaInterface() 

    rospy.loginfo("Waiting for VelmaInterface initialization...")
    if not velma.waitForInit(timeout_s=10.0):
        rospy.loginfo("Could not initialize VelmaInterface\n")
        exitError(1)
    rospy.loginfo("Initialization ok!\n")

    if velma.enableMotors() != 0:
        exitError(2)
    print("Sending head pan motor START_HOMING command...")
    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(3)
    print("Head pan motor homing successful.")

    print("Sending head tilt motor START_HOMING command...")
    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(4)
    print("Head tilt motor homing successful.")

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        rospy.loginfo("Motors must be homed and ready to use for this test.")
        exitError(5)

    rospy.loginfo("waiting for Planner init...")
    velma.maxJointTrajLen = lambda: 100 #pohuj o ?
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        rospy.loginfo("could not initialize PLanner")
        exitError(6)
    rospy.loginfo("Planner initialized")

    rospy.loginfo("Initializing Octomap")
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    if not oml:
        rospy.loginfo("Failed Initializing Octomap")
        exitError(7)
    rospy.loginfo("Octomap initialization complete")

    rospy.loginfo("Getting Octomap")
    octomap = oml.getOctomap(timeout_s=5.0)
    if not octomap:
        rospy.loginfo("Octomap loading failed")
    rospy.loginfo("Getting Octomap Succesful")
    
    rospy.loginfo("Processing octomap")
    if not p.processWorld(octomap):
        rospy.loginfo("Processing octomap Failed") 
    rospy.loginfo("Processing octomap Succesful") 

    q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
    velma.moveHandLeft(q,[10,10,10,10],[10,10,10,10],100,False)
    velma.moveHandRight(q,[10,10,10,10],[10,10,10,10],100,False)

    return velma, p


lgr_close = True
rgr_close = True
is_holding = False
# which_table = Enum('Table', ['Left', 'Right'])

def gripper_change(velma, left_gripper = True):
    global lgr_close
    global rgr_close
    
    if left_gripper:
        if lgr_close:
            # rospy.loginfo("Otwieram lewa lape")
            q = [math.radians(0), math.radians(0), math.radians(0), math.radians(0)]
            # if velma.waitForHandLeft() != 0:
            #     exitError(2)
        else:
            # rospy.loginfo("Zamykam lewa lape")
            q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
            # if velma.waitForHandLeft() != 0:
            #     exitError(2)
        lgr_close = not lgr_close
        velma.moveHandLeft(q,[10,10,10,10],[0,0,0,0],0.1,False)
    else:
        if rgr_close:
            # rospy.loginfo("Otwieram prawa lape")
            q = [math.radians(0), math.radians(0), math.radians(0), math.radians(0)]
            # if velma.waitForHandRight() != 0:
            #     exitError(4)
        else:
            # rospy.loginfo("Zamykam prawa lape")
            q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
            # if velma.waitForHandRight() != 0:
            #     exitError(4)
        rgr_close = not rgr_close
        velma.moveHandRight(q,[10,10,10,10],[10,10,10,10],0.1,False)


def calculateGoalPosition(velma, offset):
    T_W_O = velma.getTf("B", "object1") 
    rot_M = T_W_O.M
    vect_p = T_W_O.p
    angles = rot_M.GetRPY()
    rot_M.DoRotX(-angles[0])
    rot_M.DoRotY(-angles[1])
    rot_M.DoRotZ(-angles[2])
    rot_M.DoRotZ(angles[2]% 1.56)
    velma.moveHead((math.atan(T_W_O.p[1]/T_W_O.p[0]) , 0.15), 1, start_time=0.5)

    T_W_O= PyKDL.Frame(rot_M, vect_p)

    print("Calculating translation")   
    trans= PyKDL.Vector(0, 0, offset)
    T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0)), trans)
    
    T_G_P=velma.getTf("Gl", "Pl")
    T_P_E=velma.getTf("Pl", "El")

    return T_W_O*T_O_G*T_G_P*T_P_E

def calculateTablePosition(velma, offset, right_table=True):


    if right_table:
        T_W_B = velma.getTf("B", "table1")
        # trans= PyKDL.Vector(T_W_B.p[0] - 0.15, T_W_B.p[1] + 0.15, T_W_B.p[2] + 0.62)
    else:
        T_W_B = velma.getTf("B", "table2")

    r = 0.3
    x_table_cent = T_W_B.p[0]
    y_table_cent = T_W_B.p[1]

    landing_x = -x_table_cent/sqrt(x_table_cent**2 + y_table_cent**2)*r + x_table_cent
    landing_y = -y_table_cent/sqrt(x_table_cent**2 + y_table_cent**2)*r + y_table_cent

    trans= PyKDL.Vector(landing_x, landing_y, T_W_B.p[2]*2 + 0.12)
    T_W_O = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(-180), math.radians(0), math.radians(0)), trans)

    velma.moveHead((math.atan(T_W_B.p[1]/T_W_B.p[0]) , 0.15), 1, start_time=0.5)


    T_O_P=velma.getTf("Gl", "Pl")
    T_P_E=velma.getTf("Pl", "El")

    return T_W_O*T_O_P*T_P_E

def is_object_on_right_table(velma):
    T_W_O = velma.getTf("B", "object1")
    
    # print("\n")
    # print(T_W_O)
    # print("\n")
    # print(T_W_O.p[1])

    if T_W_O.p[1] > 0.1:
        rospy.loginfo("Zlokalizowalem obiekt na stoliku lewym")
        rospy.sleep(1.0)
        return False
    else:
        rospy.loginfo("Zlokalizowalem obiekt na stoliku prawym")
        rospy.sleep(1.0)
        return True




def calculate_IK(velma, T_B_Goal):
    rospy.loginfo("------------ IK ------------")
    rospy.loginfo("Getting object location")

    if not T_B_Goal:
        rospy.loginfo("Failed Getting object location")
        exitError(8)
    rospy.loginfo("Got object location")
    
    solv = KinematicsSolverVelma()
    if not solv:
        rospy.loginfo("Failed to initialize IK solver")
    rospy.loginfo("Succesful initialization of IK Solver")

    valid_sol_dict = []
    rospy.loginfo("Rozwiezuje odwrtone kinematyke")
    
    for elbow_circle_angle in np.linspace(-math.pi, math.pi, 30):
        for torso_angle in np.linspace(-1.4, 1.4, 30):
            q = solv.calculateIkArm("left", T_B_Goal, torso_angle, elbow_circle_angle, True, True, True)
            
            if not q[0] is None:
                q_dict = {}
                q_dict["torso_0_joint"] = torso_angle
                for i in range(len(q)):
                        q_dict['left_arm_{}_joint'.format(i)] = q[i]
                valid_sol_dict.append(q_dict)
    if len(valid_sol_dict) != 0:
        print("Liczba rozwiazan ik: ", len(valid_sol_dict))
    
    if len(valid_sol_dict) == 0:
        rospy.loginfo("Brak Rozwiazan IK")
    
    return valid_sol_dict


def get_object_in_hand():
    object1 = AttachedCollisionObject()
    object1.link_name = "left_HandGripLink"
    object1.object.header.frame_id = "left_HandGripLink"
    object1.object.id = "object1"
    object1_prim = SolidPrimitive()
    object1_prim.type = SolidPrimitive.BOX
    object1_prim.dimensions=[None, None]    # set initial size of the list to 2
    object1_prim.dimensions[SolidPrimitive.BOX_X] = 0.1
    object1_prim.dimensions[SolidPrimitive.BOX_Y] = 0.1
    object1_prim.dimensions[SolidPrimitive.BOX_Z] = 0.1
    object1_pose = pm.toMsg(PyKDL.Frame(PyKDL.Rotation.RotY(math.pi/2)))
    object1.object.primitives.append(object1_prim)
    object1.object.primitive_poses.append(object1_pose)
    object1.object.operation = CollisionObject.ADD
    object1.touch_links = ['left_HandPalmLink',
        'left_HandFingerOneKnuckleOneLink',
        'left_HandFingerOneKnuckleTwoLink',
        'left_HandFingerOneKnuckleThreeLink',
        'left_HandFingerTwoKnuckleOneLink',
        'left_HandFingerTwoKnuckleTwoLink',
        'left_HandFingerTwoKnuckleThreeLink',
        'left_HandFingerThreeKnuckleTwoLink',
        'left_HandFingerThreeKnuckleThreeLink']
    return object1


def make_and_execute_plan(velma, ik_solutions, p):
    global is_holding
    if is_holding:
        object1 = get_object_in_hand()

    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        rospy.loginfo("The action should have ended without error, but the error code is", error)
        exitError(9)

    js = velma.getLastJointState()
    
    goal_constraints = [qMapToConstraints(q, 0.01, group=velma.getJointGroup("left_arm_torso")) for q in ik_solutions]

    print("Executing plan, one of ", str(len(goal_constraints)))

    for i in range(5):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."

        if is_holding:
            traj = p.plan(js[1], goal_constraints, "left_arm_torso", num_planning_attempts=40, max_velocity_scaling_factor=0.15, planner_id="RTTConnect", attached_collision_objects=[object1])
        else:
            traj = p.plan(js[1], goal_constraints, "left_arm_torso", num_planning_attempts=40, max_velocity_scaling_factor=0.15, planner_id="RTTConnect")
        
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(10)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue
    

def move_cart(velma, offset):
    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(11)
    if velma.waitForEffectorRight() != 0:
        exitError(12)

    rospy.sleep(0.5)

    print "Reset tools for both arms..."
    T_B_Wr = velma.getTf("B", "Wr")
    T_B_Wl = velma.getTf("B", "Wl")
    if not velma.moveCartImpRight([T_B_Wr], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if not velma.moveCartImpLeft([T_B_Wl], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(14)
    if velma.waitForEffectorLeft() != 0:
        exitError(15)

    print "Moving right wrist to pose defined in world frame..."

    T_B_Goal = calculateGoalPosition(velma, offset)
    
    if not velma.moveCartImpLeft([T_B_Goal], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(16)
    if velma.waitForEffectorLeft() != 0:
        exitError(17)
    rospy.sleep(0.5)

def go_to_starting_pos(velma,p):

    q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
        'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

    ik_solutions = []
    for i in range(100):
        ik_solutions.append(q_map_starting)

    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        rospy.loginfo("The action should have ended without error, but the error code is", error)
        exitError(18)

    js = velma.getLastJointState()
    
    goal_constraints = [qMapToConstraints(q, 0.1, group=velma.getJointGroup("impedance_joints")) for q in ik_solutions]
    print(len(goal_constraints))
    for i in range(15):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        if is_holding:
            traj = p.plan(js[1], goal_constraints, "impedance_joints", num_planning_attempts=40, max_velocity_scaling_factor=0.15, planner_id="RTTConnect", attached_collision_objects=[object1])
        else:
            traj = p.plan(js[1], goal_constraints, "impedance_joints", num_planning_attempts=40, max_velocity_scaling_factor=0.15, planner_id="RTTConnect")
        
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(19)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue

def main():
    # starting position
    velma, p = init()
    
    is_right_table = is_object_on_right_table(velma)
    
    rospy.loginfo("Obliczenie trajektori do klocka")
    T_B_G = calculateGoalPosition(velma, 0.2)
    ik_sol = calculate_IK(velma, T_B_G)

    rospy.loginfo("Jazda po klocek")
    make_and_execute_plan(velma, ik_sol, p)
    rospy.sleep(1)

    rospy.loginfo("Otworzenie chwytaka")
    gripper_change(velma,True)
    
    rospy.loginfo("Podejscie do klocka")
    move_cart(velma, 0.05)
    rospy.sleep(1)

    rospy.loginfo("Chwycenie klocka")
    gripper_change(velma,True)
    is_holding = True
    rospy.sleep(1)


    rospy.loginfo("Uniesienie klocka")
    move_cart(velma, 0.3)
    
    rospy.loginfo("Obliczenie trajektori nad 2-gi stolik")
    T_B_G = calculateTablePosition(velma, 0.25, not is_right_table)
    ik_sol = calculate_IK(velma, T_B_G)
    
    
    rospy.loginfo("Jazda nad 2-gi stolik")
    make_and_execute_plan(velma, ik_sol, p)
    rospy.sleep(1)


    rospy.loginfo("Otworzenie chwytaka")
    gripper_change(velma,True)
    is_holding = False
    rospy.sleep(1)

    rospy.loginfo("Odejscie od klocka")
    move_cart(velma, 0.3)
    
    rospy.loginfo("Zamkniecie chwytaka")
    rospy.sleep(1)
    gripper_change(velma,True)

    rospy.loginfo("Powrot do pozycji poczatkowej")
    go_to_starting_pos(velma,p)

if __name__ == '__main__':
    main()
