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

    rospy.logwarn("Waiting for VelmaInterface initialization...")
    if not velma.waitForInit(timeout_s=10.0):
        rospy.logwarn("Could not initialize VelmaInterface\n")
        exitError(1)
    rospy.logwarn("Initialization ok!\n")

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
        rospy.logwarn("Motors must be homed and ready to use for this test.")
        exitError(5)

    rospy.logwarn("waiting for Planner init...")
    velma.maxJointTrajLen = lambda: 100 #pohuj o ?
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        rospy.logwarn("could not initialize PLanner")
        exitError(6)
    rospy.logwarn("Planner initialized")

    rospy.logwarn("Initializing Octomap")
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    if not oml:
        rospy.logwarn("Failed Initializing Octomap")
        exitError(7)
    rospy.logwarn("Octomap initialization complete")

    rospy.logwarn("Getting Octomap")
    octomap = oml.getOctomap(timeout_s=5.0)
    if not octomap:
        rospy.logwarn("Octomap loading failed")
    rospy.logwarn("Getting Octomap Succesful")
    
    rospy.logwarn("Processing octomap")
    if not p.processWorld(octomap):
        rospy.logwarn("Processing octomap Failed") 
    rospy.logwarn("Processing octomap Succesful") 

    q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
    velma.moveHandLeft(q,[10,10,10,10],[10,10,10,10],100,False)
    velma.moveHandRight(q,[10,10,10,10],[10,10,10,10],100,False)

    return velma, p


lgr_close = True
rgr_close = True
is_holding = False

def gripper_change(velma, left_gripper = True, mode = 0):
    global lgr_close
    global rgr_close
    
    if left_gripper:
        if lgr_close:
            # rospy.logwarn("Otwieram lewa lape")
            if mode == 0: #full open_close
                q = [math.radians(0), math.radians(0), math.radians(0), math.radians(0)]
            elif mode == 1: # partial
                q = [math.radians(70), math.radians(70), math.radians(70), math.radians(0)]
            elif mode == 2: # without thumb
                q = [math.radians(70), math.radians(70), math.radians(0), math.radians(0)]
            elif mode == 3: # one side push
                q = [math.radians(0), math.radians(0), math.radians(0), math.radians(180)]
            elif mode == 4: # one side push
                q = [math.radians(50), math.radians(50), math.radians(180), math.radians(0)]
            
        else:
            # rospy.logwarn("Zamykam lewa lape")
            if mode == 0:
                q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
            elif mode == 1:
                q = [math.radians(90), math.radians(90), math.radians(90), math.radians(0)]
            elif mode == 2:
                q = [math.radians(100), math.radians(100), math.radians(0), math.radians(0)]
            elif mode == 3: 
                q = [math.radians(90), math.radians(90), math.radians(90), math.radians(180)]
            elif mode == 4:
                q = [math.radians(180), math.radians(180), math.radians(180), math.radians(180)]

        lgr_close = not lgr_close
        velma.moveHandLeft(q,[2,2,2,2],[0,0,0,0],0.1,False)
    else:
        if rgr_close:
            # rospy.logwarn("Otwieram prawa lape")
            q = [math.radians(0), math.radians(0), math.radians(0), math.radians(0)]

        else:
            # rospy.logwarn("Zamykam prawa lape")
            q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
        rgr_close = not rgr_close
        velma.moveHandRight(q,[10,10,10,10],[10,10,10,10],0.1,False)


def calculateGoalPosition(velma, offset, rot_z = 0.0):
    off_x, off_y, off_z = offset
    T_W_O = velma.getTf("B", "cabinet_door_fragile")
    
    print("Calculating translation")   
    trans= PyKDL.Vector(off_x, off_y, off_z)
    T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(0), math.radians(90), math.radians(-180) + rot_z), trans)

    T_G_P=velma.getTf("Gl", "Pl")
    T_P_E=velma.getTf("Pl", "El")

    return T_W_O*T_O_G*T_G_P*T_P_E


def calculateCabinetPosition(velma, offset, rot_z = 0.0):
    off_x, off_y, off_z = offset
    T_W_O = velma.getTf("B", "cabinet_door_fragile_cabinet")
    
    print("Calculating translation")   
    trans= PyKDL.Vector(off_x, off_y, off_z)
    T_O_G = PyKDL.Frame(PyKDL.Rotation.RPY(math.radians(0), math.radians(90), math.radians(-180) + rot_z), trans)

    T_G_P=velma.getTf("Gl", "Pl")
    T_P_E=velma.getTf("Pl", "El")
    
    return T_W_O*T_O_G*T_G_P*T_P_E


def calculate_IK(velma, T_B_Goal):
    rospy.logwarn("------------ IK ------------")
    rospy.logwarn("Getting object location")

    if not T_B_Goal:
        rospy.logwarn("Failed Getting object location")
        exitError(8)
    rospy.logwarn("Got object location")
    
    solv = KinematicsSolverVelma()
    if not solv:
        rospy.logwarn("Failed to initialize IK solver")
    rospy.logwarn("Succesful initialization of IK Solver")

    valid_sol_dict = []
    rospy.logwarn("Rozwiezuje odwrtone kinematyke")
    
    flips = []
    for flip_shoulder in (True, False):
        for flip_elbow in (True, False):
            for flip_ee in (True, False):
                flips.append((flip_shoulder, flip_elbow, flip_ee))

    for flip_shoulder, flip_elbow, flip_ee in flips:
        for elbow_circle_angle in np.linspace(-math.pi, math.pi, 30):
            for torso_angle in np.linspace(-1.4, 1.4, 1):
                torso_angle = 0.0
                q = solv.calculateIkArm("left", T_B_Goal, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)
                
                if not q[0] is None:
                    q_dict = {}
                    q_dict["torso_0_joint"] = torso_angle
                    for i in range(len(q)):
                            q_dict['left_arm_{}_joint'.format(i)] = q[i]
                    valid_sol_dict.append(q_dict)
        
    if len(valid_sol_dict) != 0:
        print("Liczba rozwiazan ik: ", len(valid_sol_dict))
    
    if len(valid_sol_dict) == 0:
        rospy.logwarn("Brak Rozwiazan IK")
    
    return valid_sol_dict


def make_and_execute_plan(velma, ik_solutions, p):
    global is_holding

    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        rospy.logwarn("The action should have ended without error, but the error code is", error)
        exitError(9)

    js = velma.getLastJointState()
    
    goal_constraints = [qMapToConstraints(q, 0.01, group=velma.getJointGroup("left_arm")) for q in ik_solutions]


    print("Executing plan, one of ", str(len(goal_constraints)))

    for i in range(5):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."

        traj = p.plan(js[1], goal_constraints, "left_arm", num_planning_attempts=40, max_velocity_scaling_factor=0.15, planner_id="RTTConnect")

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
    
def makeWrench(lx,ly,lz,rx,ry,rz):
    return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))

def makePathTol(lx,ly,lz,rx,ry,rz):
    return PyKDL.Twist(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))


def switch_to_cart_imp(velma):
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


def move_cart(velma, offset, rot_z = 0.0, cabinet_goal = False, imp_list=None, path_tol = None):
    print "Moving left wrist to pose defined in world frame..."

    if not cabinet_goal:
        T_B_Goal = calculateGoalPosition(velma, offset, rot_z)
    else:
        T_B_Goal = calculateCabinetPosition(velma, offset, rot_z)
    
    imp_list = [imp_list]

    if not velma.moveCartImpLeft([T_B_Goal], [3.0], None, None, imp_list, [5.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=path_tol):
        exitError(16)
    if velma.waitForEffectorLeft() != 0:
        gripper_change(velma,True,True)
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
        rospy.logwarn("The action should have ended without error, but the error code is", error)
        exitError(18)

    js = velma.getLastJointState()
    
    goal_constraints = [qMapToConstraints(q, 0.1, group=velma.getJointGroup("left_arm")) for q in ik_solutions]
    print(len(goal_constraints))
    for i in range(15):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
       
        traj = p.plan(js[1], goal_constraints, "left_arm", num_planning_attempts=40, max_velocity_scaling_factor=0.15, planner_id="RTTConnect")
        
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


def open_cabinet(velma):

    print "Opening cabinet"

    rospy.logwarn("first part of move")
    T_B_Goal = calculateGoalPosition(velma, (0.12, 0.0, 0.075), rot_z= math.asin(0.12/0.29) + math.radians(30))
    if not velma.moveCartImpLeft([T_B_Goal], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,
                                ):
        exitError(2004)
    if velma.waitForEffectorLeft() != 0:
        gripper_change(velma,True,True)
        exitError(2005)
    # rospy.sleep(0.5)

    rospy.logwarn("poprawka kata")
    T_B_Goal = calculateGoalPosition(velma, (0.0, 0.0, 0.075), rot_z= math.asin(0.12/0.29) + math.radians(30))
    if not velma.moveCartImpLeft([T_B_Goal], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,
                                ):
        exitError(2006)
    if velma.waitForEffectorLeft() != 0:
        gripper_change(velma,True,True)
        exitError(2007)
    # rospy.sleep(3)


    rospy.logwarn("second part of move")
    T_B_Goal = calculateGoalPosition(velma, (0.12, 0.0, 0.075), rot_z= math.asin(0.12/0.29)+ math.radians(30))
    if not velma.moveCartImpLeft([T_B_Goal], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,
                                ):
        exitError(2004)
    if velma.waitForEffectorLeft() != 0:
        gripper_change(velma,True,True)
        exitError(2005)
    rospy.sleep(3)


def main():
    global lgr_close
    # starting position
    velma, p = init()

    
    rospy.logwarn("Obliczenie trajektori do klamki")
    T_B_G = calculateGoalPosition(velma, (0.1, 0.0, 0.075))
    ik_sol = calculate_IK(velma, T_B_G)

    rospy.logwarn("Jazda do szafki")
    make_and_execute_plan(velma, ik_sol, p)
    rospy.sleep(1)

    rospy.logwarn("Otworzenie chwytaka")
    gripper_change(velma,True, mode = 2)
    rospy.sleep(3)
  
    switch_to_cart_imp(velma)

    rospy.logwarn("Podejscie do klamki")
    # move_cart(velma, (0.025, 0.0, 0.075))
    move_cart(velma, (0.02, 0.0, 0.075), rot_z=0, cabinet_goal=False, imp_list=makeWrench(100,100,1000,100,100,100),path_tol=makePathTol(0.1,0.05,0.1,0.2,0.2,0.2) )
    rospy.sleep(1)

    rospy.logwarn("Zamkniecie chwytaka")
    gripper_change(velma,True, mode = 2)
    print(lgr_close)
    rospy.sleep(3)

    #kat backup rot_z=math.asin(0.1/0.29)
    rospy.logwarn("otworzenie szafki")
    move_cart(velma, (0.25, 0.0, 0.075), rot_z=math.radians(20), cabinet_goal=False, imp_list=makeWrench(100,100,1000,100,100,100),path_tol=makePathTol(0.2,0.5,0.1,0.2,0.2,0.2) )
    rospy.sleep(2)

    # lgr_close = True # mysli ze jest zamkniety
    gripper_change(velma, True, mode = 0) #otwarta
    rospy.sleep(2)

    lgr_close = True
    gripper_change(velma, True, 3) #otwarta
    rospy.sleep(2)
    
    rospy.logwarn("drobna poprawka ")
    # move_cart(velma, (0.0, 0.2, 0.075), rot_z=math.radians(30))
    move_cart(velma, (0.0, 0.2, 0.075), rot_z=math.radians(30), cabinet_goal=False, imp_list=makeWrench(100,100,1000,100,100,100),path_tol=makePathTol(0.1,0.1,0.1,0.2,0.2,0.2))
    rospy.sleep(2)

    gripper_change(velma, True, 0) #zamknieta
    rospy.sleep(3)
    
    rospy.logwarn("ustawienie manipulatora za drzwi")
    move_cart(velma, (0.3, 0.1, 0.0), rot_z=math.radians(30), cabinet_goal=True, imp_list=makeWrench(100,100,1000,100,100,100) )
    rospy.sleep(2) 
    rospy.logwarn("ustawienie chwytaka do pchania")
    gripper_change(velma, True, 4) #zamknieta
    rospy.sleep(2)
    
    rospy.logwarn("pizd ")
    move_cart(velma, (0.3, -0.2, 0.0), rot_z=math.radians(10), cabinet_goal=True, imp_list=makeWrench(200,200,1000,100,100,100) )
    rospy.sleep(2)

    move_cart(velma, (0.3, 0.0, 0.075), rot_z=math.radians(0), cabinet_goal=True, imp_list=makeWrench(200,200,1000,100,100,100) )
    gripper_change(velma,True,mode = 0)
    rospy.sleep(1)

    rospy.logwarn("Powrot do pozycji poczatkowej")
    go_to_starting_pos(velma,p)

if __name__ == '__main__':
    main()