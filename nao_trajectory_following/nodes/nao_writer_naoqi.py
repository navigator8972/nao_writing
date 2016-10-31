#!/usr/bin/env python
"""
Listens for a trajectory to write and sends it to the nao via naoqi SDK.

Requires a running robot/simulation with ALNetwork proxies.

"""
import threading
import numpy as np

from naoqi import ALModule, ALBroker, ALProxy
import motion

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
#<hyin/Mar-31st-2016> use customized message type for multi stroke movements
from nao_writing_msgs.msg import MultiPaths
import rospy
import tf
from copy import deepcopy
from naoqi import ALProxy

# masks for which axes naoqi is to control with its planning
AXIS_MASK_X = 1
AXIS_MASK_Y = 2
AXIS_MASK_Z = 4
AXIS_MASK_WX = 8
AXIS_MASK_WY = 16
AXIS_MASK_WZ = 32

NAO_WHOLEBODY = False

def max_min_range(data):

    d = 0
    x_ma = []
    y_mi = []
    x_mi = []
    y_ma = []
    for stroke in data:
        #print stroke
        x_max = x_ma.append(int(round(max(stroke[ :, 0]) + d)))
        y_max = y_ma.append(int(round(max(stroke[ :, 1]) + d)))
        x_min = x_mi.append(int(round(min(stroke[ :, 0]) + d)))
        y_min = y_mi.append(int(round(min(stroke[ :, 1]) + d)))
    x_max = max(x_ma)
    y_max = max(y_ma)
    x_min = min(x_mi)
    y_min = min(y_mi)

    return x_max, y_max, x_min, y_min

def centering_data(data):
    tol_n_pnts = 0
    coord_integral = np.zeros(2)
    for strk in data:
        tol_n_pnts += len(strk)
        coord_integral = coord_integral + np.sum(strk, axis=0)

    center = coord_integral / tol_n_pnts
    centered_data = [strk - center for strk in data]
    return centered_data

def autofit_letter(data):
    """
    helper to fit the letter strokes in proper workspace of NAO
    """
    centered_data = centering_data(data)
    x_max, y_max, x_min, y_min = max_min_range(data)
    scale = 0.05 / np.abs(max([x_max - x_min, y_max - y_min]))

    fit_data = [strk * scale for strk in centered_data]
    return fit_data

def MultiPath_to_ndarray(msg):
    #extract a list of ndarray for subsequent processing convenience
    data = [np.array([[ps.pose.position.x, ps.pose.position.y] for ps in path.poses]) for path in msg.paths]
    return data

def autofit_msg(msg):
    data = MultiPath_to_ndarray(msg)
    fit_data = autofit_letter(data)
    # print fit_data
    fit_msg = deepcopy(msg)
    for path, strk in zip(fit_msg.paths, fit_data):
        for ps, d in zip(path.poses, strk):
            ps.pose.position.x = d[0]
            ps.pose.position.y = d[1]
    return fit_msg

def execute_motion_handler(strk_idx, effector,space,paths,axisMask,times,isAbsolute):
    if strk_idx >= len(paths):
        #done with the motion
        rospy.loginfo("finish the motion...")
        return
    rospy.loginfo("spawn a motion...")

    motionProxy.positionInterpolation(effector,space,paths[strk_idx],axisMask,times[strk_idx],isAbsolute);

    #spawn another thread for the next stroke
    t = threading.Thread(target=execute_motion_handler, args=(strk_idx+1, effector,space,paths,axisMask,times,isAbsolute))
    t.start()
    return

def on_traj(traj_lst):
    rospy.loginfo("got traj at "+str(rospy.Time.now()))
    if(hasFallen == False): #no harm in executing trajectory
        # if(effector == "LArm"):
        #     motionProxy.openHand("LHand");
        #     roll = -1.7; #rotate wrist to the left (about the x axis, w.r.t. robot frame)
        # else:
        #     motionProxy.openHand("RHand");
        #     roll = 1.7; #rotate wrist to the right (about the x axis, w.r.t. robot frame)

        roll = 1.7

        target = PoseStamped()

        target_frame = traj_lst.header.frame_id
        target.header.frame_id = target_frame

        '''
        #go to first point then wait
        path = []; times = [];
        trajStartPosition = traj.poses[0].pose.position;
        traj.poses[0].pose.position.z = 0.05
        target.pose.position = deepcopy(traj.poses[0].pose.position)
        target.pose.orientation = deepcopy(traj.poses[0].pose.orientation)
        trajStartPosition_robot = tl.transformPose("base_footprint",target)
        point = [trajStartPosition_robot.pose.position.x,trajStartPosition_robot.pose.position.y,trajStartPosition_robot.pose.position.z,roll,0,0];

        path.append(point);
        timeToStartPosition = traj.poses[0].header.stamp.to_sec();
        times.append(timeToStartPosition);
        motionProxy.setPosition(effector,space,point,0.5,axisMask);#,times,isAbsolute);
        '''
        #refrain the motion into a fixed range to prevent large scale movement...
        rospy.loginfo("start to fit trajectories.")
        fit_traj_lst = autofit_msg(traj_lst)
        rospy.loginfo("finish fitting trajectories")
        path_lst = []; time_lst = [];
        #<hyin/Mar-31st-2016> now only consider the first stroke...
        #<hyin/Apr-14th-2016> extend to multiple strokes...
        for traj in fit_traj_lst.paths:
            path = []; times = []
            for trajp in traj.poses:

                #<hyin/Apr-12th-2016> need to consider offset for both wholebody and bust
                # trajp.pose.position.z = 0.05

                target.pose.position = deepcopy(trajp.pose.position)
                target.pose.orientation = deepcopy(trajp.pose.orientation)

                if NAO_WHOLEBODY:
                    target_robot = tl.transformPose("base_footprint",target)
                    # z_offset = 0.27
                else:
                    target_robot = tl.transformPose("base_torso", target)
                    #offset along the vertical direction for torso, note in this case the origin will be the torso
                    # z_offset = 0.0

                point = [target_robot.pose.position.x,target_robot.pose.position.y,target_robot.pose.position.z,roll,0,0]#roll,pitch,yaw];
                # point = [0+x_offset,-target.pose.position.x+y_offset,target.pose.position.y+z_offset,roll,0,0]#roll,pitch,yaw];
                path.append(point);
                times.append(trajp.header.stamp.to_sec() )#- timeToStartPosition);
            path_lst.append(path)
            time_lst.append(times)

        #wait until time instructed to start executing
        #rospy.sleep(fit_traj_lst.header.stamp-rospy.Time.now())#+rospy.Duration(timeToStartPosition));
        #rospy.loginfo("executing rest of traj at "+str(rospy.Time.now())) ;
        startTime = rospy.Time.now();
        strk_idx = 0

        # print path
        # print times
        # print len(path), len(times)
        # <hyin/Apr-13th-2016> I really doubt if it is the right design to put a block call here...
        # I think it is preferred to spawn a thread to carry this call
        # motionProxy.positionInterpolation(effector,space,path,axisMask,times,isAbsolute);
        t = threading.Thread(target=execute_motion_handler, args=(strk_idx, effector,space,path_lst,axisMask,time_lst,isAbsolute))
        t.start()
        # print path

        #rospy.loginfo("Time taken for rest of trajectory: "+str((rospy.Time.now()-startTime).to_sec()));

    else:
        rospy.loginfo("Got traj but not allowed to execute it because I've fallen");

    return

class FallResponder(ALModule):
  """ Module to react to robotHasFallen events """

  def __init__(self, name, motionProxy, memoryProxy):
      ALModule.__init__(self, name)
      self.motionProxy = motionProxy;
      memoryProxy.subscribeToEvent("robotHasFallen",name,self.has_fallen.__name__);
      rospy.loginfo("Subscribed to robotHasFallen event");
  def has_fallen(self, *_args):
      global hasFallen
      hasFallen = True;
      self.motionProxy.killAll();
      rospy.loginfo("Stopped task");

def StiffnessSwitch(proxy, on=0):
    #We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = on#1.0 0 for tiffne off
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

if __name__ == "__main__":
    rospy.init_node("nao_writer");

    TRAJ_TOPIC = rospy.get_param('~trajectory_input_topic','/nao_writing/nao_character')
    NAO_IP = rospy.get_param('~nao_ip','127.0.0.1'); #default behaviour is
                                        #to connect to simulator locally
    NAO_HANDEDNESS = rospy.get_param('~nao_handedness','right')
    # <hyin/Apr-12th-2016> try to support NAO bust
    NAO_WHOLEBODY = rospy.get_param('~nao_wholebody', False)

    if(NAO_HANDEDNESS.lower()=='right'):
        effector   = "RArm"
    elif(NAO_HANDEDNESS.lower()=='left'):
        effector = "LArm"
    else:
        rospy.logerr('error in handedness param')


    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    port = 9559;
    myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
        "0.0.0.0",   # listen to anyone
        0,           # find a free port and use it
        NAO_IP,      # parent broker IP
        port)        # parent broker port
    hasFallen = False;
    motionProxy = ALProxy("ALMotion", NAO_IP, port);
    memoryProxy = ALProxy("ALMemory", NAO_IP, port);
    postureProxy = ALProxy("ALRobotPosture", NAO_IP, port)
    fallResponder = FallResponder("fallResponder",motionProxy,memoryProxy);
    pub_traj = rospy.Subscriber(TRAJ_TOPIC, MultiPaths, on_traj)

    if NAO_WHOLEBODY:
        motionProxy.wbEnableEffectorControl(effector,False); #if robot has fallen it will have a hard time getting up if the effector is still trying to be kept in a particular position
        postureProxy.goToPosture("StandInit", 0.2)

        motionProxy.wbEnableEffectorControl(effector,True);

        space = motion.FRAME_ROBOT  #FRAME_ROBOT
    else:
        #just stiff on motors if NAO is not wholebody
        StiffnessSwitch(motionProxy, on=1)

        space = motion.FRAME_TORSO     #FRAME_TORSO


    tl = tf.TransformListener()
    rospy.sleep(2)

    axisMask   = AXIS_MASK_X+AXIS_MASK_Y+AXIS_MASK_Z#+AXIS_MASK_WX#+AXIS_MASK_WY#+AXIS_MASK_WY#control all the effector's axes 7 almath.AXIS_MASK_VEL    # just control position
    isAbsolute = True

    handPos = motionProxy.getPosition("RHand",space,False)
    print handPos

    rospy.spin()

    #release the stiffness
    StiffnessSwitch(motionProxy, on=0)
    myBroker.shutdown()
