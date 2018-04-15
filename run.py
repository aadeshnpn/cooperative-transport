import cozmo
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from cozmo.util import degrees, radians
import asyncio
from controller import (PIctrl, to_nppose, calc_wheel_velo, calc_vw, 
    get_direction, plan_virtualpoint, visualize_path, 
    add_obj_path, featureTracking)

from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

#np.array(image.raw_image.convert("L"))
kMinNumFeature = 20
goal = None
transport = None

## Function related to CV
def custom_objects(robot):
    # Defined the geometical attribute of the goal object
    goal_obj = robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                              CustomObjectMarkers.Triangles5,
                                              44,
                                              30, 30, True)

    box_obj = robot.world.define_custom_box(CustomObjectTypes.CustomType03,
                                              CustomObjectMarkers.Hexagons5,
                                              CustomObjectMarkers.Diamonds5,                                           
                                              CustomObjectMarkers.Hexagons4,                                              
                                              CustomObjectMarkers.Hexagons3,
                                              CustomObjectMarkers.Circles5,                                              
                                              CustomObjectMarkers.Triangles4,
                                              125, 160,105,
                                              100,100, True) 
    """                                              
    # Defined the geometrical attribute of the object of interest (oject to transport)
    wall_obj = robot.world.define_custom_wall(CustomObjectTypes.CustomType02,
                                              CustomObjectMarkers.Hexagons5,
                                              150, 120,
                                              70, 70, True)
    """
    if (goal_obj is not None) and (box_obj is not None):
        print ("Goal obect and object to transport defined properly")


def lookAroundBehavior(robot):
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    custom_objects(robot)
    cube = None
    try:
        # This is for cube
        #cube = robot.world.wait_for_observed_light_cube(timeout=30)

        # This for for custom object
        world_objects = robot.world.wait_until_observe_num_objects(2, object_type=CustomObject, timeout=30, include_existing=False)
        global goal, transport
        for item in world_objects:
            area = item.x_size_mm * item.y_size_mm * item.z_size_mm #180000
            if area == 2100000:
                transport = item
            else:
                goal = item

    except asyncio.TimeoutError:
        print ("Didn't find interesting  object :(")
    finally:
        print ("stopping behavior")
        look_around.stop()

    look_around.stop()
    #return goal #, transport
    return transport, goal
    #return cube

def gotoangle(robot, angle, path=None):
    ctrl = PIctrl()
    while True:
        rangle = robot.pose.rotation.angle_z.radians            
        robot_pos = to_nppose(robot.pose.position.x_y_z, rangle)           

        img = add_obj_path(path, robot_pos)
        cv2.imshow('path', img)#[:,:,::-1])
        ctrl.pos_update(robot_pos)
        val = ctrl.update_theta(angle)
        vr, vl = calc_wheel_velo(val)
        robot.drive_wheels(vl, vr)
        print ('angle error', val[2])
        if np.linalg.norm(val[2]) < 0.1:
            break
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break            

def gotobehavior(robot, loc, path=None, th=35, tracking=False):
    ctrl = PIctrl()
    #Set the reference point i.e object position
    ctrl.ref_point = loc
    #robot_pos, rangle = get_pose(robot)
    i = 0
    t0 = time.time() 
    runKLT = False   
    detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
    while True:
        i += 1
        ## Calc robot current pose
        rangle = robot.pose.rotation.angle_z.radians            
        robot_pos = to_nppose(robot.pose.position.x_y_z, rangle)   
        
        img = add_obj_path(path, robot_pos)

        ctrl.pos_update(robot_pos)        
        ctrl.robot = robot
        ## Update the refrence object for the controller
        ctrl.ref_point = loc
        ## Run the PID controller
        t1 = time.time()
        dt = t1 - t0        
        val = ctrl.update(dt)
        t0 = t1
        ## Get get while velocityies from linear and angular velocity
        vr, vl = calc_wheel_velo(val)
        #print (i, robot_pos, loc)
        ## Send the velocity commands to the robot
        robot.drive_wheels(vl, vr)
        #print('error', val[2])
        #img = img[::-1,:,:]
        cv2.imshow('path', img)#[:,:,::-1])
        if tracking:
            object_seen = robot.world.visible_object_count()
            if object_seen > 0 and runKLT == False:
                old_image = robot.world.latest_image
                old_image = np.array(old_image.raw_image.convert("L"))
                p0 = detector.detect(old_image)
                p0 = np.array([x.pt for x in p0], dtype=np.float32)
                runKLT = True
            if runKLT == True:
                new_image = robot.world.latest_image
                new_image = np.array(new_image.raw_image.convert("L"))
                p0,p1,vel = featureTracking(old_image, new_image, p0)
                p0 = p1
                old_image = new_image
                print (i, 'KLT vel', vel)
            #image = robot.world.latest_image
            #gray = np.array(image.raw_image.convert("L"))
            #rvec, tvec, corners = get_rt(gray)
            #gray = aruco.drawDetectedMarkers(gray, corners)
            #cv2.imshow('tracking', gray)
        err = robot_pos - loc
        if np.linalg.norm(err[:2]) < th:
            break
        #if np.linalg.norm(val[2]) < th:
        #    break         
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        #if i>= timeout:
        #    break

def alignPose(robot, loc, timeout=100):
    pass

def get_pose(objects):
    angle = objects.pose.rotation.angle_z.radians
    pose = to_nppose(objects.pose.position.x_y_z, angle)
    return (pose, angle)

def cozmo_program(robot: cozmo.robot.Robot):
    robot.camera.image_stream_enabled = True
    i = 0
    transport, goal = lookAroundBehavior(robot) 
    print (transport,robot.pose,goal) 

    #if cube:
    #    cube.set_lights(cozmo.lights.green_light.flash())

    ##Robot pose
    robot_pos, rangle = get_pose(robot)

    ##Object pose
    object_pos, cangle = get_pose(transport)

    ##Goal pose
    goal_pos, gangle = get_pose(goal)
    #loc = np.array([x1,y1,[0]])

    ##Plan a virtual point
    vp = plan_virtualpoint(goal_pos[:2], object_pos[:2],robot_pos[:2])
    vp = np.array([[vp[0]],[vp[1]],[0]])

    ##Controller initialization
    ctrl = PIctrl()
    #ctrl.robot = robot
    #Set the reference point i.e object position
    ctrl.ref_point = vp
    print ('vp', vp)
    print('ro', robot_pos)
    print('go', goal_pos)
    print('ob', object_pos)
    img = visualize_path(goal_pos, object_pos, vp, robot_pos)
    #temp = np.copy(img[::-1,:,:])
    cv2.imwrite('path.jpg',img)
    angle = get_direction(vp, robot_pos)
    angle = angle - rangle

    #robot.turn_in_place(radians(angle)).wait_for_completed()
    #print ('robot turing completed')
    #return 
    #robot.turn_in_place(radians(np.pi + angle)).wait_for_completed()

    #Set the current robot location
    #ctrl.pos_update(robot_pos)
    #Update the controller i.e get new values of v and w
    #ctrl.update()
    gotobehavior(robot, vp, path=img, th=10)
    print ('gotobehavior for vp complte')

    ## Change the ange to 90

    gotoangle(robot, -np.pi/2, path=img)
    print ('gotoangle behavior completed')

    gotobehavior(robot, object_pos, path=img, th=20, tracking=True)
    print ('gotobehavior for object complte')    
    return
    ##goal pose
    # Get the direction to the object
    goal_pos, gangle = get_pose(goal)    
    robot_pos, rangle = get_pose(robot)    
    angle = get_direction(robot_pos, goal_pos)
    angle = angle - rangle
    print (angle)
    #time.sleep(200)
    robot.stop_all_motors()
    robot.turn_in_place(radians(np.pi+angle)).wait_for_completed()
    print ('direction changed towards goal')
    print ('gotobehavior for goal started')    
    gotobehavior(robot, goal_pos,th=55)
    robot.stop_all_motors()    


def main():
    cozmo.run_program(cozmo_program, use_viewer=True, force_viewer_on_top=True)
    #cozmo.run_program(cozmo_program)

if __name__ == '__main__':
    main()