import cozmo
import cv2
#import cv2.aruco as aruco
import numpy as np
import time
from cozmo.util import degrees, radians
import asyncio
from controller import PIctrl, to_nppose, calc_wheel_velo, calc_vw, get_direction
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes


goal = None
transport = None

"""
# Cozmo handlers
def handle_object_appeared(evt, **kw):
    # This will be called whenever an EvtObjectAppeared is dispatched -
    # whenever an Object comes into view.
    if isinstance(evt.obj, CustomObject):
        global goal, transport
        if evt.obj.object_id==5:
            goal = evt.obj

        if evt.obj.object_id==6:
            transport = evt.obj            
        print("Cozmo started seeing a %s" % str(evt.obj.object_type))
        print('custome object pose',evt.obj.object_id, evt.obj.pose)

def handle_object_disappeared(evt, **kw):
    # This will be called whenever an EvtObjectDisappeared is dispatched -
    # whenever an Object goes out of view.
    if isinstance(evt.obj, CustomObject):
        print("Cozmo stopped seeing a %s" % str(evt.obj.object_type))
"""
## Function related to CV
def custom_objects(robot):
    # Defined the geometical attribute of the goal object
    goal_obj = robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                              CustomObjectMarkers.Triangles5,
                                              44,
                                              30, 30, True)
    # Defind the geometical attribute of the objec ot interest                                              
    wall_obj = robot.world.define_custom_wall(CustomObjectTypes.CustomType02,
                                              CustomObjectMarkers.Hexagons5,
                                              160, 125,
                                              105, 100, True)                                              
    """
    # Defined the geometrical attribute of the object of interest (oject to transport)
    # This dimension was for a android block
    wall_obj = robot.world.define_custom_wall(CustomObjectTypes.CustomType02,
                                              CustomObjectMarkers.Hexagons5,
                                              150, 120,
                                              70, 70, True)
    """

    if (goal_obj is not None) and (wall_obj is not None):
        print ("Goal obect and object to transport defined properly")
        #return goal_obj, wall_obj
    #else:
    #    return None, None


def lookAroundBehavior(robot):
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    custom_objects(robot)
    cube = None
    try:
        # This is for cube
        #cube = robot.world.wait_for_observed_light_cube(timeout=30)

        # This for for custom object
        world_objects = robot.world.wait_until_observe_num_objects(2, object_type=CustomObject, timeout=50, include_existing=False)
        global goal, transport
        for item in world_objects:
            area = item.x_size_mm * item.y_size_mm * item.z_size_mm
            if area == 180000:
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

def gotobehavior(robot, loc, th=35):
    ctrl = PIctrl()
    #Set the reference point i.e object position
    ctrl.ref_point = loc
    #robot_pos, rangle = get_pose(robot)
    i = 0
    t0 = time.time()
    while True:
        i += 1
        ## Calc robot current pose
        rangle = robot.pose.rotation.angle_z.radians            
        robot_pos = to_nppose(robot.pose.position.x_y_z, rangle)        
        ctrl.pos_update(robot_pos)        

        ## Update the refrence object for the controller
        ctrl.ref_point = loc
        ## Run the PID controller
        t1 = time.time()
        dt = t1 - t0
        val = ctrl.update(dt)
        t0 = t1
        ## Get get while velocityies from linear and angular velocity
        vr, vl = calc_wheel_velo(val)
        ## Send the velocity commands to the robot
        robot.drive_wheels(vl, vr)
        #print('motion',robot_pos, loc)
        if np.linalg.norm(val[2]) <th:
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
    x1 = robot_pos[0];y1 = object_pos[1]
    #loc = np.array([x1,y1,[0]])


    ##Controller initialization
    ctrl = PIctrl()
    #Set the reference point i.e object position
    ctrl.ref_point = object_pos

    # Get the direction to the object
    robot_pos, rangle = get_pose(robot)    
    angle = get_direction(robot_pos, object_pos)
    angle = angle - rangle

    robot.turn_in_place(radians(np.pi + angle)).wait_for_completed()
    print ('robot turing completed')
    exit()
    #robot.turn_in_place(radians(np.pi + angle)).wait_for_completed()

    #Set the current robot location
    #ctrl.pos_update(robot_pos)
    #Update the controller i.e get new values of v and w
    #ctrl.update()
    gotobehavior(robot, object_pos, th=1)
    print ('gotobehavior for object complte')
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
    """
    while True:
        image = robot.world.latest_image
        try:
            i+=1
            ## Set cozmo lights to know which object has been detected
            #cube.set_lights(cozmo.lights.green_light.flash())            
            #print ('from loop')
            ## Opencv camera view object
            gray = np.array(image.raw_image.convert("L"))
            cv2.imshow('cozmo', gray)

            ## Calc robot current pose
            rangle = robot.pose.rotation.angle_z.radians            
            robot_pos = to_nppose(robot.pose.position.x_y_z, rangle)
            
            ## Calc object current pose
            cangle = cube.pose.rotation.angle_z.radians
            object_pos = to_nppose(cube.pose.position.x_y_z, cangle)

            ## Update the refrence object for the controller
            ctrl.ref_point = object_pos
            ## Run the PID controller
            val = ctrl.update()
            ## Get get while velocityies from linear and angular velocity
            vr, vl = calc_wheel_velo(val)
            ## Update the current position of robot
            ctrl.pos_update(robot_pos)

            #ctrl.ref_dyna = calc_vw(robot.right_wheel_speed.speed_mmps, robot.left_wheel_speed.speed_mmps)
            ## Send the velocity commands to the robot
            #print (vr, vl)
            robot.drive_wheels(vl, vr)

            ##opencv function to wait. Required to display the video of cozmo
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break            

            ## Check is the object has reached the destined 
            ## 45 for cube, 75 for actual box
            if np.linalg.norm(val[2]) < 35:
                ## Attached to the object
                ## Now next controller into play
                break
            else:
                ## Continue with the same controller
                print (i, np.linalg.norm(val[2]))
                
            if i>40000:
                break
            #if cube:
            #    print ("cube seen. Using a controller to drive to cube")

        except (AttributeError, cv2.error):
        #    print (AttributeError, cv2.error)
            pass
    """

def main():
    cozmo.run_program(cozmo_program, use_viewer=True, force_viewer_on_top=True)
    #cozmo.run_program(cozmo_program)

if __name__ == '__main__':
    main()