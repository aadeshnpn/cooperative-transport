import cozmo
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from cozmo.util import degrees, radians
import asyncio
import sys
from controller import (PIctrl, to_nppose, calc_wheel_velo, calc_vw, 
    get_direction, plan_virtualpoint, visualize_path, 
    add_obj_path, featureTracking, compute_features)

from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

#np.array(image.raw_image.convert("L"))
kMinNumFeature = 20
#goal = None
#transport = None

class CoopsTrans:
    def __init__(self, robot):
        self.robot = robot
        self.rand = np.random.random()*100
        self.cvviewname = 'path'+str(self.rand)
        self.timage0 = None
        self.gimage0 = None
    ## Function related to CV
    async def custom_objects(self):
        # Defined the geometical attribute of the goal object
        goal_obj = await self.robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                                CustomObjectMarkers.Triangles5,
                                                44,
                                                30, 30, True)

        box_obj = await self.robot.world.define_custom_box(CustomObjectTypes.CustomType03,
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

    async def fixed_goal_object(self,x,y,z):
        fixed_goal = await self.robot.world.create_custom_fixed_object(cozmo.util.Pose(x,y,z,angle_z=radians(0)), 10,10,10,relative_to_robot=False,use_robot_origin=True)
        if fixed_goal is not None:
            print ("Fixed goal object infron of the object created")
            return fixed_goal


    async def avoid_obstacles(self):
        i = 0
        while (len(self.tp) > 5 or len(self.gp) > 5):
            curr_img = self.robot.world.latest_image
            curr_img = np.array(curr_img.raw_image.convert("L"))        
            self.flow(self.tp, self.gp, curr_img)
            print (len(self.tp), len(self.gp))
            await asyncio.sleep(0.1)
            cv2.imshow('track', curr_img)
            cv2.waitKey(10)
            if i >=20:
                break
            await self.robot.drive_wheels(-5,5)
        

    def flow(self, tp, gp, curr_img):
        _, tp1 = featureTracking(self.timage0, curr_img, tp)
        _, tg1 = featureTracking(self.gimage0, curr_img, gp)
        if tp1 is not None:
            if len(tp1) > 2:
                self.timage0 = curr_img
                self.tp = tp1

        if tg1 is not None:
            if len(tg1) > 2:
                self.gimage0 = curr_img
                self.gp = tg1
        #return tp, gp

    async def lookAroundBehavior(self):
        look_around = self.robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)

        cube = None
        goal,transport = None, None
        t_points, g_points = None, None
        try:
            # This is for cube
            #cube = robot.world.wait_for_observed_light_cube(timeout=30)

            # This for for custom object
            world_objects = await self.robot.world.wait_until_observe_num_objects(1, object_type=CustomObject, timeout=40, include_existing=False)
            #global goal, transport
            for item in world_objects:
                #print(dir(item))
                #print (item.last_observed_image_box)
                area = item.x_size_mm * item.y_size_mm * item.z_size_mm #180000
                if area == 2100000:
                    transport = item
                    box = transport.last_observed_image_box
                    img = self.robot.world.latest_image
                    img = np.array(img.raw_image.convert("L"))
                    t_points,_ = compute_features(box, img)
                    self.timage0 = img
                else:
                    goal = item
                    goal = item
                    box = goal.last_observed_image_box
                    img = self.robot.world.latest_image
                    img = np.array(img.raw_image.convert("L"))
                    g_points,_ = compute_features(box, img)
                    self.gimage0 = img

        except asyncio.TimeoutError:
            print ("Didn't find interesting  object :(")
        finally:
            print ("stopping behavior")
            look_around.stop()

        look_around.stop()
        #return goal #, transport
        return transport, goal, t_points, g_points
        #return cube

    async def gotoangle(self, angle, path=None):
        ctrl = PIctrl()
        while True:
            rangle = self.robot.pose.rotation.angle_z.radians            
            robot_pos = to_nppose(self.robot.pose.position.x_y_z, rangle)           
            #print (robot)
            img = add_obj_path(path, robot_pos)
            cv2.imshow(self.cvviewname, img)#[:,:,::-1])
            ctrl.pos_update(robot_pos)
            val = ctrl.update_theta(angle)
            vr, vl = calc_wheel_velo(val)
            await self.robot.drive_wheels(vl, vr)
            await asyncio.sleep(0.1)        
            #print ('angle error', val[2])
            if np.linalg.norm(val[2]) < 0.1:
                break
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break            

    async def gotobehavior(self, loc, path=None, th=35, tracking=False):
        ctrl = PIctrl()
        #Set the reference point i.e object position
        ctrl.ref_point = loc
        #robot_pos, rangle = get_pose(robot)
        i = 0
        t0 = time.time() 
        runKLT = False   
        #detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        while True:
            i += 1
            ## Calc robot current pose
            rangle = self.robot.pose.rotation.angle_z.radians            
            robot_pos = to_nppose(self.robot.pose.position.x_y_z, rangle)
            print (i, robot_pos, t0)
            ctrl.pos_update(robot_pos)        
            ctrl.robot = self.robot
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
            await self.robot.drive_wheels(vl, vr)
            await asyncio.sleep(0.1)
            await self.avoid_obstacles()
            #await robot.
            if path is not None:
                img = add_obj_path(path, robot_pos)        
                cv2.imshow(self.cvviewname, img)#[:,:,::-1])
                old_image = self.robot.world.latest_image
                old_image = np.array(old_image.raw_image.convert("L"))
                #cv2.imshow('old',old_image)\
            """    
            if tracking:
                if runKLT == False:
                    old_image = self.robot.world.latest_image
                    old_image = np.array(old_image.raw_image.convert("L"))
                    #p0 = detector.detect(old_image)
                    #p0 = np.array([x.pt for x in p0], dtype=np.float32)
                    runKLT = True
                if runKLT == True:
                    new_image = self.robot.world.latest_image
                    new_image = np.array(new_image.raw_image.convert("L"))
                    #p0,p1,vel = featureTracking(old_image, new_image, p0)
                    #if p0 is not None:
                    #    p0 = p1
                    #    old_image = new_image
                    #    print (i, 'KLT vel', vel)
                    #else:
                    #    break
            """
            err = robot_pos - loc
            if np.linalg.norm(err[:2]) < th:
                break
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        return 

    def alignPose(self, loc, timeout=100):
        pass


    def secondphase(self, path):
        self.custom_objects()
        look_around = self.robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        try:    
            transport = self.robot.world.wait_until_observe_num_objects(1, object_type=CustomObject, timeout=30, include_existing=False)
            look_around.stop()        
            object_pos, cangle = self.get_pose(transport[0])
            self.gotobehavior(object_pos, path, th=60, tracking=True)
        except asyncio.TimeoutError:
            print ("Couldn't relocate the transport oject. Stopping")
            pass
        finally:
            look_around.stop()


    def get_pose(self, objects):
        angle = objects.pose.rotation.angle_z.radians
        pose = to_nppose(objects.pose.position.x_y_z, angle)
        return (pose, angle)

    async def cozmo_program(self):
        self.robot.camera.image_stream_enabled = True
        i = 0
        await self.custom_objects()        
        transport, goal, tp, gp = await self.lookAroundBehavior()
        if transport is None:             
            transport, _, tp, _ = await self.lookAroundBehavior()
        elif goal is None:
            _, goal, _, gp = await self.lookAroundBehavior()

        self.tp = tp
        self.gp = gp
        #print (transport,goal)
        #return
        ##Robot pose
        robot_pos, rangle = self.get_pose(self.robot)

        ##Object pose
        object_pos, cangle = self.get_pose(transport)

        ##Goal pose
        goal_pos, gangle = self.get_pose(goal)
        #loc = np.array([x1,y1,[0]])

        ##Plan a virtual point
        vp = plan_virtualpoint(goal_pos[:2], object_pos[:2],robot_pos[:2])
        vp = np.array([[vp[0]],[vp[1]],[0]])

        await self.robot.world.delete_all_custom_objects()
        ##Controller initialization

        ctrl = PIctrl()

        ctrl.ref_point = vp

        img = visualize_path(goal_pos, object_pos, vp, robot_pos)
        cv2.imwrite('path.jpg',img)
        
        await self.gotobehavior(vp, path=img, th=10)
        print ('gotobehavior for vp complte')
        return
        ## Change the ange to 90

        await self.gotoangle(-np.pi/2, path=img)
        print ('gotoangle behavior completed')
        self.robot.stop_all_motors()    
        return

        #print ('Second phase starting')
        #secondphase(robot, path=img)
        #print ('second phase end')
        #return

        #Last phase of pushing
        #gotobehavior(robot, goal_pos, path=img, th=50)
        #print ("object moved to goal")
        #robot.stop_all_motors()    
        #return 

    #async def multi_agent(conn):
    #    robot = await sdk_conn.wait_for_robot()    
    #    cozmo.run_program(cozmo_program(robot), use_viewer=True, force_viewer_on_top=True)

    async def multi_agent(self):
        #await cozmo.run_program(cozmo_program(robot), use_viewer=True, force_viewer_on_top=True)
        await self.cozmo_program()

#def main():
#    cozmo.run_program(cozmo_program, use_viewer=True, force_viewer_on_top=True)
    #cozmo.run_program(cozmo_program)

#if __name__ == '__main__':
#    main()