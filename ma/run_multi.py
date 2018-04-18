#!/usr/bin/env python3

# Copyright (c) 2016 Anki, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''An example of running independent concurrent routines on multiple Cozmos.

Each robot requires its own device to control it.
'''

import asyncio
import sys

import cozmo
import numpy as np
from cozmo.util import degrees
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from run import CoopsTrans
from controller import visualize_objects, compute_features
import cv2

multi_agent = False

async def test_goal(robot):
    task = CoopsTrans(robot)
    fixed_goal = await task.fixed_goal_object(100, -100, 0)
    #print (task.robot.pose)
    #item,_  = await task.lookAroundBehavior()
    print (robot.robot_id)
    #print ('item',item)
    print ('fixed', fixed_goal.pose)
    #print (task.robot.pose = item.pose)
    print ('robot',task.robot.pose)
    new_pose = task.robot.pose.define_pose_relative_this(fixed_goal.pose)
    await asyncio.sleep(0.1)    
    #print(dir(robot))
    task.robot._pose = new_pose
    print ('after robot', task.robot.pose.position)
    #print ('after fixed', fixed_goal.pose.position)   
    #visualize_objects(robot)
    #cv2.imshow(self.cvviewname, img)
    #if cv2.waitKey(9000) & 0xFF == ord('q'):
    #    return
    #pass
    return robot.pose.position

async def turn_left(sdk_conn):
    robot = await sdk_conn.wait_for_robot()
    cozmo.logger.info("multi agent")
    task = CoopsTrans(robot)
    task.robot.camera.image_stream_enabled = True    
    await task.multi_agent()
    #await multi_agent(robot)
    """
    item, goal = await task.lookAroundBehavior()
    task.robot.stop_all_motors()
    box = item.last_observed_image_box
    img = task.robot.world.latest_image
    img = np.array(img.raw_image.convert("L"))  
    p0, mask_image = compute_features(box, img) 
    """
    """    
    x1,y1 = int(box.left_x), int(box.bottom_y)
    x2,y2 = int(box.right_x), int(box.top_y)
    w,h= int(box.width), int(box.height)

    cv2.rectangle(img,(x1,y1),(x2,y2),(255,255,0),5)

    mask_image = img.copy()
    mask_image[:,:] = 255
    start_point = (x1, y1)
    end_point = (x2, y2)
    
    mask_image[y2:y2+h, x1:x1+w] = img[y2:y2+h, x1:x1+w]
    detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
    p0 = detector.detect(mask_image)
    """
    #kpimg = cv2.drawKeypoints(mask_image, p0, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv2.imshow('object', kpimg)
    #if cv2.waitKey(9000) & 0xFF == ord('q'):
    #    return    
    
    #pos = await test_goal(robot)
    #return pos
    """
    loc = np.ones((3,1))
    loc[1] = 100    
    loc[1] = 20
    loc[2] = 0.2
    #await gotobehavior(robot,loc,path=None, th=1,tracking=False)
    #await gotoangle(robot,np.pi/2)
    i = 0
    while True:
        i += 1
        print (i,robot.pose)
        await robot.drive_wheels(50,50)
        #print (dir(robot))
        await asyncio.sleep(0.1)
        #break
    """
    
    #await robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace).wait_for_robot()
    #t1 = await robot.world.wait_until_observe_num_objects(1, object_type=CustomObject, timeout=30, include_existing=False)

if __name__ == '__main__':
    cozmo.setup_basic_logging()
    loop = asyncio.get_event_loop()

    # Connect to both robots
    try:
        conn1 = cozmo.connect_on_loop(loop)
        if multi_agent:
            conn2 = cozmo.connect_on_loop(loop)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)

    # Run two independent coroutines concurrently, one on each connection
    task1 = asyncio.ensure_future(turn_left(conn1), loop=loop)
    if multi_agent:
        task2 = asyncio.ensure_future(turn_left(conn2), loop=loop)
        ret = loop.run_until_complete(asyncio.gather(task1, task2))
        print (ret)
    else:
        loop.run_until_complete(asyncio.gather(task1))
    # wait for both coroutines to complete before exiting the program
