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


multi_agent = False

async def turn_left(sdk_conn):
    robot = await sdk_conn.wait_for_robot()
    cozmo.logger.info("multi agent")
    task = CoopsTrans(robot)
    await task.multi_agent()
    #await multi_agent(robot)
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
        loop.run_until_complete(asyncio.gather(task1, task2))
    else:
        loop.run_until_complete(asyncio.gather(task1))
    # wait for both coroutines to complete before exiting the program
