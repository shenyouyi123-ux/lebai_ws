#!/usr/bin/env python3
import asyncio
import lebai_sdk
import nest_asyncio

nest_asyncio.apply()

async def main():
    lebai_sdk.init()
    robot_ip = "10.20.17.1" 
    lebai = await lebai_sdk.connect(robot_ip, False) 
    
   
    cartesian_pose = {'x': 0.14963173555890788, 'y': 0.3491659341000903, 'z': 0.6409318568220694, 'rx': -2.6196965207853453, 'ry': -0.010823764024352738, 'rz': -0.07815202317393552}
    a = 5
    v = 6
    t = 0
    r = 0.5
    motion_id2 = await lebai.movej(cartesian_pose, a, v, t, r)

if __name__ == '__main__':
    asyncio.run(main())