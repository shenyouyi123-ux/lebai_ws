#!/usr/bin/env python3
import asyncio
import lebai_sdk
import nest_asyncio

nest_asyncio.apply()

async def main():
    lebai_sdk.init()
    robot_ip = "10.20.17.1" 
    lebai = await lebai_sdk.connect(robot_ip, False) 
    
    cartesian_pose = {'x': -0.3965707803457132, 'y': -0.12013732070099106, 'z': 0.4599167012982839, 'rx': -3.1375334965390245, 'ry': -0.008939972222492161, 'rz': -1.560954199852243}
    a = 5
    v = 10
    t = 0
    r = 0.5
    
    motion_id2 = await lebai.movej(cartesian_pose, a, v, t, r)

if __name__ == '__main__':
    asyncio.run(main())