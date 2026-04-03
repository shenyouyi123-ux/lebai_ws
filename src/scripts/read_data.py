#!/usr/bin/env python3
import asyncio
import lebai_sdk
import nest_asyncio

nest_asyncio.apply()

async def main():
    lebai_sdk.init()
    robot_ip = "10.20.17.1" 
    lebai = await lebai_sdk.connect(robot_ip, False) 
    status_dic = await lebai.get_kin_data()  
    print('反馈关节位置', status_dic['actual_joint_pose'])  
    print('反馈笛卡尔位置', status_dic['actual_tcp_pose'])

if __name__ == '__main__':
    asyncio.run(main())
