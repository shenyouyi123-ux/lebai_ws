#!/usr/bin/env python3
import asyncio
import lebai_sdk
import nest_asyncio

nest_asyncio.apply()

async def main():
    lebai_sdk.init()
    robot_ip = "10.20.17.1" 
    lebai = await lebai_sdk.connect(robot_ip, False) 
    await lebai.teach_mode()

if __name__ == '__main__':
    asyncio.run(main())