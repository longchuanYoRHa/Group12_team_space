from pymycobot.mycobot280 import MyCobot280
import time
#输入以上代码导入工程所需要的包

def gripper_test(mc):
    print("Start check IO part of api\n")
    # 检测夹爪是否正在移动
    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)

    # Set the current position to (2048).
    # Use it when you are sure you need it.
    # Gripper has been initialized for a long time. Generally, there
    # is no need to change the method.
    # mc.set_gripper_ini()
    # 设置关节点1，让其转动到2048这个位置
    mc.set_encoder(1, 2048, 20)
    time.sleep(2)
    # 设置六个关节位，让机械臂以20的速度转动到该位置
    mc.set_encoders([1024, 1024, 1024, 1024, 1024, 1024], 20)
    time.sleep(3)


    # 以70的速度让夹爪到达100状态
    mc.set_gripper_value(100, 70)
    time.sleep(3)
    # 以70的速度让夹爪到达0状态
    mc.set_gripper_value(0, 70)
    time.sleep(3)

    # 设置夹爪的状态，让其以70的速度快速打开爪子
    mc.set_gripper_state(0, 70)
    time.sleep(3)
    # 设置夹爪的状态，让其以70的速度快速收拢爪子
    mc.set_gripper_state(1, 70)
    time.sleep(3)

    # 获取夹爪的值
    print("")
    print(mc.get_gripper_value())


if __name__ == "__main__":
    # MyCobot 类初始化需要两个参数：串口号和波特率
    # 初始化一个MyCobot280对象
    # 下面为PI版本创建对象代码
    mc = MyCobot280('/dev/ttyAMA0', 1000000)
    # 让其移动到零位
    mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 20)
    time.sleep(3)
    gripper_test(mc)
