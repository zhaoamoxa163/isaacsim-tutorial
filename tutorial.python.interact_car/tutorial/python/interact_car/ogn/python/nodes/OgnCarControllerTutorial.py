# MIT License
# 
# Copyright (c) 2024 <COPYRIGHT_HOLDERS>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 

"""
OmniGraph core Python API:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/Overview.html

OmniGraph attribute data types:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/dev/ogn/attribute_types.html

Collection of OmniGraph code examples in Python:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/dev/ogn/ogn_code_samples_python.html

Collection of OmniGraph tutorials:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.tutorials/latest/Overview.html
"""

# 当打开isaacsim rosbridge 时，isaacsim 自动会创建一个ros py 的环境
# 这里会报错，但你先别管，写就完了
import rclpy
import std_msgs.msg

import omni.usd as USD
from isaacsim.core.nodes import BaseResetNode   # 这个要不要其实无所谓


class OgnCarControllerTutorialTutorialInternalState(BaseResetNode):
    def __init__(self):
        self._data = None
        self._ros2_node = None
        self._subscription = None
        self.front_left_wheel = None
        # 只有当initialize 方法被调用时才进行初始化，避免提前初始化
        super().__init__(initialize=False)

    # 让 data 可以像访问属性一样访问（如 obj.data），而不是方法（如 obj.data()）。
    # @property
    # def data(self):
    #     tmp = self._data
    #     self._data = None
    #     return tmp
    
    def _callback(self, msg):
        print(f"Received string")
        self._data = msg.data

    def initialize(self, node_name, topic_name):
        try:
            rclpy.init()
        except RuntimeError as e:
                print("error occured in :", e)
        except Exception as e:
            # 捕获其他意外异常
            print(f"Unexpected error during ROS 2 initialization: {e}")

        # create ROS 2 node
        if rclpy.ok() and not self._ros2_node:
            self._ros2_node = rclpy.create_node(node_name=node_name)
        # create ROS 2 subscription
        if not self._subscription:
            self._subscription = self._ros2_node.create_subscription(
                # 回调函数中就会接收一个string 的数据类型
                msg_type=std_msgs.msg.String, topic=topic_name, callback=self._callback, qos_profile=10
            )
            print("ros2 node create done: ", self._subscription)
        
        # 获取轮关节的attribute
        if self.front_left_wheel is None:
            self.front_left_wheel = USD.get_prim_at_path("/World/jackal_basic/base_link/front_left_wheel")
            self.front_right_wheel = USD.get_prim_at_path("/World/jackal_basic/base_link/front_right_wheel")
            self.rear_left_wheel = USD.get_prim_at_path("/World/jackal_basic/base_link/rear_left_wheel")
            self.rear_right_wheel = USD.get_prim_at_path("/World/jackal_basic/base_link/rear_right_wheel")
            self.front_left_wheel_attr = self.front_left_wheel.GetAttribute("drive:angular:physics:targetVelocity")
            self.front_right_wheel_attr = self.front_right_wheel.GetAttribute("drive:angular:physics:targetVelocity")
            self.rear_left_wheel_attr = self.rear_left_wheel.GetAttribute("drive:angular:physics:targetVelocity")
            self.rear_right_wheel_attr = self.rear_right_wheel.GetAttribute("drive:angular:physics:targetVelocity")

        if self.front_left_wheel_attr is not None:
            print("wheel attribute: ", self.front_left_wheel_attr, "is ", self.front_left_wheel_attr.Get())
        else: print("OgnCarControllerTutorial get wheel attribute error!")

        # 只要在后期调用这种self，就会自动添加到当前属性中
        self.initialized = True
    
    # 启动节点
    def spin_once(self, timeout_sec=0.01):
        """Do ROS 2 work to take an incoming message from the topic, if any."""
        rclpy.spin_once(self._ros2_node, timeout_sec=timeout_sec)
        # print("OgnCarControllerTutorial ros spining ...")

# --------------------- 上边这个类，主要是做逻辑运行 ----------------------- #
# ----------------- 下边这个类，主要是做IsaacSim 运行控制 ------------------ #

class OgnCarControllerTutorial:
    """The Ogn node class"""
    _singleton_state = None

    # @staticmethod 将当前方法定义为static
    # 此方法的作用为使OgnCarControllerTutorial 这个类作为单例模式运行，不会出现每次调用都重新创建的现象
    # 此方法必写，也可以不做单例模式，其他模式也可以，但此方法必写
    @staticmethod
    def internal_state():
        if OgnCarControllerTutorial._singleton_state is None:
            OgnCarControllerTutorial._singleton_state = OgnCarControllerTutorialTutorialInternalState()
        return OgnCarControllerTutorial._singleton_state

    # 这个类是运行时方法
    # 其中的db 是一个database ，节点的传入数据和传出数据都需要使用这个db 传送。
    @staticmethod
    def compute(db) -> bool:

        state = db.per_instance_state
        speed_map = {
            "w": (500, 500, 500, 500),
            "s": (-500, -500, -500, -500),
            "a": (-300, 300, -300, 300),  # 示例：左转
            "d": (300, -300, 300, -300)   # 示例：右转
        }

        try:
            if not state.initialized:
                # 这个就是使用了db.inputs.topic 数据
                state.initialize(node_name="rcv_control_cmd", topic_name=db.inputs.topic)
                print("OgnCarControllerTutorial initial done")

            # 其实这行不调用也行，因为ros 的主线程在其他的代码中会一直刷新
            state.spin_once()
            
            # 其实不要输出的，直接在compute 中赋值，定义各方向对应的轮子速度字典
            # 获取当前方向的速度组合（默认全0）
            speeds = speed_map.get(state._data, (0, 0, 0, 0))
            print("state._data: ", state._data)

            # 依次赋值
            state.front_left_wheel_attr.Set(speeds[0])
            state.front_right_wheel_attr.Set(speeds[1])
            state.rear_left_wheel_attr.Set(speeds[2])
            state.rear_right_wheel_attr.Set(speeds[3])
            # db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED


        except Exception as e:
            db.log_error(f"OgnCarControllerTutorial Computation error: {e}")
            return False
        return True