# 当打开isaacsim rosbridge 时，isaacsim 自动会创建一个ros py 的环境
import numpy as np
import time

import omni.graph.core
import omni.replicator.core as rep

# 平均四帧发布一次，一次有40w+的点，还行（在不开rviz2 可视化的基础上）
class OgnCarLidarDataReceiverTutorialInternalState:

    def __init__(self):
        self.lidar_path = None
        self._cloud_data = None
        self._lidar_data = np.zeros((0, 3))
        self.initialized = False
        self.start = None
        self.timebase = None
        self.time_now = None
        self.internal = 0.25
        self.fulled = False

    def get_and_cloud_data(self):
        lidar_data_part = self.annotator.get_data()['data']
        # print(f'lidar_data_part size = {np.size(lidar_data_part)}')
        if np.size(lidar_data_part) == 0:
            print("OgnCarLidarDataReceiverTutorial lidar_data_part size is 0")
            return

        self._lidar_data = np.vstack([self._lidar_data, lidar_data_part])
        # print(f'OgnCarLidarDataReceiverTutorial lidar_data size = {np.size(self._lidar_data)} and lidar_data_part is {np.size(lidar_data_part)}')
        if time.time() - self.start > self.internal:
            print('fulled')
            self.time_now = time.time() - self.timebase
            self.fulled = True

    def initialize(self, lidar_path):
        self.lidar_path = lidar_path
        self.render_product = rep.create.render_product(self.lidar_path, resolution=(1024, 1024))
        self.annotator = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
        self.annotator.attach([self.render_product.path])
        print("OgnCarLidarDataReceiverTutorial annotator: ", self.annotator)
        self.timebase = time.time()
        self.start = time.time()

        self.initialized = True

    # 没法用
    def custom_reset(self):
        """On timeline stop, reset datas."""
        if not self.initialized:
            self.lidar_path = None
            self._lidar_data = None
            self.start = None
            self.timebase = None


class OgnCarLidarDataReceiverTutorial:

    _singleton_state = None

    # compute(db) → 必须实现，每次节点执行时都会被调。
    # initialize(node) → 可选，节点第一次创建时调用。
    # release(node) → 可选，节点被销毁时调用。
    # compute_async(db) → 可选，用于异步节点。

    @staticmethod
    def internal_state():
        if OgnCarLidarDataReceiverTutorial._singleton_state is None:
            OgnCarLidarDataReceiverTutorial._singleton_state = OgnCarLidarDataReceiverTutorialInternalState()
        return OgnCarLidarDataReceiverTutorial._singleton_state

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state

        try:
            if not state.initialized:
                state.initialize(lidar_path = db.inputs.lidarPath)
                print("OgnCarLidarDataReceiverTutorial initialize done")
            # -----------------
            if time.time() - state.timebase > 5:
                state.get_and_cloud_data()
                if state.fulled:
                    db.outputs.lidarData = state._lidar_data
                    db.outputs.timestamp = state.time_now
                    db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
                    state._lidar_data = np.zeros((0, 3))
                    state.fulled = False
                    state.start = time.time()
            else:
                print("OgnCarLidarDataReceiverTutorial time internal since timebase is ", time.time() - state.timebase)
            # -----------------
        except Exception as e:
            db.log_error(f"OgnCarLidarDataReceiverTutorial Computation error: {e}")
            return False
            
        return True

    # 官网上使用了一个Database 的生成类，但我不清楚为什么这里没有给我自动生成。
    # 可能是我没搞懂有其他办法，或者是官网尚未更新到这一块
    # @staticmethod
    # def release(node):
    #     """Release per-node state information."""
    #     # reset state
    #     state.reset()
    #     state.initialized = False
