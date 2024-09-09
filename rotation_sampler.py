import threading
import time
import math
import numpy as np

class RotationSampler:
    def __init__(self, interval_seconds, listener):
        self.interval_seconds = interval_seconds
        self.listener = listener
        self.is_running = False
        self.current_motion_map = {}
        self.last_motion_map = {}
        self.motion_status_map = {}
        self.pattern_type = 'UNKNOWN'
        self.thread = None
        self.init_position = (0.27, -0.27)
        # self.lock = threading.Lock()
        


    def start(self):
        if not self.is_running:
            self.is_running = True
            self.thread = threading.Thread(target=self._sample_routine)
            self.thread.start()

    def stop(self):
        if self.is_running:
            self.is_running = False
            if self.thread:
                self.thread.join()

    def reset(self):
        self.last_motion_map.clear()
        self.current_motion_map.clear()
        self.motion_status_map.clear()

    def lerp(self, start, end, t):
        return start + t * (end - start)

    def _sample_routine(self):
        while self.is_running:
            self.on_sample()
            time.sleep(self.interval_seconds)

    def on_sample(self):
        for motion_type, (cur_l, cur_r) in self.current_motion_map.items():
            if self.is_dynamic_motion(motion_type):
                last_l, last_r = self.last_motion_map.get(motion_type, self.init_position)
                final_l = self.lerp(last_l, cur_l, 0.1)
                final_r = self.lerp(last_r, cur_r, 0.1)
                self.last_motion_map[motion_type] = (final_l, final_r)
                self.listener.on_sample_update(self.pattern_type, motion_type, final_l, final_r)
            else:
                self.listener.on_sample_update(self.pattern_type, motion_type, cur_l, cur_l)

    def is_dynamic_motion(self, motion_type):
        return motion_type in ['MOVE']

    def update_motion_status(self):
        for motion_type, status in self.motion_status_map.items():
            if status == 'WORKING':
                self.start()
            else:
                time.sleep(1)  
                self.stop_motion_sample(motion_type)

    def stop_motion_sample(self, motion_type):
        if motion_type not in self.current_motion_map:
            return
        self.last_motion_map[motion_type] = self.current_motion_map.pop(motion_type)
        self.listener.on_sample_update(self.pattern_type, motion_type, *self.last_motion_map[motion_type])

        if not self.current_motion_map:
            self.stop()
            self.reset()

    def update_data_source(self, pattern_type, motion_type, rot_l, rot_r):
        if self.pattern_type != pattern_type:
            self.reset()
            self.pattern_type = pattern_type

        if abs(rot_l) > 0 or abs(rot_l) > 0:
            self.motion_status_map[motion_type] = 'WORKING'
        else:
            self.motion_status_map[motion_type] = 'IDLE'

        self.update_motion_status()
        self.current_motion_map[motion_type] = (rot_l, rot_r)

    def clear(self):
        for motion_type in list(self.motion_status_map.keys()):
            self.motion_status_map[motion_type] = 'IDLE'
        self.update_motion_status()

# Example listener class for handling the motion samples
class RotationSampleListener:
    def on_sample_update(self, pattern_type, motion_type, rot_l , rot_r):
        # print(f"Pattern: {pattern_type}, Motion: {motion_type}, rot_L: {rot_l}, rot_R: {rot_r}")
        sample_update = np.array([rot_l, rot_r])
        return sample_update


# # Usage example
# if __name__ == "__main__":
    
#     listener = MotionSampleListener()
#     sampler = MotionSampler(0.5, listener)
    
#     # Example to start sampling
#     sampler.update_data_source('WALKING', 'LEFT', 1.0, 0.5)
#     sampler.update_data_source('WALKING', 'RIGHT', 0.9, 0.3)
    
#     time.sleep(5)
#     sampler.stop()
