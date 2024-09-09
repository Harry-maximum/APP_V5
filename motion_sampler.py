import threading
import time
import math
import numpy as np

class MotionSampler:
    def __init__(self, interval_seconds, listener):
        self.interval_seconds = interval_seconds
        self.listener = listener
        self.is_running = False
        self.current_motion_map = {}
        self.last_motion_map = {}
        self.motion_status_map = {}
        self.pattern_type = 'UNKNOWN'
        self.thread = None
        self.init_position = (0.25, 0.31, 0.19, 0.25, -0.3, 0.19)
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
        for motion_type, (cur_l_x, cur_l_y, cur_l_z, cur_r_x, cur_r_y, cur_r_z) in self.current_motion_map.items():
            if self.is_dynamic_motion(motion_type):
                last_l_x, last_l_y, last_l_z, last_r_x, last_r_y, last_r_z = self.last_motion_map.get(motion_type, self.init_position)
                final_l_x = self.lerp(last_l_x, cur_l_x, 0.1)
                final_l_y = self.lerp(last_l_y, cur_l_y, 0.1)
                final_l_z = self.lerp(last_l_z, cur_l_z, 0.1)
                final_r_x = self.lerp(last_r_x, cur_r_x, 0.1)
                final_r_y = self.lerp(last_r_y, cur_r_y, 0.1)
                final_r_z = self.lerp(last_r_z, cur_r_z, 0.1)
                self.last_motion_map[motion_type] = (final_l_x, final_l_y,  final_l_z, final_r_x, final_r_y,  final_r_z)
                self.listener.on_sample_update(self.pattern_type, motion_type, final_l_x, final_l_y,  final_l_z, final_r_x, final_r_y,  final_r_z)
            else:
                self.listener.on_sample_update(self.pattern_type, motion_type, cur_l_x, cur_l_y, cur_l_z, cur_r_x, cur_r_y, cur_r_z)

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

    def update_data_source(self, pattern_type, motion_type, motion_l_x, motion_l_y, motion_l_z, motion_r_x, motion_r_y, motion_r_z):
        if self.pattern_type != pattern_type:
            self.reset()
            self.pattern_type = pattern_type

        if abs(motion_l_x) > 0 or abs(motion_l_y) > 0 or abs(motion_l_z) > 0 or abs(motion_r_x) > 0 or abs(motion_r_y) > 0 or abs(motion_r_z) > 0:
            self.motion_status_map[motion_type] = 'WORKING'
        else:
            self.motion_status_map[motion_type] = 'IDLE'

        self.update_motion_status()
        self.current_motion_map[motion_type] = (motion_l_x, motion_l_y, motion_l_z, motion_r_x, motion_r_y, motion_r_z)

    def clear(self):
        for motion_type in list(self.motion_status_map.keys()):
            self.motion_status_map[motion_type] = 'IDLE'
        self.update_motion_status()

# Example listener class for handling the motion samples
class MotionSampleListener:
    def on_sample_update(self, pattern_type, motion_type, x_l , y_l , z_l, x_r , y_r , z_r ):
        # print(f"Pattern: {pattern_type}, Motion: {motion_type}, X_L: {x_l}, Y_L: {y_l}, Z_L: {z_l}, X_R: {x_r}, Y_R: {y_r}, Z_R: {z_r}")
        sample_update = np.array([x_l, y_l , z_l, x_r , y_r , z_r])
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
