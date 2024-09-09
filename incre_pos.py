class MotionSampler:
    def __init__(self):
        self.direction_map = {}

    def calculate_position_by_timer(self, direction_type, value):
        delta_dt = 1 / 200.0
        cur_value = self.direction_map.get(direction_type, (0.0, 0.0))
        last_value, last_position = cur_value

        # 判断摇杆趋势，正拨还是回弹中
        up_forward = value > 0 and (value - last_value) >= 0
        # 计算回弹方向
        up_rebound = value > 0 and (value - last_value) < 0
        # 计算负拨动方向
        down_forward = value < 0 and (value - last_value) <= 0
        # 计算负回弹方向
        down_rebound = value < 0 and (value - last_value) > 0

        delta_position = 0.0
        if up_forward:
            # 正拨中计算增量
            delta_position = (value + last_value) * delta_dt / 2.0
        elif up_rebound:
            # 正回弹计算增量
            delta_position = (value + last_value) * delta_dt / 2.0 / 2.0
        elif down_forward:
            # 计算负向拨动增量
            delta_position = (value + last_value) * delta_dt / 2.0
        elif down_rebound:
            # 计算负回弹增量
            delta_position = (value + last_value) * delta_dt / 2.0 / 2.0

        target_position = last_position + delta_position
        
        if target_position > 2.5 / 2:
            target_position = 2.5 / 2
        elif target_position < -2.5 / 2:
            target_position = -2.5 / 2

        # 更新上次结果
        self.direction_map[direction_type] = (value, target_position)

        # 这里不做归一化处理
        # target_position /= 1.25
        # if abs(target_position) < 0.05:
        #     target_position = 0.0

        return target_position

# 使用示例
sampler = MotionSampler()
result = sampler.calculate_position_by_timer("DIRECTION_TYPE", 1.0)
print(result)
