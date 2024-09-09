import numpy as np

def string_input_hdlr(input_str: str) -> np.ndarray:
 
    # string2list = input_str.strip('[]').split(',')
    string2list = input_str.split(',')
    # 将列表转换为 NumPy 数组并指定为整数类型
    list2array = np.array(string2list, dtype=float)

    return list2array