l20_l_min = [-1.57, 0, 0, 0, 0, 0, -0.26, -0.26, -0.26, -0.26, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l20_l_max = [0, 1.57, 1.57, 1.57, 1.57, 1.57, 0.26, 0.26, 0.26, 0.26, 0, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57]
l20_l_derict = [0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1]    #这个的意思就是方向，有负数，不管是几都设计成0。。
l20_r_min = [0, 0, 0, 0, 0, -1.57, -0.26, -0.26, -0.26, -0.26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l20_r_max = [1.57, 1.57, 1.57, 1.57, 1.57, 0, 0.26, 0.26, 0.26, 0.26, 1, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57]
l20_r_derict = [-1, -1, -1, -1, -1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]

l10_l_min = [-1.03, 0, 0, 0, 0, 0, -0.26, -0.26, -0.26, -1.57]
l10_l_max = [0, 1.57, 1.3, 1.3, 1.3, 1.3, 0.26, 0.26, 0.26, 0]
l10_l_derict = [0, -1, -1, -1, -1, -1, 0, -1, -1, 0]
l10_r_min = [0, -1.57, 0, 0, 0, 0, -0.26, -0.26, -0.26, 0]
l10_r_max = [1.03, 0, 1.3, 1.3, 1.3, 1.3, 0.26, 0.26, 0.26, 1.57]
l10_r_derict = [-1, 0, -1, -1, -1, -1, -1, 0, 0, -1]

#在这里我得输入25个最大最小,,,除了三个侧摆的，其他都是255是张开状态，大拇指255侧摆也是张开状态
#2
l25_l_min=[-0.9,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.18,-0.18,-0.26,0,0,0,0,-1.57,-1.57,-1.57,-1.57,-1.57,-1.57,-1.57,-1.57,-1.57,-1.57]
l25_l_max=[0,1.57,1.57,1.57,1.57,1.3,0.18,0.0,0,0,0.61,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
l25_l_derict=[0,-1,-1,-1,-1,-1,0,0,0,-1,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

l25_r_min=[0,    0,   0,   0,  0, -1.3,-0.18,0,   0 ,    0,    -0.26,0,0,0,0,  -1.57,   0,     0,   0,    0,  -1.57,    0,    0,    0,    0]
l25_r_max=[0.9,1.57,1.57,1.57,1.57, 0 ,  0,  0,  0.18,  0.18,   0.61,0,0,0,0,    0    ,1.57,  1.57,1.57, 1.57,  0,     1.57, 1.57, 1.57, 1.57]
l25_r_derict=[-1,-1, -1,  -1, -1,   0,  -1,  0,   0,     0,      -1, 0,0,0,0,    0,     -1,    -1,  -1,   -1,   0,      -1,   -1,   -1,   -1]

def range_to_arc_right(hand_range_r):
    hand_arc_r = [0] * 20
    for i in range(20):
        if 11 <= i <= 14: continue
        val_r = is_within_range(hand_range_r[i], 0, 255)
        if l20_r_derict[i] == -1:
            hand_arc_r[i] = scale_value(val_r, 0, 255, l20_r_max[i], l20_r_min[i])
        else:
            hand_arc_r[i] = scale_value(val_r, 0, 255, l20_r_min[i], l20_r_max[i])
    return hand_arc_r


def range_to_arc_left(hand_range_l):
    hand_arc_l = [0] * 20
    for i in range(20):
        if 11 <= i <= 14: continue
        val_l = is_within_range(hand_range_l[i], 0, 255)
        if l20_l_derict[i] == -1:
            hand_arc_l[i] = scale_value(val_l, 0, 255, l20_l_max[i], l20_l_min[i])
        else:
            hand_arc_l[i] = scale_value(val_l, 0, 255, l20_l_min[i], l20_l_max[i])
    return hand_arc_l

def range_to_arc_left_l25(hand_range_l):
    hand_arc_l = [0] * 25
    for i in range(25):
        if i==7:continue
        if 11 <= i <= 14: continue
        val_l = is_within_range(hand_range_l[i], 0, 255)
        if l25_l_derict[i] == -1:
            hand_arc_l[i] = scale_value(val_l, 0, 255, l25_l_max[i], l25_l_min[i])
        else:
            hand_arc_l[i] = scale_value(val_l, 0, 255, l25_l_min[i], l25_l_max[i])
    return hand_arc_l

def range_to_arc_right_l25(hand_range_r):
    hand_arc_r = [0] * 25
    for i in range(25):
        if i==7:continue
        if 11 <= i <= 14: continue
        val_r = is_within_range(hand_range_r[i], 0, 255)
        if l25_r_derict[i] == -1:
            hand_arc_r[i] = scale_value(val_r, 0, 255, l25_r_max[i], l25_r_min[i])
        else:
            hand_arc_r[i] = scale_value(val_r, 0, 255, l25_r_min[i], l25_r_max[i])
    return hand_arc_r

def arc_to_range_right(hand_arc_r):
    hand_range_r = [0] * 20
    for i in range(20):
        if 11 <= i <= 14: continue
        val_r = is_within_range(hand_arc_r[i], l20_r_min[i], l20_r_max[i])
        if l20_r_derict[i] == -1:
            hand_range_r[i] = scale_value(val_r, l20_r_min[i], l20_r_max[i], 255, 0)
        else:
            hand_range_r[i] = scale_value(val_r, l20_r_min[i], l20_r_max[i], 0, 255)
    return hand_range_r


def arc_to_range_left(hand_arc_l):
    hand_range_l = [0] * 20    #20个范围关节
    for i in range(20):
        if 11 <= i <= 14: continue
        val_l = is_within_range(hand_arc_l[i], l20_l_min[i], l20_l_max[i])####这个返回就是返回必须在范围内的弧度，小于min的返回min，大于max的返回max。。。
        if l20_l_derict[i] == -1:
            hand_range_l[i] = scale_value(val_l, l20_l_min[i], l20_l_max[i], 255, 0)
        else:
            hand_range_l[i] = scale_value(val_l, l20_l_min[i], l20_l_max[i], 0, 255)

    return hand_range_l

def arc_to_range_left_l25(hand_arc_l):
    hand_range_l = [0] * 25    #25个范围关节
    for i in range(25):
        if i==7:continue
        if 11 <= i <= 14: continue
        val_l = is_within_range(hand_arc_l[i], l25_l_min[i], l25_l_max[i])####这个返回就是返回必须在范围内的弧度，小于min的返回min，大于max的返回max。。。
        if l25_l_derict[i] == -1:
            hand_range_l[i] = scale_value(val_l, l25_l_min[i], l25_l_max[i], 255, 0)
        else:
            hand_range_l[i] = scale_value(val_l, l25_l_min[i], l25_l_max[i], 0, 255)

    return hand_range_l

def arc_to_range_right_l25(hand_arc_r):
    hand_range_r = [0] * 25
    for i in range(25):
        if i==7:continue
        if 11 <= i <= 14: continue
        val_r = is_within_range(hand_arc_r[i], l25_r_min[i], l25_r_max[i])
        if l25_r_derict[i] == -1:
            hand_range_r[i] = scale_value(val_r, l25_r_min[i], l25_r_max[i], 255, 0)
        else:
            hand_range_r[i] = scale_value(val_r, l25_r_min[i], l25_r_max[i], 0, 255)
    return hand_range_r

def range_to_arc_right_10(hand_range_r):
    hand_arc_r = [0] * 10
    for i in range(10):
        val_r = is_within_range(hand_range_r[i], 0, 255)
        if l10_r_derict[i] == -1:
            hand_arc_r[i] = scale_value(val_r, 0, 255, l10_r_max[i], l10_r_min[i])
        else:
            hand_arc_r[i] = scale_value(val_r, 0, 255, l10_r_min[i], l10_r_max[i])

    return hand_arc_r


def range_to_arc_left_10(hand_range_l):
    hand_arc_l = [0] * 10
    for i in range(10):
        val_l = is_within_range(hand_range_l[i], 0, 255)
        if l10_l_derict[i] == -1:
            hand_arc_l[i] = scale_value(val_l, 0, 255, l10_l_max[i], l10_l_min[i])
        else:
            hand_arc_l[i] = scale_value(val_l, 0, 255, l10_l_min[i], l10_l_max[i])
    return hand_arc_l


def arc_to_range_right_10(hand_arc_r):
    hand_range_r = [0] * 10
    for i in range(10):
        val_r = is_within_range(hand_arc_r[i], l10_r_min[i], l10_r_max[i])
        if l10_r_derict[i] == -1:
            hand_range_r[i] = scale_value(val_r, l10_r_min[i], l10_r_max[i], 255, 0)
        else:
            hand_range_r[i] = scale_value(val_r, l10_r_min[i], l10_r_max[i], 0, 255)
    return hand_range_r


def arc_to_range_left_10(hand_arc_l):
    hand_range_l = [0] * 10
    for i in range(10):
        val_l = is_within_range(hand_arc_l[i], l10_l_min[i], l10_l_max[i])
        if l10_l_derict[i] == -1:
            hand_range_l[i] = scale_value(val_l, l10_l_min[i], l10_l_max[i], 255, 0)
        else:
            hand_range_l[i] = scale_value(val_l, l10_l_min[i], l10_l_max[i], 0, 255)

    return hand_range_l

                                 # -0.9   0      255   0  对于这个val为0是，转换的也是0
def scale_value(original_value, a_min, a_max, b_min, b_max):
    return (original_value - a_min) * (b_max - b_min) / (a_max - a_min) + b_min


def is_within_range(value, min_value, max_value):
    return min(max_value, max(min_value, value))
