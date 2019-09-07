from math import pi

def YawLimit(yaw):
    '''
    Restrict Yaw Error within [-pi, pi]
    '''
    while yaw > pi or yaw < -pi:
        if yaw > pi:
            yaw -= 2*pi
        if yaw < -pi:
            yaw += 2*pi
    return yaw