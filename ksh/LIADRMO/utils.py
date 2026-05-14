import math

def clamp(x, low, high):

    return max(low, min(high, x))

def rad2deg(x):

    return x * 180.0 / math.pi

def deg2rad(x):

    return x * math.pi / 180.0