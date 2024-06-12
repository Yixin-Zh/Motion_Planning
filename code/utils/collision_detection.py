import numpy as np
def line_segment_aabb(p0, p1, b):

    b_min = b[:3]
    b_max = b[3:]

    # Calculate box center-point
    c = (b_min + b_max) * 0.5
    # Calculate box half-length extents
    e = b_max - c
    # Calculate segment midpoint
    m = (p0 + p1) * 0.5
    # Calculate segment half-length vector
    d = p1 - m
    # Translate box and segment to origin
    m = m - c

    # Absolute values of the half-length vector
    adx, ady, adz = np.abs(d)
    # Adding epsilon to counteract arithmetic errors when segment is (near) parallel to a coordinate axis
    EPSILON = 1e-12
    adx += EPSILON
    ady += EPSILON
    adz += EPSILON

    # Check along coordinate axes
    if np.abs(m[0]) > e[0] + adx:
        return False
    if np.abs(m[1]) > e[1] + ady:
        return False
    if np.abs(m[2]) > e[2] + adz:
        return False

    # Check cross products of segment direction vector with coordinate axes
    if np.abs(m[1] * d[2] - m[2] * d[1]) > e[1] * adz + e[2] * ady:
        return False
    if np.abs(m[2] * d[0] - m[0] * d[2]) > e[0] * adz + e[2] * adx:
        return False
    if np.abs(m[0] * d[1] - m[1] * d[0]) > e[0] * ady + e[1] * adx:
        return False

    # No separating axis found; segment must be overlapping AABB
    return True



def collisionDetection(path, blocks):
    for i in range(len(path)-1):
        point1 = path[i]
        point2 = path[i+1]
        for j in range(blocks.shape[0]):
            block = blocks[j]
            if line_segment_aabb(point1, point2, block):
                return True
    return False

if __name__ == '__main__':

    # Test the collision detection function
    point1 = np.array([1.0, 1.0, 1.0])
    point2 = np.array([3.0, 3.0, 3.0])
    block = np.array([2.0, 2.0, 2.0, 4.0, 4.0, 4.0])
    print(line_segment_aabb(point1, point2, block))  # Expected output: True

    point1 = np.array([1.0, 1.0, 1.0])
    point2 = np.array([3.0, 3.0, 3.0])
    block = np.array([4.0, 4.0, 4.0, 6.0, 6.0, 6.0])
    print(line_segment_aabb(point1, point2, block))  # Expected output: False

    point1 = np.array([1.0, 1.0, 1.0])
    point2 = np.array([3.0, 3.0, 3.0])
    block = np.array([0.0, 0.0, 0.0, 2.0, 2.0, 2.0])
    print(line_segment_aabb(point1, point2, block))  # Expected output: False

    point1 = np.array([1.0, 1.0, 1.0])
    point2 = np.array([3.0, 3.0, 3.0])
    block = np.array([0.0, 0.0, 0.0, 4.0, 4.0, 4.0])
    print(line_segment_aabb(point1, point2, block))  # Expected output: True
