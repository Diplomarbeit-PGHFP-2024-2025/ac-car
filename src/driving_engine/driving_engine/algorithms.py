import math


def drive_to(current_position, current_direction, next_position, next_direction):
    current_normal = (current_direction[1], -current_direction[0])
    next_normal = (next_direction[1], -next_direction[0])

    cos_theta = dot_product(current_normal, next_normal) / (magnitude(current_normal) * magnitude(next_normal))

    cos_theta = max(-1, min(1, cos_theta))

    angle = math.acos(cos_theta)

    half_angle = angle/2

    r = 3

    d = math.tan(half_angle) * r

    print(math.degrees(angle))
    print(d)

    next_direction = (-next_direction[0], -next_direction[0])
    vector_difference = subtract(next_position, current_position)

    matrix = [
        [current_direction[0], -next_direction[0]],
        [current_direction[1], -next_direction[1]],
    ]

    determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]

    if determinant == 0:
        print("Cannot compute")
        return

    det_t = vector_difference[0] * matrix[1][1] - vector_difference[1] * matrix[0][1]
    t = det_t / determinant

    # Cramer's rule for s:
    det_s = matrix[0][0] * vector_difference[1] - matrix[1][0] * vector_difference[0]
    s = det_s / determinant

    print(t, s)

    point_1 = (current_position[0] + current_direction[0]*(t-d), current_position[1] + current_direction[1]*(t-d))
    point_2 = (next_position[0] + next_direction[0]*(s-d), next_position[1] + next_direction[1]*(s-d))

    print(point_1, point_2)

def dot_product(a, b):
    return a[0] * b[0] + a[1] * b[1]

def magnitude(a):
    return math.sqrt(a[0] ** 2 + a[1] ** 2)

def subtract(a, b):
    return a[0] - b[0], a[1] - b[1]