import math
import numpy
from vector import Vector2D

def check_triangle_validity(a, b, c): 
    return (a + b >= c) or (a + c >= b) or (b + c >= a) or \
           math.isclose(a + b, c) or math.isclose(a + c, b) or math.isclose(b + c, a)

def get_intersections(position1, radius1, position2, radius2):
    # Calculate distance
    distance_vector = position2 - position1
    distance = distance_vector.length()

    if math.isclose(distance, 0):
        fallback_offset = Vector2D(radius1, 0)
        return position1 + fallback_offset, position1 - fallback_offset

    # Distance = a + b
    a = max((radius1**2 - radius2**2 + distance**2) / (2 * distance), 0)

    # Calculate height
    height = math.sqrt(max(radius1**2 - a**2, 0))
    height_vector = Vector2D(-height * distance_vector.sin(), height * distance_vector.cos())

    # Calculate intersections' position
    point = position1 + distance_vector.normalized() * a
    intersection1 = point + height_vector
    intersection2 = point - height_vector

    return intersection1, intersection2

def find_side(minimal_length, maximal_length, side1, side2):
    for side in numpy.arange(maximal_length, minimal_length, -0.5):
        if check_triangle_validity(side, side1, side2):
            return side
    return 0

def resolve_ik(chain, end_effector, pole=None):
    if pole is None:
        pole = Vector2D(0, 0)

    maximal_distance = sum(chain)
    
    # Init variables
    new_vectors = []
    
    # Calculate current side
    if end_effector.length() > maximal_distance:
        end_effector = end_effector.normalized() * maximal_distance
        
    current_side_vector = end_effector

    for i in range(len(chain) - 1, 0, -1):
        current_side = current_side_vector.length()
        
        # Find possible side
        new_side = find_side(0, sum(chain[:i]), chain[i], current_side)
        
        # Find intersection nearest to the pole
        if i != 1:
            intersections = get_intersections(current_side_vector, chain[i], Vector2D(0, 0), new_side)
        else:
            intersections = get_intersections(current_side_vector, chain[i], Vector2D(0, 0), chain[0])

        intersection = Vector2D(0, 0)
        # Vector2D < operator is overloaded to compare lengths
        if (intersections[0] - pole) < (intersections[1] - pole):
            intersection = intersections[0]
        else:
            intersection = intersections[1]
            
        # Change vector
        new_vectors.insert(0, current_side_vector - intersection)
        current_side_vector = intersection

    new_vectors.insert(0, current_side_vector)

    return new_vectors

def vector_angle_degrees(vector):
    if vector.length() == 0:
        return 0.0

    return math.degrees(vector.get_angle()) % 360.0

def calculate_angles(x, y, link_lengths, pole_x=0, pole_y=0):
    """
    Calculate inverse kinematics for an n-link robotic arm.
    
    :param x: Target X coordinate for the end effector.
    :param y: Target Y coordinate for the end effector.
    :param link_lengths: List specifying the lengths of each link in the chain.
    :param pole_x: Optional X coordinate for the pole (used to choose joint orientations).
    :param pole_y: Optional Y coordinate for the pole.
    :return: List of joint angles in degrees.
    """
    end_effector = Vector2D(x, y)
    pole = Vector2D(pole_x, pole_y)
    
    vectors = resolve_ik(link_lengths, end_effector, pole)
    
    angles = [vector_angle_degrees(vec) for vec in vectors]
    
    return angles

if __name__ == "__main__":
    # Example usage:
    # Links matching the default lengths in main.py
    links = [50, 70, 60]
    target_x = 100
    target_y = 50
    
    out_angles = calculate_angles(target_x, target_y, links)
    print(f"Target: ({target_x}, {target_y})")
    print(f"Lengths: {links}")
    print(f"Calculated Angles (degrees): {out_angles}")
