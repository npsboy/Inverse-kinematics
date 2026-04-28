import pygame
import math
import numpy
from vector import Vector2D

pygame.init()

#Init window
window_width = 320*3
window_height = 180*3
sidebar_width = 320
canvas_width = window_width - sidebar_width
window = pygame.display.set_mode((window_width, window_height), pygame.RESIZABLE)
pygame.display.set_caption("Inverse kinematics")
font = pygame.font.SysFont("arial", 18)
small_font = pygame.font.SysFont("arial", 15)

def degrees_to_ticks(degrees: float, min_range: float, max_range: float) -> int:
	return int(round(degrees * 10 + min_range))

def ticks_to_degrees(ticks: int, min_range: float, max_range: float) -> float:
    return (ticks - min_range) / 10.0

joint_configs = [
    {
        "name": "shoulder_lift",
        "id": 2,
        "drive_mode": 0,
        "homing_offset": 1022,
        "range_min": 1457,
        "range_max": 3815,
    },
    {
        "name": "elbow_flex",
        "id": 3,
        "drive_mode": 0,
        "homing_offset": 1288,
        "range_min": 536,
        "range_max": 2749,
    },
    {
        "name": "wrist_flex",
        "id": 4,
        "drive_mode": 0,
        "homing_offset": -1112,
        "range_min": 313,
        "range_max": 2632,
    },
]

for joint in joint_configs:
    joint["degree_min"] = 0.0
    joint["degree_max"] = ticks_to_degrees(joint["range_max"], joint["range_min"], joint["range_max"])

# Init variables
chain = [50, 70, 60]
vectors = [Vector2D(length, 0) for length in chain]
end_effector = Vector2D(0, 0)
pole = Vector2D(0, 0)
maximal_distance = sum(chain)

screen_middle_position = Vector2D(canvas_width/2, window_height/2)
pole_global = Vector2D(0, 0)
end_effector_global = Vector2D(0, 0)
slider_min = 20
slider_max = 180
slider_track_x = canvas_width + 20
slider_track_width = sidebar_width - 40
slider_positions = [110, 225, 340]
dragging_slider = None

def update_layout(size):
    global window_width, window_height, sidebar_width, canvas_width, screen_middle_position
    global slider_track_x, slider_track_width, slider_positions

    window_width, window_height = size
    sidebar_width = max(300, min(420, int(window_width * 0.34)))
    if sidebar_width >= window_width - 200:
        sidebar_width = max(260, window_width - 200)

    canvas_width = window_width - sidebar_width
    screen_middle_position = Vector2D(canvas_width / 2, window_height / 2)
    slider_track_x = canvas_width + 24
    slider_track_width = max(160, sidebar_width - 48)

    base_y = 120
    spacing = max(102, (window_height - 240) // 3)
    slider_positions = [base_y + index * spacing for index in range(3)]

def rebuild_chain_vectors(chain):
    return [Vector2D(length, 0) for length in chain]

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def value_from_slider(mouse_x):
    ratio = (mouse_x - slider_track_x) / slider_track_width
    return int(round(slider_min + clamp(ratio, 0, 1) * (slider_max - slider_min)))

def slider_handle_x(value):
    ratio = (value - slider_min) / (slider_max - slider_min)
    return slider_track_x + ratio * slider_track_width

def vector_angle_degrees(vector):
    if vector.length() == 0:
        return 0.0

    return math.degrees(vector.get_angle()) % 360.0

def update_chain_from_slider(index, mouse_x):
    global vectors, maximal_distance, end_effector
    chain[index] = value_from_slider(mouse_x)
    vectors = rebuild_chain_vectors(chain)
    maximal_distance = sum(chain)
    if end_effector.length() > 0:
        vectors = resolve_ik(chain, vectors, end_effector, maximal_distance, pole)

def draw_sidebar(window):
    panel_rect = pygame.Rect(canvas_width, 0, sidebar_width, window_height)
    pygame.draw.rect(window, (29, 32, 44), panel_rect)
    pygame.draw.line(window, (74, 81, 102), (canvas_width, 0), (canvas_width, window_height), 2)

    title = font.render("Joint limits", True, (245, 247, 250))
    window.blit(title, (canvas_width + 18, 20))

    for index, y in enumerate(slider_positions):
        joint = joint_configs[index]
        current_angle = vector_angle_degrees(vectors[index])
        current_ticks = degrees_to_ticks(current_angle, joint["range_min"], joint["range_max"])
        label = small_font.render(joint["name"], True, (207, 212, 223))
        range_text = small_font.render(
            f"Range: {joint['degree_min']:.1f} - {joint['degree_max']:.1f} deg",
            True,
            (168, 174, 189),
        )
        angle_text = small_font.render(f"Angle: {current_angle:.1f} deg", True, (242, 242, 242))
        tick_text = small_font.render(f"Ticks: {current_ticks}", True, (168, 174, 189))
        window.blit(label, (canvas_width + 18, y - 62))
        window.blit(range_text, (canvas_width + 18, y - 42))
        window.blit(angle_text, (canvas_width + 18, y - 22))
        window.blit(tick_text, (canvas_width + 18, y - 2))

        track_rect = pygame.Rect(slider_track_x, y + 26, slider_track_width, 6)
        pygame.draw.rect(window, (74, 81, 102), track_rect, border_radius=3)

        handle_x = int(slider_handle_x(chain[index]))
        pygame.draw.circle(window, (15, 153, 113), (handle_x, y + 29), 10)
        pygame.draw.circle(window, (245, 247, 250), (handle_x, y + 29), 10, 2)

    help_text = [
        "Drag sliders to change",
        "each segment length.",
        "Left mouse: target",
        "Right mouse: pole",
    ]
    for line_index, text in enumerate(help_text):
        hint = small_font.render(text, True, (168, 174, 189))
        window.blit(hint, (canvas_width + 18, window_height - 102 + line_index * 18))

def check_triangle_validity(a, b, c): 
    return a+b>=c or a+c>=b or b+c>= a or math.isclose(a + b, c) or math.isclose(a + c, b) or math.isclose(b + c, a)

def get_intersections(position1, radius1, position2, radius2):
    # Calculate distance
    distance_vector = position2-position1
    distance = distance_vector.length()

    if math.isclose(distance, 0):
        fallback_offset = Vector2D(radius1, 0)
        return position1 + fallback_offset, position1 - fallback_offset

    # Distance = a + b
    a = max((radius1**2 - radius2**2 + distance**2)/(2*distance), 0)

    # Calculate height
    height = math.sqrt(max(radius1**2 - a**2, 0))
    height_vector = Vector2D(-height * distance_vector.sin(), height * distance_vector.cos())

    # Calculate intersections' position
    point = position1 + distance_vector.normalized()*a
    intersection1 = point + height_vector
    intersection2 = point - height_vector

    '''position1_global = Vector2D(position1.x, -position1.y) + screen_middle_position
    position2_global = Vector2D(position2.x, -position2.y) + screen_middle_position
    pygame.draw.circle(window, (255, 255, 255), (int(position1_global.x), int(position1_global.y)), int(radius1), 1)
    pygame.draw.circle(window, (255, 255, 255), (int(position2_global.x), int(position2_global.y)), int(radius2), 1)'''

    return intersection1, intersection2

def find_side(minimal_length, maximal_length, side1, side2):
    for side in numpy.arange(maximal_length, minimal_length, -0.5):
        # print(side, side1, side2, check_triangle_validity(side, side1, side2))
        if check_triangle_validity(side, side1, side2):
            return side
    return 0

def resolve_ik(chain, vectors, end_effector, maximal_distance, pole):
    # Init variables
    new_vectors = []
    # Calculate current side
    if end_effector.length() > maximal_distance:
        end_effector = end_effector.normalized() * maximal_distance
    current_side_vector = end_effector

    for i in range(chain.__len__()-1, 0, -1):
        current_side = current_side_vector.length()
        # Find possible side
        new_side = find_side(0, sum(chain[:i]), chain[i], current_side)
        # Find intersection nearest to the pole
        intersections = []
        if i != 1:
            intersections = get_intersections(current_side_vector, chain[i], Vector2D(0, 0), new_side)
        else:
            intersections = get_intersections(current_side_vector, chain[i], Vector2D(0, 0), chain[0])

        intersection = Vector2D(0, 0)
        if intersections[0]-pole < intersections[1]-pole:
            intersection = intersections[0]
        else:
            intersection = intersections[1]
        # Change vector
        new_vectors.insert(0, current_side_vector - intersection)

        # print(new_vectors.__len__())
        current_side_vector = intersection
        print(new_side)

    new_vectors.insert(0, current_side_vector)

    return new_vectors

def draw_vectors_chain(window, position, chain, color, width=1, draw_circles=False, radius=1, circle_color=(255, 255, 255)):
        for vector in chain:
            new_vector = Vector2D(vector.x + position.x, -vector.y + position.y)
            pygame.draw.line(window, color, (position.x, position.y), (new_vector.x, new_vector.y), width)  
            if draw_circles:
                pygame.draw.circle(window, circle_color, (position.x, position.y), radius)
                pygame.draw.circle(window, circle_color, (new_vector.x, new_vector.y), radius)

            position = new_vector

# Main loop
fps = 60
clock = pygame.time.Clock()
run = True

update_layout(window.get_size())

while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        elif event.type == pygame.VIDEORESIZE:
            window = pygame.display.set_mode(event.size, pygame.RESIZABLE)
            update_layout(event.size)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mouse_x, mouse_y = event.pos
            for index, slider_y in enumerate(slider_positions):
                slider_rect = pygame.Rect(slider_track_x, slider_y + 12, slider_track_width, 28)
                if slider_rect.collidepoint(mouse_x, mouse_y):
                    dragging_slider = index
                    update_chain_from_slider(index, mouse_x)
                    break
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            dragging_slider = None
        elif event.type == pygame.MOUSEMOTION and dragging_slider is not None:
            update_chain_from_slider(dragging_slider, event.pos[0])

    window.fill((56, 128, 235))

    mouse_x, mouse_y = pygame.mouse.get_pos()

    if pygame.mouse.get_pressed()[0] and mouse_x < canvas_width and dragging_slider is None:
        end_effector_global = Vector2D(mouse_x, mouse_y)
        end_effector = end_effector_global - screen_middle_position
        end_effector.y *= -1
        vectors = resolve_ik(chain, vectors, end_effector, maximal_distance, pole)
        
    if pygame.mouse.get_pressed()[2] and mouse_x < canvas_width:
        pole_global = Vector2D(mouse_x, mouse_y)
        pole = pole_global - screen_middle_position
        pole.y *= -1

    # draw_vectors_chain(window, screen_middle_position, [end_effector.normalized() * maximal_distance], (15, 153, 113), width=4)

    # Draw pole and end effector
    pygame.draw.circle(window, (0, 242, 255), (int(pole_global.x), int(pole_global.y)), 5)
    pygame.draw.circle(window, (15, 153, 113), (int(end_effector_global.x), int(end_effector_global.y)), 5)

    draw_vectors_chain(window, screen_middle_position, vectors, (255, 255, 255), width=7, draw_circles=True, circle_color=(55, 59, 68), radius=5)
    # draw_vectors_chain(window, screen_middle_position, vectors, (255, 255, 255))
    draw_sidebar(window)

    delta_time = clock.tick(fps)

    pygame.display.update()
