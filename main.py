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

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

joint_configs = [
    {
        "name": "shoulder_lift",
        "id": 2,
    },
    {
        "name": "elbow_flex",
        "id": 3,
    }
]

# Init variables
chain = [100, 100]
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
slider_positions = [110, 225]
dragging_slider = None
prefer_positive_offset = True
current_shoulder_angle_deg = 0.0
current_elbow_angle_deg = 0.0

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

    base_y = 170
    spacing = max(130, (window_height - 320) // 2) if (window_height - 320) > 0 else 130
    slider_positions = [base_y + index * spacing for index in range(len(chain))]


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

    toggle_rect = pygame.Rect(canvas_width + 18, 54, sidebar_width - 36, 34)
    pygame.draw.rect(window, (42, 46, 61), toggle_rect, border_radius=10)
    pygame.draw.rect(window, (74, 81, 102), toggle_rect, 2, border_radius=10)

    toggle_label = small_font.render("Offset branch", True, (207, 212, 223))
    toggle_value = small_font.render(
        "+ angle" if prefer_positive_offset else "- angle",
        True,
        (15, 153, 113) if prefer_positive_offset else (255, 122, 80),
    )
    window.blit(toggle_label, (canvas_width + 30, 62))
    window.blit(toggle_value, (canvas_width + sidebar_width - 102, 62))

    toggle_knob_x = canvas_width + sidebar_width - 48 if prefer_positive_offset else canvas_width + sidebar_width - 84
    pygame.draw.circle(window, (245, 247, 250), (toggle_knob_x, 71), 8)

    # Show computed angles
    shoulder_text = small_font.render(f"Shoulder: {current_shoulder_angle_deg:.1f} deg", True, (242, 242, 242))
    elbow_text = small_font.render(f"Elbow: {current_elbow_angle_deg:.1f} deg", True, (242, 242, 242))
    window.blit(shoulder_text, (canvas_width + 18, 96))
    window.blit(elbow_text, (canvas_width + 18, 114))

    for index, y in enumerate(slider_positions):
        joint = joint_configs[index]
        current_angle = vector_angle_degrees(vectors[index])
        
        label = small_font.render(joint["name"], True, (207, 212, 223))
        angle_text = small_font.render(f"Angle: {current_angle:.1f} deg", True, (242, 242, 242))
        
        window.blit(label, (canvas_width + 18, y - 36))
        window.blit(angle_text, (canvas_width + 18, y - 16))

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
        window.blit(hint, (canvas_width + 18, window_height - 110 + line_index * 18))

check_triangle_validity = None
get_intersections = None
find_side = None

def resolve_ik(chain, vectors, end_effector, maximal_distance, pole):
    a = chain[0]
    b = chain[1]

    x = end_effector.x
    y = end_effector.y

    c = math.sqrt(x**2 + y**2)

    if c > a + b:
        c = a + b
        end_effector = end_effector.normalized() * c
        x = end_effector.x
        y = end_effector.y
    elif c < abs(a - b):
        c = abs(a - b)
        if c > 0:
            end_effector = end_effector.normalized() * c
        x = end_effector.x
        y = end_effector.y

    if c == 0:
        return [Vector2D(a, 0), Vector2D(b, 0)]

    val1 = (c**2 + a**2 - b**2) / (2 * a * c)
    val1 = max(-1.0, min(1.0, val1))

    angle_offset = math.acos(val1)
    base_angle = math.atan2(y, x)

    theta1_a = base_angle + angle_offset
    theta1_b = base_angle - angle_offset

    elbow_a = Vector2D(a * math.cos(theta1_a), a * math.sin(theta1_a))
    elbow_b = Vector2D(a * math.cos(theta1_b), a * math.sin(theta1_b))

    if prefer_positive_offset:
        shoulder_theta = theta1_a
        elbow = elbow_a
    else:
        shoulder_theta = theta1_b
        elbow = elbow_b

    # Compute explicit elbow angle (theta2) using law of cosines
    # theta2 = arccos((a^2 + b^2 - c^2) / (2ab))
    try:
        denom = 2.0 * a * b
        if denom == 0:
            theta2 = 0.0
        else:
            v = (a * a + b * b - c * c) / denom
            v = max(-1.0, min(1.0, v))
            theta2 = math.acos(v)
    except Exception:
        theta2 = 0.0

    # Update global readable angles (degrees)
    global current_shoulder_angle_deg, current_elbow_angle_deg
    current_shoulder_angle_deg = math.degrees(shoulder_theta) if shoulder_theta is not None else 0.0
    current_elbow_angle_deg = math.degrees(theta2)

    return [elbow, end_effector - elbow]

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
            toggle_rect = pygame.Rect(canvas_width + 18, 52, sidebar_width - 36, 42)
            if toggle_rect.collidepoint(mouse_x, mouse_y):
                prefer_positive_offset = not prefer_positive_offset
                if end_effector.length() > 0:
                    vectors = resolve_ik(chain, vectors, end_effector, maximal_distance, pole)
                continue
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
