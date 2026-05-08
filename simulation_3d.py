import math
import pygame

from angle_calculator import calculate_angles

pygame.init()

WINDOW_WIDTH = 1100
WINDOW_HEIGHT = 720
BACKGROUND = (18, 20, 30)
GRID_COLOR = (40, 44, 58)
AXIS_X_COLOR = (234, 95, 95)
AXIS_Y_COLOR = (95, 234, 160)
AXIS_Z_COLOR = (95, 155, 234)
ARM_COLOR = (245, 245, 245)
JOINT_COLOR = (74, 82, 104)
TARGET_COLOR = (255, 196, 0)
POLE_COLOR = (255, 130, 90)
ANGLE_BASE_COLOR = (170, 190, 220)
ANGLE_ARC_COLOR = (255, 210, 120)
ANGLE_TEXT_COLOR = (255, 230, 160)
TEXT_COLOR = (220, 225, 235)
PANEL_BG = (26, 30, 44)
PANEL_BORDER = (60, 68, 88)
SLIDER_TRACK = (74, 81, 102)
SLIDER_HANDLE = (15, 153, 113)
SLIDER_HANDLE_ACTIVE = (32, 191, 142)
INPUT_BG = (16, 18, 28)
INPUT_BORDER = (76, 84, 104)
INPUT_BORDER_ACTIVE = (255, 196, 0)
INPUT_BORDER_ERROR = (234, 95, 95)

DEFAULT_LINK_LENGTHS = [11.5, 13.5, 16.0]
DEFAULT_TARGET_RATIO = (0.5, 0.5, 0.22)
MIN_LINK_LENGTH = 2.0
MAX_LINK_LENGTH = 120.0
BASE_TOTAL_LENGTH = 180.0
BASE_AXIS_LENGTH = 80.0
BASE_GRID_SIZE = 320.0
BASE_GRID_SPACING = 20.0
BASE_ARM_WIDTH = 5
BASE_AXIS_WIDTH = 3
BASE_JOINT_RADIUS = 6.0
BASE_TARGET_RADIUS = 7.0
DASH_LENGTH = 6
DASH_GAP = 6

window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption("Inverse kinematics 3D")
font = pygame.font.SysFont("arial", 16)


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_scale(a, scale):
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def vec_length(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_normalize(a):
    length = vec_length(a)
    if length <= 1e-6:
        return (0.0, 0.0, 0.0)
    return vec_scale(a, 1.0 / length)


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def format_component(value):
    return f"{value:.2f}"


def parse_component(text):
    try:
        return float(text)
    except ValueError:
        return None


def rotate_z(point, angle):
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    x, y, z = point
    return (x * cos_a - y * sin_a, x * sin_a + y * cos_a, z)


def rotate_x(point, angle):
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    x, y, z = point
    return (x, y * cos_a - z * sin_a, y * sin_a + z * cos_a)


def world_to_camera(point, yaw, pitch, distance):
    # Rotate world opposite to camera movement, then push it forward.
    rotated = rotate_z(point, -yaw)
    rotated = rotate_x(rotated, -pitch)
    return (rotated[0], rotated[1] + distance, rotated[2])


def camera_to_world(point, yaw, pitch, distance):
    x, y, z = point
    shifted = (x, y - distance, z)
    rotated = rotate_x(shifted, pitch)
    return rotate_z(rotated, yaw)


def project(point, center, fov):
    x, y, z = point
    if y <= 1:
        return None
    scale = fov / y
    screen_x = center[0] + x * scale
    screen_y = center[1] - z * scale
    return (int(screen_x), int(screen_y)), scale


def draw_line_3d(surface, p1, p2, color, camera, center, fov, width=2):
    cam_p1 = world_to_camera(p1, camera["yaw"], camera["pitch"], camera["distance"])
    cam_p2 = world_to_camera(p2, camera["yaw"], camera["pitch"], camera["distance"])
    proj1 = project(cam_p1, center, fov)
    proj2 = project(cam_p2, center, fov)
    if proj1 is None or proj2 is None:
        return
    pygame.draw.line(surface, color, proj1[0], proj2[0], width)


def draw_dotted_line_3d(surface, p1, p2, color, camera, center, fov, width=1, dash=6, gap=6):
    direction = vec_sub(p2, p1)
    length = vec_length(direction)
    if length <= 0.0001:
        return
    if length <= dash:
        draw_line_3d(surface, p1, p2, color, camera, center, fov, width)
        return

    step = dash + gap
    unit = vec_scale(direction, 1.0 / length)
    traveled = 0.0

    while traveled < length:
        segment_start = traveled
        segment_end = min(traveled + dash, length)
        start_point = vec_add(p1, vec_scale(unit, segment_start))
        end_point = vec_add(p1, vec_scale(unit, segment_end))
        draw_line_3d(surface, start_point, end_point, color, camera, center, fov, width)
        traveled += step


def draw_point_3d(surface, point, color, camera, center, fov, radius=6):
    cam_point = world_to_camera(point, camera["yaw"], camera["pitch"], camera["distance"])
    projected = project(cam_point, center, fov)
    if projected is None:
        return
    position, scale = projected
    scaled_radius = max(2, int(radius * scale * 0.65))
    pygame.draw.circle(surface, color, position, scaled_radius)


def draw_text_3d(surface, text, point, color, camera, center, fov, offset=(6, -18)):
    cam_point = world_to_camera(point, camera["yaw"], camera["pitch"], camera["distance"])
    projected = project(cam_point, center, fov)
    if projected is None:
        return
    screen_pos, _ = projected
    render = font.render(text, True, color)
    surface.blit(render, (screen_pos[0] + offset[0], screen_pos[1] + offset[1]))


def screen_to_world(screen_pos, depth, camera, center, fov):
    x = (screen_pos[0] - center[0]) * depth / fov
    z = -(screen_pos[1] - center[1]) * depth / fov
    return camera_to_world((x, depth, z), camera["yaw"], camera["pitch"], camera["distance"])


def pick_drag_target(mouse_pos, target, pole, camera, center, fov, settings):
    candidates = []
    for label, point, radius in (
        ("target", target, settings["target_radius"]),
        ("pole", pole, settings["target_radius"] * 0.85),
    ):
        cam_point = world_to_camera(point, camera["yaw"], camera["pitch"], camera["distance"])
        projected = project(cam_point, center, fov)
        if projected is None:
            continue
        screen_pos, scale = projected
        hit_radius = max(6, int(radius * scale * 0.85))
        dist = math.hypot(mouse_pos[0] - screen_pos[0], mouse_pos[1] - screen_pos[1])
        if dist <= hit_radius:
            candidates.append((dist, label, cam_point[1], point))

    if not candidates:
        return None

    return min(candidates, key=lambda item: item[0])


class InputField:
    def __init__(self, label, rect, text="", owner=None, component=None):
        self.label = label
        self.rect = rect
        self.text = text
        self.owner = owner
        self.component = component
        self.active = False
        self.error = False

    def set_rect(self, rect):
        self.rect = rect

    def handle_text_input(self, text):
        if not self.active:
            return
        self.text += text
        self.error = False

    def handle_keydown(self, key):
        if not self.active:
            return None
        if key == pygame.K_BACKSPACE:
            self.text = self.text[:-1]
        elif key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            return "submit"
        return None

    def draw(self, surface):
        border_color = INPUT_BORDER_ACTIVE if self.active else INPUT_BORDER
        if self.error:
            border_color = INPUT_BORDER_ERROR
        pygame.draw.rect(surface, INPUT_BG, self.rect, border_radius=6)
        pygame.draw.rect(surface, border_color, self.rect, 2, border_radius=6)
        label = font.render(self.label, True, TEXT_COLOR)
        surface.blit(label, (self.rect.x, self.rect.y - 18))
        display_text = self.text if self.text else "x, y, z"
        render = font.render(display_text, True, TEXT_COLOR)
        surface.blit(render, (self.rect.x + 8, self.rect.y + 4))


def draw_grid(surface, size, spacing, camera, center, fov):
    half = int(round(size / 2.0))
    spacing = max(2, int(round(spacing)))
    for offset in range(-half, half + 1, spacing):
        p1 = (-half, offset, 0)
        p2 = (half, offset, 0)
        draw_line_3d(surface, p1, p2, GRID_COLOR, camera, center, fov, 1)
        p3 = (offset, -half, 0)
        p4 = (offset, half, 0)
        draw_line_3d(surface, p3, p4, GRID_COLOR, camera, center, fov, 1)


def compute_joint_positions(target, link_lengths):
    angles = calculate_angles(target[0], target[1], target[2], link_lengths)
    yaw = math.radians(angles["shoulder_pan"] - 90.0)
    pitch_angles = [
        math.radians(angles["shoulder_lift"]),
        math.radians(angles["elbow_flex"]),
        math.radians(angles["wrist_flex"]),
    ]

    positions = [(0.0, 0.0, 0.0)]
    current_pitch = 0.0
    for length, pitch in zip(link_lengths, pitch_angles):
        current_pitch += pitch
        forward = length * math.cos(current_pitch)
        up = length * math.sin(current_pitch)
        dx = forward * math.sin(yaw)
        dy = forward * math.cos(yaw)
        dz = up
        positions.append(vec_add(positions[-1], (dx, dy, dz)))

    return positions, angles


def pole_to_plane_coords(pole, target):
    extension = math.hypot(target[0], target[1])
    if extension <= 1e-6:
        return pole[1], pole[2]

    yaw = math.atan2(target[0], target[1])
    pole_extension = pole[0] * math.sin(yaw) + pole[1] * math.cos(yaw)
    return pole_extension, pole[2]


def compute_joint_positions_with_pole(target, link_lengths, pole, use_pole):
    if use_pole:
        pole_extension, pole_z = pole_to_plane_coords(pole, target)
        angles = calculate_angles(
            target[0],
            target[1],
            target[2],
            link_lengths,
            pole_y=pole_extension,
            pole_z=pole_z,
        )
    else:
        angles = calculate_angles(target[0], target[1], target[2], link_lengths)

    yaw = math.radians(angles["shoulder_pan"] - 90.0)
    pitch_angles = [
        math.radians(angles["shoulder_lift"]),
        math.radians(angles["elbow_flex"]),
        math.radians(angles["wrist_flex"]),
    ]

    positions = [(0.0, 0.0, 0.0)]
    current_pitch = 0.0
    for length, pitch in zip(link_lengths, pitch_angles):
        current_pitch += pitch
        forward = length * math.cos(current_pitch)
        up = length * math.sin(current_pitch)
        dx = forward * math.sin(yaw)
        dy = forward * math.cos(yaw)
        dz = up
        positions.append(vec_add(positions[-1], (dx, dy, dz)))

    return positions, angles


def default_target_for_lengths(link_lengths):
    total = sum(link_lengths)
    return [
        total * DEFAULT_TARGET_RATIO[0],
        total * DEFAULT_TARGET_RATIO[1],
        total * DEFAULT_TARGET_RATIO[2],
    ]


def default_camera_for_lengths(link_lengths):
    total = sum(link_lengths)
    return {
        "yaw": math.radians(45.0),
        "pitch": math.radians(25.0),
        "distance": max(60.0, total * 2.0),
    }


def default_pole_for_lengths(link_lengths):
    return [0.0, 10.0, 30.0]


def scene_settings(link_lengths):
    total = sum(link_lengths)
    scale = total / BASE_TOTAL_LENGTH if BASE_TOTAL_LENGTH > 0 else 1.0
    return {
        "axis_length": max(6.0, BASE_AXIS_LENGTH * scale),
        "grid_size": max(40.0, BASE_GRID_SIZE * scale),
        "grid_spacing": max(2.0, BASE_GRID_SPACING * scale),
        "arm_width": max(1, int(round(BASE_ARM_WIDTH * scale))),
        "axis_width": max(1, int(round(BASE_AXIS_WIDTH * scale))),
        "joint_radius": max(1.2, BASE_JOINT_RADIUS * scale),
        "target_radius": max(1.6, BASE_TARGET_RADIUS * scale),
    }


def slider_layout(window_size, count):
    width, height = window_size
    panel_width = min(320, max(220, int(width * 0.3)))
    panel_height = 28 + count * 46
    panel_x = 16
    panel_y = max(16, height - panel_height - 16)
    track_width = panel_width - 32
    track_height = 6

    tracks = []
    for index in range(count):
        track_y = panel_y + 28 + index * 46 + 18
        tracks.append(pygame.Rect(panel_x + 16, track_y, track_width, track_height))

    return pygame.Rect(panel_x, panel_y, panel_width, panel_height), tracks


def input_layout(window_size, panel_index, count):
    width, height = window_size
    panel_width = min(380, max(280, int(width * 0.34)))
    panel_height = 86
    panel_x = width - panel_width - 16
    panel_y = 16 + panel_index * (panel_height + 12)
    field_height = 28
    field_gap = 8
    field_width = (panel_width - 32 - field_gap * (count - 1)) // count

    fields = []
    for index in range(count):
        field_x = panel_x + 16 + index * (field_width + field_gap)
        field_y = panel_y + 34
        fields.append(pygame.Rect(field_x, field_y, field_width, field_height))

    return pygame.Rect(panel_x, panel_y, panel_width, panel_height), fields


def slider_value_to_x(value, track_rect):
    ratio = (value - MIN_LINK_LENGTH) / (MAX_LINK_LENGTH - MIN_LINK_LENGTH)
    ratio = clamp(ratio, 0.0, 1.0)
    return int(track_rect.x + ratio * track_rect.width)


def slider_x_to_value(mouse_x, track_rect):
    ratio = (mouse_x - track_rect.x) / track_rect.width
    ratio = clamp(ratio, 0.0, 1.0)
    return MIN_LINK_LENGTH + ratio * (MAX_LINK_LENGTH - MIN_LINK_LENGTH)


def clamp_target_to_reach(target, link_lengths, axis=None):
    total = sum(link_lengths)
    x, y, z = target

    def max_component(other1, other2):
        remaining = total * total - other1 * other1 - other2 * other2
        return math.sqrt(max(0.0, remaining))

    if axis == "x":
        limit = max_component(y, z)
        x = clamp(x, -limit, limit)
    elif axis == "y":
        limit = max_component(x, z)
        y = clamp(y, -limit, limit)
    elif axis == "z":
        limit = max_component(x, y)
        z = clamp(z, -limit, limit)
    else:
        distance = math.sqrt(x * x + y * y + z * z)
        if distance > total and distance > 0:
            scale = total / distance
            x *= scale
            y *= scale
            z *= scale

    return [x, y, z]


def draw_overlay(surface, target, pole, angles, link_lengths, use_pole, clock):
    lines = [
        "3D IK simulation",
        "Arrows: move target in X/Y",
        "Q/E: move target up/down",
        "Mouse drag: orbit camera",
        "Mouse wheel: zoom",
        "I/K: pole Y  J/L: pole X  U/O: pole Z",
        "Click inputs: type x, y, z",
        "P: toggle pole",
        "Drag sliders to change lengths",
        "R: reset target   C: reset camera",
        (
            "Links: "
            f"{link_lengths[0]:.1f}, {link_lengths[1]:.1f}, {link_lengths[2]:.1f}"
        ),
        f"Target: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})",
        f"Pole: ({pole[0]:.1f}, {pole[1]:.1f}, {pole[2]:.1f})  [{'on' if use_pole else 'off'}]",
        (
            "Angles: "
            f"pan {angles['shoulder_pan']:.1f}  "
            f"lift {angles['shoulder_lift']:.1f}  "
            f"elbow {angles['elbow_flex']:.1f}  "
            f"wrist {angles['wrist_flex']:.1f}"
        ),
        f"FPS: {clock.get_fps():.0f}",
    ]
    y = 12
    for text in lines:
        render = font.render(text, True, TEXT_COLOR)
        surface.blit(render, (12, y))
        y += 20


def draw_sliders(surface, link_lengths, active_index, window_size):
    panel_rect, tracks = slider_layout(window_size, len(link_lengths))

    pygame.draw.rect(surface, PANEL_BG, panel_rect, border_radius=10)
    pygame.draw.rect(surface, PANEL_BORDER, panel_rect, 1, border_radius=10)

    title = font.render("Link lengths", True, TEXT_COLOR)
    surface.blit(title, (panel_rect.x + 16, panel_rect.y + 8))

    for index, track in enumerate(tracks):
        label = font.render(f"Link {index + 1}: {link_lengths[index]:.1f}", True, TEXT_COLOR)
        surface.blit(label, (track.x, track.y - 20))

        pygame.draw.rect(surface, SLIDER_TRACK, track, border_radius=3)
        handle_x = slider_value_to_x(link_lengths[index], track)
        handle_color = SLIDER_HANDLE_ACTIVE if index == active_index else SLIDER_HANDLE
        handle_radius = 7
        pygame.draw.circle(surface, handle_color, (handle_x, track.centery), handle_radius)
        pygame.draw.circle(surface, TEXT_COLOR, (handle_x, track.centery), handle_radius, 2)


def draw_input_panel(surface, panels, window_size):
    for panel_index, (title_text, panel_fields) in enumerate(panels):
        panel_rect, field_rects = input_layout(window_size, panel_index, len(panel_fields))
        pygame.draw.rect(surface, PANEL_BG, panel_rect, border_radius=10)
        pygame.draw.rect(surface, PANEL_BORDER, panel_rect, 1, border_radius=10)
        title = font.render(title_text, True, TEXT_COLOR)
        surface.blit(title, (panel_rect.x + 16, panel_rect.y + 6))
        for field, rect in zip(panel_fields, field_rects):
            field.set_rect(rect)
            field.draw(surface)


def draw_axis_labels(surface, settings, camera, center, fov):
    axis_length = settings["axis_length"]
    labels = [
        ("X", (axis_length, 0, 0), AXIS_X_COLOR),
        ("Y", (0, axis_length, 0), AXIS_Y_COLOR),
        ("Z", (0, 0, axis_length), AXIS_Z_COLOR),
    ]
    for text, position, color in labels:
        cam_position = world_to_camera(position, camera["yaw"], camera["pitch"], camera["distance"])
        projected = project(cam_position, center, fov)
        if projected is None:
            continue
        screen_pos, _ = projected
        render = font.render(text, True, color)
        surface.blit(render, (screen_pos[0] + 6, screen_pos[1] + 6))


def draw_coordinate_guides(surface, target, camera, center, fov):
    x, y, z = target
    origin = (0.0, 0.0, 0.0)
    y_point = (0.0, y, 0.0)
    xy_point = (x, y, 0.0)
    xyz_point = (x, y, z)

    draw_dotted_line_3d(
        surface,
        origin,
        y_point,
        AXIS_Y_COLOR,
        camera,
        center,
        fov,
        width=1,
        dash=DASH_LENGTH,
        gap=DASH_GAP,
    )
    draw_dotted_line_3d(
        surface,
        y_point,
        xy_point,
        AXIS_X_COLOR,
        camera,
        center,
        fov,
        width=1,
        dash=DASH_LENGTH,
        gap=DASH_GAP,
    )
    draw_dotted_line_3d(
        surface,
        xy_point,
        xyz_point,
        AXIS_Z_COLOR,
        camera,
        center,
        fov,
        width=1,
        dash=DASH_LENGTH,
        gap=DASH_GAP,
    )

    label_data = [
        ("y", y, (0.0, y / 2.0, 0.0), AXIS_Y_COLOR, (6, -18)),
        ("x", x, (x / 2.0, y, 0.0), AXIS_X_COLOR, (6, -18)),
        ("z", z, (x, y, z / 2.0), AXIS_Z_COLOR, (6, -18)),
    ]

    for axis, value, position, color, offset in label_data:
        cam_position = world_to_camera(position, camera["yaw"], camera["pitch"], camera["distance"])
        projected = project(cam_position, center, fov)
        if projected is None:
            continue
        screen_pos, _ = projected
        render = font.render(f"{axis}={value:.1f}", True, color)
        surface.blit(render, (screen_pos[0] + offset[0], screen_pos[1] + offset[1]))

    cam_target = world_to_camera(xyz_point, camera["yaw"], camera["pitch"], camera["distance"])
    projected_target = project(cam_target, center, fov)
    if projected_target is not None:
        screen_pos, _ = projected_target
        render = font.render(f"({x:.1f}, {y:.1f}, {z:.1f})", True, TARGET_COLOR)
        surface.blit(render, (screen_pos[0] + 10, screen_pos[1] + 10))


def draw_pole(surface, pole, camera, center, fov, radius):
    draw_dotted_line_3d(
        surface,
        (0.0, 0.0, 0.0),
        (pole[0], pole[1], pole[2]),
        POLE_COLOR,
        camera,
        center,
        fov,
        width=1,
        dash=DASH_LENGTH,
        gap=DASH_GAP,
    )
    draw_point_3d(surface, (pole[0], pole[1], pole[2]), POLE_COLOR, camera, center, fov, radius)


def angle_in_plane(direction, forward, up):
    forward_component = vec_dot(direction, forward)
    up_component = vec_dot(direction, up)
    return math.atan2(up_component, forward_component)


def normalize_angle_delta(delta):
    return (delta + math.pi) % (2.0 * math.pi) - math.pi


def draw_arc_3d(surface, center, radius, start_angle, end_angle, forward, up, color, camera, screen_center, fov, width=2):
    delta = normalize_angle_delta(end_angle - start_angle)
    if abs(delta) < 1e-4:
        return

    steps = max(6, int(abs(delta) * 20))
    last_point = None
    for index in range(steps + 1):
        angle = start_angle + delta * (index / steps)
        offset = vec_add(vec_scale(forward, radius * math.cos(angle)), vec_scale(up, radius * math.sin(angle)))
        point = vec_add(center, offset)
        if last_point is not None:
            draw_line_3d(surface, last_point, point, color, camera, screen_center, fov, width)
        last_point = point


def draw_angle_guides(surface, target, joints, camera, center, fov, settings):
    x, y, _ = target
    planar = math.hypot(x, y)
    if planar > 1e-4:
        pan_radius = max(8.0, settings["axis_length"] * 0.22)
        pan_base = (0.0, 1.0, 0.0)
        pan_up = (1.0, 0.0, 0.0)
        pan_dir = (x / planar, y / planar, 0.0)

        draw_line_3d(
            surface,
            (0.0, 0.0, 0.0),
            vec_scale(pan_base, pan_radius * 1.15),
            ANGLE_BASE_COLOR,
            camera,
            center,
            fov,
            2,
        )
        draw_line_3d(
            surface,
            (0.0, 0.0, 0.0),
            vec_scale(pan_dir, pan_radius * 1.15),
            ANGLE_BASE_COLOR,
            camera,
            center,
            fov,
            2,
        )

        draw_arc_3d(
            surface,
            (0.0, 0.0, 0.0),
            pan_radius,
            0.0,
            math.atan2(x, y),
            pan_base,
            pan_up,
            ANGLE_ARC_COLOR,
            camera,
            center,
            fov,
            2,
        )
        mid_angle = math.atan2(x, y) * 0.5
        pan_label_point = vec_add(
            (0.0, 0.0, 0.0),
            vec_add(
                vec_scale(pan_base, pan_radius * 1.3 * math.cos(mid_angle)),
                vec_scale(pan_up, pan_radius * 1.3 * math.sin(mid_angle)),
            ),
        )
        draw_text_3d(surface, "pan", pan_label_point, ANGLE_TEXT_COLOR, camera, center, fov)

    if len(joints) < 2:
        return

    yaw = math.atan2(x, y) if planar > 1e-4 else 0.0
    plane_forward = (math.sin(yaw), math.cos(yaw), 0.0)
    plane_up = (0.0, 0.0, 1.0)

    link_dirs = []
    link_lengths = []
    for index in range(len(joints) - 1):
        direction = vec_sub(joints[index + 1], joints[index])
        link_len = vec_length(direction)
        link_dirs.append(vec_normalize(direction))
        link_lengths.append(link_len)

    pitch_labels = ["lift", "elbow", "wrist"]
    for joint_index in range(min(3, len(link_dirs))):
        joint_pos = joints[joint_index]
        if joint_index == 0:
            start_dir = plane_forward
        else:
            start_dir = link_dirs[joint_index - 1]
        end_dir = link_dirs[joint_index]

        if vec_length(start_dir) <= 1e-6 or vec_length(end_dir) <= 1e-6:
            continue

        start_angle = angle_in_plane(start_dir, plane_forward, plane_up)
        end_angle = angle_in_plane(end_dir, plane_forward, plane_up)
        delta = normalize_angle_delta(end_angle - start_angle)
        if abs(delta) < 1e-4:
            continue

        base_len = max(8.0, min(link_lengths[joint_index] * 0.45, settings["axis_length"] * 0.22))
        draw_line_3d(
            surface,
            joint_pos,
            vec_add(joint_pos, vec_scale(start_dir, base_len)),
            ANGLE_BASE_COLOR,
            camera,
            center,
            fov,
            2,
        )
        draw_line_3d(
            surface,
            joint_pos,
            vec_add(joint_pos, vec_scale(end_dir, base_len)),
            ANGLE_BASE_COLOR,
            camera,
            center,
            fov,
            2,
        )

        draw_arc_3d(
            surface,
            joint_pos,
            base_len * 0.85,
            start_angle,
            start_angle + delta,
            plane_forward,
            plane_up,
            ANGLE_ARC_COLOR,
            camera,
            center,
            fov,
            2,
        )

        mid_angle = start_angle + delta * 0.5
        label_offset = vec_add(
            vec_scale(plane_forward, base_len * 1.1 * math.cos(mid_angle)),
            vec_scale(plane_up, base_len * 1.1 * math.sin(mid_angle)),
        )
        label_point = vec_add(joint_pos, label_offset)
        draw_text_3d(surface, pitch_labels[joint_index], label_point, ANGLE_TEXT_COLOR, camera, center, fov)


def main():
    clock = pygame.time.Clock()
    running = True

    link_lengths = list(DEFAULT_LINK_LENGTHS)
    target = default_target_for_lengths(link_lengths)
    pole = default_pole_for_lengths(link_lengths)
    use_pole = True
    camera = default_camera_for_lengths(link_lengths)
    fov = 520.0

    target_fields = [
        InputField("X", pygame.Rect(0, 0, 0, 0), format_component(target[0]), owner="target", component=0),
        InputField("Y", pygame.Rect(0, 0, 0, 0), format_component(target[1]), owner="target", component=1),
        InputField("Z", pygame.Rect(0, 0, 0, 0), format_component(target[2]), owner="target", component=2),
    ]
    pole_fields = [
        InputField("X", pygame.Rect(0, 0, 0, 0), format_component(pole[0]), owner="pole", component=0),
        InputField("Y", pygame.Rect(0, 0, 0, 0), format_component(pole[1]), owner="pole", component=1),
        InputField("Z", pygame.Rect(0, 0, 0, 0), format_component(pole[2]), owner="pole", component=2),
    ]
    input_panels = [("Target", target_fields), ("Pole", pole_fields)]
    input_fields = target_fields + pole_fields
    text_input_active = False

    dragging = False
    dragging_slider = None
    dragging_target = False
    dragging_pole = False
    drag_depth = None
    drag_offset = (0.0, 0.0, 0.0)
    last_mouse = (0, 0)

    while running:
        dt = clock.tick(60) / 1000.0

        for panel_index, (_, panel_fields) in enumerate(input_panels):
            _, field_rects = input_layout(window.get_size(), panel_index, len(panel_fields))
            for field, rect in zip(panel_fields, field_rects):
                field.set_rect(rect)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                pygame.display.set_mode(event.size, pygame.RESIZABLE)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                hit_field = None
                for field in input_fields:
                    if field.rect.collidepoint(event.pos):
                        hit_field = field
                        break
                if hit_field is not None:
                    for field in input_fields:
                        field.active = field == hit_field
                        field.error = False
                    if not text_input_active:
                        pygame.key.start_text_input()
                        text_input_active = True
                    continue
                if any(field.active for field in input_fields):
                    for field in input_fields:
                        field.active = False
                    if text_input_active:
                        pygame.key.stop_text_input()
                        text_input_active = False
                panel_rect, tracks = slider_layout(window.get_size(), len(link_lengths))
                for index, track in enumerate(tracks):
                    hit_rect = pygame.Rect(track.x, track.y - 10, track.width, track.height + 20)
                    if hit_rect.collidepoint(event.pos):
                        dragging_slider = index
                        link_lengths[index] = slider_x_to_value(event.pos[0], track)
                        target = clamp_target_to_reach(target, link_lengths)
                        break
                else:
                    center = (window.get_width() // 2, window.get_height() // 2)
                    settings = scene_settings(link_lengths)
                    hit = pick_drag_target(event.pos, target, pole, camera, center, fov, settings)
                    if hit is not None and hit[2] > 1.0:
                        _, label, depth, point = hit
                        drag_depth = depth
                        drag_offset = vec_sub(point, screen_to_world(event.pos, depth, camera, center, fov))
                        dragging_target = label == "target"
                        dragging_pole = label == "pole"
                        dragging = False
                    else:
                        dragging = True
                        last_mouse = event.pos
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging = False
                dragging_slider = None
                dragging_target = False
                dragging_pole = False
            elif event.type == pygame.MOUSEMOTION:
                if dragging_slider is not None:
                    panel_rect, tracks = slider_layout(window.get_size(), len(link_lengths))
                    track = tracks[dragging_slider]
                    link_lengths[dragging_slider] = slider_x_to_value(event.pos[0], track)
                    target = clamp_target_to_reach(target, link_lengths)
                elif dragging_target or dragging_pole:
                    if drag_depth is None:
                        continue
                    center = (window.get_width() // 2, window.get_height() // 2)
                    new_point = screen_to_world(event.pos, drag_depth, camera, center, fov)
                    new_point = vec_add(new_point, drag_offset)
                    if dragging_target:
                        target = clamp_target_to_reach([new_point[0], new_point[1], new_point[2]], link_lengths)
                    else:
                        pole = [new_point[0], new_point[1], new_point[2]]
                elif dragging:
                    dx = event.pos[0] - last_mouse[0]
                    dy = event.pos[1] - last_mouse[1]
                    camera["yaw"] += dx * 0.005
                    camera["pitch"] = max(-1.2, min(1.2, camera["pitch"] + dy * 0.005))
                    last_mouse = event.pos
            elif event.type == pygame.MOUSEWHEEL:
                camera["distance"] = max(60.0, camera["distance"] - event.y * 20.0)
            elif event.type == pygame.KEYDOWN:
                active_field = next((field for field in input_fields if field.active), None)
                if active_field is not None:
                    if event.key == pygame.K_ESCAPE:
                        active_field.active = False
                        if text_input_active:
                            pygame.key.stop_text_input()
                            text_input_active = False
                        continue
                    result = active_field.handle_keydown(event.key)
                    if result == "submit":
                        parsed = parse_component(active_field.text)
                        if parsed is None:
                            active_field.error = True
                        else:
                            active_field.error = False
                            if active_field.owner == "target":
                                target[active_field.component] = parsed
                                target = clamp_target_to_reach(target, link_lengths)
                            else:
                                pole[active_field.component] = parsed
                    continue
                if event.key == pygame.K_r:
                    target = default_target_for_lengths(link_lengths)
                elif event.key == pygame.K_c:
                    camera = default_camera_for_lengths(link_lengths)
                elif event.key == pygame.K_p:
                    use_pole = not use_pole
            elif event.type == pygame.TEXTINPUT:
                active_field = next((field for field in input_fields if field.active), None)
                if active_field is not None:
                    active_field.handle_text_input(event.text)

        keys = pygame.key.get_pressed()
        if not any(field.active for field in input_fields):
            move_speed = max(12.0, sum(link_lengths) * 1.2)
            move_speed *= 2.0 if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT] else 1.0
            if keys[pygame.K_LEFT]:
                target[0] -= move_speed * dt
            if keys[pygame.K_RIGHT]:
                target[0] += move_speed * dt
            if keys[pygame.K_UP]:
                target[1] += move_speed * dt
            if keys[pygame.K_DOWN]:
                target[1] -= move_speed * dt
            if keys[pygame.K_q]:
                target[2] += move_speed * dt
            if keys[pygame.K_e]:
                target[2] -= move_speed * dt

            pole_speed = move_speed
            if keys[pygame.K_j]:
                pole[0] -= pole_speed * dt
            if keys[pygame.K_l]:
                pole[0] += pole_speed * dt
            if keys[pygame.K_i]:
                pole[1] += pole_speed * dt
            if keys[pygame.K_k]:
                pole[1] -= pole_speed * dt
            if keys[pygame.K_u]:
                pole[2] += pole_speed * dt
            if keys[pygame.K_o]:
                pole[2] -= pole_speed * dt

            moved_x = keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]
            moved_y = keys[pygame.K_UP] or keys[pygame.K_DOWN]
            moved_z = keys[pygame.K_q] or keys[pygame.K_e]
            if moved_x or moved_y or moved_z:
                active_axes = sum([moved_x, moved_y, moved_z])
                if active_axes == 1:
                    if moved_x:
                        target = clamp_target_to_reach(target, link_lengths, axis="x")
                    elif moved_y:
                        target = clamp_target_to_reach(target, link_lengths, axis="y")
                    else:
                        target = clamp_target_to_reach(target, link_lengths, axis="z")
                else:
                    target = clamp_target_to_reach(target, link_lengths)

        for field in target_fields:
            if not field.active:
                field.text = format_component(target[field.component])
        for field in pole_fields:
            if not field.active:
                field.text = format_component(pole[field.component])

        window.fill(BACKGROUND)
        center = (window.get_width() // 2, window.get_height() // 2)
        settings = scene_settings(link_lengths)

        draw_grid(window, size=settings["grid_size"], spacing=settings["grid_spacing"], camera=camera, center=center, fov=fov)
        draw_line_3d(
            window,
            (0, 0, 0),
            (settings["axis_length"], 0, 0),
            AXIS_X_COLOR,
            camera,
            center,
            fov,
            settings["axis_width"],
        )
        draw_line_3d(
            window,
            (0, 0, 0),
            (0, settings["axis_length"], 0),
            AXIS_Y_COLOR,
            camera,
            center,
            fov,
            settings["axis_width"],
        )
        draw_line_3d(
            window,
            (0, 0, 0),
            (0, 0, settings["axis_length"]),
            AXIS_Z_COLOR,
            camera,
            center,
            fov,
            settings["axis_width"],
        )
        draw_axis_labels(window, settings, camera, center, fov)
        draw_coordinate_guides(window, target, camera, center, fov)
        draw_pole(window, pole, camera, center, fov, settings["target_radius"] * 0.85)

        joints, angles = compute_joint_positions_with_pole(target, link_lengths, pole, use_pole)
        draw_angle_guides(window, target, joints, camera, center, fov, settings)
        for index in range(len(joints) - 1):
            draw_line_3d(
                window,
                joints[index],
                joints[index + 1],
                ARM_COLOR,
                camera,
                center,
                fov,
                settings["arm_width"],
            )

        for joint in joints:
            draw_point_3d(window, joint, JOINT_COLOR, camera, center, fov, settings["joint_radius"])

        draw_point_3d(
            window,
            (target[0], target[1], target[2]),
            TARGET_COLOR,
            camera,
            center,
            fov,
            settings["target_radius"],
        )
        draw_overlay(window, target, pole, angles, link_lengths, use_pole, clock)
        draw_sliders(window, link_lengths, dragging_slider, window.get_size())
        draw_input_panel(window, input_panels, window.get_size())

        pygame.display.update()

    pygame.quit()


if __name__ == "__main__":
    main()
