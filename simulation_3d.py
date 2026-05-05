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
TEXT_COLOR = (220, 225, 235)
PANEL_BG = (26, 30, 44)
PANEL_BORDER = (60, 68, 88)
SLIDER_TRACK = (74, 81, 102)
SLIDER_HANDLE = (15, 153, 113)
SLIDER_HANDLE_ACTIVE = (32, 191, 142)

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


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


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
    for length, pitch in zip(link_lengths, pitch_angles):
        forward = length * math.cos(pitch)
        up = length * math.sin(pitch)
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
    for length, pitch in zip(link_lengths, pitch_angles):
        forward = length * math.cos(pitch)
        up = length * math.sin(pitch)
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
    total = sum(link_lengths)
    return [0.0, total * 0.7, total * 0.45]


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


def main():
    clock = pygame.time.Clock()
    running = True

    link_lengths = list(DEFAULT_LINK_LENGTHS)
    target = default_target_for_lengths(link_lengths)
    pole = default_pole_for_lengths(link_lengths)
    use_pole = True
    camera = default_camera_for_lengths(link_lengths)
    fov = 520.0

    dragging = False
    dragging_slider = None
    last_mouse = (0, 0)

    while running:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                pygame.display.set_mode(event.size, pygame.RESIZABLE)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                panel_rect, tracks = slider_layout(window.get_size(), len(link_lengths))
                for index, track in enumerate(tracks):
                    hit_rect = pygame.Rect(track.x, track.y - 10, track.width, track.height + 20)
                    if hit_rect.collidepoint(event.pos):
                        dragging_slider = index
                        link_lengths[index] = slider_x_to_value(event.pos[0], track)
                        target = clamp_target_to_reach(target, link_lengths)
                        break
                else:
                    dragging = True
                    last_mouse = event.pos
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging = False
                dragging_slider = None
            elif event.type == pygame.MOUSEMOTION:
                if dragging_slider is not None:
                    panel_rect, tracks = slider_layout(window.get_size(), len(link_lengths))
                    track = tracks[dragging_slider]
                    link_lengths[dragging_slider] = slider_x_to_value(event.pos[0], track)
                    target = clamp_target_to_reach(target, link_lengths)
                elif dragging:
                    dx = event.pos[0] - last_mouse[0]
                    dy = event.pos[1] - last_mouse[1]
                    camera["yaw"] += dx * 0.005
                    camera["pitch"] = max(-1.2, min(1.2, camera["pitch"] + dy * 0.005))
                    last_mouse = event.pos
            elif event.type == pygame.MOUSEWHEEL:
                camera["distance"] = max(60.0, camera["distance"] - event.y * 20.0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    target = default_target_for_lengths(link_lengths)
                elif event.key == pygame.K_c:
                    camera = default_camera_for_lengths(link_lengths)
                elif event.key == pygame.K_p:
                    use_pole = not use_pole

        keys = pygame.key.get_pressed()
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

        pygame.display.update()

    pygame.quit()


if __name__ == "__main__":
    main()
