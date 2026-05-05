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
TEXT_COLOR = (220, 225, 235)

LINK_LENGTHS = [50, 70, 60]

window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption("Inverse kinematics 3D")
font = pygame.font.SysFont("arial", 16)


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


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


def draw_point_3d(surface, point, color, camera, center, fov, radius=6):
    cam_point = world_to_camera(point, camera["yaw"], camera["pitch"], camera["distance"])
    projected = project(cam_point, center, fov)
    if projected is None:
        return
    position, scale = projected
    scaled_radius = max(2, int(radius * scale * 0.65))
    pygame.draw.circle(surface, color, position, scaled_radius)


def draw_grid(surface, size, spacing, camera, center, fov):
    half = size // 2
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


def draw_overlay(surface, target, angles, clock):
    lines = [
        "3D IK simulation",
        "Arrows: move target in X/Y",
        "Q/E: move target up/down",
        "Mouse drag: orbit camera",
        "Mouse wheel: zoom",
        "R: reset target   C: reset camera",
        f"Target: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})",
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


def main():
    clock = pygame.time.Clock()
    running = True

    target = [90.0, 90.0, 40.0]
    camera = {"yaw": math.radians(45.0), "pitch": math.radians(25.0), "distance": 360.0}
    fov = 520.0

    dragging = False
    last_mouse = (0, 0)

    while running:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                pygame.display.set_mode(event.size, pygame.RESIZABLE)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                dragging = True
                last_mouse = event.pos
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging = False
            elif event.type == pygame.MOUSEMOTION and dragging:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                camera["yaw"] += dx * 0.005
                camera["pitch"] = max(-1.2, min(1.2, camera["pitch"] + dy * 0.005))
                last_mouse = event.pos
            elif event.type == pygame.MOUSEWHEEL:
                camera["distance"] = max(120.0, camera["distance"] - event.y * 20.0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    target = [90.0, 90.0, 40.0]
                elif event.key == pygame.K_c:
                    camera = {"yaw": math.radians(45.0), "pitch": math.radians(25.0), "distance": 360.0}

        keys = pygame.key.get_pressed()
        move_speed = 120.0 * (2.0 if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT] else 1.0)
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

        window.fill(BACKGROUND)
        center = (window.get_width() // 2, window.get_height() // 2)

        draw_grid(window, size=320, spacing=20, camera=camera, center=center, fov=fov)
        draw_line_3d(window, (0, 0, 0), (80, 0, 0), AXIS_X_COLOR, camera, center, fov, 3)
        draw_line_3d(window, (0, 0, 0), (0, 80, 0), AXIS_Y_COLOR, camera, center, fov, 3)
        draw_line_3d(window, (0, 0, 0), (0, 0, 80), AXIS_Z_COLOR, camera, center, fov, 3)

        joints, angles = compute_joint_positions(target, LINK_LENGTHS)
        for index in range(len(joints) - 1):
            draw_line_3d(window, joints[index], joints[index + 1], ARM_COLOR, camera, center, fov, 5)

        for joint in joints:
            draw_point_3d(window, joint, JOINT_COLOR, camera, center, fov, 6)

        draw_point_3d(window, (target[0], target[1], target[2]), TARGET_COLOR, camera, center, fov, 7)
        draw_overlay(window, target, angles, clock)

        pygame.display.update()

    pygame.quit()


if __name__ == "__main__":
    main()
