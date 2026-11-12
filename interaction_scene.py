"""Two rooms, a motorized door, and a free inspection camera (iGibson 2.x).

This is a scene inspector, not a navigation episode. The motor represents an
automatic door; Turtlebot does not manipulate a handle. No downloads are made.
"""

import argparse
import math
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parent
WINDOW = "Two rooms - free camera"


def build_room_urdf():
    """8 x 6 m interior, 2.5 m walls, 1.4 m doorway; floor surface at z=0.

    Door hinge is (-0.67, 0, 0.03), opening toward room B (positive Y).
    A massless root anchors the structure without an external fixed constraint.
    """
    root = ET.Element("robot", name="two_rooms")
    base = ET.SubElement(root, "link", name="building")
    inertial = ET.SubElement(base, "inertial")
    ET.SubElement(inertial, "mass", value="0")
    ET.SubElement(inertial, "inertia", ixx="0", iyy="0", izz="0", ixy="0", ixz="0", iyz="0")

    def box(link, name, xyz, size, color, collision=True):
        for tag in (("visual", "collision") if collision else ("visual",)):
            elem = ET.SubElement(link, tag, name=name)
            ET.SubElement(elem, "origin", xyz=" ".join(map(str, xyz)))
            geom = ET.SubElement(elem, "geometry")
            ET.SubElement(geom, "box", size=" ".join(map(str, size)))
            if tag == "visual":
                material = ET.SubElement(elem, "material", name=name + "_color")
                ET.SubElement(material, "color", rgba=color)

    box(base, "floor_a", [0, -1.5, -0.1], [8.4, 3, 0.2], "0.42 0.56 0.65 1")
    box(base, "floor_b", [0, 1.5, -0.1], [8.4, 3, 0.2], "0.66 0.62 0.45 1")
    for name, xyz, size in [
        ("west", [-4.1, 0, 1.25], [0.2, 6.4, 2.5]),
        ("east", [4.1, 0, 1.25], [0.2, 6.4, 2.5]),
        ("south", [0, -3.1, 1.25], [8, 0.2, 2.5]),
        ("north", [0, 3.1, 1.25], [8, 0.2, 2.5]),
        ("partition_left", [-2.35, 0, 1.25], [3.3, 0.2, 2.5]),
        ("partition_right", [2.35, 0, 1.25], [3.3, 0.2, 2.5]),
        ("lintel", [0, 0, 2.4], [1.4, 0.2, 0.2]),
    ]:
        box(base, name, xyz, size, "0.80 0.83 0.86 1")

    leaf = ET.SubElement(root, "link", name="door_leaf")
    inertia = ET.SubElement(leaf, "inertial")
    ET.SubElement(inertia, "origin", xyz="0.67 0 1.1")
    ET.SubElement(inertia, "mass", value="8")
    ET.SubElement(inertia, "inertia", ixx="3.228", iyy="4.424", izz="1.199", ixy="0", ixz="0", iyz="0")
    box(leaf, "door", [0.67, 0, 1.1], [1.34, 0.06, 2.2], "0.78 0.35 0.12 1")
    # Visual stripe makes the moving leaf easy to distinguish from the frame.
    box(leaf, "stripe", [1.18, -0.034, 1.1], [0.05, 0.01, 1.8], "0.95 0.85 0.3 1", False)
    joint = ET.SubElement(root, "joint", name="door_hinge", type="revolute")
    ET.SubElement(joint, "parent", link="building")
    ET.SubElement(joint, "child", link="door_leaf")
    ET.SubElement(joint, "origin", xyz="-0.67 0 0.03")
    ET.SubElement(joint, "axis", xyz="0 0 1")
    ET.SubElement(joint, "limit", lower="0", upper=str(math.pi / 2), effort="40", velocity="0.5")
    ET.SubElement(joint, "dynamics", damping="0.4", friction="0.1")
    return ET.tostring(root, encoding="unicode")


class FreeCamera:
    def __init__(self, config):
        self.config = config
        self.drag = None
        self.reset()

    def reset(self):
        self.position = np.array(self.config["camera_position"], dtype=float)
        direction = np.array(self.config["camera_target"]) - self.position
        self.yaw = math.atan2(direction[1], direction[0])
        self.pitch = math.atan2(direction[2], np.linalg.norm(direction[:2]))
        self.drag = None

    @property
    def direction(self):
        return np.array([math.cos(self.pitch) * math.cos(self.yaw),
                         math.cos(self.pitch) * math.sin(self.yaw), math.sin(self.pitch)])

    def look(self, dx, dy):
        self.yaw -= dx * 0.004
        self.pitch = float(np.clip(self.pitch - dy * 0.004, -1.5, 1.5))

    def move(self, key):
        forward = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        right = np.array([math.sin(self.yaw), -math.cos(self.yaw), 0.0])
        vectors = {"w": forward, "s": -forward, "a": -right, "d": right,
                   "q": np.array([0, 0, -1]), "e": np.array([0, 0, 1])}
        if key in vectors:
            self.position += vectors[key] * self.config["camera_step"]


class DoorController:
    """Shared interface for manual buttons and future system commands."""
    def __init__(self, physics, body, config):
        self.p, self.body, self.config = physics, body, config
        self.joint = next(i for i in range(physics.getNumJoints(body))
                          if physics.getJointInfo(body, i)[1] == b"door_hinge")
        self.target = 0.0
        self.close_door()

    def _command(self, angle):
        self.target = angle
        self.p.setJointMotorControl2(self.body, self.joint, self.p.POSITION_CONTROL,
                                    targetPosition=angle, force=self.config["door_force"],
                                    maxVelocity=self.config["door_speed"])

    def open_door(self):
        self._command(self.config["door_open_angle"])

    def close_door(self):
        self._command(0.0)

    def reset(self):
        self.p.resetJointState(self.body, self.joint, 0, targetVelocity=0)
        self.close_door()

    @property
    def angle(self):
        return self.p.getJointState(self.body, self.joint)[0]

    @property
    def status(self):
        angle = self.angle
        if abs(angle - self.target) > 0.03:
            return "OPENING / BLOCKED" if self.target else "CLOSING / BLOCKED"
        return "OPEN" if self.target else "CLOSED"


def load_config(path):
    with Path(path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    for name in ("robot_position", "target_position", "camera_position", "camera_target"):
        value = np.asarray(config[name], dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError(name + " must be finite XYZ")
    for name in ("camera_step", "door_speed", "door_force"):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(name + " must be finite and positive")
    if not 0 < config["door_open_angle"] <= math.pi / 2:
        raise ValueError("door_open_angle must be in (0, pi/2]")
    if not math.isfinite(config["robot_yaw"]):
        raise ValueError("robot_yaw must be finite")
    if np.allclose(config["camera_position"], config["camera_target"]):
        raise ValueError("Camera position and target must differ")
    for name in ("image_width", "image_height"):
        if type(config[name]) is not int or config[name] <= 0:
            raise ValueError(name + " must be a positive integer")
    return config


def run(config, steps=None, headless=False):
    # Delay heavy imports so --help and geometry tests work without iGibson.
    import cv2
    import pybullet as p
    from igibson.objects.articulated_object import ArticulatedObject
    from igibson.objects.visual_marker import VisualMarker
    from igibson.render.mesh_renderer.mesh_renderer_settings import MeshRendererSettings
    from igibson.robots.turtlebot import Turtlebot
    from igibson.scenes.empty_scene import EmptyScene
    from igibson.simulator import Simulator

    sim = None
    with tempfile.TemporaryDirectory(prefix="igibson_two_rooms_") as folder:
        path = Path(folder) / "two_rooms.urdf"
        path.write_text(build_room_urdf(), encoding="utf-8")
        try:
            # Own the OpenCV window and event loop; avoid competing waitKey calls
            # and the native viewer's robot-control/manipulation shortcuts.
            sim = Simulator(mode="headless", image_width=config["image_width"],
                            image_height=config["image_height"], vertical_fov=65,
                            rendering_settings=MeshRendererSettings(enable_shadow=False, enable_pbr=False))
            sim.import_scene(EmptyScene(render_floor_plane=False))
            room = ArticulatedObject(str(path), merge_fixed_links=False)
            sim.import_object(room)
            door = DoorController(p, room.get_body_ids()[0], config)
            robot = Turtlebot(action_type="continuous", action_normalize=True,
                              controller_config={"base": {"name": "DifferentialDriveController"}})
            sim.import_object(robot)
            marker = VisualMarker(visual_shape=p.GEOM_CYLINDER, radius=0.22,
                                  length=0.025, rgba_color=[0.1, 0.95, 0.2, 1])
            sim.import_object(marker)
            marker.set_position(config["target_position"])

            def reset():
                robot.reset()
                robot.set_position_orientation(config["robot_position"],
                                               p.getQuaternionFromEuler([0, 0, config["robot_yaw"]]))
                for body in robot.get_body_ids():
                    p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
                robot.keep_still()
                door.reset()
                sim.sync(force_sync=True)

            reset()
            camera = FreeCamera(config)
            if not headless:
                cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)

                def mouse(event, x, y, flags, _):
                    if event == cv2.EVENT_LBUTTONDOWN:
                        camera.drag = (x, y)
                    elif event == cv2.EVENT_LBUTTONUP:
                        camera.drag = None
                    elif event == cv2.EVENT_MOUSEMOVE and camera.drag is not None:
                        camera.look(x - camera.drag[0], y - camera.drag[1])
                        camera.drag = (x, y)

                cv2.setMouseCallback(WINDOW, mouse)
            print("WASD move | Q/E down/up | left drag look | O open | C close | R reset | V view | Esc exit")
            frame_count = 0
            while steps is None or frame_count < steps:
                started = time.monotonic()
                sim.step()
                frame_count += 1
                if headless:
                    continue
                sim.renderer.set_camera(camera.position, camera.position + camera.direction, [0, 0, 1])
                rgb = sim.renderer.render(modes=("rgb",))[0][:, :, :3]
                frame = cv2.cvtColor((np.clip(rgb, 0, 1) * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
                labels = ["WASD move | Q/E down/up | Left drag: look | V: default view",
                          "O: open door | C: close door | R: reset scene | Esc: exit",
                          "Room A: blue floor / robot | Room B: sand floor / green goal",
                          f"Door: {door.status} | angle: {math.degrees(door.angle):.1f} deg"]
                for i, line in enumerate(labels):
                    cv2.putText(frame, line, (15, 25 + i * 25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.52, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(frame, line, (15, 25 + i * 25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.52, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(WINDOW, frame)
                key = cv2.waitKey(max(1, int((1 / 30 - (time.monotonic() - started)) * 1000))) & 0xFF
                if key == 27 or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
                char = chr(key).lower()
                if char == "o":
                    door.open_door()
                elif char == "c":
                    door.close_door()
                elif char == "r":
                    reset()
                elif char == "v":
                    camera.reset()
                else:
                    camera.move(char)
        finally:
            if sim is not None:
                sim.disconnect()
            if not headless:
                cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "interaction_scene.yaml")
    parser.add_argument("--headless", action="store_true", help="No window; requires --steps")
    parser.add_argument("--steps", type=int, help="Exit after N simulation steps")
    args = parser.parse_args(argv)
    if args.steps is not None and args.steps <= 0:
        parser.error("--steps must be positive")
    if args.headless and args.steps is None:
        parser.error("--headless requires --steps")
    try:
        run(load_config(args.config), args.steps, args.headless)
    except ModuleNotFoundError as exc:
        raise SystemExit(
            f"Missing dependency: {exc.name}. Run this entry in the working iGibson "
            "environment; see INTERACTION_SCENE.md."
        ) from exc


if __name__ == "__main__":
    main()
