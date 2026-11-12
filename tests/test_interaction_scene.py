import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from interaction_scene import ROOT, DoorController, FreeCamera, build_room_urdf, load_config


def test_camera_moves_independently_and_resets():
    config = load_config(ROOT / "interaction_scene.yaml")
    camera = FreeCamera(config)
    original = camera.position.copy()
    camera.move("w")
    assert camera.position[2] == original[2]
    assert np.linalg.norm(camera.position - original) == pytest.approx(config["camera_step"])
    camera.look(100, 10000)
    assert camera.pitch == -1.5
    assert np.linalg.norm(camera.direction) == pytest.approx(1)
    camera.reset()
    np.testing.assert_allclose(camera.position, original)
    expected = np.array(config["camera_target"]) - original
    np.testing.assert_allclose(camera.direction, expected / np.linalg.norm(expected))


def test_door_has_physical_leaf_and_fixed_building():
    root = ET.fromstring(build_room_urdf())
    assert root.find("link[@name='building']/inertial/mass").get("value") == "0"
    assert root.find("link[@name='door_leaf']/collision/geometry/box") is not None
    joint = root.find("joint[@name='door_hinge']")
    assert joint.get("type") == "revolute"
    assert float(joint.find("limit").get("upper")) == pytest.approx(math.pi / 2)


def test_door_status_requires_feedback_not_just_a_command():
    class Physics:
        POSITION_CONTROL = 2
        ACTIVATION_STATE_WAKE_UP = 4
        angle = 0.0

        def changeDynamics(self, body, link, **kwargs):
            assert kwargs["activationState"] == self.ACTIVATION_STATE_WAKE_UP
            self.awake_links.add(link)

        def __init__(self):
            self.awake_links = set()

        def getNumJoints(self, body):
            return 1

        def getJointInfo(self, body, joint):
            return (0, b"door_hinge")

        def setJointMotorControl2(self, body, joint, mode, **kwargs):
            assert self.awake_links == {-1, 0}
            self.command = kwargs

        def getJointState(self, body, joint):
            return (self.angle,)

        def resetJointState(self, body, joint, angle, targetVelocity):
            self.angle = angle

    physics = Physics()
    config = load_config(ROOT / "interaction_scene.yaml")
    door = DoorController(physics, 0, config)
    door.open_door()
    assert door.status == "OPENING / BLOCKED"
    assert physics.angle == 0
    assert physics.command["maxVelocity"] == config["door_speed"]
    physics.angle = config["door_open_angle"]
    assert door.status == "OPEN"
    door.close_door()
    assert door.status == "CLOSING / BLOCKED"
    door.reset()
    assert door.status == "CLOSED"


def test_physical_door_opens_clears_passage_and_resets(tmp_path):
    p = pytest.importorskip("pybullet")
    path = tmp_path / "scene.urdf"
    path.write_text(build_room_urdf())
    client = p.connect(p.DIRECT)
    try:
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1 / 240)
        body = p.loadURDF(str(path), flags=p.URDF_ENABLE_SLEEPING)
        door = DoorController(p, body, load_config(ROOT / "interaction_scene.yaml"))
        assert p.getDynamicsInfo(body, -1)[0] == 0
        assert p.rayTest([0, -1, 0.3], [0, 1, 0.3])[0][1] == door.joint
        # Reproduce opening after the user has left the inspection scene idle.
        for link in range(-1, p.getNumJoints(body)):
            p.changeDynamics(body, link, activationState=p.ACTIVATION_STATE_SLEEP)
        door.open_door()
        for _ in range(120):
            p.stepSimulation()
        assert 0.05 < door.angle < 0.4  # visible motion, not a teleport
        for _ in range(1200):
            p.stepSimulation()
        assert door.status == "OPEN"
        assert p.rayTest([0, -1, 0.3], [0, 1, 0.3])[0][0] == -1
        assert not p.getContactPoints(body, body)
        door.close_door()
        for _ in range(1200):
            p.stepSimulation()
        assert door.status == "CLOSED"
        assert p.rayTest([0, -1, 0.3], [0, 1, 0.3])[0][1] == door.joint
        door.open_door()
        door.reset()
        assert door.status == "CLOSED"
        assert door.angle == 0
    finally:
        p.disconnect(client)
