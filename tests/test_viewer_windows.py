import sys
from types import SimpleNamespace
import pytest
from viewer_windows import resizable_viewer_windows


@pytest.fixture
def cv2(monkeypatch):
    module = SimpleNamespace(WINDOW_AUTOSIZE=1, WINDOW_FREERATIO=256,
                             WINDOW_NORMAL=0, WINDOW_KEEPRATIO=0,
                             namedWindow=lambda *args: None)
    monkeypatch.setitem(sys.modules, 'cv2', module)
    return module


def test_native_windows_resizable_and_unrelated_windows_unchanged(monkeypatch, cv2):
    calls = []
    original = lambda name, flags=cv2.WINDOW_AUTOSIZE: calls.append((name, flags))
    monkeypatch.setattr(cv2, 'namedWindow', original)
    with resizable_viewer_windows():
        cv2.namedWindow('Viewer')
        cv2.namedWindow('RobotView', cv2.WINDOW_AUTOSIZE | cv2.WINDOW_FREERATIO)
        cv2.namedWindow('Other')
    assert cv2.namedWindow is original
    for name, flags in calls[:2]:
        assert not flags & cv2.WINDOW_AUTOSIZE
        assert not flags & cv2.WINDOW_FREERATIO
    assert calls[2] == ('Other', cv2.WINDOW_AUTOSIZE)


def test_creation_failure_restores_opencv(cv2):
    original = cv2.namedWindow
    with pytest.raises(RuntimeError):
        with resizable_viewer_windows():
            raise RuntimeError('environment creation failed')
    assert cv2.namedWindow is original
