import sys
from types import SimpleNamespace
import pytest
from viewer_windows import resizable_viewer_windows
from viewer_windows import DisplayAdapter
import numpy as np


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
        assert flags & cv2.WINDOW_FREERATIO
    assert calls[2] == ('Other', cv2.WINDOW_AUTOSIZE)


def test_creation_failure_restores_opencv(cv2):
    original = cv2.namedWindow
    with pytest.raises(RuntimeError):
        with resizable_viewer_windows():
            raise RuntimeError('environment creation failed')
    assert cv2.namedWindow is original


def test_text_stays_fixed_and_mouse_maps_through_letterbox():
    draws=[]; shown=[]
    backend=SimpleNamespace(
        getWindowImageRect=lambda name:(0,0,800,800),
        resize=lambda image,size:np.ones((size[1],size[0],3),dtype=image.dtype),
        putText=lambda image,*args,**kwargs:draws.append((image.shape,args)),
        imshow=lambda name,image:shown.append(image))
    adapter=DisplayAdapter(backend)
    source=np.zeros((960,1280,3),dtype=np.uint8)
    text_args=('camera',(10,20),0,.5,(255,255,255),1)
    adapter.putText(source,*text_args)
    assert not draws
    adapter.imshow('Viewer',source)
    assert draws[0]==((800,800,3),text_args)
    assert draws[1]==((960,1280,3),text_args)  # original-size recording
    assert not shown[0][:100].any()
    assert adapter.mouse_position('Viewer',400,400)==(640,480)
    assert adapter.mouse_position('Viewer',400,50) is None
    assert adapter.mouse_position('Viewer',400,50,clamp=True)==(640,0)
    backend.getWindowImageRect=lambda name:(0,0,1600,1200)
    adapter.putText(source,*text_args)
    adapter.imshow('Viewer',source)
    assert draws[2]==((1200,1600,3),text_args)
