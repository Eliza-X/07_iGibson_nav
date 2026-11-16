"""Use resizable, aspect-preserving native iGibson OpenCV windows.

Apply only during environment creation, before iGibson installs its mouse
callbacks. No installed package edits or persistent OpenCV monkey patches.
"""
from contextlib import contextmanager
import numpy as np
from types import FunctionType, MethodType


def fit_image(source_width, source_height, width, height):
    scale = min(width / source_width, height / source_height)
    w, h = max(1, round(source_width * scale)), max(1, round(source_height * scale))
    return (width-w)//2, (height-h)//2, w, h


class DisplayAdapter:
    """Local cv2 facade: compose at window size, then draw fixed-pixel text."""
    def __init__(self, cv2):
        self.cv2 = cv2
        self.text = []
        self.layout = {}

    def __getattr__(self, name):
        return getattr(self.cv2, name)

    def putText(self, image, *args, **kwargs):
        self.text.append((image, args, kwargs))
        return image

    def imshow(self, name, image):
        if name not in ('Viewer', 'RobotView'):
            return self.cv2.imshow(name, image)
        _, _, width, height = self.cv2.getWindowImageRect(name)
        if width <= 1 or height <= 1:
            width, height = image.shape[1], image.shape[0]
        x, y, w, h = fit_image(image.shape[1], image.shape[0], width, height)
        canvas = np.zeros((height, width, image.shape[2]), dtype=image.dtype)
        canvas[y:y+h, x:x+w] = self.cv2.resize(image, (w,h))
        self.layout[name] = (x,y,w,h,image.shape[1],image.shape[0])
        remaining = []
        for source, args, kwargs in self.text:
            if source is image:
                self.cv2.putText(canvas, *args, **kwargs)
                # Preserve native recording output at its original dimensions.
                self.cv2.putText(image, *args, **kwargs)
            else:
                remaining.append((source,args,kwargs))
        self.text = remaining
        return self.cv2.imshow(name, canvas)

    def mouse_position(self, name, x, y, clamp=False):
        if name not in self.layout:
            return None
        left,top,width,height,source_w,source_h = self.layout[name]
        if not clamp and not (left <= x < left+width and top <= y < top+height):
            return None
        return (int(np.clip((x-left)*source_w/width,0,source_w-1)),
                int(np.clip((y-top)*source_h/height,0,source_h-1)))


def attach_viewer_display(env):
    """Adapt only this viewer instance; leave installed modules and cv2 intact."""
    viewer = getattr(getattr(env, 'simulator', None), 'viewer', None)
    if viewer is None:
        return
    import cv2
    adapter = DisplayAdapter(cv2)
    def clone(method):
        function = method.__func__
        namespace = dict(function.__globals__, cv2=adapter)
        replacement = FunctionType(function.__code__, namespace, function.__name__,
                                   function.__defaults__, function.__closure__)
        replacement.__kwdefaults__ = function.__kwdefaults__
        return MethodType(replacement, viewer)
    if hasattr(viewer, 'show_help_text'):
        viewer.show_help_text = clone(viewer.show_help_text)
    update = clone(viewer.update)
    def display_update():
        adapter.text.clear()
        try:
            return update()
        finally:
            adapter.text.clear()
    viewer.update = display_update
    if hasattr(viewer, 'mouse_callback'):
        callback = viewer.mouse_callback
        releases = (cv2.EVENT_LBUTTONUP,cv2.EVENT_MBUTTONUP,cv2.EVENT_RBUTTONUP)
        def mouse(event,x,y,flags,param):
            position = adapter.mouse_position('Viewer',x,y,event in releases)
            if position is not None:
                callback(event,*position,flags,param)
        cv2.setMouseCallback('Viewer', mouse)
    viewer._display_adapter = adapter


@contextmanager
def resizable_viewer_windows(enabled=True):
    if not enabled:
        yield
        return
    try:
        import cv2
    except ModuleNotFoundError:
        # Allows environment test doubles without GUI dependencies. A real
        # iGibson viewer itself requires OpenCV and will report its import error.
        yield
        return

    original = cv2.namedWindow

    def named_window(name, flags=cv2.WINDOW_AUTOSIZE):
        if name in ('Viewer', 'RobotView'):
            # The adapter supplies a canvas matching the full client area;
            # it handles aspect ratio and mouse mapping itself.
            flags = (flags & ~cv2.WINDOW_AUTOSIZE & ~cv2.WINDOW_FREERATIO)
            flags |= cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO
        return original(name, flags)

    cv2.namedWindow = named_window
    try:
        yield
    finally:
        cv2.namedWindow = original
