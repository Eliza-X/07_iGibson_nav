"""Use resizable, aspect-preserving native iGibson OpenCV windows.

Apply only during environment creation, before iGibson installs its mouse
callbacks. No installed package edits or persistent OpenCV monkey patches.
"""
from contextlib import contextmanager


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
            # Keep the original image coordinates for iGibson mouse picking.
            # HighGUI handles scaling the image and mapping mouse coordinates.
            flags = (flags & ~cv2.WINDOW_AUTOSIZE & ~cv2.WINDOW_FREERATIO)
            flags |= cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
        return original(name, flags)

    cv2.namedWindow = named_window
    try:
        yield
    finally:
        cv2.namedWindow = original
