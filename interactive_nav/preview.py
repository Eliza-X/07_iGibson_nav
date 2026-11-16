"""Resizable, aspect-preserving scene inspection; no navigation or physics ticks."""
import math
import numpy as np


def preview_scene(env):
    import tkinter as tk
    from PIL import Image, ImageTk

    env.reset()  # Land once; inspection subsequently leaves the physics state frozen.
    root = tk.Tk()
    root.title('Door bypass — scene preview')
    root.geometry('1100x780')
    root.minsize(480, 360)
    root.resizable(True, True)
    tk.Label(root, text='Drag window edges: resize | Left drag: orbit | Wheel: zoom | R: reset view | Esc: close').pack(fill='x')
    canvas = tk.Canvas(root, background='#181818', highlightthickness=0)
    canvas.pack(fill='both', expand=True)
    item = canvas.create_image(0, 0, anchor='center')
    renderer = env.simulator.renderer
    saved = [np.array(getattr(renderer, name), copy=True) for name in ('camera', 'target', 'up')]
    state = dict(azimuth=-math.pi/2, elevation=1.05, distance=11., drag=None, dirty=True, frame=None)

    def repaint(event=None):
        if state['frame'] is None:
            return
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        source = state['frame']
        ratio = min(width/source.width, height/source.height)
        size = (max(1, round(source.width*ratio)), max(1, round(source.height*ratio)))
        canvas.photo = ImageTk.PhotoImage(source.resize(size, Image.Resampling.LANCZOS))
        canvas.itemconfigure(item, image=canvas.photo)
        canvas.coords(item, width/2, height/2)

    def press(event):
        state['drag'] = (event.x, event.y)

    def drag(event):
        if state['drag'] is not None:
            x, y = state['drag']
            state['azimuth'] -= (event.x-x)*.006
            state['elevation'] = float(np.clip(state['elevation']+(event.y-y)*.006, .15, 1.5))
            state['dirty'] = True
        state['drag'] = (event.x, event.y)

    def zoom(event):
        direction = (1 if event.num == 4 else -1) if getattr(event, 'num', None) in (4,5) else (1 if event.delta > 0 else -1)
        state['distance'] = float(np.clip(state['distance']*(.9 if direction>0 else 1.1), 2., 25.))
        state['dirty'] = True

    def reset(event=None):
        state.update(azimuth=-math.pi/2, elevation=1.05, distance=11., dirty=True)

    errors = []
    def render():
        try:
            if state['dirty']:
                az, el, distance = state['azimuth'], state['elevation'], state['distance']
                target = np.array([0.,0.,.3])
                camera = target + distance*np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
                env.simulator.sync(force_sync=True)
                renderer.set_camera(camera, target, [0,0,1], cache=False)
                frame = np.asarray(renderer.render(modes=('rgb',))[0])[...,:3]
                if np.issubdtype(frame.dtype,np.floating):
                    frame = np.clip(frame,0,1)*255
                state['frame'] = Image.fromarray(np.asarray(frame,dtype=np.uint8))
                state['dirty'] = False
                repaint()
            root.after(33, render)
        except Exception as exc:
            errors.append(exc)
            root.destroy()

    canvas.bind('<Configure>', repaint)
    canvas.bind('<ButtonPress-1>', press)
    canvas.bind('<B1-Motion>', drag)
    canvas.bind('<MouseWheel>', zoom)
    canvas.bind('<Button-4>', zoom)
    canvas.bind('<Button-5>', zoom)
    root.bind('<r>', reset)
    root.bind('<Escape>', lambda event: root.destroy())
    try:
        root.after(0, render)
        root.mainloop()
        if errors:
            raise errors[0]
    finally:
        renderer.set_camera(*saved, cache=False)
