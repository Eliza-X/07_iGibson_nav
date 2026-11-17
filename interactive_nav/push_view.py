"""Fixed-camera display; UI waits never advance simulation physics."""
import time
import numpy as np


class PushView:
    def __init__(self, env, initial, distance, speed):
        import tkinter as tk
        from PIL import Image, ImageTk
        self.tk, self.Image, self.ImageTk = tk, Image, ImageTk
        self.env, self.speed = env, speed
        self.closed, self.paused = False, True
        self.root = tk.Tk()
        self.root.title('Push probe - fixed side view')
        self.root.geometry('1100x780')
        self.root.minsize(600, 420)
        self.status = tk.Label(self.root, anchor='w', justify='left', font=('Consolas', 13))
        self.status.pack(fill='x', padx=12, pady=8)
        tk.Label(self.root, text='Space: start / pause | Esc: close | Final frame stays until closed').pack(fill='x')
        self.canvas = tk.Canvas(self.root, background='#181818', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.item = self.canvas.create_image(0, 0, anchor='center')
        self.frame = None
        self.canvas.bind('<Configure>', self.repaint)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind('<Escape>', lambda event: self.close())
        self.root.bind('<space>', self.toggle)
        # Cover the robot start, initial box and final box with one fixed view.
        left = initial['robot_position'][0]-.4
        right = initial['box_position'][0]+distance+.5
        self.target = np.array([(left+right)/2, 0., .2])
        span = max(2., right-left)
        self.camera = self.target+np.array([-.3*span, -1.15*span, .75*span])

    def toggle(self, event=None):
        self.paused = not self.paused

    def close(self):
        if not self.closed:
            self.closed = True
            self.root.destroy()

    def repaint(self, event=None):
        if self.closed or self.frame is None:
            return
        w, h = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        ratio = min(w/self.frame.width, h/self.frame.height)
        resized = self.frame.resize((max(1, round(self.frame.width*ratio)), max(1, round(self.frame.height*ratio))), self.Image.Resampling.LANCZOS)
        self.photo = self.ImageTk.PhotoImage(resized)
        self.canvas.itemconfigure(self.item, image=self.photo)
        self.canvas.coords(self.item, w/2, h/2)

    def show(self, task, s):
        if self.closed:
            return
        ref = task.reference
        displacement = s['box_position'][0]-ref[0] if ref else 0.
        lateral = s['box_position'][1]-ref[1] if ref else 0.
        self.description = (f'State: {task.state}    Contact: {"YES" if s["contacts"] else "NO"}    Time: {task.elapsed:.1f} s\n'
                            f'Box: {displacement:.3f} / {task.c["push_distance"]:.2f} m    Lateral: {lateral:+.3f} m\n'
                            f'Result: {task.reason or "running"}')
        if hasattr(task, 'path_available'):
            detail = task.push.state if task.state == 'INTERACT' else task.state
            distance = np.linalg.norm(np.asarray(s['robot_position'][:2])-task.c['goal'])
            self.description += f'\nStage: {detail}    Path: {task.path_available}    Goal distance: {distance:.2f} m'
        self.env.simulator.sync(force_sync=True)
        renderer = self.env.simulator.renderer
        renderer.set_camera(self.camera, self.target, [0, 0, 1], cache=False)
        frame = np.asarray(renderer.render(modes=('rgb',))[0])[..., :3]
        if np.issubdtype(frame.dtype, np.floating):
            frame = np.clip(frame, 0, 1)*255
        self.frame = self.Image.fromarray(np.asarray(frame, dtype=np.uint8))
        self.repaint()
        self.pump()

    def pump(self):
        if not self.closed:
            self.status.configure(text=self.description+f'    Playback: {self.speed:g}x'+(' [PAUSED]' if self.paused else ''))
            self.root.update()

    def wait(self, deadline=0., final=False):
        while not self.closed:
            self.pump()
            if not final and not self.paused and time.perf_counter() >= deadline:
                return
            time.sleep(.01)
