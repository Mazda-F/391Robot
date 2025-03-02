from tkinter import Tk, Label, Scale, Button, StringVar, Text, HORIZONTAL, VERTICAL
import queue
from math import atan2, pi

class Dashboard:
    def __init__(self, queue, lock, readlock):
        self.queue = queue
        self.lock = lock
        self.readlock = readlock
        self.root = Tk()
        self.root.title("dashboard")
        self.window_w = 520
        self.window_h = 800
        self.root.geometry(f"{self.window_w}x{self.window_h}")

        self.pressed_keys = set()
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

        self.label_up = Label(self.root, text="⬆", font=("Arial", 25), width=8, height=2, bg="grey", fg="white")
        self.label_down = Label(self.root, text="⬇", font=("Arial", 25), width=8, height=2, bg="grey", fg="white")
        self.label_left = Label(self.root, text="⬅", font=("Arial", 25), width=8, height=2, bg="grey", fg="white")
        self.label_right = Label(self.root, text="➡", font=("Arial", 25), width=8, height=2, bg="grey", fg="white")
        
        self.label_speed = Label(self.root, text="Speed: 0", font=("Arial", 25))
        self.label_yaw = Label(self.root, text="Yaw Angle: 0", font=("Arial", 25))
        self.label_pitch = Label(self.root, text="Recieved Pitch: 0 deg", font=("Arial", 25))
        self.lscale = Scale(self.root, from_=8, to=-8, orient=VERTICAL, length=300, tickinterval=5, command=self.on_slider_input)
        self.rscale = Scale(self.root, from_=8, to=-8, orient=VERTICAL, length=300, tickinterval=5, command=self.on_slider_input)

        self.kp_tbox = Text(self.root, height=3)
        self.ki_tbox = Text(self.root, height=3)
        self.kd_tbox = Text(self.root, height=3)

        self.setkbtn = Button(self.root, text="SET K VALUES", font=("Arial", 12), command=self.on_setk)


        self.label_lm = Label(self.root, text="L", font=("Arial", 25), width=8, height=2, bg="white", fg="black")
        self.label_rm = Label(self.root, text="R", font=("Arial", 25), width=8, height=2, bg="white", fg="black")
        self.toggle_lable = StringVar()
        self.toggle_tilt_control = Button(self.root, text="Start Running", font=("Arial", 12), command=self.on_control_toggle)

        # self.label_up.grid(row=0, column=1, padx=5, pady=5)
        # self.label_left.grid(row=1, column=0, padx=5, pady=5)
        # self.label_down.grid(row=1, column=1, padx=5, pady=5)
        # self.label_right.grid(row=1, column=2, padx=5, pady=5)

        self.label_speed.grid(row=2, column=0, columnspan=3, padx=10)
        self.label_yaw.grid(row=3, column=0, columnspan=3, padx=10)
        self.label_pitch.grid(row=4, column=0, columnspan=3, padx=10)
        # self.lscale.grid(row=5, column=0)
        # self.rscale.grid(row=5, column=2)
        # self.label_lm.grid(row=6, column=0)
        # self.label_rm.grid(row=6, column=2)
        self.toggle_tilt_control.grid(row=7,column=1)
        self.kp_tbox.grid(row=8,column=1)
        self.ki_tbox.grid(row=9,column=1)
        self.kd_tbox.grid(row=10,column=1)
        self.setkbtn.grid(row=11,column=1)

        self.states = {"Idle" : 0, "Running" : 1}

        with self.lock:
            self.control_state = self.states["Idle"]
            self.speed = 0
            self.yaw = 0
            self.motor_left = 0
            self.motor_right = 0
            self.Kp = 0.0
            self.Ki = 0.0
            self.Kd = 0.0

        with self.readlock:
            self.pitch = 0

        self.label_map = {
            "Up":    self.label_up,
            "Down":  self.label_down,
            "Left":  self.label_left,
            "Right": self.label_right
        }

        self.update_read()
         
    

    def processInput(self):
        f = 1 if "Up" in self.pressed_keys else 0
        b = 1 if "Down" in self.pressed_keys else 0
        l = 1 if "Left" in self.pressed_keys else 0
        r = 1 if "Right" in self.pressed_keys else 0
        
        speed = (f-b)*100
        yaw = atan2(l-r,1) * 180.0/pi

        motor_left = self.lscale.get()
        motor_right = self.rscale.get()


        # self.queue.put({"speed": speed})
        # self.queue.put({"steer": yaw})  
        # self.queue.put((speed, yaw))

        with self.lock:
            self.speed = speed 
            self.yaw = yaw
            self.motor_left = motor_left
            self.motor_right = motor_right

        self.update_graphics(speed, yaw)

    def on_setk(self):
        Ki = float(self.ki_tbox.get(1.0, "end-1c") )
        Kd = float(self.kd_tbox.get(1.0, "end-1c") )
        Kp = float(self.kp_tbox.get(1.0, "end-1c") )
        with self.lock:
            self.Ki = Ki
            self.Kd = Kd
            self.Kp = Kp
        
    def on_key_press(self, e): # e is event being passed into arg
        if e.keysym in ("Left", "Right", "Up", "Down"):
            self.pressed_keys.add(e.keysym)
            self.processInput()
    
    def on_key_release(self, e):
        if e.keysym in self.pressed_keys:
            self.pressed_keys.remove(e.keysym)
            self.processInput()

    def on_slider_input(self, e):
        self.processInput()

    def on_control_toggle(self):
        with self.lock:
            if self.control_state == self.states["Idle"]:
                self.control_state = self.states["Running"]
                self.toggle_tilt_control.config(text="Switch to Idle")
                
            elif self.control_state == self.states["Running"]:
                self.control_state = self.states["Idle"]
                self.toggle_tilt_control.config(text="Start Running")

    def update_graphics(self, speed, yaw):
        for direction, label in self.label_map.items():
            if direction in self.pressed_keys:
                label.config(bg="black")
            else:
                label.config(bg="grey")
        
        self.label_speed.config(text = f"Speed: {speed}")
        self.label_yaw.config(text = f"Yaw Angle: {yaw}")

    def update_read(self):
        with self.readlock:
            pitch = self.pitch
        self.label_pitch.config(text = f"Recieved Pitch: {pitch} deg")
        self.root.after(10, self.update_read)



if __name__ == "__main__":
    event_queue = queue.LifoQueue()
    dash = Dashboard(event_queue)
    dash.root.mainloop()