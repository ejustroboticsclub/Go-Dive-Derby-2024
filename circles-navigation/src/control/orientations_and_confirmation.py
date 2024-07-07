import tkinter as tk
import tkinter.messagebox as msg
import cv2
import subprocess


from class_detect import CirclesDetector



class GUIsmol():
    def __init__(self):
        window = tk.Tk()
        window.title("Simple Window")
        window.geometry("800x400")
        self.frame = tk.Frame(window)

        self.frame = None
        self.orientations = []
        self.i = 1

        self.detector = CirclesDetector()


        self.second_var = tk.StringVar()
        self.third_var  = tk.StringVar()

        self.label2 = tk.Label(self.frame, text="Second Circle Orientation: ")
        self.label2.grid(row = 0, column = 0)

        self.label3 = tk.Label(self.frame, text="Third Circle Orientation: ")
        self.label3.grid(row = 0, column = 2)

        self.label_orien = tk.Label(self.frame, text=f"Current Orientations: {self.orientations}")
        self.label_orien.grid(row = 4, column = 0)
        self.label_helpmsg = tk.Label(self.frame, 
                                    text="Change the orientation, then confirm the images. If it's bad, the mechanism to take another image is replaced by just changing the photo for now. If it's good, the automatic control file is executed with the selected values from radio boxes as the parameters for the orientation of the 2nd and the 3rd circle respectively.",
                                    justify=tk.LEFT,
                                    wraplength=500)  # adjust this value as needed
        self.label_helpmsg.grid(row=10, column=0, sticky='w')


        self.r1 = tk.Radiobutton(self.frame, text="Right", variable=self.second_var, value="r")
        self.r1.grid(row = 1, column = 0)

        self.r2 = tk.Radiobutton(self.frame, text="Left", variable=self.second_var, value="l")
        self.r2.grid(row =2, column = 0)

        self.r3 = tk.Radiobutton(self.frame, text="Right", variable=self.third_var, value="r")
        self.r3.grid(row = 1, column = 2)

        self.r4 = tk.Radiobutton(self.frame, text="Left", variable=self.third_var, value="l")
        self.r4.grid(row = 2, column = 2)

        self.button1 = tk.Button(self.frame, text="Change Orientations", command=self.change_orientations)
        self.button1.grid(row = 4, column= 2)

        self.button1 = tk.Button(self.frame, text="Confirm Image", command=self.confirm_circles)
        self.button1.grid(row = 10, column= 2)

        window.mainloop()


    def change_orientations(self):
        self.orientations = [self.second_var.get(), self.third_var.get()]
        
        self.label_orien = tk.Label(self.frame, text=f"Current Orientations: {self.orientations}")
        self.label_orien.grid(row = 4, column = 0)

        print(self.orientations)
    
    def confirm_circles(self):
        self.frame = "circles" + str(self.i) + ".jpg"
        frame = cv2.imread(self.frame)
        _, circle = self.detector.detect(frame)
        cv2.imshow("frame", self.detector.draw_circles(frame, circle))
        response = msg.askyesno("Question", "bogos gud?")

        if response:
            cv2.destroyWindow("frame")
            command = ["python", "Automatic_Control.py", self.orientations[0], self.orientations[1]]
            subprocess.Popen(command)
        else:
            self.i = self.i+1 if self.i < 6 else 1
            self.confirm_circles()


GUI = GUIsmol()
