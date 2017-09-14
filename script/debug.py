import tkinter as Tk

CS_command_list = [
  ("CS0", "CANSAT OFF"),
  ("CS1", "CANSAT ON"),
]

L_command_list = [
  ("L0",  "LED OFF"),
  ("L1",  "LED ON"),
  ("LT",  "Toggle LED"),
]

CC_command_list = [
  ("T",   "run test"),
  ("S",   "CC status"),
  ("CCC", "CC configure"),
  ("CCT", "STX"),
  ("CCR", "SRX"),
  ("CC0", "CC OFF"),
  ("CC1", "CC ON"),
  ]

PA_command_list = [
  ("PA0", "PA OFF"),
  ("PA1", "PA ON"),
]

class DebugApp:
    buttons = []
    tty = "/dev/ttyUSB0"

    def __init__(self, master):
        frame = Tk.Frame(master, height = 600, width = 800)
        frame.pack_propagate(0)
        frame.pack()

        self.buttons.append(Tk.Label(frame, text="CANSAT commands", width=20, pady=10))

        for b in CS_command_list:
            self.buttons.append(Tk.Button(frame, text=b[1], command=(lambda b=b:print(b[0])), padx=5, pady=5, width=20))

        self.buttons.append(Tk.Label(frame, text="LED commands", width=20, pady=10))

        for b in L_command_list:
            self.buttons.append(Tk.Button(frame, text=b[1], command=(lambda b=b:print(b[0])), padx=5, pady=5, width=20))

        self.buttons.append(Tk.Label(frame, text="CC1125 commands", width=20, pady=10))

        for b in CC_command_list:
            self.buttons.append(Tk.Button(frame, text=b[1], command=(lambda b=b:print(b[0])), padx=5, pady=5, width=20))

        self.buttons.append(Tk.Label(frame, text="PA commands", width=20, pady=10))

        for b in PA_command_list:
            self.buttons.append(Tk.Button(frame, text=b[1], command=(lambda b=b:print(b[0])), padx=5, pady=5, width=20))

        for i,b in enumerate(self.buttons):
            b.pack(anchor=Tk.W)

        self.button = Tk.Button(frame, text="exit", command=frame.quit)
        self.button.pack(side=Tk.BOTTOM)

        self.rb1 = Tk.Radiobutton(frame, text="/dev/ttyUSB0", variable=self.tty, value="/dev/ttyUSB0").pack(anchor=Tk.W)
        self.rb2 = Tk.Radiobutton(frame, text="/dev/ttyUSB1", variable=self.tty, value="/dev/ttyUSB1").pack(anchor=Tk.W)


root = Tk.Tk()
app = DebugApp(root)

root.mainloop()
