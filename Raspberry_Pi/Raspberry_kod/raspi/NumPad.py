from tkinter import *
from tkinter import simpledialog,Text

class NumPad(simpledialog.Dialog):
    def __init__(self,root=None,parent=None,masa="min"):
        self.root=root
        self.masa = masa
        self.parent=parent
        self.top = Toplevel(master=parent.frame)
        self.top.title(masa)
        self.top.protocol("WM_DELETE_WINDOW", self.OK)

        self.EntryFrame = Frame(self.top)
        self.BtnsFrame = Frame(self.top)
        self.createWidgets()
        self.top.wait_visibility()
        self.top.grab_set()

    def createWidgets(self):
        self.entry = Entry(self.EntryFrame,font=("Helvetica",32),width=5,justify=CENTER)
        self.entry.pack(pady=10)
        btn_list = ['7',  '8',  '9', '4',  '5',  '6', '1',  '2',  '3', 'Del',  '0',  'OK']
        # create and position all buttons with a for-loop
        # r, c used for row, column grid values
        r = 0
        c = 0
        n = 0
        # list(range()) needed for Python3
        btns = []
        for label in btn_list:
            # partial takes care of function and argument
            cmd = lambda x = label: self.click(x)
            btns.append(Button(self.BtnsFrame, text=label, width=10, height=5, command=cmd))
            btns[-1].grid(row=r, column=c)
            # increment button index
            n += 1
            # update row/column position
            c += 1
            if c == 3:
                c = 0
                r += 1
        self.EntryFrame.pack()
        self.BtnsFrame.pack()

    def click(self,label):
        if self.masa == "min":
            if label == 'Del':
                currentText = str(self.parent.masa_min.get())
                self.parent.masa_min.set(currentText[:-1])
                self.parent.entry_masa_min.delete(0, END)
                self.entry.delete(0,END)
                self.parent.entry_masa_min.insert(0, self.parent.masa_min.get())
                self.entry.insert(0,self.parent.masa_min.get())
            elif label == 'OK':
                self.OK()
            else:
                currentText = str(self.parent.masa_min.get())
                self.parent.entry_masa_min.delete(0,END)
                self.entry.delete(0, END)
                self.parent.masa_min.set(int(currentText+label))
                self.parent.entry_masa_min.insert(0, self.parent.masa_min.get())
                self.entry.insert(0,self.parent.masa_min.get())

        elif self.masa == "max":
            if label == 'Del':
                currentText = str(self.parent.masa_max.get())
                self.parent.masa_max.set(currentText[:-1])
                self.parent.entry_masa_max.delete(0, END)
                self.entry.delete(0, END)
                self.parent.entry_masa_max.insert(0, self.parent.masa_max.get())
                self.entry.insert(0, self.parent.masa_max.get())
            elif label == 'OK':
                self.OK()
            else:
                currentText = str(self.parent.masa_max.get())
                self.parent.entry_masa_max.delete(0,END)
                self.entry.delete(0, END)
                self.parent.masa_max.set(int(currentText+label))
                self.parent.entry_masa_max.insert(0, self.parent.masa_max.get())
                self.entry.insert(0, self.parent.masa_max.get())

    def OK(self):
        self.top.destroy()
        self.top.master.focus()