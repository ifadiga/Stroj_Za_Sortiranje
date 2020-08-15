from tkinter import Label,Button,Radiobutton,Entry,IntVar
from tkinter.constants import *
from NumPad import NumPad

RB_WIDTH=6
RB_HEIGHT=1
RB_FONT_SIZE=15
TOG_WIDTH=8

''' klasa za spremnike, toggle buttons, radio buttons, etc. '''
class Spremnik:
    def __init__(self,root,frame):
        self.root=root
        self.frame=frame
        self.oblik = IntVar()
        self.boja = IntVar()
        self.masa_min = IntVar()
        self.masa_max = IntVar()
        Label(frame, text="OBLICI").grid(row=0, column=0)
        Label(frame, text="BOJA").grid(row=0, column=1)
        Label(frame, text="MASA").grid(row=0, column=2)
        self.toggle_btn_oblik = Button(frame, text="OFF", bg='red', command=self.ToggleOblik, width=TOG_WIDTH)
        self.toggle_btn_oblik.grid(row=1, column=0)
        self.toggle_btn_boja = Button(frame, text="OFF", bg='red', command=self.ToggleBoja, width=TOG_WIDTH)
        self.toggle_btn_boja.grid(row=1, column=1)
        self.toggle_btn_masa = Button(frame, text="OFF", bg='red', command=self.ToggleMasa, width=TOG_WIDTH)
        self.toggle_btn_masa.grid(row=1, column=2)
        self.radio_btn_kugla = Radiobutton(frame, variable=self.oblik, value=1, text='kugla', state=DISABLED, width=RB_WIDTH, height=RB_HEIGHT, anchor=W, font=(None, RB_FONT_SIZE))
        self.radio_btn_kugla.grid(row=2, column=0)
        self.radio_btn_kocka = Radiobutton(frame, variable=self.oblik, value=2, text='kocka', state=DISABLED, width=RB_WIDTH, height=RB_HEIGHT, anchor=W, font=(None, RB_FONT_SIZE))
        self.radio_btn_kocka.grid(row=3, column=0)
        self.radio_btn_piramida = Radiobutton(frame, variable=self.oblik, value=3, text='piramida', state=DISABLED, width=RB_WIDTH, height=RB_HEIGHT, anchor=W, font=(None, RB_FONT_SIZE))
        self.radio_btn_piramida.grid(row=4, column=0)
        self.radio_btn_zelena = Radiobutton(frame, variable=self.boja, value=1, text='zelena', state=DISABLED, width=RB_WIDTH, height=RB_HEIGHT, anchor=W, font=(None, RB_FONT_SIZE))
        self.radio_btn_zelena.grid(row=2, column=1)
        self.radio_btn_crvena = Radiobutton(frame, variable=self.boja, value=2, text='crvena', state=DISABLED, width=RB_WIDTH, height=RB_HEIGHT, anchor=W, font=(None, RB_FONT_SIZE))
        self.radio_btn_crvena.grid(row=3, column=1)
        self.radio_btn_plava = Radiobutton(frame, variable=self.boja, value=3, text='plava', state=DISABLED, width=RB_WIDTH, height=RB_HEIGHT, anchor=W, font=(None, RB_FONT_SIZE))
        self.radio_btn_plava.grid(row=4, column=1)
        self.radio_btn_zuta = Radiobutton(frame, variable=self.boja, value=4, text='zuta', state=DISABLED, width=RB_WIDTH, height=RB_HEIGHT, anchor=W, font=(None, RB_FONT_SIZE))
        self.radio_btn_zuta.grid(row=5, column=1)
        self.label_masa_min = Label(frame, text="  MIN", fg="grey")
        self.label_masa_min.grid(row=2, column=2, sticky=W)
        self.entry_masa_min = Entry(frame, width=5, state=DISABLED)
        self.entry_masa_min.grid(row=2, column=2, columnspan=2, sticky=E)
        self.label_masa_max = Label(frame, text="  MAX", fg="grey")
        self.label_masa_max.grid(row=3, column=2, sticky=W)
        self.entry_masa_max = Entry(frame, width=5, state=DISABLED)
        self.entry_masa_max.grid(row=3, column=2, sticky=E)
        self.entry_masa_min.bind("<Button-1>",self.Numpad_Show_Min)
        self.entry_masa_max.bind("<Button-1>",self.Numpad_Show_Max)
        self._oblikState=False
        self._bojaState=False
        self._masaState=False

    def ToggleOblik(self):
        self._oblikState = not self._oblikState
        #print(self._oblikState)
        if self._oblikState == True:
            self.toggle_btn_oblik.config(bg='light green')
            self.toggle_btn_oblik.config(text='ON')
            self.radio_btn_kugla.config(state=NORMAL)
            self.radio_btn_kocka.config(state=NORMAL)
            self.radio_btn_piramida.config(state=NORMAL)
            self.radio_btn_kugla.select()
        elif self._oblikState == False:
            self.toggle_btn_oblik.config(bg='red')
            self.toggle_btn_oblik.config(text='OFF')
            self.radio_btn_kugla.config(state=DISABLED)
            self.radio_btn_kocka.config(state=DISABLED)
            self.radio_btn_piramida.config(state=DISABLED)
            self.oblik.set(0)

    def ToggleBoja(self):
        self._bojaState = not self._bojaState
        if self._bojaState == True:
            self.toggle_btn_boja.config(bg='light green')
            self.toggle_btn_boja.config(text='ON')
            self.radio_btn_crvena.config(state=NORMAL)
            self.radio_btn_plava.config(state=NORMAL)
            self.radio_btn_zelena.config(state=NORMAL)
            self.radio_btn_zuta.config(state=NORMAL)
            self.radio_btn_zelena.select()
        elif self._bojaState == False:
            self.toggle_btn_boja.config(bg='red')
            self.toggle_btn_boja.config(text='OFF')
            self.radio_btn_crvena.config(state=DISABLED)
            self.radio_btn_plava.config(state=DISABLED)
            self.radio_btn_zuta.config(state=DISABLED)
            self.radio_btn_zelena.config(state=DISABLED)
            self.boja.set(0)

    def ToggleMasa(self):
        self._masaState = not self._masaState
        if self._masaState == True:
            self.toggle_btn_masa.config(bg='light green')
            self.toggle_btn_masa.config(text='ON')
            self.entry_masa_max.config(state=NORMAL)
            self.entry_masa_min.config(state=NORMAL)
            self.label_masa_min.config(fg='black')
            self.label_masa_max.config(fg='black')
            self.masa_min.set(0)
            self.masa_max.set(1000)
            self.entry_masa_min.insert(0, 0)
            self.entry_masa_max.insert(0, 1000)
        elif self._masaState == False:
            self.toggle_btn_masa.config(bg='red')
            self.toggle_btn_masa.config(text='OFF')
            self.entry_masa_min.delete(0, END)
            self.entry_masa_max.delete(0, END)
            self.entry_masa_max.config(state=DISABLED)
            self.entry_masa_min.config(state=DISABLED)
            self.label_masa_min.config(fg='grey')
            self.label_masa_max.config(fg='grey')
            self.masa_min.set(0)
            self.masa_max.set(0)

    def Numpad_Show_Min(self,arg):
        if self._masaState == True:
            new = NumPad(self.root, self, "min")

    def Numpad_Show_Max(self,arg):
        if self._masaState == True:
            self.masa_max.set(0)
            new = NumPad(self.root, self, "max")