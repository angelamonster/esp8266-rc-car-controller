import tkinter as tk
import socket
import getopt, sys
import json
import time
import datetime

class commandType():
    CONFIG  = 0
    CONTROL = 1

class Controller:
    DEFAULT_PORT = 1111

    def __init__(self, ip):
        self.pressed = {}
        self.prevPressed = {}
        self._initPresses()
        self._create_ui()
        self._host = ip
        self._port = self.DEFAULT_PORT
        self._lastsend_interval = 100 #ms
        t = time.time()
        self._lastsend_time = int(round(t * 1000))
        self.MULTI = 2
        self._key_press_event = False
        self.x=0
        self.y=0
            
    def _initPresses(self):
        self.pressed["w"] = False
        self.pressed["a"] = False
        self.pressed["s"] = False
        self.pressed["d"] = False
        self.pressed["mouse1"] = False
        self.prevPressed["w"] = False
        self.prevPressed["a"] = False
        self.prevPressed["s"] = False
        self.prevPressed["d"] = False
        self.prevPressed["mouse1"] = False

    def _create_ui(self):
        self.root = tk.Tk()
        self.root.title('RC Controller')
        self.root.geometry('500x500+200+20')
        self._canvas = tk.Canvas(self.root, bg='gray', height=400, width=400)
        #image_file = tk.PhotoImage(file='pic.gif')  # 图片位置（相对路径，与.py文件同一文件夹下，也可以用绝对路径，需要给定图片具体绝对路径）
        #image = canvas.create_image(250, 0, anchor='n',image=image_file)        # 图片锚定点（n图片顶端的中间点位置）放在画布（250,0）坐标处
        #x0, y0, x1, y1 = 100, 100, 150, 150
        #line = self._canvas.create_line(x0-50, y0-50, x1-50, y1-50)                   # 画直线
        self._oval_circle = self._canvas.create_oval(180-2, 180-2, 220+2, 220+2)                             # Circle
        self._oval = self._canvas.create_oval(180, 180, 220, 220, fill='black')               # 画圆 用黄色填充
        self._line = self._canvas.create_line(0, 200, 180-2, 200,dash=(4, 2))                   # 画直线
        self._line = self._canvas.create_line(220+2, 200, 400, 200,dash=(4, 2))                   # 画直线
        self._line = self._canvas.create_line(200, 0, 200, 180-2,dash=(4, 2))                   # 画直线
        self._line = self._canvas.create_line(200, 220+2, 200, 400,dash=(4, 2))                   # 画直线
        #arc = self._canvas.create_arc(x0, y0+50, x1, y1+50, start=0, extent=180)      # 画扇形 从0度打开收到180度结束
        #rect = self._canvas.create_rectangle(330, 30, 330+20, 30+20)                  # 画矩形正方形
        self._canvas.pack()
 
        self._xLabel = tk.Label(self.root, text="RC Controller")
        self._xLabel.pack()

        self._set_bindings()

    def _set_bindings(self):
        for char in ["w","s","d", "a","l", "m"]:
            self.root.bind("<KeyPress-%s>" % char, self._pressed)
            self.root.bind("<KeyRelease-%s>" % char, self._released)
            self.root.bind("<ButtonPress-1>", self._mouse_pressed)
            self.root.bind("<B1-Motion>", self._mouse_move)
            self.root.bind("<ButtonRelease-1>", self._mouse_released)
            self.pressed[char] = False
            
    def start(self):
        self._heartbeat_control_command()
        self.root.mainloop()

    def send_control_command(self, force = False):
        t = time.time()
        tms = int(round(t * 1000))
        if((not force) and (tms - self._lastsend_time) < self._lastsend_interval):
            #print(tms-self._lastsend_time,self._lastsend_interval)
            print('Ignore this command due to too frequently sent commands')
            return
            

        #s.sendto(content, (self._host, self._port))
        command = []
        if(self._key_press_event or self.pressed["d"] or self.pressed["s"] or self.pressed["a"] or self.pressed["w"]):
            self._key_press_event = False
            print("conrtol keys pressed or released")
            command = [commandType.CONTROL,self.x,self.y]    
        elif(self._key_press_event or self.pressed["mouse1"]):
            self._key_press_event = False
            print("control mouse pressed")
            command = [commandType.CONTROL,self.x,self.y]    
        else:
            #print("no keys or mouse pressed")
            return
            
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        strcmd = json.dumps(command)
        s.sendto(strcmd.encode('utf-8'), (self._host, self._port))
        self._lastsend_time = tms


    def _heartbeat_control_command(self):   
        self.send_control_command()
        
        self.root.after(self._lastsend_interval*2, self._heartbeat_control_command)
    
    def _pressed(self, event):
        print("key _pressed",event.char)
        if(self.prevPressed[event.char] == False):
            self._key_press_event = True
        self.prevPressed[event.char] = self.pressed[event.char]
        self.pressed[event.char] = True
        if(self._key_press_event):
            x = 100 if self.pressed["d"] else 0
            x = -100 if self.pressed["a"] else x
            y = 100 if self.pressed["w"] else 0
            y = -100 if self.pressed["s"] else y             
            self.x = x
            self.y = y
            self.send_control_command(True)
            self._xLabel.configure(text=("x,y=",x,y))
            self._canvas.coords(self._oval,self.MULTI*x+180,-self.MULTI*y+180,self.MULTI*x+220,-self.MULTI*y+220)

    def _released(self, event):
        self.prevPressed[event.char] = False
        self.pressed[event.char] = False
        self._key_press_event = True        
        self.x = 0
        self.y = 0
        self.send_control_command(True)        
        self._xLabel.configure(text=("x,y=",self.x,self.y))
        self._canvas.coords(self._oval,self.MULTI*self.x+180,-self.MULTI*self.y+180,self.MULTI*self.x+220,-self.MULTI*self.y+220)
        
        
    def _mouse_pressed(self,event):
        #print("moouse pressed,x,y=",event.x,event.y)
        self.pressed["mouse1"] = True
        self.x_o = event.x
        self.y_o = event.y

    
    def _mouse_move(self, event):
        self.x_n = event.x
        self.y_n = event.y
        
        x = (event.x - self.x_o)/self.MULTI
        y = -(event.y - self.y_o)/self.MULTI
        x = 100 if(x>100) else x
        y = 100 if(y>100) else y
        x = -100 if(x<-100) else x
        y = -100 if(y<-100) else y
        
        self.x = x
        self.y = y
        
        self.send_control_command()
        self._xLabel.configure(text=("x,y=",x,y))
        self._canvas.coords(self._oval,self.MULTI*x+180,-self.MULTI*y+180,self.MULTI*x+220,-self.MULTI*y+220)

    def _mouse_released(self, event):
        self.pressed["mouse1"] = False
        self._key_press_event = True
        self.x_n = self.x_o
        self.y_n = self.y_o    
            
        self.x = 0
        self.y = 0
        
        self.send_control_command(True)        
        self._canvas.coords(self._oval,180,180,220,220)

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], "h:", ["host="])
    except getopt.GetoptError as err:
        print(str(err))
        usage()
        sys.exit(2)
    host = ""
    for o, a in opts:
        if o in ("-h", "--host"):
            host = a

    if host == "":
        print("Did not define host, use -h or --host to pass the host name of the car")
        sys.exit(2)

    p = Controller(host)
    p.start()

def usage():
    print("Available options:")
    print("-h, --host  Define RC car IP address")


if __name__ == "__main__":
    main()
