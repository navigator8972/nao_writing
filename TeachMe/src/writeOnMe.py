
import random
import os
import time, datetime 
import hashlib

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.graphics import Color, Ellipse, Line
from kivy.properties import ObjectProperty, BooleanProperty, StringProperty

from kivy.logger import Logger

import logging
Logger.setLevel(logging.DEBUG)


class PainterBox(Widget):
    btn = ObjectProperty()
    
    def __init__(self, **kwargs):
        super(PainterBox, self).__init__(**kwargs)
        self.color = (random.random(), 1., 1.)
        self.line = None
        self.starttime = None
        self.timing = None
    
    def on_touch_down(self, touch):
         
        with self.canvas:
            Color(*self.color, mode ="hsv")    
            d = 30.
            self.line = Line(points=(touch.x, touch.y), width=3)          
            self.starttime = time.time()
            self.timing = [0]
            touch.ud["owner"] = self
            self.btn.state = 'down'
            self.btn.disabled = False 
    def on_touch_move(self, touch):
        self.line.points += (touch.x, touch.y)        
        self.timing.append(int((time.time() - self.starttime) * 1000))
        self.save()
        
    def on_touch_up(self, touch):
        if not self.collide_point(touch.x, touch.y):
            return
        self.save() 
        
    def save(self): 
        filename = "%s" % (hashlib.sha1(str(id(self)) + str(datetime.datetime.now())).hexdigest()[:6])
        fullpath = os.path.join(app.user_data_dir, filename)
        Logger.info("Saving drawing in %s " % fullpath)
        with open(fullpath, 'w') as data:                
                data.write("# date: %s\n" % datetime.datetime.now())                
                data.write("# timestamp (ms) --- x (px) --- y (px)\n")
                for i, t in enumerate(self.timing):
                    data.write("%d\t%.2f\t%.2f\n" % (t, 
                                self.line.points[2 * i] - self.x,
                                self.line.points[2 * i + 1] - self.y))  
 

class DrawLetterApp(App):
    
    def build(self):
        parent = Widget()
        self.clearbtn = Button(text='Clear')
        self.painter = PainterBox(btn = self.clearbtn)
        self.clearbtn.bind(on_release=self.clear_canvas)
        self.clearbtn.disabled = True
        parent.add_widget(self.painter)
        parent.add_widget(self.clearbtn)
        return parent

    def clear_canvas(self, obj):
        self.painter.canvas.clear()


if __name__ == '__main__':
    app = DrawLetterApp()
    app.run()
