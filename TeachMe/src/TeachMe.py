#!/usr/bin/env python
import cPickle as cp
import numpy as np
import glob
import math
import os
import time, datetime
import hashlib
import random

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.graphics import Color, Ellipse, Line, Rectangle
from kivy.properties import ObjectProperty, BooleanProperty, StringProperty
from kivy.uix.screenmanager import ScreenManager, Screen, FadeTransition, FallOutTransition, NoTransition
from kivy.uix.boxlayout import BoxLayout
from kivy.garden.graph import Graph, LinePlot
from kivy.clock import Clock

from kivy.logger import Logger

import logging
Logger.setLevel(logging.DEBUG)

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nao_writing_msgs.msg import MultiPaths

class PainterBox(Widget):
    btn = ObjectProperty()


    def __init__(self, **kwargs):
        super(PainterBox, self).__init__(**kwargs)
        self.color = (random.random(), 1., 1.)
        self.lines = []
        self.starttime = None
        self.timing = []

    def on_touch_down(self, touch):

        if not self.collide_point(touch.x, touch.y):
            return

        with self.canvas:
            Color(*self.color, mode ="hsv")
            d = 30.
            self.lines.append(Line(points=(touch.x, touch.y), width=3))
            self.starttime = time.time()
            self.timing.append([0])
            touch.ud["owner"] = self
            #self.btn.state = 'down'
            self.btn.disabled = False
            self.btn.background_normal = "eraser.png"


    def on_touch_move(self, touch):
        if not self.collide_point(touch.x, touch.y):
            return
        self.lines[-1].points += (touch.x, touch.y)
        self.timing[-1].append(int((time.time() - self.starttime) * 1000))
        self.save()

    def on_touch_up(self, touch):
        if not self.collide_point(touch.x, touch.y):
            return

        self.save()

    def save(self):
        # filename = "%s" % (hashlib.sha1(str(id(self)) + str(datetime.datetime.now())).hexdigest()[:6])
        # fullpath = os.path.join(app.user_data_dir, filename)
        # Logger.info("Saving drawing in %s " % fullpath)
        # with open(fullpath, 'w') as data:
        #         data.write("# date: %s\n" % datetime.datetime.now())
        #         data.write("# timestamp (ms) --- x (px) --- y (px)\n")
        #         for i, t in enumerate(self.timing):
        #             data.write("%d\t%.2f\t%.2f\n" % (t,
        #                         self.line.points[2 * i] - self.x,
        #                         self.line.points[2 * i + 1] - self.y))
        return

    def draw_background(self, arg1 = None, arg2 = None):

        with self.canvas:
            Color(1, 1, 1, 1)
            self.rect = Rectangle(pos= self.pos,  size= self.size)

        self.bind(pos=self.update_rect,size=self.update_rect)

    def update_rect(self, *args):
        self.rect.pos = self.pos
        self.rect.size = self.size


    def clear_letter(self,*args):
        self.color = (random.random(), 1., 1.)
        self.canvas.clear()
        self.draw_background()
        self.lines = []
        self.timing = []

class FinalScreen(Screen):
    pass

class DrawScreen(Screen):

    def __init__(self, **kwargs):
        super(DrawScreen, self).__init__(**kwargs)

        self.graph_layout = BoxLayout(orientation = 'vertical',pos_hint = {'center_y': .55, 'center_x': .25 },
        size_hint= (.45, .85), spacing = 10)


        self.draw_layout  = BoxLayout(orientation = 'vertical',pos_hint = {'center_y': .55, 'center_x': .75},
        size_hint= (.45, .85), color = [.5,.5,.5,1], spacing = 10)

        self.button_layout = BoxLayout(orientation = 'horizontal',pos_hint = {'center_y': .0001, 'center_x': .5},
        size_hint= (.9, .1), color = [.5,.5,.5,1], spacing = 100)

        self.add_widget(self.graph_layout)
        self.add_widget(self.draw_layout)
        self.add_widget(self.button_layout)

        self.plots   = []
        self.anim_counter = 0

        self.graph = Graph(draw_border = False, background_color = [1, 1, 1, 1], border_color = [.3, .5, .6, .7])
        self.graph.pos_hint={'center_y': .5}
        self.graph.size_hint_y = .9

        clear_graph_btn_image = os.path.join(app.resource_folder, 'eraser.png')
        self.clear_graph = Button(background_normal = clear_graph_btn_image, border = [1,1,1,1],
        size_hint=(.00000001, .5),
        pos_hint = {'center_y': 1, 'center_x': .4 })
        self.clear_graph.opacity = 1
        self.clear_graph.disabled = False
        self.clear_graph.bind(on_press = self.clear_plot)


        self.gap_btn1 = Button(border = [1,1,1,1],
        size_hint=(.00000005, .5),
        pos_hint = {'center_y': 1, 'center_x': .4 })
        self.gap_btn1.opacity = 0
        self.gap_btn1.disabled = True

        self.gap_btn2 = Button(border = [1,1,1,1],
        size_hint=(.00000005, .5),
        pos_hint = {'center_y': 1, 'center_x': .4 })
        self.gap_btn2.opacity = 0
        self.gap_btn2.disabled = True

        next_btn_image = os.path.join(app.resource_folder, 'next.png')
        self.next_graph = Button(background_normal = next_btn_image, border = [1,1,1,1],
        size_hint=(.00000001, .7),
        pos_hint = {'center_y': 1, 'center_x': .5 })
        self.next_graph.opacity = 1
        self.next_graph.disabled = False
        self.next_graph.bind(on_press = self.next_plot)

        eraser_btn_image = os.path.join(app.resource_folder, 'eraser.png')
        self.eraser = Button(background_disabled_normal = eraser_btn_image,background_normal = eraser_btn_image, border = [1,1,1,1],
        size_hint=(.00000001, .5), pos_hint = {'center_y': 1, 'center_x': .8 })
        self.eraser.opacity = 1
        self.eraser.disabled = True

        # self.animate_btn = Button(background_normal = 'next.png', border = [1,1,1,1],
        # size_hint=(.00000001, .4),
        # pos_hint = {'center_y': 1, 'center_x': .5 })
        # self.animate_btn.opacity = 1
        # self.animate_btn.disabled = False
        # self.animate_btn.bind(on_press=self.start_animation)

        self.painterbox = PainterBox(btn = self.eraser)


        self.eraser.bind(on_press=self.painterbox.clear_letter)

        child_btn_image = os.path.join(app.resource_folder, 'child.png')
        self.child_btn = Button(background_disabled_normal = child_btn_image, background_normal = child_btn_image, border = [1,1,1,1],
        size_hint=(.1, .1), pos_hint = {'center_y': .9, 'center_x': .5 })
        self.child_btn.opacity = 1
        self.child_btn.disabled = False
        self.child_btn.bind(on_press=self.start_animation)

        nao_btn_image = os.path.join(app.resource_folder, 'nao.png')
        self.nao_btn = Button(background_disabled_normal = nao_btn_image, background_normal = nao_btn_image, border = [1,1,1,1],
        size_hint=(.1, .1), pos_hint = {'center_y': .9, 'center_x': .5 })
        self.nao_btn.opacity = 1
        self.nao_btn.disabled = False
        self.nao_btn.bind(on_press=self.on_nao_turn)

        self.graph_layout.add_widget(self.nao_btn)
        self.graph_layout.add_widget(self.graph)

        self.button_layout.add_widget(self.clear_graph)
        # self.button_layout.add_widget(self.animate_btn)
        self.button_layout.add_widget(self.gap_btn1)
        self.button_layout.add_widget(self.next_graph)
        self.button_layout.add_widget(self.gap_btn2)
        self.button_layout.add_widget(self.eraser)

        self.draw_layout.add_widget(self.child_btn)
        self.draw_layout.add_widget(self.painterbox)

        return

    def on_nao_turn(self, *args):
        if self.on_send_char is not None:
            #call this
            self.on_send_char()
        return

    def create_plot(self, *args):
        return LinePlot(color=[0,  0.443, 0.737, 1], line_width = 3)  # orange

    def clear_plot(self, *args):
        # self.graph.remove_plot(self.plot)
        # self.graph.remove_plot(self.plot1)
        for plot in self.plots:
            self.graph.remove_plot(plot)
        self.plots = []
        return

    def next_plot(self, *args):
        self.clear_plot()
        self.painterbox.clear_letter()
        self.eraser.disabled = True
        app.counter += 1

        if app.counter > 14 or app.counter >= len(self.keys):
            app.sm.current = 'fs'
        else:
            self.curr_char = self.keys[app.counter]
            self.curr_data = self.data[self.curr_char][0]
            self.draw(self.curr_data)
        return


    def on_enter(self):
        data_file = os.path.join(app.resource_folder, 'file_p', app.file_p)
        self.data = cp.load(open(data_file, 'rb'))
        self.keys = self.data.keys()
        self.curr_char = self.keys[0]
        self.curr_data = self.data[self.curr_char][5]
        self.draw(self.curr_data)
        return

    def max_min_range(self, data):

        d = 0
        x_ma = []
        y_mi = []
        x_mi = []
        y_ma = []
        for stroke in data:
            #print stroke
            x_max = x_ma.append(int(round(max(stroke[ :, 0]) + d)))
            y_max = y_ma.append(int(round(max(stroke[ :, 1]) + d)))
            x_min = x_mi.append(int(round(min(stroke[ :, 0]) + d)))
            y_min = y_mi.append(int(round(min(stroke[ :, 1]) + d)))
        x_max = max(x_ma)
        y_max = max(y_ma)
        x_min = min(x_mi)
        y_min = min(y_mi)

        return x_max, y_max, x_min, y_min
    def set_graph_margin(self, strokes):
        self.x_max, self.y_max, self.x_min, self.y_min = self.max_min_range(strokes)
        x_margin = round(np.abs(self.x_max - self.x_min) * 0.2)
        y_margin = round(np.abs(self.y_max - self.y_min) * 0.2)
        x_margin = 1 if x_margin < 1 else x_margin
        y_margin = 1 if y_margin < 1 else y_margin
        a = max([x_margin, y_margin])
        self.graph.xmin = self.x_min -a
        self.graph.xmax = self.x_max +a
        self.graph.ymin = self.y_max +a
        self.graph.ymax = self.y_min -a

        # print self.x_max, self.y_max, self.x_min, self.y_min
        return

    def draw(self, strokes):
        self.clear_plot()
        self.set_graph_margin(strokes)
        # process each stroke
        for s in strokes:
            self.plots.append(self.create_plot())
            self.plots[-1].points = s
            self.graph.add_plot(self.plots[-1])
        return

    def update(self, stroke):
        """
        update the last stroke plot with the given stroke, if there's currently no stroke, create one
        """
        if not self.plots:
            self.plots.append(self.create_plot())
            self.plots[-1].points = stroke
            self.graph.add_plot(self.plots[-1])
        else:
            #update the last stroke
            self.plots[-1].points = stroke

        return

    def start_animation(self, *args):
        self.animate(strokes=self.curr_data, wait=0.5, freq=20, strk_interval=1.0)
        return

    def animate(self, strokes, wait=0.5, freq=20, strk_interval=1.0):
        """
        Animating the stroke trajectories with given settings:
        wait - how long to wait before start the animating: unit - sec
        freq - how often to update the plot
        strk_interval - interval between strokes
        """
        self.set_graph_margin(strokes)
        self.clear_plot()
        # process each stroke
        self.anim_counter = 0
        self.anim_counter_strk_len = np.array([len(s) for s in strokes])
        self.anim_wait = wait
        self.anim_freq = freq
        self.anim_strk_interval = strk_interval
        self.anim_n_strks = len(strokes)
        self.anim_strk_idx = 0
        self.anim_strokes = strokes

        def schedule_stroke_handler(dt):
            Clock.schedule_interval(animate_stroke_handler, 1. / self.anim_freq)
            return False

        def animate_stroke_handler(dt):
            #check if we are at the end of te stroke
            if self.anim_counter >= self.anim_counter_strk_len[self.anim_strk_idx]:
                #close the schedule if reached the last point
                if self.anim_strk_idx >= self.anim_n_strks - 1:
                    return False
                else:
                    #refresh counter and move to the next stroke
                    self.anim_counter = 0
                    self.anim_strk_idx += 1
                    #also remember to add a new plot to this stroke
                    self.plots.append(self.create_plot())
                    self.graph.add_plot(self.plots[-1])

                    Clock.schedule_once(schedule_stroke_handler, self.anim_strk_interval)
                    return False
            #process the plot update
            stroke_update = self.anim_strokes[self.anim_strk_idx][:(self.anim_counter+1), :]
            self.update(stroke_update)
            #increase the counter
            self.anim_counter += 1
            return

        #start the schedule after specified waitting time
        Clock.schedule_once(schedule_stroke_handler, self.anim_wait)
        return


class TeachMeApp(App):

    def __init__(self, **kwargs):
        super(TeachMeApp, self).__init__(**kwargs)

        self.file_p = None
        self.file_mdl = None
        self.counter = None

    def build(self):

        self.counter = 0

        pkg_path = rospkg.RosPack().get_path('TeachMe')
        self.resource_folder = os.path.join(pkg_path, 'res')
        self.file_p = 'uji_data_normed.p'

        # print self.file_p

        self.sm =ScreenManager()

        self.drawscreen = DrawScreen(name = 'ds')
        self.sm.add_widget(self.drawscreen)

        self.finalscreen = FinalScreen(name = 'fs')
        self.sm.add_widget(self.finalscreen)

        self.sm.current = 'ds'

        self.drawscreen.on_send_char = self.on_send_char

        return self.sm

    def on_receive_char(self, msg):
        #update the curr_char of drawscreen
        rospy.loginfo('GAIPS_TEACH_ME: Received character data...')
        received_char = []
        # <hyin/Mar-31st-2016> note the message structure, Kivy/ROS seems to eat the exception throwed
        # also it is hard to observe this due to the absence of type checkin in compiling time
        for path in msg.paths:
            stroke = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in path.poses])
            received_char.append(stroke)
        self.drawscreen.curr_data = received_char
        self.drawscreen.start_animation()
        return

    def on_send_char(self):
        #publish the char on the topic, this will be assigned to DrawScreen as a callback
        #to allow the widget to call it

        return
        
if __name__ == '__main__':
    app = TeachMeApp()

    #initialize a ros node for receiving/dispatching trajectories
    #<hyin/Mar-31st-2016> note to put ros stuff outside the GUI
    #unlike C++, python seems to have a weird behavior on global blocking of threads
    #thus it might be problematic to have the node created within the GUI
    ros_node = rospy.init_node('gaips_teach_me')
    ros_pub_topic = '/nao_writing/child_character'
    ros_sub_topic = '/nao_writing/nao_character'
    app.ros_subscriber = rospy.Subscriber(ros_sub_topic, MultiPaths, app.on_receive_char)
    app.ros_publisher = rospy.Publisher(ros_pub_topic, MultiPaths, queue_size=10)

    app.run()
