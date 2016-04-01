#!/usr/bin/env python
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.pyplot import cm
matplotlib.rc('xtick', labelsize=8)
matplotlib.rc('ytick', labelsize=8)

import sys, os, random
import copy
import numpy as np
import cPickle as cp

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nao_writing_msgs.msg import MultiPaths

import pytrajkin as pytk
import utils

class PyTrajKin_GUI(QMainWindow):

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('PyTrajKin_GUI - PyQt4')
        #size
        self.resize(1024, 768)
        self.move(400, 200)

        self.create_menu()
        self.create_main_frame()
        self.create_status_bar()
        self.create_action()
        self.main_frame.show()
        self.disable_items()

    def create_menu(self):
        return

    def create_main_frame(self):
        self.main_frame = QWidget()

        hbox = QHBoxLayout()
        self.main_hbox = hbox
        vbox_ctrl_pnl = QVBoxLayout()
        vbox_ctrl_pnl.setAlignment(Qt.AlignTop)
        vbox_fig = QVBoxLayout()

        #for control panel
        #load button
        self.load_btn = QPushButton('Load')
        vbox_ctrl_pnl.addWidget(self.load_btn)
        #combo for letters
        self.char_lbl = QLabel('Characters')
        self.char_combbox = QComboBox()
        vbox_ctrl_pnl.addWidget(self.char_lbl)
        vbox_ctrl_pnl.addWidget(self.char_combbox)

        #button for train
        self.train_btn = QPushButton('Train')
        self.send_btn = QPushButton('Send')

        vbox_ctrl_pnl.addWidget(self.train_btn)
        vbox_ctrl_pnl.addWidget(self.send_btn)

        #<hyin/Feb-09-2015> add tabs...
        self.tab_main = QTabWidget()
        self.tab_char = QWidget()
        self.tab_main.addTab(self.tab_char, 'Character')

        #for drawing part
        fig_hbox = QHBoxLayout()
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        #canvas
        gs = plt.GridSpec(4, 2)
        self.ax_char_prf = self.fig.add_subplot(gs[:, 0])
        self.ax_xvel = self.fig.add_subplot(gs[0, 1])
        self.ax_yvel = self.fig.add_subplot(gs[1, 1])
        self.ax_vel_prf = self.fig.add_subplot(gs[2, 1])
        self.ax_ang_prf = self.fig.add_subplot(gs[3, 1])

        self.ax_char_prf.hold(False)
        self.ax_xvel.hold(False)
        self.ax_yvel.hold(False)
        self.ax_vel_prf.hold(False)
        self.ax_ang_prf.hold(False)

        self.ax_char_prf.set_aspect('equal')

        self.fig.tight_layout()
        fig_hbox.addWidget(self.canvas)

        #control panel for char...
        char_ctrl_pnl = QHBoxLayout()

        char_ctrl_pnl_vlayout = QVBoxLayout()
        self.idx_lbl = QLabel('Character Index')
        self.idx_combbox = QComboBox()

        self.strk_lbl = QLabel('Stroke Index')
        self.strk_combbox = QComboBox()
        #
        # self.rand_btn = QPushButton('Synthesize')

        #tab for strokes...
        #at least there's one...
        self.tab_char_strk = QTabWidget()

        self.parm_sliders_layout = []
        self.parms_sliders = []
        self.strk_tab_wgt_lst_array = []

        char_ctrl_pnl_vlayout.addWidget(self.idx_lbl)
        char_ctrl_pnl_vlayout.addWidget(self.idx_combbox)
        char_ctrl_pnl_vlayout.addWidget(self.strk_lbl)
        char_ctrl_pnl_vlayout.addWidget(self.strk_combbox)
        # char_ctrl_pnl_vlayout.addWidget(self.rand_btn, 20)

        char_ctrl_pnl.addLayout(char_ctrl_pnl_vlayout)
        char_ctrl_pnl.addWidget(self.tab_char_strk, 7)

        vbox_fig.addLayout(fig_hbox, 5)
        vbox_fig.addLayout(char_ctrl_pnl, 2)

        self.tab_char.setLayout(vbox_fig)
        #add layouts
        hbox.addLayout(vbox_ctrl_pnl, 1)
        #hbox.addLayout(vbox_fig, 5)
        hbox.addWidget(self.tab_main, 5)

        self.main_frame.setLayout(hbox)
        self.setCentralWidget(self.main_frame)
        return

    def create_status_bar(self):
        return

    def create_action(self):
        self.load_btn.clicked.connect(self.on_load_data)

        self.char_combbox.currentIndexChanged.connect(self.on_update_char_comb)
        self.idx_combbox.currentIndexChanged.connect(self.on_update_idx_comb)
        self.strk_combbox.currentIndexChanged.connect(self.on_update_strk_comb)

        self.train_btn.clicked.connect(self.on_train)
        self.send_btn.clicked.connect(self.on_send_char)

        # <hyin/Mar-31st-2016> disable unused buttons...
        # self.train_char_btn.clicked.connect(self.on_train_char)
        # self.train_all_btn.clicked.connect(self.on_train_all)

        # self.save_all_btn.clicked.connect(self.on_save_all)
        return

    def disable_items(self):
        return

    def on_load_data(self):

        #<hyin/Feb-11-2015> structure to store trained feature parms...
        self.data = None
        self.data_feats = None

        fileName = QFileDialog.getOpenFileName(self, 'Open', self.resource_folder, selectedFilter='*.p')
        if fileName:
            self.data = cp.load(open(fileName, 'rb'))
            print 'Loaded from to {0}'.format(fileName)
        else:
            return

        #refresh char comb
        self.char_combbox.blockSignals(True)
        self.char_combbox.clear()

        #only add valid letters...
        for key in self.data:
            if len(key) > 1:
                continue
            if (ord(key)<=ord('z') and ord(key)>=ord('a')) or (ord(key)<=ord('Z') and ord(key)>=ord('A')):
                self.char_combbox.addItem(key)

        self.char_mdl = []
        self.dt = 0.01
        self.char_combbox.blockSignals(False)
        self.on_update_char_comb(None)

        return

    def on_update_char_comb(self, idx):
        #refresh idx comb
        curr_char = str(self.char_combbox.currentText())
        self.idx_combbox.blockSignals(True)
        self.idx_combbox.clear()
        self.idx_combbox.addItems(map(str, range(len(self.data[curr_char]))))
        self.idx_combbox.blockSignals(False)

        self.on_update_idx_comb(None)

        return

    def on_update_idx_comb(self, idx):
        #release mdl
        self.char_mdl = []
        curr_char = str(self.char_combbox.currentText())
        curr_idx = int(self.idx_combbox.currentText())
        #check if trained feature parameters are there
        if self.data_feats is not None:
            if curr_char in self.data_feats:
                if len(self.data_feats[curr_char]) > curr_idx:
                    self.char_mdl = self.data_feats[curr_char][curr_idx]

        self.strk_combbox.blockSignals(True)
        self.strk_combbox.clear()
        self.strk_combbox.addItems(map(str, range(len(self.data[curr_char][curr_idx]))))
        self.strk_combbox.blockSignals(False)
        self.clear_parm_sliders_layout()
        #self.on_update_strk_comb(None)

        self.clear_parm_sliders_layout()
        self.populate_parm_sliders()

        self.plot_data()
        return

    def on_update_strk_comb(self, idx):
        self.refresh_parm_sliders_layout()
        return

    def on_train(self):
        #train model with current data
        curr_data = self.get_current_data()
        self.char_mdl = []
        #print curr_data
        for stroke in curr_data:
            #print 'training...'
            tmp_stroke_dict = dict()
            tmp_mdl = pytk.TrajKinMdl()
            tmp_mdl.x0 = stroke[0, 0]
            tmp_mdl.y0 = stroke[0, 1]
            tmp_vel_prf = tmp_mdl.get_vel_profile(stroke)/self.dt

            tmp_opt_parm, tmp_reg_pnts_array = tmp_mdl.train(stroke)
            #fill dict
            #tmp_stroke_dict['model'] = tmp_mdl
            tmp_stroke_dict['vel_profile'] = tmp_vel_prf
            tmp_stroke_dict['reg_pnts_array'] = tmp_reg_pnts_array
            #tmp_stroke_dict['init_guess'] = tmp_init_guess
            tmp_stroke_dict['opt_parms'] = tmp_opt_parm
            tmp_stroke_dict['start_pnt'] = stroke[0, :]

            self.char_mdl.append(tmp_stroke_dict)
        #print self.char_mdl
        #populate parms sliders & plot
        self.clear_parm_sliders_layout()
        self.populate_parm_sliders()

        self.plot_data()
        return

    def on_train_char(self):
        #print 'Train Char Button Clicked'
        """
        train current Character
        """
        curr_char = str(self.char_combbox.currentText())
        curr_char_data_lst = self.data[curr_char]
        print '========================================'
        print 'Training Character {0}...'.format(curr_char)
        print '========================================'
        char_data_feats_lst = []
        i = 0
        for char_data in curr_char_data_lst:
            print '==========================================='
            print 'Training the {0}-th character...'.format(i)
            print '==========================================='
            char_data_feats = []
            for stroke in char_data:
                tmp_stroke_dict = dict()
                tmp_mdl = pytk.TrajKinMdl()
                tmp_mdl.x0 = stroke[0, 0]
                tmp_mdl.y0 = stroke[0, 1]
                tmp_vel_prf = tmp_mdl.get_vel_profile(stroke)/self.dt

                tmp_opt_parm, tmp_reg_pnts_array = tmp_mdl.train(stroke)
                #fill dict
                #tmp_stroke_dict['model'] = tmp_mdl
                tmp_stroke_dict['vel_profile'] = tmp_vel_prf
                tmp_stroke_dict['reg_pnts_array'] = tmp_reg_pnts_array
                #tmp_stroke_dict['init_guess'] = tmp_init_guess
                tmp_stroke_dict['opt_parms'] = tmp_opt_parm
                tmp_stroke_dict['start_pnt'] = stroke[0, :]
                tmp_stroke_dict['pos_traj'] = copy.copy(stroke)

                #push back trained data
                char_data_feats.append(tmp_stroke_dict)

            char_data_feats_lst.append(char_data_feats)
            i+=1

        print '========================================'
        print 'Finish Training Character {0}.'.format(curr_char)
        print '========================================'

        #write to record
        if self.data_feats is None:
            #create new one
            self.data_feats = dict()
        else:
            pass

        self.data_feats[curr_char] = char_data_feats_lst

        select_idx = self.idx_combbox.currentIndex()
        self.char_mdl = self.data_feats[curr_char][select_idx]

        #populate parms sliders & plot
        self.clear_parm_sliders_layout()
        self.populate_parm_sliders()

        self.populate_statistics()

        self.plot_data()
        return

    def on_train_all(self):
        print 'Train All Button Clicked'
        return

    def on_save_all(self):
        curr_dir = os.path.dirname(os.path.realpath(__file__))
        #construct data
        tmp_dict = dict()
        tmp_dict['data_feats'] = self.data_feats
        tmp_dict['data_feats_stat'] = self.data_feats_stat

        default_dir = os.path.join(curr_dir, 'data')
        fileName = QFileDialog.getSaveFileName(self, 'Save', default_dir, selectedFilter='*.p')
        if fileName:
            cp.dump(tmp_dict, open(fileName, 'wb'))
            print 'Saved to {0}'.format(fileName)
        return

    def refresh_parm_sliders_layout(self):
        #get current stroke
        if not self.char_mdl:
            return
        curr_idx = int(self.strk_combbox.currentText())
        #note clear does not delete the widgets...
        self.tab_char_strk.clear()
        for i in range(len(self.strk_tab_wgt_lst_array[curr_idx])):
            self.tab_char_strk.addTab(self.strk_tab_wgt_lst_array[curr_idx][i], 'Comp {0}'.format(i))
        return

    def clear_parm_sliders_layout(self):
        #clean layout, see relevant stackoverflow threads
        for stroke_layouts in self.parm_sliders_layout:
            for layout in stroke_layouts:
                for i in reversed(range(layout.count())):
                    widgetToRemove = layout.itemAt( i ).widget()
                    # get it out of the layout list
                    layout.removeWidget( widgetToRemove )
                    # remove it form the gui
                    widgetToRemove.setParent( None )

        self.parm_sliders_layout = []
        for i in reversed(range(self.tab_char_strk.count())):
            widgetToRemove = self.tab_char_strk.widget(i)
            widgetToRemove.setParent( None )
            widgetToRemove.deleteLater()
        self.tab_char_strk.clear()
        self.parms_sliders = []
        self.strk_tab_wgt_lst_array = []
        return

    def populate_parm_sliders(self):
        if not self.char_mdl:
            return
        select_idx = self.strk_combbox.currentIndex()
        for curr_idx in range(len(self.char_mdl)):
            #check model parms, now only deal with the first stroke
            #for each component, interested parameters are : D, mu, sig, delta_theta

            tmp_effective_parms = np.array(self.char_mdl[curr_idx]['opt_parms'])[:, [0, 2,3,5]]
            i = 0
            tmp_strk_layout_lst = []
            tmp_strk_parm_slider_lst = []
            tmp_strk_tab_wgt_lst = []
            for stroke_parm in tmp_effective_parms:
                tmp_slider_lbl = QLabel('D, mu, sig, delta_theta')
                tmp_parm_sliders_layout = QHBoxLayout()
                tmp_parm_sliders_lst = []
                for tmp_parm in stroke_parm:
                    tmp_parm_slider = QSlider()
                    tmp_parm_slider.setOrientation(Qt.Vertical)
                    tmp_parm_slider.setMaximum(100)
                    tmp_parm_slider.setValue(50)
                    #connect message to plot event
                    tmp_parm_slider.valueChanged.connect(self.plot_data)
                    # self.parms_sliders.append(tmp_parm_slider)
                    # self.parm_sliders_layout.addWidget(tmp_parm_slider)
                    tmp_parm_sliders_lst.append(tmp_parm_slider)
                    tmp_parm_sliders_layout.addWidget(tmp_parm_slider)
                tmp_strk_slider_wgt = QWidget()
                tmp_strk_slider_wgt.setLayout(tmp_parm_sliders_layout)
                if select_idx == curr_idx:
                    self.tab_char_strk.addTab(tmp_strk_slider_wgt, 'Comp {0}'.format(i))
                tmp_strk_layout_lst.append(tmp_parm_sliders_layout)
                tmp_strk_parm_slider_lst.append(tmp_parm_sliders_lst)
                tmp_strk_tab_wgt_lst.append(tmp_strk_slider_wgt)
                i+=1

            self.parm_sliders_layout.append(tmp_strk_layout_lst)
            self.parms_sliders.append(tmp_strk_parm_slider_lst)
            self.strk_tab_wgt_lst_array.append(tmp_strk_tab_wgt_lst)

        return

    def get_perturbed_trajectory_and_parms(self, strk_idx):
        #registration points...
        mdl = pytk.TrajKinMdl()
        mdl.x0 = self.char_mdl[strk_idx]['start_pnt'][0]
        mdl.y0 = self.char_mdl[strk_idx]['start_pnt'][1]
        mdl.mdl_parms_ = self.char_mdl[strk_idx]['opt_parms']
        vel_profile = self.char_mdl[strk_idx]['vel_profile']
        reg_pnts_array = self.char_mdl[strk_idx]['reg_pnts_array']
        t_array = np.arange(len(vel_profile))*self.dt

        #opt
        #get noise
        num_parm_per_comp = 4
        noise_ratio_array = []

        for slider_lst in self.parms_sliders[strk_idx]:
            for slider in slider_lst:
                noise_ratio_array.append(float((slider.value()-50))/100)

        noise_ratio_array = np.reshape(noise_ratio_array, (-1, num_parm_per_comp))
        opt_parms = np.array(self.char_mdl[strk_idx]['opt_parms'])
        for row in range(opt_parms.shape[0]):
            opt_parms[row][0] += noise_ratio_array[row][0] * np.abs(opt_parms[row][0]) * 0.8
            opt_parms[row][2] += noise_ratio_array[row][1] * np.abs(opt_parms[row][2]) * 0.5
            opt_parms[row][3] += noise_ratio_array[row][2] * np.abs(opt_parms[row][3]) * 0.5
            #theta_s & theta_e: noise is applied to delta_theta
            opt_theta_s = opt_parms[row][4]
            opt_theta_e = opt_parms[row][5]
            opt_parms[row][4] = (opt_theta_s + opt_theta_e)/2 - (opt_theta_e-opt_theta_s) * (1 + noise_ratio_array[row][3]*2) / 2
            opt_parms[row][5] = (opt_theta_s + opt_theta_e)/2 + (opt_theta_e-opt_theta_s) * (1 + noise_ratio_array[row][3]*2) / 2
        traj_opt, vel_vec_opt = mdl.eval(t_array, opt_parms)
        theta_opt = utils.get_continuous_ang(traj_opt)
        return mdl, t_array, opt_parms, traj_opt, vel_vec_opt, theta_opt

    def plot_data(self):
        #plot data
        curr_data = self.get_current_data()
        bFirstStroke = True
        last_stroke_end_t = 0.0
        #currently, only consider one stroke
        for stroke in curr_data:
            #vel profile
            vel_profile = utils.get_vel_profile(stroke)/self.dt
            #t_array
            t_array = np.arange(len(vel_profile))*self.dt + last_stroke_end_t
            last_stroke_end_t = t_array[-1]
            #vel vec & theta
            vel_vec = np.diff(stroke, axis=0)/self.dt
            #theta = np.arctan2(vel_vec[:, 1], vel_vec[:, 0])

            theta = utils.get_continuous_ang(stroke)

            #plot
            #only data
            #char profile
            self.ax_char_prf.plot(stroke[:, 0], -stroke[:, 1], 'b')
            self.ax_char_prf.set_title('Character Profile', fontsize=8)
            self.ax_char_prf.set_xticks([])
            self.ax_char_prf.set_yticks([])
            #vel_x & vel_y
            self.ax_xvel.plot(t_array, vel_vec[:, 0], 'b')
            self.ax_xvel.set_title('X Velocity', fontsize=8)
            self.ax_xvel.set_xlabel('Time (s)', fontsize=8)
            self.ax_xvel.set_ylabel('Velocity (Unit/s)', fontsize=8)
            self.ax_yvel.plot(t_array, vel_vec[:, 1], 'b')
            self.ax_yvel.set_title('Y Velocity', fontsize=8)
            self.ax_yvel.set_xlabel('Time (s)', fontsize=8)
            self.ax_yvel.set_ylabel('Velocity (Unit/s)', fontsize=8)
            #vel profile
            self.ax_vel_prf.plot(t_array, vel_profile, 'b')
            self.ax_vel_prf.set_title('Velocity Maganitude', fontsize=8)
            self.ax_vel_prf.set_xlabel('Time (s)', fontsize=8)
            self.ax_vel_prf.set_ylabel('Maganitude (Unit/s)', fontsize=8)
            #ang profile
            self.ax_ang_prf.plot(t_array, theta, 'b')
            self.ax_ang_prf.set_title('Angular Position', fontsize=8)
            self.ax_ang_prf.set_xlabel('Time (s)', fontsize=8)
            self.ax_ang_prf.set_ylabel('Angular Position (rad)', fontsize=8)
            if bFirstStroke:
                self.ax_char_prf.hold(True)
                self.ax_xvel.hold(True)
                self.ax_yvel.hold(True)
                self.ax_vel_prf.hold(True)
                self.ax_ang_prf.hold(True)

                bFirstStroke = False

        colors = ['r', 'y', 'k', 'g', 'w']
        last_stroke_end_t = 0.0
        for curr_idx in range(len(self.char_mdl)):
            #hold current drawings to add new curves
            mdl, t_array, opt_parms, traj_opt, vel_vec_opt, theta_opt  = self.get_perturbed_trajectory_and_parms(curr_idx)

            self.ax_char_prf.plot(traj_opt[:, 0], -traj_opt[:, 1], 'r')
            self.ax_vel_prf.plot(t_array[:]+last_stroke_end_t, np.sum(vel_vec_opt**2, axis=1)**(1./2), 'r')

            self.ax_xvel.plot(t_array[:]+last_stroke_end_t, vel_vec_opt[:, 0], 'r')
            self.ax_yvel.plot(t_array[:]+last_stroke_end_t, vel_vec_opt[:, 1], 'r')
            self.ax_ang_prf.plot(t_array[1:]+last_stroke_end_t, theta_opt, 'r')

            #for each component
            for parm in opt_parms:
                comp_traj, comp_vel_vec = mdl.eval(t_array, [parm])
                self.ax_vel_prf.plot(t_array[:]+last_stroke_end_t, np.sum(comp_vel_vec**2, axis=1)**(1./2), 'g')
                self.ax_xvel.plot(t_array[:]+last_stroke_end_t, comp_vel_vec[:, 0], 'g')
                self.ax_yvel.plot(t_array[:]+last_stroke_end_t, comp_vel_vec[:, 1], 'g')
            last_stroke_end_t += t_array[-1]

        self.ax_char_prf.hold(False)
        self.ax_xvel.hold(False)
        self.ax_yvel.hold(False)
        self.ax_vel_prf.hold(False)
        self.ax_ang_prf.hold(False)
        self.canvas.draw()
        return

    def get_current_data(self):
        curr_char = str(self.char_combbox.currentText())
        curr_idx = self.idx_combbox.currentIndex()
        curr_data = self.data[curr_char][curr_idx]
        return curr_data

    def on_receive_char(self):
        return

    def on_send_char(self):
        #first try to send perturbed data
        if self.char_mdl:
            msg = MultiPaths()
            for curr_idx in range(len(self.char_mdl)):
                mdl, t_array, opt_parms, traj_opt, vel_vec_opt, theta_opt  = self.get_perturbed_trajectory_and_parms(curr_idx)
                tmp_path = Path()
                tmp_path.poses = [None] * len(traj_opt)
                for idx, pnt in enumerate(traj_opt):
                    tmp_path.poses[idx] = PoseStamped()
                    tmp_path.poses[idx].pose.position.x = pnt[0]
                    tmp_path.poses[idx].pose.position.y = pnt[1]
                msg.paths.append(tmp_path)
            #send this
            msg.header.frame_id = 'writing_surface'
            msg.header.stamp = rospy.Time.now()
            self.ros_publisher.publish(msg)
            rospy.loginfo('GAIPS_PYTK_VIEWER: Sent perturbed character data...')
            return

        #send original data if the perturbed data is not available
        curr_data = self.get_current_data()
        if curr_data is not None:
            msg = MultiPaths()
            for strk in curr_data:
                tmp_path = Path()
                tmp_path.poses = [None] * len(strk)
                for idx, pnt in enumerate(strk):
                    tmp_path.poses[idx] = PoseStamped()
                    tmp_path.poses[idx].pose.position.x = pnt[0]
                    tmp_path.poses[idx].pose.position.y = pnt[1]
                msg.paths.append(tmp_path)
            #send this
            self.ros_publisher.publish(msg)
            rospy.loginfo('GAIPS_PYTK_VIEWER: Sent character data...')
        return

def main():
    app = QApplication(sys.argv)
    dp = PyTrajKin_GUI()

    pkg_path = rospkg.RosPack().get_path('signorm_trajectory_generator')
    dp.resource_folder = os.path.join(pkg_path, 'res')

    #prepare ros stuff
    ros_node = rospy.init_node('gaips_pytk_viewer')
    ros_sub_topic = '/nao_writing/child_character'
    ros_pub_topic = '/nao_writing/nao_character'
    dp.ros_publisher = rospy.Publisher(ros_pub_topic, MultiPaths, queue_size=10)
    dp.ros_subscriber = rospy.Subscriber(ros_sub_topic, MultiPaths, dp.on_receive_char)

    dp.show()
    app.exec_()
    return

if __name__ == '__main__':
    main()
