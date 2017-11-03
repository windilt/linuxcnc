############################
# **** IMPORT SECTION **** #
############################

from PyQt4 import QtCore
from PyQt4 import QtGui
from qtscreen.keybindings import Keylookup
from qtscreen.aux_program_loader import Aux_program_loader
from qtscreen.notify import Notify
from qtscreen.message import Message
from qtscreen.preferences import Access
#from qtvcp_widgets.overlay_widget import LoadingOverlay

from qtvcp.qt_glib import GStat, Lcnc_Action
import linuxcnc
import sys
import os

###########################################
# **** instantiate libraries section **** #
###########################################

KEYBIND = Keylookup()
GSTAT = GStat()
ACTION = Lcnc_Action()
AUX_PRGM = Aux_program_loader()
NOTE = Notify()
MSG = Message()
PREFS = Access()

###################################
# **** HANDLER CLASS SECTION **** #
###################################

class HandlerClass:

    ########################
    # **** INITIALIZE **** #
    ########################
    # widgets allows access to  widgets from the qtvcp files
    # at this point the widgets and hal pins are not instantiated
    def __init__(self, halcomp,widgets,paths):
        self.hal = halcomp
        self.w = widgets
        self.stat = linuxcnc.stat()
        self.cmnd = linuxcnc.command()
        self.error = linuxcnc.error_channel()
        self.jog_velocity = 10.0
        self.PATH = paths.CONFIGPATH
        self.IMAGE_PATH = paths.IMAGEDIR
        #print paths.CONFIGPATH
        # connect to GStat to catch linuxcnc events
        GSTAT.connect('state-estop', self.say_estop)
        GSTAT.connect('state-on', self.on_state_on)
        GSTAT.connect('state-off', self.on_state_off)
        GSTAT.connect('jograte-changed', self.on_jograte_changed)
        GSTAT.connect('periodic', self.on_periodic)

        # Read user preferences
        self.desktop_notify = PREFS.getpref('desktop_notify', True, bool)
        self.shutdown_check = PREFS.getpref('shutdown_check', True, bool)

    ##########################################
    # Special Functions called from QTSCREEN
    ##########################################

    # at this point:
    # the widgets are instantiated.
    # the HAL pins are built but HAL is not set ready
    def initialized__(self):
        # Give notify library a reference to the statusbar
        NOTE.statusbar = self.w.statusBar
        if self.desktop_notify:
            NOTE.notify('Welcome','This is a test screen for Qtscreen',None,4)
        self.w.jog_slider.setValue(self.jog_velocity)
        self.w.feed_slider.setValue(100)
        self.w.rapid_slider.setValue(100)
        GSTAT.forced_update()

        # add a backgrund image
        self.w.setObjectName("MainWindow")
        bgpath = self.IMAGE_PATH+'/hazzy_bg_black.png'
        self.w.setStyleSheet("#MainWindow { background-image: url(%s) 0 0 0 0 stretch stretch; }"%bgpath)
        bgpath = self.IMAGE_PATH+'/frame_bg_blue.png'
        self.w.frame.setStyleSheet("#frame { border-image: url(%s) 0 0 0 0 stretch stretch; }"%bgpath)
        bgpath = self.IMAGE_PATH+'/frame_bg_grey.png'
        self.w.frame_2.setStyleSheet("QFrame { border-image: url(%s) 0 0 0 0 stretch stretch; }"%bgpath)
        # add overlay to topWidget
        #self.w.overlay = LoadingOverlay(self.w)

    def processed_key_event__(self,receiver,event,is_pressed,key,code,shift,cntrl):
        # when typing in MDI, we don't want keybinding to call functions
        # so we catch and process the events directly.
        # We do want ESC, F1 and F2 to call keybinding functions though
        if self.w.mdi_line == receiver and code not in(16777216,16777264,16777216):
            if is_pressed:
                self.w.mdi_line.keyPressEvent(event)
                event.accept()
            return True
        try:
            KEYBIND.call(self,event,is_pressed,shift,cntrl)
            return True
        except AttributeError:
            print 'Error in, or no function for: %s in handler file for-%s'%(KEYBIND.convert(event),key)
            #print 'from %s'% receiver
            return False

    ########################
    # callbacks from GSTAT #
    ########################
    def say_estop(self,w):
        print 'saying estop'

    def on_state_on(self,w):
        print 'on'

    def on_state_off(self,w):
        print 'off'


    def on_jograte_changed(self, w, rate):
        self.jog_velocity = rate

    def on_error_message(self, w, message):
        NOTE.notify('Error',message,QtGui.QMessageBox.Information,10)

    def on_periodic(self,w):
        try:
            e = self.error.poll()
            if e:
                kind, text = e
                if kind in (linuxcnc.NML_ERROR, linuxcnc.OPERATOR_ERROR):
                    if self.desktop_notify:
                        NOTE.notify('ERROR',text,None,4)
                elif kind in (linuxcnc.NML_TEXT, linuxcnc.OPERATOR_TEXT):
                   if self.desktop_notify:
                        NOTE.notify('OP MESSAGE',text,None,4)
                elif kind in (linuxcnc.NML_DISPLAY, linuxcnc.OPERATOR_DISPLAY):
                   if self.desktop_notify:
                        NOTE.notify('DISPLAY',text,None,4)
        except:
            pass

    #######################
    # callbacks from form #
    #######################

    def zero_axis(self):
        name = self.w.sender().text()
        print name
        if 'X' in name:
            GSTAT.set_axis_origin('x',0)
        elif 'Y' in name:
            GSTAT.set_axis_origin('y',0)
        elif 'Z' in name:
            GSTAT.set_axis_origin('z',0)

    def launch_status(self):
        AUX_PRGM.load_status()

    def launch_halmeter(self):
        AUX_PRGM.load_halmeter()

    def change_jograte(self, rate):
        GSTAT.set_jog_rate(float(rate))

    def change_feedrate(self, rate):
        self.cmnd.feedrate(rate/100.0)

    def change_rapidrate(self, rate):
        self.cmnd.rapidrate(rate/100.0)

    def jog_pressed(self):
        d = 1
        source = self.w.sender()
        #print source.objectName(), 'pressed'
        ACTION.ensure_mode(linuxcnc.MODE_MANUAL)
        if '-' in source.text():
            d = -1
        if 'X' in source.text():
            self.continous_jog(0, d)
        elif 'Y' in source.text():
            self.continous_jog(1, d)
        elif 'Z' in source.text():
            self.continous_jog(2, d)

    def jog_released(self):
        source = self.w.sender()
        #print source.objectName(), 'released'
        ACTION.ensure_mode(linuxcnc.MODE_MANUAL)
        if 'X' in source.text():
            self.continous_jog(0, 0)
        elif 'Y' in source.text():
            self.continous_jog(1, 0)
        elif 'Z' in source.text():
            self.continous_jog(2, 0)

    def loadfile_clicked(self):
        fname = self.w.lcnc_filedialog.LOAD()
        self.w.gcodeeditor.setFocus()

    def runfile_clicked(self):
        print 'run file'
        self.cmnd.mode(linuxcnc.MODE_AUTO)
        self.cmnd.auto(linuxcnc.AUTO_RUN,0)

    def stopfile_clicked(self):
        print 'stop file'
        self.cmnd.mode(linuxcnc.MODE_AUTO)
        self.cmnd.abort()

    def pausefile_clicked(self):
        print 'pause file',GSTAT.stat.paused
        if not GSTAT.stat.paused:
            self.cmnd.auto(linuxcnc.AUTO_PAUSE)
        else:
            print 'resume'
            self.cmnd.auto(linuxcnc.AUTO_RESUME)

    #####################
    # general functions #
    #####################

    def continous_jog(self, axis, direction):
        GSTAT.continuous_jog(axis, direction)

    #####################
    # KEY BINDING CALLS #
    #####################
    def on_keycall_ABORT(self,event,state,shift,cntrl):
        if state:
            print 'abort'
            if GSTAT.stat.interp_state == linuxcnc.INTERP_IDLE:
                print 'close'
                self.w.close()
            else:
                print 'abort'
                self.cmnd.abort()

    def on_keycall_ESTOP(self,event,state,shift,cntrl):
        if state:
            self.w.button_estop.click()
    def on_keycall_POWER(self,event,state,shift,cntrl):
        if state:
            self.w.button_machineon.click()
    def on_keycall_HOME(self,event,state,shift,cntrl):
        if state:
            self.w.button_home.click()

    def on_keycall_XPOS(self,event,state,shift,cntrl):
        if state:
            self.w.jog_pos_x.pressed.emit()
        else:
            self.w.jog_pos_x.released.emit()
    def on_keycall_XNEG(self,event,state,shift,cntrl):
        if state:
            self.w.jog_neg_x.pressed.emit()
        else:
            self.w.jog_neg_x.released.emit()

    def on_keycall_YPOS(self,event,state,shift,cntrl):
        if state:
            self.w.jog_pos_y.pressed.emit()
        else:
            self.w.jog_pos_y.released.emit()

    def on_keycall_YNEG(self,event,state,shift,cntrl):
        if state:
            self.w.jog_neg_y.pressed.emit()
        else:
            self.w.jog_neg_y.released.emit()

    def on_keycall_ZPOS(self,event,state,shift,cntrl):
        if state:
            self.w.jog_pos_z.pressed.emit()
        else:
            self.w.jog_pos_z.released.emit()
    def on_keycall_ZNEG(self,event,state,shift,cntrl):
        if state:
            self.w.jog_neg_z.pressed.emit()
        else:
            self.w.jog_neg_z.released.emit()

    ###########################
    # **** closing event **** #
    ###########################
    def closeEvent(self, event):
        if self.shutdown_check:
            GSTAT.emit('focus-overlay-changed',True,'ARE YOU SURE!',QtGui.QColor(100, 0, 0,150))
            answer = self.w.lcnc_dialog.showdialog('Do you want to shutdown now?',
                None, details='You can set a preference to not see this message',
                icon=MSG.CRITICAL, display_type=MSG.YN_TYPE)
            #self.w.lcnc_dialog.hide()
            if not answer:
                event.ignore()
                GSTAT.emit('focus-overlay-changed',False,None,None)
                return
        event.accept()

    ##############################
    # required class boiler code #
    ##############################

    def __getitem__(self, item):
        return getattr(self, item)
    def __setitem__(self, item, value):
        return setattr(self, item, value)

################################
# required handler boiler code #
################################

def get_handlers(halcomp,widgets,paths):
     return [HandlerClass(halcomp,widgets,paths)]