from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtWidgets import (
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QGridLayout,
    QHBoxLayout,
    QApplication,
    QWidget,
    )
import rospy
     

class StartButton(QPushButton):
    def __init__(self, parent):
        super(StartButton, self).__init__()
        self.setText("Start")
        self.parent = parent
        self.setDisabled(False)

    def mousePressEvent(self, e):
        self.setDisabled(True)#Disable Start
        self.parent.layout().itemAtPosition(0,1).widget().setDisabled(False)#Enable Stop
        self.parent.layout().itemAtPosition(1,0).widget().setDisabled(True)#Disable Reset
        self.parent.layout().itemAtPosition(1,1).widget().setDisabled(True)#Disable Close

class StopButton(QPushButton):
    def __init__(self,parent):
        super(StopButton, self).__init__()
        self.setText("Stop")
        self.parent = parent
        self.setDisabled(True)

    def mousePressEvent(self, e):
        self.setDisabled(True)#Disable Stop
        self.parent.layout().itemAtPosition(0,0).widget().setDisabled(True)#Disable Start
        self.parent.layout().itemAtPosition(1,0).widget().setDisabled(False)#Enable Reset
        self.parent.layout().itemAtPosition(1,1).widget().setDisabled(False)#Enable Close

class ResetButton(QPushButton):
    def __init__(self,parent):
        super(ResetButton, self).__init__()
        self.setText("Reset")
        self.parent = parent
        self.setDisabled(True)
    
    def mousePressEvent(self, e):
        self.setDisabled(True)#Reset Start
        self.parent.layout().itemAtPosition(0,0).widget().setDisabled(False)#Enable Start
        self.parent.layout().itemAtPosition(0,1).widget().setDisabled(True)#Disable Stop
        self.parent.layout().itemAtPosition(1,1).widget().setDisabled(False)#Enable Close

class CloseButton(QPushButton):
    def __init__(self,parent):
        super(CloseButton, self).__init__()
        self.parent = parent
        self.setText("Close")

    def mousePressEvent(self, e):
        QApplication.quit()
        rospy.signal_shutdown("Closing GUI")

class ManualFeedButton(QPushButton):
    def __init__(self,parent):
        super(ManualFeedButton, self).__init__()
        self.parent = parent
        self.setText("Feed")

    def mousePressEvent(self, e):
        pass

class LightsSlider(QSlider):
    def __init__(self):
        super(LightsSlider, self).__init__(Qt.Horizontal)
        self.setMinimum(0)
        self.setMaximum(2)
        self.setTickPosition(QSlider.TicksAbove|QSlider.TicksBelow)#|QSlider.TicksBothSides)
        self.setPageStep(1)

    def value_changed(self, i):
        #TODO
        pass

class ImageLabel(QLabel):
    def __init__(self):
        super(ImageLabel, self).__init__()
        self.image = None# QPixmap('images\\0.jpg')
        #self.setPixmap(self.image)
        #self.setMinimumSize(QSize(640, 480))

    def resize(self, e):
        #print(e.size())
        height = 3*e.size().height()//5
        temp_width = 4*height//3
        width = 9*e.size().width()//20
        if temp_width>width:
            height = 3*width//4
        else:
            width = temp_width
        #print(width,height)
        if self.image:
            self.setPixmap(self.image.scaled(width,height))#, aspectRatioMode=Qt.KeepAspectRatio))

    def update(self, img):
        height, width, _ = img.shape
        bytes_per_line = 3 * width
        q_img = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888)    
        self.image = QPixmap.fromImage(q_img)
        self.setPixmap(self.image)

class ImageViewLayout(QVBoxLayout):
    def __init__(self, title):
        super(ImageViewLayout, self).__init__()
        self.image = ImageLabel()
        label = QLabel(title)
        font = label.font()
        font.setPointSize(15)
        label.setFont(font)
        label.setAlignment(Qt.AlignHCenter)
        self.image.setAlignment(Qt.AlignCenter)
        self.addWidget(label)
        self.addWidget(self.image)

    def resizeEvent(self, e):
        #print("From layout",e.size())
        self.image.resize(e)
        
    def update(self, img):
        self.image.update(img)

class SettingsLayout(QGridLayout):
    def __init__(self, parent):
        super(SettingsLayout, self).__init__()
        self.parent = parent

        self.lights_control = LightsSlider()
        lights_label = QLabel("Lights dimming")
        font = lights_label.font()
        font.setPointSize(13)
        lights_label.setFont(font)
        lights_label.setAlignment(Qt.AlignHCenter)
        self.addWidget(lights_label,0,0)
        self.addWidget(self.lights_control,1,0)
        
        self.manual_feed_control = ManualFeedButton(parent)
        feed_label = QLabel("Manual Feed")
        font = feed_label.font()
        font.setPointSize(13)
        feed_label.setFont(font)
        feed_label.setAlignment(Qt.AlignHCenter)
        self.addWidget(feed_label,0,1)
        self.addWidget(self.manual_feed_control,1,1)

    def resizeEvent(self, e):
        pass

class ControlsLayout(QGridLayout):
    def __init__(self, parent):
        super(ControlsLayout, self).__init__()
        self.addWidget(StartButton(parent),0,0)
        self.addWidget(StopButton(parent),0,1)
        self.addWidget(ResetButton(parent),1,0)
        self.addWidget(CloseButton(parent),1,1)

    def resizeEvent(self, e):
        pass