from hashlib import new
from PyQt5 import uic
from picamera2 import Picamera2
from PyQt5.QtMultimedia import QCameraInfo
from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog, QMessageBox
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QTimer, QDateTime, Qt, QTime
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen
import cv2,time,sys,sysinfo
import numpy as np
import random as rnd
from Tracking_Func import Tack_Object
#import RPi.GPIO as IO

#flag
from PyQt5.uic import loadUi
import os

import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets, uic
# goi thu vien
import subprocess

class SerialThread(QThread):
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port_name, baud_rate, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.serial_port = None
        self.running = True

    def run(self):
        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=1)
            while self.running:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().strip().decode('utf-8')
                    self.data_received.emit(data)
        except serial.SerialException as e:
            self.error_occurred.emit(f"Serial error: {str(e)}")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class ThreadClass(QThread):
    ImageUpdate = pyqtSignal(np.ndarray)
    FPS = pyqtSignal(int)

    def run(self):
        try:
            piCam = Picamera2()
            config = piCam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
            piCam.configure(config)
            piCam.start()
        except Exception as e:
            print(f"Camera initialization failed: {e}")
            return

        self.ThreadActive = True
        prev_frame_time = time.time()

        while self.ThreadActive:
            try:
                frame_cap = piCam.capture_array()
                flip_frame = cv2.flip(src=frame_cap, flipCode=1)
                new_frame_time = time.time()
                fps = 1 / (new_frame_time - prev_frame_time)

                prev_frame_time = new_frame_time

                self.ImageUpdate.emit(flip_frame)
                # Lam tron
                rounded_fps = int(fps)

                # chay
                self.FPS.emit(rounded_fps)
            except Exception as e:
                print(f"Error capturing frame: {e}")
                self.stop()

    def stop(self):
        self.ThreadActive = False
        self.quit()

class boardInfoClass(QThread):
    cpu = pyqtSignal(float)
    ram = pyqtSignal(tuple)
    temp = pyqtSignal(float)

    def run(self):
        self.ThreadActive = True
        while self.ThreadActive:
            cpu = sysinfo.getCPU()
            ram = sysinfo.getRAM()
            #temp = sysinfo.getTemp()
            self.cpu.emit(cpu)
            self.ram.emit(ram)
            #self.temp.emit(temp)

    def stop(self):
        self.ThreadActive = False
        self.quit()

class randomColorClass(QThread):
    color = pyqtSignal(tuple)
    def run(self):
        self.ThreadActive = True
        while self.ThreadActive:
            color = ([rnd.randint(0,256),rnd.randint(0,256),rnd.randint(0,256)],
                     [rnd.randint(0,256),rnd.randint(0,256),rnd.randint(0,256)],
                     [rnd.randint(0,256),rnd.randint(0,256),rnd.randint(0,256)]
                     )
            self.color.emit(color)
            time.sleep(2)

    def stop(self):
        self.ThreadActive = False
        self.quit()

class Window_ErrorAlarm(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("Error.ui", self)

#   QLabel display
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("HMI_DOAN_9.ui",self)

        self.serial_thread = None

        # Lay danh sach cong COM
        ports = serial.tools.list_ports.comports()
        portList = [port[0] for port in ports]
        self.cmb_port_3.addItems(portList)

        # ket noi cac nut voi arduino
        self.btn_connect_3.clicked.connect(self.open_port)
        self.btn_disconnect_3.clicked.connect(self.close_port)

        # Timer de doc du lieu
        self.readRx = QTimer()
        self.readRx.timeout.connect(self.rx_buffer)
        self.readRx.start(100)  # Thay d?i kho?ng th?i gian n?u c?

        self.online_cam = QCameraInfo.availableCameras()
        self.camlist.addItems([c.description() for c in self.online_cam])
        self.btn_start.clicked.connect(self.StartWebCam)
        self.btn_stop.clicked.connect(self.StopWebcam)

        self.resource_usage = boardInfoClass()
        self.resource_usage.start()
        self.resource_usage.cpu.connect(self.getCPU_usage)
        self.resource_usage.ram.connect(self.getRAM_usage)
        #self.resource_usage.temp.connect(self.getTemp_usage)

        self.randomColor_usage = randomColorClass()
        self.randomColor_usage.start()
        self.randomColor_usage.color.connect(self.get_randomColors)

# Create Instance class
        self.Win_showError = Window_ErrorAlarm()

        # Track object Functions
        self.Track_Function1 = Tack_Object()
        self.Track_Function2 = Tack_Object()
        self.Track_Function3 = Tack_Object()


        self.lcd_timer = QTimer()
        self.lcd_timer.timeout.connect(self.clock)
        self.lcd_timer.start()

        self.Status_lamp = [True,True,True]
        # End QTimer Zone


        self.btn_setObject1.setCheckable(True)
        self.btn_setObject1.clicked.connect(self.GetObject_one)


        self.btn_close.clicked.connect(self.Close_software)

        self.btn_roi_set.setCheckable(True)
        self.btn_roi_set.clicked.connect(self.set_roi)

        self.ROI_X.valueChanged.connect(self.get_ROIX)
        self.ROI_Y.valueChanged.connect(self.get_ROIY)
        self.ROI_W.valueChanged.connect(self.get_ROIW)
        self.ROI_H.valueChanged.connect(self.get_ROIH)
        self.ckb_roi.setChecked(True)

        self.roi_x = 20
        self.roi_y = 20
        self.roi_w = 620
        self.roi_h = 460

        self.Win_showError.btn_e_close.clicked.connect(self.Close_Error)
        self.btn_stop.setEnabled(False)

        # ket noi Layvitritutele.py
        self.btn_receive.clicked.connect(self.run_layvitritutele)
        self.setup_timer()

        # Connect the btn_send button to the send_data_to_arduino function
        self.btn_send.clicked.connect(self.send_data_to_arduino)
        # Connect the btn_update button to the update_com_ports function
        self.btn_update.clicked.connect(self.update_com_ports)

        # Function to update the list of COM ports
        self.update_com_ports()

    def update_com_ports(self):
        # Get the list of available COM ports
        ports = serial.tools.list_ports.comports()
        com_ports = [port.device for port in ports]

        # Clear all items in the comboBox before updating
        self.cmb_port_3.clear()

        # Add the new COM ports to the comboBox
        self.cmb_port_3.addItems(com_ports)

    def send_data_to_arduino(self):
        """Function to send data from the sun_position.txt file to the Arduino."""
        try:
            with open('sun_position.txt', 'r') as file:
                lines = file.readlines()
                if len(lines) >= 2:
                    azimuth = lines[0].strip()
                    altitude = lines[1].strip()
                    if self.serial_thread and self.serial_thread.isRunning():
                        data_to_send = f"{azimuth},{altitude}\n"
                        self.serial_thread.serial_port.write(data_to_send.encode('utf-8'))
                        self.textEdit.append(f"Sent data: {data_to_send} to Arduino.")
                    else:
                        QMessageBox.warning(self, "Serial Connection", "No serial connection to send data.")
                else:
                    QMessageBox.warning(self, "Data Error", "Not enough data in sun_position.txt to send.")
        except FileNotFoundError:
            QMessageBox.critical(self, "File Error", "sun_position.txt file not found.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred while sending data: {e}")

    def setup_timer(self):
        from PyQt5.QtCore import QTimer
        timer = QTimer(self)
        timer.timeout.connect(self.update_sun_position_text_edit)
        timer.start(1000)  # Update every second

    def run_layvitritutele(self):
        # Chay Layvitritutele.py
        #subprocess.run(['python', 'Layvitritutele.py'], shell=True)
        subprocess.Popen(['python', 'Layvitritutele.py'])
        #process.wait()
        #while not os.path.exists('update_flag.txt'):
            #time.sleep(1)
        self.update_sun_position_text_edit()

    def update_sun_position_text_edit(self):
        flag_path = 'update_flag.txt'
        if os.path.exists(flag_path):
            print("Flag found. Reading data...")
            try:
                with open('sun_position.txt', 'r') as file:
                    lines = file.readlines()
                    if len(lines) >= 2:
                        azimuth = lines[0].strip()
                        altitude = lines[1].strip()
                        print(f"Azimuth: {azimuth}, Altitude: {altitude}")
                        self.textEdit_2.setText(azimuth)  # Update textEdit_2 with azimuth
                        self.textEdit_3.setText(altitude)  # Update textEdit_3 with altitude
            except Exception as e:
                print(f"Error reading sun_position.txt: {e}")
            os.remove(flag_path)  # Remove flag after reading
        else:
            print("Flag not found.")
    def open_port(self):
        port_name = self.cmb_port_3.currentText()
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()

        self.serial_thread = SerialThread(port_name, 9600)
        self.serial_thread.data_received.connect(self.handle_data_received)
        self.serial_thread.error_occurred.connect(self.handle_error_occurred)
        self.serial_thread.start()

        self.btn_connect_3.setEnabled(False)
        self.btn_disconnect_3.setEnabled(True)
        #QMessageBox.information(self, "Connection Status", f"Connected to {port_name}")
        self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Connection Status - Connected to {port_name}")

    def close_port(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()

        self.btn_connect_3.setEnabled(True)
        self.btn_disconnect_3.setEnabled(False)
        #QMessageBox.information(self, "Connection Status", "Disconnected")
        self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Connection Status - Disconnected")

    def rx_buffer(self):
        # Hien thi thong bao neu co loi
        pass

    def handle_data_received(self, data):
        data_buffer = data.split(",")
        print(f"Data incoming: {data_buffer}")

    def handle_error_occurred(self, error_message):
        QMessageBox.critical(self, "Serial Error", error_message)

    def get_randomColors(self,color):
        self.RanColor1 = color[0]
        self.RanColor2 = color[1]
        self.RanColor3 = color[2]

    def getCPU_usage(self,cpu):
        self.Qlabel_cpu.setText(str(cpu) + " %")
        if cpu > 15: self.Qlabel_cpu.setStyleSheet("color: rgb(23, 63, 95);")
        if cpu > 25: self.Qlabel_cpu.setStyleSheet("color: rgb(32, 99, 155);")
        if cpu > 45: self.Qlabel_cpu.setStyleSheet("color: rgb(60, 174, 163);")
        if cpu > 65: self.Qlabel_cpu.setStyleSheet("color: rgb(246, 213, 92);")
        if cpu > 85: self.Qlabel_cpu.setStyleSheet("color: rgb(237, 85, 59);")

    def getRAM_usage(self,ram):
        self.Qlabel_ram.setText(str(ram[2]) + " %")
        if ram[2] > 15: self.Qlabel_ram.setStyleSheet("color: rgb(23, 63, 95);")
        if ram[2] > 25: self.Qlabel_ram.setStyleSheet("color: rgb(32, 99, 155);")
        if ram[2] > 45: self.Qlabel_ram.setStyleSheet("color: rgb(60, 174, 163);")
        if ram[2] > 65: self.Qlabel_ram.setStyleSheet("color: rgb(246, 213, 92);")
        if ram[2] > 85: self.Qlabel_ram.setStyleSheet("color: rgb(237, 85, 59);")

    def getTemp_usage(self,temp):
        self.Qlabel_temp.setText(str(temp) + " *C")
        if temp > 30: self.Qlabel_temp.setStyleSheet("color: rgb(23, 63, 95);")
        if temp > 35: self.Qlabel_temp.setStyleSheet("color: rgb(60, 174, 155);")
        if temp > 40: self.Qlabel_temp.setStyleSheet("color: rgb(246,213, 92);")
        if temp > 45: self.Qlabel_temp.setStyleSheet("color: rgb(237, 85, 59);")
        if temp > 50: self.Qlabel_temp.setStyleSheet("color: rgb(255, 0, 0);")

    def get_FPS(self,fps):
        self.Qlabel_fps.setText(str(fps))
        if fps > 5: self.Qlabel_fps.setStyleSheet("color: rgb(237, 85, 59);")
        if fps > 15: self.Qlabel_fps.setStyleSheet("color: rgb(60, 174, 155);")
        if fps > 25: self.Qlabel_fps.setStyleSheet("color: rgb(85, 170, 255);")
        if fps > 35: self.Qlabel_fps.setStyleSheet("color: rgb(23, 63, 95);")

    def clock(self):
        self.DateTime = QDateTime.currentDateTime()
        self.lcd_clock.display(self.DateTime.toString('hh:mm:ss'))


# Close Error Notification window
    def Close_Error(self):
        self.Win_error.close()

# ! Function oneclick to hsv parameter
    def GetObject_one(self):
        try:
            self.hsvOne_lower = np.array([self.Slider_H_min.value(), self.Slider_S_min.value(), self.Slider_V_min.value()], dtype=np.uint8)
            self.hsvOne_upper = np.array([self.Slider_H_max.value(), self.Slider_S_max.value(), self.Slider_V_max.value()], dtype=np.uint8)
            self.textEdit.append(f"Object 1 >> HSV Lower: {self.hsvOne_lower} | HSV Upper: {self.hsvOne_upper}")
            # For Debug
            #print(f"HSV Lower: {self.hsvOne_lower} | HSV Upper: {self.hsvOne_upper}")
            hsv2mask_one = cv2.cvtColor(src=self.CopyImage,code=cv2.COLOR_BGR2HSV)
            mask_object1_range = cv2.inRange(src=hsv2mask_one,lowerb=self.hsvOne_lower,upperb=self.hsvOne_upper)
            mask_object1 = cv2.bitwise_and(src1=self.CopyImage,src2=self.CopyImage,mask=mask_object1_range)
            mask_disp1 = self.cvt_cv_qt(mask_object1)
            self.disp_obj1.setPixmap(mask_disp1)
            self.disp_obj1.setScaledContents(True)
        except:
            self.Win_error.show()
            self.Win_error.Qlabel_error.setText("Please start camera before set !")

    def set_roi(self):
        if self.btn_roi_set.isChecked():
            self.btn_roi_set.setText('RESET')
            self.ckb_roi.setChecked(False)
            self.ROI_X.setEnabled(False)
            self.ROI_Y.setEnabled(False)
            self.ROI_W.setEnabled(False)
            self.ROI_H.setEnabled(False)
        else:
            self.btn_roi_set.setText('SET')
            self.ckb_roi.setChecked(True)
            self.ROI_X.setEnabled(True)
            self.ROI_Y.setEnabled(True)
            self.ROI_W.setEnabled(True)
            self.ROI_H.setEnabled(True)

    @pyqtSlot(np.ndarray)
    def opencv_emit(self, Image):

        #QPixmap format
        original = self.cvt_cv_qt(Image)
        #Numpy Array format
        self.CopyImage =  Image[self.roi_y:self.roi_h,
                                self.roi_x:self.roi_w]

        """
        """

        self.disp_main.setPixmap(original)
        self.disp_main.setScaledContents(True)

    # ! Display 1
        if self.ckb_disp1.isChecked() and self.btn_setObject1.isChecked() == False and self.track_obj1.isChecked() == False:
            hsv_object_one_lower = np.array([self.Slider_H_min.value(), self.Slider_S_min.value(), self.Slider_V_min.value()], dtype=np.uint8)
            hsv_object_one_upper = np.array([self.Slider_H_max.value(), self.Slider_S_max.value(), self.Slider_V_max.value()], dtype=np.uint8)

            hsv_to_ObjectOne = cv2.cvtColor(src=self.CopyImage,code=cv2.COLOR_BGR2HSV)
            hsv_target_ObjectOne = cv2.inRange(src=hsv_to_ObjectOne,lowerb=hsv_object_one_lower,upperb=hsv_object_one_upper)
            hsv_one_disp = self.cvt_cv_qt(hsv_target_ObjectOne)
            self.disp_obj1.setPixmap(hsv_one_disp)
            self.disp_obj1.setScaledContents(True)


    # Display Object Tracking
        if self.track_obj1.isChecked():

            Track_object1 = self.Track_Function1.track_object(Image=self.CopyImage,
                                                                  HSVLower=self.hsvOne_lower,
                                                                  HSVUpper=self.hsvOne_upper,
                                                                  Color=self.RanColor1)
        # Get value object
            self.value_curr_object1 = self.Track_Function1.get_current()
            self.value_total_object1 = self.Track_Function1.get_total()

        # Get object in screen
            #self.IsObject1 = self.Track_Function1.check_object()


        # Convert function from Numpy to QPixmap
            cvt2Tack_1 = self.cvt_cv_qt(Track_object1)

        # Show on the main screen
            self.disp_main.setPixmap(cvt2Tack_1)
            self.disp_main.setScaledContents(True)



    def get_ROIX(self,x):
        self.roi_x = x

    def get_ROIY(self,y):
        self.roi_y = y

    def get_ROIW(self,w):
        self.roi_w = w

    def get_ROIH(self,h):
        self.roi_h = h

    def cvt_cv_qt(self, Image):
        offset = 5
        rgb_img = cv2.cvtColor(src=Image,code=cv2.COLOR_BGR2RGB)
        if self.ckb_roi.isChecked():
            rgb_img = cv2.rectangle(rgb_img,
                                         pt1=(self.roi_x,self.roi_y),
                                         pt2=(self.roi_w,self.roi_h),
                                         color=(0,255,255),
                                         thickness=2)

        h,w,ch = rgb_img.shape
        bytes_per_line = ch * w
        cvt2QtFormat = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(cvt2QtFormat)
        if self.track_obj1.isChecked():
            pixmap = QPixmap.fromImage(cvt2QtFormat)
            painter = QPainter(pixmap)
            pen = QPen(Qt.red,3)
            painter.setPen(pen)
            painter.drawRect(self.roi_x-(self.roi_x-offset),
                             self.roi_y-(self.roi_y-offset),
                             self.roi_w-30,
                             self.roi_h-30
                             )

        return pixmap #QPixmap.fromImage(cvt2QtFormat)
#----------------------------------------------------------------------------------------------------

    def StartWebCam(self,pin):
        try:
            self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Start Webcam ({self.camlist.currentText()})")
            self.btn_stop.setEnabled(True)
            self.btn_start.setEnabled(False)

            global camIndex
            camIndex = self.camlist.currentIndex()

        # Opencv QThread
            self.Worker1_Opencv = ThreadClass()
            self.Worker1_Opencv.ImageUpdate.connect(self.opencv_emit)
            self.Worker1_Opencv.FPS.connect(self.get_FPS)
            self.Worker1_Opencv.start()


        except Exception as error :
            pass

    def StopWebcam(self,pin):
        self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Stop Webcam ({self.camlist.currentText()})")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)


    def Close_software(self):
        self.Worker1_Opencv.stop()
        self.resource_usage.stop()
        sys.exit(app.exec_())




if __name__ == "__main__":

    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()