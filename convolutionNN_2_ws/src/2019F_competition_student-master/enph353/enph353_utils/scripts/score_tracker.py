#!/usr/bin/env python
from datetime import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import (QPixmap)
from PyQt5.QtCore import (Qt, QTimer, pyqtSignal)
from std_msgs.msg import String
from python_qt_binding import loadUi

import csv
import os
import rospy
import sys

NUM_LOCATIONS = 8

class Window(QtWidgets.QMainWindow):
    message_received_signal = pyqtSignal(str)

    def __init__(self):
        super(Window, self).__init__()
        loadUi("./score_tracker.ui", self)

        pixmap = QPixmap('ENPH_vs_UBC_Parking.svg')
        self.label_QL.setPixmap(pixmap)

        # Populate log file name
        now = datetime.now()
        date_time = now.strftime("%Y%m%d_%H%M%S")
        self.log_file_path = (self.team_ID_value_QL.text() + "_" + 
                              date_time + '.txt')
        self.log_file_value_QL.setText(self.log_file_path)

        # Populate tables
        LICENSE_PLATE_FILE = '/../../enph353_gazebo/scripts/plates.csv'
        SCRIPT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
        with open(SCRIPT_FILE_PATH + LICENSE_PLATE_FILE, "r") as plate_file:
            platereader = csv.reader(plate_file)
            i=0
            for row in platereader:
                if i < NUM_LOCATIONS:
                    self.license_scores_QTW.item(i, 1).setText(row[0])
                    self.log_msg("Position {}: {}".format(i+1, row[0]))
                else:
                    break
                i += 1

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.SLOT_timer_update)
        self.elapsed_time_s = 0

        # Connect widgets
        self.penalty_vehicle_QPB.clicked.connect(self.SLOT_penalty_vehicle)
        self.penalty_pedestrian_QPB.clicked.connect(self.SLOT_penalty_pedestrian)
        self.penalty_track_QPB.clicked.connect(self.SLOT_penalty_track)

        self.message_received_signal.connect(self.SLOT_message_received)

        self.start_timer_QPB.clicked.connect(self.SLOT_start_timer)

        # Set-up ROS subscribers
        self.sub = rospy.Subscriber("license_plate", String, 
                                    self.licensePlate_callback)
        rospy.init_node('competition_listener')


    def licensePlate_callback(self, data):
        self.message_received_signal.emit(str(data.data))


    def log_msg(self, message):
        now = datetime.now()
        date_time = now.strftime("%H:%M:%S")
        log_output = "<font color='blue'>{}</font>: {}".format(date_time, message)
        self.comms_log_QTE.append(log_output)
        # self.comms_log_QTE.insertHtml(log_output)

        log_file_content = self.comms_log_QTE.toPlainText()

        with open(self.log_file_path, "w") as html_file:
            html_file.write(log_file_content)


    def SLOT_message_received(self, license_string):
        teamID, teamPswd, plateLocation, plateID = str(license_string).split(',')
        plateLocation = int(plateLocation)

        # Use to register the team name (not for points)
        if plateLocation == 0:
            # Update team ID and log file name:
            if teamID !=  self.team_ID_value_QL.text():
                now = datetime.now()
                date_time = now.strftime("%Y%m%d_%H%M%S")
                self.log_file_path = teamID + "_" + date_time + '.txt'
                self.log_file_value_QL.setText(self.log_file_path)

            self.team_ID_value_QL.setText(teamID)
            self.log_msg("Message received: {}".format(license_string))
            return

        self.log_msg("Message received: {}".format(license_string))

        # Check out of bounds plate location
        if plateLocation < 0 or plateLocation > 8:
            self.log_msg("Invalid plate location: {}".format(plateLocation))
            return

        # Check license plate ID and location against gnd truth:
        self.license_scores_QTW.item(plateLocation-1, 2).setText(plateID)
        gndTruth = str(self.license_scores_QTW.item(plateLocation-1, 1).text())

        if gndTruth == plateID:
            self.license_scores_QTW.item(plateLocation-1, 3).setText(str(5))
            self.log_msg("Awarded: {} pts".format(5))
        else:
            self.license_scores_QTW.item(plateLocation-1, 3).setText(str(-5))
            self.log_msg("Awarded: {} pts".format(-5))

        self.update_license_total()


    def SLOT_penalty_pedestrian(self):
        numEvents       = int(self.penalties_scores_QTW.item(1, 1).text()) + 1
        penaltyPerEvent = int(self.penalties_scores_QTW.item(1, 2).text())
        penaltyTotal    = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(1, 1).setText(str(numEvents))
        self.penalties_scores_QTW.item(1, 3).setText(str(penaltyTotal))

        self.log_msg("Penalty: pedestrian collision: -10 pts")
        self.update_penalty_total()


    def SLOT_penalty_track(self):
        numEvents       = int(self.penalties_scores_QTW.item(2, 1).text()) + 1
        penaltyPerEvent = int(self.penalties_scores_QTW.item(2, 2).text())
        penaltyTotal    = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(2, 1).setText(str(numEvents))
        self.penalties_scores_QTW.item(2, 3).setText(str(penaltyTotal))

        self.log_msg("Penalty: track limit: -2 pts")
        self.update_penalty_total()


    def SLOT_penalty_vehicle(self):
        numEvents       = int(self.penalties_scores_QTW.item(0, 1).text()) + 1
        penaltyPerEvent = int(self.penalties_scores_QTW.item(0, 2).text())
        penaltyTotal    = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(0, 1).setText(str(numEvents))
        self.penalties_scores_QTW.item(0, 3).setText(str(penaltyTotal))

        self.log_msg("Penalty: vehicle collision: -5 pts")
        self.update_penalty_total()


    def SLOT_start_timer(self):
        self.elapsed_time_s = 0
        self.elapsed_time_value_QL.setText(
            "{:03d} sec".format(self.elapsed_time_s))
        self.timer.start(1000)
        self.log_msg("Timer started.")


    def SLOT_timer_update(self):
        self.elapsed_time_s += 1
        self.elapsed_time_value_QL.setText(
            "{:03d} sec".format(self.elapsed_time_s))
        if (self.elapsed_time_s == 120):
            self.log_msg("Out of time: 2 minutes.")


    def update_license_total(self):
        licenseTotal = 0
        for i in range(NUM_LOCATIONS):
            licenseTotal += int(self.license_scores_QTW.item(i, 3).text())

        self.license_total_value_QL.setText(str(licenseTotal))

        penaltyTotal = int(self.penalties_total_value_QL.text())
        teamTotal = penaltyTotal + licenseTotal
        self.total_score_value_QL.setText(str(teamTotal))
        self.log_msg("Team total: {} pts".format(str(teamTotal)))


    def update_penalty_total(self):
        penaltyVehicle    = int(self.penalties_scores_QTW.item(0, 3).text())
        penaltyPedestrian = int(self.penalties_scores_QTW.item(1, 3).text())
        penaltyTrack      = int(self.penalties_scores_QTW.item(2, 3).text())

        penaltyTotal = penaltyVehicle + penaltyPedestrian + penaltyTrack
        self.penalties_total_value_QL.setText(str(penaltyTotal))

        licenseTotal = int(self.license_total_value_QL.text())
        teamTotal = penaltyTotal + licenseTotal
        self.total_score_value_QL.setText(str(teamTotal))
        self.log_msg("Team total: {} pts".format(str(teamTotal)))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()

    sys.exit(app.exec_())