# ENPH 353 Class Competition Project
September 2019 - December 2019 <br>
Contributors: Ranya Ataya & Zach Laskin <br><br>

### Competition Overview
The competition requires a predefined autonomous robot in a simulated world in Gazebo to do the following:
- Drive around a track
- Detect parking lots at various positions around the track
- Read alphanumeric text on license plates at each parking lot
- Publish messages to a specified ROS topic
- Detect moving NPCs (non-playable characters) and avoid collisions with them
<br><br>

### Project Goal
The objective of this project was to program an autonomous robot to lane follow, avoid collisions, and read and report license plates to a ROS topic in a simulated world in Gazebo. Our robot was to compete against other teams in our class competition. Part of these tasks where acheived using machine learning while others were achieved using basic image manipulation with OpenCV.
<br><br>

### Parking Lot Detection
Contributor: Ranya Ataya <br><br>
Parking lot detection was achieved by colour masking the robot's image feed and looking for a certain threshold value.
<br><br>

### Reading License Plates
Contributor: Ranya Ataya <br><br>
A convolution neural network (CNN) was used to train a model for the robot. The model was trained to differentiate between alphanumeric characters. This was done using the OpenCV, Keras, and Tensoflow libraries. 
<br><br>
Training was done in Google Collaboratory and can be found here: [licensePlates and lotIDs CNN](https://colab.research.google.com/drive/1ViajBWmqxbqJCaQNv43VCiLeO4HOZuLF)
<br><br>
All of the data used for training this CNN can be found in this GitHub repository, in the following paths:<br>
enph353-team-grimm/convolutionNN_2_ws/src/convolution_net/scripts/letters_and_numbers/ <br>
enph353-team-grimm/convolutionNN_2_ws/src/convolution_net/scripts/lettersAndNums_brightness/ <br>
<br>

### Lane Following (Driving)
Contributor: Zach Laskin <br><br>
Lane following was achieved by detecting the white lines on either side of the road and determining the direction of movement accordingly. Directions and and velocities would be published to the corresponding ROS topic. 
<br><br>

### NPC Detection
Contributor: Zach Laskin <br><br>
NPC detection was achieved simply using colour masks and image thresholds.
<br><br>

### Robot's Control Node (Decision making)
Contributor: Ranya Ataya & Zach Laskin <br><br>
The control node consisted of the robot's overall decision making and setup. The overall control process consists of the robot consistently driving unless something is flagged (detection of NPC or parking lot). When a flag is initiated, the robot will react accordingly, either stopping to avoid a collision or reading and reporting the license plate using the CNN model.
<br><br>

### Notes:
All python scripts used in the final iteration of the project can be found in this GitHub repository in the following path: 
<br>enph353-team-grimm/convolutionNN_2_ws/src/convolution_net/scripts/ 
