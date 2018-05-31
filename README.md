# Baxter Safety Software
A node that monitors the robot's endpoint and joint orientations, velocities, and positions. When any safety parameters (as set forth in a yaml file) are violated, the robot E-stops and kills all activity.

## Usage
### Editing Parameters
To edit the safety parameters, see the parameters.yml file in the scripts folder

### Running Safety Node
'''
rosrun lab_baxter_safety safety.py
'''
