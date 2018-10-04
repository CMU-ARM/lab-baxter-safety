# Baxter Safety Software
Contact - Zhi - zhi.tan@ri.cmu.edu

A node that monitors the robot's endpoint and joint orientations, velocities, and positions. When any safety parameters (as set forth in a yaml file) are violated, the robot E-stops and kills all activity.

## Usage
### Editing Parameters
To edit the safety parameters, see the parameters.yml file. Leave any parameters you don't want to monitor as blank dictionary entries in the file.

### Running Safety Node
```
rosrun lab_baxter_safety safety.py
```

### Contributors
*  Joe Connolly