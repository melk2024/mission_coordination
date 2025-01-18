# mission_coordination
This project is based on ROS1 and Gazebo which aims to control the robot movement using different methods.

Start by Clone my project

In the same terminal, follow the instructions ONE AFTER THE OTHER:

```
cd ~/catkin_ws/src && git clone https://github.com/melk2024/mission_coordination.git

cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash
```
$roslaunch evry_project_description simu_robot.launch
$roslaunch evry_project_strategy agent.launch nbr_robot:=1
STEP 4: MOVE ONE ROBOT TO THE CORRESPONDING FLAG Now that you know how to start/stop the simulation, to move one robot in the environment, you need:
Q6: To modify the program to move one robot safely to its corresponding flag and stop it at this position, I used the following code
```
# simple stopping strategy
  if distance < 1:
      velocity = 0
      angle = 0   
  else:
      velocity = 2
      angle = 0
```

Q7: to adapt it to real life. For this purpose, you can implement a PID controller. It means that, when the robot is far from its goal, it moves with the highest velocity values and and as it gets closer, it slows down to a stop.
```
# PID controller class
class PIDController:
    def __init__(self, kp, ki, kd, setpoint, sample_time):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = self.setpoint - current_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * self.sample_time
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / self.sample_time
        d_term = self.kd * derivative

        # PID control output
        control_output = p_term + i_term + d_term

        # Update previous error for the next iteration
        self.prev_error = error

        return control_output
```

Now lets use the PID class in the demo section

STEP 5: Implementation of one strategy - timing solution to make sure robots do not collide with each other! 
Q8: We implemented one of the simplest strategy: timing strategy. 
The timing strategy is used to start each robot at different time. In this way, we avoided collision.
```
def run_demo():
    """Main loop"""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")
    int(robot_name[-1])
    rospy.sleep(5*int(robot_name[-1]))
    # Timing
        # Initialize state variable
    moving_forward = True  # Initial state
 
    # Initialize PID tuning parameters
    kp, ki, kd = 1.0, 0.5, 0.05
    sample_time = 0.1

    # Define goal distance 
    goal_distance = 0.5
    

    pid_controller = PIDController(kp, ki, kd, goal_distance, sample_time)
    while not rospy.is_shutdown():
        # Strategy
        #velocity = 2
        #angle = 0
        distance = float(robot.getDistanceToFlag())
        print(f"{robot_name} distance to flag = ", distance)

        # Write here your strategy..
        # Define a threshold distance for stopping the robot
        stop_distance_threshold = 1

        if moving_forward:
            # Move the robot forward with a certain velocity
           
            # Use PID controller to calculate the velocity
            velocity = pid_controller.compute(distance)

        # Ensure that velocity is within a reasonable range
            velocity = max(min(velocity, 0), 2)
            angle = 0.0

            # Check if the robot is close to the flag to transition to the "Stopped" state
            if distance <= stop_distance_threshold:
                moving_forward = False
        else:
            # Stop the robot
            velocity = 0.0
            angle = 0.0

        # Finishing by publishing the desired speed. 
        # DO NOT TOUCH.
        robot.set_speed_angle(velocity, angle)
        rospy.sleep(0.5)
```

The 3 robots started moving at different time and collission between robots efficiently avoided.


Q9: Write a launch file for this strategy.

```
<?xml version="1.0" encoding="UTF-8"?>

<launch>
  <arg name="nbr_robot" default="3"/>

  <!--MAIN CODE-->
  <node pkg="evry_project_strategy" type="timing.py" name="agent_1" output="screen" if="$(eval arg('nbr_robot') > 0)">
    <param name="robot_name" value="robot_1"/>
  </node>

  <node pkg="evry_project_strategy" type="timing.py" name="agent_2" output="screen" if="$(eval arg('nbr_robot') > 1)">
    <param name="robot_name" value="robot_2"/>
  </node>

  <node pkg="evry_project_strategy" type="timing.py" name="agent_3" output="screen" if="$(eval arg('nbr_robot') > 2)">
    <param name="robot_name" value="robot_3"/>
  </node>

</launch>
```
