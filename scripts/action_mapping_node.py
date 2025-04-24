#!/usr/bin/env python
import rospy
import json
import time 
from std_msgs.msg import String
from geometry_msgs.msg import Twist


NLU_OUTPUT_TOPIC = '/lingoros/nlu_output'
CMD_VEL_TOPIC = '/husky_velocity_controller/cmd_vel' 
NODE_NAME = 'lingoros_action_mapping_node'
DEFAULT_LINEAR_SPEED = 0.3  # m/s
DEFAULT_ANGULAR_SPEED = 0.5 # degrees/s
PUBLISH_RATE = 10 # Hz

# --- Global Variables ---
cmd_vel_publisher = None
is_busy = False #flag to prevent handling multiple commands at once

def execute_timed_velocity(linear_x, angular_z, duration):

    global cmd_vel_publisher
    global is_busy

    if cmd_vel_publisher is None:
        rospy.logerr("Cmd_vel publisher not initialized!")
        is_busy = False # Reset busy flag on error
        return

    twist_cmd = Twist()
    twist_cmd.linear.x = linear_x
    twist_cmd.linear.y = 0.0
    twist_cmd.linear.z = 0.0
    twist_cmd.angular.x = 0.0
    twist_cmd.angular.y = 0.0
    twist_cmd.angular.z = angular_z

    stop_cmd = Twist() # All zeros

    rate = rospy.Rate(PUBLISH_RATE)
    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(duration)

    rospy.loginfo(f"Executing command: Linear={linear_x:.2f} m/s, Angular={angular_z:.2f} rad/s for {duration:.2f} seconds.")

    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        cmd_vel_publisher.publish(twist_cmd)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Execution interrupted.")
            break 

    # Ensure robot stops after duration or interruption
    time.sleep(0.1)
    cmd_vel_publisher.publish(stop_cmd)
    rospy.loginfo("Finished executing command, sending stop.")

    is_busy = False # Command finished

def nlu_output_callback(message):
    """Callback function for processing the NEW detailed NLU JSON output."""
    global is_busy
    global cmd_vel_publisher 

    if is_busy:
        rospy.logwarn("Action Mapping Node is busy, ignoring new command for now.")
        return

    rospy.loginfo("Received NLU output.")
    try:
        nlu_data = json.loads(message.data)
        rospy.loginfo(f"Parsed NLU data: {nlu_data}")

        intent = nlu_data.get('intent')
        ros_interface = nlu_data.get('ros_interface', {})
        param_mapping = nlu_data.get('parameter_mapping', {})
        exec_details = nlu_data.get('execution_details', {})

        target_topic = ros_interface.get('name')
        msg_type = ros_interface.get('message_type')
        interface_type = ros_interface.get('type')

        if interface_type != 'topic' or target_topic != CMD_VEL_TOPIC or msg_type != 'geometry_msgs/Twist':
             rospy.logwarn(f"Received command for unexpected ROS interface: {ros_interface}")
             return

        
        exec_type = exec_details.get('type')

        if exec_type == 'timed_velocity':
            duration = exec_details.get('duration_sec')
            # Extract velocities directly from parameter_mapping
            target_linear_x = param_mapping.get('linear.x', 0.0)
            target_angular_z = param_mapping.get('angular.z', 0.0) 

            if duration is not None and isinstance(duration, (int, float)) and duration >= 0:
                is_busy = True 
                execute_timed_velocity(target_linear_x, target_angular_z, duration)
            else:
                rospy.logwarn(f"Invalid or missing duration for timed_velocity: {duration}")

        elif intent == "stop": # Handle a potential explicit stop command
             rospy.loginfo("Stop intent received.")
             is_busy = True # Mark as busy briefly to execute stop
             execute_timed_velocity(0.0, 0.0, 0.1) # Publish stop command

        else:
            rospy.logwarn(f"Unsupported intent '{intent}' or execution type '{exec_type}'")

    except json.JSONDecodeError as e:
        rospy.logerr(f"Failed to decode JSON from NLU output: {e}")
        rospy.logerr(f"Received data: {message.data}")
    except KeyError as e:
        rospy.logerr(f"Missing expected key in NLU JSON data: {e}")
        is_busy = False 
    except Exception as e:
        rospy.logerr(f"Error processing NLU output: {e}")
        is_busy = False 

def main():
    global cmd_vel_publisher
    global is_busy
    
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.loginfo(f"{NODE_NAME} started.")

    # Initialize Publisher
    cmd_vel_publisher = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

    # Initialize State
    is_busy = False

    # Initialize Subscriber
    rospy.Subscriber(NLU_OUTPUT_TOPIC, String, nlu_output_callback)

    rospy.loginfo(f"Subscribed to {NLU_OUTPUT_TOPIC}")
    rospy.loginfo(f"Publishing Twist commands to {CMD_VEL_TOPIC}")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception in {NODE_NAME}: {e}")