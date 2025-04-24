#!/usr/bin/env python

import rospy
import json
import ollama 
from std_msgs.msg import String
import re


OLLAMA_MODEL = 'phi3:mini' 
NODE_NAME = 'lingoros_nlu_node'
TEXT_INPUT_TOPIC = '/lingoros/text_input' # Topic to listen for commands
NLU_OUTPUT_TOPIC = '/lingoros/nlu_output' # Topic to publish detailed JSON results


DEFAULT_LINEAR_SPEED = 3 # m/s
DEFAULT_ANGULAR_SPEED = 5 # rad/s


nlu_publisher = None

def build_llm_prompt(command_text):

    available_actions = """
Available Robot Actions & Corresponding ROS Interfaces (ROS 1 Noetic Husky):
1. Direct Velocity Control:
   - Use Topic: /husky_velocity_controller/cmd_vel
   - Message Type: geometry_msgs/Twist
   - Relevant Fields:
     - linear.x: Controls forward (positive value) / backward (negative value) speed in m/s.
     - angular.z: Controls turning speed in rad/s (positive is left, negative is right).
"""
    
    example_angle_rad = 90.0 * 3.14159 / 180.0
    example_duration = example_angle_rad / DEFAULT_ANGULAR_SPEED

    
    prompt = f"""
You are an assistant integrated into a robot's control system (ROS 1 Noetic Husky).
Your task is to interpret user commands and generate a structured JSON output specifying the exact ROS topic, message type, and field values needed for execution.

{available_actions}

Analyze the following user command: "{command_text}"

Determine the intent (e.g., 'move_linear', 'rotate', 'stop'). Based on the available actions, determine the target ROS topic, message type, and calculate the necessary field values for the specified ROS message fields.
- 'forward' corresponds to positive linear.x, 'backward' to negative linear.x.
- Assume a default speed of {DEFAULT_LINEAR_SPEED} m/s for linear motion unless specified otherwise.
- 'left' turn corresponds to positive angular.z, 'right' turn to negative angular.z.
- Assume a default speed of {DEFAULT_ANGULAR_SPEED} rad/s for angular motion unless specified otherwise.
- Assume distance units are meters ('m') and angle units are degrees ('deg') unless specified. Convert degrees to radians for angular velocity calculations if needed (duration = angle_rad / speed_rad_per_sec).

If the command implies a duration or distance (like 'go 2 m forward'), calculate the execution duration based on the default speed and specify an execution type 'timed_velocity'. Duration = distance / speed.

Output ONLY the JSON object with keys: 'intent', 'ros_interface' (containing 'type', 'name', 'message_type'), 'parameter_mapping' (object mapping ROS fields like 'linear.x' or 'angular.z' to their *final calculated values*), and 'execution_details' (containing 'type'='timed_velocity' and 'duration_sec' if applicable). Ensure parameter_mapping contains all relevant fields for the message type (geometry_msgs/Twist has linear.x/y/z and angular.x/y/z), setting others to 0.0 if necessary.

Example output for "turn left 90 degrees":
{{
  "intent": "rotate",
  "ros_interface": {{
    "type": "topic",
    "name": "/husky_velocity_controller/cmd_vel",
    "message_type": "geometry_msgs/Twist"
  }},
  "parameter_mapping": {{
    "linear.x": 0.0,
    "linear.y": 0.0,
    "linear.z": 0.0,
    "angular.x": 0.0,
    "angular.y": 0.0,
    "angular.z": {DEFAULT_ANGULAR_SPEED}
  }},
  "execution_details": {{
    "type": "timed_velocity",
    "duration_sec": {example_duration}
  }}
}}

Now, process the command: "{command_text}"
Result JSON:
"""
    return prompt


def get_structured_command(command_text):
    """Sends command to Ollama, cleans response, and gets structured JSON."""
    prompt = build_llm_prompt(command_text)
    rospy.loginfo("Sending prompt to Ollama...")
  
    try:
        response = ollama.chat(
            model=OLLAMA_MODEL,
            messages=[{'role': 'user', 'content': prompt}],
            options={'temperature': 0.1}
        )

        content = response['message']['content']
        rospy.loginfo(f"Ollama raw response content:\n{content}")

        cleaned_content = re.sub(r"//.*", "", content)
     

        rospy.loginfo(f"Cleaned response content:\n{cleaned_content}") # Log cleaned version

        try:
            
            json_start = cleaned_content.find('{')
            json_end = cleaned_content.rfind('}') + 1

            if json_start != -1 and json_end != -1:
                json_string = cleaned_content[json_start:json_end]
                
                parsed_json = json.loads(json_string)
                rospy.loginfo(f"Successfully parsed JSON: {json_string}")
                return json_string # Return the validated JSON string
            else:
                rospy.logwarn(f"Could not find JSON block in cleaned response: {cleaned_content}")
                return None
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to parse JSON from cleaned Ollama response: {e}\nCleaned content was:\n{cleaned_content}")
            return None
        except Exception as e:
             rospy.logwarn(f"Error processing cleaned Ollama response content: {e}\nCleaned content was:\n{cleaned_content}")
             return None

    except Exception as e:
        rospy.logerr(f"Error communicating with Ollama model '{OLLAMA_MODEL}': {e}")
        return None


def text_command_callback(message):
    """Callback function for receiving text commands."""
    global nlu_publisher
    command = message.data
    rospy.loginfo(f"Received text command: '{command}'")

    structured_json_string = get_structured_command(command)

    if structured_json_string and nlu_publisher:
        rospy.loginfo(f"Publishing NLU output JSON string.")
        nlu_publisher.publish(String(structured_json_string))
    else:
        rospy.logwarn(f"Failed to get structured command for: '{command}'")

def main():
    global nlu_publisher
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.loginfo(f"Starting {NODE_NAME}...")

    # Create Publisher
    nlu_publisher = rospy.Publisher(NLU_OUTPUT_TOPIC, String, queue_size=10)

    # Create Subscriber to listen for text commands
    rospy.Subscriber(TEXT_INPUT_TOPIC, String, text_command_callback)

    rospy.loginfo(f"Subscribed to text commands on: {TEXT_INPUT_TOPIC}")
    rospy.loginfo(f"Publishing structured NLU results to: {NLU_OUTPUT_TOPIC}")
    rospy.loginfo(f"Using Ollama model: {OLLAMA_MODEL}")
    rospy.loginfo(f"{NODE_NAME} ready.")

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"{NODE_NAME} shutting down.")
    except Exception as e:
         rospy.logerr(f"Unhandled exception in {NODE_NAME}: {e}")