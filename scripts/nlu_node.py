#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
import os
import sys

# Add parent directory to Python path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from scripts.llm_chat_manager import LLMChatManager

NODE_NAME = 'lingoros_language_understanding_node'
TEXT_INPUT_TOPIC = '/lingoros/text_input'  # Commands from user
NLU_OUTPUT_TOPIC = '/lingoros/nlu_output'  # JSON output for robot control

# Global variables
nlu_publisher = None
llm_processor = None

def process_text_command(message):
    # Process incoming text commands and publish structured JSON
    global nlu_publisher, llm_processor
    
    if llm_processor is None:
        rospy.logerr("LLM processor not initialized")
        return
        
    user_command = message.data
    rospy.loginfo(f"Received command: '{user_command}'")

    # Process through LLM
    structured_json = llm_processor.send_command(user_command)

    if structured_json and nlu_publisher:
        try:
            # Validate JSON before publishing
            json.loads(structured_json)
            rospy.loginfo("Publishing structured command JSON")
            nlu_publisher.publish(String(structured_json))
        except json.JSONDecodeError:
            rospy.logerr(f"Invalid JSON response: {structured_json}")
    else:
        rospy.logwarn(f"Failed to process command: '{user_command}'")

def main():
    global nlu_publisher, llm_processor
    
    # Initialize ROS node
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.loginfo(f"Starting {NODE_NAME}...")
    
    # Get configuration from ROS parameters
    model_id = rospy.get_param('~model_name', 'gpt-3.5-turbo')
    temp_value = rospy.get_param('~temperature', 0.1)
    topics_file = rospy.get_param('~topics_yaml_path', None)
    api_key = rospy.get_param('~openai_api_key', None)
    
    # Initialize LLM processor
    try:
        llm_processor = LLMChatManager(
            model_name=model_id,
            temperature=temp_value,
            topics_yaml_path=topics_file,
            api_key=api_key
        )
        rospy.loginfo(f"LLM processor initialized with model {model_id}")
    except Exception as e:
        rospy.logerr(f"LLM processor initialization failed: {e}")
        return

    # Set up communication channels
    nlu_publisher = rospy.Publisher(NLU_OUTPUT_TOPIC, String, queue_size=10)
    rospy.Subscriber(TEXT_INPUT_TOPIC, String, process_text_command)

    # Log configuration
    rospy.loginfo(f"Listening for commands on: {TEXT_INPUT_TOPIC}")
    rospy.loginfo(f"Publishing structured commands to: {NLU_OUTPUT_TOPIC}")
    rospy.loginfo(f"{NODE_NAME} ready for operation")

    # Run until shutdown
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"{NODE_NAME} shutting down")
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")