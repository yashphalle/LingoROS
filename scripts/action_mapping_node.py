#!/usr/bin/env python
import rospy
import json
import time
import yaml
import os
import importlib
import threading
from std_msgs.msg import String

# Node configuration constants
NODE_NAME = 'lingoros_action_mapping_node'
NLU_OUTPUT_TOPIC = '/lingoros/nlu_output'
CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
DEFAULT_PUBLISH_RATE = 10  # Hz

# Global resource tracking
publishers = {}       # Stores publishers by topic name
topic_cache = {}      # Caches topic metadata for faster lookup
active_actions = {}   # Tracks currently running timed actions
action_locks = {}     # Ensures thread safety for actions

def load_topic_cache():
    # Loads ROS topic information from YAML config into memory cache
    global topic_cache
    
    topics_yaml_path = os.path.join(CONFIG_DIR, 'ros_topics.yaml')
    if not os.path.exists(topics_yaml_path):
        rospy.logwarn(f"Topics YAML file not found at: {topics_yaml_path}")
        return
    
    try:
        with open(topics_yaml_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
            
        if 'topics' in yaml_data and isinstance(yaml_data['topics'], list):
            # Convert list to dictionary for faster lookup
            for topic in yaml_data['topics']:
                topic_name = topic.get('name')
                if topic_name:
                    topic_cache[topic_name] = topic
            rospy.loginfo(f"Loaded {len(topic_cache)} topics into cache")
        else:
            rospy.logwarn("No valid 'topics' list found in YAML file")
    except Exception as e:
        rospy.logerr(f"Failed to load topic cache: {e}")

def get_msg_type_class(msg_type_name):
    # Dynamically imports and returns the appropriate ROS message class
    try:
        if '/' not in msg_type_name:
            rospy.logerr(f"Invalid message type format: {msg_type_name}")
            return None
            
        # Split "geometry_msgs/Twist" into package and message name
        package_name, msg_name = msg_type_name.split('/')
        
        # Import module and get class
        module = importlib.import_module(f"{package_name}.msg")
        msg_class = getattr(module, msg_name)
        return msg_class
    except ImportError:
        rospy.logerr(f"Could not import module for message type: {msg_type_name}")
        return None
    except AttributeError:
        rospy.logerr(f"Message type not found: {msg_type_name}")
        return None
    except Exception as e:
        rospy.logerr(f"Failed to get message type class for {msg_type_name}: {e}")
        return None

def get_publisher(topic_name, msg_type_name):
    # Gets existing publisher or creates a new one for the specified topic
    global publishers
    
    if topic_name in publishers:
        return publishers[topic_name]['publisher']
    
    try:
        # Create new publisher if it doesn't exist
        msg_class = get_msg_type_class(msg_type_name)
        if not msg_class:
            rospy.logerr(f"Failed to get message class for {msg_type_name}")
            return None
        
        publisher = rospy.Publisher(topic_name, msg_class, queue_size=10)
        publishers[topic_name] = {
            'publisher': publisher, 
            'msg_type': msg_type_name,
            'class': msg_class
        }
        rospy.loginfo(f"Created publisher for topic: {topic_name} ({msg_type_name})")
        return publisher
    except Exception as e:
        rospy.logerr(f"Failed to create publisher for {topic_name}: {e}")
        return None

def create_message(msg_type_name, params):
    # Creates a ROS message with the supplied parameter values
    try:
        msg_class = get_msg_type_class(msg_type_name)
        if not msg_class:
            return None
        
        msg = msg_class()
        
        # Fill message fields with parameter values
        for field_path, value in params.items():
            set_message_field(msg, field_path, value)
            
        return msg
    except Exception as e:
        rospy.logerr(f"Failed to create message of type {msg_type_name}: {e}")
        return None

def set_message_field(msg, field_path, value):
    # Sets a nested field in a ROS message (handles paths like "linear.x")
    try:
        # Split nested field paths
        path_parts = field_path.split('.')
        
        # Navigate to parent object
        current = msg
        for i in range(len(path_parts) - 1):
            current = getattr(current, path_parts[i])
        
        # Set the actual field value
        setattr(current, path_parts[-1], value)
    except Exception as e:
        rospy.logerr(f"Failed to set field {field_path}: {e}")

def create_empty_message(msg_type_name):
    # Creates an empty message (used for stop commands)
    try:
        msg_class = get_msg_type_class(msg_type_name)
        if not msg_class:
            return None
        
        return msg_class()
    except Exception as e:
        rospy.logerr(f"Failed to create empty message of type {msg_type_name}: {e}")
        return None

def execute_timed_action(topic_name, msg, duration):
    # Executes a command for a specific duration then stops
    global active_actions, action_locks
    
    # Thread safety for this action
    if topic_name not in action_locks:
        action_locks[topic_name] = threading.Lock()
    
    with action_locks[topic_name]:
        # Cancel any existing action on this topic
        if topic_name in active_actions:
            active_actions[topic_name] = False
            time.sleep(0.1)  # Give other thread time to notice cancellation
        
        # Mark this action as active
        active_actions[topic_name] = True
    
    # Get publisher information
    publisher_info = publishers.get(topic_name)
    if not publisher_info:
        rospy.logerr(f"No publisher found for topic: {topic_name}")
        return
    
    publisher = publisher_info['publisher']
    msg_type = publisher_info['msg_type']
    
    # Set up timing parameters
    rate = rospy.Rate(DEFAULT_PUBLISH_RATE)
    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(duration)
    
    rospy.loginfo(f"Executing timed action on {topic_name} for {duration:.2f} seconds")
    
    # Repeatedly publish until duration expires or action is canceled
    while (rospy.Time.now() < end_time and 
           not rospy.is_shutdown() and 
           active_actions.get(topic_name, False)):
        publisher.publish(msg)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Execution interrupted")
            break
    
    # Send stop command if action completed naturally (not cancelled)
    if active_actions.get(topic_name, False):
        if msg_type.endswith('/Twist'):  # For velocity commands
            stop_msg = create_empty_message(msg_type)
            publisher.publish(stop_msg)
            rospy.loginfo(f"Timed action completed, sending stop to {topic_name}")
    
    with action_locks[topic_name]:
        # Only mark inactive if this thread is still the active one
        if active_actions.get(topic_name, False):
            active_actions[topic_name] = False

def execute_continuous_action(topic_name, msg):
    # Publishes a message once without stopping automatically
    publisher_info = publishers.get(topic_name)
    if not publisher_info:
        rospy.logerr(f"No publisher found for topic: {topic_name}")
        return
    
    publisher_info['publisher'].publish(msg)
    rospy.loginfo(f"Published continuous action to {topic_name}")

def stop_all_actions():
    # Emergency stop function - halts all active actions
    global active_actions, publishers
    
    for topic_name, active in list(active_actions.items()):
        if active:
            publisher_info = publishers.get(topic_name)
            if publisher_info:
                # Send stop command
                stop_msg = create_empty_message(publisher_info['msg_type'])
                publisher_info['publisher'].publish(stop_msg)
                rospy.loginfo(f"Stopping action on {topic_name}")
            
            # Mark action as inactive
            active_actions[topic_name] = False
    
    rospy.loginfo("All actions stopped")

def nlu_output_callback(message):
    # Processes structured JSON from NLU and executes corresponding robot actions
    rospy.loginfo("Received NLU output")
    
    try:
        # Parse the NLU data
        nlu_data = json.loads(message.data)
        rospy.loginfo(f"Parsed NLU data: {nlu_data}")
        
        # Extract key information
        intent = nlu_data.get('intent')
        ros_interface = nlu_data.get('ros_interface', {})
        param_mapping = nlu_data.get('parameter_mapping', {})
        exec_details = nlu_data.get('execution_details', {})
        
        # Get interface details
        interface_type = ros_interface.get('type')
        target_topic = ros_interface.get('name')
        msg_type = ros_interface.get('message_type')
        
        # Special handling for stop command
        if intent == "stop":
            rospy.loginfo("Stop intent received")
            stop_all_actions()
            return
        
        # Validate required fields exist
        if not interface_type or not target_topic or not msg_type:
            rospy.logwarn("Missing required interface details in NLU output")
            return
        
        # Handle topic publications
        if interface_type == 'topic':
            publisher = get_publisher(target_topic, msg_type)
            if not publisher:
                rospy.logerr(f"Failed to get publisher for {target_topic}")
                return
            
            # Create message from parameters
            msg = create_message(msg_type, param_mapping)
            if not msg:
                rospy.logerr(f"Failed to create message of type {msg_type}")
                return
            
            # Handle different execution types
            exec_type = exec_details.get('type')
            
            if exec_type == 'timed_velocity' or exec_type == 'timed_action':
                # Execute time-limited actions (like "go forward for 2 seconds")
                duration = exec_details.get('duration_sec')
                if duration is not None and isinstance(duration, (int, float)) and duration > 0:
                    # Run in background thread to avoid blocking
                    thread = threading.Thread(
                        target=execute_timed_action,
                        args=(target_topic, msg, duration)
                    )
                    thread.daemon = True
                    thread.start()
                else:
                    rospy.logwarn(f"Invalid or missing duration: {duration}")
            
            elif exec_type == 'continuous':
                # Execute continuous action (like "start moving forward")
                execute_continuous_action(target_topic, msg)
            
            elif exec_type == 'single' or not exec_type:
                # One-time action (like "take a picture")
                publisher.publish(msg)
                rospy.loginfo(f"Published single action to {target_topic}")
            
            else:
                rospy.logwarn(f"Unsupported execution type: {exec_type}")
                
        elif interface_type == 'service':
            # For future implementation
            rospy.logwarn("Service calls not yet implemented")
            
        else:
            rospy.logwarn(f"Unsupported interface type: {interface_type}")
            
    except json.JSONDecodeError as e:
        rospy.logerr(f"Failed to decode JSON from NLU output: {e}")
        rospy.logerr(f"Received data: {message.data}")
    except Exception as e:
        rospy.logerr(f"Error processing NLU output: {e}")

def main():
    # Main entry point - initializes node and sets up subscriptions
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.loginfo(f"{NODE_NAME} started")
    
    # Optional startup delay (useful for waiting for other nodes)
    launch_delay = rospy.get_param('~launch_delay', 0.0)
    if launch_delay > 0:
        rospy.loginfo(f"Waiting {launch_delay} seconds before starting...")
        time.sleep(launch_delay)
    
    # Load available topics
    load_topic_cache()
    
    # Set up subscribers
    rospy.Subscriber(NLU_OUTPUT_TOPIC, String, nlu_output_callback)
    rospy.Subscriber('/lingoros/emergency_stop', String, 
                    lambda msg: stop_all_actions() if msg.data.lower() == 'stop' else None)
    
    rospy.loginfo(f"Subscribed to {NLU_OUTPUT_TOPIC}")
    rospy.loginfo("Ready to process commands for any ROS topic")
    
    # Keep node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception in {NODE_NAME}: {e}")