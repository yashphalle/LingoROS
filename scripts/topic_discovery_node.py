#!/usr/bin/env python3
import rospy
import yaml
import subprocess
import re
import os
from collections import defaultdict

class ROSTopicDiscovery:
    def __init__(self):
        self.topics_data = {"topics": []}
        # Get the package path for proper config directory location
        self.package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.config_dir = os.path.join(self.package_path, "config")
        
    def discover_topics(self):
        #Discover all active ROS topics and their message types
        rospy.loginfo("Discovering active ROS topics...")
        
        try:
            # Get all published topics using ROS Python API
            published_topics = rospy.get_published_topics()
            
            included_topics = 0
            skipped_topics = 0
            
            for topic_name, msg_type in published_topics:
                # Filter topics based on relevance
                if self.should_include_topic(topic_name):
                    rospy.loginfo(f"Processing topic: {topic_name} ({msg_type})")
                    topic_info = self._get_topic_info(topic_name, msg_type)
                    self.topics_data["topics"].append(topic_info)
                    included_topics += 1
                else:
                    rospy.loginfo(f"Skipping topic: {topic_name}")
                    skipped_topics += 1
                
            rospy.loginfo(f"Discovered {included_topics} topics (skipped {skipped_topics})")
            
        except Exception as e:
            rospy.logerr(f"Error discovering topics: {e}")
    
    def should_include_topic(self, topic_name):
        # Skip internal ROS topics and other non-relevant topics
        if topic_name in ['/rosout', '/rosout_agg', '/clock']:
            return False
            
        # Skip parameter and visualization topics
        if ('/parameter_' in topic_name or 
            '/twist_marker_server/update' in topic_name):
            return False
            
        # Skip gazebo and diagnostic topics
        if topic_name.startswith('/gazebo/') or topic_name == '/diagnostics':
            return False
            
        return True
    
    def _get_topic_info(self, topic_name, msg_type):
        # Collect comprehensive information about a topic
        topic_info = {
            "name": topic_name,
            "type": msg_type,
            "fields": self._get_message_fields(msg_type),
            "publishers": self._get_publishers(topic_name),
            "subscribers": self._get_subscribers(topic_name),
        }
        
        # Try to get frequency if the topic is active
        frequency = self._estimate_frequency(topic_name)
        if frequency is not None:
            topic_info["frequency"] = frequency
            
        return topic_info
    
    def _get_message_fields(self, msg_type):
        # Extract field information from a message type
        fields = []
        
        try:
            # Run rosmsg show command to get message structure
            output = subprocess.check_output(["rosmsg", "show", msg_type]).decode('utf-8')
            
            # Process the output to handle nested fields
            field_stack = []
            
            for line in output.strip().split('\n'):
                # Determine nesting level by counting leading spaces
                leading_spaces = len(line) - len(line.lstrip())
                level = leading_spaces // 2
                
                # Adjust stack based on nesting level
                while len(field_stack) > level:
                    field_stack.pop()
                
                line = line.strip()
                if not line:
                    continue
                
                # Parse field type and name
                parts = line.split(' ', 1)
                if len(parts) >= 2:
                    field_type = parts[0]
                    field_name_parts = parts[1].split('=', 1)
                    field_name = field_name_parts[0].strip()
                    
                    # Check for default values
                    default_value = None
                    if len(field_name_parts) > 1:
                        default_value = field_name_parts[1].strip()
                    
                    # Build the full field name with parent names
                    full_name = '.'.join(field_stack + [field_name]) if field_stack else field_name
                    
                    # Create field info dictionary
                    field_info = {
                        "name": full_name,
                        "data_type": field_type
                    }
                    
                    if default_value:
                        field_info["default"] = default_value
                    
                    fields.append(field_info)
                    
                    # Track complex types for nesting
                    primitive_types = ["int", "float", "bool", "string", "time", "duration"]
                    if not any(primitive in field_type for primitive in primitive_types):
                        field_stack.append(field_name)
            
            return fields
            
        except Exception as e:
            rospy.logwarn(f"Error getting message fields for {msg_type}: {e}")
            return []
    
    def _get_publishers(self, topic_name):
        # Get the list of nodes publishing to this topic
        try:
            output = subprocess.check_output(
                ["rostopic", "info", topic_name], 
                stderr=subprocess.STDOUT
            ).decode('utf-8')
            
            publishers = []
            in_publishers_section = False
            
            # Parse rostopic info output
            for line in output.strip().split('\n'):
                line = line.strip()
                if line.startswith("Publishers:"):
                    in_publishers_section = True
                    continue
                elif line.startswith("Subscribers:"):
                    in_publishers_section = False
                    continue
                
                # Extract node names
                if in_publishers_section and line.startswith("*"):
                    node_name = line[2:].split("(")[0].strip()
                    publishers.append(node_name)
            
            return publishers
            
        except Exception as e:
            rospy.logwarn(f"Error getting publishers for {topic_name}: {e}")
            return []
    
    def _get_subscribers(self, topic_name):
        # Get the list of nodes subscribing to this topic
        try:
            output = subprocess.check_output(
                ["rostopic", "info", topic_name], 
                stderr=subprocess.STDOUT
            ).decode('utf-8')
            
            subscribers = []
            in_subscribers_section = False
            
            # Parse rostopic info output
            for line in output.strip().split('\n'):
                line = line.strip()
                if line.startswith("Publishers:"):
                    in_subscribers_section = False
                    continue
                elif line.startswith("Subscribers:"):
                    in_subscribers_section = True
                    continue
                
                # Extract node names
                if in_subscribers_section and line.startswith("*"):
                    node_name = line[2:].split("(")[0].strip()
                    subscribers.append(node_name)
            
            return subscribers
            
        except Exception as e:
            rospy.logwarn(f"Error getting subscribers for {topic_name}: {e}")
            return []
    
    def _estimate_frequency(self, topic_name):
        # Estimate how frequently a topic is published
        try:
            # Use rostopic hz with a short timeout
            output = subprocess.check_output(
                ["timeout", "1", "rostopic", "hz", topic_name], 
                stderr=subprocess.STDOUT
            ).decode('utf-8')
            
            # Extract the average rate
            for line in output.strip().split('\n'):
                if "average rate:" in line:
                    match = re.search(r"average rate:\s+([\d.]+)", line)
                    if match:
                        return float(match.group(1))
            
            return None
            
        except Exception:
            return None
    
    def save_to_yaml(self, filename=None):
        # Save the discovered topics to a YAML file in the config directory
        if filename is None:
            # Ensure config directory exists
            os.makedirs(self.config_dir, exist_ok=True)
            filename = os.path.join(self.config_dir, "ros_topics.yaml")
            
        try:
            with open(filename, 'w') as file:
                yaml.dump(self.topics_data, file, default_flow_style=False, sort_keys=False)
            rospy.loginfo(f"Saved topic information to {filename}")
            return filename
        except Exception as e:
            rospy.logerr(f"Error saving to YAML file: {e}")
            return None

def main():
    rospy.init_node('ros_topic_discovery', anonymous=True)
    
    discovery = ROSTopicDiscovery()
    discovery.discover_topics()
    
    # Save to the config directory
    yaml_file = discovery.save_to_yaml()
    
    if yaml_file:
        rospy.loginfo(f"ROS Topic Discovery completed. Data saved to {yaml_file}")
    else:
        rospy.logerr("Failed to save topic information")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass