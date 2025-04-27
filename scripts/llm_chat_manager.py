#!/usr/bin/env python

import rospy
import yaml
import os
import time
import json
import re
from openai import OpenAI
from dotenv import load_dotenv

class LLMChatManager:
    def __init__(self, 
                 model_name=None, 
                 temperature=None,
                 topics_yaml_path=None,
                 max_history=5,
                 api_key=None):
        
        # Try multiple locations for .env file
        possible_env_paths = [
            os.path.join(os.getcwd(), '.env'),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '.env'),
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '.env'),
        ]
        
        for env_path in possible_env_paths:
            if os.path.exists(env_path):
                load_dotenv(env_path)
                rospy.loginfo(f"Loaded environment variables from {env_path}")
                break
        
        # Set up API key from multiple possible sources
        if api_key:
            self.api_key = api_key
        else:
            self.api_key = os.environ.get('OPENAI_API_KEY')
            if not self.api_key:
                self.api_key = rospy.get_param('~openai_api_key', None)
        
        if not self.api_key:
            rospy.logerr("No OpenAI API key found. Set OPENAI_API_KEY in .env file, environment variable, or as ROS parameter.")
            raise ValueError("OpenAI API key is required")
            
        # Set up model and parameters
        self.model_name = model_name or os.environ.get('MODEL_NAME') or rospy.get_param('~model_name', 'gpt-3.5-turbo')
        
        temp_str = os.environ.get('TEMPERATURE')
        env_temp = float(temp_str) if temp_str else None
        self.temperature = temperature if temperature is not None else (env_temp if env_temp is not None else rospy.get_param('~temperature', 0.1))
        self.max_history = max_history
            
        # Initialize client and paths
        self.client = OpenAI(api_key=self.api_key)
        
        if topics_yaml_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            self.topics_yaml_path = os.path.join(parent_dir, 'config', 'ros_topics.yaml')
        else:
            self.topics_yaml_path = topics_yaml_path
            
        # Initialize conversation and prompt
        self.history = []
        self.system_prompt = None
        
        # Load topics and build prompt
        self._load_robot_topics()
        self._build_system_prompt()
        
        rospy.loginfo(f"LLM Chat Manager initialized with model: {self.model_name}")
    
    def _load_robot_topics(self):
        # Load ROS topics from YAML
        try:
            if not os.path.exists(self.topics_yaml_path):
                rospy.logerr(f"Topics YAML file not found at: {self.topics_yaml_path}")
                self.robot_topics = {"topics": []}
                return
                
            rospy.loginfo(f"Loading topics from: {self.topics_yaml_path}")
            
            with open(self.topics_yaml_path, 'r') as file:
                self.robot_topics = yaml.safe_load(file)
                
            if 'topics' in self.robot_topics:
                self.available_topics = self.robot_topics['topics']
                rospy.loginfo(f"Loaded {len(self.available_topics)} available topics")
            else:
                rospy.logwarn("No 'topics' key found in YAML file")
                self.available_topics = []
                
        except Exception as e:
            rospy.logerr(f"Failed to load robot topics: {e}")
            self.robot_topics = {"topics": []}
            self.available_topics = []
    
    def _build_system_prompt(self):
        # Create system prompt with available topic information
        if self.available_topics:
            # Group topics by message type for clarity
            topics_by_type = {}
            for topic in self.available_topics:
                topic_name = topic.get('name', '')
                message_type = topic.get('type', '')
                
                if message_type not in topics_by_type:
                    topics_by_type[message_type] = []
                
                topics_by_type[message_type].append(topic)
            
            # Build structured topic overview
            topic_overview = "Available ROS Topics:\n\n"
            
            # Add each message type section
            for message_type, topics in topics_by_type.items():
                topic_overview += f"Message Type: {message_type}\n"
                topic_overview += "Topics:\n"
                
                for topic in topics:
                    topic_name = topic.get('name', '')
                    topic_overview += f"  - {topic_name}\n"
                    
                    # Add key fields, limiting to most relevant ones
                    fields = topic.get('fields', [])
                    if fields:
                        relevant_fields = [f for f in fields if '.' in f.get('name', '') and not f.get('name', '').endswith(']')]
                        if len(relevant_fields) > 10:
                            relevant_fields = relevant_fields[:10]
                        
                        if relevant_fields:
                            topic_overview += "    Key Fields:\n"
                            for field in relevant_fields:
                                field_name = field.get('name', '')
                                field_type = field.get('data_type', 'unknown')
                                topic_overview += f"      - {field_name} (type: {field_type})\n"
                
                topic_overview += "\n"
        else:
            # Fallback for when no topics are found
            topic_overview = "No ROS topics found. Using default topic structure:\n\n"
            topic_overview += """
- Topic: /cmd_vel
  Type: geometry_msgs/Twist
  Fields:
    - linear.x (type: float64)
    - linear.y (type: float64)
    - linear.z (type: float64)
    - angular.x (type: float64)
    - angular.y (type: float64)
    - angular.z (type: float64)
"""
        
        # Create final system prompt
        self.system_prompt = f"""
You are a specialized ROS robot control interface. Your task is to convert natural language commands into structured JSON for robot execution.

{topic_overview}

RULES:
1. ONLY output valid JSON, nothing else
2. DO NOT include explanations, advice, or any text outside the JSON
3. Select the most appropriate topic based on the command and available topics
4. Use reasonable default values for fields if the command doesn't specify them
5. Don't make up topics - only use topics that are listed as available

OUTPUT FORMAT (STRICT JSON, NO OTHER TEXT):
{{
  "intent": "[command intent based on natural language]",
  "ros_interface": {{
    "type": "topic",
    "name": "[appropriate topic from the list]",
    "message_type": "[appropriate message type]"
  }},
  "parameter_mapping": {{
    "[field_name]": [value],
    ...
  }},
  "execution_details": {{
    "type": "[execution type based on command]",
    "duration_sec": [duration if applicable]
  }}
}}

IMPORTANT: 
- OUTPUT JSON ONLY, NO EXPLANATIONS!
- Select the most appropriate topic based on the command and available topics
- Include all required fields for the selected message type
"""
        rospy.loginfo("System prompt constructed with all available robot topics")
    
    def _format_history(self):
        # Format conversation history for LLM context
        messages = [{'role': 'system', 'content': self.system_prompt}]
        
        for entry in self.history:
            messages.append({'role': 'user', 'content': entry['command']})
            messages.append({'role': 'assistant', 'content': entry['response']})
            
        return messages
    
    def _extract_json(self, text):
        # Extract JSON from response text
        json_match = re.search(r'({[\s\S]*})', text)
        if json_match:
            json_str = json_match.group(1)
            try:
                return json.loads(json_str)
            except json.JSONDecodeError:
                rospy.logwarn(f"Failed to parse JSON from response: {json_str}")
                return None
        return None
    
    def send_command(self, command_text):
        # Process command and get JSON response
        try:
            # Prepare message context
            messages = self._format_history()
            messages.append({'role': 'user', 'content': command_text})
            
            start_time = time.time()
            
            # Send to API
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                temperature=self.temperature
            )
            
            response_text = response.choices[0].message.content
            
            # Log performance
            response_time = time.time() - start_time
            rospy.loginfo(f"LLM response received in {response_time:.2f} seconds")
            
            # Process response
            json_result = self._extract_json(response_text)
            if json_result:
                response_text = json.dumps(json_result)
                rospy.loginfo("Successfully extracted JSON from response")
            else:
                rospy.logwarn("Failed to extract valid JSON from response")
            
            # Update history
            self.history.append({
                'command': command_text,
                'response': response_text,
                'timestamp': time.time()
            })
            
            # Maintain history limit
            if len(self.history) > self.max_history:
                self.history = self.history[-self.max_history:]
                
            return response_text
            
        except Exception as e:
            rospy.logerr(f"Error communicating with LLM: {e}")
            return None
    
    def reset_session(self):
        # Clear conversation history
        self.history = []
        rospy.loginfo("Chat session history reset")
        return True