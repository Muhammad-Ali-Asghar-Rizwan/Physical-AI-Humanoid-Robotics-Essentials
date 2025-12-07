# Module 4 - Vision-Language-Action (VLA)

## 1. Introduction to Vision-Language-Action (VLA)

Vision-Language-Action (VLA) represents the convergence of three critical artificial intelligence domains: computer vision, natural language processing, and robot control. In this module, we'll explore how to build systems that can receive natural language commands, interpret visual environments, and execute physical actions as a unified AI-agent.

### What is VLA? The convergence of AI and robotics

VLA systems extend traditional robotics by providing a high-level interface through natural language that abstracts away complex robot programming. Instead of manually programming each movement and decision, users can communicate with robots using everyday language. This paradigm shift enables non-expert users to direct complex robotic behaviors by expressing their goals in linguistic terms.

The VLA architecture processes language to generate action plans, uses vision to understand and interact with the environment, and controls the robot to execute these plans. This integration allows robots to operate flexibly in human environments, understanding both the semantics of language and the spatial relationships of objects.

### Evolution: rule-based → learning-based → language-based control

Robot control has evolved through distinct phases:

1. **Rule-based systems**: Early robots required explicit programming for every operation. Each action was hardcoded with specific conditions and responses, limiting flexibility to predefined scenarios.

2. **Learning-based systems**: Machine learning enabled robots to learn from experience and adapt to new situations without explicit programming. Reinforcement learning and imitation learning allowed robots to acquire complex behaviors through interaction.

3. **Language-based systems**: Natural language interfaces abstract the complexity of robot programming, allowing humans to express goals and tasks using their natural communication method.

### Why now? LLMs meet embodied AI

The recent advances in Large Language Models (LLMs) make VLA possible for the first time:

- **Complex reasoning**: LLMs can decompose complex tasks into sequences of primitive actions
- **World knowledge**: LLMs possess common-sense knowledge about the physical world
- **Natural interfaces**: Language is the most natural human-machine interaction modality
- **Transferability**: LLMs can generalize knowledge to new tasks and environments

### Real-world VLA applications (RT-1, RT-2, PaLM-E)

Several groundbreaking VLA systems have demonstrated the power of this approach:

- **RT-1 (Robotics Transformer 1)**: Google's system that translates natural language to robot actions using language conditioning
- **RT-2**: An evolution that incorporates web knowledge to improve robot reasoning
- **PaLM-E**: A multimodal language model that can plan robot trajectories from visual input and language

### Overview: voice command → physical robot action

The VLA pipeline follows this sequence:
1. Voice command received and transcribed
2. Language model interprets intent and generates action plan
3. Vision system perceives environment and identifies objects
4. Action execution with real-time feedback
5. Status reporting and potential replanning

### What students will build in this module

By the end of this module, students will have built a complete VLA system that:

- Recognizes voice commands with >90% accuracy using Whisper
- Plans multi-step tasks using LLMs (GPT-4/Claude)
- Executes complex missions like "clean the room" autonomously
- Integrates with Isaac Sim for safe testing
- Handles failures and replanning gracefully
- Includes human-in-the-loop safety overrides

## 2. The VLA Architecture

### Pipeline overview: Speech → Language → Vision → Action

The VLA architecture is a distributed system that connects multiple specialized components:

```
[Voice Input] → [Speech-to-Text] → [NLP/LLM] → [Vision System] → [Action Execution] → [Physical Robot]
       ↓              ↓              ↓            ↓                  ↓                    ↓
  Raw Audio   →  Transcribed Text → Intent → Perceived Scene → Action Plan → Physical Movement
```

Each component operates with specific responsibilities while sharing information through well-defined interfaces.

### Component breakdown:

**Voice Interface (Whisper)**
- Converts spoken language to text
- Handles real-time streaming
- Processes multiple languages
- Filters background noise

**Task Planner (LLM: GPT-4/Claude)**
- Interprets user intent from text commands
- Decomposes high-level goals into action sequences
- Incorporates world knowledge for decision-making
- Generates structured action plans

**Perception System (from Module 3)**
- Processes RGB-D camera data
- Detects and localizes objects in the environment
- Estimates 3D positions and orientations
- Provides semantic understanding of scene

**Action Executor (ROS 2 actions)**
- Translates abstract actions to robot commands
- Manages robot motion and manipulation
- Provides feedback about execution status
- Handles failures and error conditions

**Feedback Loop (replanning)**
- Monitors task execution progress
- Detects failures and obstacles
- Requests replanning when needed
- Reports status to user

### Data flow diagram explanation

```
Voice Command (e.g., "Clean the room")
              ↓
        Whisper STT → "Clean the room"
              ↓
    LLM Planner → [navigate_to(table), detect(object), grasp(object), ...]
              ↓
     Perception → "Cup detected at (2.1, 1.5, 0.8)"
              ↓
 Action Executor → Grasp cup, lift, navigate to bin
              ↓
    Physical Robot → Executed motion
              ↓
     Monitoring → Success/Failure feedback
```

### Integration with previous modules

The VLA system builds upon the foundations from Modules 1-3:

- **Module 1 (ROS 2)**: Provides the communication infrastructure for distributed components
- **Module 2 (Gazebo/Unity)**: Offers simulation environments for testing
- **Module 3 (Isaac)**: Supplies vision systems and perception capabilities

## 3. Voice Interface: OpenAI Whisper

### What is Whisper? Speech-to-text foundation model

Whisper is OpenAI's automatic speech recognition (ASR) system trained on 680,000 hours of multilingual and multitask supervised data. It performs robustly across multiple languages, accents, and domains, making it ideal for robotics applications where voice commands may come from diverse users.

Whisper's architecture is based on the Transformer sequence-to-sequence model, which can handle both speech recognition and speech translation. The model is trained jointly on transcription, translation, and language identification tasks.

### Model sizes: tiny, base, small, medium, large

Whisper comes in several sizes with trade-offs between accuracy and computational requirements:

```
Model   | Size | Required VRAM | Real-time Factor | Accuracy
--------|------|---------------|------------------|----------
tiny    | 75MB | ~1GB          | ~32x             | Lower
base    | 145MB| ~1GB          | ~16x             | Low-medium
small   | 480MB| ~2GB          | ~6x              | Medium-high
medium  | 1.5GB| ~5GB          | ~2x              | High
large   | 3.0GB| ~10GB         | ~1x              | Highest
```

For robotics applications, "medium" typically provides the best balance of accuracy and resource usage.

### Accuracy vs latency trade-offs

When selecting a Whisper model for robotics, consider:

- **Accuracy**: Higher accuracy reduces misinterpretation of commands
- **Latency**: Real-time applications require quick responses
- **Hardware**: Available GPU memory limits model choices
- **Environment**: Noisy environments may require more accurate models

### Installation and setup

```bash
# Install Whisper
pip install openai-whisper

# Install system dependencies (for audio processing)
sudo apt update
sudo apt install ffmpeg python3-pyaudio
```

### Real-time vs batch transcription

Whisper can operate in two modes:

**Batch mode**: Processes complete audio files; highest accuracy but higher latency
```python
import whisper
model = whisper.load_model("medium")
result = model.transcribe("audio.mp3")
print(result["text"])
```

**Real-time streaming**: Processes audio chunks as they arrive; lower latency but moderate accuracy
```python
# We'll implement this in the next section
```

### Language support (multilingual capability)

Whisper supports transcription in 99 languages. The model automatically detects the input language, making it suitable for international robotics applications.

## 4. Setting Up Whisper for Robotics

### Installation: pip install openai-whisper

```bash
# Install Whisper dependencies
pip install openai-whisper
pip install pyaudio  # For microphone input
pip install sounddevice  # Alternative audio library
```

### Audio capture (microphone input)

```python
import pyaudio
import wave
import numpy as np
import whisper
import threading
import queue
import time

class AudioCapture:
    def __init__(self, rate=16000, chunk=1024, channels=1):
        self.rate = rate
        self.chunk = chunk
        self.channels = channels
        self.format = pyaudio.paInt16
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.is_recording = False
        self.audio_queue = queue.Queue()

    def start_recording(self):
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        self.is_recording = True
        
        def record():
            while self.is_recording:
                data = self.stream.read(self.chunk)
                self.audio_queue.put(data)
        
        self.recording_thread = threading.Thread(target=record)
        self.recording_thread.start()

    def stop_recording(self):
        self.is_recording = False
        if self.recording_thread:
            self.recording_thread.join()
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
```

### Real-time streaming setup

```python
import whisper
import torch
import numpy as np
from scipy.io.wavfile import write

class RealTimeWhisper:
    def __init__(self, model_name="medium", language="en"):
        # Check if CUDA is available
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_name).to(device)
        self.language = language
        self.audio_buffer = np.array([])
        
    def process_audio_chunk(self, audio_chunk):
        # Append chunk to buffer
        self.audio_buffer = np.concatenate([self.audio_buffer, audio_chunk])
        
        # Process only if buffer is large enough (e.g., 1 second of audio)
        if len(self.audio_buffer) >= 16000:  # 1 second at 16kHz
            # Process with Whisper
            result = self.model.transcribe(
                self.audio_buffer,
                language=self.language,
                fp16=torch.cuda.is_available()
            )
            
            # Clear buffer (keeping some overlap to preserve context)
            self.audio_buffer = self.audio_buffer[8000:]  # Keep half a second
            
            return result["text"]
        
        return None
```

### Transcription with timestamps

```python
def transcribe_with_timestamps(self, audio_chunk):
    # Process with word-level timestamps
    result = self.model.transcribe(
        audio_chunk,
        word_timestamps=True,
        language=self.language
    )
    
    return result
```

### Handling background noise

```python
import webrtcvad  # Voice Activity Detection

class NoiseResistantWhisper:
    def __init__(self, whisper_model, vad_aggressiveness=3):
        self.whisper = whisper_model
        self.vad = webrtcvad.Vad(aggressiveness)
        self.audio_buffer = b""
        
    def is_speech(self, audio_data, sample_rate=16000):
        try:
            return self.vad.is_speech(audio_data, sample_rate)
        except:
            return False  # If VAD fails, assume it's speech
```

### Wake word detection (optional)

```python
import speech_recognition as sr

def detect_wake_word(self, audio_chunk, wake_words=["robot", "hey robot"]):
    # Simple energy-based wake word detection
    audio_level = np.sqrt(np.mean(audio_chunk**2))
    
    if audio_level > self.threshold:  # Above noise floor
        # Perform more expensive Whisper processing
        text = self.whisper.process_audio_chunk(audio_chunk)
        
        for word in wake_words:
            if word.lower() in text.lower():
                return True, text
    
    return False, None
```

## 5. Building the Voice Command Node

### ROS 2 node for audio capture

```python
# voice_command_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
import whisper
import torch
import threading
import queue

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Publisher for recognized commands
        self.command_publisher = self.create_publisher(String, '/voice_command', 10)
        
        # Initialize Whisper model
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.whisper_model = whisper.load_model("medium").to(device)
        
        # Audio parameters
        self.rate = 16000
        self.chunk = 8192  # Larger chunks for better recognition
        self.channels = 1
        self.format = pyaudio.paInt16
        
        # Audio buffer for streaming
        self.audio_buffer = np.array([])
        
        # Audio capture setup
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        # Start audio capture in separate thread
        self.capture_thread = threading.Thread(target=self.capture_audio)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info("Voice Command Node initialized")

    def capture_audio(self):
        while rclpy.ok():
            try:
                # Read audio chunk
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                
                # Append to buffer
                self.audio_buffer = np.concatenate([self.audio_buffer, audio_chunk])
                
                # Process if buffer has 2 seconds of audio
                if len(self.audio_buffer) >= 2 * self.rate:
                    # Transcribe the audio
                    transcript = self.transcribe_audio(self.audio_buffer)
                    
                    if transcript and len(transcript.strip()) > 0:
                        # Publish the recognized command
                        msg = String()
                        msg.data = transcript.strip()
                        self.command_publisher.publish(msg)
                        self.get_logger().info(f"Recognized command: {transcript}")
                    
                    # Keep only last 0.5 seconds to avoid repeated recognition
                    self.audio_buffer = self.audio_buffer[int(0.5 * self.rate):]
                    
            except Exception as e:
                self.get_logger().error(f"Audio capture error: {e}")
                break

    def transcribe_audio(self, audio_data):
        try:
            # Convert audio to required format
            audio = whisper.pad_or_trim(audio_data)
            
            # Convert to log-mel spectrogram
            mel = whisper.log_mel_spectrogram(audio).to(self.whisper_model.device)
            
            # Decode the audio
            _, probs = self.whisper_model.detect_language(mel)
            detected_lang = max(probs, key=probs.get)
            
            options = whisper.DecodingOptions(fp16=torch.cuda.is_available())
            result = whisper.decode(self.whisper_model, mel, options)
            
            return result.text
        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")
            return None

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down voice command node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Integration with Whisper API

The node above uses the local Whisper model. For cloud-based processing:

```python
import openai
import os

class CloudWhisperNode(Node):
    def __init__(self):
        super().__init__('cloud_voice_command_node')
        self.command_publisher = self.create_publisher(String, '/voice_command', 10)
        
        # Set up OpenAI API key
        openai.api_key = os.getenv("OPENAI_API_KEY")
        
        # Audio parameters
        self.temp_audio_file = "/tmp/current_audio.wav"
        
    def transcribe_audio_cloud(self, audio_data):
        # Write audio data to temporary file
        import wave
        with wave.open(self.temp_audio_file, 'w') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(audio_data.astype(np.int16).tobytes())
        
        # Transcribe using OpenAI API
        try:
            with open(self.temp_audio_file, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
                return transcript.text
        except Exception as e:
            self.get_logger().error(f"Cloud transcription error: {e}")
            return None
```

### Publishing transcribed commands

The voice command node publishes transcribed text to the `/voice_command` topic, where it can be consumed by the VLA controller:

```python
# In the main processing loop
if transcript and len(transcript.strip()) > 0:
    msg = String()
    msg.data = transcript.strip()
    self.command_publisher.publish(msg)
    self.get_logger().info(f"Recognized command: {transcript}")
```

### Testing voice input: "Navigate to the table"

```bash
# Test the voice node
ros2 run your_package voice_command_node

# Listen to recognized commands
ros2 topic echo /voice_command
```

### Error handling: silence, unclear speech

```python
def handle_transcription_errors(self, transcript):
    if not transcript:
        return "SILENCE_DETECTED"
    elif "thank you" in transcript.lower():
        return "THANK_YOU_IGNORE"
    elif len(transcript.strip()) < 3:  # Too short to be meaningful
        return "TOO_SHORT"
    else:
        return transcript
```

### Code example: whisper_voice_node.py

This is implemented in the ROS 2 node above with all the functionality for audio capture, Whisper processing, and command publishing.

## 6. Understanding Large Language Models for Robotics

### LLMs as task planners, not just chatbots

In the VLA context, LLMs serve as sophisticated task planners rather than conversational agents. They bridge the gap between human intention (expressed in natural language) and robot execution (as sequences of primitive actions). The key difference from chatbots is that LLMs in robotics must produce structured, executable outputs rather than natural language responses.

### Prompt engineering for robot control

Effective VLA systems require specialized prompt engineering that:

- Defines robot capabilities explicitly
- Provides structured output formats
- Incorporates safety constraints
- Includes environmental context
- Specifies failure handling

### Few-shot learning with examples

LLMs excel at few-shot learning where examples guide the output format and content. For robotics, this means providing a few examples of language commands and desired action sequences.

### Structured output (JSON action sequences)

```python
SYSTEM_PROMPT = """
You are a humanoid robot controller. Your role is to interpret natural language commands and generate structured action sequences.

For each command, output a JSON array of action objects with the following possible actions:

- { "action": "navigate_to", "location": "[target location description]" }
- { "action": "detect_object", "object": "[object name]", "color": "[optional color]", "location": "[optional location]" }
- { "action": "grasp_object", "object": "[object name]", "location": "[optional location]" }
- { "action": "place_object", "location": "[target location for placement]" }
- { "action": "scan_area", "location": "[area to scan]" }
- { "action": "report_status", "message": "[status message to report]" }

Important safety constraints: 
- Do not move the robot if a human is in the path
- Do not grasp objects near humans
- Always scan before navigation in unknown environments
- If uncertain about object identity, request clarification
"""
```

### Limitations: hallucinations, context length

LLMs have important limitations for robotics:

- **Hallucinations**: LLMs may generate physically impossible or unsafe actions
- **Context length**: Limited to ~4K tokens of conversation history
- **Temporal reasoning**: Struggle with multi-step planning requiring long-term memory
- **Physical reasoning**: May not understand physical constraints of the robot

### Safety constraints in prompts

```python
SAFETY_PROMPT = """
Before generating any action sequence, confirm that it adheres to these safety rules:
1. No action should cause harm to humans in the environment
2. No action should damage the robot or environment
3. No action should violate privacy constraints
4. Navigation should avoid obstacles and humans
5. Grasping should be appropriate for the robot's capabilities

If any action in your planned sequence violates these rules, modify the plan to comply with safety constraints.
"""
```

## 7. LLM Integration: GPT-4 / Claude API

### Choosing an LLM (GPT-4, Claude, or open-source)

For robotics applications, different LLMs have trade-offs:

**GPT-4**: 
- Pros: Excellent reasoning, good safety alignment, reliable
- Cons: Expensive, rate limits, no offline option
- Best for: Complex task planning, safety-critical applications

**Claude**:
- Pros: Good reasoning, long context, safety-focused
- Cons: Access limitations, cost
- Best for: Complex reasoning tasks with safety requirements

**Open-source models (Llama, Mistral)**:
- Pros: No API costs, full control, offline capability
- Cons: Less reliable reasoning, requires more prompt engineering
- Best for: Cost-sensitive applications, offline deployments

### API setup and authentication

```python
import openai
import os
import json
from typing import List, Dict

class LLMPlanner:
    def __init__(self, model_name="gpt-4", api_key=None):
        if api_key:
            openai.api_key = api_key
        else:
            openai.api_key = os.getenv("OPENAI_API_KEY")
        
        self.model_name = model_name
        self.system_prompt = """
        You are a humanoid robot controller. Your role is to interpret natural language commands and generate structured action sequences in JSON format.
        
        Available actions:
        - navigate_to: Move robot to specified location
        - detect_object: Locate specific object in environment
        - grasp_object: Pick up an object
        - place_object: Put down an object at specified location
        - scan_area: Look around to map environment
        - report_status: Communicate status to user
        
        Output format: [{"action": "...", "parameters": {...}}]
        """
    
    def plan_task(self, command: str) -> List[Dict]:
        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": f"Command: {command}"}
        ]
        
        try:
            response = openai.ChatCompletion.create(
                model=self.model_name,
                messages=messages,
                temperature=0.1,  # Low temperature for consistent outputs
                max_tokens=500,
                response_format={"type": "json_object"}  # Force JSON output
            )
            
            # Parse the JSON response
            action_json = json.loads(response.choices[0].message['content'])
            return action_json.get('actions', [])
            
        except Exception as e:
            print(f"LLM API error: {e}")
            # Return a safe fallback action
            return [{"action": "report_status", "message": f"Unable to process command: {str(e)}"}]
```

### Rate limiting and cost considerations

```python
import time
from functools import wraps

def rate_limit(calls_per_minute=60):
    min_interval = 60.0 / calls_per_minute
    last_called = [0.0]

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            elapsed = time.time() - last_called[0]
            left_to_wait = min_interval - elapsed
            if left_to_wait > 0:
                time.sleep(left_to_wait)
            ret = func(*args, **kwargs)
            last_called[0] = time.time()
            return ret
        return wrapper
    return decorator

class RateLimitedLLMPlanner(LLMPlanner):
    @rate_limit(calls_per_minute=30)  # Conservative rate limit
    def plan_task(self, command: str) -> List[Dict]:
        return super().plan_task(command)
```

### Designing system prompts for robotics

```python
class RobotSystemPrompt:
    def __init__(self):
        self.base_prompt = """
        You are a humanoid robot controller with these capabilities:
        - Navigation: can move to named locations (kitchen, table, bin, etc.)
        - Object detection: can identify and locate objects in the environment
        - Manipulation: can grasp and place objects
        - Communication: can report status and ask for clarification
        
        You must ALWAYS:
        1. Respond with a valid JSON array of action objects
        2. Respect safety constraints (no harm to humans, no dangerous movements)
        3. Ask for clarification if commands are ambiguous
        4. Include error handling in your plans
        
        Action format: {"action": "action_name", "parameters": {...}}
        """
    
    def create_environment_context(self, location_map, object_types):
        return f"""
        The robot is in an environment with these locations: {', '.join(location_map.keys())}
        The robot can recognize these object types: {', '.join(object_types)}
        """
```

### Example prompt: "You are a humanoid robot controller..."

```python
def create_robot_controller_prompt(robot_capabilities, environment_context):
    return f"""
    You are a humanoid robot controller with these capabilities:
    {robot_capabilities}
    
    {environment_context}
    
    When given a natural language command, you will:
    1. Interpret the user's intent
    2. Decompose the high-level task into primitive actions
    3. Consider safety constraints and environmental factors
    4. Output a JSON array of action objects with this format:
       [
         {{"action": "navigate_to", "location": "kitchen"}},
         {{"action": "detect_object", "object": "cup", "color": "red"}},
         {{"action": "grasp_object", "object": "cup"}},
         {{"action": "place_object", "location": "counter"}}
       ]
    
    Always ensure your action plan accounts for the robot's physical limitations and safety requirements.
    """
```

### Response parsing and validation

```python
import json
import re
from typing import List, Dict, Any

class ResponseValidator:
    def __init__(self):
        self.valid_actions = {
            "navigate_to", "detect_object", "grasp_object", 
            "place_object", "scan_area", "report_status"
        }
    
    def clean_response(self, response_text: str) -> str:
        # Extract JSON from response that might contain additional text
        json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
        if json_match:
            return json_match.group(0)
        return response_text
    
    def validate_action(self, action: Dict[str, Any]) -> bool:
        if "action" not in action:
            return False
        
        action_type = action["action"]
        if action_type not in self.valid_actions:
            return False
        
        # Validate required parameters for each action type
        required_params = {
            "navigate_to": ["location"],
            "detect_object": ["object"],
            "grasp_object": ["object"],
            "place_object": ["location"]
        }
        
        if action_type in required_params:
            for param in required_params[action_type]:
                if param not in action:
                    return False
        
        return True
    
    def validate_response(self, response_text: str) -> List[Dict[str, Any]]:
        try:
            # Clean and parse JSON
            cleaned = self.clean_response(response_text)
            parsed = json.loads(cleaned)
            
            # Validate the structure
            if not isinstance(parsed, list):
                raise ValueError("Response must be a JSON array")
            
            # Validate each action
            for action in parsed:
                if not self.validate_action(action):
                    raise ValueError(f"Invalid action: {action}")
            
            return parsed
            
        except (json.JSONDecodeError, ValueError) as e:
            print(f"Response validation error: {e}")
            # Return safe fallback
            return [{"action": "report_status", "message": "Unable to parse plan, please try again"}]
```

## 8. Prompt Engineering for Robot Tasks

### System prompt structure

The system prompt is critical for getting consistent, structured outputs from the LLM:

```python
SYSTEM_PROMPT_TEMPLATE = """
You are the task planning module for a humanoid robot. Your role is to decompose natural language commands into sequences of executable robot actions.

## Robot Capabilities
- Navigation: Move to named locations in the environment
- Perception: Detect objects of known types using vision
- Manipulation: Grasp and place objects within reach
- Communication: Report status and request clarification

## Available Actions
- navigate_to(location): Move robot to specified location
- detect_object(object_type, location=None): Locate an object in the current area or specific location
- grasp_object(object_type, location=None): Pick up a specified object
- place_object(location): Put the currently held object at the specified location
- scan_area(location): Look around to map the current area
- report_status(message): Communicate status to the user

## Response Format
Always respond with a valid JSON array of action objects:
[
  {{"action": "action_name", "parameters": {{"param1": "value1", ...}}}},
  ...
]

## Safety Constraints
- Respect human safety at all times
- Do not attempt actions beyond robot capabilities
- Plan around known environmental constraints
- Request clarification for ambiguous commands

## Example
User: "Pick up the red cup and put it on the table"
Assistant: [
  {{"action": "detect_object", "parameters": {{"object_type": "cup", "color": "red"}}}},
  {{"action": "grasp_object", "parameters": {{"object_type": "cup"}}}},
  {{"action": "navigate_to", "parameters": {{"location": "table"}}}},
  {{"action": "place_object", "parameters": {{"location": "table"}}}}
]
"""
```

### Defining available actions (navigate, grasp, place)

The action definition in the system prompt provides the LLM with a clear understanding of what the robot can do. This should match the actual action servers implemented in the robot.

### Providing context (robot capabilities, environment)

Adding environmental context helps the LLM generate more accurate plans:

```python
def create_contextual_prompt(user_command: str, robot_state: Dict, environment_map: Dict) -> str:
    return f"""
{SYSTEM_PROMPT_TEMPLATE}

## Current Robot State
- Current location: {robot_state.get('location', 'unknown')}
- Holding object: {robot_state.get('holding', 'nothing')}
- Battery level: {robot_state.get('battery', 'unknown')}%

## Environment Information  
- Available locations: {', '.join(environment_map.get('locations', []))}
- Known objects: {', '.join(environment_map.get('objects', []))}
- Recent detections: {', '.join(robot_state.get('recent_detections', []))}

## User Command
{user_command}

## Action Plan
"""
```

### Few-shot examples:

Including examples in the prompt helps the LLM understand the expected output format:

```python
FEW_SHOT_EXAMPLES = """
## Examples

Example 1:
User: "Go to the kitchen and bring me a water bottle"
[
  {"action": "navigate_to", "parameters": {"location": "kitchen"}},
  {"action": "detect_object", "parameters": {"object_type": "water bottle"}},
  {"action": "grasp_object", "parameters": {"object_type": "water bottle"}},
  {"action": "navigate_to", "parameters": {"location": "user"}},
  {"action": "place_object", "parameters": {"location": "user"}}
]

Example 2: 
User: "Clean up the table"
[
  {"action": "navigate_to", "parameters": {"location": "table"}},
  {"action": "scan_area", "parameters": {"location": "table"}},
  {"action": "detect_object", "parameters": {"object_type": "cup"}},
  {"action": "grasp_object", "parameters": {"object_type": "cup"}},
  {"action": "navigate_to", "parameters": {"location": "bin"}},
  {"action": "place_object", "parameters": {"location": "bin"}},
  {"action": "detect_object", "parameters": {"object_type": "book"}},
  {"action": "grasp_object", "parameters": {"object_type": "book"}},
  {"action": "navigate_to", "parameters": {"location": "shelf"}},
  {"action": "place_object", "parameters": {"location": "shelf"}}
]

Example 3:
User: "Find the red ball"
[
  {"action": "scan_area", "parameters": {"location": "current"}},
  {"action": "detect_object", "parameters": {"object_type": "ball", "color": "red"}}
]
"""
```

### Handling ambiguous commands

The system should handle ambiguous commands by requesting clarification:

```python
def handle_ambiguous_command(command: str) -> str:
    return f"""
User: "{command}"
[
  {{"action": "report_status", "parameters": {{"message": "I need more details about: {get_ambiguity_description(command)}"}}}},
  {{"action": "report_status", "parameters": {{"message": "Could you specify the location or describe the object more clearly?"}}}}
]
"""
```

## 9. Task Decomposition Pipeline

### Natural language → structured actions

The task decomposition pipeline transforms high-level natural language commands into sequences of primitive actions that the robot can execute:

```python
class TaskDecompositionPipeline:
    def __init__(self, llm_planner, environment_context):
        self.llm_planner = llm_planner
        self.validator = ResponseValidator()
        self.environment_context = environment_context
    
    def decompose_task(self, natural_language_command: str, robot_state: Dict) -> List[Dict]:
        # Create contextual prompt
        prompt = self.create_contextual_prompt(
            natural_language_command, 
            robot_state, 
            self.environment_context
        )
        
        # Get action plan from LLM
        response = self.llm_planner.plan_task_from_prompt(prompt)
        
        # Validate and clean the response
        validated_plan = self.validator.validate_response(response)
        
        return validated_plan
    
    def create_contextual_prompt(self, command, robot_state, env_context):
        return f"""
{SYSTEM_PROMPT_TEMPLATE}

## Current Robot State
- Location: {robot_state.get('location', 'unknown')}
- Holding: {robot_state.get('holding', 'nothing')}

## Environment Context
{env_context}

## User Command
{command}

## Action Plan (JSON Array)
"""
```

### Action primitives definition

The action primitives are the fundamental operations the robot can perform:

```python
ACTION_PRIMITIVES = {
    "navigate_to": {
        "description": "Move robot to specified location",
        "required_params": ["location"],
        "optional_params": ["speed", "avoid_humans"]
    },
    "detect_object": {
        "description": "Locate objects in environment",
        "required_params": ["object_type"],
        "optional_params": ["color", "location", "max_distance"]
    },
    "grasp_object": {
        "description": "Pick up an object",
        "required_params": ["object_type"],
        "optional_params": ["location"]
    },
    "place_object": {
        "description": "Put down object at location",
        "required_params": ["location"],
        "optional_params": ["orientation"]
    },
    "scan_area": {
        "description": "Map the surrounding area",
        "required_params": [],
        "optional_params": ["location", "radius"]
    },
    "report_status": {
        "description": "Communicate with user",
        "required_params": ["message"],
        "optional_params": ["urgency"]
    }
}
```

### JSON schema for action sequences

Using JSON schema ensures consistent output format:

```python
ACTION_SCHEMA = {
    "type": "array",
    "items": {
        "type": "object",
        "properties": {
            "action": {
                "type": "string",
                "enum": list(ACTION_PRIMITIVES.keys())
            },
            "parameters": {
                "type": "object"
            }
        },
        "required": ["action", "parameters"]
    }
}
```

### Example transformations:

```python
# Transformation examples
TRANSFORMATIONS = {
    "Pick up the red cup": [
        {"action": "detect_object", "parameters": {"object_type": "cup", "color": "red"}},
        {"action": "grasp_object", "parameters": {"object_type": "cup"}}
    ],
    
    "Go to the kitchen": [
        {"action": "navigate_to", "parameters": {"location": "kitchen"}}
    ],
    
    "Clean the room": [
        {"action": "scan_area", "parameters": {}},
        {"action": "detect_object", "parameters": {"object_type": "trash"}},
        {"action": "grasp_object", "parameters": {"object_type": "trash"}},
        {"action": "navigate_to", "parameters": {"location": "bin"}},
        {"action": "place_object", "parameters": {"location": "bin"}}
    ]
}
```

### Validation and feasibility checking

```python
class ActionFeasibilityChecker:
    def __init__(self, robot_capabilities, environment_map):
        self.robot_capabilities = robot_capabilities
        self.environment_map = environment_map
    
    def check_feasibility(self, action_sequence: List[Dict]) -> Dict[str, Any]:
        issues = []
        current_state = {
            "location": "unknown",
            "holding": None,
            "battery": 100  # percentage
        }
        
        for i, action in enumerate(action_sequence):
            action_type = action["action"]
            params = action.get("parameters", {})
            
            # Check if action is supported
            if action_type not in ACTION_PRIMITIVES:
                issues.append(f"Action {action_type} not supported")
                continue
            
            # Validate parameters
            required_params = ACTION_PRIMITIVES[action_type]["required_params"]
            for param in required_params:
                if param not in params:
                    issues.append(f"Missing required parameter '{param}' for action {action_type}")
            
            # Check feasibility based on current state
            if action_type == "grasp_object" and current_state["holding"] is not None:
                issues.append(f"Cannot grasp object when already holding {current_state['holding']}")
            
            elif action_type == "navigate_to":
                location = params.get("location")
                if location not in self.environment_map.get("locations", []):
                    issues.append(f"Unknown location: {location}")
            
            # Update current state based on action effects
            current_state = self.update_state(current_state, action)
        
        return {
            "feasible": len(issues) == 0,
            "issues": issues,
            "suggested_fixes": self.generate_fixes(issues)
        }
    
    def update_state(self, state, action):
        new_state = state.copy()
        
        if action["action"] == "grasp_object":
            new_state["holding"] = action["parameters"].get("object_type", "object")
        elif action["action"] == "place_object":
            new_state["holding"] = None
        elif action["action"] == "navigate_to":
            new_state["location"] = action["parameters"].get("location", new_state["location"])
        
        return new_state
    
    def generate_fixes(self, issues):
        fixes = []
        for issue in issues:
            # Generate potential fixes for common issues
            if "unknown location" in issue.lower():
                fixes.append("Provide available locations to the LLM or use relative positioning")
            elif "holding" in issue.lower():
                fixes.append("Add a place_object action before attempting to grasp")
        
        return fixes
```

## 10. ROS 2 Action Servers

### What are actions? (vs topics and services)

In ROS 2, actions are used for long-running tasks that provide feedback during execution. They differ from topics (continuous data streams) and services (request-response communication) in that they:

- Provide feedback during execution
- Allow for goal preemption (canceling ongoing actions)
- Return detailed results when completed
- Are appropriate for tasks like navigation, manipulation, and sensing

### Action interface definition (.action files)

Action files define the interface for communication between action clients and servers:

```
# NavigateTo.action
geometry_msgs/PoseStamped target_pose
---
geometry_msgs/PoseStamped final_pose
string message
---
geometry_msgs/PoseStamped current_pose
string message
float32 distance_remaining
```

```
# GraspObject.action
string object_name
string object_color
---
bool success
string error_message
---
float32 progress
string status
```

### Implementing action servers in Python

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.action import NavigateTo  # Replace with actual action definition
import time
import threading


class NavigateToActionServer(Node):
    def __init__(self):
        super().__init__('navigate_to_action_server')
        
        # Use a reentrant callback group to handle concurrent goals
        callback_group = ReentrantCallbackGroup()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateTo,
            'navigate_to',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group
        )
        
        self.get_logger().info("NavigateTo action server started")

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancellation request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')
        
        feedback_msg = NavigateTo.Feedback()
        feedback_msg.current_pose = None  # We'll update this during execution
        feedback_msg.distance_remaining = 0.0
        feedback_msg.status = "Navigating to target"
        
        # Simulate navigation
        for i in range(10):  # In a real implementation, this would be based on actual navigation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return NavigateTo.Result()
            
            # Publish feedback
            feedback_msg.distance_remaining = 10.0 - i  # Simulated remaining distance
            feedback_msg.status = f"Moving toward target, {10-i}m remaining"
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.5)  # Simulate real navigation time
        
        # Check if goal was successful
        goal_handle.succeed()
        result = NavigateTo.Result()
        result.success = True
        result.message = "Successfully reached destination"
        
        self.get_logger().info('Returning result: %s' % result.message)
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = NavigateToActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Feedback, result, and goal handling

- **Goal**: Initial request with parameters
- **Feedback**: Periodic updates during execution
- **Result**: Final outcome when execution completes
- **Cancellation**: Ability to stop ongoing actions

### Action client implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import NavigateTo  # Replace with actual action


class NavigateToClient(Node):
    def __init__(self):
        super().__init__('navigate_to_client')
        self._action_client = ActionClient(self, NavigateTo, 'navigate_to')

    def send_goal(self, target_pose):
        # Wait for the action server to be available
        self._action_client.wait_for_server()
        
        # Create a goal message
        goal_msg = NavigateTo.Goal()
        goal_msg.target_pose = target_pose
        
        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.status}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToClient()
    
    # Create a simple target pose (in a real system, this would come from planning)
    from geometry_msgs.msg import PoseStamped
    target_pose = PoseStamped()
    
    action_client.send_goal(target_pose)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

### Preemption and cancellation

Action servers must handle preemption requests:

```python
def execute_callback(self, goal_handle):
    """Execute the goal with preemption handling."""
    self.get_logger().info('Executing goal...')
    
    # Check for cancellation during execution
    for step in range(100):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = NavigateTo.Result()
            result.success = False
            result.message = "Goal was canceled"
            return result
            
        # Do work for this step
        # ...
        
        # Publish feedback
        feedback_msg = NavigateTo.Feedback()
        feedback_msg.progress = float(step) / 100.0
        goal_handle.publish_feedback(feedback_msg)
        
        time.sleep(0.1)  # Simulate work
    
    # If we finish without cancellation, succeed
    goal_handle.succeed()
    result = NavigateTo.Result()
    result.success = True
    result.message = "Successfully completed"
    return result
```

## 11. Building Core Action Primitives

### NavigateToAction: move to waypoint

The navigation action server interfaces with the Nav2 stack from Module 3:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose  # Nav2's built-in action
from example_interfaces.action import NavigateTo  # Our custom action


class NavigateToActionServer(Node):
    def __init__(self):
        super().__init__('navigate_to_action_server')
        
        # Create action server for our custom interface
        self._action_server = ActionServer(
            self,
            NavigateTo,
            'navigate_to',
            execute_callback=self.execute_callback
        )
        
        # Create client for Nav2's NavigateToPose action
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("NavigateTo action server initialized")

    def execute_callback(self, goal_handle):
        """Execute navigation goal."""
        self.get_logger().info(f'Navigating to location: {goal_handle.request.location}')
        
        # If location is named, convert to coordinates
        target_pose = self.get_location_pose(goal_handle.request.location)
        if not target_pose:
            goal_handle.abort()
            result = NavigateTo.Result()
            result.success = False
            result.message = f"Unknown location: {goal_handle.request.location}"
            return result
        
        # Send goal to Nav2
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = target_pose
        
        self.nav2_client.wait_for_server()
        future = self.nav2_client.send_goal_async(nav2_goal)
        
        # Wait for Nav2 to complete
        rclpy.spin_until_future_complete(self, future)
        nav2_goal_handle = future.result()
        
        if nav2_goal_handle.accepted:
            result_future = nav2_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            nav2_result = result_future.result().result
            
            goal_handle.succeed()
            result = NavigateTo.Result()
            result.success = nav2_result.result == 1  # Nav2 result codes
            result.message = "Navigation completed"
            return result
        else:
            goal_handle.abort()
            result = NavigateTo.Result()
            result.success = False
            result.message = "Nav2 rejected navigation goal"
            return result

    def get_location_pose(self, location_name: str) -> PoseStamped:
        """Convert named location to PoseStamped."""
        # In a real system, this would come from a map/waypoints service
        location_map = {
            "kitchen": self.create_pose(5.0, 2.0, 0.0),
            "table": self.create_pose(3.0, 1.0, 0.0),
            "bin": self.create_pose(1.0, 5.0, 0.0),
            "charger": self.create_pose(0.5, 0.5, 0.0),
        }
        
        return location_map.get(location_name)

    def create_pose(self, x, y, theta) -> PoseStamped:
        """Create a PoseStamped for a given position/rotation."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # Convert theta to quaternion
        from math import sin, cos
        pose.pose.orientation.z = sin(theta / 2.0)
        pose.pose.orientation.w = cos(theta / 2.0)
        return pose
```

### DetectObjectAction: find object using vision

The object detection action server uses Isaac ROS perception from Module 3:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from example_interfaces.action import DetectObject
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import threading
import time


class DetectObjectActionServer(Node):
    def __init__(self):
        super().__init__('detect_object_action_server')
        
        self._action_server = ActionServer(
            self,
            DetectObject,
            'detect_object',
            execute_callback=self.execute_callback
        )
        
        # Subscribe to Isaac ROS detection results
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/isaac_ros_detectnet/detections',
            self.detection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Store the latest detections
        self.last_detections = None
        self.detection_lock = threading.Lock()
        
        self.get_logger().info("DetectObject action server initialized")

    def detection_callback(self, msg):
        """Store the latest detections."""
        with self.detection_lock:
            self.last_detections = msg

    def execute_callback(self, goal_handle):
        """Execute object detection goal."""
        self.get_logger().info(f'Detecting object: {goal_handle.request.object}')
        
        # Wait for detections (with timeout)
        start_time = time.time()
        timeout = 10.0  # seconds
        
        while time.time() - start_time < timeout:
            with self.detection_lock:
                detections = self.last_detections
            
            if detections:
                # Look for requested object in detections
                target_objects = []
                for detection in detections.detections:
                    class_name = detection.results[0].hypothesis.name if detection.results else ""
                    
                    # Filter by requested object type
                    if goal_handle.request.object in class_name:
                        if goal_handle.request.HasField('color'):
                            # Additional color filtering would happen here
                            pass
                        target_objects.append(detection)
                
                if target_objects:
                    # Return the first detected object
                    result = DetectObject.Result()
                    result.success = True
                    result.object_found = True
                    result.object_pose = target_objects[0].bbox  # Simplified
                    result.message = f"Found {len(target_objects)} instances of {goal_handle.request.object}"
                    
                    goal_handle.succeed()
                    return result
            
            time.sleep(0.1)  # Check again
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = DetectObject.Result()
                result.success = False
                result.object_found = False
                result.message = "Detection was canceled"
                return result
        
        # If timeout reached
        goal_handle.abort()
        result = DetectObject.Result()
        result.success = False
        result.object_found = False
        result.message = f"Could not find {goal_handle.request.object} within timeout"
        return result
```

### GraspObjectAction: pick up object

Simplified grasping action for the humanoid robot:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from example_interfaces.action import GraspObject
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time


class GraspObjectActionServer(Node):
    def __init__(self):
        super().__init__('grasp_object_action_server')
        
        self._action_server = ActionServer(
            self,
            GraspObject,
            'grasp_object',
            execute_callback=self.execute_callback
        )
        
        # Publisher to control robot's gripper
        self.gripper_pub = self.create_publisher(String, '/gripper_control', 10)
        
        # Store current holding state
        self.current_object = None
        
        self.get_logger().info("GraspObject action server initialized")

    def execute_callback(self, goal_handle):
        """Execute grasping goal."""
        self.get_logger().info(f'Grasping object: {goal_handle.request.object}')
        
        # Check if already holding something
        if self.current_object is not None:
            goal_handle.abort()
            result = GraspObject.Result()
            result.success = False
            result.message = f"Cannot grasp {goal_handle.request.object}, already holding {self.current_object}"
            return result
        
        # Check if object position is known (would come from detection)
        # In a real system, this would use the object's pose from detection
        if not hasattr(goal_handle.request, 'object_pose'):
            self.get_logger().warn("Object pose not provided, using default grasp")
        
        # Publish gripper command to grasp
        grasp_cmd = String()
        grasp_cmd.data = f"grasp:{goal_handle.request.object}"
        self.gripper_pub.publish(grasp_cmd)
        
        # Simulate grasping time
        time.sleep(2.0)
        
        # Check if grasp successful (in real system, this would come from gripper feedback)
        success = True  # In real system, would check gripper sensors
        
        if success:
            self.current_object = goal_handle.request.object
            goal_handle.succeed()
            result = GraspObject.Result()
            result.success = True
            result.message = f"Successfully grasped {goal_handle.request.object}"
            return result
        else:
            goal_handle.abort()
            result = GraspObject.Result()
            result.success = False
            result.message = f"Failed to grasp {goal_handle.request.object}"
            return result
```

### PlaceObjectAction: put down object

Action for placing objects at specified locations:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from example_interfaces.action import PlaceObject
from std_msgs.msg import String
import time


class PlaceObjectActionServer(Node):
    def __init__(self):
        super().__init__('place_object_action_server')
        
        self._action_server = ActionServer(
            self,
            PlaceObject,
            'place_object',
            execute_callback=self.execute_callback
        )
        
        # Publisher to control robot's gripper
        self.gripper_pub = self.create_publisher(String, '/gripper_control', 10)
        
        # Store current holding state
        self.current_object = None
        
        self.get_logger().info("PlaceObject action server initialized")

    def execute_callback(self, goal_handle):
        """Execute placement goal."""
        self.get_logger().info(f'Placing object at location: {goal_handle.request.location}')
        
        # Check if holding an object
        if self.current_object is None:
            goal_handle.abort()
            result = PlaceObject.Result()
            result.success = False
            result.message = "Cannot place object, not holding anything"
            return result
        
        # Navigate to placement location first (would coordinate with navigation system)
        # In a real implementation, this would integrate with navigation
        time.sleep(1.0)  # Simulate navigation time
        
        # Publish gripper command to release
        release_cmd = String()
        release_cmd.data = "release"
        self.gripper_pub.publish(release_cmd)
        
        # Simulate release time
        time.sleep(1.0)
        
        # Check if release successful
        success = True  # In real system, would check gripper sensors
        
        if success:
            held_object = self.current_object
            self.current_object = None  # Clear the held object
            goal_handle.succeed()
            result = PlaceObject.Result()
            result.success = True
            result.message = f"Successfully placed {held_object} at {goal_handle.request.location}"
            return result
        else:
            goal_handle.abort()
            result = PlaceObject.Result()
            result.success = False
            result.message = f"Failed to place {self.current_object} at {goal_handle.request.location}"
            return result
```

### ScanAreaAction: look around and map

Action for scanning an area to build or update spatial knowledge:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from example_interfaces.action import ScanArea
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import time


class ScanAreaActionServer(Node):
    def __init__(self):
        super().__init__('scan_area_action_server')
        
        self._action_server = ActionServer(
            self,
            ScanArea,
            'scan_area',
            execute_callback=self.execute_callback
        )
        
        # Publisher for point cloud to build local map
        self.pc_pub = self.create_publisher(PointCloud2, '/local_map_updates', 10)
        
        # Subscriber to get current point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pc_callback,
            10
        )
        
        self.current_scan = None
        self.get_logger().info("ScanArea action server initialized")

    def pc_callback(self, msg):
        """Store the latest point cloud."""
        self.current_scan = msg

    def execute_callback(self, goal_handle):
        """Execute scanning goal."""
        self.get_logger().info(f'Scanning area: {goal_handle.request.location or "current_position"}')
        
        # In a real system, this might involve moving the head/sensors
        # to scan a wider area or integrate multiple scans
        
        # Get current scan (from callback)
        scan_data = self.current_scan
        
        if scan_data:
            # Publish to local map system (for this example, just return success)
            # In real system, this would integrate with mapping system
            
            goal_handle.succeed()
            result = ScanArea.Result()
            result.success = True
            result.message = "Area scanned successfully"
            result.map_coverage = 0.8  # 80% of area covered
            return result
        else:
            goal_handle.abort()
            result = ScanArea.Result()
            result.success = False
            result.message = "No point cloud data available for scan"
            result.map_coverage = 0.0
            return result
```

## 12. Action Server Implementation Examples

### navigate_to_server.py (using Nav2 from Module 3)

```python
#!/usr/bin/env python3
# navigate_to_server.py

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from example_interfaces.action import NavigateTo  # Our custom action

import time


class NavigateToServer(Node):
    def __init__(self):
        super().__init__('navigate_to_server')
        
        # Create our custom action server
        self._action_server = ActionServer(
            self,
            NavigateTo,
            'navigate_to',
            execute_callback=self.execute_callback
        )
        
        # Client for Nav2's NavigateToPose action
        self.nav2_action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # Predefined locations based on environment map
        self.location_map = {
            'kitchen': self.create_pose(5.0, 2.0, 0.0),
            'table': self.create_pose(3.0, 1.0, 0.0), 
            'bin': self.create_pose(1.0, 5.0, 0.0),
            'charger': self.create_pose(0.5, 0.5, 0.0),
            'sofa': self.create_pose(6.0, 4.0, 1.57)
        }
        
        self.get_logger().info("NavigateTo server initialized")

    def create_pose(self, x, y, theta):
        """Helper to create PoseStamped for given coordinates."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta (rotation) to quaternion
        from math import sin, cos
        cy = cos(theta * 0.5)
        sy = sin(theta * 0.5)
        pose.pose.orientation.z = sy
        pose.pose.orientation.w = cy
        
        return pose

    def execute_callback(self, goal_handle):
        """Execute navigation goal."""
        self.get_logger().info(f'Request to navigate to: {goal_handle.request.location}')
        
        # Look up location in our map
        if goal_handle.request.location not in self.location_map:
            self.get_logger().error(f'Unknown location: {goal_handle.request.location}')
            goal_handle.abort()
            
            result = NavigateTo.Result()
            result.success = False
            result.message = f'Unknown location: {goal_handle.request.location}'
            return result
        
        target_pose = self.location_map[goal_handle.request.location]
        
        # Wait for Nav2 action server to be available
        self.nav2_action_client.wait_for_server()
        
        # Create Nav2 goal
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = target_pose
        
        # Send goal to Nav2
        future = self.nav2_action_client.send_goal_async(nav2_goal)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        nav2_goal_handle = future.result()
        
        if not nav2_goal_handle.accepted:
            self.get_logger().error('Nav2 rejected navigation goal')
            goal_handle.abort()
            
            result = NavigateTo.Result()
            result.success = False
            result.message = 'Navigation goal rejected by Nav2'
            return result
        
        # Wait for result
        result_future = nav2_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        # Process Nav2 result
        nav2_result = result_future.result().result
        
        if nav2_result:
            self.get_logger().info(f'Successfully navigated to {goal_handle.request.location}')
            goal_handle.succeed()
            
            result = NavigateTo.Result()
            result.success = True
            result.message = f'Successfully reached {goal_handle.request.location}'
            return result
        else:
            self.get_logger().error(f'Failed to navigate to {goal_handle.request.location}')
            goal_handle.abort()
            
            result = NavigateTo.Result()
            result.success = False
            result.message = f'Failed to reach {goal_handle.request.location}'
            return result


def main(args=None):
    rclpy.init(args=args)
    
    server = NavigateToServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Server shutting down gracefully')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### detect_object_server.py (using Isaac ROS from Module 3)

```python
#!/usr/bin/env python3
# detect_object_server.py

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from example_interfaces.action import DetectObject
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from geometry_msgs.msg import Point

import threading
import time


class DetectObjectServer(Node):
    def __init__(self):
        super().__init__('detect_object_server')
        
        # Custom action server
        self._action_server = ActionServer(
            self,
            DetectObject,
            'detect_object',
            execute_callback=self.execute_callback
        )
        
        # Subscribe to Isaac ROS detection output
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/isaac_ros_detectnet/detections',  # Isaac ROS detection topic
            self.detection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Store last detections with thread-safe access
        self.last_detections = None
        self.detection_lock = threading.Lock()
        
        self.get_logger().info("DetectObject server initialized")

    def detection_callback(self, msg):
        """Store the latest detections."""
        with self.detection_lock:
            self.last_detections = msg
        self.get_logger().debug(f"Received detection with {len(msg.detections)} objects")

    def execute_callback(self, goal_handle):
        """Execute detection goal."""
        self.get_logger().info(f'Request to detect: {goal_handle.request.object_type}')
        
        # If color specified, include in search
        target_color = goal_handle.request.color if goal_handle.request.color else None
        self.get_logger().info(f'Searching for {target_color} {goal_handle.request.object_type}' if target_color else 
                              f'Searching for {goal_handle.request.object_type}')
        
        # Wait for detections with timeout
        timeout = 10.0  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Get current detections
            with self.detection_lock:
                detections = self.last_detections
            
            if detections:
                # Look for the target object in the detections
                target_detections = []
                
                for detection in detections.detections:
                    if not detection.results:
                        continue  # Skip if no results
                        
                    # Get the most likely classification
                    top_result = detection.results[0]
                    
                    # Check if this matches our target object type
                    if goal_handle.request.object_type.lower() in top_result.hypothesis.name.lower():
                        # If color specified, we'd need color information from the detection
                        # For now, just add to potential matches
                        target_detections.append(detection)
                
                if target_detections:
                    # Found the object - return the first detection
                    found_detection = target_detections[0]
                    
                    self.get_logger().info(f'Found {goal_handle.request.object_type} successfully')
                    
                    goal_handle.succeed()
                    result = DetectObject.Result()
                    result.success = True
                    result.object_found = True
                    result.object_name = goal_handle.request.object_type
                    result.object_pose = self.get_pose_from_detection(found_detection)
                    result.message = f'Found {len(target_detections)} instances of {goal_handle.request.object_type}'
                    
                    return result
            
            # Check if goal was canceled during waiting
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = DetectObject.Result()
                result.success = False
                result.object_found = False
                result.message = "Detection canceled by user"
                return result
            
            # Small delay to prevent busy waiting
            time.sleep(0.1)
        
        # If we get here, we timed out
        self.get_logger().info(f'Timed out looking for {goal_handle.request.object_type}')
        
        goal_handle.abort()
        result = DetectObject.Result()
        result.success = False
        result.object_found = False
        result.message = f'Could not find {goal_handle.request.object_type} in {timeout} seconds'
        return result

    def get_pose_from_detection(self, detection):
        """Extract position information from detection (simplified)."""
        # In a real implementation, this would convert 2D detection to 3D pose
        # using depth information from the camera
        pose = Point()
        pose.x = detection.bbox.center.position.x  # Simplified
        pose.y = detection.bbox.center.position.y
        pose.z = 0.0  # Would need depth info for actual Z
        return pose


def main(args=None):
    rclpy.init(args=args)
    
    server = DetectObjectServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Server shutting down gracefully')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### grasp_object_server.py (basic manipulation)

```python
#!/usr/bin/env python3
# grasp_object_server.py

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from example_interfaces.action import GraspObject
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time


class GraspObjectServer(Node):
    def __init__(self):
        super().__init__('grasp_object_server')
        
        # Action server
        self._action_server = ActionServer(
            self,
            GraspObject,
            'grasp_object',
            execute_callback=self.execute_callback
        )
        
        # Publisher to control gripper
        self.gripper_publisher = self.create_publisher(String, '/gripper_command', 10)
        
        # Subscriber to get gripper state
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Track current state
        self.gripper_position = 0.0  # normalized [0,1] where 0=open, 1=closed
        self.currently_holding = None
        
        self.get_logger().info("GraspObject server initialized")

    def joint_state_callback(self, msg):
        """Update gripper state from joint states."""
        try:
            # Find gripper joint in the message
            gripper_idx = msg.name.index('gripper_joint')  # or whatever the actual joint name is
            self.gripper_position = msg.position[gripper_idx]
        except ValueError:
            # Gripper joint not found in this message
            pass

    def execute_callback(self, goal_handle):
        """Execute grasping goal."""
        self.get_logger().info(f'Request to grasp: {goal_handle.request.object_type}')
        
        # Check if already holding something
        if self.currently_holding is not None:
            self.get_logger().error(f'Cannot grasp {goal_handle.request.object_type}, already holding {self.currently_holding}')
            goal_handle.abort()
            
            result = GraspObject.Result()
            result.success = False
            result.message = f'Cannot grasp {goal_handle.request.object_type}, already holding {self.currently_holding}'
            return result
        
        # Check if object is in reach (would use pose from detection)
        # For now, assume it's in reach
        
        # Open gripper first (if not already open)
        if self.gripper_position < 0.9:  # If not already open (with tolerance)
            open_cmd = String()
            open_cmd.data = 'open'
            self.gripper_publisher.publish(open_cmd)
            time.sleep(0.5)  # Time to open gripper
        
        # Move to object position (would require coordination with navigation)
        # For now, assume we're already positioned
        
        # Close gripper to grasp
        close_cmd = String()
        close_cmd.data = 'close'
        self.gripper_publisher.publish(close_cmd)
        
        # Wait for gripper to close
        time.sleep(1.0)
        
        # Check if grasp was successful (in a real system, this would check force sensors,
        # object detection to see if object is still there, etc.)
        grasp_successful = self.verify_grasp(goal_handle.request.object_type)
        
        if grasp_successful:
            self.currently_holding = goal_handle.request.object_type
            self.get_logger().info(f'Successfully grasped {goal_handle.request.object_type}')
            
            goal_handle.succeed()
            result = GraspObject.Result()
            result.success = True
            result.message = f'Successfully grasped {goal_handle.request.object_type}'
            return result
        else:
            self.get_logger().error(f'Failed to grasp {goal_handle.request.object_type}')
            
            # Try to recover by opening gripper
            open_cmd = String()
            open_cmd.data = 'open'
            self.gripper_publisher.publish(open_cmd)
            
            goal_handle.abort()
            result = GraspObject.Result()
            result.success = False
            result.message = f'Failed to grasp {goal_handle.request.object_type}'
            return result

    def verify_grasp(self, object_type):
        """Verify that the object was successfully grasped."""
        # In a real system, this would check:
        # - Force/torque sensors in gripper
        # - Object detection to see if object is no longer in the environment
        # - Joint position sensors to confirm grip
        
        # For simulation, just return success
        return True


def main(args=None):
    rclpy.init(args=args)
    
    server = GraspObjectServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Server shutting down gracefully')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Complete code with error handling

```python
#!/usr/bin/env python3
# vla_action_servers.py - Combined VLA action servers

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from example_interfaces.action import NavigateTo, DetectObject, GraspObject, PlaceObject, ScanArea
from nav2_msgs.action import NavigateToPose
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped

import threading
import time


class VLACoreActionServers(Node):
    def __init__(self):
        super().__init__('vla_core_action_servers')
        
        # Initialize all action servers
        self._navigate_to_server = ActionServer(
            self, NavigateTo, 'navigate_to', self.navigate_to_callback
        )
        self._detect_object_server = ActionServer(
            self, DetectObject, 'detect_object', self.detect_object_callback
        )
        self._grasp_object_server = ActionServer(
            self, GraspObject, 'grasp_object', self.grasp_object_callback
        )
        self._place_object_server = ActionServer(
            self, PlaceObject, 'place_object', self.place_object_callback
        )
        self._scan_area_server = ActionServer(
            self, ScanArea, 'scan_area', self.scan_area_callback
        )
        
        # Initialize Nav2 client for navigation
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initialize Isaac ROS detection subscription
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/isaac_ros_detectnet/detections',
            self.detection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Initialize gripper control
        self.gripper_publisher = self.create_publisher(String, '/gripper_command', 10)
        
        # Storage for robot state
        self.last_detections = None
        self.detection_lock = threading.Lock()
        self.gripper_position = 0.0
        self.currently_holding = None
        self.current_location = "unknown"
        
        # Predefined locations map
        self.location_map = {
            'kitchen': self.create_pose(5.0, 2.0, 0.0),
            'table': self.create_pose(3.0, 1.0, 0.0),
            'bin': self.create_pose(1.0, 5.0, 0.0),
            'charger': self.create_pose(0.5, 0.5, 0.0)
        }
        
        self.get_logger().info("VLA Core Action Servers initialized")

    def create_pose(self, x, y, theta):
        """Helper to create PoseStamped for given coordinates."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        from math import sin, cos
        cy = cos(theta * 0.5)
        sy = sin(theta * 0.5)
        pose.pose.orientation.z = sy
        pose.pose.orientation.w = cy
        
        return pose

    def detection_callback(self, msg):
        """Store the latest detections."""
        with self.detection_lock:
            self.last_detections = msg

    def navigate_to_callback(self, goal_handle):
        """Execute navigation goal."""
        self.get_logger().info(f'Navigating to: {goal_handle.request.location}')
        
        if goal_handle.request.location not in self.location_map:
            goal_handle.abort()
            result = NavigateTo.Result()
            result.success = False
            result.message = f'Unknown location: {goal_handle.request.location}'
            return result

        target_pose = self.location_map[goal_handle.request.location]
        
        # Wait for Nav2 server
        self.nav2_client.wait_for_server()
        
        # Create and send Nav2 goal
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = target_pose
        
        future = self.nav2_client.send_goal_async(nav2_goal)
        rclpy.spin_until_future_complete(self, future)
        
        nav2_goal_handle = future.result()
        if not nav2_goal_handle.accepted:
            goal_handle.abort()
            result = NavigateTo.Result()
            result.success = False
            result.message = 'Navigation goal rejected by Nav2'
            return result
        
        # Wait for result
        result_future = nav2_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        nav2_result = result_future.result().result
        if nav2_result:
            self.current_location = goal_handle.request.location
            goal_handle.succeed()
            result = NavigateTo.Result()
            result.success = True
            result.message = f'Reached {goal_handle.request.location}'
            return result
        else:
            goal_handle.abort()
            result = NavigateTo.Result()
            result.success = False
            result.message = f'Failed to reach {goal_handle.request.location}'
            return result

    def detect_object_callback(self, goal_handle):
        """Execute object detection goal."""
        self.get_logger().info(f'Detecting: {goal_handle.request.object_type}')
        
        # Wait for detections with timeout
        timeout = 10.0
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self.detection_lock:
                detections = self.last_detections
            
            if detections:
                for detection in detections.detections:
                    if not detection.results:
                        continue
                    top_result = detection.results[0]
                    
                    if goal_handle.request.object_type.lower() in top_result.hypothesis.name.lower():
                        goal_handle.succeed()
                        result = DetectObject.Result()
                        result.success = True
                        result.object_found = True
                        result.object_name = goal_handle.request.object_type
                        result.message = f'Found {goal_handle.request.object_type}'
                        return result
            
            time.sleep(0.1)
        
        goal_handle.abort()
        result = DetectObject.Result()
        result.success = False
        result.object_found = False
        result.message = f'Could not find {goal_handle.request.object_type}'
        return result

    def grasp_object_callback(self, goal_handle):
        """Execute grasping goal."""
        self.get_logger().info(f'Grasping: {goal_handle.request.object_type}')
        
        if self.currently_holding is not None:
            goal_handle.abort()
            result = GraspObject.Result()
            result.success = False
            result.message = f'Already holding {self.currently_holding}'
            return result

        # Open gripper
        open_cmd = String()
        open_cmd.data = 'open'
        self.gripper_publisher.publish(open_cmd)
        time.sleep(0.5)

        # Close gripper to grasp
        close_cmd = String()
        close_cmd.data = 'close'
        self.gripper_publisher.publish(close_cmd)
        time.sleep(1.0)

        # Verify successful grasp
        if self.verify_grasp():
            self.currently_holding = goal_handle.request.object_type
            goal_handle.succeed()
            result = GraspObject.Result()
            result.success = True
            result.message = f'Grasped {goal_handle.request.object_type}'
            return result
        else:
            goal_handle.abort()
            result = GraspObject.Result()
            result.success = False
            result.message = f'Failed to grasp {goal_handle.request.object_type}'
            return result

    def place_object_callback(self, goal_handle):
        """Execute object placement goal."""
        self.get_logger().info(f'Placing at: {goal_handle.request.location}')
        
        if self.currently_holding is None:
            goal_handle.abort()
            result = PlaceObject.Result()
            result.success = False
            result.message = 'Not holding any object'
            return result

        # Release object
        release_cmd = String()
        release_cmd.data = 'open'  # Open gripper to release
        self.gripper_publisher.publish(release_cmd)
        time.sleep(1.0)

        held_object = self.currently_holding
        self.currently_holding = None

        goal_handle.succeed()
        result = PlaceObject.Result()
        result.success = True
        result.message = f'Placed {held_object} at {goal_handle.request.location}'
        return result

    def scan_area_callback(self, goal_handle):
        """Execute area scanning goal."""
        self.get_logger().info(f'Scanning area: {goal_handle.request.location or "current"}')
        
        # In a real implementation, this would move sensors to scan area
        # For now, just return success
        goal_handle.succeed()
        result = ScanArea.Result()
        result.success = True
        result.message = 'Area scanned successfully'
        return result

    def verify_grasp(self):
        """Verify that an object is currently grasped."""
        # In a real system, this would check force sensors, joint positions, etc.
        # For simulation, return True
        return True


def main(args=None):
    rclpy.init(args=args)
    
    servers = VLACoreActionServers()
    
    try:
        rclpy.spin(servers)
    except KeyboardInterrupt:
        servers.get_logger().info('Shutting down VLA action servers')
    finally:
        servers.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 13. Perception Integration

### Connecting vision to action execution

The perception system provides the environmental understanding that enables the action execution system. This connection is critical for any autonomous robot system:

1. **Object Detection**: Identify what objects are present in the environment
2. **Localization**: Understand where objects are relative to the robot
3. **State Monitoring**: Track changes in the environment during action execution

```python
class PerceptionActionBridge:
    def __init__(self, node):
        self.node = node
        self.detected_objects = {}
        self.object_poses = {}
        self.action_context = {}
        
        # Subscribe to perception outputs
        self.detection_subscription = node.create_subscription(
            Detection2DArray,
            '/isaac_ros_detectnet/detections',
            self.detection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.pc_subscription = node.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
    
    def detection_callback(self, msg):
        """Process object detections and update internal state."""
        for detection in msg.detections:
            if detection.results:
                class_name = detection.results[0].hypothesis.name
                confidence = detection.results[0].hypothesis.score
                
                # Only store high-confidence detections
                if confidence > 0.7:
                    # Update our knowledge of objects in the environment
                    self.detected_objects[class_name] = {
                        'confidence': confidence,
                        'bbox': detection.bbox,
                        'timestamp': self.node.get_clock().now()
                    }
    
    def pointcloud_callback(self, msg):
        """Process point cloud for 3D object localization."""
        # Process point cloud to get 3D positions of objects
        # This would typically involve clustering points by 2D detection boundaries
        pass
    
    def get_object_pose(self, object_type):
        """Get the 3D pose of an object if it exists in the environment."""
        if object_type in self.detected_objects:
            # In a complete implementation, this would return the 3D pose
            # based on the combination of 2D detection and depth information
            return self.object_poses.get(object_type, None)
        return None
    
    def update_action_context(self, action_request):
        """Update context used by actions based on current perception."""
        # For detection action, provide context about what was recently seen
        if action_request.action == "detect_object":
            object_type = action_request.parameters.get("object_type")
            if object_type in self.detected_objects:
                # Add context about recent detections to help the action
                self.action_context["recent_detection"] = self.detected_objects[object_type]
```

### Object detection for "find the cup"

When the LLM generates a `detect_object` action, the perception system needs to locate the specified object:

```python
def execute_detect_object(self, goal_handle):
    """Execute object detection with perception integration."""
    object_type = goal_handle.request.object_type
    location_constraint = goal_handle.request.location if goal_handle.request.location else None
    
    # Use perception system to look for the object
    detected_object = self.find_object_in_environment(object_type, location_constraint)
    
    if detected_object:
        goal_handle.succeed()
        result = DetectObject.Result()
        result.success = True
        result.object_found = True
        result.object_name = object_type
        result.object_pose = detected_object.pose
        result.message = f"Found {object_type} at {detected_object.pose}"
        return result
    else:
        goal_handle.abort()
        result = DetectObject.Result()
        result.success = False
        result.object_found = False
        result.message = f"Could not find {object_type} in environment"
        return result

def find_object_in_environment(self, object_type, location=None):
    """Find an object in the environment using perception data."""
    # Look through recent detections
    for detection in self.get_recent_detections():
        if object_type.lower() in detection.class_name.lower():
            # Apply location constraints if specified
            if location:
                if self.is_object_in_location(detection, location):
                    return detection
            else:
                # If no location specified, return the first match
                return detection
    
    return None
```

### Depth estimation for "how far is the table?"

The perception system provides depth information that enables spatial reasoning:

```python
def get_distance_to_object(self, object_name):
    """Estimate distance to named object using depth information."""
    # Find the object in our map of known locations
    if object_name in self.known_locations:
        location_pose = self.known_locations[object_name]
        robot_pose = self.get_robot_pose()
        
        # Calculate Euclidean distance
        distance = self.calculate_3d_distance(robot_pose, location_pose)
        return distance
    
    # If not in known locations, try to find via detection
    for detection in self.get_recent_detections():
        if object_name.lower() in detection.class_name.lower():
            # Use depth from point cloud at detection location
            depth = self.get_depth_at_detection(detection)
            return depth
    
    return None

def calculate_3d_distance(self, pose1, pose2):
    """Calculate 3D Euclidean distance between two poses."""
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    dz = pose1.position.z - pose2.position.z
    return (dx*dx + dy*dy + dz*dz) ** 0.5
```

### Visual grounding: language ↔ image regions

Visual grounding connects linguistic references to image regions:

```python
class VisualGrounding:
    def __init__(self, node):
        self.node = node
        # In a real implementation, this would connect to a visual grounding model
        # that can link text descriptions to image regions
    
    def ground_reference(self, language_ref, image):
        """Ground a language reference to an image region."""
        # Example: "the red cup to your left"
        # Would return bounding box/coordinates of the referenced object
        pass
    
    def resolve_pronouns(self, pronoun, context):
        """Resolve pronouns in commands to specific objects."""
        # Example: "pick it up" - what does "it" refer to?
        # Based on context of what was detected/mentioned previously
        pass
```

### Using Isaac ROS perception nodes

The perception system integrates Isaac ROS nodes that provide various AI-powered perception capabilities:

```python
class IsaacPerceptionManager:
    def __init__(self, node):
        self.node = node
        
        # Isaac ROS provides various perception nodes
        # Subscribe to their outputs
        self.detection_sub = node.create_subscription(
            Detection2DArray,
            '/isaac_ros_detectnet/detections',  # Object detection
            self.detection_callback,
            10
        )
        
        self.segmentation_sub = node.create_subscription(
            Image,  # Or SemanticSegmentationImage
            '/isaac_ros_segmentation/segmentation',
            self.segmentation_callback,
            10
        )
        
        self.depth_sub = node.create_subscription(
            Image,  # Depth image
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        self.pointcloud_sub = node.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )
    
    def detection_callback(self, msg):
        """Handle Isaac ROS detection outputs."""
        # Process detections and update world model
        for detection in msg.detections:
            if detection.results:
                class_name = detection.results[0].hypothesis.name
                confidence = detection.results[0].hypothesis.score
                bbox = detection.bbox
                
                # Update world model with detected object
                self.update_world_object(class_name, bbox, confidence)
    
    def segmentation_callback(self, msg):
        """Handle semantic segmentation outputs."""
        # Process segmentation mask to identify object regions
        pass
    
    def depth_callback(self, msg):
        """Handle depth image for 3D positioning."""
        # Process depth to get 3D positions of objects
        pass
    
    def pointcloud_callback(self, msg):
        """Handle point cloud for 3D scene understanding."""
        # Process point cloud for detailed 3D understanding
        pass
    
    def update_world_object(self, class_name, bbox, confidence):
        """Update the world model with a detected object."""
        if confidence > 0.7:  # Confidence threshold
            self.node.get_logger().info(f"Detected {class_name} with confidence {confidence}")
            # Update internal representation of objects in environment
```

### Real-time object tracking

For smooth action execution, objects need to be tracked as they move or as the robot moves:

```python
class ObjectTracker:
    def __init__(self):
        self.tracked_objects = {}  # dict of object_id -> tracking_info
        self.next_id = 0
    
    def update_detections(self, detections, timestamp):
        """Update object tracking with new detections."""
        for detection in detections:
            matched = False
            for obj_id, track_info in self.tracked_objects.items():
                if self.is_same_object(detection, track_info):
                    # Update existing track
                    track_info['bbox'] = detection.bbox
                    track_info['last_seen'] = timestamp
                    track_info['history'].append(detection.bbox)
                    matched = True
                    break
            
            if not matched:
                # Create new track
                new_id = self.next_id
                self.next_id += 1
                self.tracked_objects[new_id] = {
                    'class': detection.results[0].hypothesis.name if detection.results else 'unknown',
                    'bbox': detection.bbox,
                    'first_seen': timestamp,
                    'last_seen': timestamp,
                    'history': [detection.bbox]
                }
    
    def predict_position(self, obj_id, future_time):
        """Predict where an object will be at a future time."""
        if obj_id in self.tracked_objects:
            track = self.tracked_objects[obj_id]
            # Use history to predict future position (simple linear extrapolation)
            # In practice, this would use more sophisticated tracking algorithms
            pass
    
    def is_same_object(self, detection, track_info):
        """Determine if a detection corresponds to an existing tracked object."""
        # Compare position, size, class, etc. to determine if it's the same object
        # This uses spatial overlap and feature similarity
        pass
```

## 14. The VLA Control Loop

### Main orchestration node: vla_controller.py

The VLA controller orchestrates the entire system, receiving voice commands and executing action sequences:

```python
#!/usr/bin/env python3
# vla_controller.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from example_interfaces.action import NavigateTo, DetectObject, GraspObject, PlaceObject, ScanArea
from geometry_msgs.msg import Point

import threading
import time
import json


class VLAController(Node):
    def __init__(self):
        super().__init__('vla_controller')
        
        # Subscriber for voice commands
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )
        
        # Action clients for all core actions
        self.navigate_client = ActionClient(self, NavigateTo, 'navigate_to')
        self.detect_client = ActionClient(self, DetectObject, 'detect_object') 
        self.grasp_client = ActionClient(self, GraspObject, 'grasp_object')
        self.place_client = ActionClient(self, PlaceObject, 'place_object')
        self.scan_client = ActionClient(self, ScanArea, 'scan_area')
        
        # Publisher for status reporting
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        
        # Internal state tracking
        self.current_plan = []
        self.current_action_index = 0
        self.is_executing = False
        self.held_object = None
        self.current_location = "unknown"
        
        # LLM integration
        self.llm_planner = LLMPlanner()  # Defined in previous sections
        
        self.get_logger().info("VLA Controller initialized")

    def voice_command_callback(self, msg):
        """Handle incoming voice commands."""
        command = msg.data
        self.get_logger().info(f"Received voice command: {command}")
        
        # Publish initial status
        status_msg = String()
        status_msg.data = f"Processing command: {command}"
        self.status_publisher.publish(status_msg)
        
        # Plan the task using LLM
        try:
            action_sequence = self.llm_planner.plan_task(command)
            self.get_logger().info(f"Generated action sequence: {action_sequence}")
            
            # Execute the plan
            self.execute_plan(action_sequence)
            
        except Exception as e:
            self.get_logger().error(f"Error planning task: {e}")
            error_msg = String()
            error_msg.data = f"Error processing command: {str(e)}"
            self.status_publisher.publish(error_msg)

    def execute_plan(self, action_sequence):
        """Execute a sequence of actions."""
        if self.is_executing:
            self.get_logger().warn("Already executing a plan, rejecting new command")
            return
        
        self.is_executing = True
        self.current_plan = action_sequence
        self.current_action_index = 0
        
        try:
            for i, action in enumerate(action_sequence):
                self.get_logger().info(f"Executing action {i+1}/{len(action_sequence)}: {action}")
                
                # Update status
                status_msg = String()
                status_msg.data = f"Executing: {action['action']} - Step {i+1}/{len(action_sequence)}"
                self.status_publisher.publish(status_msg)
                
                # Execute the action
                success = self.execute_single_action(action)
                
                if not success:
                    self.get_logger().error(f"Action failed: {action}")
                    # Try to recover or replan
                    self.handle_action_failure(action, action_sequence, i)
                    break
                
                # Allow other processes to run
                time.sleep(0.1)
            
            # Plan completed
            self.get_logger().info("Plan execution completed")
            completion_msg = String()
            completion_msg.data = "Task completed successfully"
            self.status_publisher.publish(completion_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error during plan execution: {e}")
            error_msg = String()
            error_msg.data = f"Plan execution failed: {str(e)}"
            self.status_publisher.publish(error_msg)
        
        finally:
            self.is_executing = False
            self.current_plan = []
            self.current_action_index = 0

    def execute_single_action(self, action):
        """Execute a single action from the plan."""
        action_type = action['action']
        params = action.get('parameters', {})
        
        try:
            if action_type == 'navigate_to':
                return self.execute_navigate_to(params)
            elif action_type == 'detect_object':
                return self.execute_detect_object(params)
            elif action_type == 'grasp_object':
                return self.execute_grasp_object(params)
            elif action_type == 'place_object':
                return self.execute_place_object(params)
            elif action_type == 'scan_area':
                return self.execute_scan_area(params)
            elif action_type == 'report_status':
                return self.execute_report_status(params)
            else:
                self.get_logger().error(f"Unknown action type: {action_type}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error executing {action_type}: {e}")
            return False

    def execute_navigate_to(self, params):
        """Execute navigation action."""
        if 'location' in params:
            goal = NavigateTo.Goal()
            goal.location = params['location']
            
            self.navigate_client.wait_for_server()
            future = self.navigate_client.send_goal_async(goal)
            
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                
                result = result_future.result().result
                if result.success:
                    self.current_location = params['location']
                    self.get_logger().info(f"Navigated to {params['location']}")
                    return True
                else:
                    self.get_logger().error(f"Navigation failed: {result.message}")
                    return False
            else:
                self.get_logger().error("Navigation goal was rejected")
                return False
        else:
            self.get_logger().error("Navigate action missing location parameter")
            return False

    def execute_detect_object(self, params):
        """Execute object detection action."""
        if 'object_type' in params:
            goal = DetectObject.Goal()
            goal.object_type = params['object_type']
            goal.color = params.get('color', '')  # Color is optional
            
            self.detect_client.wait_for_server()
            future = self.detect_client.send_goal_async(goal)
            
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                
                result = result_future.result().result
                if result.success and result.object_found:
                    self.get_logger().info(f"Detected {params['object_type']}")
                    return True
                else:
                    self.get_logger().error(f"Detection failed: {result.message}")
                    return False
            else:
                self.get_logger().error("Detection goal was rejected")
                return False
        else:
            self.get_logger().error("Detect action missing object_type parameter")
            return False

    def execute_grasp_object(self, params):
        """Execute grasping action."""
        if 'object_type' in params:
            goal = GraspObject.Goal()
            goal.object_type = params['object_type']
            
            self.grasp_client.wait_for_server()
            future = self.grasp_client.send_goal_async(goal)
            
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                
                result = result_future.result().result
                if result.success:
                    self.held_object = params['object_type']
                    self.get_logger().info(f"Grasped {params['object_type']}")
                    return True
                else:
                    self.get_logger().error(f"Grasping failed: {result.message}")
                    return False
            else:
                self.get_logger().error("Grasp goal was rejected")
                return False
        else:
            self.get_logger().error("Grasp action missing object_type parameter")
            return False

    def execute_place_object(self, params):
        """Execute object placement action."""
        if 'location' in params:
            goal = PlaceObject.Goal()
            goal.location = params['location']
            
            self.place_client.wait_for_server()
            future = self.place_client.send_goal_async(goal)
            
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                
                result = result_future.result().result
                if result.success:
                    self.get_logger().info(f"Placed object at {params['location']}")
                    self.held_object = None  # Clear held object
                    return True
                else:
                    self.get_logger().error(f"Placement failed: {result.message}")
                    return False
            else:
                self.get_logger().error("Place goal was rejected")
                return False
        else:
            self.get_logger().error("Place action missing location parameter")
            return False

    def execute_scan_area(self, params):
        """Execute area scanning action."""
        goal = ScanArea.Goal()
        goal.location = params.get('location', self.current_location)  # Default to current location
        
        self.scan_client.wait_for_server()
        future = self.scan_client.send_goal_async(goal)
        
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result().result
            if result.success:
                self.get_logger().info("Area scanned successfully")
                return True
            else:
                self.get_logger().error(f"Scanning failed: {result.message}")
                return False
        else:
            self.get_logger().error("Scan goal was rejected")
            return False

    def execute_report_status(self, params):
        """Execute status reporting action."""
        if 'message' in params:
            status_msg = String()
            status_msg.data = params['message']
            self.status_publisher.publish(status_msg)
            self.get_logger().info(f"Reported status: {params['message']}")
            return True
        else:
            self.get_logger().error("Report status action missing message parameter")
            return False

    def handle_action_failure(self, failed_action, full_plan, failed_index):
        """Handle failure of an action in the plan."""
        self.get_logger().error(f"Action failed: {failed_action}")
        
        # Determine if we should attempt recovery or replan
        if self.can_retry_action(failed_action):
            # Simple retry once
            success = self.execute_single_action(failed_action)
            if success:
                self.get_logger().info("Action recovery successful")
                # Continue with the rest of the plan
                for i in range(failed_index + 1, len(full_plan)):
                    action = full_plan[i]
                    success = self.execute_single_action(action)
                    if not success:
                        break
        else:
            # Request replanning from LLM with failure context
            status_msg = String()
            status_msg.data = f"Action failed, requesting replan: {failed_action}"
            self.status_publisher.publish(status_msg)
            
            # In a complete implementation, we would send context about the failure
            # to the LLM planner and generate a new plan

    def can_retry_action(self, action):
        """Determine if an action can be retried."""
        # Some actions can be safely retried, others might require different approach
        non_retryable = ['grasp_object']  # Grasping might fail for structural reasons
        return action['action'] not in non_retryable


def main(args=None):
    rclpy.init(args=args)
    
    controller = VLAController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('VLA Controller shutting down')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Listen for voice commands

The controller listens to the `/voice_command` topic published by the voice interface:

```python
def voice_command_callback(self, msg):
    """Handle incoming voice commands."""
    command = msg.data
    self.get_logger().info(f"Received voice command: {command}")
    
    # Publish initial status
    status_msg = String()
    status_msg.data = f"Processing command: {command}"
    self.status_publisher.publish(status_msg)
    
    # Plan the task using LLM
    try:
        action_sequence = self.llm_planner.plan_task(command)
        self.get_logger().info(f"Generated action sequence: {action_sequence}")
        
        # Execute the plan
        self.execute_plan(action_sequence)
        
    except Exception as e:
        self.get_logger().error(f"Error planning task: {e}")
        error_msg = String()
        error_msg.data = f"Error processing command: {str(e)}"
        self.status_publisher.publish(error_msg)
```

### Send command to LLM planner

The controller sends commands to the LLM planner to decompose high-level tasks:

```python
def plan_task_with_llm(self, command):
    """Plan a task using the LLM."""
    # Include context about robot state and environment
    robot_context = {
        "location": self.current_location,
        "holding": self.held_object,
        "detected_objects": self.get_recent_detections()
    }
    
    # Generate prompt with context
    prompt = self.create_planning_prompt(command, robot_context)
    
    # Get action sequence from LLM
    action_sequence = self.llm_planner.generate_actions(prompt)
    
    return action_sequence
```

### Parse action sequence

The controller parses the action sequence from the LLM and executes it:

```python
def execute_plan(self, action_sequence):
    """Execute a sequence of actions."""
    if self.is_executing:
        self.get_logger().warn("Already executing a plan, rejecting new command")
        return
    
    self.is_executing = True
    self.current_plan = action_sequence
    self.current_action_index = 0
    
    try:
        for i, action in enumerate(action_sequence):
            self.get_logger().info(f"Executing action {i+1}/{len(action_sequence)}: {action}")
            
            # Update status
            status_msg = String()
            status_msg.data = f"Executing: {action['action']} - Step {i+1}/{len(action_sequence)}"
            self.status_publisher.publish(status_msg)
            
            # Execute the action
            success = self.execute_single_action(action)
            
            if not success:
                self.get_logger().error(f"Action failed: {action}")
                # Try to recover or replan
                self.handle_action_failure(action, action_sequence, i)
                break
            
            # Allow other processes to run
            time.sleep(0.1)
        
        # Plan completed
        self.get_logger().info("Plan execution completed")
        completion_msg = String()
        completion_msg.data = "Task completed successfully"
        self.status_publisher.publish(completion_msg)
        
    except Exception as e:
        self.get_logger().error(f"Error during plan execution: {e}")
        error_msg = String()
        error_msg.data = f"Plan execution failed: {str(e)}"
        self.status_publisher.publish(error_msg)
    
    finally:
        self.is_executing = False
        self.current_plan = []
        self.current_action_index = 0
```

### Execute actions via action clients

The controller uses action clients to execute each primitive action:

```python
def execute_single_action(self, action):
    """Execute a single action from the plan."""
    action_type = action['action']
    params = action.get('parameters', {})
    
    try:
        if action_type == 'navigate_to':
            return self.execute_navigate_to(params)
        elif action_type == 'detect_object':
            return self.execute_detect_object(params)
        elif action_type == 'grasp_object':
            return self.execute_grasp_object(params)
        elif action_type == 'place_object':
            return self.execute_place_object(params)
        elif action_type == 'scan_area':
            return self.execute_scan_area(params)
        elif action_type == 'report_status':
            return self.execute_report_status(params)
        else:
            self.get_logger().error(f"Unknown action type: {action_type}")
            return False
            
    except Exception as e:
        self.get_logger().error(f"Error executing {action_type}: {e}")
        return False
```

### Monitor progress and provide feedback

The controller monitors execution and provides feedback:

```python
def execute_with_feedback(self, goal, action_client, action_type):
    """Execute an action with progress feedback."""
    action_client.wait_for_server()
    future = action_client.send_goal_async(goal)
    
    # Create feedback callback
    feedback_callback = lambda feedback: self.handle_action_feedback(feedback, action_type)
    
    rclpy.spin_until_future_complete(self, future)
    goal_handle = future.result()
    
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        return result.success, result.message
    else:
        return False, "Action goal was rejected"

def handle_action_feedback(self, feedback, action_type):
    """Handle action feedback during execution."""
    # Update status based on feedback
    status_msg = String()
    if hasattr(feedback, 'status'):
        status_msg.data = f"Action {action_type}: {feedback.status}"
    elif hasattr(feedback, 'progress'):
        status_msg.data = f"Action {action_type}: {feedback.progress * 100:.1f}% complete"
    
    self.status_publisher.publish(status_msg)
```

### Handle failures and replan

The controller handles execution failures and can trigger replanning:

```python
def handle_action_failure(self, failed_action, full_plan, failed_index):
    """Handle failure of an action in the plan."""
    self.get_logger().error(f"Action failed: {failed_action}")
    
    # Determine if we should attempt recovery or replan
    if self.can_retry_action(failed_action):
        # Simple retry once
        success = self.execute_single_action(failed_action)
        if success:
            self.get_logger().info("Action recovery successful")
            # Continue with the rest of the plan
            for i in range(failed_index + 1, len(full_plan)):
                action = full_plan[i]
                success = self.execute_single_action(action)
                if not success:
                    break
    else:
        # Request replanning from LLM with failure context
        status_msg = String()
        status_msg.data = f"Action failed, requesting replan: {failed_action}"
        self.status_publisher.publish(status_msg)
        
        # In a complete implementation, we would send context about the failure
        # to the LLM planner and generate a new plan
        self.request_replanning(failed_action, full_plan, failed_index)

def request_replanning(self, failed_action, original_plan, failed_index):
    """Request a new plan from LLM considering the failure."""
    # Create context about what happened
    failure_context = {
        "failed_action": failed_action,
        "original_plan": original_plan,
        "failed_at_step": failed_index,
        "current_state": {
            "location": self.current_location,
            "holding": self.held_object,
        }
    }
    
    # Generate new plan with failure context
    new_plan = self.llm_planner.replan_after_failure(failure_context)
    
    # Execute new plan
    if new_plan:
        self.get_logger().info("Executing replanned sequence")
        self.execute_plan(new_plan)
    else:
        self.get_logger().error("Could not generate replan after failure")
        error_msg = String()
        error_msg.data = "Could not complete task after failure and replanning"
        self.status_publisher.publish(error_msg)
```

## 15. Error Handling and Replanning

### Action failure types:

Different types of failures require different handling strategies:

```python
class ActionFailureTypes:
    NAVIGATION_BLOCKED = "navigation_blocked"
    OBJECT_NOT_FOUND = "object_not_found"
    GRASP_FAILED = "grasp_failed"
    TIMEOUT = "timeout"
    SAFETY_VIOLATION = "safety_violation"
    ENVIRONMENT_CHANGED = "environment_changed"
    ROBOT_ERROR = "robot_error"
```

### Navigation blocked

When the robot cannot reach its destination:

```python
def handle_navigation_blocked(self, goal_location):
    """Handle case where navigation is blocked."""
    self.get_logger().warn(f"Navigation to {goal_location} is blocked")
    
    # Check if the blockage is temporary (moving obstacle) or permanent
    # This might involve sensing for temporary obstacles
    
    # Try alternative routes if available
    alternative_goals = self.find_alternative_destinations(goal_location)
    if alternative_goals:
        for alt_goal in alternative_goals:
            try:
                success = self.execute_navigation_to(alt_goal)
                if success:
                    self.get_logger().info(f"Fallback navigation to {alt_goal} successful")
                    return True
            except:
                continue
    
    # If no alternatives work, report failure
    return False

def find_alternative_destinations(self, original_goal):
    """Find alternative locations that serve the same purpose."""
    # For example, if "kitchen" is blocked, maybe "kitchen_entrance" or "counter" works
    alternatives = {
        "kitchen": ["kitchen_entrance", "counter", "dining_area"],
        "bin": ["waste_container", "disposal_area"],
        "charger": ["charging_station", "docking_area", "home_base"]
    }
    
    return alternatives.get(original_goal, [])
```

### Object not found

When the expected object is not detected:

```python
def handle_object_not_found(self, object_type, search_location):
    """Handle case where expected object is not found."""
    self.get_logger().warn(f"Could not find {object_type} in {search_location}")
    
    # Expand search area
    expanded_search_result = self.search_in_expanded_area(search_location)
    if expanded_search_result:
        for obj in expanded_search_result:
            if object_type in obj['class']:
                self.get_logger().info(f"Found {object_type} in expanded search: {obj['location']}")
                return obj['location']
    
    # Report to user and request clarification
    self.request_user_assistance(f"Could not find {object_type}. Can you describe its location?")
    
    # Or use LLM to generate alternative plan
    return self.generate_alternative_plan_for_missing_object(object_type)

def search_in_expanded_area(self, center_location):
    """Search in a wider area around the specified location."""
    # In a real implementation, this would trigger more extensive search
    # patterns around the expected location
    pass

def request_user_assistance(self, query):
    """Request help from the user via speech synthesis."""
    # Use text-to-speech to ask for help
    # In a complete implementation, this would integrate speech synthesis
    status_msg = String()
    status_msg.data = query
    self.status_publisher.publish(status_msg)

def generate_alternative_plan_for_missing_object(self, object_type):
    """Ask LLM for alternative plan when object is missing."""
    # This would send the context to the LLM and request a new plan
    pass
```

### Grasp failed

When the robot cannot grasp an object:

```python
def handle_grasp_failed(self, object_info):
    """Handle case where grasping fails."""
    self.get_logger().warn(f"Grasp failed for {object_info.get('type', 'unknown object')}")
    
    # Check if grasp failed due to:
    # 1. Incorrect grasp pose
    # 2. Object is too heavy/small
    # 3. Object is in an inaccessible position
    
    # Try different grasp approach
    alternative_grasps = self.compute_alternative_grasps(object_info)
    for grasp_pose in alternative_grasps:
        if self.attempt_grasp_at_pose(object_info, grasp_pose):
            return True
    
    # If all grasps fail, report to user
    error_msg = String()
    error_msg.data = f"Cannot grasp {object_info.get('type', 'object')}. It might be too heavy, small, or in a hard-to-reach position."
    self.status_publisher.publish(error_msg)
    
    return False

def compute_alternative_grasps(self, object_info):
    """Compute alternative grasp poses for an object."""
    # In a real implementation, this would use grasp planning algorithms
    # to find different ways to grasp the same object
    pass
```

### Timeout failures

When actions take too long:

```python
def handle_timeout(self, action_type, action_params, timeout_duration):
    """Handle timeout for any action."""
    self.get_logger().warn(f"Action {action_type} timed out after {timeout_duration}s")
    
    # For navigation, this might mean getting stuck
    if action_type == "navigate_to":
        return self.handle_navigation_timeout(action_params)
    
    # For detection, this might mean object not in view
    elif action_type == "detect_object":
        return self.handle_detection_timeout(action_params)
    
    # For grasping, this might mean object too far away
    elif action_type == "grasp_object":
        return self.handle_grasp_timeout(action_params)
    
    # Default timeout handling
    else:
        return self.default_timeout_handling(action_type, action_params)

def handle_navigation_timeout(self, params):
    """Handle navigation timeout."""
    # Stop the robot immediately
    self.emergency_stop()
    
    # Re-evaluate current situation
    current_pos = self.get_robot_position()
    
    # Check if we're close enough to goal to consider it reached
    target_pos = self.get_location_coordinates(params['location'])
    if self.calculate_distance(current_pos, target_pos) < 0.5:  # 0.5m tolerance
        self.get_logger().info("Navigation timeout, but close enough to target")
        return True
    
    # Otherwise, report failure and request replanning
    return False
```

### Recovery strategies:

Different types of errors require specific recovery strategies:

```python
class RecoveryStrategies:
    def __init__(self, controller):
        self.controller = controller
    
    def retry_with_adjustments(self, failed_action):
        """Retry the same action with minor adjustments."""
        # For navigation: try with different planner parameters
        # For grasping: try with different grasp points
        # For detection: adjust camera angle or move robot position
        pass
    
    def request_clarification_from_user(self, failed_action):
        """Ask the user for clarification or help."""
        # Use speech synthesis to ask for help
        query = self.generate_help_request(failed_action)
        self.controller.request_user_assistance(query)
    
    def replan_with_llm(self, failed_action, context):
        """Request the LLM to generate an alternative plan."""
        # Send failure context to LLM and get new plan
        new_plan = self.controller.llm_planner.handle_failure(failed_action, context)
        return new_plan
    
    def abort_and_report(self, failed_action):
        """Abort the current task and report failure."""
        # Publish failure status
        error_msg = String()
        error_msg.data = f"Task failed at step: {failed_action}"
        self.controller.status_publisher.publish(error_msg)
        
        return False  # Indicates failure to continue

def generate_help_request(self, failed_action):
    """Generate a help request for a failed action."""
    action_type = failed_action['action']
    params = failed_action.get('parameters', {})
    
    if action_type == 'navigate_to':
        return f"I cannot reach the {params.get('location', 'destination')}. Is the path clear?"
    elif action_type == 'detect_object':
        return f"I cannot find the {params.get('object_type', 'object')}. Can you point it out?"
    elif action_type == 'grasp_object':
        return f"I cannot pick up the {params.get('object_type', 'object')}. Is it accessible?"
    else:
        return f"I failed to execute the action: {action_type}. Can you help?"
```

### Implementing robust error handling

A comprehensive error handling system:

```python
class VLAErrorManager:
    def __init__(self, controller):
        self.controller = controller
        self.failure_history = []  # Track failures to avoid infinite loops
    
    def handle_action_error(self, failed_action, original_plan, failed_step_index):
        """Main entry point for error handling."""
        error_type = self.classify_error(failed_action)
        self.log_failure(failed_action, error_type)
        
        # Check if this failure pattern has occurred before
        if self.is_recurring_failure(failed_action):
            return self.execute_escalation_procedure(failed_action)
        
        # Try recovery strategies in order of preference
        recovery_strategies = [
            lambda: self.try_simple_recovery(failed_action),
            lambda: self.request_user_assistance(failed_action),
            lambda: self.replan_after_failure(failed_action, original_plan, failed_step_index)
        ]
        
        for recovery_strategy in recovery_strategies:
            try:
                result = recovery_strategy()
                if result is not None and result != False:
                    return result
            except Exception as e:
                self.controller.get_logger().warn(f"Recovery strategy failed: {e}")
                continue
        
        # If all recovery strategies fail, abort
        return self.abort_execution(failed_action)
    
    def classify_error(self, failed_action):
        """Classify the type of error based on the action and result."""
        # In a real system, this would have more sophisticated classification
        return "unknown_error"
    
    def log_failure(self, failed_action, error_type):
        """Log the failure for future analysis."""
        failure_record = {
            'timestamp': self.controller.get_clock().now(),
            'action': failed_action,
            'error_type': error_type,
            'context': self.get_current_context()
        }
        self.failure_history.append(failure_record)
    
    def is_recurring_failure(self, failed_action):
        """Check if this type of failure has happened recently."""
        # Check if similar failures have occurred in the last N attempts
        recent_failures = [f for f in self.failure_history 
                          if (self.controller.get_clock().now() - f['timestamp']).nanoseconds < 30e9]  # 30 seconds
        
        for failure in recent_failures:
            if failure['action']['action'] == failed_action['action']:
                return True
        return False
    
    def try_simple_recovery(self, failed_action):
        """Try simple recovery approaches."""
        action_type = failed_action['action']
        
        if action_type == 'navigate_to':
            # Try to clear local costmap
            self.controller.clear_costmaps()
            # Retry with same goal
            return self.controller.execute_single_action(failed_action)
        
        elif action_type == 'detect_object':
            # Try to reposition robot to get better view
            self.controller.reposition_for_detection(failed_action.get('parameters', {}))
            return self.controller.execute_single_action(failed_action)
        
        # Add similar simple recovery for other action types
        return None
    
    def request_user_assistance(self, failed_action):
        """Request help from the user."""
        help_request = self.generate_help_request(failed_action)
        self.controller.request_user_assistance(help_request)
        # Wait for user response before continuing (implementation dependent)
        return None  # Return None to continue to next strategy
    
    def replan_after_failure(self, failed_action, original_plan, failed_index):
        """Request the LLM to replan after failure."""
        # Provide failure context to LLM
        failure_context = {
            'failed_action': failed_action,
            'original_plan': original_plan,
            'failed_at_index': failed_index,
            'current_state': self.get_current_context()
        }
        
        try:
            new_plan = self.controller.llm_planner.replan_after_failure(failure_context)
            if new_plan:
                self.controller.get_logger().info("Executing replanned sequence")
                return self.controller.execute_plan(new_plan)
        except Exception as e:
            self.controller.get_logger().error(f"Replanning failed: {e}")
        
        return None
    
    def abort_execution(self, failed_action):
        """Abort the current execution with proper cleanup."""
        self.controller.get_logger().error(f"Aborting execution after failure: {failed_action}")
        
        # Emergency stop if robot is in motion
        self.controller.emergency_stop()
        
        # Reset robot state if needed
        if self.controller.held_object:
            self.controller.attempt_safe_place()
        
        # Report failure
        error_msg = String()
        error_msg.data = f"Task aborted due to unrecoverable error: {failed_action['action']}"
        self.controller.status_publisher.publish(error_msg)
        
        return False
    
    def get_current_context(self):
        """Get current robot state and environment context."""
        return {
            'location': self.controller.current_location,
            'holding': self.controller.held_object,
            'battery_level': self.get_battery_level(),
            'last_detections': self.controller.get_recent_detections(),
            'obstacle_map': self.controller.get_local_map()
        }

def get_battery_level(self):
    """Get current battery level of the robot."""
    # In a real implementation, this would get actual battery status
    return 85  # Simulated value
```

## 16. Feedback and Human-in-the-Loop

### Speech synthesis for robot responses

The robot uses text-to-speech to communicate with users:

```python
import pyttsx3
import threading
import queue


class SpeechSynthesizer:
    def __init__(self):
        self.engine = pyttsx3.init()
        
        # Configure speech properties
        self.engine.setProperty('rate', 150)  # Speed of speech
        self.engine.setProperty('volume', 0.9)  # Volume level (0.0 to 1.0)
        
        # Queue for speech requests to handle them sequentially
        self.speech_queue = queue.Queue()
        self.speaking_thread = threading.Thread(target=self.speech_worker, daemon=True)
        self.speaking_thread.start()
    
    def speak(self, text, blocking=False):
        """Add text to speech queue."""
        self.speech_queue.put((text, blocking))
    
    def speech_worker(self):
        """Worker thread to handle speech in sequence."""
        while True:
            text, blocking = self.speech_queue.get()
            
            if blocking:
                # Wait for speech to complete
                self.engine.say(text)
                self.engine.runAndWait()
            else:
                # Non-blocking speech
                self.engine.say(text)
                self.engine.startLoop(False)
                
                # Process events to keep speech moving
                while self.engine.isBusy():
                    self.engine.iterate()
                
            self.speech_queue.task_done()
    
    def interrupt_speech(self):
        """Stop current speech and clear queue."""
        self.engine.stop()
        with self.speech_queue.mutex:
            self.speech_queue.queue.clear()


class RobotCommunicator(Node):
    def __init__(self):
        super().__init__('robot_communicator')
        
        # Initialize speech synthesizer
        self.speech_synthesizer = SpeechSynthesizer()
        
        # Subscribe to robot status updates to know when to speak
        self.status_subscription = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10
        )
        
        # Publisher for user requests/confirmations
        self.user_request_publisher = self.create_publisher(String, '/user_requests', 10)
    
    def status_callback(self, msg):
        """Handle status updates from the VLA controller."""
        status_text = msg.data
        
        # Determine if this status update should trigger speech
        if self.should_speak_status(status_text):
            self.speech_synthesizer.speak(status_text)
    
    def should_speak_status(self, status_text):
        """Determine if a status should be spoken aloud."""
        # Only speak important status updates, not internal processing details
        important_keywords = [
            "completed", "arrived", "grasped", "placed", 
            "failed", "help", "cannot", "error", "success"
        ]
        
        text_lower = status_text.lower()
        return any(keyword in text_lower for keyword in important_keywords)
    
    def request_user_confirmation(self, action_description):
        """Request user confirmation for critical actions."""
        confirmation_request = f"About to {action_description}. Confirm by saying 'yes'."
        self.speech_synthesizer.speak(confirmation_request)
        
        # Also publish to a user request topic for other systems
        request_msg = String()
        request_msg.data = f"CONFIRM:{action_description}"
        self.user_request_publisher.publish(request_msg)
```

### "I'm navigating to the table..."

Status updates during action execution:

```python
class ActionWithFeedback:
    def __init__(self, node):
        self.node = node
        self.status_publisher = node.status_publisher
    
    def publish_action_start(self, action_type, params):
        """Publish status when an action starts."""
        status_msg = String()
        if action_type == 'navigate_to':
            status_msg.data = f"Starting navigation to {params['location']}"
        elif action_type == 'detect_object':
            obj_desc = f"{params.get('color', '')} {params['object_type']}".strip()
            status_msg.data = f"Looking for {obj_desc}"
        elif action_type == 'grasp_object':
            status_msg.data = f"Attempting to grasp {params['object_type']}"
        elif action_type == 'place_object':
            status_msg.data = f"Placing object at {params['location']}"
        
        self.status_publisher.publish(status_msg)
        # Also speak if appropriate
        self.node.speech_synthesizer.speak(status_msg.data)
    
    def publish_action_progress(self, action_type, progress_info):
        """Publish progress updates during action execution."""
        status_msg = String()
        status_msg.data = f"{action_type}: {progress_info}"
        self.status_publisher.publish(status_msg)
    
    def publish_action_complete(self, action_type, result):
        """Publish status when an action completes."""
        status_msg = String()
        if result.success:
            if action_type == 'navigate_to':
                status_msg.data = f"Successfully reached {result.message}"
            elif action_type == 'detect_object':
                status_msg.data = f"Found the object successfully"
            elif action_type == 'grasp_object':
                status_msg.data = f"Successfully grasped the object"
            elif action_type == 'place_object':
                status_msg.data = f"Successfully placed the object"
        else:
            status_msg.data = f"{action_type} failed: {result.message}"
        
        self.status_publisher.publish(status_msg)
        # Speak completion
        self.node.speech_synthesizer.speak(status_msg.data)
```

### "I found the cup, grasping now..."

Real-time updates during complex operations:

```python
def execute_detect_and_grasp_sequence(self, object_type):
    """Execute a sequence of detection followed by grasping."""
    
    # First, publish what we're doing
    self.publish_action_start('detect_object', {'object_type': object_type})
    
    # Execute detection
    detect_success, detect_result = self.execute_detection(object_type)
    
    if detect_success and detect_result.object_found:
        self.publish_action_complete('detect_object', detect_result)
        
        # Now we know where the object is, publish grasp intent
        self.publish_action_start('grasp_object', {'object_type': object_type})
        
        # Execute grasp
        grasp_success, grasp_result = self.execute_grasp(object_type)
        
        if grasp_success:
            self.publish_action_complete('grasp_object', grasp_result)
            return True
        else:
            self.publish_action_complete('grasp_object', grasp_result)
            return False
    else:
        # Detection failed
        self.publish_action_complete('detect_object', detect_result)
        return False

def execute_detection(self, object_type):
    """Execute detection with real-time progress updates."""
    goal = DetectObject.Goal()
    goal.object_type = object_type
    
    self.detect_client.wait_for_server()
    future = self.detect_client.send_goal_async(goal)
    
    # Set up feedback callback
    def feedback_callback(feedback_msg):
        self.publish_action_progress('detect_object', feedback_msg.status)
    
    # In a real implementation, this would connect to the actual feedback
    # For now, we'll just wait and then return
    rclpy.spin_until_future_complete(self, future)
    goal_handle = future.result()
    
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return result.success, result
    else:
        result = DetectObject.Result()
        result.success = False
        result.message = "Detection goal was rejected"
        return False, result
```

### "I couldn't find the object, please help"

Requesting user assistance when autonomous execution fails:

```python
def request_user_assistance(self, reason):
    """Request help from the user when autonomous execution fails."""
    # Use speech synthesis to ask for help
    help_request = f"I couldn't {reason}. Can you help me?"
    self.speech_synthesizer.speak(help_request)
    
    # Publish request to user interface
    request_msg = String()
    request_msg.data = f"ASSISTANCE_REQUESTED:{reason}"
    self.user_request_publisher.publish(request_msg)
    
    # Also log the request for other systems
    self.get_logger().info(f"Assistance requested: {reason}")

def handle_object_not_found_assistance(self, object_type):
    """Handle case where object can't be found and request user help."""
    self.get_logger().warn(f"Could not find {object_type}")
    
    # Provide specific information to help user
    current_location = self.current_location
    last_seen_location = self.get_last_known_location(object_type)
    
    if last_seen_location and last_seen_location != current_location:
        help_text = f"I can't find the {object_type}. It was last seen at {last_seen_location}. " \
                   f"I am currently at {current_location}. Can you help me locate it?"
    else:
        help_text = f"I can't find the {object_type} in my current area. " \
                   f"Can you point it out or tell me where it is?"
    
    self.request_user_assistance(help_text)
```

### User confirmation for critical actions

For safety-critical actions, request user confirmation:

```python
def execute_with_confirmation(self, action, params, is_critical=False):
    """Execute an action, requesting confirmation for critical ones."""
    if is_critical:
        # For critical actions, request user confirmation
        confirmation = self.request_action_confirmation(action, params)
        if not confirmation:
            self.get_logger().info(f"User denied permission for critical action: {action}")
            return False
    
    # Execute the action
    return self.execute_single_action(action, params)

def request_action_confirmation(self, action, params):
    """Request user confirmation for a critical action."""
    action_descriptions = {
        'navigate_to': lambda p: f"navigate to {p.get('location', 'unknown location')}",
        'grasp_object': lambda p: f"grasp the {p.get('object_type', 'unknown object')}",
        'place_object': lambda p: f"place the object at {p.get('location', 'unknown location')}",
    }
    
    if action in action_descriptions:
        action_desc = action_descriptions[action](params)
        confirmation_query = f"About to {action_desc}. Confirm by saying 'yes' or deny by saying 'no'."
    else:
        confirmation_query = f"About to execute {action} with params {params}. Confirm by saying 'yes'."
    
    # Publish confirmation request
    request_msg = String()
    request_msg.data = f"CONFIRMATION_NEEDED:{confirmation_query}"
    self.user_request_publisher.publish(request_msg)
    
    # Use speech to ask
    self.speech_synthesizer.speak(confirmation_query)
    
    # Wait for user response (in a real implementation, this would have a timeout)
    return self.wait_for_user_confirmation()

def wait_for_user_confirmation(self):
    """Wait for user confirmation (simplified implementation)."""
    # In a real implementation, this would listen for voice commands
    # or button presses indicating yes/no
    # For simulation, we'll return True
    return True
```

### Emergency stop mechanisms

Critical safety feature for human-robot interaction:

```python
class SafetyManager:
    def __init__(self, node):
        self.node = node
        
        # Subscribe to emergency stop commands
        self.emergency_stop_subscription = node.create_subscription(
            String,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Subscribe to safety sensor inputs
        self.safety_sensor_subscription = node.create_subscription(
            String,  # In reality, this might be a custom safety message
            '/safety_sensors',
            self.safety_sensor_callback,
            10
        )
        
        self.safety_enabled = True
    
    def emergency_stop_callback(self, msg):
        """Handle emergency stop commands."""
        if msg.data == "STOP" and self.safety_enabled:
            self.execute_emergency_stop()
    
    def safety_sensor_callback(self, msg):
        """Handle safety sensor inputs."""
        # Check if safety sensors indicate dangerous situation
        if self.is_safety_violation(msg):
            self.execute_safety_stop()
    
    def is_safety_violation(self, sensor_msg):
        """Determine if sensors indicate safety violation."""
        # In a real implementation, this would check various safety sensors
        # like proximity sensors, force sensors, collision sensors, etc.
        return "VIOLATION" in sensor_msg.data
    
    def execute_emergency_stop(self):
        """Execute emergency stop of all robot motion."""
        self.node.get_logger().error("EMERGENCY STOP ACTIVATED")
        
        # Stop all motion
        self.node.stop_all_motion()
        
        # Cancel any active goals
        self.node.cancel_all_goals()
        
        # Speak warning
        self.node.speech_synthesizer.speak("Emergency stop activated. Robot halted.")
        
        # Disable safety system until reset
        self.safety_enabled = False
    
    def execute_safety_stop(self):
        """Execute safety stop for non-emergency but unsafe conditions."""
        self.node.get_logger().warn("SAFETY STOP ACTIVATED")
        
        # Reduce speed to safe level
        self.node.reduce_speed_to_safe_level()
        
        # Speak warning
        self.node.speech_synthesizer.speak("Safety stop: continuing at safe speed.")
```

## 17. Complete VLA Pipeline Implementation

### Integrated system launch file

```python
# launch/vla_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get package directory
    pkg_name = 'your_vla_package'  # Replace with your actual package name
    
    return LaunchDescription([
        # Declare launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Voice command node
        Node(
            package='your_vla_package',
            executable='voice_command_node',
            name='voice_command_node',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        # LLM planner node
        Node(
            package='your_vla_package', 
            executable='llm_planner_node',
            name='llm_planner_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'model_name': 'gpt-4'},  # or whatever LLM you're using
                {'api_key': ''}  # In production, pass via secure means
            ],
            output='screen'
        ),
        
        # VLA controller node
        Node(
            package='your_vla_package',
            executable='vla_controller',
            name='vla_controller',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        # Core action servers
        Node(
            package='your_vla_package',
            executable='navigate_to_server',
            name='navigate_to_server',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        Node(
            package='your_vla_package', 
            executable='detect_object_server',
            name='detect_object_server',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        Node(
            package='your_vla_package',
            executable='grasp_object_server', 
            name='grasp_object_server',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        Node(
            package='your_vla_package',
            executable='place_object_server',
            name='place_object_server', 
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        # Isaac Sim integration nodes (if applicable)
        Node(
            package='isaac_ros_detectnet',
            executable='isaac_ros_detectnet',
            name='isaac_ros_detectnet',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'model_file': 'config/yolov5n.pt'}  # Example Isaac model
            ],
            output='screen'
        )
    ])
```

### All nodes running together:

The complete system includes:

- **Whisper voice node**: Handles voice command recognition
- **LLM planner node**: Translates commands to action sequences
- **Action servers**: Navigate, detect, grasp, place
- **VLA controller node**: Orchestrates the entire pipeline
- **Isaac Sim / Gazebo simulation**: Provides the environment

### Testing end-to-end flow

```python
# test/vla_integration_test.py
import unittest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from example_interfaces.action import NavigateTo, DetectObject, GraspObject, PlaceObject


class VLASystemIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('vla_integration_tester')
        
        # Create clients for all action servers
        self.navigate_client = ActionClient(self.node, NavigateTo, 'navigate_to')
        self.detect_client = ActionClient(self.node, DetectObject, 'detect_object')
        self.grasp_client = ActionClient(self.node, GraspObject, 'grasp_object')
        self.place_client = ActionClient(self.node, PlaceObject, 'place_object')

    def tearDown(self):
        self.node.destroy_node()

    def test_simple_pick_and_place(self):
        """Test a simple pick and place operation."""
        # This test would simulate the entire VLA pipeline
        # For a real test, you would need a simulated environment
        
        # In a real implementation, this would:
        # 1. Simulate voice command: "Pick up the red cup and put it on the table"
        # 2. Verify LLM generates appropriate action sequence
        # 3. Verify each action executes correctly
        # 4. Verify final state matches expectations
        
        # For this example, we'll just verify action servers are available
        self.assertTrue(self.navigate_client.wait_for_server(timeout_sec=5.0))
        self.assertTrue(self.detect_client.wait_for_server(timeout_sec=5.0))
        self.assertTrue(self.grasp_client.wait_for_server(timeout_sec=5.0))
        self.assertTrue(self.place_client.wait_for_server(timeout_sec=5.0))

    def test_command_processing_pipeline(self):
        """Test the command processing pipeline from voice to action."""
        # Test that voice commands flow through the system correctly
        # This would involve publishing to /voice_command and verifying 
        # that appropriate actions are sent to the action servers
        
        # For this example, we'll just verify the publisher can be created
        command_publisher = self.node.create_publisher(String, '/voice_command', 10)
        self.assertIsNotNone(command_publisher)


def main():
    unittest.main()
```

### Example mission: "Bring me the water bottle"

The complete flow for this command:

```python
def example_mission_flow():
    """
    Example flow for: "Bring me the water bottle"
    
    1. Voice Command: "Bring me the water bottle"
    2. Whisper: Transcribes to text
    3. LLM Planner: Generates action sequence
       [
         {"action": "detect_object", "parameters": {"object_type": "water bottle"}},
         {"action": "grasp_object", "parameters": {"object_type": "water bottle"}},
         {"action": "navigate_to", "parameters": {"location": "user"}},
         {"action": "place_object", "parameters": {"location": "user"}}
       ]
    4. VLA Controller: Executes the sequence
       - Detect water bottle using Isaac perception
       - Navigate to bottle location
       - Grasp the bottle
       - Navigate back to user
       - Place bottle near user
    5. System reports completion
    """
    pass
```

## 18. Capstone Project: The Autonomous Humanoid

### Project overview and objectives

The capstone project integrates all components learned in Modules 1-4 to create a fully autonomous humanoid robot capable of understanding natural language commands, perceiving its environment, planning complex tasks, and executing physical actions.

**Project Goals:**
- Demonstrate end-to-end VLA (Vision-Language-Action) capabilities
- Implement safe human-robot interaction
- Showcase perception-reasoning-action loop
- Develop error handling and replanning systems
- Create an engaging user experience with speech output

**Success Metrics:**
- Successfully complete the "clean the room" mission 80% of the time
- Achieve >90% voice command recognition accuracy
- Execute multi-step tasks with fewer than 3 failures per mission
- Respond to human operators with appropriate status updates

### Mission scenario: "Clean the room"

The primary capstone mission involves:

1. **Receive voice command**: "Clean the room"
2. **LLM task decomposition**: Break down into navigation, detection, manipulation steps
3. **Environmental perception**: Detect objects that need to be cleaned up
4. **Navigation**: Move to objects that need to be picked up
5. **Manipulation**: Grasp objects and place them in appropriate locations
6. **Monitoring**: Track progress and handle failures
7. **Reporting**: Communicate mission status and completion

### Detailed requirements:

#### 1. Receive voice command: "Clean the room"
- System recognizes the command using Whisper
- LLM interprets the high-level goal
- Generates a task plan for room cleaning

#### 2. LLM generates action sequence:
```
[
  {"action": "scan_area", "parameters": {"location": "current"}},
  {"action": "navigate_to", "parameters": {"location": "table"}},
  {"action": "detect_object", "parameters": {"object_type": "trash"}},
  {"action": "grasp_object", "parameters": {"object_type": "trash"}},
  {"action": "navigate_to", "parameters": {"location": "bin"}},
  {"action": "place_object", "parameters": {"location": "bin"}},
  {"action": "report_status", "parameters": {"message": "Placed first item in bin"}}
]
```

#### 3. Execute in Isaac Sim with full perception
- Use Isaac Sim environment for safe testing
- Integrate Isaac perception nodes for object detection
- Use Isaac navigation capabilities

#### 4. Handle obstacles and failures gracefully
- Detect when objects can't be found
- Navigate around unexpected obstacles
- Request human assistance when needed
- Replan when initial attempts fail

#### 5. Report completion
- Use speech synthesis to report progress
- Confirm mission completion
- Return to home position

## 19. Capstone: Scene Setup

### Isaac Sim room environment

Setting up the Isaac Sim environment for the capstone project:

1. **Environment Design**:
   - Room layout with furniture (tables, chairs, bins)
   - Cluttered areas with objects to "clean up"
   - Clear paths for robot navigation
   - Designated locations for object placement (bins, shelves)

2. **Lighting and Materials**:
   - Realistic lighting to test perception
   - Varied surface materials for realistic physics
   - Proper textures for visual recognition

3. **Robot Placement**:
   - Humanoid robot positioned at starting location
   - Proper joint configurations for manipulation
   - Sensor configurations (cameras, LiDAR) correctly positioned

### Humanoid robot with sensors (RGB-D, LiDAR)

```python
# In Isaac Sim, configure the humanoid robot with appropriate sensors

# Add RGB-D camera to robot head
omni.kit.commands.execute(
    "Isaac.Sensors.CreateRgbdCamera",
    path="/World/Robot/head/rgbd_camera",
    position=(0.0, 0.0, 0.7),  # Mount on head
    rotation=(0, 0, 0, 1),
    resolution="1280x720"
)

# Add 2D LiDAR to robot torso
omni.kit.commands.execute(
    "Isaac.Sensors.CreateLidar",
    path="/World/Robot/torso/lidar",
    position=(0.0, 0.0, 0.5),  # Mount on torso
    rotation=(0, 0, 0, 1),
    config="VLP-16"  # Example LiDAR configuration
)
```

### Objects: cups, books, trash items

Scene includes various objects for the robot to detect and manipulate:

- **Cups**: Various sizes, colors, materials
- **Books**: Different orientations, sizes
- **Trash items**: Paper, small objects, bottles
- **Placement targets**: Trash bin, shelves, tables

### Trash bin as target location

Designated location where the robot should place misplaced objects:

- Properly configured collision volumes
- Recognizable visual and semantic features
- Appropriate size for robot to approach and place objects

### Navigation waypoints

Pre-defined locations in the environment where the robot can navigate:

- Table areas (for object detection)
- Bin location (for object placement)
- Charging station (mission completion)
- Obstacle-free paths between waypoints

### Collision objects (furniture)

Environment includes static obstacles that the robot must navigate around:

- Tables, chairs, walls
- Proper collision volumes configured
- Physics properties set appropriately

## 20. Capstone: Implementation Steps

### Step 1: Voice command capture

```python
class CapstoneVoiceInterface(Node):
    def __init__(self):
        super().__init__('capstone_voice_interface')
        
        # Publisher for voice commands to VLA controller
        self.command_publisher = self.create_publisher(String, '/capstone/voice_command', 10)
        
        # Initialize Whisper for voice recognition
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.whisper_model = whisper.load_model("medium").to(device)
        
        # Audio capture setup
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=8192
        )
        
        # Audio buffer for processing
        self.audio_buffer = np.array([])
        
        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self.capture_audio)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info("Capstone Voice Interface initialized")

    def capture_audio(self):
        """Continuously capture audio and process for wake word/commands."""
        while rclpy.ok():
            try:
                data = self.stream.read(8192, exception_on_overflow=False)
                audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                
                # Add to buffer for processing
                self.audio_buffer = np.concatenate([self.audio_buffer, audio_chunk])
                
                # Process when buffer has sufficient audio (2 seconds)
                if len(self.audio_buffer) >= 2 * 16000:
                    # Check if this contains the start of a command
                    # For the capstone, we'll listen for commands like "Clean the room"
                    transcript = self.transcribe_audio(self.audio_buffer[:16000])  # Process 1 second
                    
                    if transcript and self.is_valid_command(transcript):
                        # Publish the command to the VLA controller
                        cmd_msg = String()
                        cmd_msg.data = transcript.strip()
                        self.command_publisher.publish(cmd_msg)
                        self.get_logger().info(f"Published capstone command: {transcript}")
                    
                    # Keep some overlap for continuous recognition
                    self.audio_buffer = self.audio_buffer[8000:]  # Keep half second overlap
                    
            except Exception as e:
                self.get_logger().error(f"Audio capture error: {e}")

    def is_valid_command(self, transcript):
        """Check if the transcript is a valid capstone command."""
        valid_keywords = [
            "clean the room", "clean up", "pick up", "tidy up",
            "organize", "put away", "throw away", "dispose"
        ]
        transcript_lower = transcript.lower()
        return any(keyword in transcript_lower for keyword in valid_keywords)
```

### Step 2: LLM task decomposition

```python
class CapstoneLLMPlanner:
    def __init__(self):
        # Initialize LLM client (OpenAI, Anthropic, etc.)
        self.system_prompt = self.create_capstone_system_prompt()
    
    def create_capstone_system_prompt(self):
        """Create specialized system prompt for capstone room cleaning task."""
        return f"""
        You are the task planner for a humanoid robot performing room cleaning. 
        The user has asked you to "Clean the room".
        
        Your job is to decompose this high-level task into a sequence of specific actions.
        Use these available actions:
        
        - scan_area: Look around to identify objects needing cleanup
        - navigate_to: Move to a specific location (table, bin, etc.)
        - detect_object: Find a specific type of object
        - grasp_object: Pick up an object
        - place_object: Put down an object at a location
        - report_status: Communicate status to the user
        - patrol_area: Move between multiple locations to find objects
        
        Generate action sequences that:
        1. First scan the area to find objects to clean
        2. Navigate to objects
        3. Grasp them
        4. Place them in appropriate locations (bin for trash, shelf for books, etc.)
        5. Patrolling to find more objects until area is clean
        6. Return to home position when done
        
        Example for "Clean the room":
        [
          {{"action": "scan_area", "parameters": {{}}}},
          {{"action": "patrol_area", "parameters": {{"locations": ["table1", "table2", "floor"]}}}},
          {{"action": "detect_object", "parameters": {{"object_type": "trash"}}}},
          {{"action": "navigate_to", "parameters": {{"location": "object_location"}}}},
          {{"action": "grasp_object", "parameters": {{"object_type": "trash"}}}},
          {{"action": "navigate_to", "parameters": {{"location": "bin"}}}},
          {{"action": "place_object", "parameters": {{"location": "bin"}}}},
          {{"action": "report_status", "parameters": {{"message": "Placed trash in bin"}}}},
          {{"action": "scan_area", "parameters": {{}}}},
          {{"action": "navigate_to", "parameters": {{"location": "home"}}}}
        ]
        """
    
    def plan_cleaning_task(self, command="Clean the room"):
        """Generate action plan for room cleaning task."""
        # For the capstone, we'll use a specialized prompt for room cleaning
        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": f"User command: {command}"},
            {"role": "assistant", "content": '```json\n[\n  {"action": "scan_area", "parameters": {}},\n  {"action": "patrol_area", "parameters": {"locations": ["table", "desk", "floor"]}},\n  {"action": "detect_object", "parameters": {"object_type": "clutter"}},\n  {"action": "navigate_to", "parameters": {"location": "object_location"}},\n  {"action": "grasp_object", "parameters": {"object_type": "clutter"}},\n  {"action": "navigate_to", "parameters": {"location": "bin"}},\n  {"action": "place_object", "parameters": {"location": "bin"}},\n  {"action": "report_status", "parameters": {"message": "Placed clutter in bin. Continuing cleanup..."}},\n  {"action": "scan_area", "parameters": {}},\n  {"action": "navigate_to", "parameters": {"location": "home"}}\n]\n```'}
        ]
        
        # In a real implementation, this would call the LLM API
        # For this example, return a pre-defined plan
        return [
            {"action": "scan_area", "parameters": {}},
            {"action": "patrol_area", "parameters": {"locations": ["table", "desk", "floor"]}},
            {"action": "detect_object", "parameters": {"object_type": "clutter"}},
            {"action": "navigate_to", "parameters": {"location": "object_location"}},
            {"action": "grasp_object", "parameters": {"object_type": "clutter"}},
            {"action": "navigate_to", "parameters": {"location": "bin"}},
            {"action": "place_object", "parameters": {"location": "bin"}},
            {"action": "report_status", "parameters": {"message": "Placed clutter in bin. Continuing cleanup..."}},
            {"action": "scan_area", "parameters": {}},
            {"action": "navigate_to", "parameters": {"location": "home"}},
            {"action": "report_status", "parameters": {"message": "Room cleaning completed!"}}
        ]
```

### Step 3: Visual scene understanding

```python
class CapstonePerceptionManager:
    def __init__(self, node):
        self.node = node
        
        # Isaac ROS perception nodes
        self.detection_subscription = node.create_subscription(
            Detection2DArray,
            '/isaac_ros_detectnet/detections',
            self.detection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.pc_subscription = node.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Store perceived objects
        self.perceived_objects = {}
        self.room_map = {}  # Locations of furniture, bins, etc.
        self.clutter_locations = []  # Locations of objects needing cleanup
        
    def detection_callback(self, msg):
        """Update perception of objects in the environment."""
        for detection in msg.detections:
            if detection.results:
                class_name = detection.results[0].hypothesis.name
                confidence = detection.results[0].hypothesis.score
                
                if confidence > 0.7:  # Confidence threshold
                    obj_info = {
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': detection.bbox,
                        'timestamp': self.node.get_clock().now()
                    }
                    
                    # Categorize objects for cleaning task
                    if self.is_clutter_object(class_name):
                        self.clutter_locations.append({
                            'object': obj_info,
                            'position': self.estimate_3d_position(detection.bbox) # Simplified
                        })
                    
                    self.perceived_objects[class_name] = obj_info
    
    def is_clutter_object(self, class_name):
        """Determine if an object type should be considered clutter for cleaning."""
        clutter_types = [
            'bottle', 'cup', 'book', 'paper', 'box', 'trash', 
            'object', 'item', 'thing'  # Generic terms
        ]
        return any(clutter_type in class_name.lower() for clutter_type in clutter_types)
    
    def estimate_3d_position(self, bbox):
        """Estimate 3D position from 2D bounding box using depth information."""
        # In a real implementation, this would use depth data
        # to estimate 3D position from 2D detection
        center_x = (bbox.center.x + bbox.size_x / 2)
        center_y = (bbox.center.y + bbox.size_y / 2)
        # Estimate depth from location in image and depth map
        return (center_x, center_y, 1.0)  # Simplified
```

### Step 4: Navigation to first object

```python
def execute_room_cleaning_mission(self):
    """Execute the complete room cleaning mission."""
    self.get_logger().info("Starting room cleaning mission")
    
    # First, scan the entire area to identify all objects needing cleanup
    self.publish_status("Scanning the room for items to clean")
    
    # Generate patrol path to visit likely locations of clutter
    patrol_locations = ["table_area", "desk_area", "floor_area", "corner_area"]
    
    objects_collected = 0
    max_objects = 10  # Don't clean forever
    
    for location in patrol_locations:
        if objects_collected >= max_objects:
            break
            
        # Navigate to the area
        nav_success = self.execute_navigate_to(location)
        if not nav_success:
            continue  # Try next location
        
        # Detect objects in this area
        self.publish_status(f"Looking for objects at {location}")
        
        # For each detected clutter object in this area
        area_objects = self.get_clutter_objects_at_location(location)
        for obj in area_objects:
            if objects_collected >= max_objects:
                break
                
            # Navigate to object
            obj_nav_success = self.execute_navigate_to_object(obj)
            if not obj_nav_success:
                continue
            
            # Grasp the object
            grasp_success = self.execute_grasp_object(obj)
            if not grasp_success:
                continue
            
            # Navigate to disposal location (bin)
            bin_nav_success = self.execute_navigate_to("bin")
            if not bin_nav_success:
                # If we can't reach bin, put object down where it's safe
                self.execute_place_object("safe_area")
                continue
            
            # Place object in bin
            place_success = self.execute_place_object("bin")
            if place_success:
                objects_collected += 1
                self.publish_status(f"Object {objects_collected} placed in bin")
    
    # Return to home position
    self.execute_navigate_to("home")
    self.publish_status(f"Misson completed! Collected {objects_collected} objects.")
```

### Step 5: Object detection and pose estimation

```python
def detect_and_approach_objects(self, area_name):
    """Detect objects in an area and approach them."""
    # Allow some time for detection after moving to area
    time.sleep(2.0)
    
    # Get current detections from perception system
    current_detections = self.get_recent_detections()
    
    objects_to_approach = []
    
    for detection in current_detections:
        if self.is_relevant_object(detection):
            # Estimate 3D pose from detection and depth
            estimated_pose = self.estimate_object_pose(detection)
            
            if estimated_pose and self.is_accessible_object(estimated_pose):
                objects_to_approach.append({
                    'detection': detection,
                    'pose': estimated_pose
                })
    
    return objects_to_approach

def estimate_object_pose(self, detection):
    """Estimate full 3D pose of an object from detection and depth."""
    # This would integrate 2D detection with depth information
    # to get 3D position and orientation
    bbox = detection.bbox
    depth_at_center = self.get_depth_at_point(bbox.center.x, bbox.center.y)
    
    if depth_at_center:
        # Convert 2D pixel coordinates + depth to 3D world coordinates
        x, y, z = self.pixel_to_world(
            bbox.center.x, 
            bbox.center.y, 
            depth_at_center
        )
        
        return {
            'position': (x, y, z),
            'orientation': (0, 0, 0, 1),  # Simplified: assume upright objects
            'bbox_3d': self.estimate_3d_bbox(detection, depth_at_center)
        }
    
    return None
```

### Step 6: Grasping (simplified or full manipulation)

```python
def execute_grasp_object(self, object_info):
    """Execute grasping of the specified object."""
    self.publish_status(f"Attempting to grasp {object_info['detection'].results[0].hypothesis.name}")
    
    # Move end effector to object position
    grasp_pose = self.calculate_grasp_pose(object_info['pose'])
    
    # Check if approach is safe
    if not self.verify_safe_approach(grasp_pose):
        self.publish_status("Safe approach not possible for this object")
        return False
    
    # Execute pre-grasp motion
    pregrasp_success = self.execute_motion_to_pregrasp(grasp_pose)
    if not pregrasp_success:
        return False
    
    # Execute grasp
    grasp_success = self.attempt_grasp_at_pose(grasp_pose)
    
    if grasp_success:
        self.currently_holding = object_info['detection'].results[0].hypothesis.name
        self.publish_status("Successfully grasped object")
        return True
    else:
        self.publish_status("Grasp failed, will try alternative approach")
        return False

def calculate_grasp_pose(self, object_pose):
    """Calculate appropriate grasp pose for an object."""
    # In a full implementation, this would use grasp planning algorithms
    # For this capstone, use simple heuristics based on object type
    
    obj_position = object_pose['position']
    obj_type = self.get_object_type_from_pose(object_pose)
    
    # Different approaches for different object types
    if obj_type in ['bottle', 'cup']:
        # Side grasp for containers
        grasp_pose = {
            'position': (obj_position[0], obj_position[1] - 0.1, obj_position[2] + 0.1),  # Approach from side
            'orientation': self.calculate_side_grasp_orientation()
        }
    else:
        # Top-down grasp for other objects
        grasp_pose = {
            'position': (obj_position[0], obj_position[1], obj_position[2] + 0.2),  # 20cm above object
            'orientation': self.calculate_top_down_orientation()
        }
    
    return grasp_pose
```

### Step 7: Navigate to bin

```python
def execute_navigate_to_bin(self):
    """Navigate to the designated bin location."""
    # In the capstone environment, the bin location is known
    bin_location = self.get_bin_location()
    
    if not bin_location:
        self.get_logger().error("Bin location not known")
        return False
    
    # Navigate to the bin
    goal = NavigateTo.Goal()
    goal.location = bin_location
    
    self.navigate_client.wait_for_server()
    future = self.navigate_client.send_goal_async(goal)
    
    rclpy.spin_until_future_complete(self, future)
    goal_handle = future.result()
    
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.success:
            self.current_location = bin_location
            self.publish_status("Successfully reached the bin")
            return True
        else:
            self.publish_status(f"Failed to reach bin: {result.message}")
            return False
    else:
        self.publish_status("Navigation to bin was rejected")
        return False
```

### Step 8: Place object

```python
def execute_place_object(self, location):
    """Execute placement of currently held object."""
    if not self.currently_holding:
        self.publish_status("Cannot place object, not holding anything")
        return False
    
    self.publish_status(f"Placing {self.currently_holding} at {location}")
    
    # Determine placement pose based on location
    placement_pose = self.calculate_placement_pose(location)
    
    # Move to placement position
    move_success = self.move_to_placement_pose(placement_pose)
    if not move_success:
        return False
    
    # Open gripper to release object
    release_success = self.open_gripper()
    if not release_success:
        return False
    
    # Move away slightly to avoid collision
    self.move_gripper_up()
    
    # Update internal state
    held_object = self.currently_holding
    self.currently_holding = None
    
    self.publish_status(f"Successfully placed {held_object} at {location}")
    return True

def calculate_placement_pose(self, location):
    """Calculate appropriate placement pose for location."""
    # For the bin, place at a safe position inside
    if location == "bin":
        # Use known bin dimensions and placement algorithm
        return {
            'position': (bin_center_x, bin_center_y, bin_base_height + 0.1),
            'orientation': (0, 0, 0, 1)  # Default orientation
        }
    elif location == "table":
        # Place at center of table surface
        return {
            'position': (table_center_x, table_center_y, table_height + 0.1),
            'orientation': (0, 0, 0, 1)
        }
    else:
        # Default: place at location coordinates
        return self.get_location_pose(location)
```

### Step 9: Repeat for remaining objects

```python
def continue_cleaning_until_complete(self):
    """Continue cleaning until no more objects are detected or limit reached."""
    max_cleaning_time = 300  # 5 minutes max cleaning time
    start_time = time.time()
    
    while (time.time() - start_time < max_cleaning_time and 
           self.get_remaining_clutter_count() > 0 and
           self.get_battery_level() > 0.2):  # Stop if battery low
        
        # Scan again to find remaining objects
        self.publish_status("Scanning for remaining objects...")
        remaining_objects = self.scan_for_remaining_objects()
        
        if not remaining_objects:
            self.publish_status("No more objects detected, cleaning complete")
            break
        
        # Process each remaining object
        for obj in remaining_objects:
            success = self.process_single_object(obj)
            if success:
                self.publish_status("Successfully processed one object")
    
    self.publish_status("Cleaning phase complete")
```

### Step 10: Final report via speech

```python
def report_completion(self):
    """Report mission completion via speech synthesis."""
    # Calculate statistics
    objects_cleaned = self.get_objects_cleaned_count()
    time_taken = time.time() - self.mission_start_time
    efficiency = objects_cleaned / (time_taken / 60.0) if time_taken > 0 else 0
    
    completion_report = f"Room cleaning mission completed. " \
                       f"I cleaned up {objects_cleaned} objects " \
                       f"in {time_taken:.1f} seconds. " \
                       f"My efficiency was {efficiency:.2f} objects per minute. " \
                       f"The room should be tidier now!"
    
    self.publish_status(completion_report)
    self.speech_synthesizer.speak(completion_report)
    
    # Return to home position
    self.execute_navigate_to("home_position")
    self.publish_status("Returned to home position")
```

## 21. Capstone: Testing and Validation

### Success criteria:

#### 1. Voice command recognized correctly
```python
class CapstoneTesting:
    def test_voice_command_recognition(self):
        """Test that voice commands are correctly recognized."""
        # Simulate voice commands
        test_commands = [
            "Clean the room",
            "Tidy up the space", 
            "Pick up the clutter"
        ]
        
        success_count = 0
        for command in test_commands:
            result = self.simulate_voice_input(command)
            if result == command.lower().strip():
                success_count += 1
        
        accuracy = success_count / len(test_commands)
        return accuracy >= 0.9  # 90% accuracy requirement
```

#### 2. LLM generates valid action plan
```python
def test_llm_planning_validity(self):
    """Test that LLM generates valid action sequences."""
    command = "Clean the room"
    action_plan = self.llm_planner.plan_cleaning_task(command)
    
    # Verify action plan is valid
    if not isinstance(action_plan, list):
        return False
    
    for action in action_plan:
        if not isinstance(action, dict):
            return False
        if 'action' not in action or 'parameters' not in action:
            return False
        if action['action'] not in ['navigate_to', 'detect_object', 'grasp_object', 'place_object', 'scan_area', 'report_status']:
            return False
    
    return True
```

#### 3. Robot navigates without collisions
```python
def test_navigation_safety(self):
    """Test that robot navigates without collisions."""
    # In simulation, check that robot doesn't collide with obstacles
    initial_safe_count = self.get_safe_navigation_count()
    
    # Execute navigation sequence
    self.execute_navigation_test_sequence()
    
    final_safe_count = self.get_safe_navigation_count()
    
    # Should have completed navigation without collisions
    return (final_safe_count - initial_safe_count) > 0
```

#### 4. Detects at least 3 objects
```python
def test_detection_capability(self):
    """Test that robot can detect multiple objects."""
    # Place known objects in environment
    known_objects = ["bottle", "cup", "book"]
    
    # Execute detection
    detected_objects = self.execute_detection_sequence()
    
    # Count how many of the known objects were detected
    detected_count = sum(1 for obj in known_objects if any(obj in det for det in detected_objects))
    
    return detected_count >= 3
```

#### 5. Successfully grasps 2+ objects
```python
def test_manipulation_capability(self):
    """Test that robot successfully grasps objects."""
    test_objects = ["bottle", "cup"]  # Objects that should be graspable
    
    successful_grasps = 0
    for obj in test_objects:
        if self.simulate_grasp_object(obj):
            successful_grasps += 1
    
    return successful_grasps >= 2
```

#### 6. Places objects in bin
```python
def test_placement_accuracy(self):
    """Test that robot places objects in correct location."""
    # Simulate picking up and placing objects in bin
    success_count = 0
    
    for i in range(3):
        # Grasp an object
        if self.simulate_grasp_object("test_object"):
            # Navigate to bin
            if self.execute_navigate_to("bin"):
                # Place object
                if self.execute_place_object("bin"):
                    # Verify object is now in bin location
                    if self.verify_object_in_location("test_object", "bin"):
                        success_count += 1
    
    return success_count >= 2
```

#### 7. Reports completion
```python
def test_completion_reporting(self):
    """Test that robot reports mission completion."""
    # Execute mission
    self.execute_room_cleaning_mission()
    
    # Check that completion was reported
    return self.was_completion_reported()
```

### Edge cases to test:

#### 1. Ambiguous commands
```python
def test_ambiguous_commands(self):
    """Test handling of ambiguous commands."""
    ambiguous_commands = [
        "Do something with that",  # Unclear reference
        "Clean over there",       # Vague location
        "Organize those things"   # Non-specific objects
    ]
    
    for cmd in ambiguous_commands:
        # System should request clarification
        response = self.process_command_with_llm(cmd)
        if "clarify" not in response.lower() and "where" not in response.lower():
            return False
    
    return True
```

#### 2. Object not found
```python
def test_object_not_found_handling(self):
    """Test handling when objects can't be found."""
    # In simulation, temporarily hide objects
    self.hide_objects_in_simulation()
    
    # Try to detect an object that isn't visible
    result = self.execute_detect_object("hidden_object")
    
    # System should handle this gracefully (not crash)
    return result in ["object_not_found", "search_extended", "request_assistance"]
```

#### 3. Path blocked
```python
def test_blocked_path_handling(self):
    """Test handling when navigation path is blocked."""
    # In simulation, add obstacle to path
    self.add_obstacle_to_path("kitchen_to_bin")
    
    # Try to navigate through blocked path
    result = self.execute_navigate_to("bin")
    
    # System should find alternative route or report issue
    return result in ["alternative_route", "navigation_failed", "request_assistance"]
```

#### 4. Grasp failure
```python
def test_grasp_failure_handling(self):
    """Test handling when grasp fails."""
    # Configure a grasp that will fail
    self.configure_difficult_grasp()
    
    # Try to grasp object
    success = self.execute_grasp_object("difficult_object")
    
    # Should handle failure gracefully
    if success:
        return True  # Sometimes grasps succeed
    else:
        # Check that robot doesn't crash and handles failure appropriately
        return self.assess_grasp_failure_handling()
```

### Performance metrics: task completion time, accuracy

```python
def benchmark_performance(self):
    """Benchmark performance metrics."""
    results = {}
    
    # Task completion time
    start_time = time.time()
    success = self.execute_room_cleaning_mission()
    completion_time = time.time() - start_time
    results['completion_time'] = completion_time
    
    # Accuracy of object placement
    results['placement_accuracy'] = self.calculate_placement_accuracy()
    
    # Energy efficiency (estimated from motion)
    results['energy_efficiency'] = self.calculate_energy_efficiency()
    
    # Success rate over multiple runs
    success_count = 0
    for i in range(5):
        if self.execute_room_cleaning_mission():
            success_count += 1
    results['success_rate'] = success_count / 5.0
    
    return results
```

## 22. Capstone: Code Structure

```
vla_capstone/
├── launch/
│   └── autonomous_humanoid.launch.py
├── config/
│   ├── llm_config.yaml
│   ├── actions.yaml
│   └── voice_config.yaml
├── src/
│   ├── voice_interface_node.py
│   ├── llm_planner_node.py
│   ├── vla_controller_node.py
│   ├── action_servers/
│   │   ├── navigate_server.py
│   │   ├── detect_server.py
│   │   ├── grasp_server.py
│   │   └── place_server.py
│   └── utils/
│       ├── prompt_templates.py
│       └── action_parser.py
└── README.md
```

### Complete implementation following this structure:

#### 1. Launch file:
```python
# launch/autonomous_humanoid.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        
        # Core VLA nodes
        Node(
            package='vla_capstone',
            executable='voice_interface_node',
            name='voice_interface',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        Node(
            package='vla_capstone', 
            executable='llm_planner_node',
            name='llm_planner',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        Node(
            package='vla_capstone',
            executable='vla_controller_node', 
            name='vla_controller',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Action servers
        Node(
            package='vla_capstone',
            executable='navigate_server',
            name='navigate_server',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        Node(
            package='vla_capstone',
            executable='detect_server',
            name='detect_server', 
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        Node(
            package='vla_capstone',
            executable='grasp_server',
            name='grasp_server',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        Node(
            package='vla_capstone', 
            executable='place_server',
            name='place_server',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
```

#### 2. Configuration files:

```yaml
# config/llm_config.yaml
llm_planner:
  ros__parameters:
    model_name: "gpt-4"
    api_key: ""  # In production, pass securely
    max_tokens: 500
    temperature: 0.1
    system_prompt: "You are a humanoid robot controller..."
```

```yaml
# config/actions.yaml
navigate_to_server:
  ros__parameters:
    default_max_time: 30.0
    collision_threshold: 0.5
    approach_distance: 0.3

detect_object_server:
  ros__parameters:
    detection_timeout: 10.0
    confidence_threshold: 0.7
    max_detection_range: 3.0

grasp_object_server:
  ros__parameters:
    grasp_timeout: 5.0
    force_threshold: 10.0
    approach_distance: 0.1
```

#### 3. Core nodes:

The core nodes have been detailed in previous sections, implementing the full VLA pipeline.

## 23. Real-World Deployment Considerations

### Sim-to-real transfer challenges

Moving from simulation to real robots presents several challenges:

1. **Visual appearance differences**:
   - Lighting conditions vary significantly between sim and real
   - Materials and textures render differently
   - Camera noise and characteristics differ

2. **Physics discrepancies**:
   - Friction coefficients may not match
   - Object weights and balances differ
   - Motor dynamics and control precision vary

3. **Sensor differences**:
   - Real sensors have noise, latency, and calibration issues
   - Field of view and quality differ from simulated sensors
   - Data rates and processing times vary

4. **Environmental factors**:
   - Real environments have dynamic changes
   - Objects move or are relocated
   - Humans and other agents interact with the environment

### Hardware requirements (Jetson Orin, microphones, speakers)

For real-world deployment, specific hardware considerations:

```python
class HardwareRequirements:
    def __init__(self):
        self.min_specs = {
            "compute": "NVIDIA Jetson Orin (64-core ARM CPU, 2048-core GPU)",
            "memory": "32GB LPDDR5 RAM",
            "storage": "128GB NVMe SSD",
            "audio": "USB microphone array, speakers with amplification",
            "network": "WiFi 6 with 5GHz support",
            "power": "200W power supply for full system"
        }
        
        self.sensors = {
            "visual": "RGB-D camera (Intel RealSense, Intel D400 series)",
            "depth": "Lidar (Ouster OS0, Velodyne Puck)",
            "audio": "High-quality microphone array for far-field speech recognition"
        }
```

### Network latency (LLM API calls)

When using cloud-based LLMs, network latency becomes a critical factor:

```python
class LatencyOptimization:
    def __init__(self):
        self.connection_threshold = 0.5  # 500ms max acceptable latency
        self.fallback_mechanisms = True  # Use local models when cloud unavailable
    
    def optimize_for_latency(self):
        """Implement optimizations for network latency."""
        # Preemptive planning when connection is good
        # Caching of common responses
        # Graceful degradation when connection is slow
        pass
```

### Edge deployment: on-device LLMs (LLaMA, Mistral)

For systems requiring low-latency or offline operation:

```python
class EdgeLLM:
    def __init__(self, model_name="llama-2-7b"):
        # Load local LLM model for edge deployment
        self.model = self.load_quantized_model(model_name)
    
    def load_quantized_model(self, model_name):
        """Load a quantized model for efficient edge inference."""
        # Implementation would load models like Llama.cpp
        pass
    
    def execute_local_planning(self, command):
        """Execute task planning using local LLM."""
        # Process command with local model
        pass
```

### Safety certifications for physical robots

Real-world robots must meet safety standards:

- **ISO 13482** for service robots
- **IEC 62565** for personal care robots
- **UL 3800** for service robots
- Local regulations for public spaces

### User privacy (voice data handling)

```python
class PrivacyManager:
    def __init__(self):
        self.encryption_enabled = True
        self_local_processing = True
        self_data_retention_policy = "No voice data retained after processing"
    
    def handle_voice_data(self, audio_data):
        """Process voice data with privacy in mind."""
        # Process locally when possible
        # Encrypt when transmission necessary
        # Anonymize where possible
        pass
```

## 24. Safety and Ethics in Autonomous Robots

### Asimov's Laws revisited

In modern robotics, Asimov's laws need updating for VLA systems:

1. **First Law (Revised)**: A robot may not harm humanity, or, through inaction, allow humanity to come to harm.
2. **Second Law (Revised)**: A robot must obey human commands except where such commands would conflict with the First Law.
3. **Third Law (Revised)**: A robot must protect its own existence as long as such protection does not conflict with the First or Second Laws.

For VLA systems, this translates to:
- **Language understanding**: The robot must correctly interpret commands to avoid harming humans
- **Intent verification**: Clarify potentially harmful requests
- **Context awareness**: Consider environment when executing commands

### Safety constraints in LLM prompts

```python
SAFETY_SYSTEM_PROMPT = """
You are a humanoid robot controller with built-in safety constraints:

1. NO HARM to humans in any form
2. NO VIOLATION of privacy in homes or public spaces
3. NO UNAUTHORIZED access to restricted areas
4. NO DAMAGING of property unless necessary for safety
5. ALWAYS ask for clarification on ambiguous commands
6. REPORT safety concerns immediately
7. STOP and request human intervention when uncertain

If a command violates these constraints, respond with a safe alternative and explain why the original command is problematic.
"""
```

### Human override mechanisms

Critical safety feature for autonomous robots:

```python
class HumanOverrideSystem:
    def __init__(self):
        self.emergency_stop_active = False
        self.override_requested = False
        self.override_callback = None
    
    def check_for_human_override(self):
        """Check for human override commands."""
        # Check for emergency stop signal
        if self.is_emergency_stop_triggered():
            self.activate_emergency_stop()
            return True
        
        # Check for pause command
        if self.is_pause_requested():
            self.pause_execution()
            return True
        
        # Check for direct command override
        if self.is_direct_command_received():
            self.execute_direct_command()
            return True
        
        return False
    
    def activate_emergency_stop(self):
        """Stop all robot motion immediately."""
        self.emergency_stop_active = True
        # Send stop commands to all motion controllers
        self.send_stop_commands()
```

### Fail-safe behaviors

```python
class FailSafeBehaviors:
    def __init__(self):
        self.safe_positions = {
            'home': (0.0, 0.0, 0.0),
            'charging_station': (1.0, 1.0, 0.0),
            'safe_zone': (5.0, 5.0, 0.0)
        }
    
    def execute_fail_safe(self):
        """Execute fail-safe behavior when critical error occurs."""
        # Stop all motion
        self.stop_all_actuators()
        
        # Close grippers (to safely hold or release objects)
        self.close_grippers_safely()
        
        # Navigate to safe position
        self.navigate_to_safe_position('home')
        
        # Report error and wait for human intervention
        self.report_error_and_wait()
```

### Privacy considerations (cameras, microphones)

```python
class PrivacyProtection:
    def __init__(self):
        self.camera_blur_enabled = True
        self_audio_encryption = True
        self_data_localization = True
    
    def protect_privacy(self):
        """Implement privacy protection measures."""
        # Anonymize faces in camera feed
        self.blur_faces_in_video()
        
        # Encrypt audio data locally
        self.encrypt_audio_data()
        
        # Minimize data collection
        self.only_record_when_necessary()
```

### Bias in LLMs affecting robot behavior

LLMs may exhibit biases that affect robot behavior:

- **Stereotypical assumptions**: LLMs might assume certain objects are in certain locations
- **Cultural biases**: Commands might be interpreted differently based on cultural context
- **Gender biases**: Interaction patterns might be biased

### Responsible AI principles

```python
class ResponsibleAI:
    def __init__(self):
        self.fairness_measures = True
        self.transparency_requirements = True
        self.accountability_framework = True
        self.inclusion_considerations = True
    
    def ensure_responsibility(self):
        """Implement responsible AI principles."""
        # Regular bias testing
        self.test_for_bias_regularly()
        
        # Transparency in decision making
        self.explain_decisions()
        
        # Human oversight
        self.maintain_human_in_the_loop()
```

### Regulatory compliance (FDA, CE, RoHS for commercial robots)

Commercial robotic systems must comply with various regulations:

- **FDA**: For robots in healthcare settings
- **CE**: For European market access
- **RoHS**: Restriction of Hazardous Substances
- **FCC**: For radio frequency emissions in the US
- **UL**: Safety certification in the US

## 25. Advanced Topics and Future Directions

### Multimodal LLMs (vision + language together)

Next-generation models that process vision and language jointly:

```python
class MultimodalVLA:
    def __init__(self):
        # Using models like GPT-4V, Claude with image understanding
        self.model = self.load_multimodal_model()
    
    def process_visual_language_command(self, image, command):
        """Process command that refers to specific visual elements."""
        # Instead of just "pick up the cup", now the system can handle:
        # "pick up the cup on the left" with visual context
        return self.model.generate_action_plan(image, command)
```

### Reinforcement learning from human feedback (RLHF)

Training systems to improve from human preferences:

```python
class RLHFSystem:
    def __init__(self):
        self.feedback_buffer = []
        self.rl_model = self.initialize_rl_model()
    
    def collect_human_feedback(self, action_taken, human_preference):
        """Collect human feedback on robot actions."""
        feedback = {
            'action': action_taken,
            'preference': human_preference,
            'context': self.get_current_context()
        }
        self.feedback_buffer.append(feedback)
    
    def update_policy_with_feedback(self):
        """Update robot policy based on human feedback."""
        if len(self.feedback_buffer) > 100:  # Batch update
            self.rl_model.train_from_feedback(self.feedback_buffer)
            self.feedback_buffer = []  # Clear buffer
```

### Continuous learning and adaptation

```python
class ContinuousLearning:
    def __init__(self):
        self.knowledge_base = {}
        self.experience_replay_buffer = []
    
    def learn_from_interaction(self, state, action, result):
        """Learn from each interaction with the environment."""
        experience = (state, action, result, self.get_context())
        self.experience_replay_buffer.append(experience)
        
        # Update internal models
        self.update_object_models(experience)
        self.update_navigation_models(experience)
        self.update_interaction_models(experience)
    
    def adapt_behavior_over_time(self):
        """Gradually adapt robot behavior based on accumulated experience."""
        if len(self.experience_replay_buffer) > 1000:
            # Retrain models with new experiences
            self.retrain_behavior_models()
```

### Multi-robot coordination via language

```python
class MultiRobotCoordination:
    def __init__(self, robot_id, total_robots):
        self.robot_id = robot_id
        self.total_robots = total_robots
        self.team_communication = TeamCommunicationChannel()
    
    def coordinate_with_team(self, task, location):
        """Coordinate with other robots using language-based communication."""
        # Request team for assistance
        request = f"Robot_{self.robot_id}: Need help with {task} at {location}"
        self.team_communication.broadcast(request)
        
        # Wait for responses and coordinate
        responses = self.team_communication.receive_responses()
        return self.compute_coordination_plan(responses)
```

### Embodied AI research frontiers (RT-X, Mobile ALOHA)

Current research directions in embodied AI:

- **RT-X (Robotics Transformer-X)**: Generalist robot policies
- **Mobile ALOHA**: Learning bimanual mobile manipulation
- **EmbodiedGPT**: LLM-guided embodied behavior
- **HuggingGPT**: Task planning with vision-language models

### Open-source VLA models (OpenVLA)

Emerging open-source alternatives to proprietary VLA systems:

- **OpenVLA**: Open Vision-Language-Action models
- **Octo**: Open-world robot manipulation
- **CLOPS**: Closed-loop vision-language-action policies

## 26. Optimization and Performance

### Reducing LLM latency (caching, batching)

```python
class PerformanceOptimizer:
    def __init__(self):
        self.response_cache = {}
        self.request_batcher = RequestBatcher()
    
    def get_cached_response(self, command):
        """Check for cached response to similar command."""
        cache_key = self.hash_command(command)
        if cache_key in self.response_cache:
            return self.response_cache[cache_key]
        return None
    
    def batch_planning_requests(self, commands):
        """Batch multiple planning requests to LLM."""
        return self.request_batcher.batch_and_process(commands)
```

### Local LLM deployment (Ollama, llama.cpp)

```python
class LocalLLM:
    def __init__(self):
        # Deploy LLM locally using Ollama or llama.cpp
        self.llm = self.setup_local_inference()
    
    def setup_local_inference(self):
        """Set up local LLM inference for low-latency responses."""
        # Implementation using llama.cpp or Ollama
        pass
```

### Action execution parallelization

```python
class ParallelActionExecutor:
    def __init__(self):
        self.executor_pool = ThreadPoolExecutor(max_workers=4)
    
    def execute_compatible_actions_in_parallel(self, action_list):
        """Execute actions that don't conflict with each other in parallel."""
        # Group actions that can run simultaneously
        compatible_groups = self.group_compatible_actions(action_list)
        
        # Execute each group in parallel
        for group in compatible_groups:
            futures = [self.executor_pool.submit(self.execute_single_action, action) for action in group]
            results = [future.result() for future in futures]
        
        return results
```

### Perception pipeline optimization

```python
class PerceptionPipelineOptimizer:
    def __init__(self):
        self.dynamic_resolution = True
        self.multi_task_learning = True
    
    def optimize_perception_pipeline(self):
        """Optimize perception pipeline for speed and accuracy."""
        # Use lower resolution when high detail isn't needed
        # Share feature representations between tasks
        # Use temporal consistency to reduce computation
        pass
```

### Memory management for long tasks

```python
class MemoryManager:
    def __init__(self):
        self.episodic_memory = EpisodicMemoryBuffer()
        self.working_memory = WorkingMemory()
        self.long_term_memory = LongTermKnowledgeBase()
    
    def manage_memory_during_long_tasks(self):
        """Manage memory during extended task execution."""
        # Compress episodic memory of completed actions
        # Keep working memory current with active task elements
        # Query long-term memory for general knowledge
        pass
```

### Real-time constraints handling

```python
class RealTimeConstraints:
    def __init__(self):
        self.deadlines = {}
        self.priority_scheduler = PriorityScheduler()
    
    def handle_real_time_constraints(self, deadline_requirements):
        """Ensure real-time constraints are met."""
        # Schedule actions with priority based on deadlines
        # Use real-time operating system features
        # Monitor and adjust for timing violations
        pass
```

## 27. Debugging and Troubleshooting

### LLM produces invalid actions → improve prompts

```python
class InvalidActionHandler:
    def __init__(self):
        self.action_validator = ActionValidator()
        self.prompt_improver = PromptImprover()
    
    def handle_invalid_action(self, invalid_action, original_command):
        """Handle when LLM produces an invalid action."""
        # Log the invalid action
        self.log_invalid_action(invalid_action, original_command)
        
        # Improve the prompt based on the error
        improved_prompt = self.prompt_improver.generate_better_prompt(
            original_command, invalid_action
        )
        
        # Regenerate action with improved prompt
        return self.generate_action_with_prompt(original_command, improved_prompt)
```

### Voice recognition fails → adjust Whisper parameters

```python
class VoiceRecognitionDebugger:
    def __init__(self):
        self.whisper_model = None
        self.audio_quality_analyzer = AudioQualityAnalyzer()
    
    def debug_voice_recognition(self, audio_input):
        """Debug and fix voice recognition issues."""
        # Analyze audio quality
        quality_report = self.audio_quality_analyzer.analyze(audio_input)
        
        if quality_report['noise_level'] > 0.5:
            # Apply noise reduction
            audio_input = self.apply_noise_reduction(audio_input)
        
        if quality_report['volume_level'] < 0.2:
            # Apply volume normalization
            audio_input = self.apply_volume_normalization(audio_input)
        
        return self.transcribe_with_adjusted_parameters(audio_input)
```

### Action server timeouts → increase timeouts

```python
class TimeoutManager:
    def __init__(self):
        self.default_timeout = 30.0
        self.adaptive_timeout = True
    
    def handle_action_timeout(self, action_type, params):
        """Handle action timeout with adaptive strategy."""
        if action_type == 'navigation':
            # Increase timeout based on distance
            distance = self.estimate_navigation_distance(params['location'])
            new_timeout = min(300.0, distance * 10.0)  # 10s per meter, max 5 minutes
            return self.execute_with_timeout(action_type, params, new_timeout)
        elif action_type == 'manipulation':
            # For grasping, try different approaches
            return self.attempt_alternative_grasps(params)
```

### Robot gets stuck → improve recovery behaviors

```python
class RecoveryBehaviorManager:
    def __init__(self):
        self.recovery_strategies = [
            self.clear_costmaps,
            self.force_localization,
            self.replan_with_different_parameters,
            self.request_human_assistance
        ]
    
    def handle_robot_stuck(self):
        """Handle when robot becomes stuck."""
        for strategy in self.recovery_strategies:
            try:
                if strategy():
                    return True  # Recovery successful
            except Exception as e:
                self.get_logger().warn(f"Recovery strategy failed: {e}")
                continue
        
        # If all strategies fail, request human help
        return self.request_human_assistance()
```

### API rate limits → implement queuing

```python
class APIRateLimitHandler:
    def __init__(self):
        self.request_queue = Queue()
        self.last_request_time = 0
        self.requests_per_minute = 60
    
    def handle_api_rate_limit(self, api_call):
        """Handle API rate limiting gracefully."""
        # Implement queueing to respect rate limits
        min_interval = 60.0 / self.requests_per_minute
        current_time = time.time()
        time_since_last = current_time - self.last_request_time
        
        if time_since_last < min_interval:
            # Sleep to respect rate limit
            time.sleep(min_interval - time_since_last)
        
        result = api_call()
        self.last_request_time = time.time()
        return result
```

### Simulation crashes → reduce scene complexity

```python
class SimulationStabilityManager:
    def __init__(self):
        self.scene_complexity_monitor = SceneComplexityMonitor()
    
    def handle_simulation_crash(self):
        """Handle simulation crashes by reducing complexity."""
        current_complexity = self.scene_complexity_monitor.get_current_level()
        
        if current_complexity > 'medium':
            # Reduce complexity
            self.reduce_scene_complexity()
            self.get_logger().info("Reduced scene complexity after crash")
        
        # Restart simulation
        return self.restart_simulation()
    
    def reduce_scene_complexity(self):
        """Reduce simulation complexity to improve stability."""
        self.remove_detailed_meshes()
        reduce_physics_accuracy()
        reduce_render_quality()
```

### Common integration issues

```python
class IntegrationDebugger:
    def __init__(self):
        self.connection_status = {}
        self.message_flow_analyzer = MessageFlowAnalyzer()
    
    def debug_integration_issues(self):
        """Debug common integration issues between components."""
        # Check all connections are alive
        for component, status in self.connection_status.items():
            if not status:
                self.attempt_reconnect(component)
        
        # Analyze message flow
        flow_report = self.message_flow_analyzer.check_flow()
        
        for issue in flow_report.issues:
            self.resolve_message_flow_issue(issue)
```

## 28. Testing Strategies

### Unit tests for individual components

```python
import unittest

class TestVoiceInterface(unittest.TestCase):
    def setUp(self):
        self.voice_interface = VoiceInterface()
    
    def test_audio_processing(self):
        # Test that audio is properly processed
        audio_sample = self.load_test_audio()
        result = self.voice_interface.process_audio(audio_sample)
        self.assertIsNotNone(result)
    
    def test_command_recognition(self):
        # Test that commands are properly recognized
        result = self.voice_interface.recognize_command("Clean the room")
        self.assertIn("clean", result.lower())

class TestActionServers(unittest.TestCase):
    def test_navigate_to_success(self):
        server = NavigateToServer()
        goal = NavigateTo.Goal()
        goal.location = "kitchen"
        
        # Mock the navigation system
        server.nav2_client = MockNav2Client(success=True)
        
        result = server.execute_callback(goal)
        self.assertTrue(result.success)

if __name__ == '__main__':
    unittest.main()
```

### Integration tests for action sequences

```python
class IntegrationTestVLASequence(unittest.TestCase):
    def setUp(self):
        # Set up mock environment for testing
        self.vla_system = MockVLASystem()
    
    def test_clean_room_sequence(self):
        """Test the complete 'clean the room' sequence."""
        # Send the command
        command = "Clean the room"
        result = self.vla_system.execute_command(command)
        
        # Verify the sequence of actions
        expected_actions = [
            'scan_area',
            'detect_object', 
            'navigate_to',
            'grasp_object',
            'navigate_to',  # to bin
            'place_object',
            'scan_area',    # check for more objects
            'navigate_to'   # to home
        ]
        
        self.assertEqual(result.action_sequence[:len(expected_actions)], expected_actions)
        self.assertTrue(result.success)
```

### End-to-end capstone validation

```python
class CapstoneMissionTest(unittest.TestCase):
    def test_complete_room_cleaning_mission(self):
        """Test the complete capstone mission."""
        # Set up the complete system
        capstone_system = CapstoneSystem()
        
        # Execute the mission
        success, metrics = capstone_system.execute_room_cleaning_mission(
            mission_command="Clean the room",
            max_time=300,  # 5 minutes
            success_threshold=0.8  # 80% success rate
        )
        
        # Validate success criteria
        self.assertTrue(success)
        self.assertGreaterEqual(metrics['objects_cleaned'], 3)
        self.assertLess(metrics['collision_count'], 1)
        self.assertGreaterEqual(metrics['completion_rate'], 0.8)
```

### Simulation vs hardware testing

```python
class SimulationVsHardwareValidation(unittest.TestCase):
    def setUp(self):
        self.simulation_system = VLASystem(simulated=True)
        self.hardware_system = VLASystem(simulated=False)
    
    def test_behavior_consistency(self):
        """Test that behavior is consistent between sim and hardware."""
        command = "Pick up red cup and place on table"
        
        sim_result = self.simulation_system.execute_command(command)
        hw_result = self.hardware_system.execute_command(command)
        
        # Compare task completion success
        self.assertEqual(sim_result.success, hw_result.success)
        
        # Compare high-level behavior patterns
        self.assertEqual(
            self.extract_behavior_pattern(sim_result), 
            self.extract_behavior_pattern(hw_result)
        )
```

### Edge case scenario testing

```python
class EdgeCaseTesting(unittest.TestCase):
    def test_blocked_navigation(self):
        """Test navigation when path is blocked."""
        # Set up simulation with blocked path
        self.setup_blocked_environment()
        
        # Execute navigation
        result = self.vla_system.execute_navigate_to("kitchen")
        
        # Should handle gracefully
        self.assertIn(result.status, ["alternative_path_found", "request_assistance"])
    
    def test_empty_room(self):
        """Test cleaning command in empty room."""
        # Set up empty room
        self.setup_empty_room()
        
        # Execute cleaning command
        result = self.vla_system.execute_command("Clean the room")
        
        # Should complete without errors
        self.assertTrue(result.success)
        self.assertEqual(result.objects_cleaned, 0)
```

### Performance benchmarking

```python
class PerformanceBenchmarking(unittest.TestCase):
    def test_command_response_time(self):
        """Benchmark command response time."""
        start_time = time.time()
        result = self.vla_system.execute_command("Navigate to kitchen")
        end_time = time.time()
        
        response_time = end_time - start_time
        self.assertLess(response_time, 10.0)  # Should respond in <10 seconds
    
    def test_concurrent_command_handling(self):
        """Test handling multiple concurrent commands."""
        with ThreadPoolExecutor(max_workers=3) as executor:
            futures = [
                executor.submit(self.vla_system.execute_command, "Clean the room"),
                executor.submit(self.vla_system.execute_command, "Go to the table"),
                executor.submit(self.vla_system.execute_command, "Find the cup")
            ]
            
            results = [future.result() for future in futures]
            
        # All should complete without interfering with each other
        for result in results:
            self.assertIsNotNone(result)
```

## 29. Common Issues and Solutions

### "Whisper not detecting voice" → check microphone permissions

```python
def troubleshoot_microphone_issues():
    """Common microphone troubleshooting steps."""
    # 1. Check if microphone is accessible
    import pyaudio
    audio = pyaudio.PyAudio()
    mic_count = audio.get_device_count()
    print(f"Available audio devices: {mic_count}")
    
    # 2. Test audio input
    for i in range(mic_count):
        dev_info = audio.get_device_info_by_index(i)
        if dev_info['maxInputChannels'] > 0:
            print(f"Input device {i}: {dev_info['name']}")
    
    # 3. Check for permission issues
    # On Linux: check pulseaudio permissions
    # On Windows: check privacy settings
    # On macOS: check microphone access in Security settings
```

### "LLM hallucinating actions" → refine system prompt

```python
def fix_llm_hallucinations():
    """Solutions for LLM hallucinations."""
    # 1. Use strict action validation
    ACTION_DEFINITIONS = {
        "valid_actions": [
            "navigate_to", "detect_object", "grasp_object", 
            "place_object", "scan_area", "report_status"
        ],
        "required_parameters": {
            "navigate_to": ["location"],
            "detect_object": ["object_type"],
            "grasp_object": ["object_type"]
        }
    }
    
    # 2. Add JSON schema enforcement
    # Force LLM to output valid JSON with predefined schema
    
    # 3. Implement response validation
    # Check all generated actions are valid before execution
```

### "Action server not responding" → verify ROS connections

```python
def troubleshoot_ros_connections():
    """Troubleshooting ROS 2 connections."""
    import subprocess
    
    # 1. Check if ROS domain is correct
    domain_id = os.environ.get('ROS_DOMAIN_ID', 0)
    print(f"Current ROS domain ID: {domain_id}")
    
    # 2. List active ROS nodes
    result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    print("Active ROS nodes:", result.stdout)
    
    # 3. Check action servers
    result = subprocess.run(['ros2', 'action', 'list'], capture_output=True, text=True)
    print("Active action servers:", result.stdout)
    
    # 4. Verify action types
    result = subprocess.run(['ros2', 'action', 'types'], capture_output=True, text=True)
    print("Available action types:", result.stdout)
```

### "Robot collides with objects" → tune Nav2 costmaps

```python
def fix_navigation_collisions():
    """Solutions for navigation collisions."""
    # 1. Increase costmap inflation
    costmap_config = {
        'inflation_layer': {
            'inflation_radius': 1.0,  # Increase from default
            'cost_scaling_factor': 5.0  # Increase cost scaling
        }
    }
    
    # 2. Improve obstacle detection
    # Ensure LiDAR and depth camera data feed into costmaps
    
    # 3. Adjust robot footprint
    robot_footprint = [
        [-0.5, -0.3], [0.5, -0.3],
        [0.5, 0.3], [-0.5, 0.3]
    ]  # Make more conservative
```

### "Out of API credits" → implement rate limiting

```python
def implement_api_cost_management():
    """Solutions for API cost management."""
    # 1. Implement request queuing
    class RequestQueue:
        def __init__(self, requests_per_minute=60):
            self.max_requests = requests_per_minute
            self.request_times = []
        
        def can_make_request(self):
            current_time = time.time()
            # Remove requests older than 1 minute
            self.request_times = [t for t in self.request_times if current_time - t < 60]
            return len(self.request_times) < self.max_requests
        
        def make_request(self):
            if self.can_make_request():
                self.request_times.append(time.time())
                return True
            return False
    
    # 2. Add caching for common queries
    # 3. Use local alternatives when possible
```

### "Slow response time" → optimize perception pipeline

```python
def optimize_perception_pipeline():
    """Optimize perception for better response time."""
    # 1. Use lower resolution for initial detection
    detection_config = {
        'detection_resolution': '640x480',  # Lower than full-res
        'confidence_threshold': 0.8,       # Higher threshold to reduce false positives
        'max_detection_range': 2.0         # Limit range for faster processing
    }
    
    # 2. Implement temporal consistency
    # Use tracking to avoid re-detecting the same objects
```

## 30. Summary and Course Conclusion

### What you've learned across all 4 modules:

#### Module 1: ROS 2 foundations
- ROS 2 communication patterns (topics, services, actions)
- Robot control with node-based architecture
- URDF for robot description and simulation
- Launch files and parameter management

#### Module 2: Physics simulation mastery
- Gazebo simulation environments and physics
- Sensor integration (LiDAR, cameras, IMU)
- Unity for immersive robot environments
- Simulation-to-reality transfer principles

#### Module 3: Advanced perception with Isaac
- Isaac Sim for photorealistic simulation
- GPU-accelerated perception with Isaac ROS
- Visual SLAM for localization
- Nav2 navigation stack configuration
- Synthetic data generation for training

#### Module 4: Language-based robot control
- Voice command recognition with Whisper
- LLM integration for task planning
- Vision-Language-Action pipeline
- Humanoid manipulation and interaction
- Capstone: Autonomous humanoid mission

### The complete stack: from voice to physical action

The VLA system creates an end-to-end pipeline:

1. **Voice Input**: Natural language command
2. **Speech Recognition**: Whisper converts speech to text
3. **Language Understanding**: LLM decomposes command into action sequence
4. **Perception**: Detect and locate objects in environment
5. **Planning**: Determine navigation paths and manipulation plans
6. **Execution**: Execute actions through robotic control
7. **Feedback**: Report status and handle errors
8. **Human Interaction**: Continuous loop with user feedback

### Skills acquired: full-stack robotics engineer

By completing all modules, you've developed:

1. **System Architecture**: Understanding end-to-end robotics systems
2. **Multi-Modal Integration**: Combining vision, language, and action
3. **Real-time Control**: Managing concurrent perception and action
4. **Safety and Ethics**: Implementing responsible AI practices
5. **Simulation and Deployment**: Moving from simulation to real systems
6. **Troubleshooting**: Diagnosing and fixing complex system issues

### Next steps:

#### Build your own robot projects
- Enhance the autonomous humanoid with new capabilities
- Create domain-specific applications (healthcare, industrial, domestic)
- Extend the VLA pipeline with additional sensors or modalities

#### Contribute to open-source robotics
- Contribute to ROS 2 packages
- Enhance Isaac Sim examples
- Develop new perception algorithms
- Improve VLA research implementations

#### Explore research opportunities
- Multimodal learning for robots
- Safe human-robot interaction
- Long-horizon task planning
- Embodied AI and cognition

#### Join robotics competitions
- RoboCup@Home challenges
- Amazon Picking Challenge
- NASA Space Robotics Challenge
- European Rover Challenge

#### Commercial applications
- Service robotics implementations
- Industrial automation solutions
- Research and development roles
- Robotics system integration

### Resources for continued learning

1. **Documentation**:
   - ROS 2 Documentation: docs.ros.org
   - Isaac Sim Documentation: docs.omniverse.nvidia.com
   - Nav2 Documentation: navigation.ros.org

2. **Communities**:
   - ROS Discourse: discourse.ros.org
   - Isaac Sim Forums: forums.developer.nvidia.com
   - Robotics Stack Exchange: robotics.stackexchange.com

3. **Research**:
   - Robotics journals: IJRR, T-RO, RA-L
   - Conferences: ICRA, IROS, RSS
   - Preprint servers: arXiv.org (cs.RO, cs.AI)

### Community and support channels

- ROS Answers for technical questions
- NVIDIA Developer Forums for Isaac platforms
- GitHub repositories for issue tracking
- Local robotics meetups and hackathons
- Academic research groups and labs

## 31. Final Project Showcase Ideas

### Personal assistant robot
- Voice-controlled home automation
- Object retrieval and delivery
- Calendar and reminder integration
- Person following and recognition

### Warehouse automation
- Inventory management through voice commands
- Automated picking and placing
- Real-time tracking and reporting
- Collaborative operations with humans

### Elder care assistant
- Medication reminders and delivery
- Fall detection and alert systems
- Social interaction and engagement
- Health monitoring and reporting

### Educational robot tutor
- Interactive learning experiences
- Multi-language support
- Adaptive teaching methods
- Progress tracking and reporting

### Restaurant service robot
- Order taking and delivery
- Table clearing and cleaning
- Customer interaction and feedback
- Inventory management

### Agricultural robot
- Crop monitoring and maintenance
- Harvesting based on voice commands
- Field navigation and mapping
- Quality assessment and sorting

### Search and rescue robot
- Voice-commanded exploration
- Victim detection and localization
- Hazard identification and reporting
- Communication relay in emergencies

## 32. Appendix: API Keys and Security

### Environment variables best practices

```bash
# .env file (not committed to Git)
OPENAI_API_KEY=your_actual_api_key_here
ANTHROPIC_API_KEY=your_anthropic_key
ROS_DOMAIN_ID=42
```

### .env file setup

```python
# Install python-dotenv
pip install python-dotenv

# In your code
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

# Access API keys
openai_api_key = os.getenv("OPENAI_API_KEY")
```

### Never commit API keys to Git

```bash
# .gitignore file
*.env
.env.local
.env.production
*.key
*.pem
```

### Using ROS parameters for secrets

```python
class SecureLLMPlanner(Node):
    def __init__(self):
        super().__init__('secure_llm_planner')
        
        # Declare parameters but don't provide defaults for secrets
        self.declare_parameter('llm_api_key', '')
        self.declare_parameter('llm_model_name', 'gpt-4')
        
        # Get the API key from launch file or environment
        self.api_key = self.get_parameter('llm_api_key').value
        if not self.api_key:
            self.get_logger().error("LLM API key not provided!")
```

### API key rotation

```python
class APIKeyManager:
    def __init__(self):
        self.current_key = None
        self.key_rotation_schedule = 30  # days
    
    def should_rotate_key(self):
        """Check if it's time to rotate the API key."""
        return self.days_since_last_rotation() >= self.key_rotation_schedule
    
    def rotate_key(self):
        """Rotate to a new API key."""
        # Implementation to get new key from secure storage
        pass
```

### Cost monitoring and alerts

```python
class APICostMonitor:
    def __init__(self, budget_limit=100.0):  # Monthly budget in USD
        self.budget_limit = budget_limit
        self.monthly_cost = 0.0
    
    def track_usage(self, operation, cost):
        """Track API usage and costs."""
        self.monthly_cost += cost
        
        if self.monthly_cost > self.budget_limit * 0.9:  # 90% of budget
            self.send_budget_warning()
        
        if self.monthly_cost > self.budget_limit:
            self.send_budget_exceeded_alert()
    
    def send_budget_warning(self):
        """Send warning when approaching budget limit."""
        print(f"WARNING: API usage approaching budget limit. Current: ${self.monthly_cost:.2f}")
```

This comprehensive module covers all aspects of Vision-Language-Action systems for humanoid robots, from basic voice recognition through complex autonomous missions. The implementation integrates all previous modules (ROS 2, simulation, perception) into a cohesive system that can understand natural language commands and execute them as physical robot actions.

The content includes practical code examples, testing strategies, troubleshooting guides, and ethical considerations necessary for building safe and effective autonomous humanoid robots. The capstone project ties together all concepts in a realistic "clean the room" scenario that demonstrates the complete VLA pipeline in action.