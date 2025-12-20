---
sidebar_position: 18
---

# Module 4: Vision-Language-Action (VLA)

## The Convergence of LLMs and Robotics

**Vision-Language-Action (VLA)** models represent the cutting edge of robotic AI, combining:
- 👁️ **Computer Vision** - Understanding visual scenes
- 💬 **Natural Language** - Processing human instructions
- 🤖 **Action Generation** - Executing robot behaviors

This convergence enables robots to understand commands like "Clean the room" and translate them into complex action sequences.

---

## Architecture of VLA Systems

### Traditional vs. VLA Pipeline

**Traditional Approach:**
```
Voice Input → Speech Recognition → Intent Classification
→ Rule-Based Planner → Hard-Coded Actions → Robot Control
```

**VLA Approach:**
```
Voice Input → Whisper (Speech-to-Text)
→ Vision-Language Model (GPT-4V, Gemini, RT-2)
→ Action Tokens → Robot Control
```

### System Architecture

```
┌─────────────────────────────────────────┐
│         Voice Interface                 │
│  ┌──────────────────────────────────┐   │
│  │  OpenAI Whisper                  │   │
│  │  (Speech Recognition)            │   │
│  └────────────┬─────────────────────┘   │
└───────────────┼─────────────────────────┘
                │
┌───────────────▼─────────────────────────┐
│    Vision-Language Model                │
│  ┌──────────────────────────────────┐   │
│  │  Visual Encoder                  │   │
│  │  (CLIP, DINOv2)                  │   │
│  └────────────┬─────────────────────┘   │
│               │                          │
│  ┌────────────▼─────────────────────┐   │
│  │  Language Model                  │   │
│  │  (GPT-4, Gemini, PaLM)           │   │
│  └────────────┬─────────────────────┘   │
│               │                          │
│  ┌────────────▼─────────────────────┐   │
│  │  Action Decoder                  │   │
│  │  (Robot Action Tokens)           │   │
│  └────────────┬─────────────────────┘   │
└───────────────┼─────────────────────────┘
                │
┌───────────────▼─────────────────────────┐
│      Robot Controller (ROS 2)           │
│  ┌──────────────────────────────────┐   │
│  │  Motion Planning                 │   │
│  │  Manipulation Control            │   │
│  │  Navigation                      │   │
│  └──────────────────────────────────┘   │
└─────────────────────────────────────────┘
```

---

## Voice-to-Action with OpenAI Whisper

### What is Whisper?

**OpenAI Whisper** is a robust speech recognition model trained on 680,000 hours of multilingual audio data.

**Key Features:**
- ✅ 99 languages supported
- ✅ Robust to accents and noise
- ✅ Real-time capable
- ✅ Multiple model sizes (tiny → large)
- ✅ On-device deployment possible

### Installation and Setup

```bash
# Install Whisper
pip install openai-whisper

# Or for faster inference with GPU
pip install faster-whisper

# Install ROS 2 audio capture
sudo apt-get install ros-humble-audio-common
```

### Basic Whisper Integration

```python
import whisper
import pyaudio
import numpy as np
import wave

class WhisperVoiceInterface:
    def __init__(self, model_size='base'):
        """Initialize Whisper model"""
        # Load Whisper model
        self.model = whisper.load_model(model_size)

        # Audio parameters
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 1024
        self.RECORD_SECONDS = 3

        self.audio = pyaudio.PyAudio()

    def record_audio(self):
        """Record audio from microphone"""
        stream = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )

        print("🎤 Listening...")
        frames = []

        for _ in range(0, int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
            data = stream.read(self.CHUNK)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        # Convert to numpy array
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0

        return audio_data

    def transcribe(self, audio_data):
        """Transcribe audio to text"""
        result = self.model.transcribe(
            audio_data,
            language='en',
            task='transcribe'
        )

        return result['text']

    def listen_and_transcribe(self):
        """Record and transcribe voice command"""
        audio_data = self.record_audio()
        text = self.transcribe(audio_data)
        print(f"📝 Transcription: {text}")
        return text

# Usage
voice_interface = WhisperVoiceInterface(model_size='base')
command = voice_interface.listen_and_transcribe()
```

### ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import whisper
import numpy as np

class WhisperROS2Node(Node):
    def __init__(self):
        super().__init__('whisper_voice_recognition')

        # Load Whisper model
        self.model = whisper.load_model('base')

        # Publishers
        self.command_pub = self.create_publisher(
            String,
            '/voice_command',
            10
        )

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/audio',
            self.audio_callback,
            10
        )

        # Buffer for audio
        self.audio_buffer = []
        self.buffer_duration = 3.0  # seconds
        self.sample_rate = 16000

    def audio_callback(self, msg):
        """Accumulate audio data"""
        self.audio_buffer.extend(msg.data)

        # Check if buffer is full
        expected_samples = int(self.buffer_duration * self.sample_rate)
        if len(self.audio_buffer) >= expected_samples:
            self.process_audio()
            self.audio_buffer = []

    def process_audio(self):
        """Transcribe accumulated audio"""
        # Convert to numpy array
        audio_data = np.array(self.audio_buffer, dtype=np.float32)
        audio_data = audio_data / 32768.0  # Normalize

        # Transcribe
        result = self.model.transcribe(audio_data, language='en')
        text = result['text'].strip()

        if text:
            self.get_logger().info(f'Voice command: {text}')

            # Publish command
            msg = String()
            msg.data = text
            self.command_pub.publish(msg)

def main():
    rclpy.init()
    node = WhisperROS2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Cognitive Planning with LLMs

### From Natural Language to Robot Actions

LLMs can decompose high-level instructions into executable robot actions.

### Task Decomposition Pipeline

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from openai import OpenAI
import json

class LLMTaskPlanner(Node):
    def __init__(self):
        super().__init__('llm_task_planner')

        # OpenAI client
        self.llm = OpenAI(api_key="your-api-key")

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )

        # Publishers for robot actions
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_pub = self.create_publisher(String, '/robot_action', 10)

        # System prompt for task planning
        self.system_prompt = """You are a robot task planner. Convert natural language commands into a sequence of robot actions.

Available robot actions:
1. navigate(x, y, theta) - Move to location
2. pick_object(object_name) - Pick up an object
3. place_object(location) - Place held object
4. open_gripper() - Open gripper
5. close_gripper() - Close gripper
6. rotate(angle) - Rotate in place
7. wait(seconds) - Wait for duration

Return ONLY valid JSON:
{
  "task": "original command",
  "reasoning": "brief explanation",
  "actions": [
    {"action": "navigate", "params": {"x": 1.0, "y": 0.5, "theta": 0.0}},
    {"action": "pick_object", "params": {"object_name": "cup"}}
  ]
}"""

    def command_callback(self, msg):
        """Process voice command"""
        command = msg.data
        self.get_logger().info(f'Planning for: {command}')

        # Get action plan from LLM
        plan = self.plan_task(command)

        if plan:
            # Execute plan
            self.execute_plan(plan)

    def plan_task(self, command):
        """Use LLM to create action plan"""
        try:
            response = self.llm.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=0.3,
                max_tokens=500
            )

            plan_json = response.choices[0].message.content
            plan = json.loads(plan_json)

            self.get_logger().info(f'Plan: {plan["reasoning"]}')
            return plan

        except Exception as e:
            self.get_logger().error(f'Planning failed: {str(e)}')
            return None

    def execute_plan(self, plan):
        """Execute action sequence"""
        for action_item in plan['actions']:
            action = action_item['action']
            params = action_item['params']

            self.get_logger().info(f'Executing: {action} with {params}')

            if action == 'navigate':
                self.navigate(params['x'], params['y'], params['theta'])
            elif action == 'pick_object':
                self.pick_object(params['object_name'])
            elif action == 'place_object':
                self.place_object(params['location'])
            elif action == 'rotate':
                self.rotate(params['angle'])
            elif action == 'wait':
                self.wait(params['seconds'])
            # Add more actions...

    def navigate(self, x, y, theta):
        """Navigate to position"""
        # Implementation with Nav2
        self.get_logger().info(f'Navigating to ({x}, {y}, {theta})')
        # Call Nav2 action server...

    def pick_object(self, object_name):
        """Pick up object"""
        self.get_logger().info(f'Picking up: {object_name}')
        # Implementation with MoveIt or manipulation controller...

    def place_object(self, location):
        """Place object at location"""
        self.get_logger().info(f'Placing object at: {location}')
        # Implementation...

    def rotate(self, angle):
        """Rotate robot"""
        twist = Twist()
        twist.angular.z = angle
        self.cmd_vel_pub.publish(twist)

    def wait(self, seconds):
        """Wait for duration"""
        import time
        time.sleep(seconds)

def main():
    rclpy.init()
    node = LLMTaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Example Interactions

```python
# Example 1: Simple navigation
User: "Go to the kitchen"
LLM Plan: {
  "actions": [
    {"action": "navigate", "params": {"x": 5.0, "y": 2.0, "theta": 1.57}}
  ]
}

# Example 2: Object manipulation
User: "Pick up the red cup on the table"
LLM Plan: {
  "actions": [
    {"action": "navigate", "params": {"x": 2.0, "y": 1.0, "theta": 0.0}},
    {"action": "open_gripper", "params": {}},
    {"action": "pick_object", "params": {"object_name": "red_cup"}},
    {"action": "close_gripper", "params": {}}
  ]
}

# Example 3: Complex task
User: "Clean the room"
LLM Plan: {
  "reasoning": "Cleaning involves identifying objects, picking them up, and placing them in appropriate locations",
  "actions": [
    {"action": "navigate", "params": {"x": 0.0, "y": 0.0, "theta": 0.0}},
    {"action": "scan_environment", "params": {}},
    {"action": "identify_objects", "params": {"category": "clutter"}},
    {"action": "pick_object", "params": {"object_name": "detected_object_1"}},
    {"action": "navigate", "params": {"x": 3.0, "y": 1.0, "theta": 0.0}},
    {"action": "place_object", "params": {"location": "trash_bin"}},
    # Repeat for other objects...
  ]
}
```

---

## Vision-Language Models for Robotics

### RT-2: Vision-Language-Action Model

**RT-2** (Robotic Transformer 2) directly maps visual observations and language instructions to robot actions.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from cv_bridge import CvBridge
import torch
from transformers import RT2Model, RT2Processor

class RT2RobotController(Node):
    def __init__(self):
        super().__init__('rt2_controller')

        # Load RT-2 model (example - not actual API)
        self.processor = RT2Processor.from_pretrained("google/rt-2-base")
        self.model = RT2Model.from_pretrained("google/rt-2-base")
        self.model.eval()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/language_instruction',
            self.command_callback,
            10
        )

        # Publishers
        self.action_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        self.bridge = CvBridge()
        self.current_image = None
        self.current_instruction = None

    def image_callback(self, msg):
        """Receive camera image"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def command_callback(self, msg):
        """Receive language instruction"""
        self.current_instruction = msg.data
        self.generate_action()

    def generate_action(self):
        """Generate robot action from vision and language"""
        if self.current_image is None or self.current_instruction is None:
            return

        # Prepare inputs
        inputs = self.processor(
            text=self.current_instruction,
            images=self.current_image,
            return_tensors="pt"
        )

        # Generate action tokens
        with torch.no_grad():
            outputs = self.model(**inputs)
            action_tokens = outputs.action_tokens

        # Decode to robot actions
        robot_actions = self.decode_actions(action_tokens)

        # Execute actions
        self.execute_actions(robot_actions)

    def decode_actions(self, action_tokens):
        """Convert action tokens to robot commands"""
        # Decode discretized actions
        actions = {
            'gripper_position': action_tokens[0],
            'arm_joints': action_tokens[1:8],
            'base_velocity': action_tokens[8:10]
        }
        return actions

    def execute_actions(self, actions):
        """Execute robot actions"""
        # Convert to joint trajectory
        trajectory = JointTrajectory()
        # ... populate trajectory
        self.action_pub.publish(trajectory)
```

### Vision-Language Grounding

```python
from openai import OpenAI
import base64
import cv2

class VisionLanguageGrounding(Node):
    def __init__(self):
        super().__init__('vision_language_grounding')

        self.llm = OpenAI(api_key="your-api-key")

        # Subscribe to camera and commands
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10)

        self.bridge = CvBridge()
        self.current_image = None

    def image_callback(self, msg):
        """Store current image"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def command_callback(self, msg):
        """Process command with visual context"""
        command = msg.data

        if self.current_image is None:
            return

        # Encode image
        _, buffer = cv2.imencode('.jpg', self.current_image)
        image_base64 = base64.b64encode(buffer).decode('utf-8')

        # Send to GPT-4V
        response = self.llm.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": f"""You are a robot viewing this scene.
                            User command: "{command}"

                            Describe:
                            1. What objects do you see?
                            2. Where are they located?
                            3. What actions should you take?
                            4. Return JSON with object locations and actions."""
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_base64}"
                            }
                        }
                    ]
                }
            ],
            max_tokens=500
        )

        analysis = response.choices[0].message.content
        self.get_logger().info(f'Scene analysis: {analysis}')

        # Parse and execute
        self.execute_from_analysis(analysis)

    def execute_from_analysis(self, analysis):
        """Execute based on visual analysis"""
        # Parse JSON response and execute actions
        pass
```

---

## Multi-Modal Reasoning

### Combining Speech, Vision, and Context

```python
class MultiModalRobotController(Node):
    def __init__(self):
        super().__init__('multimodal_controller')

        # Components
        self.whisper = WhisperVoiceInterface()
        self.vision_model = VisionLanguageGrounding()
        self.llm_planner = LLMTaskPlanner()

        # State
        self.robot_state = {
            'position': (0, 0, 0),
            'holding_object': None,
            'battery_level': 100,
            'recent_actions': []
        }

        # Timer for continuous operation
        self.timer = self.create_timer(1.0, self.main_loop)

    def main_loop(self):
        """Main control loop"""
        # 1. Listen for voice command
        command = self.whisper.listen_and_transcribe()

        if not command:
            return

        # 2. Get visual context
        visual_context = self.vision_model.analyze_scene()

        # 3. Consider robot state
        state_description = f"""
        Robot Status:
        - Position: {self.robot_state['position']}
        - Holding: {self.robot_state['holding_object']}
        - Battery: {self.robot_state['battery_level']}%
        - Recent actions: {self.robot_state['recent_actions'][-3:]}

        Visual Context:
        {visual_context}

        User Command: {command}

        Generate appropriate action plan considering current state and context.
        """

        # 4. Generate context-aware plan
        plan = self.llm_planner.plan_task(state_description)

        # 5. Execute plan
        if plan:
            self.execute_with_monitoring(plan)

    def execute_with_monitoring(self, plan):
        """Execute plan with real-time monitoring"""
        for action in plan['actions']:
            # Check if action still valid
            if not self.validate_action(action):
                self.get_logger().warn('Action invalid, replanning...')
                return

            # Execute
            self.execute_single_action(action)

            # Update state
            self.update_robot_state(action)

    def validate_action(self, action):
        """Check if action is still valid given current state"""
        # Check battery, object presence, safety, etc.
        return True

    def update_robot_state(self, action):
        """Update internal state after action"""
        self.robot_state['recent_actions'].append(action)
        # Update other state variables...
```

---

## Advanced Features

### Continuous Learning from Feedback

```python
class LearningRobotController(Node):
    def __init__(self):
        super().__init__('learning_controller')

        # Store interaction history
        self.interaction_history = []

    def execute_with_feedback(self, command, plan):
        """Execute and learn from results"""
        # Execute plan
        success = self.execute_plan(plan)

        # Get user feedback
        feedback = self.get_user_feedback()

        # Store experience
        experience = {
            'command': command,
            'plan': plan,
            'success': success,
            'feedback': feedback,
            'timestamp': self.get_clock().now()
        }
        self.interaction_history.append(experience)

        # Update LLM context with examples
        if len(self.interaction_history) > 10:
            self.update_llm_context()

    def get_user_feedback(self):
        """Ask user for feedback"""
        # Voice: "Was that correct?"
        # Wait for response: "Yes" or "No, you should..."
        pass

    def update_llm_context(self):
        """Update LLM with successful examples"""
        successful_examples = [
            exp for exp in self.interaction_history
            if exp['success'] and exp['feedback'] == 'positive'
        ]

        # Add to system prompt as few-shot examples
        examples_text = "\n".join([
            f"User: {exp['command']}\nSuccessful Plan: {exp['plan']}"
            for exp in successful_examples[-5:]
        ])

        self.system_prompt += f"\n\nSuccessful Examples:\n{examples_text}"
```

### Error Handling and Recovery

```python
class RobustRobotController(Node):
    def execute_with_recovery(self, plan):
        """Execute with automatic error recovery"""
        for i, action in enumerate(plan['actions']):
            try:
                # Execute action
                self.execute_single_action(action)

            except ActionFailedException as e:
                self.get_logger().error(f'Action failed: {e}')

                # Ask LLM for recovery plan
                recovery_plan = self.generate_recovery_plan(
                    original_plan=plan,
                    failed_action=action,
                    error=str(e)
                )

                if recovery_plan:
                    # Try recovery
                    self.execute_with_recovery(recovery_plan)
                else:
                    # Ask human for help
                    self.request_human_intervention()

    def generate_recovery_plan(self, original_plan, failed_action, error):
        """Generate recovery strategy"""
        prompt = f"""
        Original plan failed at action: {failed_action}
        Error: {error}

        Original plan: {original_plan}

        Generate a recovery plan to continue the task or safely abort.
        """

        # Query LLM for recovery strategy
        response = self.llm.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": prompt}
            ]
        )

        return json.loads(response.choices[0].message.content)
```

---

## Real-World Deployment

### Complete Voice-Controlled Robot

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import whisper
from openai import OpenAI
import pyaudio
import numpy as np

class VoiceControlledRobot(Node):
    def __init__(self):
        super().__init__('voice_controlled_robot')

        # Initialize components
        self.whisper_model = whisper.load_model('base')
        self.openai_client = OpenAI()

        # Audio setup
        self.audio = pyaudio.PyAudio()

        # ROS publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Start listening
        self.get_logger().info('🎤 Voice-controlled robot ready!')
        self.listen_loop()

    def listen_loop(self):
        """Continuous listening loop"""
        while rclpy.ok():
            # Record audio
            audio_data = self.record_audio()

            # Transcribe
            text = self.whisper_model.transcribe(audio_data)['text']

            if text.strip():
                self.get_logger().info(f'📝 Heard: {text}')

                # Process command
                self.process_command(text)

    def process_command(self, command):
        """Process voice command"""
        # Use LLM to generate action
        response = self.openai_client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "Convert command to robot action JSON"},
                {"role": "user", "content": command}
            ]
        )

        action = json.loads(response.choices[0].message.content)
        self.execute_action(action)

def main():
    rclpy.init()
    robot = VoiceControlledRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Conclusion

Vision-Language-Action systems represent the future of human-robot interaction, enabling:

- 🎤 **Natural voice control** with Whisper
- 🧠 **Intelligent planning** with LLMs
- 👁️ **Visual understanding** with vision-language models
- 🤖 **Seamless execution** integrated with ROS 2

By combining these technologies, robots can understand complex instructions, perceive their environment, and execute sophisticated behaviors—all from natural language commands.

**Key Takeaways:**
1. Whisper enables robust speech recognition
2. LLMs provide task planning and reasoning
3. Vision-language models ground language in visual perception
4. ROS 2 integration connects AI to physical robot control
5. Multi-modal fusion enables context-aware robotics

---

## Resources

### Documentation
- [OpenAI Whisper](https://github.com/openai/whisper)
- [OpenAI API](https://platform.openai.com/docs)
- [RT-2 Paper](https://robotics-transformer2.github.io/)
- [PaLM-E](https://palm-e.github.io/)

### Community
- Robotics AI Discord
- r/robotics Reddit
- ROS Discourse

---

<div style={{textAlign: 'center', padding: '2rem', background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)', borderRadius: '12px', color: 'white', marginTop: '2rem'}}>
  <h3 style={{color: 'white'}}>🎉 You've Completed All Modules!</h3>
  <p>You now have the knowledge to build voice-controlled, AI-powered humanoid robots</p>
  <a href="/chapter1" style={{color: 'white', textDecoration: 'underline'}}>← Back to Chapter 1</a>
  {' | '}
  <a href="/claude" style={{color: 'white', textDecoration: 'underline'}}>Learn about Claude Code →</a>
</div>
