---
sidebar_position: 2
title: 4.1 Voice-to-Action with OpenAI Whisper
---

# Chapter 4.1: Voice-to-Action with OpenAI Whisper

Voice commands enable natural human-robot interaction. OpenAI Whisper provides state-of-the-art speech recognition that converts spoken language to text—the first step in the VLA pipeline. This chapter covers Whisper installation, ROS 2 integration, and real-time voice command processing.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Install** OpenAI Whisper and configure for speech recognition
- **Integrate** Whisper with ROS 2 for real-time voice command processing
- **Handle** audio preprocessing and noise reduction
- **Process** multi-language voice input
- **Debug** common audio and recognition issues

## Prerequisites

- **ROS 2 Humble** configured (Module 1)
- **Python 3.10+** with pip
- **Microphone** connected (or simulated audio for testing)
- **Basic audio** concepts (sample rate, channels, formats)
- **Linux audio** setup (ALSA/PulseAudio)

## Part 1: Whisper Fundamentals

### What is OpenAI Whisper?

**Whisper** is OpenAI's automatic speech recognition (ASR) system that:
- **Transcribes** audio to text with high accuracy
- **Supports** 99+ languages
- **Handles** accents, background noise, and technical vocabulary
- **Runs** locally (no API calls required)
- **Open-source**: Free to use and modify

**Why Whisper for humanoid robots?**
- **Accuracy**: State-of-the-art recognition (WER < 5% for English)
- **Robustness**: Works with noise and accents
- **Privacy**: Runs locally (no cloud API)
- **Multi-language**: Supports international deployment
- **Real-time**: Can process audio streams in real-time

### Whisper Models

| Model | Size | Speed | Accuracy | Use Case |
|-------|------|-------|----------|----------|
| **tiny** | 39M | Fastest | Good | Real-time, low-resource |
| **base** | 74M | Fast | Very Good | Real-time, balanced |
| **small** | 244M | Medium | Excellent | High accuracy |
| **medium** | 769M | Slow | Excellent | Best accuracy |
| **large** | 1550M | Slowest | Best | Maximum accuracy |

**For real-time robotics**: Use `base` or `small` (good balance of speed/accuracy).

### Audio Processing Pipeline

**Typical pipeline**:
1. **Audio Capture**: Microphone → raw audio stream
2. **Preprocessing**: Noise reduction, normalization
3. **Whisper Inference**: Audio → text transcription
4. **Post-processing**: Text cleaning, command extraction
5. **ROS 2 Publishing**: Text → ROS 2 topic

## Part 2: Hands-On Tutorial

### Project: Real-Time Voice Command System

**Goal**: Set up Whisper for real-time speech recognition and publish transcribed text to ROS 2 topics.

**Tools**: OpenAI Whisper, ROS 2 Humble, Python 3.10+, microphone

### Step 1: Install Whisper

**Install Whisper**:
```bash
# Install via pip
pip3 install openai-whisper

# Install audio dependencies
sudo apt install ffmpeg

# Verify installation
whisper --help
```

**Install ROS 2 audio packages** (optional, for ROS 2 audio topics):
```bash
sudo apt install ros-humble-audio-common
```

### Step 2: Test Whisper with Audio File

**Download test audio**:
```bash
# Download sample audio (or record your own)
wget https://github.com/openai/whisper/raw/main/tests/jfk.flac

# Transcribe
whisper jfk.flac --model base

# Expected output: Text file with transcription
cat jfk.flac.txt
```

**Test with microphone** (record first):
```bash
# Record 5 seconds of audio
arecord -d 5 -f cd test.wav

# Transcribe
whisper test.wav --model base
```

### Step 3: Create ROS 2 Whisper Node

**Create package**:
```bash
cd ~/isaac_ros_ws/src
ros2 pkg create --build-type ament_python voice_commands --dependencies rclpy std_msgs
cd voice_commands
mkdir -p voice_commands
```

**Create node**: `voice_commands/whisper_node.py`

```python
#!/usr/bin/env python3
"""
ROS 2 node for OpenAI Whisper speech recognition
ROS 2 Humble | Python 3.10+ | Whisper
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import numpy as np
import queue
import threading
from datetime import datetime

class WhisperNode(Node):
    """
    Real-time speech recognition using OpenAI Whisper
    Publishes transcribed text to /voice_commands/text topic
    """
    def __init__(self):
        super().__init__('whisper_node')
        
        # Parameters
        self.declare_parameter('model', 'base')  # Whisper model size
        self.declare_parameter('language', 'en')  # Language code
        self.declare_parameter('sample_rate', 16000)  # Audio sample rate
        self.declare_parameter('chunk_size', 4096)  # Audio chunk size
        self.declare_parameter('vad_threshold', 0.5)  # Voice activity detection
        
        model_name = self.get_parameter('model').value
        self.language = self.get_parameter('language').value
        sample_rate = self.get_parameter('sample_rate').value
        chunk_size = self.get_parameter('chunk_size').value
        
        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_name}')
        self.model = whisper.load_model(model_name)
        self.get_logger().info('Whisper model loaded')
        
        # Publisher
        self.text_pub = self.create_publisher(String, '/voice_commands/text', 10)
        
        # Audio setup
        self.audio = pyaudio.PyAudio()
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio_queue = queue.Queue()
        
        # Start audio capture thread
        self.audio_thread = threading.Thread(target=self.audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        # Start processing timer
        self.process_timer = self.create_timer(2.0, self.process_audio)  # Process every 2 seconds
        
        self.get_logger().info('Whisper node started, listening...')
    
    def audio_capture_loop(self):
        """Capture audio from microphone in background thread"""
        try:
            stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
            
            self.get_logger().info('Microphone opened, capturing audio...')
            
            while rclpy.ok():
                try:
                    audio_data = stream.read(self.chunk_size, exception_on_overflow=False)
                    audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
                    self.audio_queue.put(audio_array)
                except Exception as e:
                    self.get_logger().error(f'Audio capture error: {e}')
                    
        except Exception as e:
            self.get_logger().error(f'Failed to open microphone: {e}')
            self.get_logger().error('Make sure microphone is connected and permissions granted')
    
    def process_audio(self):
        """Process accumulated audio with Whisper"""
        if self.audio_queue.qsize() < 10:  # Need enough audio chunks
            return
        
        # Collect audio chunks
        audio_chunks = []
        while not self.audio_queue.empty() and len(audio_chunks) < 50:  # ~2 seconds
            audio_chunks.append(self.audio_queue.get())
        
        if len(audio_chunks) == 0:
            return
        
        # Concatenate audio
        audio_data = np.concatenate(audio_chunks)
        
        # Check for voice activity (simple energy-based VAD)
        energy = np.mean(audio_data ** 2)
        vad_threshold = self.get_parameter('vad_threshold').value
        
        if energy < vad_threshold:
            return  # No voice detected
        
        # Transcribe with Whisper
        try:
            result = self.model.transcribe(
                audio_data,
                language=self.language,
                task='transcribe',
                fp16=False  # Use FP32 for compatibility
            )
            
            text = result['text'].strip()
            
            if text and len(text) > 2:  # Ignore very short transcriptions
                self.get_logger().info(f'Transcribed: "{text}"')
                
                # Publish to ROS 2
                msg = String()
                msg.data = text
                self.text_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Whisper transcription error: {e}')
    
    def destroy_node(self):
        """Cleanup"""
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Install dependencies** (`setup.py`):
```python
from setuptools import setup

setup(
    name='voice_commands',
    version='0.0.1',
    packages=['voice_commands'],
    install_requires=[
        'setuptools',
        'rclpy',
        'openai-whisper',
        'pyaudio',
        'numpy',
    ],
    entry_points={
        'console_scripts': [
            'whisper_node = voice_commands.whisper_node:main',
        ],
    },
)
```

**Install system dependencies**:
```bash
# Install PyAudio dependencies
sudo apt install portaudio19-dev python3-pyaudio

# Install Python packages
pip3 install pyaudio numpy
```

### Step 4: Build and Launch

**Build package**:
```bash
cd ~/isaac_ros_ws
colcon build --packages-select voice_commands
source install/setup.bash
```

**Launch node**:
```bash
ros2 run voice_commands whisper_node
```

**Expected Output**:
```
[INFO] [whisper_node]: Loading Whisper model: base
[INFO] [whisper_node]: Whisper model loaded
[INFO] [whisper_node]: Microphone opened, capturing audio...
[INFO] [whisper_node]: Whisper node started, listening...
[INFO] [whisper_node]: Transcribed: "hello robot"
[INFO] [whisper_node]: Transcribed: "pick up the cup"
```

**Verify topic**:
```bash
# Terminal 2: Echo transcribed text
ros2 topic echo /voice_commands/text
```

### Step 5: Command Processing Node

**Create command processor**: `voice_commands/command_processor.py`

```python
#!/usr/bin/env python3
"""
Process voice commands and extract intent
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

class CommandProcessor(Node):
    """
    Processes transcribed text and extracts robot commands
    """
    def __init__(self):
        super().__init__('command_processor')
        
        # Subscribe to transcribed text
        self.text_sub = self.create_subscription(
            String,
            '/voice_commands/text',
            self.text_callback,
            10
        )
        
        # Publisher for processed commands
        self.cmd_pub = self.create_publisher(String, '/voice_commands/command', 10)
        
        # Command patterns (simple keyword matching)
        self.commands = {
            'navigate': ['go to', 'move to', 'navigate', 'walk to'],
            'pick': ['pick up', 'grab', 'take', 'get'],
            'place': ['place', 'put', 'set down', 'drop'],
            'stop': ['stop', 'halt', 'freeze'],
            'come': ['come here', 'come to me', 'follow me'],
        }
        
        self.get_logger().info('Command processor started')
    
    def text_callback(self, msg):
        """Process transcribed text and extract command"""
        text = msg.data.lower()
        
        # Simple keyword matching (will be replaced with LLM in Chapter 4.2)
        for command_type, keywords in self.commands.items():
            for keyword in keywords:
                if keyword in text:
                    self.get_logger().info(f'Command detected: {command_type} from "{text}"')
                    
                    # Publish command
                    cmd_msg = String()
                    cmd_msg.data = f'{command_type}:{text}'
                    self.cmd_pub.publish(cmd_msg)
                    return

def main(args=None):
    rclpy.init(args=args)
    node = CommandProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Add to setup.py**:
```python
entry_points={
    'console_scripts': [
        'whisper_node = voice_commands.whisper_node:main',
        'command_processor = voice_commands.command_processor:main',
    ],
},
```

### Step 6: Test Complete Pipeline

**Launch both nodes**:
```bash
# Terminal 1: Whisper node
ros2 run voice_commands whisper_node

# Terminal 2: Command processor
ros2 run voice_commands command_processor

# Terminal 3: Monitor commands
ros2 topic echo /voice_commands/command
```

**Test commands**:
- Say: "Go to the kitchen"
- Say: "Pick up the cup"
- Say: "Stop"

**Expected Output**:
```
[INFO] [command_processor]: Command detected: navigate from "go to the kitchen"
data: "navigate:go to the kitchen"
---
[INFO] [command_processor]: Command detected: pick from "pick up the cup"
data: "pick:pick up the cup"
```

### Step 7: Debugging Common Issues

#### Issue 1: "No module named 'whisper'"
**Symptoms**: Import error when launching node

**Solutions**:
```bash
# Verify Whisper installed
pip3 show openai-whisper

# Reinstall if needed
pip3 install --upgrade openai-whisper

# Check Python path
python3 -c "import whisper; print(whisper.__file__)"
```

#### Issue 2: "Microphone not found" or "Permission denied"
**Symptoms**: Audio capture fails

**Solutions**:
```bash
# List audio devices
arecord -l

# Test microphone
arecord -d 5 test.wav

# Check permissions (Linux)
groups  # Should include 'audio' group
sudo usermod -a -G audio $USER
# Log out and back in

# Check PulseAudio
pulseaudio --check
pulseaudio --start
```

#### Issue 3: "Low recognition accuracy"
**Symptoms**: Whisper transcribes incorrectly

**Solutions**:
```python
# Use larger model
model_name = 'small'  # Instead of 'base'

# Specify language explicitly
language = 'en'  # English

# Improve audio quality
# Reduce background noise
# Speak clearly and closer to microphone
```

#### Issue 4: "High latency" or "Delayed transcription"
**Symptoms**: Long delay between speech and transcription

**Solutions**:
```python
# Use smaller model
model_name = 'tiny'  # Faster but less accurate

# Reduce processing interval
process_timer = self.create_timer(1.0, self.process_audio)  # Process every 1 second

# Optimize audio chunk size
chunk_size = 2048  # Smaller chunks = faster processing
```

#### Issue 5: "Memory usage too high"
**Symptoms**: System runs out of memory

**Solutions**:
```python
# Use smaller model
model_name = 'tiny'  # 39M parameters vs. 769M for medium

# Process audio in smaller batches
while len(audio_chunks) < 25:  # Shorter audio segments

# Clear audio queue periodically
if self.audio_queue.qsize() > 100:
    # Clear old audio
    while not self.audio_queue.empty():
        self.audio_queue.get()
```

## Part 3: Advanced Topics (Optional)

### Streaming Whisper

**Real-time streaming** (lower latency):
```python
# Use streaming API (if available)
# Process audio chunks as they arrive
# Instead of accumulating 2 seconds
```

### Multi-Language Support

**Detect language automatically**:
```python
# Whisper can detect language
result = self.model.transcribe(
    audio_data,
    language=None,  # Auto-detect
    task='transcribe'
)

detected_language = result['language']
self.get_logger().info(f'Detected language: {detected_language}')
```

### Noise Reduction

**Preprocess audio** (reduce noise):
```python
import noisereduce as nr

# Reduce noise before transcription
audio_clean = nr.reduce_noise(
    y=audio_data,
    sr=self.sample_rate
)

# Then transcribe cleaned audio
result = self.model.transcribe(audio_clean, ...)
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Voice input**: Capstone will accept natural language commands via Whisper
- **Real-time processing**: Low-latency transcription enables responsive interaction
- **Multi-language**: Supports international deployment
- **Robustness**: Works with noise and accents (real-world conditions)

Understanding Whisper integration now is essential for the capstone voice interface.

## Summary

You learned:
- ✅ Installed **OpenAI Whisper** for speech recognition
- ✅ Created **ROS 2 node** for real-time voice transcription
- ✅ Implemented **command processing** pipeline
- ✅ Handled **audio capture** and preprocessing
- ✅ Debugged **common audio and recognition issues**

**Next steps**: In Chapter 4.2, you'll integrate LLM-based planning to decompose voice commands into executable robot actions.

---

## Exercises

### Exercise 1: Basic Voice Recognition (Required)

**Objective**: Set up Whisper and verify transcription accuracy.

**Tasks**:
1. Install Whisper and test with audio file
2. Record 10 voice commands
3. Transcribe each command
4. Measure accuracy (correct words / total words)
5. Test with different Whisper models (tiny, base, small)

**Acceptance Criteria**:
- [ ] Whisper installed and working
- [ ] 10 commands transcribed
- [ ] Accuracy > 80% for clear speech
- [ ] Comparison of model performance documented

**Estimated Time**: 90 minutes

### Exercise 2: ROS 2 Integration (Required)

**Objective**: Integrate Whisper with ROS 2 for real-time commands.

**Tasks**:
1. Create Whisper ROS 2 node
2. Configure microphone input
3. Publish transcribed text to ROS 2 topic
4. Create command processor node
5. Test end-to-end pipeline

**Acceptance Criteria**:
- [ ] Whisper node publishes to `/voice_commands/text`
- [ ] Command processor extracts commands
- [ ] Commands published to `/voice_commands/command`
- [ ] Latency < 3 seconds from speech to command

**Estimated Time**: 120 minutes

### Exercise 3: Noise Robustness Test (Challenge)

**Objective**: Test Whisper accuracy with different noise levels.

**Tasks**:
1. Record commands in quiet environment (baseline)
2. Record same commands with background noise
3. Record with different microphone distances
4. Compare accuracy across conditions
5. Implement noise reduction if needed

**Metrics**:
- Word Error Rate (WER) for each condition
- Recognition latency
- Command extraction success rate

**Estimated Time**: 180 minutes

---

## Additional Resources

- [OpenAI Whisper GitHub](https://github.com/openai/whisper) - Official repository
- [Whisper Paper](https://arxiv.org/abs/2212.04356) - Research paper
- [PyAudio Documentation](https://people.csail.mit.edu/hubert/pyaudio/docs/) - Audio I/O library
- [ROS 2 Audio Common](http://wiki.ros.org/audio_common) - ROS 2 audio packages

---

**Next**: [Chapter 4.2: LLM-Based Cognitive Planning →](chapter-4 to 2.md)
