---
sidebar_position: 3
title: 4.2 LLM-Based Cognitive Planning
---

# Chapter 4.2: LLM-Based Cognitive Planning

Large Language Models (LLMs) enable robots to understand natural language and decompose complex goals into executable steps. This chapter covers integrating GPT-4 or open-source LLMs (Llama, Mistral) with ROS 2 for task planning, prompt engineering, and handling ambiguous commands.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Set up** GPT-4 API or local LLM (Llama, Mistral) for task planning
- **Implement** prompt engineering for robot task decomposition
- **Create** planning pipeline: goal → sub-tasks → executable actions
- **Handle** ambiguous commands and error recovery via LLM reasoning
- **Integrate** LLM planning with ROS 2 action system

## Prerequisites

- **Chapter 4.1** completed (Whisper voice recognition)
- **OpenAI API key** (for GPT-4) OR **local LLM setup** (for open-source models)
- **Python 3.10+** with pip
- **Basic NLP concepts**: Prompts, tokens, embeddings (we'll cover as needed)
- **Understanding** of ROS 2 Actions (from Module 1)

## Part 1: LLM Planning Fundamentals

### What is LLM-Based Planning?

**LLM planning** uses language models to:
- **Understand** natural language goals ("Pick up the red cup and place it on the table")
- **Decompose** complex tasks into sub-tasks
- **Generate** executable action sequences
- **Handle** ambiguity and context ("the cup" → which cup?)
- **Recover** from errors by replanning

**Why LLMs for robot planning?**
- **Natural language**: No need to program every scenario
- **Reasoning**: LLMs can infer implicit steps
- **Generalization**: Handles novel situations
- **Context awareness**: Understands "the cup" refers to previously mentioned object

### LLM Options

| Model | Type | Cost | Latency | Use Case |
|-------|------|------|---------|----------|
| **GPT-4** | API (OpenAI) | $$$ | Low | Production, high accuracy |
| **GPT-3.5-turbo** | API (OpenAI) | $ | Low | Development, good accuracy |
| **Claude 3** | API (Anthropic) | $$ | Low | Alternative to GPT-4 |
| **Llama 3** | Local (Meta) | Free | Medium | Privacy, offline |
| **Mistral** | Local/API | Free/$ | Medium | Open-source alternative |

**This chapter covers**: GPT-4 API (easiest) and Llama 3 local (privacy-focused).

### Planning Pipeline

**Typical pipeline**:
1. **Voice Input**: "Pick up the red cup and bring it to me"
2. **LLM Prompt**: System prompt + user command
3. **Task Decomposition**: LLM generates step-by-step plan
4. **Action Extraction**: Parse LLM output → ROS 2 Actions
5. **Execution**: Execute actions sequentially
6. **Feedback**: LLM replans if action fails

## Part 2: Hands-On Tutorial

### Project: LLM Task Planner for Humanoid Robot

**Goal**: Set up LLM (GPT-4 or Llama) to decompose voice commands into executable robot actions.

**Tools**: OpenAI API (or local LLM), LangChain, ROS 2 Humble, Python 3.10+

### Step 1: Set Up OpenAI API (Option A)

**Get API key**:
1. Visit: [https://platform.openai.com/api-keys](https://platform.openai.com/api-keys)
2. Create account (or sign in)
3. Generate API key
4. Save key securely (don't commit to git!)

**Install OpenAI Python library**:
```bash
pip3 install openai python-dotenv
```

**Create environment file**: `.env`
```bash
OPENAI_API_KEY=sk-your-api-key-here
```

**Test API connection**:
```python
#!/usr/bin/env python3
"""
Test OpenAI API connection
"""
import os
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()

client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

response = client.chat.completions.create(
    model="gpt-4",
    messages=[
        {"role": "user", "content": "Say hello"}
    ]
)

print(response.choices[0].message.content)
```

### Step 2: Set Up Local LLM (Option B - Llama 3)

**Install Ollama** (easy local LLM runner):
```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull Llama 3 model
ollama pull llama3

# Test
ollama run llama3 "Say hello"
```

**Install Python client**:
```bash
pip3 install ollama
```

**Test connection**:
```python
import ollama

response = ollama.chat(model='llama3', messages=[
    {'role': 'user', 'content': 'Say hello'}
])

print(response['message']['content'])
```

### Step 3: Create LLM Planner Node

**Create planner node**: `voice_commands/llm_planner.py`

```python
#!/usr/bin/env python3
"""
LLM-based task planner for humanoid robot
ROS 2 Humble | Python 3.10+ | GPT-4 or Llama
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from dotenv import load_dotenv

# Try OpenAI first, fallback to Ollama
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

try:
    import ollama
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False

class LLMPlanner(Node):
    """
    Uses LLM to decompose natural language commands into robot actions
    """
    def __init__(self):
        super().__init__('llm_planner')
        
        # Parameters
        self.declare_parameter('llm_provider', 'openai')  # 'openai' or 'ollama'
        self.declare_parameter('model', 'gpt-4')  # Model name
        
        provider = self.get_parameter('llm_provider').value
        model_name = self.get_parameter('model').value
        
        # Initialize LLM client
        if provider == 'openai' and OPENAI_AVAILABLE:
            load_dotenv()
            api_key = os.getenv('OPENAI_API_KEY')
            if not api_key:
                self.get_logger().error('OPENAI_API_KEY not found in .env file')
                raise ValueError('OpenAI API key required')
            self.client = OpenAI(api_key=api_key)
            self.model = model_name
            self.provider = 'openai'
            self.get_logger().info(f'Using OpenAI model: {self.model}')
        elif provider == 'ollama' and OLLAMA_AVAILABLE:
            self.client = ollama
            self.model = model_name if model_name != 'gpt-4' else 'llama3'
            self.provider = 'ollama'
            self.get_logger().info(f'Using Ollama model: {self.model}')
        else:
            self.get_logger().error('No LLM provider available. Install openai or ollama.')
            raise RuntimeError('LLM provider not available')
        
        # System prompt for robot planning
        self.system_prompt = """You are a task planner for a humanoid robot. 
Your job is to decompose natural language commands into a sequence of executable robot actions.

Available actions:
- navigate_to(location): Move robot to specified location
- pick_up(object): Grasp and lift an object
- place(object, location): Put object at location
- look_at(object): Turn head/camera toward object
- speak(text): Say something to the human
- wait(duration): Wait for specified seconds

Output format: JSON array of actions, each with "action" and "parameters" fields.
Example:
Input: "Pick up the red cup and place it on the table"
Output: [
  {"action": "look_at", "parameters": {"object": "red cup"}},
  {"action": "navigate_to", "parameters": {"location": "red cup"}},
  {"action": "pick_up", "parameters": {"object": "red cup"}},
  {"action": "navigate_to", "parameters": {"location": "table"}},
  {"action": "place", "parameters": {"object": "red cup", "location": "table"}}
]

Be precise and break down complex tasks into clear steps."""
        
        # Subscribe to voice commands
        self.cmd_sub = self.create_subscription(
            String,
            '/voice_commands/command',
            self.command_callback,
            10
        )
        
        # Publisher for planned actions
        self.plan_pub = self.create_publisher(String, '/voice_commands/plan', 10)
        
        self.get_logger().info('LLM planner started, ready to plan tasks')
    
    def command_callback(self, msg):
        """Process voice command and generate plan"""
        command = msg.data
        
        # Extract command text (remove prefix like "navigate:")
        if ':' in command:
            command = command.split(':', 1)[1]
        
        self.get_logger().info(f'Planning for command: "{command}"')
        
        # Generate plan using LLM
        plan = self.generate_plan(command)
        
        if plan:
            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            
            self.get_logger().info(f'Generated plan: {len(plan)} actions')
            for i, action in enumerate(plan, 1):
                self.get_logger().info(f'  {i}. {action["action"]}({action["parameters"]})')
    
    def generate_plan(self, command):
        """Use LLM to generate action plan"""
        try:
            if self.provider == 'openai':
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": self.system_prompt},
                        {"role": "user", "content": f"Plan the following task: {command}"}
                    ],
                    temperature=0.3,  # Lower = more deterministic
                    max_tokens=500
                )
                content = response.choices[0].message.content
            else:  # ollama
                response = self.client.chat(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": self.system_prompt},
                        {"role": "user", "content": f"Plan the following task: {command}"}
                    ]
                )
                content = response['message']['content']
            
            # Parse JSON from LLM response
            # LLM might include markdown code blocks, extract JSON
            if '```json' in content:
                json_start = content.find('```json') + 7
                json_end = content.find('```', json_start)
                content = content[json_start:json_end].strip()
            elif '```' in content:
                json_start = content.find('```') + 3
                json_end = content.find('```', json_start)
                content = content[json_start:json_end].strip()
            
            plan = json.loads(content)
            
            # Validate plan structure
            if isinstance(plan, list) and all('action' in item for item in plan):
                return plan
            else:
                self.get_logger().error(f'Invalid plan format: {plan}')
                return None
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response as JSON: {e}')
            self.get_logger().error(f'LLM response: {content}')
            return None
        except Exception as e:
            self.get_logger().error(f'LLM planning error: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanner()
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

**Add to setup.py**:
```python
install_requires=[
    'setuptools',
    'rclpy',
    'openai',  # Optional
    'ollama',  # Optional
    'python-dotenv',
],
```

### Step 4: Test LLM Planning

**Launch planner**:
```bash
cd ~/isaac_ros_ws
colcon build --packages-select voice_commands
source install/setup.bash

# Launch planner (with OpenAI)
ros2 run voice_commands llm_planner --ros-args -p llm_provider:=openai -p model:=gpt-4

# Or with Ollama
ros2 run voice_commands llm_planner --ros-args -p llm_provider:=ollama -p model:=llama3
```

**Test with command**:
```bash
# Terminal 2: Send test command
ros2 topic pub --once /voice_commands/command std_msgs/String \
  "{data: 'navigate:Pick up the red cup and place it on the table'}"

# Terminal 3: Monitor plan
ros2 topic echo /voice_commands/plan
```

**Expected Output**:
```json
{
  "data": "[{\"action\": \"look_at\", \"parameters\": {\"object\": \"red cup\"}}, {\"action\": \"navigate_to\", \"parameters\": {\"location\": \"red cup\"}}, {\"action\": \"pick_up\", \"parameters\": {\"object\": \"red cup\"}}, {\"action\": \"navigate_to\", \"parameters\": {\"location\": \"table\"}}, {\"action\": \"place\", \"parameters\": {\"object\": \"red cup\", \"location\": \"table\"}}]"
}
```

### Step 5: Enhanced Prompt Engineering

**Improved system prompt** (with context awareness):
```python
self.system_prompt = """You are a task planner for a humanoid robot named "Robo".

Robot capabilities:
- Navigation: Can move to named locations (kitchen, living room, table, etc.)
- Manipulation: Can pick up objects (cups, books, tools) and place them
- Perception: Has cameras and can identify objects by color/shape
- Communication: Can speak to humans

Current context:
- Robot is in the living room
- Known objects: red cup (on coffee table), blue book (on shelf), green bottle (on floor)
- Known locations: kitchen, living room, bedroom, table, shelf

When planning:
1. Break complex tasks into atomic actions
2. Consider preconditions (e.g., must navigate before picking up)
3. Use context to resolve ambiguous references ("the cup" → "red cup")
4. Add look_at actions before manipulation for object identification
5. Verify actions are executable (robot can reach, object exists)

Output format: JSON array of actions.
Each action has:
- "action": action name (navigate_to, pick_up, place, look_at, speak, wait)
- "parameters": dict with action-specific parameters
- "precondition": (optional) what must be true before this action
- "postcondition": (optional) what becomes true after this action

Example:
Input: "Bring me the red cup"
Output: [
  {"action": "look_at", "parameters": {"object": "red cup"}},
  {"action": "navigate_to", "parameters": {"location": "coffee table"}},
  {"action": "pick_up", "parameters": {"object": "red cup"}},
  {"action": "navigate_to", "parameters": {"location": "human"}},
  {"action": "place", "parameters": {"object": "red cup", "location": "human hand"}}
]"""
```

### Step 6: Handle Ambiguity and Context

**Context manager**: `voice_commands/context_manager.py`

```python
#!/usr/bin/env python3
"""
Manages robot context for LLM planning
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class ContextManager(Node):
    """
    Tracks robot state and provides context to LLM
    """
    def __init__(self):
        super().__init__('context_manager')
        
        # Robot state
        self.current_location = "living_room"
        self.held_object = None
        self.known_objects = {
            "red_cup": {"location": "coffee_table", "color": "red"},
            "blue_book": {"location": "shelf", "color": "blue"},
            "green_bottle": {"location": "floor", "color": "green"},
        }
        
        # Subscribe to action completion
        self.action_complete_sub = self.create_subscription(
            String,
            '/voice_commands/action_complete',
            self.action_complete_callback,
            10
        )
        
        # Publisher for context updates
        self.context_pub = self.create_publisher(String, '/voice_commands/context', 10)
        
    def action_complete_callback(self, msg):
        """Update context when action completes"""
        result = json.loads(msg.data)
        action = result.get('action')
        
        if action == 'pick_up':
            self.held_object = result.get('object')
            # Update object location
            if self.held_object in self.known_objects:
                self.known_objects[self.held_object]['location'] = 'robot_hand'
        elif action == 'place':
            self.held_object = None
            if 'object' in result and 'location' in result:
                if result['object'] in self.known_objects:
                    self.known_objects[result['object']]['location'] = result['location']
        elif action == 'navigate_to':
            self.current_location = result.get('location', self.current_location)
        
        # Publish updated context
        self.publish_context()
    
    def publish_context(self):
        """Publish current context"""
        context = {
            "current_location": self.current_location,
            "held_object": self.held_object,
            "known_objects": self.known_objects
        }
        
        msg = String()
        msg.data = json.dumps(context)
        self.context_pub.publish(msg)
    
    def get_context_string(self):
        """Get context as string for LLM prompt"""
        context_str = f"Robot is currently in: {self.current_location}\n"
        if self.held_object:
            context_str += f"Robot is holding: {self.held_object}\n"
        context_str += "Known objects:\n"
        for obj, info in self.known_objects.items():
            context_str += f"  - {obj} ({info['color']}) at {info['location']}\n"
        return context_str
```

**Update LLM planner** to use context:
```python
# In LLMPlanner.__init__
self.context_sub = self.create_subscription(
    String,
    '/voice_commands/context',
    self.context_callback,
    10
)
self.context = {}

# In generate_plan
context_str = self.get_context_string()
messages=[
    {"role": "system", "content": self.system_prompt},
    {"role": "user", "content": f"Context:\n{context_str}\n\nPlan the following task: {command}"}
]
```

### Step 7: Error Recovery and Replanning

**Error handler**: `voice_commands/error_handler.py`

```python
#!/usr/bin/env python3
"""
Handle action failures and replan using LLM
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler')
        
        # Subscribe to action failures
        self.error_sub = self.create_subscription(
            String,
            '/voice_commands/action_error',
            self.error_callback,
            10
        )
        
        # Publisher for replanning request
        self.replan_pub = self.create_publisher(String, '/voice_commands/replan', 10)
        
    def error_callback(self, msg):
        """Handle action failure and request replanning"""
        error_data = json.loads(msg.data)
        failed_action = error_data.get('action')
        error_message = error_data.get('error')
        original_command = error_data.get('original_command')
        
        self.get_logger().warn(f'Action failed: {failed_action} - {error_message}')
        
        # Request LLM to replan with error context
        replan_request = {
            "original_command": original_command,
            "failed_action": failed_action,
            "error": error_message,
            "request": "replan"
        }
        
        msg = String()
        msg.data = json.dumps(replan_request)
        self.replan_pub.publish(msg)
```

**Update LLM planner** to handle replanning:
```python
# Subscribe to replan requests
self.replan_sub = self.create_subscription(
    String,
    '/voice_commands/replan',
    self.replan_callback,
    10
)

def replan_callback(self, msg):
    """Replan after action failure"""
    request = json.loads(msg.data)
    original_command = request['original_command']
    failed_action = request['failed_action']
    error = request['error']
    
    # Ask LLM to replan considering the error
    replan_prompt = f"""Previous plan failed:
- Original command: {original_command}
- Failed action: {failed_action}
- Error: {error}

Please replan the task, avoiding the failed action or finding an alternative approach."""
    
    # Generate new plan
    new_plan = self.generate_plan_with_prompt(replan_prompt)
    # ... (publish new plan)
```

### Step 8: Debugging Common Issues

#### Issue 1: "API key not found" or "Rate limit exceeded"
**Symptoms**: OpenAI API errors

**Solutions**:
```bash
# Check .env file exists and has API key
cat .env | grep OPENAI_API_KEY

# Verify API key is valid
python3 -c "from openai import OpenAI; import os; from dotenv import load_dotenv; load_dotenv(); client = OpenAI(api_key=os.getenv('OPENAI_API_KEY')); print('API key valid')"

# For rate limits: Use smaller model or add retry logic
```

#### Issue 2: "LLM returns invalid JSON"
**Symptoms**: JSON parsing errors

**Solutions**:
```python
# Add JSON extraction (already in code)
# Try multiple parsing strategies
# Use structured output (if available): response_format={"type": "json_object"}

# For OpenAI GPT-4:
response = client.chat.completions.create(
    ...,
    response_format={"type": "json_object"}  # Forces JSON output
)
```

#### Issue 3: "Planning too slow" (high latency)
**Symptoms**: Long delay between command and plan

**Solutions**:
```python
# Use faster model
model = 'gpt-3.5-turbo'  # Faster than gpt-4

# Reduce max_tokens
max_tokens = 300  # Shorter responses

# Use local LLM (Ollama) for lower latency
provider = 'ollama'
model = 'llama3'
```

#### Issue 4: "Plan doesn't match command intent"
**Symptoms**: LLM generates wrong actions

**Solutions**:
```python
# Improve system prompt (more specific)
# Add examples in few-shot learning
# Use temperature=0.0 for more deterministic output
temperature = 0.0  # Most deterministic

# Add validation: Check if plan makes sense
def validate_plan(plan, command):
    # Check plan contains expected actions
    # Verify parameters are reasonable
    pass
```

## Part 3: Advanced Topics (Optional)

### Few-Shot Learning

**Add examples to prompt**:
```python
self.system_prompt += """

Examples:

Input: "Go to the kitchen"
Output: [{"action": "navigate_to", "parameters": {"location": "kitchen"}}]

Input: "Pick up the cup"
Output: [
  {"action": "look_at", "parameters": {"object": "cup"}},
  {"action": "navigate_to", "parameters": {"location": "cup"}},
  {"action": "pick_up", "parameters": {"object": "cup"}}
]

Input: "Bring me the red cup"
Output: [
  {"action": "look_at", "parameters": {"object": "red cup"}},
  {"action": "navigate_to", "parameters": {"location": "red cup"}},
  {"action": "pick_up", "parameters": {"object": "red cup"}},
  {"action": "navigate_to", "parameters": {"location": "human"}},
  {"action": "place", "parameters": {"object": "red cup", "location": "human hand"}}
]"""
```

### Function Calling (Structured Output)

**Use OpenAI function calling** (more reliable JSON):
```python
response = client.chat.completions.create(
    model="gpt-4",
    messages=[...],
    functions=[{
        "name": "generate_plan",
        "description": "Generate robot action plan",
        "parameters": {
            "type": "object",
            "properties": {
                "actions": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "action": {"type": "string"},
                            "parameters": {"type": "object"}
                        }
                    }
                }
            }
        }
    }],
    function_call={"name": "generate_plan"}
)

# Extract function call result
function_call = response.choices[0].message.function_call
plan = json.loads(function_call.arguments)['actions']
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Task decomposition**: Capstone will use LLM to break complex commands into steps
- **Natural language understanding**: Handles ambiguous and complex commands
- **Error recovery**: LLM replans when actions fail
- **Context awareness**: Understands references and maintains state

Understanding LLM planning now is essential for the capstone cognitive system.

## Summary

You learned:
- ✅ Set up **GPT-4 API or local LLM** (Llama) for task planning
- ✅ Implemented **prompt engineering** for robot task decomposition
- ✅ Created **planning pipeline** converting goals to executable actions
- ✅ Handled **ambiguous commands** and context awareness
- ✅ Implemented **error recovery** via LLM replanning

**Next steps**: In Chapter 4.3, you'll map LLM-generated plans to ROS 2 Actions for execution.

---

## Exercises

### Exercise 1: Basic LLM Planning (Required)

**Objective**: Set up LLM and generate plans for simple commands.

**Tasks**:
1. Set up OpenAI API or Ollama
2. Create LLM planner node
3. Test with 5 different commands:
   - "Go to the kitchen"
   - "Pick up the cup"
   - "Bring me the red cup"
   - "Place the book on the shelf"
   - "Stop"
4. Verify plans are valid JSON
5. Document plan quality (correctness, completeness)

**Acceptance Criteria**:
- [ ] LLM planner node running
- [ ] Plans generated for all 5 commands
- [ ] Plans are valid JSON
- [ ] Plans contain reasonable actions
- [ ] Planning latency < 5 seconds

**Estimated Time**: 120 minutes

### Exercise 2: Prompt Engineering (Required)

**Objective**: Optimize prompts for better planning quality.

**Tasks**:
1. Create baseline prompt
2. Test with 10 commands
3. Measure plan quality (correctness, completeness)
4. Iterate on prompt (add examples, clarify actions)
5. Compare plan quality before/after prompt improvements

**Metrics**:
- Plan correctness (% of correct actions)
- Plan completeness (% of required steps included)
- Action parameter accuracy

**Estimated Time**: 180 minutes

### Exercise 3: Context-Aware Planning (Challenge)

**Objective**: Implement context management for better planning.

**Tasks**:
1. Create context manager tracking robot state
2. Integrate context into LLM prompts
3. Test with context-dependent commands:
   - "Pick up the cup" (when multiple cups exist)
   - "Put it down" (referring to held object)
   - "Go back" (return to previous location)
4. Verify context improves plan accuracy
5. Document context impact on planning

**Requirements**:
- Context manager node
- Context integrated into prompts
- Test results showing improvement

**Estimated Time**: 240 minutes

---

## Additional Resources

- [OpenAI API Documentation](https://platform.openai.com/docs) - GPT-4 API reference
- [Ollama Documentation](https://ollama.com/docs) - Local LLM setup
- [LangChain Documentation](https://python.langchain.com/) - LLM orchestration
- [Prompt Engineering Guide](https://www.promptingguide.ai/) - Best practices

---

**Next**: [Chapter 4.3: Natural Language to ROS 2 Actions →](chapter-4 to 3.md)
