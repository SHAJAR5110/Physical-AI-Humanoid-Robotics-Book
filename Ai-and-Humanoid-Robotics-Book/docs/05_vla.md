# Chapter 5: Vision-Language-Action Models

## What are Vision-Language-Action (VLA) Models?

Vision-Language-Action (VLA) models represent the frontier of Physical AI. They are multimodal neural networks that:

- **Perceive**: Process camera images to understand the scene
- **Reason**: Use language understanding to interpret instructions
- **Act**: Generate robot control commands (joint velocities, gripper commands)

VLAs bridge the gap between human intent and robot action, enabling intuitive robot control through natural language.

### VLA Architecture

```
┌─────────────┐         ┌────────────┐         ┌──────────────┐
│   Image     │         │  Language  │         │   Action     │
│  Encoder    │         │  Encoder   │         │   Decoder    │
│ (Vision)    │         │ (LLM)      │         │ (Motor Ctrl) │
└──────┬──────┘         └──────┬─────┘         └──────┬───────┘
       │                       │                      │
       └───────────┬───────────┘                      │
                   │                                  │
            ┌──────▼──────┐                          │
            │  Fusion      │                          │
            │  (Attention) │──────────────────────────┘
            └──────────────┘
```

### Why VLAs Matter

1. **Intuitive Control**: Robots understand natural language instructions ("pick up the red block")
2. **Few-Shot Learning**: Learn new tasks from demonstrations
3. **Generalization**: Transfer knowledge across environments and robots
4. **Safety**: Language grounding enables safer interactions
5. **Accessibility**: Non-programmers can command robots

## Training Vision-Language-Action Models

### Dataset Collection

VLAs require large datasets of (image, language, action) triplets:

```python
class RobotDatasetCollector:
    def __init__(self, robot, camera, save_path):
        self.robot = robot
        self.camera = camera
        self.save_path = save_path
        self.dataset = []

    def collect_demonstration(self, task_description):
        """Collect human demonstration of a task"""
        frames = []
        joint_positions = []
        gripper_states = []

        print(f"Collecting data for: {task_description}")
        for step in range(max_steps):
            # Capture frame
            frame = self.camera.get_rgb()
            frames.append(frame)

            # Record joint positions
            joint_positions.append(self.robot.get_joint_positions())
            gripper_states.append(self.robot.gripper.get_state())

            if step % 10 == 0:
                print(f"  Step {step}/{max_steps}")

        # Save demonstration
        demo = {
            "task": task_description,
            "frames": frames,
            "joint_positions": joint_positions,
            "gripper_states": gripper_states,
            "timestamp": datetime.now().isoformat()
        }

        filepath = f"{self.save_path}/{task_description}_{datetime.now().timestamp()}.pkl"
        with open(filepath, 'wb') as f:
            pickle.dump(demo, f)

        self.dataset.append(filepath)
        print(f"Saved demonstration to {filepath}")

    def collect_n_demonstrations(self, task_description, n=5):
        """Collect multiple demonstrations of same task"""
        for i in range(n):
            input(f"Press Enter to start demonstration {i+1}/{n}")
            self.collect_demonstration(task_description)
```

### VLA Training Pipeline

```python
import torch
from torch.utils.data import DataLoader, Dataset
from transformers import CLIPVisionModel, AutoTokenizer

class RobotActionDataset(Dataset):
    def __init__(self, demonstrations, max_frames=10):
        self.demonstrations = demonstrations
        self.max_frames = max_frames

    def __len__(self):
        return len(self.demonstrations)

    def __getitem__(self, idx):
        demo = self.demonstrations[idx]

        # Sample frames uniformly
        frame_indices = np.linspace(0, len(demo['frames']) - 1,
                                   self.max_frames).astype(int)
        frames = [demo['frames'][i] for i in frame_indices]

        # Stack into batch
        image_tensor = torch.stack([
            torch.from_numpy(frame).permute(2, 0, 1) / 255.0
            for frame in frames
        ]).float()

        # Text encoding
        text = demo['task']

        # Action sequence (joint positions over time)
        actions = demo['joint_positions'][:self.max_frames]
        action_tensor = torch.tensor(actions, dtype=torch.float32)

        return {
            'images': image_tensor,
            'text': text,
            'actions': action_tensor
        }

class VLAModel(torch.nn.Module):
    def __init__(self, action_dim=7):
        super().__init__()

        # Vision encoder (pre-trained CLIP)
        self.vision_encoder = CLIPVisionModel.from_pretrained("openai/clip-vit-base-patch32")

        # Language encoder (pre-trained CLIP)
        self.language_encoder = AutoTokenizer.from_pretrained("openai/clip-vit-base-patch32")

        # Fusion module
        self.fusion = torch.nn.Sequential(
            torch.nn.Linear(512 + 512, 256),
            torch.nn.ReLU(),
            torch.nn.Linear(256, 128)
        )

        # Action decoder (produces joint targets)
        self.action_decoder = torch.nn.Sequential(
            torch.nn.Linear(128, 256),
            torch.nn.ReLU(),
            torch.nn.Linear(256, action_dim)
        )

    def forward(self, images, text):
        # Encode images [B, T, C, H, W]
        B, T, C, H, W = images.shape
        images_flat = images.view(B*T, C, H, W)
        image_features = self.vision_encoder(images_flat).last_hidden_state.mean(dim=1)
        image_features = image_features.view(B, T, -1).mean(dim=1)  # Average over time

        # Encode text
        text_tokens = self.language_encoder(text, return_tensors="pt", padding=True)
        text_features = self.vision_encoder.text_model(**text_tokens).pooler_output

        # Fuse modalities
        fused = self.fusion(torch.cat([image_features, text_features], dim=1))

        # Decode actions
        actions = self.action_decoder(fused)

        return actions

# Training loop
model = VLAModel(action_dim=7)
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
loss_fn = torch.nn.MSELoss()

train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)

for epoch in range(50):
    total_loss = 0
    for batch in train_loader:
        images = batch['images'].to(device)
        text = batch['text']
        actions = batch['actions'].to(device)

        # Forward pass
        predicted_actions = model(images, text)

        # Loss
        loss = loss_fn(predicted_actions, actions)

        # Backward
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        total_loss += loss.item()

    print(f"Epoch {epoch}: Loss = {total_loss / len(train_loader):.4f}")

torch.save(model.state_dict(), "vla_model.pth")
```

## Using VLAs with Claude API

You can leverage Claude's vision capabilities to power VLA-like reasoning without training a custom model:

```python
import anthropic
import base64
from pathlib import Path

client = anthropic.Anthropic()

def get_robot_action_from_vision(image_path: str, instruction: str, robot_info: dict) -> dict:
    """
    Use Claude's vision to generate robot actions from instruction and image.

    Args:
        image_path: Path to camera image from robot
        instruction: Natural language instruction (e.g., "pick up the blue cube")
        robot_info: Robot specifications (DOF, joint limits, gripper type)

    Returns:
        dict with joint_targets, gripper_command, confidence
    """

    # Read and encode image
    with open(image_path, "rb") as image_file:
        image_data = base64.standard_b64encode(image_file.read()).decode("utf-8")

    # Prepare robot context
    robot_context = f"""
    You are a robot motion planner. The robot has the following specifications:
    - Type: {robot_info['type']}
    - DOF: {robot_info['dof']}
    - Joint limits: {robot_info['joint_limits']}
    - Gripper: {robot_info['gripper_type']}
    - Workspace: {robot_info['workspace']}
    """

    # Get Claude's analysis
    message = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=1024,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/jpeg",
                            "data": image_data,
                        },
                    },
                    {
                        "type": "text",
                        "text": f"""{robot_context}

Current task: {instruction}

Analyze the scene and provide:
1. What objects are visible?
2. Where is the target object?
3. What motion sequence achieves the task?
4. Recommended joint positions (0.0-1.0 normalized, one per DOF)
5. Gripper command (open/close)
6. Confidence (0.0-1.0)

Format as JSON:
{{
    "objects_detected": [...],
    "target_location": {{"x": 0.0, "y": 0.0, "z": 0.0}},
    "motion_plan": "description",
    "joint_targets": [0.5, 0.3, 0.7, ...],
    "gripper_command": "close",
    "confidence": 0.85
}}
"""
                    }
                ],
            }
        ],
    )

    # Parse response
    import json
    response_text = message.content[0].text
    action_dict = json.loads(response_text)

    return action_dict

# Example usage
image_path = "/camera/frame_001.jpg"
instruction = "Pick up the red cube and place it on the blue block"
robot_specs = {
    "type": "UR10",
    "dof": 6,
    "joint_limits": [[-3.14, 3.14]] * 6,
    "gripper_type": "Robotiq 2F-140",
    "workspace": {"x": [-2, 2], "y": [-2, 2], "z": [0, 2.5]}
}

action = get_robot_action_from_vision(image_path, instruction, robot_specs)

# Apply action to robot
robot.set_joint_positions(action['joint_targets'])
robot.gripper.command(action['gripper_command'])
```

## Open-Source VLA Models

Several VLA models are available for download and fine-tuning:

### 1. OpenVLA (OpenAI/UC Berkeley)

```python
from transformers import AutoModelForVision2Seq, AutoImageProcessor, AutoTokenizer

# Load pre-trained OpenVLA model
model_id = "openvla/openvla-7b"
model = AutoModelForVision2Seq.from_pretrained(
    model_id,
    trust_remote_code=True,
    torch_dtype=torch.float16,
    device_map="auto"
)

processor = AutoImageProcessor.from_pretrained(model_id, trust_remote_code=True)
tokenizer = AutoTokenizer.from_pretrained(model_id)

# Inference
image = Image.open("robot_camera.jpg")
instruction = "Pick up the block"

inputs = processor(
    images=image,
    text=instruction,
    return_tensors="pt"
).to("cuda")

outputs = model.generate(
    **inputs,
    max_new_tokens=128,
    num_beams=5
)

actions = tokenizer.decode(outputs[0], skip_special_tokens=True)
print(f"Predicted actions: {actions}")
```

### 2. Octo (UC Berkeley)

```python
from octo.model import OctoModel

# Load Octo model
model = OctoModel.from_pretrained(
    "hf-hub:rail-berkeley/octo-base",
    device="cuda"
)

# Prepare observation (image + instruction)
from PIL import Image
image = Image.open("robot_camera.jpg")

observation = {
    "image_primary": image,
    "language_instruction": "grasp the blue cube"
}

# Get action
action = model.predict_action(observation)
print(f"Predicted joint velocities: {action}")
```

## Fine-Tuning VLAs for Your Robot

```python
# Fine-tune OpenVLA for your specific robot
from transformers import Trainer, TrainingArguments

training_args = TrainingArguments(
    output_dir="./vla-finetuned",
    num_train_epochs=3,
    per_device_train_batch_size=4,
    per_device_eval_batch_size=4,
    learning_rate=5e-5,
    warmup_steps=500,
    weight_decay=0.01,
    evaluation_strategy="epoch",
    save_strategy="epoch"
)

trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=train_dataset,
    eval_dataset=eval_dataset,
    callbacks=[
        EarlyStoppingCallback(early_stopping_patience=3)
    ]
)

trainer.train()
```

## Deploying VLAs on Real Robots

```python
import time
from pathlib import Path

class VLAController:
    def __init__(self, robot, model_path, device="cuda"):
        self.robot = robot
        self.camera = robot.camera
        self.device = device

        # Load VLA model
        self.model = VLAModel.load(model_path, device=device)

    def execute_instruction(self, instruction: str, timeout=10.0):
        """
        Execute a single natural language instruction.

        Args:
            instruction: Natural language command
            timeout: Max time to execute

        Returns:
            success: Whether instruction completed
        """

        start_time = time.time()

        while time.time() - start_time < timeout:
            # Capture frame
            frame = self.camera.get_rgb()

            # Predict action
            with torch.no_grad():
                action = self.model.predict(frame, instruction)

            # Apply action
            self.robot.set_joint_velocities(action[:6])
            self.robot.gripper.set_command(action[6])

            # Check if task complete (using vision or force/torque feedback)
            if self.is_task_complete(instruction):
                return True

            time.sleep(0.1)

        return False

    def is_task_complete(self, instruction: str) -> bool:
        """Detect task completion from vision or proprioception"""
        # Implementation depends on task
        # Could use object detection, force/torque thresholds, etc.
        return False

# Deploy on robot
controller = VLAController(robot, "vla_model.pth")

instructions = [
    "Pick up the red block",
    "Place it on the blue block",
    "Move back to home"
]

for instruction in instructions:
    success = controller.execute_instruction(instruction)
    print(f"'{instruction}': {'Success' if success else 'Failed'}")
```

## Best Practices for VLA Deployment

1. **Safety First**
   - Always have deadman switch ready
   - Start with low velocities
   - Test in simulation first

2. **Data Quality**
   - Consistent lighting and camera calibration
   - Diverse demonstrations
   - Clean labels

3. **Generalization**
   - Test on new objects and scenes
   - Iterate with new data
   - Monitor failure cases

4. **Continuous Learning**
   - Log all deployments
   - Retrain periodically
   - Version models

## Integration with RAG Chatbot

Combine VLAs with RAG for context-aware robot control:

```python
class RobotChatAgent:
    def __init__(self, robot, vla_model, rag_system):
        self.robot = robot
        self.vla = vla_model
        self.rag = rag_system

    def chat(self, user_message: str) -> str:
        # Retrieve relevant chapter content
        relevant_docs = self.rag.search(user_message)

        # Get robot action
        robot_action = self.vla.predict(
            camera_frame=self.robot.camera.get_rgb(),
            instruction=user_message
        )

        # Execute action
        success = self.robot.execute_action(robot_action)

        # Generate response
        response = f"Action: {user_message}\n"
        response += f"Status: {'Success' if success else 'Failed'}\n"
        response += f"Context: {relevant_docs[0]['content'][:200]}..."

        return response

# Deploy agent
agent = RobotChatAgent(robot, vla_model, rag_system)
response = agent.chat("Pick up the blue cube and explain ROS 2 topics")
```

## Next Steps

- **Collect demonstrations** of your robot's capabilities
- **Fine-tune models** for your specific hardware
- **Deploy to real robots** with safety mechanisms
- **Iterate** based on failures
- **Next Chapter**: Capstone Project

---

**Chapter 5 Summary**:
- VLAs combine vision, language, and action understanding
- Train on demonstration data or fine-tune pre-trained models
- Deploy with safety considerations for real-world use
- Integrate with RAG systems for context-aware control

**Estimated Reading Time**: 30 minutes
**Code Examples**: 8 (dataset collection, training, Claude API integration, open-source models, deployment, safety)
**Hands-on**: Train a VLA on sim-to-real robot task
**Next Chapter**: Capstone Project - End-to-End Implementation
