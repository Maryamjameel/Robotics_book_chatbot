---
id: lesson-03-language-perception
title: "Lesson 3: LLMs, Speech, and Vision-Language Models"
---

## 3.3: LLMs and Whisper for Robotics

Natural language understanding is critical for intuitive human-robot interaction. Modern approaches combine Transformer-based large language models (LLMs) with speech recognition and vision systems.

### 3.3.1: Transformer Architecture Basics

**Transformers** use self-attention mechanisms to process sequences in parallel:

$$\text{Attention}(\mathbf{Q}, \mathbf{K}, \mathbf{V}) = \text{softmax}\left(\frac{\mathbf{Q}\mathbf{K}^T}{\sqrt{d_k}}\right)\mathbf{V}$$

where:
- $\mathbf{Q}$ (Query), $\mathbf{K}$ (Key), $\mathbf{V}$ (Value) are learned projections
- $d_k$ is the key dimension (prevents attention weights from becoming too small)

Multi-head attention allows the model to attend to different parts of the input simultaneously.

### 3.3.2: OpenAI Whisper for Speech Recognition

**Whisper** is a robust, multilingual speech-to-text model trained on 680,000 hours of multilingual audio:

- **Robustness**: Works with background noise, accents, technical jargon
- **Multilingual**: Supports 99+ languages
- **Real-time capable**: Inference in seconds on CPU

Integration into robotics: Stream audio from robot microphone → Whisper → JSON transcript

### 3.3.3: LLM Prompt Engineering for Robotics

Prompt engineering guides LLMs to produce robot-actionable outputs. Example prompt:

```
You are a robot task planner. Given a user instruction, decompose it into atomic actions.
Each action must be one of: GRASP, PLACE, MOVE_TO, ROTATE, PUSH, PULL.
Output: JSON list of actions with parameters.

User: "Pick up the red cube and place it on the table"
```

The LLM learns to output structured JSON suitable for robot execution.

### 3.3.4: Vision-Language Models (CLIP, BLIP)

**CLIP** (Contrastive Language-Image Pre-training) learns to match images with text descriptions:

$$\text{Similarity}(\text{image}, \text{text}) = \cos(\text{encode}_\text{image}(img), \text{encode}_\text{text}(txt))$$

**BLIP** extends this with visual question answering and image captioning, enabling robots to:
- Understand object properties from images
- Answer questions about scene content
- Generate descriptions of perceived scenes

### 3.3.5: Integration with Robot Control

ROS 2 nodes integrate these components:

1. **Perception node**: Publishes camera images, object detections
2. **Language node**: Subscribes to images, publishes interpretations
3. **Planning node**: Subscribes to language outputs, generates control commands
4. **Control node**: Executes motion primitives via actuators
