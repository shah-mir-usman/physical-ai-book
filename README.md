# Physical AI & Humanoid Robotics: The Encyclopedia of Embodied Intelligence

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![Python Version](https://img.shields.io/badge/python-3.10%2B-blue)
![Deployment](https://img.shields.io/badge/deployment-vercel-black)

**LIVE TEXTBOOK:** [https://physical-ai-book-en2t.vercel.app](https://physical-ai-book-en2t.vercel.app)

---

## Project Overview

This project is a comprehensive **24-Chapter Digital Encyclopedia** developed for the **Panaversity & GIAIC Hackathon**. It bridges the critical gap between **Digital AI** (LLMs, Transformers) and **Physical AI** (Robotics, Control Theory).

The development of this platform was accelerated using **Claude CLI** and **Spec Kit Plus**, enabling rapid prototyping of complex architectures. Beyond a standard static resource, this platform has been engineered as an **Interactive Intelligent System**. It integrates **Retrieval-Augmented Generation (RAG)** to provide a real-time AI Tutor capable of answering complex queries strictly based on the provided curriculum while maintaining the ability to handle general conversational context via Large Language Models.

Designed as a post-graduate level resource, it guides engineers from the mathematical foundations of Inverse Kinematics to deploying Vision-Language-Action (VLA) models on humanoid robots using **ROS 2** and **NVIDIA Isaac Sim**.

---

## Interactive AI Features

### 1. Hybrid AI Reasoning Engine
The application implements a dual-layer logic system to ensure accuracy and usability:
* **Context-Strict Mode:** For curriculum-related queries, the system retrieves specific data chunks from the Vector Database (Qdrant) to prevent hallucinations and ensure factual precision based on the textbook.
* **General Knowledge Fallback:** For greetings or general interactions, the system bypasses retrieval to function as a natural conversational agent using Google Gemini.

### 2. Context-Aware Text Selection
A distinct feature allowing users to highlight specific paragraphs within the textbook. The AI detects the selection and prioritizes it as the primary context, offering targeted explanations for specific technical concepts.

---

## Curriculum Architecture
The textbook is organized into **6 Advanced Modules**:

| Module | Focus Area | Key Technologies |
| :--- | :--- | :--- |
| **I** | **Mathematical Foundations** | Linear Algebra, Kinematics, Dynamics, Control Theory (PID/MPC) |
| **II** | **The Nervous System** | ROS 2 (Humble), DDS Middleware, Real-Time Linux (PREEMPT_RT) |
| **III** | **The Digital Twin** | Gazebo, Unity, Physics Engines, URDF/SDF Modeling |
| **IV** | **Perception** | VSLAM, Sensor Fusion (EKF), Point Clouds, NVIDIA Isaac ROS |
| **V** | **Navigation & Control** | Nav2 Stack, Path Planning (A*), Bipedal Locomotion |
| **VI** | **Cognitive Robotics** | Generative AI, Vision Transformers (ViT), VLA Models, Voice Pipelines |

---

## Technical Architecture

The system operates on a decoupled architecture where the React frontend communicates with a Python FastAPI backend via RESTful API endpoints.

```
graph TD;
    User[User Interface] -->|HTTP POST /chat| API[FastAPI Gateway];
    API -->|Text Input| Embedding[Google Gemini Embeddings];
    Embedding -->|Vector (768 dim)| VectorDB[Qdrant Cloud];
    VectorDB -->|Top 3 Semantic Matches| API;
    API -->|Construct Prompt + Context| LLM[Gemini 2.5 Flash];
    LLM -->|Generated Response| API;
    API -->|JSON Response| User;
```
---
## Technology Stack

**Component  |	 Technology   |	  Description**
| :--- | :--- | :--- |

**Frontend Framework** | Docusaurus 3 (React) | Static Site Generator with custom Glassmorphism styling |
**Backend API**	| Python / FastAPI	| Asynchronous server for handling AI logic |
**AI Model** | Google Gemini 2.5 Flash | Large Language Model for reasoning |
**Vector Database** | Qdrant (Cloud) | Storing and retrieving high-dimensional textbook embeddings |
**Deployment** | Vercel	| Monorepo deployment handling both Python and Static assets |
**Math Rendering** | KaTeX / MDX Rendering complex mathematical formulas |

---

## Author

**Shah Mir Usman**

Aspiring Cybersecurity Professional & Agentic AI Architect|Panaversity & GIAIC Hackathon Project


**Built for the Future of Embodied Intelligence.**
