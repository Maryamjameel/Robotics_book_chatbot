"""Pytest configuration and fixtures for embedding pipeline tests."""

import json
import os
import tempfile
from pathlib import Path
from typing import Generator, List
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def temp_chapters_dir() -> Generator[str, None, None]:
    """Create temporary directory with sample chapter markdown files."""
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create sample chapter 1
        ch01_content = """# Introduction to Robotics

## Overview

Robotics is an interdisciplinary field that combines mechanical engineering, electrical engineering, computer science, and control theory. It involves designing, building, programming, and testing robots.

## History

The word "robot" comes from the Czech word "robota" meaning "forced labor". The first industrial robot was installed by Unimation in 1961 at a General Motors factory.

## Applications

Robots are used in manufacturing, healthcare, exploration, entertainment, and research. They can perform tasks that are dangerous, repetitive, or require high precision."""

        ch02_content = """# Kinematics and Motion Planning

## Forward Kinematics

Forward kinematics is the problem of computing the position and orientation of the end-effector of a robot manipulator given the joint angles. The transformation from joint space to Cartesian space is defined by the robot's kinematic structure.

## Inverse Kinematics

Inverse kinematics solves the reverse problem: given a desired end-effector position and orientation in Cartesian space, compute the required joint angles. This is more complex because the solution may not be unique or may not exist.

## Denavit-Hartenberg Convention

The DH convention provides a systematic way to assign coordinate frames to robot links and describe the kinematic parameters of a manipulator. It uses four parameters per joint: link length, link twist, joint offset, and joint angle."""

        ch03_content = """# Control Systems

## PID Control

Proportional-Integral-Derivative (PID) control is a control loop feedback mechanism widely used in robotics. The PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable.

## Feedback Control

Feedback control systems use sensors to measure the current state and adjust actuators to reach a desired goal state. This enables robots to adapt to disturbances and uncertainties in their environment."""

        # Write files
        Path(tmpdir, "ch01.md").write_text(ch01_content)
        Path(tmpdir, "ch02.md").write_text(ch02_content)
        Path(tmpdir, "ch03.md").write_text(ch03_content)

        yield tmpdir


@pytest.fixture
def mock_openai_client() -> MagicMock:
    """Create a mock OpenAI client for testing embedding generation."""
    mock_client = MagicMock()

    # Mock embedding response
    def mock_create(*args, **kwargs):
        texts = kwargs.get("input", [])
        if isinstance(texts, str):
            texts = [texts]

        # Create mock embeddings (1536 dimensions of small random values)
        embeddings = []
        for i, text in enumerate(texts):
            embedding = [(i + j) * 0.001 for j in range(1536)]
            embeddings.append({"embedding": embedding, "index": i})

        return type('obj', (object,), {
            'data': embeddings,
            'model': kwargs.get('model', 'text-embedding-3-small'),
            'usage': {'prompt_tokens': sum(len(t.split()) for t in texts), 'total_tokens': 1000}
        })()

    mock_client.embeddings.create = mock_create
    return mock_client


@pytest.fixture
def mock_qdrant_client() -> MagicMock:
    """Create a mock Qdrant client for testing vector storage."""
    mock_client = MagicMock()

    # Store points in memory for testing
    stored_points = {}

    def mock_create_collection(*args, **kwargs):
        return True

    def mock_collection_exists(*args, **kwargs):
        return True

    def mock_upsert(*args, **kwargs):
        collection_name = args[0] if args else kwargs.get('collection_name')
        points = kwargs.get('points', [])
        for point in points:
            stored_points[point.id] = point
        return type('obj', (object,), {'operation_id': 1})()

    def mock_get_collection(*args, **kwargs):
        return type('obj', (object,), {
            'points_count': len(stored_points),
            'config': type('obj', (object,), {
                'vector_size': 1536,
                'distance': 'cosine'
            })()
        })()

    def mock_search(*args, **kwargs):
        query_vector = args[1] if len(args) > 1 else kwargs.get('query_vector')
        limit = kwargs.get('limit', 5)

        # Return mock search results
        results = []
        for point_id in list(stored_points.keys())[:limit]:
            point = stored_points[point_id]
            results.append(type('obj', (object,), {
                'id': point_id,
                'score': 0.85,
                'payload': getattr(point, 'payload', {})
            })())
        return results

    mock_client.create_collection = mock_create_collection
    mock_client.collection_exists = mock_collection_exists
    mock_client.upsert = mock_upsert
    mock_client.get_collection = mock_get_collection
    mock_client.search = mock_search

    return mock_client


@pytest.fixture
def sample_chapters() -> List[dict]:
    """Create sample chapter data for testing."""
    return [
        {
            "chapter_id": "ch01",
            "chapter_title": "Introduction to Robotics",
            "sections": [
                {"title": "Overview", "content": "Robotics is..."},
                {"title": "History", "content": "The word robot..."},
                {"title": "Applications", "content": "Robots are used..."}
            ]
        },
        {
            "chapter_id": "ch02",
            "chapter_title": "Kinematics",
            "sections": [
                {"title": "Forward Kinematics", "content": "Forward kinematics is..."},
                {"title": "Inverse Kinematics", "content": "Inverse kinematics is..."}
            ]
        }
    ]
