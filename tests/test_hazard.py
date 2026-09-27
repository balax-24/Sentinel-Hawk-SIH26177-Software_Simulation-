"""Unit tests for modular disaster hazard detection."""

import numpy as np
import cv2
import pytest

from perception.sih_perception.sih_perception.hazard_detector_node import (
    FireHazardDetector,
    SmokeHazardDetector,
    StructuralDamageDetector,
    ChemicalLeakDetector,
    HazardDetectorNode,
)


def test_fire_hazard_detection():
    """Verify fire detector identifies active flame signatures."""
    detector = FireHazardDetector()
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    # Bright fire patch (yellow-orange in BGR)
    cv2.rectangle(img, (50, 50), (100, 100), (0, 215, 255), -1)

    hazards = detector.detect(img, obstacles=[])
    assert len(hazards) > 0
    assert hazards[0]["hazard_type"] == "FIRE"
    assert hazards[0]["severity"] == "CRITICAL"
    assert hazards[0]["confidence"] > 0.7


def test_structural_damage_detection():
    """Verify structural damage detector identifies collapsed rubble from obstacles."""
    detector = StructuralDamageDetector()
    obstacles = [
        {"position": (5.0, 5.0, 0.4), "dimensions": (4.0, 3.0, 0.5), "confidence": 0.90},
    ]

    hazards = detector.detect(bgr_image=None, obstacles=obstacles)
    assert len(hazards) == 1
    assert hazards[0]["hazard_type"] == "DAMAGED_STRUCTURE"
    assert hazards[0]["severity"] == "HIGH"


def test_chemical_leak_detection():
    """Verify chemical detector identifies toxic fluid discoloration."""
    detector = ChemicalLeakDetector()
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    # Fluorescent green patch in BGR
    cv2.rectangle(img, (40, 40), (110, 110), (0, 240, 60), -1)

    hazards = detector.detect(img, obstacles=[])
    assert len(hazards) > 0
    assert hazards[0]["hazard_type"] == "CHEMICAL_LEAK"
    assert hazards[0]["severity"] == "CRITICAL"


def test_hazard_detector_node_standalone():
    """Verify full hazard node orchestration and spatial deduplication."""
    node = HazardDetectorNode(uav_id="uav_test_1")
    img = np.zeros((300, 300, 3), dtype=np.uint8)
    cv2.rectangle(img, (50, 50), (120, 120), (0, 215, 255), -1)

    node._run_detection_pipeline(img)
    assert len(node.tracked_hazards) >= 1
    h = list(node.tracked_hazards.values())[0]
    assert h["source_uav"] == "uav_test_1"
