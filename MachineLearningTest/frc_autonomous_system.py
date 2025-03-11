#!/usr/bin/env python3
"""
FRC 2025 Autonomous Navigation System
=====================================
This module integrates camera feeds, depth estimation, object detection,
and navigation planning to create an autonomous driving system for FRC robots.

Currently in simulation mode using OBS Virtual Camera input.

Installation:
------------
1. Install Mambaforge from https://github.com/conda-forge/miniforge#mambaforge
2. Run the setup script:
   - On Windows: setup_mamba_env.bat
   - On Linux/macOS: bash setup_mamba_env.sh
3. Activate the environment: mamba activate frc-ml

Usage:
------
python frc_autonomous_system.py [--config CONFIG] [--simulation] [--no-display]
"""

import os
import cv2
import numpy as np
import time
import argparse
from threading import Thread
import json

# Import our custom modules
from camera_manager import CameraManager
from depth_estimator import DepthEstimator
from object_detector import ObjectDetector
from navigation_planner import NavigationPlanner

# For FRC-specific game piece analysis
GAME_PIECE_CLASSES = ['sports ball']  # Stand-in for game pieces until custom model trained
ROBOT_CLASSES = ['car', 'truck']  # Stand-in for other robots

class FRCAutonomousSystem:
    """Main class for FRC autonomous navigation system"""
    
    def __init__(self, config=None):
        """
        Initialize the autonomous system
        
        Args:
            config: Configuration dictionary or path to config file
        """
        # Load configuration
        self.config = self._load_config(config)
        
        # Initialize system state
        self.running = False
        self.simulation_mode = self.config.get('simulation_mode', True)
        self.visualize = self.config.get('visualize', True)
        self.enable_logging = self.config.get('enable_logging', False)
        
        # Initialize subsystems
        self.camera_manager = CameraManager(use_virtual_camera=self.simulation_mode)
        self.depth_estimator = DepthEstimator(
            use_cuda=self.config.get('use_cuda', True)
        )
        self.object_detector = ObjectDetector(
            model_path=self.config.get('object_detection_model', None),
            confidence=self.config.get('detection_confidence', 0.25)
        )
        self.navigation_planner = NavigationPlanner(
            robot_width=self.config.get('robot_width', 0.8),
            robot_length=self.config.get('robot_length', 0.8),
            safety_margin=self.config.get('safety_margin', 0.3)
        )
        
        # Initialize data containers
        self.camera_frames = {}
        self.depth_maps = {}
        self.colored_depth_maps = {}
        self.detections = {}
        self.frc_detections = {
            'robots': [],
            'game_pieces': [],
            'humans': [],
            'others': []
        }
        self.current_path = []
        
        # Performance metrics
        self.fps = 0
        self.last_fps_update = time.time()
        self.frame_count = 0
        
        # For connecting to FRC robot control
        self.robot_interface = None
        
    def _load_config(self, config):
        """Load configuration from file or dictionary"""
        if config is None:
            # Default configuration
            return {
                'simulation_mode': True,
                'visualize': True,
                'use_cuda': True,
                'detection_confidence': 0.25,
                'robot_width': 0.8,  # meters
                'robot_length': 0.8,  # meters
                'safety_margin': 0.3,  # meters
                'enable_logging': False
            }
        elif isinstance(config, str):
            # Load from file
            try:
                with open(config, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"Error loading config file: {str(e)}")
                return self._load_config(None)
        else:
            # Assume config is a dictionary
            return config
        
    def initialize(self):
        """Initialize all subsystems"""
        # Initialize cameras
        success = self.camera_manager.initialize_cameras()
        if not success:
            print("Failed to initialize cameras")
            return False
        
        print("System initialized and ready")
        return True
        
    def run(self):
        """Run the main processing loop"""
        self.running = True
        self.last_fps_update = time.time()
        self.frame_count = 0
        
        while self.running:
            start_time = time.time()
            
            # Process one frame
            self._process_frame()
            
            # Calculate FPS
            self.frame_count += 1
            if time.time() - self.last_fps_update >= 1.0:
                self.fps = self.frame_count / (time.time() - self.last_fps_update)
                self.frame_count = 0
                self.last_fps_update = time.time()
                print(f"FPS: {self.fps:.2f}")
                
            # Sleep to maintain desired frame rate
            elapsed = time.time() - start_time
            target_frame_time = 1.0 / 30  # Target 30 FPS
            if elapsed < target_frame_time:
                time.sleep(target_frame_time - elapsed)
            
            # Check for key presses if visualization is enabled
            if self.visualize:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.running = False
                elif key == ord('s'):
                    self._save_current_frame()
                    
    def stop(self):
        """Stop the system and release resources"""
        self.running = False
        self.camera_manager.stop_all()
        cv2.destroyAllWindows()
        print("System stopped")
        
    def _process_frame(self):
        """Process a single frame from all cameras"""
        # Get frames from all cameras
        if self.simulation_mode:
            # In simulation, we get one frame and split it
            main_frame = self.camera_manager.get_frame("VirtualCam")
            if main_frame is None:
                return
                
            # Split into four views
            fl, fr, bl, br = self.camera_manager.split_virtual_camera_frame(main_frame)
            self.camera_frames = {
                "FrontLeft": fl,
                "FrontRight": fr,
                "BackLeft": bl,
                "BackRight": br
            }
        else:
            # In real mode, get frames from all cameras
            self.camera_frames = self.camera_manager.get_all_frames()
            
        # Process each camera view
        self.depth_maps = {}
        self.colored_depth_maps = {}
        self.detections = {}
        
        # For now, we'll only process the front left camera for performance
        # In a full implementation, you would process all cameras, potentially in parallel
        if "FrontLeft" in self.camera_frames and self.camera_frames["FrontLeft"] is not None:
            # Get depth map
            depth_map, colored_depth = self.depth_estimator.estimate_depth(
                self.camera_frames["FrontLeft"]
            )
            self.depth_maps["FrontLeft"] = depth_map
            self.colored_depth_maps["FrontLeft"] = colored_depth
            
            # Get object detections
            detections, annotated_frame = self.object_detector.detect(
                self.camera_frames["FrontLeft"]
            )
            self.detections["FrontLeft"] = detections
            
            # Store the annotated frame for visualization
            self.camera_frames["FrontLeft_Annotated"] = annotated_frame
            
        # Combine detections from all cameras
        all_detections = []
        for camera_detections in self.detections.values():
            all_detections.extend(camera_detections)
            
        # Categorize FRC-specific detections
        self.frc_detections = self.object_detector.get_frc_detections(all_detections)
        
        # Plan navigation
        self.current_path = self.navigation_planner.plan_path(self.depth_maps, self.detections)
        
        # Visualize results
        if self.visualize:
            self._update_visualization()
            
    def _update_visualization(self):
        """Update visualization windows"""
        # Create quad view of all cameras
        quad_view = self.camera_manager.get_quad_display()
        if quad_view is not None:
            cv2.putText(quad_view, f"FPS: {self.fps:.1f}", (10, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Camera Views", quad_view)
            
        # Show navigation plan if available
        if "FrontLeft" in self.camera_frames:
            fl_frame = self.camera_frames["FrontLeft"]
            if fl_frame is not None:
                nav_vis = self.navigation_planner.visualize_plan(fl_frame)
                cv2.imshow("Navigation Plan", nav_vis)
                
        # Show object detections
        if "FrontLeft_Annotated" in self.camera_frames:
            # Add FRC-specific information
            annotated = self.camera_frames["FrontLeft_Annotated"].copy()
            
            # Display counts
            cv2.putText(
                annotated,
                f"Robots: {len(self.frc_detections['robots'])}, Game Pieces: {len(self.frc_detections['game_pieces'])}", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )
            
            cv2.imshow("Object Detection", annotated)
            
        # Show depth map if available
        if "FrontLeft" in self.colored_depth_maps and self.colored_depth_maps["FrontLeft"] is not None:
            cv2.imshow("Depth Map", self.colored_depth_maps["FrontLeft"])
            
    def _save_current_frame(self):
        """Save current frame and processing results for debugging"""
        if not os.path.exists("captures"):
            os.makedirs("captures")
            
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        
        # Save camera view
        if "FrontLeft" in self.camera_frames:
            cv2.imwrite(f"captures/camera_{timestamp}.png", self.camera_frames["FrontLeft"])
            
        # Save annotated view
        if "FrontLeft_Annotated" in self.camera_frames:
            cv2.imwrite(f"captures/detection_{timestamp}.png", self.camera_frames["FrontLeft_Annotated"])
            
        # Save depth map
        if "FrontLeft" in self.colored_depth_maps and self.colored_depth_maps["FrontLeft"] is not None:
            cv2.imwrite(f"captures/depth_{timestamp}.png", self.colored_depth_maps["FrontLeft"])
            
        # Save navigation plan
        if "FrontLeft" in self.camera_frames:
            nav_vis = self.navigation_planner.visualize_plan(self.camera_frames["FrontLeft"])
            cv2.imwrite(f"captures/navigation_{timestamp}.png", nav_vis)
            
        print(f"Saved current frame captures with timestamp {timestamp}")

def main():
    """Main entry point"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='FRC Autonomous Navigation System')
    parser.add_argument('--config', type=str, help='Path to config file')
    parser.add_argument('--simulation', action='store_true', help='Run in simulation mode')
    parser.add_argument('--no-display', action='store_true', help='Disable visualization')
    args = parser.parse_args()
    
    # Override config with command line arguments
    config = None
    if args.config:
        config = args.config
        
    # Create autonomous system
    system = FRCAutonomousSystem(config)
    
    # Apply command line overrides
    if args.simulation:
        system.config['simulation_mode'] = True
    if args.no_display:
        system.config['visualize'] = False
    
    # Initialize the system
    if not system.initialize():
        print("Failed to initialize autonomous system")
        return
    
    try:
        # Run the system
        system.run()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        system.stop()

if __name__ == "__main__":
    main()