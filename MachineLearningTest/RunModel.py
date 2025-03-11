#!/usr/bin/env python3
"""
ZoeDepth Model Testing for FRC
==============================
This is a simplified entry point for testing the depth estimation model.
For the full autonomous navigation system, use frc_autonomous_system.py instead.

Usage:
------
python RunModel.py [--camera CAMERA_INDEX] [--no-cuda] [--model {N,K,NK}]
"""

import cv2
import numpy as np
import time
import argparse
import os
from depth_estimator import DepthEstimator

def try_open_camera(index):
    """Try to open camera with given index"""
    print(f"Attempting to open camera {index}...")
    cap = cv2.VideoCapture(index)
    success = cap.isOpened()
    if success:
        print(f"Successfully opened camera {index}")
    else:
        print(f"Failed to open camera {index}")
    return cap, success

def main():
    """Main function for running the depth model test"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="ZoeDepth Model Test for FRC")
    parser.add_argument("--camera", type=int, default=0, help="Camera index to use (default: 0)")
    parser.add_argument("--no-cuda", action="store_true", help="Disable CUDA (use CPU only)")
    parser.add_argument("--save-dir", type=str, default="captures", help="Directory to save captures")
    parser.add_argument("--download-dir", type=str, default="model_cache", help="Directory to store model weights")
    parser.add_argument("--model", type=str, default="N", choices=["N", "K", "NK"], 
                        help="ZoeDepth model type: N, K, or NK (default: N)")
    args = parser.parse_args()
    
    print("Starting ZoeDepth model test...")
    print(f"CUDA enabled: {not args.no_cuda}")
    print(f"Model type: ZoeD_{args.model}")
    
    # Initialize depth estimator with model download directory
    depth_estimator = DepthEstimator(
        use_cuda=not args.no_cuda,
        download_dir=args.download_dir,
        model_type=args.model
    )
    
    # Open camera with requested index
    cap, success = try_open_camera(args.camera)
    
    # If the requested camera failed, try some alternatives
    if not success:
        print("Trying alternative camera indices...")
        for alt_index in [1, 2, 0]:  # Try these camera indices
            if alt_index == args.camera:
                continue  # Skip the one we already tried
            
            cap, success = try_open_camera(alt_index)
            if success:
                print(f"Using camera index {alt_index} instead")
                break
    
    if not success:
        print("Could not open any camera. Please check your camera connections.")
        return
    
    # Create capture directory if it doesn't exist
    if not os.path.exists(args.save_dir):
        os.makedirs(args.save_dir)
        print(f"Created directory {args.save_dir} for saving captures")
    
    # Initialize FPS calculation
    fps = 0
    frame_count = 0
    start_time = time.time()
    
    print("Press 'q' to quit, 's' to save current frame")
    
    try:
        while True:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame. Camera may be disconnected.")
                break
            
            # Estimate depth
            depth_map, colored_depth, grayscale_depth = depth_estimator.estimate_depth(frame)
            
            if colored_depth is not None and grayscale_depth is not None:
                # Resize all images to have the same height for display
                h, w = frame.shape[:2]
                grayscale_depth = cv2.resize(grayscale_depth, (w, h))
                colored_depth = cv2.resize(colored_depth, (w, h))
                
                # Add labels to the images
                cv2.putText(frame, "RGB", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(grayscale_depth, "Depth (Gray)", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(colored_depth, "Depth (Color)", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # Create multi-view display
                # Top row: RGB and Grayscale depth side by side
                top_row = np.hstack((frame, grayscale_depth))
                
                # Bottom row: Colored depth (resized to match top row width)
                colored_depth_resized = cv2.resize(colored_depth, (top_row.shape[1], colored_depth.shape[0]))
                
                # Combine rows
                display_img = np.vstack((top_row, colored_depth_resized))
                
                # Calculate FPS
                frame_count += 1
                if frame_count >= 10:
                    current_time = time.time()
                    fps = frame_count / (current_time - start_time)
                    frame_count = 0
                    start_time = current_time
                
                # Add FPS information
                cv2.putText(display_img, f"FPS: {fps:.1f}", (display_img.shape[1] - 150, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_img, f"Model: ZoeD_{args.model}", (display_img.shape[1] - 150, 60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_img, "Press 'q' to quit, 's' to save", (10, display_img.shape[0] - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow("RGB and Depth", display_img)
                
            else:
                # If depth estimation failed, just show the RGB frame
                cv2.putText(frame, "Depth estimation failed", (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, "Check console output for error details", (10, 60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("RGB Only", frame)
            
            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                
                # Save all views
                rgb_path = os.path.join(args.save_dir, f"rgb_{timestamp}.png")
                cv2.imwrite(rgb_path, frame)
                print(f"Saved RGB frame to {rgb_path}")
                
                if grayscale_depth is not None:
                    gray_depth_path = os.path.join(args.save_dir, f"depth_gray_{timestamp}.png")
                    cv2.imwrite(gray_depth_path, grayscale_depth)
                    print(f"Saved grayscale depth to {gray_depth_path}")
                
                if colored_depth is not None:
                    color_depth_path = os.path.join(args.save_dir, f"depth_color_{timestamp}.png")
                    cv2.imwrite(color_depth_path, colored_depth)
                    print(f"Saved colored depth to {color_depth_path}")
    
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("ZoeDepth model test terminated")

if __name__ == "__main__":
    main()
