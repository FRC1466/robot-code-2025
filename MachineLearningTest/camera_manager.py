import cv2
import numpy as np
import time
from threading import Thread

class CameraFeed:
    """Class to manage a single camera feed"""
    def __init__(self, camera_id, name, position):
        """
        Initialize a camera feed
        
        Args:
            camera_id: Camera index or video file path
            name: Name identifier for the camera
            position: Position of camera (FL, FR, BL, BR)
        """
        self.camera_id = camera_id
        self.name = name
        self.position = position
        self.cap = None
        self.frame = None
        self.depth_frame = None
        self.running = False
        self.thread = None
        self.last_frame_time = 0
        self.fps = 0
    
    def start(self):
        """Start capturing from this camera in a separate thread"""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"Error: Cannot open camera {self.name} with ID {self.camera_id}")
            return False
        
        self.running = True
        self.thread = Thread(target=self._update_frame, daemon=True)
        self.thread.start()
        return True
    
    def _update_frame(self):
        """Thread function to continuously update frames"""
        prev_time = time.time()
        frame_count = 0
        
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
                self.last_frame_time = time.time()
                
                # Calculate FPS
                frame_count += 1
                current_time = time.time()
                elapsed = current_time - prev_time
                if elapsed >= 1.0:
                    self.fps = frame_count / elapsed
                    frame_count = 0
                    prev_time = current_time
            else:
                print(f"Warning: Failed to capture frame from camera {self.name}")
                time.sleep(0.1)  # Avoid busy-waiting if camera fails
    
    def stop(self):
        """Stop capturing from this camera"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
    
    def get_frame(self):
        """Get the latest frame from this camera"""
        return self.frame
    
    def set_depth_frame(self, depth_frame):
        """Set the depth frame corresponding to this camera"""
        self.depth_frame = depth_frame
    
    def get_depth_frame(self):
        """Get the depth frame for this camera"""
        return self.depth_frame

class CameraManager:
    """Class to manage multiple camera feeds"""
    def __init__(self, use_virtual_camera=True):
        self.cameras = {}
        self.use_virtual_camera = use_virtual_camera
    
    def initialize_cameras(self):
        """Initialize all cameras based on configuration"""
        if self.use_virtual_camera:
            # For simulation with OBS virtual camera, use a single camera and split it
            self.add_camera(1, "VirtualCam", "MAIN")
            print("Using OBS Virtual Camera for simulation")
            return True
        else:
            # For physical robot with four cameras
            success = True
            success &= self.add_camera(0, "FrontLeft", "FL")
            success &= self.add_camera(1, "FrontRight", "FR")
            success &= self.add_camera(2, "BackLeft", "BL")
            success &= self.add_camera(3, "BackRight", "BR")
            return success
    
    def add_camera(self, camera_id, name, position):
        """Add and start a camera"""
        camera = CameraFeed(camera_id, name, position)
        if camera.start():
            self.cameras[name] = camera
            return True
        return False
    
    def get_frame(self, camera_name):
        """Get the latest frame from a specific camera"""
        if camera_name in self.cameras:
            return self.cameras[camera_name].get_frame()
        return None
    
    def get_all_frames(self):
        """Get frames from all cameras as a dictionary"""
        return {name: cam.get_frame() for name, cam in self.cameras.items()}
    
    def split_virtual_camera_frame(self, frame):
        """
        Split a single virtual camera frame into four quadrants to simulate
        four separate camera feeds in simulation
        """
        if frame is None:
            return None, None, None, None
        
        height, width = frame.shape[:2]
        half_h, half_w = height // 2, width // 2
        
        fl = frame[:half_h, :half_w].copy()  # Front Left
        fr = frame[:half_h, half_w:].copy()  # Front Right
        bl = frame[half_h:, :half_w].copy()  # Back Left
        br = frame[half_h:, half_w:].copy()  # Back Right
        
        return fl, fr, bl, br
    
    def get_quad_display(self, include_depth=False):
        """
        Create a quad display of all cameras
        
        Args:
            include_depth: If True, include depth frames in the display
        
        Returns:
            A combined frame with all cameras
        """
        if self.use_virtual_camera:
            # Get the virtual camera frame
            frame = self.get_frame("VirtualCam")
            if frame is None:
                return None
            
            # Split the frame into simulated camera views
            fl, fr, bl, br = self.split_virtual_camera_frame(frame)
            
            # Create top and bottom rows
            if include_depth:
                fl_depth = self.cameras["VirtualCam"].get_depth_frame()
                if fl_depth is not None:
                    fl_depth = cv2.resize(fl_depth, (fl.shape[1], fl.shape[0]))
                    fl = np.hstack((fl, fl_depth))
                
                # In a real setup, we would include depth for all cameras
            
            top_row = np.hstack((fl, fr))
            bottom_row = np.hstack((bl, br))
            combined = np.vstack((top_row, bottom_row))
            
        else:
            # Get all camera frames
            frames = self.get_all_frames()
            
            # Make sure all frames exist and have the same size
            valid_frames = [f for f in frames.values() if f is not None]
            if not valid_frames:
                return None
            
            # Resize all frames to match the first valid frame
            target_size = valid_frames[0].shape[:2]
            resized_frames = {}
            for name, frame in frames.items():
                if frame is not None:
                    resized_frames[name] = cv2.resize(frame, (target_size[1], target_size[0]))
                else:
                    # Create a black frame with "No Signal" text
                    blank = np.zeros((target_size[0], target_size[1], 3), dtype=np.uint8)
                    cv2.putText(blank, f"No Signal - {name}", (30, target_size[0]//2),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    resized_frames[name] = blank
            
            # Create top and bottom rows
            top_row = np.hstack((resized_frames.get("FrontLeft", blank), 
                                resized_frames.get("FrontRight", blank)))
            bottom_row = np.hstack((resized_frames.get("BackLeft", blank), 
                                   resized_frames.get("BackRight", blank)))
            combined = np.vstack((top_row, bottom_row))
        
        return combined
    
    def stop_all(self):
        """Stop all camera feeds"""
        for camera in self.cameras.values():
            camera.stop()

# Test function to demonstrate the camera manager
def test_camera_manager():
    """Test the camera manager by displaying camera feeds"""
    manager = CameraManager(use_virtual_camera=True)
    manager.initialize_cameras()
    
    try:
        while True:
            quad_view = manager.get_quad_display()
            if quad_view is not None:
                cv2.imshow("Camera Feeds", quad_view)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        manager.stop_all()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera_manager()