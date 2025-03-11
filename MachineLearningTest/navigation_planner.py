import numpy as np
import cv2
import math
from scipy.ndimage import gaussian_filter

class NavigationPlanner:
    """Class to plan navigation paths based on depth and detection data"""
    
    def __init__(self, robot_width=0.8, robot_length=0.8, safety_margin=0.3):
        """
        Initialize the navigation planner
        
        Args:
            robot_width: Width of the robot in meters
            robot_length: Length of the robot in meters
            safety_margin: Safety margin around obstacles in meters
        """
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.safety_margin = safety_margin
        
        # Navigation parameters
        self.depth_threshold = 1.0  # Minimum depth for navigable area (meters)
        self.max_planning_distance = 5.0  # Maximum planning distance (meters)
        
        # Define navigation goals and current path
        self.current_goal = None
        self.current_path = None
        
        # Create obstacle map
        self.obstacle_map = None
        self.map_resolution = 0.05  # meters per pixel
    
    def _depth_to_obstacle_map(self, depth_map, depth_scale=1.0):
        """
        Convert depth map to obstacle map
        
        Args:
            depth_map: Depth map (numpy array)
            depth_scale: Scale factor to convert depth values to meters
        
        Returns:
            Obstacle map (numpy array) where 0=free, 1=obstacle
        """
        if depth_map is None:
            return None
        
        # Create obstacle map
        obstacle_map = np.zeros_like(depth_map)
        
        # Mark areas with depth less than threshold as obstacles
        obstacle_map[depth_map < self.depth_threshold / depth_scale] = 1
        
        # Mark areas with no depth data as unknown (conservatively as obstacles)
        obstacle_map[depth_map <= 0] = 1
        
        # Apply dilation to ensure safety margin
        kernel_size = int(self.safety_margin / self.map_resolution)
        if kernel_size > 0:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            obstacle_map = cv2.dilate(obstacle_map, kernel, iterations=1)
        
        # Apply gaussian smoothing to create cost gradient around obstacles
        obstacle_map = gaussian_filter(obstacle_map, sigma=2)
        
        return obstacle_map
    
    def _add_detections_to_obstacle_map(self, obstacle_map, detections, camera_matrix=None):
        """
        Add detected objects to obstacle map
        
        Args:
            obstacle_map: Obstacle map (numpy array)
            detections: List of detection dictionaries
            camera_matrix: Camera intrinsic matrix for 3D projection
        
        Returns:
            Updated obstacle map
        """
        if obstacle_map is None or len(detections) == 0:
            return obstacle_map
        
        # Create a copy to modify
        updated_map = obstacle_map.copy()
        
        # For each detection, add a rectangular obstacle
        for det in detections:
            if det['class_name'] not in ['person', 'car', 'truck']:
                continue  # Only consider people or vehicles as obstacles for now
            
            # Get bounding box
            x1, y1, x2, y2 = det['box']
            
            # Calculate obstacle rectangle on the map
            # This is a simplified approach without proper 3D projection
            # In a real system, you would use camera intrinsics and depth
            # to project the detection into world coordinates
            
            center_x = (x1 + x2) // 2
            bottom_y = y2  # Assume bottom of the box is at ground level
            
            # Mark the area as obstacle
            rect_size = max(x2 - x1, y2 - y1) // 2
            cv2.rectangle(updated_map, 
                        (center_x - rect_size, bottom_y - rect_size * 2),
                        (center_x + rect_size, bottom_y),
                        1, -1)  # Fill with 1 (obstacle)
        
        return updated_map
    
    def plan_path(self, depth_maps, detections):
        """
        Plan a navigation path based on depth maps and detections
        
        Args:
            depth_maps: Dictionary of depth maps for each camera
            detections: Dictionary of detections for each camera
        
        Returns:
            Planned path as a list of waypoints
        """
        # For now, we'll just create a simple straight-line path
        # In a real system, you would use A* or RRT* for path planning
        
        # Create obstacle map by combining depth maps from all cameras
        combined_obstacle_map = None
        
        for camera_name, depth_map in depth_maps.items():
            if depth_map is None:
                continue
                
            # Get camera-specific detections
            camera_detections = detections.get(camera_name, [])
            
            # Create obstacle map for this camera
            obstacle_map = self._depth_to_obstacle_map(depth_map)
            
            # Add detected obstacles
            obstacle_map = self._add_detections_to_obstacle_map(obstacle_map, camera_detections)
            
            # Combine with overall map
            if combined_obstacle_map is None:
                combined_obstacle_map = obstacle_map
            else:
                # Take the maximum obstacle probability
                combined_obstacle_map = np.maximum(combined_obstacle_map, obstacle_map)
        
        # Store the combined obstacle map
        self.obstacle_map = combined_obstacle_map
        
        # For simulation, generate a simple test path
        # In the real implementation, this would be replaced with A* or RRT*
        if combined_obstacle_map is not None:
            h, w = combined_obstacle_map.shape
            start = (w//2, h-20)  # Bottom center
            goal = (w//2, 20)     # Top center
            
            # Very simple path - just a straight line
            path = []
            num_steps = 10
            for i in range(num_steps+1):
                t = i / num_steps
                x = int(start[0] * (1-t) + goal[0] * t)
                y = int(start[1] * (1-t) + goal[1] * t)
                path.append((x, y))
            
            self.current_path = path
            return path
        
        return []
    
    def visualize_plan(self, frame, path=None):
        """
        Visualize the navigation plan on an image
        
        Args:
            frame: Image to draw on
            path: Path to draw, if None uses current_path
        
        Returns:
            Frame with visualization
        """
        if frame is None:
            return None
            
        vis_frame = frame.copy()
        
        # Draw the obstacle map if available
        if self.obstacle_map is not None:
            # Resize to match frame
            h, w = frame.shape[:2]
            obstacle_vis = cv2.resize(self.obstacle_map, (w, h))
            
            # Convert to heatmap
            obstacle_vis = (obstacle_vis * 255).astype(np.uint8)
            obstacle_vis = cv2.applyColorMap(obstacle_vis, cv2.COLORMAP_JET)
            
            # Blend with original frame
            alpha = 0.3
            vis_frame = cv2.addWeighted(vis_frame, 1-alpha, obstacle_vis, alpha, 0)
        
        # Draw path if available
        path_to_draw = path if path is not None else self.current_path
        if path_to_draw:
            # Resize path points to match frame
            h, w = frame.shape[:2]
            obstacle_h, obstacle_w = self.obstacle_map.shape if self.obstacle_map is not None else (h, w)
            
            scaled_path = []
            for x, y in path_to_draw:
                scaled_x = int(x * w / obstacle_w)
                scaled_y = int(y * h / obstacle_h)
                scaled_path.append((scaled_x, scaled_y))
            
            # Draw path as connected line segments
            for i in range(len(scaled_path)-1):
                cv2.line(vis_frame, scaled_path[i], scaled_path[i+1], (0, 255, 0), 2)
                
            # Draw waypoints
            for x, y in scaled_path:
                cv2.circle(vis_frame, (x, y), 5, (0, 0, 255), -1)
        
        # Add text information
        cv2.putText(vis_frame, "Navigation Plan", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if path_to_draw:
            cv2.putText(vis_frame, f"Path Length: {len(path_to_draw)}", (10, 60), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return vis_frame

# Test function
def test_navigation_planner():
    """Test the navigation planner with a simulated depth map"""
    from camera_manager import CameraManager
    from depth_estimator import DepthEstimator
    from object_detector import ObjectDetector
    
    # Initialize components
    camera_manager = CameraManager(use_virtual_camera=True)
    camera_manager.initialize_cameras()
    depth_estimator = DepthEstimator()
    object_detector = ObjectDetector()
    navigation_planner = NavigationPlanner()
    
    try:
        while True:
            # Get camera frames
            main_frame = camera_manager.get_frame("VirtualCam")
            if main_frame is None:
                print("No frame from camera")
                continue
            
            # Create simulated multi-camera view
            fl, fr, bl, br = camera_manager.split_virtual_camera_frame(main_frame)
            
            # Get depth maps for each view
            fl_depth_map, fl_colored_depth = depth_estimator.estimate_depth(fl)
            
            # For this test, we'll just use the front left camera
            depth_maps = {"FrontLeft": fl_depth_map}
            
            # Get object detections
            detections, _ = object_detector.detect(fl)
            
            # Plan path using depth and detections
            navigation_planner.plan_path(depth_maps, {"FrontLeft": detections})
            
            # Visualize the navigation plan
            nav_vis = navigation_planner.visualize_plan(fl)
            
            # Show the visualization
            cv2.imshow("Navigation Plan", nav_vis)
            
            # Display the quad view
            quad_view = camera_manager.get_quad_display()
            if quad_view is not None:
                cv2.imshow("Camera Views", quad_view)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera_manager.stop_all()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_navigation_planner()