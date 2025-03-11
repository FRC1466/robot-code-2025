import torch
import cv2
import numpy as np
import time
import os
import traceback
from PIL import Image
import open3d as o3d

class DepthEstimator:
    """Class to estimate depth from RGB images using ZoeDepth"""
    
    def __init__(self, use_cuda=True, download_dir=None, model_type="N", depth_scale=0.5):
        """
        Initialize the depth estimator
        
        Args:
            use_cuda: Whether to use CUDA for inference if available
            download_dir: Directory to download model weights
            model_type: ZoeDepth model type: "N", "K", or "NK"
            depth_scale: Scale factor for depth values in point cloud (smaller = less stretched)
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() and use_cuda else "cpu")
        print(f"Using device: {self.device} for depth estimation")
        
        # Set download directory for model weights if specified
        if download_dir:
            os.makedirs(download_dir, exist_ok=True)
            torch.hub.set_dir(download_dir)
            print(f"Using {download_dir} for model downloads")
        
        self.model_type = model_type
        self.depth_scale = depth_scale  # Scale factor for depth in point cloud
        
        # Initialize point cloud visualization
        self.point_cloud = None
        self.vis = None
        self.time_offset = 0  # For subtle animation effect
        
        # Load ZoeDepth model
        try:
            print(f"Loading ZoeDepth model type: ZoeD_{model_type}")
            self.model = self._load_zoe_model(model_type)
            self.model_loaded = True
            self.using_zoe = True
            print("ZoeDepth model loaded successfully")
        except Exception as e:
            print(f"Error loading ZoeDepth model: {str(e)}")
            print("Trying MiDaS model as fallback...")
            try:
                self.model = self._load_midas_fallback(high_quality=True)
                self.model_loaded = True
                self.using_zoe = False
                self.using_midas = True
                print("Using high-quality MiDaS model as fallback")
            except Exception as e2:
                print(f"High-quality MiDaS loading failed: {str(e2)}")
                print("Trying smaller MiDaS model...")
                try:
                    self.model = self._load_midas_fallback(high_quality=False)
                    self.model_loaded = True
                    self.using_zoe = False
                    self.using_midas = True
                    print("Using smaller MiDaS model as fallback")
                except Exception as e3:
                    print(f"All model loading attempts failed: {str(e3)}")
                    self.model_loaded = False
                    self.using_zoe = False
                    self.using_midas = False
    
    def _load_zoe_model(self, model_type):
        """Load ZoeDepth model using the official method"""
        repo = "isl-org/ZoeDepth"
        
        if model_type == "N":
            model = torch.hub.load(repo, "ZoeD_N", pretrained=True, trust_repo=True)
        elif model_type == "K":
            model = torch.hub.load(repo, "ZoeD_K", pretrained=True, trust_repo=True)
        elif model_type == "NK":
            model = torch.hub.load(repo, "ZoeD_NK", pretrained=True, trust_repo=True)
        else:
            # Default to N model if invalid type specified
            model = torch.hub.load(repo, "ZoeD_N", pretrained=True, trust_repo=True)
        
        model.to(self.device).eval()
        return model
    
    def _load_midas_fallback(self, high_quality=True):
        """Load MiDaS model as a fallback"""
        # Load MiDaS model - use high quality DPT_Large model if requested
        if high_quality:
            print("Loading high-quality MiDaS DPT_Large model...")
            model = torch.hub.load("intel-isl/MiDaS", "DPT_Large", pretrained=True, trust_repo=True)
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.midas_transform = midas_transforms.dpt_transform
        else:
            print("Loading smaller MiDaS model...")
            model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", pretrained=True, trust_repo=True)
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.midas_transform = midas_transforms.small_transform
        
        model.to(self.device).eval()
        return model
    
    def _create_point_cloud(self, rgb_image, depth_map, downsample_factor=1):
        """
        Create a 3D point cloud from RGB image and depth map
        """
        if rgb_image is None or depth_map is None:
            return None
            
        # Get dimensions
        h, w = depth_map.shape[:2]
        
        # Use higher resolution for better detail
        h_sample = max(1, int(h * 0.9))  # Sample 90% of pixels
        w_sample = max(1, int(w * 0.9))
        y, x = np.mgrid[0:h:h/h_sample, 0:w:w/w_sample]
        
        # Get corresponding depth values
        z = depth_map[y.astype(np.int32), x.astype(np.int32)].copy()
        
        # Enhanced depth normalization with outlier removal
        valid_depth = z > 0
        if np.any(valid_depth):
            z_valid = z[valid_depth]
            z_mean = np.mean(z_valid)
            z_std = np.std(z_valid)
            z_min = max(0, z_mean - 2 * z_std)
            z_max = z_mean + 2 * z_std
            z = np.clip(z, z_min, z_max)
            z = (z - z_min) / (z_max - z_min) * self.depth_scale
        
        # Project to 3D coordinates with enhanced parameters
        fx = w * 1.2  # Increased focal length reduces perspective distortion
        fy = h * 1.2
        cx = w/2
        cy = h/2
        
        # Create 3D points with optimized scaling
        x_3d = (x - cx) * z / fx * 1.5  # Adjusted scaling factors
        y_3d = (y - cy) * z / fy * 1.5
        z_3d = -z - 0.2  # Offset from camera
        
        # Stack coordinates
        xyz = np.stack([x_3d, y_3d, z_3d], axis=-1)
        xyz = xyz.reshape(-1, 3).astype(np.float64)
        
        # Get corresponding colors with proper indexing
        colors = cv2.resize(rgb_image, (w, h))[y.astype(np.int32), x.astype(np.int32)] / 255.0
        colors = colors.reshape(-1, 3).astype(np.float64)
        
        # Create point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd
        
    def visualize_point_cloud(self, rgb_image, depth_map):
        """
        Visualize RGB point cloud from depth and color data
        """
        if rgb_image is None or depth_map is None:
            return False
            
        try:
            # Create new point cloud with higher resolution
            pcd = self._create_point_cloud(rgb_image, depth_map, downsample_factor=1)
            if pcd is None:
                return False
                
            # Initialize visualizer if needed
            if self.vis is None:
                self.vis = o3d.visualization.Visualizer()
                self.vis.create_window("3D Depth View", width=1024, height=768)
                
                # Add initial point cloud
                self.point_cloud = pcd
                self.vis.add_geometry(self.point_cloud)
                
                # Set default view
                ctr = self.vis.get_view_control()
                ctr.set_zoom(0.8)
                ctr.set_front([0, 0, -1])
                ctr.set_lookat([0, 0, 0])
                ctr.set_up([0, -1, 0])
                
                # Set render options with smaller point size for higher density
                opt = self.vis.get_render_option()
                opt.point_size = 0.5  # Reduced from 1.0 for higher density
                opt.background_color = np.asarray([0, 0, 0])
                
                self.time_offset = time.time()
            else:
                # Update points and colors
                self.point_cloud.points = pcd.points
                self.point_cloud.colors = pcd.colors
                
                # Explicitly update geometry
                self.vis.update_geometry(self.point_cloud)
                
            # Update view
            self.vis.poll_events()
            self.vis.update_renderer()
            
            return True
            
        except Exception as e:
            print(f"Error in point cloud visualization: {str(e)}")
            traceback.print_exc()
            return False
    
    def estimate_depth(self, frame):
        """
        Estimate depth from an RGB image
        
        Args:
            frame: RGB or BGR image (numpy array)
        
        Returns:
            Tuple of (depth_map, colored_depth, grayscale_depth) where:
                - depth_map is the raw depth values (numpy array)
                - colored_depth is a visualization of the depth map (BGR image)
                - grayscale_depth is a grayscale visualization (BGR image)
        """
        if not self.model_loaded or frame is None:
            return None, None, None
        
        # Start timing
        start_time = time.time()
        
        try:
            # Convert to RGB if it's BGR
            if frame.shape[2] == 3:
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                img_rgb = frame
                
            if self.using_zoe:
                # Convert to PIL Image
                pil_image = Image.fromarray(img_rgb)
                
                # Using the recommended inference method
                depth_np = self.model.infer_pil(pil_image)
                
            else:  # Using MiDaS fallback
                # Apply MiDaS transform
                input_batch = self.midas_transform(img_rgb).to(self.device)
                
                # Run inference
                with torch.no_grad():
                    prediction = self.model(input_batch)
                    prediction = torch.nn.functional.interpolate(
                        prediction.unsqueeze(1),
                        size=frame.shape[:2],
                        mode="bicubic",
                        align_corners=False,
                    ).squeeze()
                
                # Get depth map
                depth_np = prediction.cpu().numpy()
            
            # Remove any NaN values or infinities
            depth_np = np.nan_to_num(depth_np, nan=0.0, posinf=10.0, neginf=0.0)
            
            # Normalize depth for visualization (min-max normalization)
            depth_norm = cv2.normalize(depth_np, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            
            # Create grayscale visualization
            grayscale_depth = cv2.cvtColor(depth_norm, cv2.COLOR_GRAY2BGR)
            
            # Create a colored visualization of the depth map
            if self.using_zoe:
                # Use ZoeDepth's colorization if available
                try:
                    from zoedepth.utils.misc import colorize
                    colored_depth_np = colorize(depth_np)
                    colored_depth = cv2.cvtColor(colored_depth_np, cv2.COLOR_RGB2BGR)
                except ImportError:
                    # Fallback to OpenCV colormap
                    colored_depth = cv2.applyColorMap(depth_norm, cv2.COLORMAP_INFERNO)
            else:
                # Use OpenCV colormap for MiDaS
                colored_depth = cv2.applyColorMap(depth_norm, cv2.COLORMAP_INFERNO)
            
            # Calculate inference time
            inference_time = time.time() - start_time
            
            # Print inference time every few frames
            if hasattr(self, 'frame_count'):
                self.frame_count += 1
                if self.frame_count % 10 == 0:
                    print(f"Depth estimation time: {inference_time:.3f}s ({1/inference_time:.2f} FPS)")
            else:
                self.frame_count = 0
            
            # Create 3D point cloud visualization
            try:
                self.visualize_point_cloud(img_rgb, depth_np)
            except Exception as e:
                print(f"Point cloud visualization error: {str(e)}")
            
            return depth_np, colored_depth, grayscale_depth
            
        except Exception as e:
            print(f"Error during depth estimation: {str(e)}")
            import traceback
            traceback.print_exc()
            return None, None, None
    
    def estimate_depth_batch(self, frames):
        """
        Estimate depth for multiple frames
        
        Args:
            frames: List of RGB or BGR images
        
        Returns:
            List of (depth_map, colored_depth, grayscale_depth) tuples
        """
        results = []
        for frame in frames:
            depth_map, colored_depth, grayscale_depth = self.estimate_depth(frame)
            results.append((depth_map, colored_depth, grayscale_depth))
        return results
    
    def close(self):
        """Release resources"""
        if self.vis is not None:
            self.vis.destroy_window()
            self.vis = None

# Test function
def test_depth_estimator():
    """Test the depth estimator using a webcam"""
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Test depth estimation")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--no-cuda", action="store_true", help="Disable CUDA")
    parser.add_argument("--download-dir", type=str, default="model_cache", help="Model download directory")
    parser.add_argument("--model", type=str, default="N", choices=["N", "K", "NK", "midas"], 
                        help="ZoeDepth model type: N, K, NK, or midas (default: N)")
    parser.add_argument("--depth-scale", type=float, default=0.5, 
                        help="Scale factor for depth values in point cloud (default: 0.5)")
    parser.add_argument("--full-screen", action="store_true", help="Run in full-screen mode")
    args = parser.parse_args()
    
    # Initialize depth estimator
    depth_estimator = DepthEstimator(
        use_cuda=not args.no_cuda,
        download_dir=args.download_dir,
        model_type=args.model,
        depth_scale=args.depth_scale
    )
    
    # Try different camera indices if needed
    cameras_to_try = [args.camera, 0, 1, 2]
    cap = None
    
    for cam_idx in cameras_to_try:
        print(f"Trying camera {cam_idx}...")
        cap = cv2.VideoCapture(cam_idx)
        if cap.isOpened():
            print(f"Successfully opened camera {cam_idx}")
            break
        else:
            print(f"Failed to open camera {cam_idx}")
    
    if not cap or not cap.isOpened():
        print("Error: Could not open any camera")
        return
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break
            
            # Get depth estimation
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
                
                # Display side by side: RGB, Grayscale Depth, Colored Depth
                # First combine RGB and grayscale
                top_row = np.hstack((frame, grayscale_depth))
                
                # Resize colored depth to match the width of the top row
                colored_depth_resized = cv2.resize(colored_depth, (top_row.shape[1], colored_depth.shape[0]))
                
                # Create a black image to pad the bottom row to match the width of the top row
                bottom_padding = np.zeros((colored_depth.shape[0], top_row.shape[1] - colored_depth.shape[1], 3), dtype=np.uint8)
                bottom_row = np.hstack((colored_depth, bottom_padding)) if bottom_padding.shape[1] > 0 else colored_depth_resized
                
                # Combine rows
                display = np.vstack((top_row, bottom_row))
                
                cv2.imshow("Depth Estimation Results", display)
            else:
                cv2.putText(frame, "Depth estimation failed", (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("RGB Only", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        depth_estimator.close()

if __name__ == "__main__":
    test_depth_estimator()