#!/usr/bin/env python3
"""
ROS Node: Multi-Fisheye Compressed Image Synchronization & BEV Processing

This node subscribes to 4 compressed fisheye camera topics, synchronizes them,
and optionally processes 2D BEV and/or 3D BEV based on command-line arguments.

Usage:
    python test_bev_ros.py --mode 2d       # Only 2D BEV
    python test_bev_ros.py --mode 3d       # Only 3D BEV
    python test_bev_ros.py --mode both     # Both 2D and 3D BEV (default)
"""

import argparse
import sys
import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from message_filters import ApproximateTimeSynchronizer, Subscriber
import threading

# Add the script directory to path for imports
base_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, base_dir)

import config
import build_bev


class FisheyeBEVProcessor:
    """Process fisheye images and generate 2D/3D BEV."""
    
    def __init__(self, mode='both'):
        """
        Initialize the BEV processor.
        
        Args:
            mode: 'both', '2d', or '3d'
        """
        self.mode = mode
        self.lock = threading.Lock()
        self.latest_images = {
            'cam1': None,
            'cam2': None,
            'cam3': None,
            'cam4': None,
        }
        self.cam_names = ['cam1', 'cam2', 'cam3', 'cam4']
        self.bev_2d_lut = {}
        self.bev_3d_lut = {}
        
        rospy.loginfo(f"BEV Processor initialized with mode: {mode}")
        
        # Pre-generate BEV LUTs if needed (one-time setup)
        self._setup_bev_luts()
    
    def _setup_bev_luts(self):
        """Set up BEV lookup tables."""
        self.car_overlay = self._create_car_overlay()
        self.car_mask = self.car_overlay > 0
        if self.mode in ['both', '2d']:
            lut_path = os.path.join(base_dir, 'data', 'bev_2d', 'luts')
            if not os.path.exists(lut_path) or len(os.listdir(lut_path)) < 4:
                rospy.logerr("Please run build_bev.py for generating 2D BEV LUTs...")
                sys.exit(1)
            else:
                for cam in self.cam_names:
                    lut_file = os.path.join(lut_path, f"lut_{cam}.npz")
                    if os.path.exists(lut_file):
                        with np.load(lut_file) as data:
                            self.bev_2d_lut[cam] = {
                                'map_x': data['map_x'],
                                'map_y': data['map_y'],
                                'weight': np.stack([data['weight']] * 3, axis=-1).astype(np.float32)
                            }
                    else:
                        rospy.logwarn(f"Missing LUT file: {lut_file}")
                        sys.exit(1)
        
        if self.mode in ['both', '3d']:
            lut_path = os.path.join(base_dir, 'data', 'bev_3d', 'luts')
            if not os.path.exists(lut_path) or len(os.listdir(lut_path)) < 4:
                rospy.logerr("Please run build_bev.py for generating 3D BEV LUTs...")
                sys.exit(1)
            else:
                for cam in self.cam_names:
                    lut_file = os.path.join(lut_path, f"lut_bowl_{cam}.npz")
                    if os.path.exists(lut_file):
                        with np.load(lut_file) as data:
                            self.bev_3d_lut[cam] = {
                                'map_x': data['map_x'],
                                'map_y': data['map_y'],
                                'weight': np.stack([data['weight']] * 3, axis=-1).astype(np.float32)
                            }
                    else:
                        rospy.logwarn(f"Missing LUT file: {lut_file}")
                        sys.exit(1)
                    # Simulate bounding limits
                    u, v = np.meshgrid(np.arange(config.BEV_WIDTH), np.arange(config.BEV_HEIGHT))
                    X = 5.0 - (v / config.PIXELS_PER_METER)
                    Y = 5.0 - (u / config.PIXELS_PER_METER)
                    self.R_abs = np.sqrt(X**2 + Y**2)
    
    # Pre-draw the Car Icon overlay
    def _create_car_overlay(self):
        overlay = np.zeros((config.BEV_HEIGHT, config.BEV_WIDTH, 3), dtype=np.uint8)
        car_top = int(config.BEV_HEIGHT / 2 - (config.CAR_LENGTH / 2.0) * config.PIXELS_PER_METER)
        car_bot = int(config.BEV_HEIGHT / 2 + (config.CAR_LENGTH / 2.0) * config.PIXELS_PER_METER)
        car_left = int(config.BEV_WIDTH / 2 - (config.CAR_WIDTH / 2.0) * config.PIXELS_PER_METER)
        car_right = int(config.BEV_WIDTH / 2 + (config.CAR_WIDTH / 2.0) * config.PIXELS_PER_METER)
        cv2.rectangle(overlay, (car_left, car_top), (car_right, car_bot), (30, 30, 30), -1)
        cv2.rectangle(
            overlay, (car_left, car_top), (car_right, car_bot), (255, 255, 255), 3
        )
        cv2.putText(
            overlay,
            "FRONT",
            (int(config.BEV_WIDTH / 2 - 40), car_top + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        return overlay

    def callback_sync(self, compressed_cam1, compressed_cam2, compressed_cam3, compressed_cam4):
        """
        Synchronized callback for 4 compressed fisheye images.
        
        Args:
            compressed_cam1/2/3/4: sensor_msgs.CompressedImage messages
        """
        try:
            with self.lock:
                # Decode compressed images
                img1 = self._decompress_image(compressed_cam1)
                img2 = self._decompress_image(compressed_cam2)
                img3 = self._decompress_image(compressed_cam3)
                img4 = self._decompress_image(compressed_cam4)
                
                if img1 is None or img2 is None or img3 is None or img4 is None:
                    rospy.logwarn("Failed to decompress one or more images")
                    return
                
                # Store images
                self.latest_images['cam1'] = img1
                self.latest_images['cam2'] = img2
                self.latest_images['cam3'] = img3
                self.latest_images['cam4'] = img4
                
                rospy.loginfo(f"Synchronized frame received at {rospy.Time.now()}")
                
                # Process based on mode
                if self.mode in ['both', '2d']:
                    self._process_2d_bev()
                
                if self.mode in ['both', '3d']:
                    self._process_3d_bev()
        
        except Exception as e:
            rospy.logerr(f"Error in sync callback: {e}")
    
    def _decompress_image(self, compressed_msg):
        """
        Decompress a CompressedImage message.
        
        Args:
            compressed_msg: sensor_msgs.CompressedImage
        
        Returns:
            OpenCV image (BGR) or None on failure
        """
        try:
            np_arr = np.frombuffer(compressed_msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return img
        except Exception as e:
            rospy.logerr(f"Decompression error: {e}")
            return None
    
    def _process_2d_bev(self):
        """Process 2D BEV from latest images."""
        try:
            bev_2d = np.zeros((config.BEV_HEIGHT, config.BEV_WIDTH, 3), dtype=np.float32)
            
            for cam_name, img in self.latest_images.items():
                if img is None:
                    continue
                
                lut = self.bev_2d_lut.get(cam_name)

                # Remap image to BEV
                warped = cv2.remap(
                    img,
                    lut["map_x"],
                    lut["map_y"],
                    cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=(0, 0, 0),
                )
                
                bev_2d += warped.astype(np.float32) * lut["weight"]
            
            final_bev_2d = bev_2d.astype(np.uint8)
            
            if config.DRAW_CAR_MASK:
                final_bev_2d[self.car_mask] = self.car_overlay[self.car_mask]
            
            # Optionally display
            cv2.imshow("2D BEV", final_bev_2d)
            cv2.waitKey(1)
        
        except Exception as e:
            rospy.logerr(f"Error in 2D BEV processing: {e}")
    
    def _process_3d_bev(self):
        """Process 3D BEV from latest images."""
        try:
            bev_3d = np.zeros((config.BEV_HEIGHT, config.BEV_WIDTH, 3), dtype=np.float32)
            
            for cam_name, img in self.latest_images.items():
                if img is None:
                    continue
                
                lut = self.bev_3d_lut.get(cam_name)

                # Remap image to BEV
                warped = cv2.remap(
                    img,
                    lut["map_x"],
                    lut["map_y"],
                    cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=(0, 0, 0),
                )
                
                bev_3d += warped.astype(np.float32) * lut["weight"]
            
            final_bev_3d = bev_3d.astype(np.uint8)
            
            if config.DRAW_CAR_MASK:
                final_bev_3d[self.car_mask] = self.car_overlay[self.car_mask]

            final_bev_rgba = cv2.cvtColor(final_bev_3d, cv2.COLOR_BGR2BGRA)
            final_bev_rgba[self.R_abs > 4.9] = (0, 0, 0, 0)
            
            # Optionally display
            cv2.imshow("3D BEV", final_bev_rgba)
            cv2.waitKey(1)
        
        except Exception as e:
            rospy.logerr(f"Error in 3D BEV processing: {e}")


def main():
    """Main ROS node entry point."""
    parser = argparse.ArgumentParser(description='Fisheye BEV Processor ROS Node')
    parser.add_argument(
        '--mode',
        type=str,
        choices=['2d', '3d', 'both'],
        default='both',
        help='Processing mode: 2d, 3d, or both (default: both)'
    )
    args = parser.parse_args()
    
    # Initialize ROS node
    rospy.init_node('fisheye_bev_processor', anonymous=True)
    rospy.loginfo(f"Starting Fisheye BEV Processor (mode: {args.mode})")
    
    # Create processor
    processor = FisheyeBEVProcessor(mode=args.mode)
    
    # Create subscribers for 4 compressed image topics
    # Adjust topic names according to your actual ROS setup
    sub_cam1 = Subscriber('/front_left/compressed', CompressedImage)
    sub_cam2 = Subscriber('/front_right/compressed', CompressedImage)
    sub_cam3 = Subscriber('/back_right/compressed', CompressedImage)
    sub_cam4 = Subscriber('/back_left/compressed', CompressedImage)
    
    # Use ApproximateTimeSynchronizer to sync messages with 0.5s tolerance
    ts = ApproximateTimeSynchronizer(
        [sub_cam1, sub_cam2, sub_cam3, sub_cam4],
        queue_size=10,
        slop=0.1  # 500ms tolerance
    )
    ts.registerCallback(processor.callback_sync)
    
    rospy.loginfo("Synchronized subscribers registered. Waiting for messages...")
    
    # Keep node alive
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
