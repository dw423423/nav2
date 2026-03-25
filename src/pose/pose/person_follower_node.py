#!/usr/bin/env python3

import math
import threading
import time
from enum import Enum, auto
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
import torch
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf2_ros import TransformException
from ultralytics import YOLO
from visualization_msgs.msg import Marker


class FollowerState(Enum):
    WAIT_TARGET = auto()
    LOCKING = auto()
    FOLLOWING = auto()


class PersonFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('person_follower_node')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.color_image_msg: Optional[Image] = None
        self.depth_image_msg: Optional[Image] = None
        self.depth_camera_info_msg: Optional[CameraInfo] = None
        self.scan_msg: Optional[LaserScan] = None

        self.state = FollowerState.WAIT_TARGET
        self.locked_person: Optional[Dict[str, float]] = None
        self.pending_person: Optional[Dict[str, float]] = None
        self.pending_since_sec: Optional[float] = None
        self.last_seen_sec: Optional[float] = None
        self.current_target_point_base: Optional[np.ndarray] = None
        self.last_cmd_log_sec: float = 0.0
        self.last_state_reason: str = 'waiting for valid target'
        self.last_reason_log_sec: float = 0.0
        self.filtered_target_distance: Optional[float] = None
        self.last_cmd_linear_x: float = 0.0
        self.last_cmd_angular_z: float = 0.0
        self.last_target_update_sec: float = 0.0
        self.last_diag_log_sec: float = 0.0
        self.last_control_time_sec: Optional[float] = None
        self.forward_motion_active: bool = False
        self.turn_motion_direction: int = 0
        self.safety_stop_active: bool = False
        self.front_obstacle_active: bool = False
        self.front_obstacle_hit_count: int = 0
        self.front_obstacle_clear_count: int = 0
        self.last_front_obstacle_distance: Optional[float] = None
        self.last_lateral_error: Optional[float] = None
        self.filtered_lateral_error_rate: float = 0.0
        self.latest_color_image_msg: Optional[Image] = None
        self.latest_depth_image_msg: Optional[Image] = None
        self.latest_depth_camera_info_msg: Optional[CameraInfo] = None
        self.latest_scan_msg: Optional[LaserScan] = None
        self.latest_color_image_cv: Optional[np.ndarray] = None
        self.latest_detections: List[List[float]] = []
        self.latest_candidates: List[Dict[str, float]] = []
        self.latest_selected_target: Optional[Dict[str, float]] = None
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.processing_thread: Optional[threading.Thread] = None
        self.last_color_frame_sec: float = 0.0
        self.last_depth_frame_sec: float = 0.0
        self.last_depth_info_sec: float = 0.0
        self.last_scan_frame_sec: float = 0.0
        self.color_frame_count: int = 0

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.perception_group = ReentrantCallbackGroup()
        self.control_group = ReentrantCallbackGroup()

        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_device', 'auto')
        self.declare_parameter('yolo_imgsz', 320)
        self.declare_parameter('yolo_half', True)
        self.declare_parameter('yolo_max_det', 4)
        self.declare_parameter('yolo_conf_threshold', 0.45)
        self.declare_parameter('min_person_box_height_ratio', 0.18)
        self.declare_parameter('min_person_aspect_ratio', 1.1)
        self.declare_parameter('min_person_box_area_ratio', 0.015)
        self.declare_parameter('lock_duration_sec', 2.0)
        self.declare_parameter('lost_timeout_sec', 1.0)
        self.declare_parameter('min_target_distance_m', 0.35)
        self.declare_parameter('max_target_distance_m', 4.0)
        self.declare_parameter('center_min_ratio', 0.35)
        self.declare_parameter('center_max_ratio', 0.65)
        self.declare_parameter('depth_window_size', 5)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_optical_frame', 'camera_depth_optical_frame')
        self.declare_parameter('person_frame', 'person_frame')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('follow_distance_m', 0.9)
        self.declare_parameter('fixed_linear_speed', 0.3)
        self.declare_parameter('linear_fast_speed', 0.3)
        self.declare_parameter('fixed_angular_speed', 0.4)
        self.declare_parameter('angular_segment_k1', 2.0)
        self.declare_parameter('angular_segment_k2', 4.0)
        self.declare_parameter('angular_fast_speed', 0.8)
        self.declare_parameter('angular_d_gain', 0.0)
        self.declare_parameter('angular_d_filter_alpha', 0.2)
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('angular_kp', 0.9)
        self.declare_parameter('min_linear_speed', 0.15)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('min_angular_speed', 0.4)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('stop_deadband_m', 0.1)
        self.declare_parameter('lateral_deadband_m', 0.08)
        self.declare_parameter('forward_engage_error_m', 0.03)
        self.declare_parameter('forward_release_error_m', 0.01)
        self.declare_parameter('turn_engage_error_m', 0.03)
        self.declare_parameter('turn_release_error_m', 0.01)
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('debug_rate_hz', 10.0)
        self.declare_parameter('safety_stop_distance_m', 0.6)
        self.declare_parameter('safety_resume_distance_m', 0.8)
        self.declare_parameter('close_box_height_ratio_stop', 0.85)
        self.declare_parameter('max_linear_accel', 5.0)
        self.declare_parameter('max_angular_accel', 8.0)
        self.declare_parameter('target_update_timeout_sec', 1.0)
        self.declare_parameter('image_stale_timeout_sec', 0.8)
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('debug_image_enabled', False)
        self.declare_parameter('enable_per_frame_logging', False)
        self.declare_parameter('enable_timing_logs', False)
        self.declare_parameter('low_speed_error_m', 0.2)
        self.declare_parameter('medium_speed_error_m', 0.5)
        self.declare_parameter('low_angular_error_m', 0.12)
        self.declare_parameter('medium_angular_error_m', 0.25)
        self.declare_parameter('distance_filter_alpha', 0.20)
        self.declare_parameter('use_scan_distance_correction', True)
        self.declare_parameter('scan_topic', '/d435_scan')
        self.declare_parameter('scan_angle_window_deg', 4.0)
        self.declare_parameter('max_scan_correction_m', 0.8)
        self.declare_parameter('scan_stale_timeout_sec', 0.5)
        self.declare_parameter('enable_front_obstacle_check', True)
        self.declare_parameter('front_obstacle_sector_deg', 40.0)
        self.declare_parameter('front_obstacle_stop_distance_m', 0.75)
        self.declare_parameter('front_obstacle_resume_distance_m', 0.95)
        self.declare_parameter('front_obstacle_slowdown_distance_m', 1.20)
        self.declare_parameter('front_obstacle_slowdown_ratio', 0.35)
        self.declare_parameter('front_obstacle_min_points', 2)
        self.declare_parameter('front_obstacle_trigger_count', 3)
        self.declare_parameter('front_obstacle_clear_count_threshold', 3)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'depth_camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )

        self.yolo_model_path = self.get_parameter('yolo_model').value
        requested_yolo_device = self.get_parameter('yolo_device').value
        self.yolo_imgsz = max(160, int(self.get_parameter('yolo_imgsz').value))
        self.yolo_half = bool(self.get_parameter('yolo_half').value)
        self.yolo_max_det = max(1, int(self.get_parameter('yolo_max_det').value))
        self.yolo_conf_threshold = float(
            self.get_parameter('yolo_conf_threshold').value
        )
        self.min_person_box_height_ratio = float(
            self.get_parameter('min_person_box_height_ratio').value
        )
        self.min_person_aspect_ratio = float(
            self.get_parameter('min_person_aspect_ratio').value
        )
        self.min_person_box_area_ratio = float(
            self.get_parameter('min_person_box_area_ratio').value
        )
        self.lock_duration_sec = float(
            self.get_parameter('lock_duration_sec').value
        )
        self.lost_timeout_sec = float(
            self.get_parameter('lost_timeout_sec').value
        )
        self.min_target_distance_m = float(
            self.get_parameter('min_target_distance_m').value
        )
        self.max_target_distance_m = float(
            self.get_parameter('max_target_distance_m').value
        )
        self.center_min_ratio = float(
            self.get_parameter('center_min_ratio').value
        )
        self.center_max_ratio = float(
            self.get_parameter('center_max_ratio').value
        )
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.person_frame = self.get_parameter('person_frame').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.follow_distance_m = float(self.get_parameter('follow_distance_m').value)
        self.fixed_linear_speed = float(
            self.get_parameter('fixed_linear_speed').value
        )
        self.linear_fast_speed = float(
            self.get_parameter('linear_fast_speed').value
        )
        self.fixed_angular_speed = float(
            self.get_parameter('fixed_angular_speed').value
        )
        self.angular_segment_k1 = float(
            self.get_parameter('angular_segment_k1').value
        )
        self.angular_segment_k2 = float(
            self.get_parameter('angular_segment_k2').value
        )
        self.angular_fast_speed = float(
            self.get_parameter('angular_fast_speed').value
        )
        self.angular_d_gain = float(
            self.get_parameter('angular_d_gain').value
        )
        self.angular_d_filter_alpha = float(
            self.get_parameter('angular_d_filter_alpha').value
        )
        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.min_angular_speed = float(
            self.get_parameter('min_angular_speed').value
        )
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.stop_deadband_m = float(self.get_parameter('stop_deadband_m').value)
        self.lateral_deadband_m = float(self.get_parameter('lateral_deadband_m').value)
        self.forward_engage_error_m = float(
            self.get_parameter('forward_engage_error_m').value
        )
        self.forward_release_error_m = float(
            self.get_parameter('forward_release_error_m').value
        )
        self.turn_engage_error_m = float(
            self.get_parameter('turn_engage_error_m').value
        )
        self.turn_release_error_m = float(
            self.get_parameter('turn_release_error_m').value
        )
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.debug_rate_hz = float(self.get_parameter('debug_rate_hz').value)
        self.safety_stop_distance_m = float(
            self.get_parameter('safety_stop_distance_m').value
        )
        self.safety_resume_distance_m = float(
            self.get_parameter('safety_resume_distance_m').value
        )
        self.close_box_height_ratio_stop = float(
            self.get_parameter('close_box_height_ratio_stop').value
        )
        self.max_linear_accel = float(
            self.get_parameter('max_linear_accel').value
        )
        self.max_angular_accel = float(
            self.get_parameter('max_angular_accel').value
        )
        self.target_update_timeout_sec = float(
            self.get_parameter('target_update_timeout_sec').value
        )
        self.image_stale_timeout_sec = float(
            self.get_parameter('image_stale_timeout_sec').value
        )
        self.process_every_n_frames = max(
            1,
            int(self.get_parameter('process_every_n_frames').value),
        )
        self.debug_image_enabled = bool(
            self.get_parameter('debug_image_enabled').value
        )
        self.enable_per_frame_logging = bool(
            self.get_parameter('enable_per_frame_logging').value
        )
        self.enable_timing_logs = bool(
            self.get_parameter('enable_timing_logs').value
        )
        self.low_speed_error_m = float(self.get_parameter('low_speed_error_m').value)
        self.medium_speed_error_m = float(
            self.get_parameter('medium_speed_error_m').value
        )
        self.low_angular_error_m = float(
            self.get_parameter('low_angular_error_m').value
        )
        self.medium_angular_error_m = float(
            self.get_parameter('medium_angular_error_m').value
        )
        self.distance_filter_alpha = float(
            self.get_parameter('distance_filter_alpha').value
        )
        self.use_scan_distance_correction = bool(
            self.get_parameter('use_scan_distance_correction').value
        )
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_angle_window_deg = float(
            self.get_parameter('scan_angle_window_deg').value
        )
        self.max_scan_correction_m = float(
            self.get_parameter('max_scan_correction_m').value
        )
        self.scan_stale_timeout_sec = float(
            self.get_parameter('scan_stale_timeout_sec').value
        )
        self.enable_front_obstacle_check = bool(
            self.get_parameter('enable_front_obstacle_check').value
        )
        self.front_obstacle_sector_deg = float(
            self.get_parameter('front_obstacle_sector_deg').value
        )
        self.front_obstacle_stop_distance_m = float(
            self.get_parameter('front_obstacle_stop_distance_m').value
        )
        self.front_obstacle_resume_distance_m = float(
            self.get_parameter('front_obstacle_resume_distance_m').value
        )
        self.front_obstacle_slowdown_distance_m = float(
            self.get_parameter('front_obstacle_slowdown_distance_m').value
        )
        self.front_obstacle_slowdown_ratio = float(
            self.get_parameter('front_obstacle_slowdown_ratio').value
        )
        self.front_obstacle_min_points = max(
            1,
            int(self.get_parameter('front_obstacle_min_points').value),
        )
        self.front_obstacle_trigger_count = max(
            1,
            int(self.get_parameter('front_obstacle_trigger_count').value),
        )
        self.front_obstacle_clear_count_threshold = max(
            1,
            int(self.get_parameter('front_obstacle_clear_count_threshold').value),
        )
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_camera_info_topic = self.get_parameter(
            'depth_camera_info_topic'
        ).value
        self.depth_window_size = max(
            3,
            int(self.get_parameter('depth_window_size').value),
        )
        if self.depth_window_size % 2 == 0:
            self.depth_window_size += 1

        self.yolo_device = self.resolve_yolo_device(requested_yolo_device)
        if self.yolo_device == 'cpu':
            self.yolo_half = False

        self.yolo_model = YOLO(self.yolo_model_path)

        self.color_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.color_image_callback,
            sensor_qos,
            callback_group=self.perception_group,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            '/person_follower/debug_image',
            1,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            '/person_follower/target_marker',
            10,
        )
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_image_callback,
            sensor_qos,
            callback_group=self.perception_group,
        )
        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            self.depth_camera_info_topic,
            self.depth_camera_info_callback,
            sensor_qos,
            callback_group=self.perception_group,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            sensor_qos,
            callback_group=self.perception_group,
        )

        self.state_log_timer = self.create_timer(
            2.0,
            self.log_current_state,
            callback_group=self.control_group,
        )
        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self.control_loop,
            callback_group=self.control_group,
        )
        self.debug_timer = None
        if self.debug_image_enabled:
            self.debug_timer = self.create_timer(
                1.0 / max(self.debug_rate_hz, 1.0),
                self.debug_publish_loop,
                callback_group=self.control_group,
            )

        self.get_logger().info('Person follower node started')
        self.get_logger().info(
            f'Loaded YOLO model: {self.yolo_model_path}, '
            f'device={self.yolo_device}, '
            f'imgsz={self.yolo_imgsz}, '
            f'half={self.yolo_half}, '
            f'max_det={self.yolo_max_det}, '
            f'conf_threshold={self.yolo_conf_threshold:.2f}, '
            f'min_box_height_ratio={self.min_person_box_height_ratio:.2f}, '
            f'min_aspect_ratio={self.min_person_aspect_ratio:.2f}'
        )
        self.get_logger().info(
            f'Subscribed topics: color={self.color_topic}, '
            f'depth={self.depth_topic}, '
            f'camera_info={self.depth_camera_info_topic}, '
            f'scan={self.scan_topic}'
        )
        self.get_logger().info(
            f'Control: cmd_vel={self.cmd_vel_topic}, '
            f'follow_distance={self.follow_distance_m:.2f} m, '
            f'fixed_linear_speed={self.fixed_linear_speed:.2f} m/s, '
            f'linear_fast_speed={self.linear_fast_speed:.2f} m/s, '
            f'fixed_angular_speed={self.fixed_angular_speed:.2f} rad/s, '
            f'angular_fast_speed={self.angular_fast_speed:.2f} rad/s, '
            f'angular_d_gain={self.angular_d_gain:.2f}, '
            f'angular_d_filter_alpha={self.angular_d_filter_alpha:.2f}, '
            f'control_rate={self.control_rate_hz:.1f} Hz, '
            f'debug_rate={self.debug_rate_hz:.1f} Hz, '
            f'process_every_n_frames={self.process_every_n_frames}, '
            f'debug_image_enabled={self.debug_image_enabled}, '
            f'safety_stop={self.safety_stop_distance_m:.2f} m, '
            f'max_linear_accel={self.max_linear_accel:.2f} m/s^2, '
            f'max_angular_accel={self.max_angular_accel:.2f} rad/s^2, '
            f'target_timeout={self.target_update_timeout_sec:.2f} s, '
            f'image_stale_timeout={self.image_stale_timeout_sec:.2f} s'
        )
        self.get_logger().info(
            f'Front obstacle check: enabled={self.enable_front_obstacle_check}, '
            f'sector={self.front_obstacle_sector_deg:.1f} deg, '
            f'stop={self.front_obstacle_stop_distance_m:.2f} m, '
            f'resume={self.front_obstacle_resume_distance_m:.2f} m, '
            f'slowdown={self.front_obstacle_slowdown_distance_m:.2f} m, '
            f'slowdown_ratio={self.front_obstacle_slowdown_ratio:.2f}, '
            f'trigger_count={self.front_obstacle_trigger_count}, '
            f'clear_count={self.front_obstacle_clear_count_threshold}'
        )
        self.log_state_transition(None, self.state)
        self.processing_thread = threading.Thread(
            target=self.processing_loop,
            name='person_follower_processing',
            daemon=True,
        )
        self.processing_thread.start()

    def resolve_yolo_device(self, requested_device: str) -> str:
        device = str(requested_device).strip().lower()
        if device and device != 'auto':
            return device

        if torch.cuda.is_available():
            return 'cuda:0'
        if hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            return 'mps'
        return 'cpu'

    def detect_persons(self, image) -> List[List[float]]:
        """Run YOLO person detection on one BGR OpenCV image.

        Expected output format:
        [[x1, y1, x2, y2, conf], ...]
        """
        detections: List[List[float]] = []
        image_height, image_width = image.shape[:2]
        results = self.yolo_model.predict(
            source=image,
            verbose=False,
            device=self.yolo_device,
            imgsz=self.yolo_imgsz,
            half=self.yolo_half,
            conf=self.yolo_conf_threshold,
            classes=[0],
            max_det=self.yolo_max_det,
        )

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                confidence = float(box.conf[0].item())

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                if not self.is_valid_person_bbox(
                    x1,
                    y1,
                    x2,
                    y2,
                    image_width,
                    image_height,
                ):
                    continue
                detections.append([
                    float(x1),
                    float(y1),
                    float(x2),
                    float(y2),
                    confidence,
                ])

        return detections

    def is_valid_person_bbox(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        image_width: int,
        image_height: int,
    ) -> bool:
        box_w = max(1.0, x2 - x1)
        box_h = max(1.0, y2 - y1)
        aspect_ratio = box_h / box_w
        box_height_ratio = box_h / max(1.0, float(image_height))
        box_area_ratio = (box_w * box_h) / max(1.0, float(image_width * image_height))

        if box_height_ratio < self.min_person_box_height_ratio:
            return False
        if aspect_ratio < self.min_person_aspect_ratio:
            return False
        if box_area_ratio < self.min_person_box_area_ratio:
            return False
        return True

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def compute_depth_median_from_bbox(
        self,
        depth_image: np.ndarray,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        depth_encoding: str,
    ) -> Optional[float]:
        height, width = depth_image.shape[:2]
        box_w = max(1.0, x2 - x1)
        box_h = max(1.0, y2 - y1)

        # Use a torso/lower-body subregion rather than the bbox center, which
        # tends to drift onto background or limbs.
        sample_x1 = int(max(0, x1 + 0.40 * box_w))
        sample_x2 = int(min(width, x1 + 0.60 * box_w))
        sample_y1 = int(max(0, y1 + 0.55 * box_h))
        sample_y2 = int(min(height, y1 + 0.85 * box_h))

        # Fall back to a small window around the bbox center if the subregion collapses.
        if sample_x2 <= sample_x1 or sample_y2 <= sample_y1:
            center_x = int((x1 + x2) * 0.5)
            center_y = int((y1 + y2) * 0.5)
            half = self.depth_window_size // 2
            sample_x1 = max(0, center_x - half)
            sample_x2 = min(width, center_x + half + 1)
            sample_y1 = max(0, center_y - half)
            sample_y2 = min(height, center_y + half + 1)

        window = depth_image[sample_y1:sample_y2, sample_x1:sample_x2]
        if window.size == 0:
            return None

        values = window.astype(np.float32).reshape(-1)
        values = values[np.isfinite(values)]
        values = values[values > 0.0]

        if values.size == 0:
            return None

        encoding = depth_encoding.lower()
        if encoding in ('16uc1', 'mono16'):
            values = values / 1000.0
        elif encoding not in ('32fc1',):
            if np.nanmedian(values) > 10.0:
                values = values / 1000.0

        values = values[
            (values >= self.min_target_distance_m)
            & (values <= self.max_target_distance_m)
        ]
        if values.size == 0:
            return None

        # Trim extremes to reduce edge/background leakage.
        values = np.sort(values)
        trim = int(values.size * 0.15)
        if trim > 0 and values.size > 2 * trim:
            values = values[trim:-trim]

        return float(np.median(values))

    def get_scan_corrected_depth(
        self,
        depth_distance: float,
        center_x: float,
    ) -> Optional[float]:
        if not self.use_scan_distance_correction:
            return None
        if self.depth_camera_info_msg is None:
            return None
        if self.last_scan_frame_sec <= 0.0:
            return None
        if (self.now_sec() - self.last_scan_frame_sec) > self.scan_stale_timeout_sec:
            return None

        with self.frame_lock:
            scan_msg = self.latest_scan_msg
        if scan_msg is None or not scan_msg.ranges:
            return None

        k = self.depth_camera_info_msg.k
        fx = float(k[0])
        cx = float(k[2])
        if fx == 0.0:
            return None

        target_angle = math.atan2(float(center_x) - cx, fx)
        half_window = math.radians(max(0.5, self.scan_angle_window_deg) * 0.5)
        angle_min = target_angle - half_window
        angle_max = target_angle + half_window

        valid_ranges = []
        for idx, rng in enumerate(scan_msg.ranges):
            if not math.isfinite(rng):
                continue
            if rng < scan_msg.range_min or rng > scan_msg.range_max:
                continue
            angle = scan_msg.angle_min + idx * scan_msg.angle_increment
            if angle_min <= angle <= angle_max:
                valid_ranges.append(float(rng))

        if not valid_ranges:
            return None

        scan_range = min(valid_ranges)
        corrected_depth = scan_range * math.cos(target_angle)
        if corrected_depth < self.min_target_distance_m or corrected_depth > self.max_target_distance_m:
            return None

        depth_overestimate = depth_distance - corrected_depth
        if depth_overestimate <= 0.0:
            return None
        if depth_overestimate > self.max_scan_correction_m:
            return None
        return corrected_depth

    def get_front_obstacle_distance(self) -> Optional[float]:
        if not self.enable_front_obstacle_check:
            return None
        if self.last_scan_frame_sec <= 0.0:
            return None
        if (self.now_sec() - self.last_scan_frame_sec) > self.scan_stale_timeout_sec:
            return None

        with self.frame_lock:
            scan_msg = self.latest_scan_msg
        if scan_msg is None or not scan_msg.ranges:
            return None

        half_window = math.radians(max(1.0, self.front_obstacle_sector_deg) * 0.5)
        valid_ranges = []
        for idx, rng in enumerate(scan_msg.ranges):
            if not math.isfinite(rng):
                continue
            if rng < scan_msg.range_min or rng > scan_msg.range_max:
                continue
            angle = scan_msg.angle_min + idx * scan_msg.angle_increment
            if -half_window <= angle <= half_window:
                valid_ranges.append(float(rng))

        if len(valid_ranges) < self.front_obstacle_min_points:
            return None

        return min(valid_ranges)

    def apply_front_obstacle_constraints(
        self,
        target_linear_x: float,
        target_angular_z: float,
    ) -> tuple[float, float]:
        obstacle_distance = self.get_front_obstacle_distance()
        self.last_front_obstacle_distance = obstacle_distance

        if obstacle_distance is None:
            self.front_obstacle_hit_count = 0
            self.front_obstacle_clear_count += 1
            if (
                self.front_obstacle_active
                and self.front_obstacle_clear_count >= self.front_obstacle_clear_count_threshold
            ):
                self.front_obstacle_active = False
            return target_linear_x, target_angular_z

        self.front_obstacle_clear_count = 0

        if obstacle_distance <= self.front_obstacle_stop_distance_m:
            self.front_obstacle_hit_count += 1
        else:
            self.front_obstacle_hit_count = 0

        if self.front_obstacle_active:
            if obstacle_distance < self.front_obstacle_resume_distance_m:
                return 0.0, target_angular_z
            self.front_obstacle_active = False

        if self.front_obstacle_hit_count >= self.front_obstacle_trigger_count:
            self.front_obstacle_active = True
            self.forward_motion_active = False
            now_sec = self.now_sec()
            if now_sec - self.last_cmd_log_sec > 0.5:
                self.get_logger().warn(
                    f'Front obstacle stop: min_range={obstacle_distance:.3f} m, '
                    f'keep angular_z={target_angular_z:.3f}'
                )
                self.last_cmd_log_sec = now_sec
            return 0.0, target_angular_z

        if obstacle_distance <= self.front_obstacle_slowdown_distance_m:
            slowdown_ratio = min(max(self.front_obstacle_slowdown_ratio, 0.0), 1.0)
            return target_linear_x * slowdown_ratio, target_angular_z

        return target_linear_x, target_angular_z

    def apply_distance_filter(self, distance: float) -> float:
        if self.filtered_target_distance is None:
            self.filtered_target_distance = distance
            return distance

        alpha = min(max(self.distance_filter_alpha, 0.0), 1.0)
        filtered = (
            alpha * distance
            + (1.0 - alpha) * self.filtered_target_distance
        )
        self.filtered_target_distance = filtered
        return filtered

    def with_filtered_distance(
        self,
        candidate: Dict[str, float],
    ) -> Dict[str, float]:
        filtered_candidate = dict(candidate)
        filtered_candidate['distance'] = self.apply_distance_filter(
            candidate.get('raw_distance', candidate['distance'])
        )
        return filtered_candidate

    def build_candidates(
        self,
        detections: List[List[float]],
        depth_image: np.ndarray,
        depth_encoding: str,
        image_width: int,
        use_center_filter: bool = True,
    ) -> List[Dict[str, float]]:
        candidates: List[Dict[str, float]] = []
        center_min = image_width * self.center_min_ratio
        center_max = image_width * self.center_max_ratio

        for det in detections:
            x1, y1, x2, y2, conf = det
            center_x = int((x1 + x2) * 0.5)
            center_y = int((y1 + y2) * 0.5)

            if use_center_filter and (center_x < center_min or center_x > center_max):
                continue

            raw_distance = self.compute_depth_median_from_bbox(
                depth_image,
                x1,
                y1,
                x2,
                y2,
                depth_encoding,
            )
            if raw_distance is None:
                continue

            scan_corrected_distance = self.get_scan_corrected_depth(
                raw_distance,
                center_x,
            )
            corrected_distance = (
                scan_corrected_distance
                if scan_corrected_distance is not None
                else raw_distance
            )

            candidates.append({
                'x1': float(x1),
                'y1': float(y1),
                'x2': float(x2),
                'y2': float(y2),
                'conf': float(conf),
                'cx': float(center_x),
                'cy': float(center_y),
                'box_height_ratio': float((y2 - y1) / max(1.0, float(depth_image.shape[0]))),
                'raw_distance': float(raw_distance),
                'scan_corrected_distance': (
                    None
                    if scan_corrected_distance is None
                    else float(scan_corrected_distance)
                ),
                'distance': float(corrected_distance),
            })

        return candidates

    def analyze_rejection_reason(
        self,
        detections: List[List[float]],
        depth_image: np.ndarray,
        depth_encoding: str,
        image_width: int,
        use_center_filter: bool = True,
    ) -> str:
        if not detections:
            return 'no person detections'

        center_min = image_width * self.center_min_ratio
        center_max = image_width * self.center_max_ratio
        found_in_center = False
        found_valid_depth = False

        for det in detections:
            x1, y1, x2, y2, _ = det
            center_x = int((x1 + x2) * 0.5)
            center_y = int((y1 + y2) * 0.5)

            if use_center_filter and (center_x < center_min or center_x > center_max):
                continue

            found_in_center = True
            distance = self.compute_depth_median_from_bbox(
                depth_image,
                x1,
                y1,
                x2,
                y2,
                depth_encoding,
            )
            if distance is not None:
                found_valid_depth = True
                break

        if use_center_filter and not found_in_center:
            return 'detected persons outside center region'

        if not found_valid_depth:
            return 'invalid or out-of-range depth near detection center'

        return 'no candidate matched selection rule'

    def same_target(
        self,
        candidate: Dict[str, float],
        reference: Dict[str, float],
    ) -> bool:
        pixel_distance = math.hypot(
            candidate['cx'] - reference['cx'],
            candidate['cy'] - reference['cy'],
        )
        depth_delta = abs(candidate['distance'] - reference['distance'])
        return pixel_distance < 80.0 and depth_delta < 0.35

    def choose_best_candidate(
        self,
        candidates: List[Dict[str, float]],
        reference: Optional[Dict[str, float]],
        image_width: int,
    ) -> Optional[Dict[str, float]]:
        if not candidates:
            return None

        image_center_x = image_width * 0.5
        if reference is None:
            return min(
                candidates,
                key=lambda cand: (
                    abs(cand['cx'] - image_center_x),
                    cand['distance'],
                    -cand['conf'],
                ),
            )

        return min(
            candidates,
            key=lambda cand: (
                math.hypot(cand['cx'] - reference['cx'], cand['cy'] - reference['cy']),
                abs(cand['distance'] - reference['distance']),
                abs(cand['cx'] - image_center_x),
                -cand['conf'],
            ),
        )

    def publish_person_tf(self, target: Dict[str, float]) -> Optional[np.ndarray]:
        if self.depth_camera_info_msg is None:
            return None

        k = self.depth_camera_info_msg.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
        if fx == 0.0 or fy == 0.0:
            return None

        z_cam = target['distance']
        x_cam = (target['cx'] - cx) * z_cam / fx
        y_cam = (target['cy'] - cy) * z_cam / fy

        point_camera = np.array([x_cam, y_cam, z_cam], dtype=np.float64)

        try:
            base_from_camera = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Failed to lookup TF {self.base_frame} <- {self.camera_optical_frame}: {exc}'
            )
            return None

        translation = base_from_camera.transform.translation
        rotation = base_from_camera.transform.rotation
        rotation_matrix = self.quaternion_to_rotation_matrix(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        point_base = rotation_matrix @ point_camera + np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float64,
        )

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.person_frame
        transform.transform.translation.x = float(point_base[0])
        transform.transform.translation.y = float(point_base[1])
        transform.transform.translation.z = float(point_base[2])
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)
        return point_base

    def quaternion_to_rotation_matrix(
        self,
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> np.ndarray:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return np.eye(3)

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float64)

    def color_image_callback(self, msg: Image) -> None:
        self.color_image_msg = msg
        self.last_color_frame_sec = self.now_sec()
        self.color_frame_count += 1
        with self.frame_lock:
            self.latest_color_image_msg = msg
        self.log_diagnostic('color image received')
        if self.color_frame_count % self.process_every_n_frames != 0:
            return
        self.frame_event.set()

    def depth_image_callback(self, msg: Image) -> None:
        self.depth_image_msg = msg
        self.last_depth_frame_sec = self.now_sec()
        with self.frame_lock:
            self.latest_depth_image_msg = msg
        self.log_diagnostic('depth image received')

    def depth_camera_info_callback(self, msg: CameraInfo) -> None:
        self.depth_camera_info_msg = msg
        self.last_depth_info_sec = self.now_sec()
        with self.frame_lock:
            self.latest_depth_camera_info_msg = msg
        self.log_diagnostic('depth camera_info received')

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_msg = msg
        self.last_scan_frame_sec = self.now_sec()
        with self.frame_lock:
            self.latest_scan_msg = msg

    def log_diagnostic(self, message: str) -> None:
        now_sec = self.now_sec()
        if now_sec - self.last_diag_log_sec < 1.0:
            return
        self.get_logger().info(f'DIAG: {message}')
        self.last_diag_log_sec = now_sec

    def process_frame(self) -> None:
        frame_start = time.perf_counter()
        self.log_diagnostic('process_frame entered')
        with self.frame_lock:
            color_msg = self.latest_color_image_msg
            depth_msg = self.latest_depth_image_msg
            depth_info_msg = self.latest_depth_camera_info_msg

        if color_msg is None:
            return

        if depth_msg is None or depth_info_msg is None:
            self.get_logger().debug('Waiting for depth image and camera info')
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(
                color_msg,
                desired_encoding='bgr8',
            )
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough',
            )
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        if self.debug_image_enabled:
            with self.frame_lock:
                self.latest_color_image_cv = color_image.copy()

        infer_start = time.perf_counter()
        detections = self.detect_persons(color_image)
        if self.yolo_device.startswith('cuda'):
            torch.cuda.synchronize()
        infer_ms = (time.perf_counter() - infer_start) * 1000.0
        if self.enable_timing_logs:
            self.get_logger().info(f'YOLO inference ms: {infer_ms:.1f}')
        if self.enable_per_frame_logging:
            self.get_logger().info(
                f'Persons detected in current frame: {len(detections)}'
            )
        use_center_filter = self.state == FollowerState.WAIT_TARGET
        candidates = self.build_candidates(
            detections,
            depth_image,
            depth_msg.encoding,
            color_image.shape[1],
            use_center_filter=use_center_filter,
        )
        rejection_reason = self.analyze_rejection_reason(
            detections,
            depth_image,
            depth_msg.encoding,
            color_image.shape[1],
            use_center_filter=use_center_filter,
        )
        selected_target = self.update_state_machine(
            candidates,
            color_image.shape[1],
            rejection_reason,
        )
        with self.frame_lock:
            self.latest_detections = [list(det) for det in detections]
            self.latest_candidates = [dict(cand) for cand in candidates]
            self.latest_selected_target = (
                dict(selected_target) if selected_target is not None else None
            )
        process_ms = (time.perf_counter() - frame_start) * 1000.0
        if self.enable_timing_logs:
            self.get_logger().info(f'process_frame total ms: {process_ms:.1f}')

    def processing_loop(self) -> None:
        while not self.shutdown_event.is_set():
            self.frame_event.wait(timeout=0.1)
            if self.shutdown_event.is_set():
                break
            self.frame_event.clear()
            self.process_frame()

    def update_state_machine(
        self,
        candidates: List[Dict[str, float]],
        image_width: int,
        rejection_reason: str,
    ) -> Optional[Dict[str, float]]:
        now_sec = self.now_sec()
        selected_target: Optional[Dict[str, float]] = None
        is_locked = self.state == FollowerState.FOLLOWING and self.locked_person is not None

        if self.state == FollowerState.WAIT_TARGET:
            candidate = self.choose_best_candidate(candidates, None, image_width)
            if candidate is not None:
                self.filtered_target_distance = None
                self.pending_person = candidate
                self.pending_since_sec = now_sec
                selected_target = candidate
                self.last_state_reason = (
                    f'candidate found at distance {candidate["distance"]:.2f} m, '
                    f'center=({int(candidate["cx"])}, {int(candidate["cy"])})'
                )
                self.set_state(FollowerState.LOCKING)
            else:
                self.pending_person = None
                self.pending_since_sec = None
                self.last_state_reason = rejection_reason

        elif self.state == FollowerState.LOCKING:
            candidate = self.choose_best_candidate(
                candidates,
                self.pending_person,
                image_width,
            )
            if candidate is None:
                self.pending_person = None
                self.pending_since_sec = None
                self.last_state_reason = f'locking interrupted: {rejection_reason}'
                self.set_state(FollowerState.WAIT_TARGET)
            else:
                selected_target = candidate
                if self.pending_person is None or not self.same_target(
                    candidate,
                    self.pending_person,
                ):
                    self.filtered_target_distance = None
                    self.pending_person = candidate
                    self.pending_since_sec = now_sec
                    self.last_state_reason = (
                        f'candidate changed during locking to distance '
                        f'{candidate["distance"]:.2f} m, '
                        f'center=({int(candidate["cx"])}, {int(candidate["cy"])})'
                    )
                else:
                    self.pending_person = candidate
                    elapsed = 0.0
                    if self.pending_since_sec is not None:
                        elapsed = now_sec - self.pending_since_sec
                    self.last_state_reason = (
                        f'locking stable for {elapsed:.2f} s, '
                        f'distance={candidate["distance"]:.2f} m'
                    )

                if (
                    self.pending_since_sec is not None
                    and (now_sec - self.pending_since_sec) >= self.lock_duration_sec
                ):
                    self.filtered_target_distance = None
                    self.locked_person = self.with_filtered_distance(candidate)
                    self.last_seen_sec = now_sec
                    is_locked = True
                    self.last_state_reason = (
                        f'target locked after {now_sec - self.pending_since_sec:.2f} s'
                    )
                    self.set_state(FollowerState.FOLLOWING)

        elif self.state == FollowerState.FOLLOWING:
            candidate = self.choose_best_candidate(
                candidates,
                self.locked_person,
                image_width,
            )
            if candidate is not None:
                candidate = self.with_filtered_distance(candidate)
                self.locked_person = candidate
                self.last_seen_sec = now_sec
                selected_target = candidate
                is_locked = True
                self.last_state_reason = (
                    f'tracking target at distance {candidate["distance"]:.2f} m, '
                    f'center=({int(candidate["cx"])}, {int(candidate["cy"])})'
                )
                point_base = self.publish_person_tf(candidate)
                if point_base is not None:
                    self.publish_target_marker(point_base, candidate)
                    self.current_target_point_base = point_base
                    self.last_target_update_sec = now_sec
                else:
                    self.current_target_point_base = None
            elif (
                self.last_seen_sec is not None
                and (now_sec - self.last_seen_sec) > self.lost_timeout_sec
            ):
                self.locked_person = None
                self.pending_person = None
                self.pending_since_sec = None
                self.last_seen_sec = None
                self.current_target_point_base = None
                self.filtered_target_distance = None
                self.last_target_update_sec = 0.0
                is_locked = False
                self.last_state_reason = (
                    f'target lost for more than {self.lost_timeout_sec:.2f} s'
                )
                self.delete_target_marker()
                self.set_state(FollowerState.WAIT_TARGET)
            else:
                selected_target = self.locked_person
                is_locked = self.locked_person is not None
                if not is_locked:
                    self.current_target_point_base = None
                    self.last_state_reason = 'locked target unavailable'
                else:
                    self.last_state_reason = 'temporarily keeping last locked target'

        else:
            self.current_target_point_base = None
            self.last_state_reason = 'unknown state fallback'

        self.log_target_debug(selected_target, is_locked)
        return selected_target

    def log_target_debug(
        self,
        target: Optional[Dict[str, float]],
        is_locked: bool,
    ) -> None:
        if target is None:
            if self.enable_per_frame_logging:
                self.get_logger().info(
                    f'Target debug: distance=N/A, center=N/A, locked={is_locked}'
                )
            return

        if self.enable_per_frame_logging:
            self.get_logger().info(
                'Target debug: '
                    f'distance={target["distance"]:.3f} m, '
                    f'raw_distance={target.get("raw_distance", target["distance"]):.3f} m, '
                    f'center=({int(target["cx"])}, {int(target["cy"])}), '
                    f'locked={is_locked}'
            )

    def publish_target_marker(
        self,
        point_base: np.ndarray,
        target: Dict[str, float],
    ) -> None:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'person_follower'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point_base[0])
        marker.pose.position.y = float(point_base[1])
        marker.pose.position.z = float(point_base[2])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.a = 0.9
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.text = f'{target["distance"]:.2f}m'
        self.marker_pub.publish(marker)

    def delete_target_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'person_follower'
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def publish_follow_cmd(self, point_base: np.ndarray) -> None:
        target_linear_x = 0.0
        target_angular_z = 0.0
        distance = float(np.linalg.norm(point_base[:2]))
        target = self.locked_person
        raw_distance = None if target is None else target.get('raw_distance')
        filtered_distance = None if target is None else target.get('distance')
        box_height_ratio = 0.0 if target is None else target.get('box_height_ratio', 0.0)

        too_close = False
        if raw_distance is not None and raw_distance <= self.safety_stop_distance_m:
            too_close = True
        if filtered_distance is not None and filtered_distance <= self.safety_stop_distance_m:
            too_close = True
        if distance <= self.safety_stop_distance_m:
            too_close = True
        if box_height_ratio >= self.close_box_height_ratio_stop:
            too_close = True

        if self.safety_stop_active:
            safe_to_resume = True
            if raw_distance is not None and raw_distance < self.safety_resume_distance_m:
                safe_to_resume = False
            if filtered_distance is not None and filtered_distance < self.safety_resume_distance_m:
                safe_to_resume = False
            if distance < self.safety_resume_distance_m:
                safe_to_resume = False
            if box_height_ratio >= (self.close_box_height_ratio_stop * 0.9):
                safe_to_resume = False
            if safe_to_resume:
                self.safety_stop_active = False
            else:
                self.publish_smoothed_cmd(0.0, 0.0)
                return

        if too_close:
            self.safety_stop_active = True
            self.forward_motion_active = False
            self.turn_motion_direction = 0
            self.publish_smoothed_cmd(0.0, 0.0)
            now_sec = self.now_sec()
            if now_sec - self.last_cmd_log_sec > 0.5:
                self.get_logger().warn(
                    f'Safety stop: target too close '
                    f'(dist={distance:.3f}, raw={raw_distance}, filtered={filtered_distance}, '
                    f'box_h={box_height_ratio:.2f})'
                )
                self.last_cmd_log_sec = now_sec
            return

        forward_error = float(point_base[0] - self.follow_distance_m)
        lateral_error = float(point_base[1])

        if self.forward_motion_active:
            if forward_error <= self.forward_release_error_m:
                self.forward_motion_active = False
        else:
            if forward_error > self.forward_engage_error_m:
                self.forward_motion_active = True

        if self.forward_motion_active:
            target_linear_x = self.compute_step_linear_speed(forward_error)

        target_angular_z = self.compute_segmented_angular_speed(lateral_error)
        target_linear_x, target_angular_z = self.apply_front_obstacle_constraints(
            target_linear_x,
            target_angular_z,
        )
        if target_angular_z > 0.0:
            self.turn_motion_direction = 1
        elif target_angular_z < 0.0:
            self.turn_motion_direction = -1
        else:
            self.turn_motion_direction = 0

        self.publish_smoothed_cmd(target_linear_x, target_angular_z)
        now_sec = self.now_sec()
        if now_sec - self.last_cmd_log_sec > 0.5:
            self.get_logger().info(
                f'Follow cmd: linear_x={self.last_cmd_linear_x:.3f}, '
                f'angular_z={self.last_cmd_angular_z:.3f}'
            )
            self.last_cmd_log_sec = now_sec

    def compute_step_linear_speed(self, forward_error: float) -> float:
        if forward_error <= self.forward_release_error_m:
            return 0.0

        if forward_error < self.low_speed_error_m:
            speed = self.min_linear_speed
        elif forward_error < self.medium_speed_error_m:
            speed = self.fixed_linear_speed
        else:
            speed = self.linear_fast_speed

        return min(speed, self.max_linear_speed)

    def compute_step_angular_speed(self, lateral_error: float) -> float:
        return self.compute_segmented_angular_speed(lateral_error)

    def compute_segmented_angular_speed(self, lateral_error: float) -> float:
        abs_error = abs(lateral_error)
        dt = None
        if self.last_control_time_sec is not None:
            dt = max(1e-3, self.now_sec() - self.last_control_time_sec)

        if abs_error <= self.turn_release_error_m:
            self.last_lateral_error = lateral_error
            self.filtered_lateral_error_rate = 0.0
            return 0.0

        direction = 1.0 if lateral_error > 0.0 else -1.0

        if abs_error < self.low_angular_error_m:
            speed = abs_error * self.angular_segment_k1
            speed = max(speed, self.min_angular_speed)
            speed = min(speed, self.fixed_angular_speed)
        elif abs_error < self.medium_angular_error_m:
            speed = abs_error * self.angular_segment_k2
            speed = max(speed, self.fixed_angular_speed)
            speed = min(speed, self.angular_fast_speed)
        else:
            speed = self.angular_fast_speed

        speed = min(speed, self.max_angular_speed)
        base_cmd = direction * speed

        if dt is None or self.last_lateral_error is None:
            raw_error_rate = 0.0
        else:
            raw_error_rate = (lateral_error - self.last_lateral_error) / dt
        self.last_lateral_error = lateral_error

        alpha = min(max(self.angular_d_filter_alpha, 0.0), 1.0)
        self.filtered_lateral_error_rate = (
            alpha * raw_error_rate
            + (1.0 - alpha) * self.filtered_lateral_error_rate
        )
        damped_cmd = base_cmd - (self.angular_d_gain * self.filtered_lateral_error_rate)
        damped_cmd = max(-self.max_angular_speed, min(self.max_angular_speed, damped_cmd))

        if base_cmd > 0.0 and damped_cmd < 0.0:
            damped_cmd = 0.0
        elif base_cmd < 0.0 and damped_cmd > 0.0:
            damped_cmd = 0.0

        return damped_cmd

    def stop_robot(self) -> None:
        self.forward_motion_active = False
        self.turn_motion_direction = 0
        self.last_lateral_error = None
        self.filtered_lateral_error_rate = 0.0
        self.publish_smoothed_cmd(0.0, 0.0)

    def publish_smoothed_cmd(
        self,
        target_linear_x: float,
        target_angular_z: float,
    ) -> None:
        now_sec = self.now_sec()
        if self.last_control_time_sec is None:
            dt = 1.0 / max(self.control_rate_hz, 1.0)
        else:
            dt = max(1e-3, now_sec - self.last_control_time_sec)
        self.last_control_time_sec = now_sec

        linear_step = self.max_linear_accel * dt
        angular_step = self.max_angular_accel * dt

        new_linear_x = self.slew_limit(
            self.last_cmd_linear_x,
            target_linear_x,
            linear_step,
        )
        new_angular_z = self.slew_limit(
            self.last_cmd_angular_z,
            target_angular_z,
            angular_step,
        )

        cmd = Twist()
        cmd.linear.x = float(new_linear_x)
        cmd.angular.z = float(new_angular_z)
        self.last_cmd_linear_x = cmd.linear.x
        self.last_cmd_angular_z = cmd.angular.z
        self.cmd_vel_pub.publish(cmd)

    def slew_limit(
        self,
        current: float,
        target: float,
        max_step: float,
    ) -> float:
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def control_loop(self) -> None:
        now_sec = self.now_sec()
        color_age = (
            float('inf')
            if self.last_color_frame_sec <= 0.0
            else now_sec - self.last_color_frame_sec
        )
        depth_age = (
            float('inf')
            if self.last_depth_frame_sec <= 0.0
            else now_sec - self.last_depth_frame_sec
        )
        if (
            color_age > self.image_stale_timeout_sec
            or depth_age > self.image_stale_timeout_sec
        ):
            self.current_target_point_base = None
            self.last_cmd_linear_x = 0.0
            self.last_cmd_angular_z = 0.0
            self.last_state_reason = (
                f'image stream stale: color_age={color_age:.2f}s, '
                f'depth_age={depth_age:.2f}s'
            )
            self.stop_robot()
            return

        if (
            self.state == FollowerState.FOLLOWING
            and self.current_target_point_base is not None
        ):
            if (
                self.last_target_update_sec <= 0.0
                or (now_sec - self.last_target_update_sec) > self.target_update_timeout_sec
            ):
                self.current_target_point_base = None
                self.last_cmd_linear_x = 0.0
                self.last_cmd_angular_z = 0.0
                self.last_state_reason = (
                    f'control stopped: target update timeout '
                    f'({now_sec - self.last_target_update_sec:.2f} s)'
                )
                self.stop_robot()
                return
            self.publish_follow_cmd(self.current_target_point_base)
            return

        self.stop_robot()

    def publish_debug_image(
        self,
        color_image: np.ndarray,
        detections: List[List[float]],
        candidates: List[Dict[str, float]],
        selected_target: Optional[Dict[str, float]],
        color_msg: Optional[Image],
    ) -> None:
        debug_image = color_image.copy()
        image_height, image_width = debug_image.shape[:2]

        center_min_x = int(image_width * self.center_min_ratio)
        center_max_x = int(image_width * self.center_max_ratio)
        cv2.line(debug_image, (center_min_x, 0), (center_min_x, image_height - 1), (255, 255, 0), 2)
        cv2.line(debug_image, (center_max_x, 0), (center_max_x, image_height - 1), (255, 255, 0), 2)

        for det in detections:
            x1, y1, x2, y2, conf = det
            pt1 = (int(x1), int(y1))
            pt2 = (int(x2), int(y2))
            cv2.rectangle(debug_image, pt1, pt2, (0, 0, 255), 2)
            cv2.putText(
                debug_image,
                f'person {conf:.2f}',
                (pt1[0], max(0, pt1[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

        for cand in candidates:
            x1 = int(cand['x1'])
            y1 = int(cand['y1'])
            x2 = int(cand['x2'])
            y2 = int(cand['y2'])
            cx = int(cand['cx'])
            cy = int(cand['cy'])
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.circle(debug_image, (cx, cy), 4, (0, 255, 255), -1)
            cv2.putText(
                debug_image,
                f'{cand["distance"]:.2f}m ({cand.get("raw_distance", cand["distance"]):.2f})',
                (x1, min(image_height - 10, y2 + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

        if selected_target is not None:
            x1 = int(selected_target['x1'])
            y1 = int(selected_target['y1'])
            x2 = int(selected_target['x2'])
            y2 = int(selected_target['y2'])
            cx = int(selected_target['cx'])
            cy = int(selected_target['cy'])
            target_color = (0, 255, 0) if self.state == FollowerState.FOLLOWING else (0, 165, 255)
            label = 'LOCKED' if self.state == FollowerState.FOLLOWING else 'LOCKING'

            cv2.rectangle(debug_image, (x1, y1), (x2, y2), target_color, 3)
            cv2.circle(debug_image, (cx, cy), 6, target_color, -1)
            cv2.putText(
                debug_image,
                f'{label} {selected_target["distance"]:.2f}m ({selected_target.get("raw_distance", selected_target["distance"]):.2f})',
                (x1, max(20, y1 - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                target_color,
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            debug_image,
            f'State: {self.state.name}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_image,
            f'Reason: {self.last_state_reason[:80]}',
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_image,
            f'Cmd: vx={self.last_cmd_linear_x:.2f} m/s  wz={self.last_cmd_angular_z:.2f} rad/s',
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        target_age = 0.0
        if self.last_target_update_sec > 0.0:
            target_age = max(0.0, self.now_sec() - self.last_target_update_sec)
        cv2.putText(
            debug_image,
            f'Target age: {target_age:.2f} s',
            (10, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_image,
            f'Flags: forward={self.forward_motion_active} turn_dir={self.turn_motion_direction} safe_stop={self.safety_stop_active}',
            (10, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to convert debug image: {exc}')
            return

        if color_msg is not None:
            debug_msg.header = color_msg.header
        self.debug_image_pub.publish(debug_msg)
        self.log_diagnostic('debug image published')

    def debug_publish_loop(self) -> None:
        now_sec = self.now_sec()
        with self.frame_lock:
            color_image = None if self.latest_color_image_cv is None else self.latest_color_image_cv.copy()
            color_msg = self.latest_color_image_msg
            detections = [list(det) for det in self.latest_detections]
            candidates = [dict(cand) for cand in self.latest_candidates]
            selected_target = (
                None if self.latest_selected_target is None
                else dict(self.latest_selected_target)
            )

        if color_image is None:
            return

        color_age = (
            float('inf')
            if self.last_color_frame_sec <= 0.0
            else now_sec - self.last_color_frame_sec
        )
        depth_age = (
            float('inf')
            if self.last_depth_frame_sec <= 0.0
            else now_sec - self.last_depth_frame_sec
        )
        if (
            color_age > self.image_stale_timeout_sec
            or depth_age > self.image_stale_timeout_sec
        ):
            stale_lines = [
                'STALE IMAGE STREAM',
                f'color_age={color_age:.2f}s depth_age={depth_age:.2f}s',
                'Try restarting d435_scan.launch.py',
            ]
            cv2.rectangle(color_image, (20, 30), (620, 170), (0, 0, 0), -1)
            cv2.rectangle(color_image, (20, 30), (620, 170), (0, 0, 255), 2)
            for idx, line in enumerate(stale_lines):
                cv2.putText(
                    color_image,
                    line,
                    (35, 70 + idx * 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9 if idx == 0 else 0.7,
                    (0, 0, 255) if idx == 0 else (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

        self.publish_debug_image(
            color_image,
            detections,
            candidates,
            selected_target,
            color_msg,
        )

    def set_state(self, new_state: FollowerState) -> None:
        if new_state == self.state:
            return

        previous_state = self.state
        self.state = new_state
        self.log_state_transition(previous_state, new_state)

    def log_state_transition(
        self,
        previous_state: Optional[FollowerState],
        new_state: FollowerState,
    ) -> None:
        if previous_state is None:
            self.get_logger().info(f'Initial state: {new_state.name}')
            return

        self.get_logger().info(
            f'State changed: {previous_state.name} -> {new_state.name}'
        )

    def log_current_state(self) -> None:
        now_sec = self.now_sec()
        if now_sec - self.last_reason_log_sec < 0.5:
            return

        self.get_logger().info(
            f'Current state: {self.state.name} | reason: {self.last_state_reason}'
        )
        self.last_reason_log_sec = now_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonFollowerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_event.set()
        node.frame_event.set()
        if node.processing_thread is not None:
            node.processing_thread.join(timeout=1.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
