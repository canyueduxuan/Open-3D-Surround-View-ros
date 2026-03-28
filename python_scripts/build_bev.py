import os
import yaml
import numpy as np
import config
import cv2
import camera_models.eucm_model as eucm

PIXELS_PER_METER = config.PIXELS_PER_METER
BEV_WIDTH = config.BEV_WIDTH
BEV_HEIGHT = config.BEV_HEIGHT
X_RANGE = config.X_RANGE
Y_RANGE = config.Y_RANGE
CAR_LENGTH = config.CAR_LENGTH
CAR_WIDTH = config.CAR_WIDTH
FLAT_MARGIN = config.BOWL_FLAT_MARGIN
BOWL_STEEPNESS = config.BOWL_STEEPNESS

base_dir = os.path.abspath(os.path.dirname(__file__))
intr_dir = os.path.join(base_dir, 'calibration', 'intrinsics', 'params')
extr_dir = os.path.join(base_dir, 'calibration', 'extrinsics', 'params')
bev_2d_dir = os.path.join(base_dir, 'data', 'bev_2d')
bev_2d_lut = os.path.join(base_dir, 'data', 'bev_2d', 'luts')
bev_2d_debug = os.path.join(base_dir, 'data', 'bev_2d', 'debug')
bev_3d_dir = os.path.join(base_dir, 'data', 'bev_3d')
bev_3d_lut = os.path.join(base_dir, 'data', 'bev_3d', 'luts')
bev_3d_debug = os.path.join(base_dir, 'data', 'bev_3d', 'debug')

os.makedirs(intr_dir, exist_ok=True)
os.makedirs(extr_dir, exist_ok=True)
os.makedirs(bev_2d_dir, exist_ok=True)
os.makedirs(bev_2d_lut, exist_ok=True)
os.makedirs(bev_2d_debug, exist_ok=True)
os.makedirs(bev_3d_dir, exist_ok=True)
os.makedirs(bev_3d_lut, exist_ok=True)
os.makedirs(bev_3d_debug, exist_ok=True)


def inject_calibration_parameters():
    yaml_path = os.path.join(base_dir, 'calibration', 'calibration.yaml')

    if not os.path.exists(yaml_path):
        print("Error: data/sample/calibration.yaml not found!")
        exit(1)

    with open(yaml_path, "r") as f:
        calib = yaml.safe_load(f)

    for cam_name, params in calib.items():
        print(f"Injecting {cam_name} parameters...")
        intr_npz_path = os.path.join(intr_dir, f"{cam_name}_intrinsics.npz")
        extr_npz_path = os.path.join(extr_dir, f"{cam_name}_extrinsics.npz")
        
        transform = np.array(params['transform'], dtype=np.float64).reshape(4, 4)
        K = np.array(params['intrinsics']['K'], dtype=np.float64)
        D = np.array(params['intrinsics']['D'], dtype=np.float64)
        np.savez_compressed(extr_npz_path, transform=transform)
        np.savez_compressed(intr_npz_path, K=K, D=D)

    print("Calibration parameters have been injected successfully.")

def creating_bev_2d_luts():
    cameras = ['cam1', 'cam2', 'cam3', 'cam4']
    u, v = np.meshgrid(np.arange(BEV_WIDTH), np.arange(BEV_HEIGHT))

    X = X_RANGE[1] - (v / PIXELS_PER_METER)
    Y = Y_RANGE[1] - (u / PIXELS_PER_METER)
    Z = np.full_like(X, -0.4)

    pts_3d = np.stack((X, Y, Z), axis=-1).reshape(-1, 1, 3).astype(np.float32)

    bev_image_float = np.zeros((BEV_HEIGHT, BEV_WIDTH, 3), dtype=np.float32)
    blend_weights = np.zeros((BEV_HEIGHT, BEV_WIDTH), dtype=np.float32)

    camera_maps = {}  # Dictionary to store data for LUTs

    for cam_name in cameras:
        intr_path = os.path.join(intr_dir, f"{cam_name}_intrinsics.npz")
        extr_path = os.path.join(extr_dir, f"{cam_name}_extrinsics.npz")
        img_path = os.path.join(base_dir, 'test_sample', f"{cam_name}.png")

        if not os.path.exists(intr_path) or not os.path.exists(extr_path) or not os.path.exists(img_path):
            print(f"Error: file {intr_path}, {extr_path}, or {img_path} not found!")
            exit(1)
        
        with np.load(intr_path) as intr_data:
            K = intr_data["K"]
            D = intr_data["D"]

        with np.load(extr_path) as extr_data:
            transform_bc = extr_data["transform"]
        
        img = cv2.imread(img_path)
        img_h, img_w = img.shape[:2]
        print(f"  [Procesing] {cam_name}: Mapping pixels...")

        R_bc = transform_bc[:3, :3]
        t_bc = transform_bc[:3, 3]
        R_cb = R_bc.T
        t_cb = -R_cb @ t_bc
        pts_cam = (R_cb @ pts_3d.reshape(-1, 3).T + t_cb[:, np.newaxis]) #3xN
        z_cam = pts_cam[2, :].reshape(BEV_HEIGHT, BEV_WIDTH)

        camera_model = eucm.EUCMCameraModel(eucm.CameraIntrinsics(K[0], K[1], K[2], K[3], D))

        pts_2d = camera_model.project(pts_cam.T).reshape(BEV_HEIGHT, BEV_WIDTH, 2)

        map_x = pts_2d[..., 0].astype(np.float32)
        map_y = pts_2d[..., 1].astype(np.float32)

        warped = cv2.remap(
            img,
            map_x,
            map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        z_mask = z_cam > 0

        valid_x = (map_x >= 0) & (map_x < img_w - 1)
        valid_y = (map_y >= 0) & (map_y < img_h - 1)

        mask_radius_scale = config.MASK_RADIUS_SCALE

        max_radius = (np.min([img_w, img_h]) / 2.0) * mask_radius_scale
        radial_dist = np.sqrt((map_x - img_w / 2.0) ** 2 + (map_y - img_h / 2.0) ** 2) / max_radius

        weight = np.clip(1.0 - (radial_dist ** 2), 0.0, 1.0)

        valid_mask = z_mask & valid_x & valid_y

        weight = weight * valid_mask.astype(np.float32)

        # 1. View projected on BEV plane
        cv2.imwrite(os.path.join(bev_2d_debug, f"{cam_name}_01_project_bev.png"), warped)

        # 2. Mask before weight
        cv2.imwrite(
            os.path.join(bev_2d_debug, f"{cam_name}_02_mask_before_weight.png"),
            (valid_mask * 255).astype(np.uint8),
        )

        # 3. Mask after weight
        cv2.imwrite(
            os.path.join(bev_2d_debug, f"{cam_name}_03_mask_after_weight.png"),
            (weight * 255).astype(np.uint8),
        )

        # 4. Weighted mask applied and project on BEV plane
        weighted_warped = (warped.astype(np.float32) * weight[..., np.newaxis]).astype(
            np.uint8
        )
        cv2.imwrite(
            os.path.join(bev_2d_debug, f"{cam_name}_04_weighted_project_bev.png"), weighted_warped
        )

        # --------------------

        # 5. Accumulate colors
        for c in range(3):
            bev_image_float[..., c] += warped[..., c].astype(np.float32) * weight
        blend_weights += weight

        # Store parameters for LUT generation
        camera_maps[cam_name] = {"map_x": map_x, "map_y": map_y, "weight": weight}

    print("\nFinalizing stitching overlap logic...")
    valid_pixels = blend_weights > 0
    for c in range(3):
        bev_image_float[..., c][valid_pixels] /= blend_weights[valid_pixels]

    bev_image = np.clip(bev_image_float, 0, 255).astype(np.uint8)

    print(
        "\nGenerating and saving optimized LUTs (Look-Up Tables) for Real-Time rendering..."
    )
    # We pre-divide the weights here so the real-time render loop doesn't have to do
    # Floating point division on every pixel, every frame.
    safe_blend_weights = np.maximum(blend_weights, 1e-6)

    for cam, maps in camera_maps.items():
        # Normalize blending weight cleanly
        norm_weight = maps["weight"] / safe_blend_weights
        # Clean up regions outside any validation mask just in case
        norm_weight[maps["weight"] == 0] = 0.0

        lut_path = os.path.join(bev_2d_lut, f"lut_{cam}.npz")
        np.savez_compressed(
            lut_path, map_x=maps["map_x"], map_y=maps["map_y"], weight=norm_weight
        )
        print(f"  Saved LUT -> {lut_path}")

    # Render the Central Car Icon properly oriented
    if config.DRAW_CAR_MASK:
        car_top_pixels = int(BEV_HEIGHT / 2 - (CAR_LENGTH / 2.0) * PIXELS_PER_METER)
        car_bottom_pixels = int(BEV_HEIGHT / 2 + (CAR_LENGTH / 2.0) * PIXELS_PER_METER)
        car_left_pixels = int(BEV_WIDTH / 2 - (CAR_WIDTH / 2.0) * PIXELS_PER_METER)
        car_right_pixels = int(BEV_WIDTH / 2 + (CAR_WIDTH / 2.0) * PIXELS_PER_METER)

        cv2.rectangle(
            bev_image,
            (car_left_pixels, car_top_pixels),
            (car_right_pixels, car_bottom_pixels),
            (30, 30, 30),
            -1,
        )
        cv2.rectangle(
            bev_image,
            (car_left_pixels, car_top_pixels),
            (car_right_pixels, car_bottom_pixels),
            (255, 255, 255),
            3,
        )
        cv2.putText(
            bev_image,
            "FRONT",
            (int(BEV_WIDTH / 2 - 40), car_top_pixels + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
    output_path = os.path.join(bev_2d_dir, "bev.png")
    cv2.imwrite(output_path, bev_image)
    print(
        f"\nSUCCESS: Stunning 2D Bird's-Eye View successfully rendered and mapped to {output_path}"
    )

def creating_bev_3d_luts():
    cameras = ['cam1', 'cam2', 'cam3', 'cam4']
    u, v = np.meshgrid(np.arange(BEV_WIDTH), np.arange(BEV_HEIGHT))

    X = X_RANGE[1] - (v / PIXELS_PER_METER)
    Y = Y_RANGE[1] - (u / PIXELS_PER_METER)

    R_abs = np.sqrt(X**2 + Y**2)

    dx = np.maximum(np.abs(X) - config.BOWL_FLAT_RECT_X, 0.0)
    dy = np.maximum(np.abs(Y) - config.BOWL_FLAT_RECT_Y, 0.0)
    R_dist = np.sqrt(dx**2 + dy**2)

    Z = np.where(R_dist <= FLAT_MARGIN, 0.0, ((R_dist - FLAT_MARGIN) ** 2) * BOWL_STEEPNESS) - 0.8

    pts_3d = np.stack((X, Y, Z), axis=-1).reshape(-1, 1, 3).astype(np.float32)

    bev_image_float = np.zeros((BEV_HEIGHT, BEV_WIDTH, 3), dtype=np.float32)
    blend_weights = np.zeros((BEV_HEIGHT, BEV_WIDTH), dtype=np.float32)

    camera_maps = {}  # Dictionary to store data for LUTs

    for cam_name in cameras:
        intr_path = os.path.join(intr_dir, f"{cam_name}_intrinsics.npz")
        extr_path = os.path.join(extr_dir, f"{cam_name}_extrinsics.npz")
        img_path = os.path.join(base_dir, 'test_sample', f"{cam_name}.png")

        if not os.path.exists(intr_path) or not os.path.exists(extr_path) or not os.path.exists(img_path):
            print(f"Error: file {intr_path}, {extr_path}, or {img_path} not found!")
            exit(1)
        
        with np.load(intr_path) as intr_data:
            K = intr_data["K"]
            D = intr_data["D"]

        with np.load(extr_path) as extr_data:
            transform_bc = extr_data["transform"]
        
        img = cv2.imread(img_path)
        img_h, img_w = img.shape[:2]
        print(f"  [Procesing] {cam_name}: Mapping pixels...")

        # transform_cb = np.linalg.inv(transform_bc)
        # R_cb = transform_cb[:3, :3]
        # t_cb = transform_cb[:3, 3]
        R_bc = transform_bc[:3, :3]
        t_bc = transform_bc[:3, 3]
        R_cb = R_bc.T
        t_cb = -R_cb @ t_bc
        pts_cam = (R_cb @ pts_3d.reshape(-1, 3).T + t_cb[:, np.newaxis]) #3xN
        z_cam = pts_cam[2, :].reshape(BEV_HEIGHT, BEV_WIDTH)

        camera_model = eucm.EUCMCameraModel(eucm.CameraIntrinsics(K[0], K[1], K[2], K[3], D))

        pts_2d = camera_model.project(pts_cam.T).reshape(BEV_HEIGHT, BEV_WIDTH, 2)

        map_x = pts_2d[..., 0].astype(np.float32)
        map_y = pts_2d[..., 1].astype(np.float32)

        warped = cv2.remap(
            img,
            map_x,
            map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        z_mask = z_cam > 0

        valid_x = (map_x >= 0) & (map_x < img_w - 1)
        valid_y = (map_y >= 0) & (map_y < img_h - 1)

        mask_radius_scale = config.MASK_RADIUS_SCALE

        max_radius = (np.min([img_w, img_h]) / 2.0) * mask_radius_scale
        radial_dist = np.sqrt((map_x - img_w / 2.0) ** 2 + (map_y - img_h / 2.0) ** 2) / max_radius

        weight = np.clip(1.0 - (radial_dist ** 2), 0.0, 1.0)

        valid_mask = z_mask & valid_x & valid_y

        weight = weight * valid_mask.astype(np.float32)

        for c in range(3):
            bev_image_float[..., c] += warped[..., c].astype(np.float32) * weight
        blend_weights += weight

        # Store parameters for LUT generation
        camera_maps[cam_name] = {"map_x": map_x, "map_y": map_y, "weight": weight}
    
    print("\nFinalizing stitching overlap logic...")
    valid_pixels = blend_weights > 0
    for c in range(3):
        bev_image_float[..., c][valid_pixels] /= blend_weights[valid_pixels]

    bev_image = np.clip(bev_image_float, 0, 255).astype(np.uint8)

    print("\nGenerating and saving optimized LUTs for Real-Time 3D Texture rendering...")

    # Pre-divide the weights here so the real-time render loop avoids floating point division
    safe_blend_weights = np.maximum(blend_weights, 1e-6)

    for cam, maps in camera_maps.items():
        norm_weight = maps["weight"] / safe_blend_weights
        norm_weight[maps["weight"] == 0] = 0.0

        lut_path = os.path.join(bev_3d_lut, f"lut_bowl_{cam}.npz")
        np.savez_compressed(
            lut_path, map_x=maps["map_x"], map_y=maps["map_y"], weight=norm_weight
        )
        print(f"  Saved 3D Bowl LUT -> {lut_path}")

    # Central Car Icon
    if config.DRAW_CAR_MASK:
        car_top = int(BEV_HEIGHT / 2 - (CAR_LENGTH / 2.0) * PIXELS_PER_METER)
        car_bot = int(BEV_HEIGHT / 2 + (CAR_LENGTH / 2.0) * PIXELS_PER_METER)
        car_left = int(BEV_WIDTH / 2 - (CAR_WIDTH / 2.0) * PIXELS_PER_METER)
        car_right = int(BEV_WIDTH / 2 + (CAR_WIDTH / 2.0) * PIXELS_PER_METER)

        cv2.rectangle(bev_image, (car_left, car_top), (car_right, car_bot), (30, 30, 30), -1)
        cv2.rectangle(bev_image, (car_left, car_top), (car_right, car_bot), (255, 255, 255), 3)
        cv2.putText(
            bev_image,
            "FRONT",
            (int(BEV_WIDTH / 2 - 40), car_top + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

    final_bev_rgba = cv2.cvtColor(bev_image, cv2.COLOR_BGR2BGRA)
    final_bev_rgba[R_abs > 4.9] = (0, 0, 0, 0)

    output_path = os.path.join(bev_3d_dir, "bowl_texture.png")
    cv2.imwrite(output_path, final_bev_rgba)
    print(
        f"\nSUCCESS: Stunning 3D Bowl Texture successfully rendered and mapped to {output_path}"
    )


if __name__ == "__main__":
    inject_calibration_parameters()

    creating_bev_2d_luts()

    creating_bev_3d_luts()

    

    