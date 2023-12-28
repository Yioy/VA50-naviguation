#ifndef _GEN_CIRCULATION_CONFIG_H
#define _GEN_CIRCULATION_CONFIG_H

// THIS FILE IS AUTOGENERATED
// EDIT GENCONFIG.PY AND THE CONFIG FILE TO MODIFY 

#include <array>
#include <string>

namespace config {
	namespace node {
		constexpr char trajectory_node_name[] = "trajectory";
		constexpr char transform_node_name[] = "transform_batch_server";
		constexpr char visualization_node_name[] = "visualization";
		constexpr char image_topic[] = "/zoe2/sensors/cameras/front_top/image_raw";
		constexpr char camerainfo_topic[] = "/forwardCamera/camera_info";
		constexpr char velocity_topic[] = "/dev_tools/twist_from_tf";
		constexpr char pointcloud_topic[] = "/zoe2/sensors/lidars/middle";
		constexpr char trajectory_topic[] = "/navigation/trajectory";
		constexpr char direction_topic[] = "/navigation/direction";
		constexpr char traffic_sign_topic[] = "/navigation/traffic_sign";
		constexpr char speed_topic[] = "/ZOE2UTBM/control/speed";
		constexpr char speed_cap_topic[] = "/ZOE2UTBM/control/max_speed";
		constexpr char steering_angle_topic[] = "/ZOE2UTBM/control/steering_angle";
		constexpr char lines_viz_topic[] = "/navigation/viz/lines";
		constexpr char trajectory_viz_topic[] = "/navigation/viz/trajectory";
		constexpr char trafficsigns_viz_topic[] = "/navigation/viz/trafficsigns";
		constexpr char transform_service_name[] = "/circulation/TransformBatch";
		constexpr char drop_service_name[] = "/circulation/DropVelocity";
		constexpr char road_frame[] = "road_frame";
		constexpr char world_frame[] = "odom";
		constexpr bool visualize = true;
		constexpr bool time_discrepancy = false;
	}
	namespace transform {
		constexpr float sim_interval = 0.005;
	}
	namespace control {
		constexpr float target_speed = 8.0;
		constexpr float k = 0.1;
		constexpr float Lfc = 5.0;
		constexpr float Kp = 0.5;
		constexpr float dt = 0.1;
		constexpr float WB = 2.588;
		constexpr int brake_distance = 10;
	}
	namespace environment {
		constexpr float lane_width = 3.5;
		constexpr float crosswalk_width = 0.5;
	}
	namespace birdeye {
		constexpr int x_range = 15;
		constexpr float y_range = 21.8;
		constexpr int roi_y = 3;
		constexpr int birdeye_size = 500;
	}
	namespace preprocess {
		constexpr int threshold_window = 33;
		constexpr int threshold_bias = -5;
		constexpr int open_kernel_size = 4;
		constexpr int close_kernel_size = 5;
	}
	namespace markings {
		constexpr float size_tolerance = 0.2;
	}
	namespace intersection {
		namespace intersection_hint_match_threshold {
			constexpr float trafficsign = 1.0;
			constexpr float marking = 3.85;
		}
		constexpr int mode_switch_distance = 3;
		constexpr int default_rejoin_distance = 15;
		constexpr int min_turn_radius = 5;
		constexpr int max_turn_radius = 25;
		constexpr int default_turn_radius = 12;
		constexpr int hint_detection_buffer = 15;
		constexpr float min_confidence = 0.9;
	}
	namespace trajectory {
		constexpr int history_size = 15;
		constexpr int line_reliability_range = 22;
		constexpr int line_reliability_dampening = 3;
		constexpr float line_reliability_extension_penalty = 1.8;
		constexpr float trajectory_step = 0.5;
		constexpr int trajectory_range = 15;
		constexpr int trajectory_start = 0;
		constexpr float line_score_threshold = 0.5;
		constexpr float trajectory_score_threshold = 0.5;
		constexpr float max_output_angle = 0.279;
		constexpr float max_parallel_distance = 0.9;
	}
	namespace find_lines_parameters {
		constexpr int savgol_degree = 2;
		constexpr int initial_filter_window = 25;
		constexpr int smoothing_filter_window = 9;
		constexpr int branch_step = 10;
		constexpr int min_branch_length = 8;
		constexpr int min_line_length = 35;
		constexpr int max_curvature_metric = 1;
		constexpr int curvature_filter_size = 9;
		constexpr int curvature_filter_deviation = 1;
		constexpr int merge_max_distance = 140;
		constexpr int estimate_start = 1;
		constexpr int estimate_end = 8;
		constexpr float max_angle_diff = 0.86;
		constexpr int max_rmse = 2;
	}
	namespace fuzzy_lines {
		constexpr int local_area_x = 5;
		constexpr int local_area_y = 10;
		constexpr int base_score = 4;
		namespace centers {
			constexpr float forward_distance[3] = {4.8, 10.8, 21.8};
			constexpr int line_distance[3] = {0, 1, 2};
			constexpr float line_lengths[3] = {10, 5, 0.0};
			constexpr float parallel_distances[3] = {0, 0.3, 1};
			constexpr float parallel_angles[3] = {0, 0.51, 1.24};
			constexpr float output[5] = {0, 0.25, 0.5, 0.8, 1};
		}
		namespace malus {
			constexpr int forward_distance[3] = {0, -1, -3};
			constexpr int line_distance[3] = {0, -2, -3};
			constexpr int line_lengths[3] = {0, -1, -2};
			constexpr int parallel_distances[3] = {0, -2, -5};
			constexpr int parallel_angles[3] = {0, -1, -5};
		}
		constexpr float lane_selection_threshold = 0.7;
		constexpr float single_line_selection_threshold = 0.6;
		constexpr float vertical_angle_tolerance = 0.5;
		constexpr float main_angle_cut = 0.43;
	}
	namespace visualization {
		constexpr int circulation_lines_id = 0;
		constexpr int circulation_trajectory_id = 1;
		constexpr int trafficsigns_id = 2;
	}
	namespace distance_extractor {
		constexpr char traffic_sign_detector_model[] = "yolo_v4_tiny";
	}
}

#endif
