import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    XMLLaunchDescriptionSource
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare

# OpaqueFunction
def launch_nodes_based_on_argument(context, *args, **kwargs):

    suffixes_str = context.perform_substitution(LaunchConfiguration('output_topic_suffixes'))
    model_base_name = context.perform_substitution(LaunchConfiguration('model_base_name'))
    suffixes_list = [s.strip() for s in suffixes_str.split(',') if s.strip()]

    composable_node_descriptions = []

    for suffix in suffixes_list:
        current_model_name = f"{model_base_name}_{suffix}"
        
        # Tritonノードの定義
        triton_node = ComposableNode(
            package='isaac_ros_triton',
            plugin='nvidia::isaac_ros::dnn_inference::TritonNode',
            name=f'triton_node_{suffix}',
            parameters=[{
                'model_name': current_model_name, 
                'model_repository_paths': [LaunchConfiguration('model_repository_path')],
                'input_tensor_names': LaunchConfiguration('triton_input_tensor_names'),
                'output_tensor_names': LaunchConfiguration('triton_output_tensor_names'),
                'input_binding_names': ['input_1'],
                'output_binding_names': ['output_1'],
                'input_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                'output_tensor_formats': ['nitros_tensor_list_nhwc_rgb_f32'],
            }],
            remappings=[
                ('tensor_pub', '/encoder/tensor_pub'),
                ('tensor_sub', f'/pilot_net_{suffix}/inference_result'), 
            ]
        )
        
        # Decoderノードの定義
        decoder_node = ComposableNode(
            package='isaac_ros_e2e_pilot',
            plugin='nvidia::isaac_ros::pilot_net::PilotNetDecoderNode',
            name=f'pilot_net_decoder_node_{suffix}', 
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('isaac_ros_e2e_pilot'), 'config', 'params.yaml'
                ]),
                {'tensor_name': LaunchConfiguration('decoder_input_tensor_name')}
            ],
            remappings=[
                ('/tensor_out', f'/pilot_net_{suffix}/inference_result'), 
                ('cmd_ackermann', f'/ackermann_cmd_{suffix}'),
            ]
        )
        
        composable_node_descriptions.append(triton_node)
        composable_node_descriptions.append(decoder_node)

    load_nodes_action = LoadComposableNodes(
        composable_node_descriptions=composable_node_descriptions,
        target_container=LaunchConfiguration('container_name'),
    )

    return [load_nodes_action]


def generate_launch_description():
    
    # ==========================================
    # 引数の定義
    # ==========================================
    declared_arguments = [
        DeclareLaunchArgument(
            'input_image_topic', default_value='/realsense2_camera/color/image_raw'),
        DeclareLaunchArgument(
            'input_camera_info_topic', default_value='/realsense2_camera/color/camera_info'),
        DeclareLaunchArgument(
            'output_cmd_topic', default_value='/ackermann_cmd_raw'),
        DeclareLaunchArgument(
            'output_filtered_control_cmd', default_value='/ackermann_cmd'),
        DeclareLaunchArgument(
            'model_repository_path', default_value='/workspaces/isaac_ros_assets/models/'),
        DeclareLaunchArgument(
            'model_base_name', default_value='pilotnet'), 
        DeclareLaunchArgument(
            'triton_input_tensor_names', default_value="['input_1']"),
        DeclareLaunchArgument(
            'triton_output_tensor_names', default_value="['output_1']"),
        DeclareLaunchArgument(
            'encoder_output_tensor_name', default_value='input_1'),
        DeclareLaunchArgument(
            'decoder_input_tensor_name', default_value='output_1'),
        DeclareLaunchArgument(
            'container_name', default_value='localization_container'),
        DeclareLaunchArgument(
            'network_image_width', default_value='160'),
        DeclareLaunchArgument(
            'network_image_height', default_value='120'),
        DeclareLaunchArgument(
            'original_image_width', default_value='640'),
        DeclareLaunchArgument(
            'original_image_height', default_value='480'),
        DeclareLaunchArgument(
            'output_topic_suffixes', default_value='race,540'),
    ]

    # ==========================================
    # パラメータファイルのパス (2つに分割)
    # ==========================================
    # Mapper用
    section_map_config = PathJoinSubstitution([
        FindPackageShare('control_selector'), 'config', 'section_map_node.param.yaml'
    ])
    
    # Selector用
    control_selector_config = PathJoinSubstitution([
        FindPackageShare('control_selector'), 'config', 'control_selector.param.yaml'
    ])

    # ==========================================
    # Encoder Node (共通)
    # ==========================================
    encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_dnn_image_encoder'),
                'launch', 'dnn_image_encoder.launch.py'
            ])
        ]),
        launch_arguments={
            'input_image_width': LaunchConfiguration('original_image_width'),
            'input_image_height': LaunchConfiguration('original_image_height'),
            'network_image_width': LaunchConfiguration('network_image_width'),
            'network_image_height': LaunchConfiguration('network_image_height'),
            'image_input_topic': LaunchConfiguration('input_image_topic'),
            'camera_info_input_topic': LaunchConfiguration('input_camera_info_topic'),
            'tensor_output_topic': '/encoder/tensor_pub',
            'image_mean': '[0.5, 0.5, 0.5]',
            'image_stddev': '[0.5, 0.5, 0.5]',
            'enable_padding': 'False',
            'final_tensor_name': LaunchConfiguration('encoder_output_tensor_name'),
            'attach_to_shared_component_container': 'True',
            'component_container_name': LaunchConfiguration('container_name'),
            'dnn_image_encoder_namespace': 'pilotnet_encoder',
        }.items()
    )

    # ==========================================
    # Section Mapper Node (セクション -> モードID)
    # ==========================================
    section_mapper_node = Node(
        package='control_selector',
        executable='section_map_node',    # <-- 実行ファイル名変更
        name='section_map_node',          # <-- ノード名変更 (YAMLと一致させる)
        output='screen',
        parameters=[section_map_config]   # <-- 専用YAMLを読み込み
    )

    # ==========================================
    # Ackermann Mode Selector Node (モードID -> トピック)
    # ==========================================
    ackermann_mode_selector_node = Node(
        package='control_selector',
        executable='control_selector_node', # <-- 実行ファイル名変更
        name='control_selector_node',       # <-- ノード名変更 (YAMLと一致させる)
        output='screen',
        parameters=[
            control_selector_config, # <-- 専用YAMLを読み込み
            
            # YAML内の 'output_topic' をLaunch引数で上書き
            {'output_topic': LaunchConfiguration('output_cmd_topic')}
        ]
    )

    # ==========================================
    # control_filter 
    # ==========================================
    control_filter_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('control_filter'),
                'launch', 'filter.launch.xml'
            ])
        ]),
        launch_arguments={
            'control_filter_param': PathJoinSubstitution([
                FindPackageShare('control_filter'), 'config', 'control_filter.param.yaml'
            ]),
            'input_raw_control_cmd': LaunchConfiguration('output_cmd_topic'),
            'output_filtered_control_cmd': LaunchConfiguration('output_filtered_control_cmd'),
        }.items()
    )


    # ==========================================
    # LaunchDescriptionの構築
    # ==========================================
    return LaunchDescription(
        declared_arguments +
        [
            encoder_launch,
            section_mapper_node,          
            ackermann_mode_selector_node,
            control_filter_launch,
            OpaqueFunction(function=launch_nodes_based_on_argument)
        ]
    )