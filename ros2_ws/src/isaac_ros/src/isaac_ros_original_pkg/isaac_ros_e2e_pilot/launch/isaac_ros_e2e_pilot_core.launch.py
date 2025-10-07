# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# launch/pilot_net_fragment.py

import os
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
from isaac_ros_examples import IsaacROSLaunchFragment
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode

# PilotNetモデルの入力解像度
NETWORK_IMAGE_WIDTH = 160
NETWORK_IMAGE_HEIGHT = 120


class IsaacROSPilotNetLaunchFragment(IsaacROSLaunchFragment):
    """
    PilotNet推論パイプライン用のLaunch Fragment
    エンコーダー -> TensorRT推論 -> デコーダー の順でノードを定義
    """

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:
        """
        パイプラインを構成するComposableNodeのリストを返します。
        """
        # パラメータファイルとモデルファイルのパスを設定
        pkg_dir = get_package_share_directory('isaac_ros_e2e_pilot')
        params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
        
        # モデルへのパス (環境に合わせて調整してください)
        isaac_ros_ws_path = os.environ.get('ISAAC_ROS_WS', '')
        model_dir_path = os.path.join(isaac_ros_ws_path, 'isaac_ros_assets/models/pilotnet')
        engine_file_path = os.path.join(model_dir_path, 'pilotnet.plan')

        return {
            # 推論ノード: TensorRTを使用してモデル推論を実行
            'tensor_rt_node': ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensor_rt_node',
                parameters=[{
                    'engine_file_path': engine_file_path,
                    'input_tensor_names': ['input_tensor'],
                    'input_binding_names': ['input_1'], # ★モデルの入力名に合わせる
                    'output_tensor_names': ['output_tensor'],
                    'output_binding_names': ['output_1'], # ★モデルの出力名に合わせる
                }],
                remappings=[
                    ('tensor_pub', '/tensor_pub'),
                    ('tensor_sub', '/tensor_sub')
                ]
            ),
            # デコーダーノード: 推論結果のテンソルを制御コマンドに変換
            'pilot_net_decoder_node': ComposableNode(
                package='isaac_ros_e2e_pilot',
                plugin='nvidia::isaac_ros::pilot_net::PilotNetDecoderNode',
                name='pilot_net_decoder_node',
                parameters=[params_file],
                remappings=[
                    ('tensor_sub', '/control_tensor'),
                    ('cmd_ackermann', '/ackermann_cmd')
                ]
            ),
        }

    @staticmethod
    def get_launch_actions(interface_specs: Dict[str, Any]) -> Dict[str, Any]:
        """
        パイプラインに必要な追加のLaunch Action (エンコーダーの起動など) を返します。
        """
        encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
        return {
            # エンコーダーノードを別のlaunchファイルからインクルード
            'dnn_image_encoder': IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
                ),
                # エンコーダーに渡すパラメータ
                launch_arguments={
                    # 入力カメラ画像の解像度 (Fragmentを呼び出す側から渡される)
                    'input_image_width': str(interface_specs['camera_resolution']['width']),
                    'input_image_height': str(interface_specs['camera_resolution']['height']),
                    
                    # モデルの入力解像度
                    'network_image_width': str(NETWORK_IMAGE_WIDTH),
                    'network_image_height': str(NETWORK_IMAGE_HEIGHT),
                    
                    # 正規化のためのパラメータ (画像を[-1, 1]の範囲にスケーリング)
                    'image_mean': str([0.5, 0.5, 0.5]),
                    'image_stddev': str([0.5, 0.5, 0.5]),
                    
                    # 各ノードを同じコンテナで起動するための設定
                    'attach_to_shared_component_container': 'True',
                    'component_container_name': interface_specs['component_container_name'],
                    
                    # トピック名
                    'image_input_topic': interface_specs['image_input_topic'],
                    'tensor_output_topic': '/tensor_pub',
                }.items(),
            )
        }