#!/bin/bash

# --- グローバル変数 ---
readonly LAUNCH_DIR="/workspaces/src/launch/e2e_launch/launch"
instance_list=()
section_map_list=()
default_mode=""

# --- ヘルパー関数: 利用可能なモードを表示 ---
print_available_modes() {
    echo "---" >&2
    echo "利用可能なモード:" >&2
    local i=0
    for instance in "${instance_list[@]}"; do
        echo "  Mode ID $i: $instance" >&2
        i=$((i + 1))
    done
    echo "---" >&2
}

# --- ヘルパー関数: 数値と範囲のバリデーション ---
validate_numeric_input() {
    local input="$1"
    local min="$2"
    local max="$3"
    
    if ! [[ "$input" =~ ^[0-9]+$ ]]; then
        echo "エラー: 有効な数値ではありません。" >&2
        return 1
    fi
    if [ "$input" -lt "$min" ] || [ "$input" -ge "$max" ]; then
        echo "エラー: IDは $min から $((max - 1)) の範囲で指定してください。" >&2
        return 1
    fi
    return 0
}


# === ステップ 1: Tritonインスタンス (モード) の定義 ===
echo "--- ステップ 1: Tritonインスタンス (モード) の定義 ---" >&2
echo "Tritonインスタンス名を順番に入力してください (これがモードになります)。" >&2
echo "完了するには、何も入力せずにEnterキーを押してください。" >&2
echo "" >&2

counter=1
while true; do
    read -r -p "${counter}個目のインスタンス名 (終了はEnterのみ): " instance_name

    if [ -z "$instance_name" ]; then
        if [ ${#instance_list[@]} -eq 0 ]; then
            echo "エラー: 少なくとも1つのインスタンス名が必要です。" >&2
            continue 
        else
            echo "インスタンスの入力を完了しました。" >&2
            break
        fi
    else
        instance_list+=("$instance_name")
        counter=$((counter + 1))
    fi
done

# === ステップ 2: デフォルトモードの定義 ===
echo "" >&2
echo "--- ステップ 2: control_selector のデフォルトモード定義 ---" >&2
print_available_modes

while true; do
    read -r -p "起動時のデフォルトモードIDを入力してください: " input_mode
    if validate_numeric_input "$input_mode" 0 ${#instance_list[@]}; then
        default_mode="$input_mode"
        echo "デフォルトモードを $default_mode (${instance_list[$default_mode]}) に設定しました。" >&2
        break
    fi
done

# === ステップ 3: セクションマッピングの定義 ===
echo "" >&2
echo "--- ステップ 3: section_map のマッピング定義 ---" >&2
echo "各セクション (Section) がどのモードIDを使用するか定義します。" >&2
print_available_modes

section_counter=0
while true; do
    read -r -p "Section $section_counter に割り当てるモードID (終了はEnterのみ): " mode_id
    
    if [ -z "$mode_id" ]; then
        if [ ${#section_map_list[@]} -eq 0 ]; then
            echo "警告: セクションマッピングが1つも定義されませんでした。" >&2
        fi
        echo "セクションマッピングの入力を完了しました。" >&2
        break
    fi

    if validate_numeric_input "$mode_id" 0 ${#instance_list[@]}; then
        # YAMLのインライン文字列形式 ('0:1') で配列に保存
        section_map_list+=("'${section_counter}:${mode_id}'")
        echo "  -> Section $section_counter は Mode $mode_id (${instance_list[$mode_id]}) を使用します。" >&2
        section_counter=$((section_counter + 1))
    else
        continue
    fi
done

# === ステップ 4: 出力先ファイルの定義 ===
echo "" >&2
echo "--- ステップ 4: 出力先ファイルの定義 ---" >&2

# Launchファイルのベース名
launch_base_name=""
while true; do
    read -r -p "Launchファイル名 (例: e2e_pipeline): " input_name
    if [ -z "$input_name" ]; then
        echo "エラー: ファイル名は必須です。" >&2
    else
        launch_base_name="$input_name"
        break
    fi
done

# --- 最終的なファイルパスを決定 ---
readonly launch_filepath="${LAUNCH_DIR}/${launch_base_name}.launch.xml"

echo "" >&2
echo "以下のファイルを生成します:" >&2
echo "  Launch: $launch_filepath" >&2
echo "" >&2
read -r -p "よろしければEnterキーを押して続行してください (Ctrl+Cで中止)..."

# === ステップ 5: ファイル生成 ===

# --- 5a. Param文字列の準備 ---

# 1. input_topics のYAMLリスト文字列を生成
topic_list=()
for instance in "${instance_list[@]}"; do
    topic_list+=("'/ackermann_cmd_${instance}'")
done
topics_string="[$(IFS=,; echo "${topic_list[*]}")]"

# 2. section_to_mode_map のYAMLリスト文字列を生成
map_string=""
if [ ${#section_map_list[@]} -eq 0 ]; then
    map_string="[]"
else
    map_string="[$(IFS=,; echo "${section_map_list[*]}")]"
fi


# --- 5b. <launch_file>.launch.xml の生成 ---
echo "Generating $launch_filepath ..." >&2
{
# --- 5b-1. ヘッダー (変数を展開しない) ---
cat << 'EOF'
<?xml version="1.0"?>
<launch>
    <arg name="input_image_topic" default="/realsense2_camera/color/image_raw" />
    <arg name="input_camera_info_topic" default="/realsense2_camera/color/camera_info" />
    <arg name="output_cmd_topic" default="/ackermann_cmd_raw" />
    <arg name="output_filtered_control_cmd" default="/ackermann_cmd"/>
    <arg name="model_repository_path" default="/workspaces/isaac_ros_assets/models/" />

    <arg name="triton_input_tensor_names" default="['input_1']" />
    <arg name="triton_output_tensor_names" default="['output_1']" />
    <arg name="encoder_output_tensor_name" default="input_1" />
    <arg name="decoder_input_tensor_name" default="output_1" />

    <arg name="container_name" default="localization_container" />

    <arg name="network_image_width" default="160" />
    <arg name="network_image_height" default="120" />
    <arg name="original_image_width" default="640" />
    <arg name="original_image_height" default="480" />

    <arg name="encoder_topic" default="/encoder/tensor_pub" />

    <include file="$(find-pkg-share isaac_ros_dnn_image_encoder)/launch/dnn_image_encoder.launch.py">
        <arg name="input_image_width" value="$(var original_image_width)" />
        <arg name="input_image_height" value="$(var original_image_height)" />
        <arg name="network_image_width" value="$(var network_image_width)" />
        <arg name="network_image_height" value="$(var network_image_height)" />
        <arg name="image_input_topic" value="$(var input_image_topic)" />
        <arg name="camera_info_input_topic" value="$(var input_camera_info_topic)" />
        <arg name="tensor_output_topic" value="/encoder/tensor_pub" />
        <arg name="image_mean" value="[0.5, 0.5, 0.5]" />
        <arg name="image_stddev" value="[0.5, 0.5, 0.5]" />
        <arg name="enable_padding" value="False" />
        
        <arg name="final_tensor_name" value="$(var encoder_output_tensor_name)" />
        
        <arg name="attach_to_shared_component_container" value="True" />
        <arg name="component_container_name" value="$(var container_name)" />
        <arg name="dnn_image_encoder_namespace" value="pilotnet_encoder" />
    </include>

    EOF

# --- 5b-2. 動的Tritonインスタンス (変数を展開する) ---
for instance in "${instance_list[@]}"; do
    model="pilotnet_${instance}"
    cat << EOF
    <include file="\$(find-pkg-share e2e_launch)/launch/triton_instance.launch.xml">
        <arg name="instance_name" value="${instance}" />
        <arg name="model_name" value="${model}" /> 
        
        <arg name="container_name" value="\$(var container_name)" />
        <arg name="model_repository_path" value="\$(var model_repository_path)" />
        <arg name="triton_input_tensor_names" value="\$(var triton_input_tensor_names)" />
        <arg name="triton_output_tensor_names" value="\$(var triton_output_tensor_names)" />
        <arg name="decoder_input_tensor_name" value="\$(var decoder_input_tensor_name)" />
        <arg name="encoder_output_topic" value="\$(var encoder_topic)" />
    </include>
EOF
done

# --- 5b-3. フッター (paramを直接埋め込む) ---
# EOF をクォートせず、シェル変数 $default_mode, $topics_string, $map_string を展開
cat << EOF

    <node pkg="control_selector" exec="section_map_node" name="section_map_node" output="screen">
        <param name="section_topic" value="/current_section"/>
        <param name="mode_topic" value="/current_mode"/>
        <param name="section_to_mode_map" value="${map_string}"/>
    </node>

    <node pkg="control_selector" exec="control_selector_node" name="control_selector_node" output="screen">
        <param name="mode_topic" value="/current_mode"/>
        <param name="output_topic" value="/ackermann_cmd_raw"/>
        <param name="default_mode" value="${default_mode}"/>
        <param name="input_topics" value="${topics_string}"/>
    </node>

    <include file="\$(find-pkg-share control_filter)/launch/filter.launch.xml">
        <arg name="control_filter_param" value="\$(find-pkg-share control_filter)/config/control_filter.param.yaml" />
        <arg name="input_raw_control_cmd" value="\$(var output_cmd_topic)"/>
        <arg name="output_filtered_control_cmd" value="\$(var output_filtered_control_cmd)"/>
    </include>
</launch>
EOF
} > "$launch_filepath" # launch.xml ファイルへのリダイレクト終了

echo "" >&2
echo "--- Launchファイル生成完了 ---" >&2
echo "  $launch_filepath" >&2
echo "（control_selector と section_map のパラメータはファイル内に埋め込まれました）" >&2
echo "" >&2
echo "--- ステップ 6: colcon ビルドの実行 ---" >&2
echo "cd /workspaces/ に移動し、ビルドを実行します..." >&2

# /workspaces/ に移動
if ! cd /workspaces/; then
    echo "エラー: /workspaces/ ディレクトリに移動できませんでした。" >&2
    echo "ビルドをスキップします。" >&2
    exit 1
fi # <--- ★★★ 修正: 抜けていた fi を追加 ★★★

# colcon build を実行
colcon build --symlink-install --packages-select e2e_launch

# ビルドの成功/失敗をチェック
if [ $? -eq 0 ]; then
    echo "" >&2
    echo "--- 完了 ---" >&2
    echo "ビルドが正常に完了しました。" >&2
else
    echo "" >&2
    echo "--- エラー ---" >&2
    echo "ビルドに失敗しました。" >&2
    exit 1
fi