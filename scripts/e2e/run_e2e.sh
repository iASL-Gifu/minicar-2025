#!/bin/bash

# このスクリプトは、すでに ROS 2 環境が source されている
# ターミナルで実行することを想定しています。

# 検索対象のディレクトリ
readonly LAUNCH_DIR="/workspaces/src/launch/e2e_launch/launch"

# ディレクトリの存在チェック
if [ ! -d "$LAUNCH_DIR" ]; then
    echo "エラー: Launchディレクトリが見つかりません。"
    echo "  パス: $LAUNCH_DIR"
    exit 1
fi

# launchファイルの一覧を配列に読み込む
# (cd ...): サブシェル内で移動することで、スクリプト全体のカレントディレクトリを変更しない
# nullglob: マッチするファイルがなくてもエラーにしない (配列を空にする)
shopt -s nullglob
launch_files=($(cd "$LAUNCH_DIR" && echo *.launch.xml))
shopt -u nullglob

# launchファイルが見つかるかチェック
if [ ${#launch_files[@]} -eq 0 ]; then
    echo "エラー: $LAUNCH_DIR 内に *.launch.xml ファイルが見つかりません。"
    exit 1
fi

echo "--- 実行するLaunchファイルを選択してください ---"

# bashのselect機能でインタラクティブな選択肢を表示
# PS3: selectのプロンプトメッセージを設定
PS3="番号を選択してください (終了はCtrl+C): "
select filename in "${launch_files[@]}"; do
    if [ -n "$filename" ]; then
        # 有効な選択
        echo "選択: $filename"
        break
    else
        # 無効な選択 (数字以外や範囲外)
        echo "無効な選択です。リストから番号を選んでください。"
    fi
done

# 選択されたlaunchファイルを実行
# "$@": このスクリプトに渡された引数をすべてそのままros2 launchに渡す
echo "------------------------------------------------"
echo "パッケージ: e2e_launch"
echo "ファイル:   $filename"
echo "追加引数: $@"
echo "------------------------------------------------"
echo "起動します..."

ros2 launch e2e_launch "$filename" "$@"