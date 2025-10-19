#!/bin/bash

# --- 設定項目 ---
readonly REMOTE_USER="tamiya"
# 重みファイルが保存されているリモートの親ディレクトリ
readonly REMOTE_BASE_DIR="/home/tamiya/workspace/minicar-2025/python_ws/ckpts"
# -----------------

# --- 1. リモートホストの選択 ---
echo "🔄 接続先のリモートホストを選択してください。"
HOST_OPTIONS=("10.42.0.1" "192.168.55.1" "手動で入力")

PS3="番号を入力してください (qで終了): "
select HOST_OPT in "${HOST_OPTIONS[@]}" "quit"; do
    case "$HOST_OPT" in
        "quit")
            echo "処理を終了します。"
            exit 0
            ;;
        "10.42.0.1" | "192.168.55.1")
            REMOTE_HOST="$HOST_OPT"
            echo "✅ $REMOTE_HOST を選択しました。"
            break
            ;;
        "手動で入力")
            read -p "IPアドレスまたはホスト名を入力してください: " INPUT_HOST
            # 入力が空でないことを確認
            if [ -n "$INPUT_HOST" ]; then
                REMOTE_HOST="$INPUT_HOST"
                echo "✅ $REMOTE_HOST を使用します。"
                break
            else
                echo "❌ 入力がありません。やり直してください。"
            fi
            ;;
        *)
            echo "無効な選択です。リストから番号を選んでください。"
            ;;
    esac
done

# 選択されたホストを使ってリモートアドレスを構築
readonly REMOTE_HOST
readonly REMOTE_ADDRESS="${REMOTE_USER}@${REMOTE_HOST}"
echo "----------------------------------------"


# --- 2. ローカル保存先パスの決定 ---
# スクリプトの第1引数 ($1) が指定されていればそれを使い、
# 指定されていなければデフォルト値 ($HOME/ckpts) を使います。
readonly LOCAL_DEST_DIR="${1:-$HOME/ckpts}"

# ローカルの保存先ディレクトリが存在しない場合は作成します。
mkdir -p "$LOCAL_DEST_DIR"
echo "✅ ローカル保存先: $LOCAL_DEST_DIR"
echo "----------------------------------------"


# --- 3. リモートのモデルディレクトリ一覧の取得 ---
echo "⏳ リモート ($REMOTE_ADDRESS) からモデルディレクトリ一覧を取得中..."
echo "  (対象: $REMOTE_BASE_DIR)"

# ssh経由でfindコマンドを実行し、指定ディレクトリ直下にあるディレクトリ名だけを取得します。
REMOTE_DIRS_CMD="find \"$REMOTE_BASE_DIR\" -mindepth 1 -maxdepth 1 -type d -printf '%f\n'"

# sshコマンドの実行結果を `MODEL_DIRS` という配列に格納します。
mapfile -t MODEL_DIRS < <(ssh "$REMOTE_ADDRESS" "$REMOTE_DIRS_CMD")

# ssh接続失敗、またはfindコマンド失敗のチェック
if [ $? -ne 0 ]; then
    echo "❌ エラー: リモートホスト ($REMOTE_ADDRESS) への接続に失敗したか、ディレクトリの検索に失敗しました。"
    exit 1
fi

# ディレクトリが一つも見つからなかった場合のチェック
if [ ${#MODEL_DIRS[@]} -eq 0 ]; then
    echo "❌ エラー: リモートの $REMOTE_BASE_DIR 内にモデルディレクトリが見つかりませんでした。"
    exit 1
fi

echo "✅ モデルディレクトリ一覧の取得完了。"
echo "----------------------------------------"


# --- 4. モデルディレクトリの選択 ---
echo "どのモデルのチェックポイントをコピーしますか？"

# select構文で、取得したモデルディレクトリ一覧と「終了」を選択肢として表示します。
PS3="番号を入力してください (qで終了): "
select DIR_NAME in "${MODEL_DIRS[@]}" "quit"; do
    case "$DIR_NAME" in
        "quit")
            # "quit" が選ばれたら終了
            echo "処理を終了します。"
            exit 0
            ;;
        "")
            # 不正な入力（数字以外など）
            echo "無効な選択です。リストから番号を選んでください。"
            ;;
        *)
            # 有効なディレクトリが選ばれたらループを抜ける
            echo "✅ モデル「$DIR_NAME」を選択しました。"
            break
            ;;
    esac
done


# --- 5. scp の実行 ---
# 選択されたモデルディレクトリのフルパスを構築
readonly REMOTE_SOURCE_PATH="${REMOTE_BASE_DIR}/${DIR_NAME}"

echo "----------------------------------------"
echo "🔄 コピーを実行します..."
echo "   From: $REMOTE_ADDRESS:$REMOTE_SOURCE_PATH"
echo "   To:   $LOCAL_DEST_DIR"
echo "----------------------------------------"

scp -r "$REMOTE_ADDRESS:$REMOTE_SOURCE_PATH" "$LOCAL_DEST_DIR"

# --- 6. 完了メッセージ ---
if [ $? -eq 0 ]; then
    echo "🎉 コピーが完了しました。"
    echo "   保存先: $LOCAL_DEST_DIR/$DIR_NAME"
else
    echo "❌ コピー中にエラーが発生しました。"
fi

exit 0
