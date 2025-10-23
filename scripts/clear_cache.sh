# スクリプト内には sudo を書かない
#!/bin/bash
echo "現在のメモリ状況（前）"
free -h
sync
echo 3 > /proc/sys/vm/drop_caches
swapoff -a
swapon -a
echo "現在のメモリ状況（後）"
free -h