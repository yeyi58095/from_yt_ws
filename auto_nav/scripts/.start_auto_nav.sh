#!/bin/bash
echo "🔧 啟動初始化 all_done_nav..."
roslaunch auto_nav all_done_empty.launch &
PID=$!

sleep 10  # 等待 gazebo + tf + sensors ready

echo "🛑 關閉初始化環境..."
kill $PID

sleep 2

echo "🚀 啟動最終導航任務 all_done_auto_nav..."
roslaunch auto_nav all_done_auto_nav.launch
