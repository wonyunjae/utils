#!/bin/bash

echo "🚗 GLIM Localization System - 4분할 화면 배치"
echo "============================================="

# wmctrl 설치 확인
if ! command -v wmctrl &> /dev/null; then
    echo "❌ wmctrl이 설치되지 않았습니다."
    echo "설치하시겠습니까? (y/n)"
    read -n 1 answer
    echo
    if [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
        echo "📦 wmctrl 설치 중..."
        sudo apt-get update && sudo apt-get install -y wmctrl
    else
        echo "❌ wmctrl 없이는 창 배치를 할 수 없습니다."
        exit 1
    fi
fi

echo "🪟 창 배치를 시작합니다..."
sleep 1

# 화면 해상도 가져오기
SCREEN_WIDTH=$(xrandr | grep '\*' | awk '{print $1}' | cut -d'x' -f1 | head -1)
SCREEN_HEIGHT=$(xrandr | grep '\*' | awk '{print $1}' | cut -d'x' -f2 | head -1)

# 창 크기 계산 (화면의 절반)
WINDOW_WIDTH=$((SCREEN_WIDTH / 2))
WINDOW_HEIGHT=$((SCREEN_HEIGHT / 2))

echo "📺 화면 해상도: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
echo "🪟 창 크기: ${WINDOW_WIDTH}x${WINDOW_HEIGHT}"

# 1. UI Controller - 왼쪽 상단
echo "1️⃣ UI Controller 배치 중..."
for window_name in "GLIM Localization System Controller" "localization_ui" "Controller"; do
    if wmctrl -r "$window_name" -e 0,0,0,$WINDOW_WIDTH,$WINDOW_HEIGHT 2>/dev/null; then
        echo "   ✅ UI Controller 위치 조정됨: 왼쪽 상단"
        break
    fi
done

# 2. Google Earth Map - 오른쪽 상단
echo "2️⃣ Google Earth Map 배치 중..."
for window_name in "Vehicle Path Visualizer" "Google Chrome" "Chrome" "Chromium" "Firefox" "Mozilla Firefox"; do
    if wmctrl -r "$window_name" -e 0,$WINDOW_WIDTH,0,$WINDOW_WIDTH,$WINDOW_HEIGHT 2>/dev/null; then
        echo "   ✅ Google Earth Map 위치 조정됨: 오른쪽 상단"
        break
    fi
done

# 3. FoundationStereo/Terminal - 왼쪽 하단
echo "3️⃣ FoundationStereo Terminal 배치 중..."
for window_name in "Terminal" "gnome-terminal" "foundation" "Foundation Stereo"; do
    if wmctrl -r "$window_name" -e 0,0,$WINDOW_HEIGHT,$WINDOW_WIDTH,$WINDOW_HEIGHT 2>/dev/null; then
        echo "   ✅ FoundationStereo Terminal 위치 조정됨: 왼쪽 하단"
        break
    fi
done

# 4. GLIM Map Viewer - 오른쪽 하단
echo "4️⃣ GLIM Map Viewer 배치 중..."
for window_name in "ORB-SLAM3: Map Viewer" "Map Viewer" "SLAM" "glim"; do
    if wmctrl -r "$window_name" -e 0,$WINDOW_WIDTH,$WINDOW_HEIGHT,$WINDOW_WIDTH,$WINDOW_HEIGHT 2>/dev/null; then
        echo "   ✅ GLIM Map Viewer 위치 조정됨: 오른쪽 하단"
        break
    fi
done

echo ""
echo "🎉 창 배치 완료!"
echo ""

# 현재 열린 창들 목록 출력
echo "📋 현재 열린 창들:"
echo "=================="
wmctrl -l | grep -v "Desktop" | while IFS= read -r line; do
    window_id=$(echo "$line" | awk '{print $1}')
    window_name=$(echo "$line" | awk '{$1=$2=$3=""; print $0}' | sed 's/^[ \t]*//')
    echo "  🪟 $window_name (ID: $window_id)"
done

echo ""
echo "💡 팁: 창이 제대로 배치되지 않았다면 해당 프로그램을 먼저 실행한 후 다시 시도하세요."
