#!/bin/bash

# Color definitions
CYAN='\033[0;36m'
GREEN='\033[0;32m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'
WHITE='\033[0;97m'
BOLD='\033[1m'

TARGET_FOLDER="${1:-firmware}"
DEVICE="${2:-GD32C103CB}"

ts() {
    date +"%H:%M:%S"
}

log() {
    local msg="$1"
    local level="${2:-INFO}"
    local tag="INFO "
    local color=""
    
    case "${level^^}" in
        "INFO"|"WAIT"|"READY")
            tag="INFO "
            color="$CYAN"
            ;;
        "SUCCESS"|"DONE")
            tag="DONE "
            color="$GREEN"
            ;;
        "ERROR"|"FAILED"|"ERR")
            tag="ERR  "
            color="$RED"
            ;;
        "RAW")
            tag="RAW  "
            color="$GRAY"
            ;;
        *)
            tag=$(printf "%-5s" "${level}")
            color=""
            ;;
    esac
    
    echo -e "${GRAY}[$(ts)]${RESET}${color}[${tag}]${RESET} ${msg}"
}

result_block() {
    local ok="$1"
    local msg1="$2"
    local msg2="$3"
    echo
    local lvl="SUCCESS"
    if [ "$ok" != "true" ]; then
        lvl="FAILED"
    fi
    log "$msg1" "$lvl"
    if [ -n "$msg2" ]; then
        log "$msg2" "$lvl"
    fi
    echo
}

clear
echo -e "${CYAN}======================================================================${RESET}"
echo -e "${BOLD}${WHITE}             C2_GD J-Link SWD Auto Flashing Tool            ${RESET}"
echo -e "${CYAN}======================================================================${RESET}"
echo

log "GD32C103CB自动烧录脚本启动 (目标目录: $TARGET_FOLDER)" "INFO"

# 1. Search for JLinkExe
log "正在检索 J-Link Commander 路径..." "INFO"

JLINK_PATH=""
if [ -f "env/JLinkExe" ]; then
    # Add env/ to LD_LIBRARY_PATH so JLinkExe can find libjlinkarm.so in the same folder
    export LD_LIBRARY_PATH="$(pwd)/env:$LD_LIBRARY_PATH"
    JLINK_PATH="env/JLinkExe"
    log "使用本地便携版 J-Link: $JLINK_PATH" "INFO"
elif which JLinkExe &>/dev/null; then
    JLINK_PATH=$(which JLinkExe)
    log "使用系统环境变量 J-Link: $JLINK_PATH" "INFO"
else
    # Check standard install locations
    STANDARD_PATHS=(
        "/usr/bin/JLinkExe"
        "/usr/local/bin/JLinkExe"
        "/opt/SEGGER/JLink/JLinkExe"
        "/opt/SEGGER/jlink/JLinkExe"
    )
    for path in "${STANDARD_PATHS[@]}"; do
        if [ -f "$path" ]; then
            JLINK_PATH="$path"
            log "使用找到的安装路径 J-Link: $JLINK_PATH" "INFO"
            break
        fi
    done
fi

if [ -z "$JLINK_PATH" ]; then
    log "未找到 J-Link Commander (JLinkExe)！" "ERR"
    echo
    echo "请执行以下操作之一："
    echo "  1. 安装 SEGGER J-Link 驱动软件。"
    echo "  2. 或将 'JLinkExe' 及 'libjlinkarm.so' 复制到 'tools/auto_flash/linux/env/' 目录。"
    echo
    read -p "按回车退出..."
    exit 1
fi

# 2. Check firmware binaries
log "正在校验本地固件 binaries..." "INFO"

LOCAL_BOOT=""
LOCAL_APP=""
MISSING_FIRMWARE=0

# SCRIPT_DIR containing flash.sh
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BOOT_DIR="$SCRIPT_DIR/../../../../Firmware_boot/MDK-ARM/object"
APP_DIR="$SCRIPT_DIR/../../../../Firmware_app/MDK-ARM/object"

# Check Bootloader
BOOT_FILES=("$BOOT_DIR"/dgm_boot_released_fw*.bin)
if [ -f "${BOOT_FILES[0]}" ]; then
    LOCAL_BOOT="${BOOT_FILES[0]}"
    log "找到 Bootloader 固件: $(basename "$LOCAL_BOOT")" "INFO"
else
    log "未在 Firmware_boot/MDK-ARM/object 目录下找到以 dgm_boot_released_fw*.bin 命名的固件！" "ERR"
    MISSING_FIRMWARE=1
fi

# Check App
APP_FILES=("$APP_DIR"/dgm_app_released_fw*.bin)
if [ -f "${APP_FILES[0]}" ]; then
    LOCAL_APP="${APP_FILES[0]}"
    log "找到 Application 固件: $(basename "$LOCAL_APP")" "INFO"
else
    log "未在 Firmware_app/MDK-ARM/object 目录下找到以 dgm_app_released_fw*.bin 命名的固件！" "ERR"
    MISSING_FIRMWARE=1
fi

if [ $MISSING_FIRMWARE -eq 1 ]; then
    echo
    echo "请确保分别编译了 Bootloader 和 Application 项目，并且对应的 object/ 目录下生成了对应的 dgm_boot_released_fw*.bin 和 dgm_app_released_fw*.bin 打包固件。"
    echo
    read -p "按回车退出..."
    exit 1
fi

# Device Connection Check function
get_device_state() {
    echo -e "connect\nqc" > detect.jlink
    
    local usb_connected=0
    local target_connected=0
    
    while read -r line; do
        if echo "$line" | grep -E "Connecting to J-Link via USB...O.K.|S/N: " >/dev/null; then
            usb_connected=1
        fi
        if echo "$line" | grep -E "Cortex-M4 identified|Found SWD-DP" >/dev/null; then
            target_connected=1
        fi
    done < <("$JLINK_PATH" -device "$DEVICE" -if SWD -speed 4000 -autoconnect 1 -ExitOnError 1 -NoGui 1 -CommanderScript detect.jlink 2>&1)
    
    rm -f detect.jlink
    
    if [ $target_connected -eq 1 ]; then
        echo "READY"
    elif [ $usb_connected -eq 1 ]; then
        echo "WAIT_BOARD"
    else
        echo "NO_JLINK"
    fi
}

# 3. Main Production Loop
log "开始进入生产线烧录监控循环 (按 Ctrl+C 可退出程序)..." "INFO"
STATE=""

while true; do
    # Check current hardware connection state
    CURRENT_STATE=$(get_device_state)
    
    MSG=""
    LEVEL="INFO"
    
    if [ "$CURRENT_STATE" = "NO_JLINK" ]; then
        MSG="未检测到仿真器 (Debug probe not found)"
        LEVEL="ERROR"
    elif [ "$CURRENT_STATE" = "WAIT_BOARD" ]; then
        MSG="等待板子连接 (请将探针压紧至目标板)..."
        LEVEL="INFO"
    elif [ "$CURRENT_STATE" = "READY" ]; then
        MSG="探针已接触，开始烧录..."
        LEVEL="INFO"
    fi
    
    # Print status message only when connection state transitions
    if [ "$CURRENT_STATE" != "$STATE" ]; then
        log "$MSG" "$LEVEL"
        STATE="$CURRENT_STATE"
    fi
    
    # Trigger programming once target is detected
    if [ "$CURRENT_STATE" = "READY" ]; then
        log "正在执行合并烧录并启动 (Boot + App)..." "INFO"
        
        cat <<EOF > flash.jlink
r
h
erase
loadbin $LOCAL_BOOT 0x08016800
loadbin $LOCAL_APP 0x08000000
r
g
qc
EOF

        FLASH_SUCCESS=0
        
        # Stream logs in real-time
        while read -r line; do
            # Strip CR/LF
            line=$(echo "$line" | tr -d '\r\n')
            [ -z "$line" ] && continue
            
            # Print raw line under RAW tag
            log "$line" "RAW"
            
            if echo "$line" | grep -E "Failed|Error|Cannot connect" >/dev/null; then
                if ! echo "$line" | grep -E "will now exit on Error" >/dev/null; then
                    FLASH_SUCCESS=1
                fi
            fi
        done < <("$JLINK_PATH" -device "$DEVICE" -if SWD -speed 4000 -autoconnect 1 -ExitOnError 1 -NoGui 1 -CommanderScript flash.jlink 2>&1)
        
        rm -f flash.jlink
        
        if [ $FLASH_SUCCESS -eq 0 ]; then
            result_block "true" "Boot + App 烧录成功并已启动" "请松开并拔下板子/探针"
            
            # Wait for target unplugging
            log "等待探针抬起 (请断开连接)..." "INFO"
            UNPLUG_COUNT=0
            while true; do
                CHECK_STATE=$(get_device_state)
                if [ "$CHECK_STATE" != "READY" ]; then
                    ((UNPLUG_COUNT++))
                else
                    UNPLUG_COUNT=0
                fi
                
                # Debounce connection check: require 3 consecutive unplug states
                if [ $UNPLUG_COUNT -ge 3 ]; then
                    log "检测到探针已断开" "INFO"
                    sleep 0.3
                    break
                fi
                sleep 0.1
            done
            STATE="" # Force connection status print in next cycle
        else
            result_block "false" "烧录失败" "请检查连接并重新压紧探针"
            
            log "按回车以重新开始连接并烧录..." "INFO"
            read -r
            STATE="" # Force connection status print in next cycle
        fi
    fi
    
    sleep 0.1
done
