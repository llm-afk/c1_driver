# GD32C103CB J-Link SWD Auto Flashing PowerShell Script
param(
    [string]$TargetFolder = "firmware",
    [string]$Device = "GD32C103CB"
)
$ErrorActionPreference = "Stop"

# Log formatting functions
function Get-Timestamp {
    return (Get-Date -Format "HH:mm:ss")
}

function Log-Message ($msg, $level = "INFO") {
    $ts = Get-Timestamp
    $lvl = $level.ToUpper().Trim()
    $tag = "INFO "
    $color = "Cyan"
    
    if ($lvl -eq "INFO" -or $lvl -eq "WAIT" -or $lvl -eq "READY") {
        $tag = "INFO "
        $color = "Cyan"
    } elseif ($lvl -eq "SUCCESS" -or $lvl -eq "DONE") {
        $tag = "DONE "
        $color = "Green"
    } elseif ($lvl -eq "ERROR" -or $lvl -eq "FAILED" -or $lvl -eq "ERR") {
        $tag = "ERR  "
        $color = "Red"
    } elseif ($lvl -eq "RAW") {
        $tag = "RAW  "
        $color = "Gray"
    } else {
        $tag = $lvl.PadRight(5).Substring(0, 5)
        $color = "White"
    }
    
    # Print formatted output
    Write-Host -NoNewline "[$ts]" -ForegroundColor Gray
    Write-Host -NoNewline "[$tag] " -ForegroundColor $color
    Write-Host $msg
}

function Log-ResultBlock ($ok, $msg1, $msg2 = "") {
    Write-Host ""
    $lvl = if ($ok) { "SUCCESS" } else { "FAILED" }
    Log-Message $msg1 $lvl
    if ($msg2) {
        Log-Message $msg2 $lvl
    }
    Write-Host ""
}

Clear-Host
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "             A2 J-Link SWD Auto Flashing Tool            " -ForegroundColor White -Bold
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host ""

Log-Message "GD32C103CB自动烧录脚本启动 (目标目录: $TargetFolder)" "INFO"

# 1. Search J-Link Path
Log-Message "正在检索 J-Link Commander 路径..." "INFO"

$jlinkPath = ""
# Check local env
$localJLink = Join-Path $PSScriptRoot "env\JLink.exe"
if (Test-Path $localJLink) {
    $jlinkPath = $localJLink
    Log-Message "使用本地便携版 J-Link: $jlinkPath" "INFO"
} else {
    # Check system PATH
    $sysJLink = Get-Command JLink.exe -ErrorAction SilentlyContinue
    if ($sysJLink) {
        $jlinkPath = $sysJLink.Source
        Log-Message "使用系统环境变量 J-Link: $jlinkPath" "INFO"
    } else {
        # Check standard installation locations
        $pathsToCheck = @(
            "D:\Program Files\SEGGER\JLink_V942\JLink.exe",
            "D:\Program Files\SEGGER\JLink\JLink.exe",
            "C:\Program Files\SEGGER\JLink\JLink.exe",
            "C:\Program Files (x86)\SEGGER\JLink\JLink.exe",
            "D:\Program Files (x86)\SEGGER\JLink\JLink.exe"
        )
        foreach ($path in $pathsToCheck) {
            if (Test-Path $path) {
                $jlinkPath = $path
                Log-Message "使用找到的安装路径 J-Link: $jlinkPath" "INFO"
                break
            }
        }
    }
}

if (-not $jlinkPath) {
    Log-Message "未找到 J-Link Commander (JLink.exe)！" "ERR"
    Write-Host ""
    Write-Host "请执行以下操作之一："
    Write-Host "  1. 安装 SEGGER J-Link 驱动软件。"
    Write-Host "  2. 或将 'JLink.exe' 及 'JLinkARM.dll' 复制到 'tools\auto_flash\windows\env\' 目录。"
    Write-Host ""
    Write-Host "按任意键退出..."
    [void]$Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
    exit 1
}

# 2. Firmware Check
Log-Message "正在校验本地固件 binaries..." "INFO"

$bootDir = Resolve-Path (Join-Path $PSScriptRoot "../../../../Firmware_boot/MDK-ARM/object") -ErrorAction SilentlyContinue
$appDir = Resolve-Path (Join-Path $PSScriptRoot "../../../../Firmware_app/MDK-ARM/object") -ErrorAction SilentlyContinue
$localBoot = $null
$localApp = $null
$missingFirmware = $false

# Check Bootloader
if ($bootDir -and (Test-Path $bootDir)) {
    $bootFiles = Get-ChildItem -Path $bootDir -Filter "dgm_boot_released_A2*.bin" -File
    if ($bootFiles.Count -ge 1) {
        $localBoot = $bootFiles[0].FullName
        Log-Message ("找到 Bootloader 固件: " + $bootFiles[0].Name) "INFO"
    } else {
        Log-Message "未在 Firmware_boot/MDK-ARM/object 目录下找到以 dgm_boot_released_A2*.bin 命名的固件！" "ERR"
        $missingFirmware = $true
    }
} else {
    Log-Message "未找到 Firmware_boot/MDK-ARM/object 编译输出目录！" "ERR"
    $missingFirmware = $true
}

# Check App
if ($appDir -and (Test-Path $appDir)) {
    $appFiles = Get-ChildItem -Path $appDir -Filter "dgm_app_released_A2*.bin" -File
    if ($appFiles.Count -ge 1) {
        $localApp = $appFiles[0].FullName
        Log-Message ("找到 Application 固件: " + $appFiles[0].Name) "INFO"
    } else {
        Log-Message "未在 Firmware_app/MDK-ARM/object 目录下找到以 dgm_app_released_A2*.bin 命名的固件！" "ERR"
        $missingFirmware = $true
    }
} else {
    Log-Message "未找到 Firmware_app/MDK-ARM/object 编译输出目录！" "ERR"
    $missingFirmware = $true
}

if ($missingFirmware) {
    Write-Host ""
    Write-Host "请确保分别编译了 Bootloader 和 Application 项目，并且对应的 object/ 目录下生成了对应的 dgm_boot_released_A2*.bin 和 dgm_app_released_A2*.bin 打包固件。"
    Write-Host ""
    Write-Host "按任意键退出..."
    [void]$Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
    exit 1
}

# Device Connection Check function
function Get-DeviceState {
    $detectScriptPath = Join-Path $PSScriptRoot "detect.jlink"
    "connect`nqc" | Set-Content -Path $detectScriptPath -Encoding Ascii
    
    $detectLogPath = Join-Path $PSScriptRoot "detect.log"
    
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $jlinkPath
    $psi.Arguments = '-device ' + $Device + ' -if SWD -speed 4000 -autoconnect 1 -ExitOnError 1 -NoGui 1 -CommanderScript "' + $detectScriptPath + '"'
    $psi.RedirectStandardOutput = $true
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true
    
    $proc = New-Object System.Diagnostics.Process
    $proc.StartInfo = $psi
    [void]$proc.Start()
    
    $usbConnected = $false
    $targetConnected = $false
    
    while (-not $proc.StandardOutput.EndOfStream) {
        $line = $proc.StandardOutput.ReadLine()
        if ($line -like "*Connecting to J-Link via USB...O.K.*" -or $line -like "*S/N: *") {
            $usbConnected = $true
        }
        if ($line -like "*Cortex-M4 identified*" -or $line -like "*Found SWD-DP*") {
            $targetConnected = $true
        }
    }
    $proc.WaitForExit()
    
    Remove-Item -Path $detectScriptPath -Force -ErrorAction SilentlyContinue
    
    if ($targetConnected) {
        return "READY"
    } elseif ($usbConnected) {
        return "WAIT_BOARD"
    } else {
        return "NO_JLINK"
    }
}

# 3. Main Production Loop
Log-Message "开始进入生产线烧录监控循环 (按 Ctrl+C 可退出程序)..." "INFO"
$state = $null

while ($true) {
    # Check current hardware connection state
    $currentState = Get-DeviceState
    
    $msg = ""
    $level = "INFO"
    
    if ($currentState -eq "NO_JLINK") {
        $msg = "未检测到仿真器 (Debug probe not found)"
        $level = "ERROR"
    } elseif ($currentState -eq "WAIT_BOARD") {
        $msg = "等待板子连接 (请将探针压紧至目标板)..."
        $level = "INFO"
    } elseif ($currentState -eq "READY") {
        $msg = "探针已接触，开始烧录..."
        $level = "INFO"
    }
    
    # Print status message only when connection state transitions
    if ($currentState -ne $state) {
        Log-Message $msg $level
        $state = $currentState
    }
    
    # Trigger programming once target is detected
    if ($currentState -eq "READY") {
        Log-Message "正在执行合并烧录并启动 (Boot + App)..." "INFO"
        
        $flashScript = @(
            "r",
            "h",
            "erase",
            ('loadbin "' + $localBoot + '" 0x08016800'),
            ('loadbin "' + $localApp + '" 0x08000000'),
            "r",
            "g",
            "qc"
        ) -join "`r`n"
        
        $flashScriptPath = Join-Path $PSScriptRoot "flash.jlink"
        Set-Content -Path $flashScriptPath -Value $flashScript -Encoding Ascii
        
        # Start J-Link with redirected output streaming
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = $jlinkPath
        $psi.Arguments = '-device ' + $Device + ' -if SWD -speed 4000 -autoconnect 1 -ExitOnError 1 -NoGui 1 -CommanderScript "' + $flashScriptPath + '"'
        $psi.RedirectStandardOutput = $true
        $psi.UseShellExecute = $false
        $psi.CreateNoWindow = $true
        
        $proc = New-Object System.Diagnostics.Process
        $proc.StartInfo = $psi
        [void]$proc.Start()
        
        $flashSuccess = $true
        $logContent = @()
        
        while (-not $proc.StandardOutput.EndOfStream) {
            $line = $proc.StandardOutput.ReadLine()
            $trimmed = $line.Trim()
            if (-not $trimmed) { continue }
            
            # Print raw outputs in real-time under RAW tag (kept in English for debugging)
            Log-Message $trimmed "RAW"
            $logContent += $line
            
            if (($line -like "*Failed*" -or $line -like "*Error*" -or $line -like "*Cannot connect*") -and -not ($line -like "*will now exit on Error*")) {
                $flashSuccess = $false
            }
        }
        $proc.WaitForExit()
        Remove-Item -Path $flashScriptPath -Force -ErrorAction SilentlyContinue
        
        if ($proc.ExitCode -ne 0) {
            $flashSuccess = $false
        }
        
        if ($flashSuccess) {
            Log-ResultBlock $true "Boot + App 烧录成功并已启动" "请松开并拔下板子/探针"
            
            # Wait for target unplugging
            Log-Message "等待探针抬起 (请断开连接)..." "INFO"
            $unplugCount = 0
            while ($true) {
                $checkState = Get-DeviceState
                if ($checkState -ne "READY") {
                    $unplugCount++
                } else {
                    $unplugCount = 0
                }
                
                # Debounce connection check: require 3 consecutive unplug states
                if ($unplugCount -ge 3) {
                    Log-Message "检测到探针已断开" "INFO"
                    Start-Sleep -Milliseconds 300
                    break
                }
                Start-Sleep -Milliseconds 100
            }
            $state = $null # Force connection status print in next cycle
        } else {
            Log-ResultBlock $false "烧录失败" "请检查连接并重新压紧探针"
            
            Log-Message "按任意键以重新开始连接并烧录..." "INFO"
            [void]$Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
            $state = $null # Force connection status print in next cycle
        }
    }
    
    Start-Sleep -Milliseconds 100
}
