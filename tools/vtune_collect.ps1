# 以管理员身份采集 VTune 硬件计数器（PMU）。
#
# 为什么要提权：采样驱动 `sepdrv5` 已经加载，但 **VTune 的采集器本身**要管理员
# 才能编程 PMU——不提权只会得到 "Microarchitecture performance insights will
# not be available" + "Level 2/3 capability: not detected"，拿到的还是用户态采样。
#
# 用法（由外部 Start-Process -Verb RunAs 拉起）：
#   powershell -File tools/vtune_collect.ps1 -ResultDir out/vt_s4 -Sub 4 -Stamp 1
param(
    [string]$ResultDir = "out\vt_run",
    [string]$Sub = "4",
    [string]$Stamp = "1",
    [int]$Rounds = 15,
    [string]$Exe = ""
)

$ErrorActionPreference = "Stop"
$root = "C:\Users\15796\WorkBuddy AI\CNCSIM"
$vt = "C:\Program Files (x86)\Intel\oneAPI\vtune\latest\bin64\vtune.exe"

if (-not $Exe) {
    $Exe = (Get-ChildItem "$root\target\release\deps\creator_analysis-*.exe" |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
}

# 目标进程要读这三个环境变量；提权进程不继承外部环境，所以在这里显式设
$env:SUB = $Sub
$env:STAMP = $Stamp
$env:ROUNDS = "$Rounds"

$res = Join-Path $root $ResultDir
$log = "$res.log"
if (Test-Path $res) { Remove-Item $res -Recurse -Force }

& $vt -collect memory-access -knob sampling-interval=5 -result-dir $res `
    -- $Exe profile_run --ignored --nocapture *> $log

Write-Output ("exit=" + $LASTEXITCODE)
Write-Output ("log=" + $log)
