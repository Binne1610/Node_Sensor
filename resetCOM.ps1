param (
    [Parameter(Mandatory = $true)]
    [string]$ComPort
)

Write-Host "=== Resetting $ComPort ==="

# 1) Kiểm tra COM có tồn tại không
$device = Get-PnpDevice | Where-Object { $_.FriendlyName -match $ComPort }

if (-not $device) {
    Write-Host "❌ Không tìm thấy thiết bị $ComPort trong hệ thống."
    exit 1
}

Write-Host "✔ Tìm thấy thiết bị: $($device.FriendlyName)"
$instanceId = $device.InstanceId

# 2) Tìm các tiến trình có thể đang giữ COM
Write-Host "`n🔍 Đang kiểm tra tiến trình đang giữ $ComPort..."

try {
    $handles = (handle.exe $ComPort 2>$null | Select-String ".exe")
    if ($handles) {
        $pids = $handles | ForEach-Object {
            ($_ -split "\s+")[-1]
        }

        foreach ($pid in $pids) {
            Write-Host "⚠️  Tiến trình PID $pid đang giữ $ComPort — killing..."
            taskkill /PID $pid /F | Out-Null
        }
    } else {
        Write-Host "✔ Không có tiến trình nào đang giữ $ComPort."
    }
}
catch {
    Write-Host "⚠️ handle.exe không có — bỏ qua bước kiểm tra tiến trình."
}

# 3) Disable COM
Write-Host "`n⛔ Disabling device $ComPort..."
Disable-PnpDevice -InstanceId $instanceId -Confirm:$false

Start-Sleep -Seconds 1

# 4) Enable COM
Write-Host "▶ Enabling device $ComPort..."
Enable-PnpDevice -InstanceId $instanceId -Confirm:$false

Write-Host "`n✅ Hoàn tất reset $ComPort!"
