param(
  [string]$PortName = 'COM21',
  [int]$BaudRate = 115200,
  [string]$LogPath,
  [string]$StatusPath
)

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
$port = $null

try {
  $port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
  $port.ReadTimeout = 200
  $port.WriteTimeout = 200
  $port.NewLine = "`r`n"
  $port.Encoding = [System.Text.Encoding]::ASCII
  $port.Open()
  $port.DiscardInBuffer()

  [System.IO.File]::WriteAllText($StatusPath, ('OPENED ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss')), $utf8NoBom)
  [System.IO.File]::WriteAllText($LogPath, ('=== CAPTURE START ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') + ' ===' + "`r`n"), $utf8NoBom)

  while ($true) {
    try {
      $line = $port.ReadLine()
      if ($null -ne $line) {
        Add-Content -Path $LogPath -Value $line -Encoding UTF8
      }
    } catch [System.TimeoutException] {
      Start-Sleep -Milliseconds 20
    }
  }
}
catch {
  [System.IO.File]::WriteAllText($StatusPath, ('ERROR: ' + $_.Exception.Message), $utf8NoBom)
  exit 1
}
finally {
  if (($null -ne $port) -and $port.IsOpen) {
    $port.Close()
  }
}