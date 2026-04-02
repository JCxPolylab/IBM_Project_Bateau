$udp = New-Object System.Net.Sockets.UdpClient
$udp.Connect("10.199.28.138",5000)
$bytes = [Text.Encoding]::ASCII.GetBytes("PING")
$udp.Send($bytes,$bytes.Length) | Out-Null
$udp.Close()