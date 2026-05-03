$client = New-Object System.Net.Sockets.TcpClient("10.199.28.138",5000)
$stream = $client.GetStream()
$writer = New-Object System.IO.StreamWriter($stream)
$writer.AutoFlush = $true
$writer.WriteLine("PING")
Start-Sleep -Milliseconds 200
$reader = New-Object System.IO.StreamReader($stream)
# lit une ligne si ACK line-mode
$stream.ReadTimeout = 1000
try { $reader.ReadLine() } catch { "No reply" }
$client.Close()