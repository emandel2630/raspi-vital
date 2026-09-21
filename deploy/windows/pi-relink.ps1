# Point the WSL->Pi SSH relay at the Pi's current IPv6 link-local address.
#
# On a direct cable with no DHCP the Pi only has an IPv6 link-local address.
# WSL sits behind its own NAT and cannot use a link-local (it is scoped to the
# Windows Ethernet interface), so Windows relays 172.17.80.1:2222 to the Pi's
# [fe80::...%<if>]:22. The Pi's address can change across reboots, so re-resolve
# it by mDNS and rewrite the rule. Must run elevated (netsh portproxy).
$ErrorActionPreference = 'Continue'

$ifIdx = (Get-NetAdapter -Name 'Ethernet').ifIndex
$addr = $null
foreach ($try in 1..10) {
    try {
        $addr = (Resolve-DnsName -Name 'emandelraspi2.local' -Type AAAA -DnsOnly:$false -ErrorAction Stop |
                 Where-Object { $_.IPAddress -like 'fe80*' } | Select-Object -First 1).IPAddress
    } catch { }
    if ($addr) { break }
    Start-Sleep -Seconds 2
}
if (-not $addr) {
    Write-Host "Pi did not answer mDNS on the Ethernet link. Is it powered and plugged in?"
    exit 1
}

$target = "$addr%$ifIdx"
Write-Host "Pi link-local: $target"
netsh interface portproxy delete v4tov6 listenaddress=172.17.80.1 listenport=2222 | Out-Null
netsh interface portproxy add v4tov6 listenaddress=172.17.80.1 listenport=2222 `
      connectaddress="$target" connectport=22 | Out-Null
if (-not (Get-NetFirewallRule -DisplayName 'WSL to RasPi SSH proxy' -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -DisplayName 'WSL to RasPi SSH proxy' -Direction Inbound -LocalPort 2222 `
        -Protocol TCP -Action Allow -Profile Any | Out-Null
}
netsh interface portproxy show all
Write-Host "`nFrom WSL:  ssh -p 2222 <user>@172.17.80.1"
