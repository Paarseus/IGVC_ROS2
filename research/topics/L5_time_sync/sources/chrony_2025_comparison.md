# c h r o n y

  * [Introduction](./index.html)
  * [News](./news.html)
  * [Download](./download.html)
  * [Documentation](./documentation.html)
  * [FAQ](./faq.html)
  * [Examples](./examples.html)
  * [Mailing lists](./lists.html)
  * [Comparison](./comparison.html)
  * [Contributing](./contributing.html)
  * [Links](./links.html)



# Comparison of NTP implementations

This document compares features and performance of the following NTP implementations:

  * [chrony-4.9](https://chrony-project.org)

  * [ntp-4.2.8p18](https://www.ntp.org)

  * [ntpsec-1.2.5](https://www.ntpsec.org)

  * [openntpd-7.9p1 (portable)](https://www.openntpd.org/portable.html)




The presence of the features was determined from the documentation, observed behaviour, and source code. There may be mistakes. Please let us know if you find any.

## Features

### Basic

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Supported systems | Linux, FreeBSD, NetBSD, OpenBSD, illumos, macOS | Linux, FreeBSD, NetBSD, OpenBSD, Solaris, macOS, Windows, …​ | Linux, FreeBSD, NetBSD, OpenBSD, macOS | Linux, FreeBSD, NetBSD, OpenBSD, Solaris, macOS  
License | GPLv2 | MIT + BSD | BSD 2-Clause + BSD 3-Clause, MIT | BSD  
Programming language | C | C | C, Python | C  
Size of stripped daemon binary in default configuration on Linux x86-64 | 343 KB | 925 KB | 315 KB | 108 KB  
  
### Time sources

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
NTP | Yes | Yes | Yes | Yes  
Reference clocks | Yes | Yes | Yes | Yes  
Manual input | Yes | No | No | No  
  
### Source tracking

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Fixed sample filtering | Yes | Yes | Yes | Yes  
Adaptive sample filtering | Yes | No | No | No  
Sample weighting | Yes | No | No | No  
Frequency tracking | Yes | No | No | No  
Restore state from file | Yes | No | No | No  
  
### Source selection

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Nonrandom selection | Yes | Yes | Yes | Yes  
Falseticker detection | Yes | Yes | Yes | No  
Clustering | No | Yes | Yes | No  
Offset combining | Yes | Yes | Yes | No  
Frequency combining | Yes | N/A | N/A | N/A  
Minimum number of sources | 1 (configurable) | 1 (configurable) | 1 (configurable) | 1  
  
### Clock discipline

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Independent phase and frequency control | Yes | No | No | Yes  
Allowed random update interval (e.g. intermittent connection) | Yes | No | No | Yes  
Step threshold | Infinity (configurable) | 0.128 s (configurable) | 0.128 s (configurable) | N/A  
Limited number of steps | Yes (configurable) | No | No | Yes  
Panic threshold | Infinity (configurable) | 1000 s (configurable) | 1000 s (configurable) | N/A  
Maximum slew rate | System specific (Linux: 100000 ppm, FreeBSD, NetBSD, macOS: 5000 ppm, illumos: 32500 ppm) (configurable) | 500 ppm | 500 ppm | System specific (Linux: 500 ppm, FreeBSD, NetBSD: 5000 ppm, illumos: 65000 ppm)  
Restore frequency from file | Yes | Yes | Yes | Yes  
Limited wakeups (power saving) | Yes | No | No | Yes  
Temperature compensation | Yes | No | No | No  
  
### NTP RFCs

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
RFC 5905 (NTPv4) | Yes | Yes | Yes | Yes  
RFC 5906 (Autokey) | No | Yes | No | No  
RFC 7821 (checksum complement) | No | No | No | No  
RFC 7822 (extension fields) | Yes | No | Yes | No  
RFC 8573 (AES-CMAC MAC) | Yes | Yes | Yes | No  
RFC 8915 (Network Time Security) | Yes | No | Yes | No  
RFC 9109 (port randomization) | Yes | No | No | Yes  
RFC 9327 (control messages) | No | Yes | Yes | No  
RFC 9523 (Khronos) | No | No | No | No  
RFC 9769 (interleaved modes) | Yes | Yes | No | No  
RFC 10030 (NTP over PTP) | Yes | No | No | No  
  
### NTP modes

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Basic client | Yes | Yes | Yes | Yes  
Interleaved client | Yes | ? | No | No  
Basic server | Yes | Yes | Yes | Yes  
Interleaved server | Yes | No | No | No  
Basic active symmetric | Yes | Yes | No | No  
Interleaved active symmetric | Yes | Yes | No | No  
Basic passive+ephemeral symmetric | No | Yes | No | No  
Interleaved passive+ephemeral symmetric | No | Yes | No | No  
Basic broadcast server | Yes | Yes | No | No  
Interleaved broadcast server | No | Yes | No | No  
Basic broadcast client | No | Yes | No | No  
Interleaved broadcast client | No | Yes | No | No  
  
### NTP client

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Multiple servers per name (pool) | Yes | Yes | Yes | Yes  
Fixed delay-based sample filtering | Yes | Yes | Yes | Yes  
Adaptive delay-based sample filtering | Yes | No | No | No  
Estimation of asymmetric jitter | Yes | No | No | No  
KoD RATE handling | Yes | Yes | Yes | No  
Ready for next NTP era (year 2036) | Yes | Yes | Yes | No  
Extra timestamp validation | No | No | No | Yes (HTTPS date)  
Configurable port number | Yes | No | No | No  
Multicast messaging | No | Yes | No | No  
  
### NTP server

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Protocol version | NTPv4 | NTPv4 | NTPv4 | SNTPv4  
Root dispersion/delay accumulation | Yes | Yes | Yes | No  
Adaptive dispersion rate | Yes | No | No | N/A  
Access control | Yes | Yes | Yes | No  
Response rate limiting | Yes | Yes | Yes | No  
Local reference | Yes | Yes | Yes | No  
Orphan mode | Yes | Yes | Yes | No  
Served time not fixed to system time | Yes | No | No | Yes  
Time smoothing | Yes | N/A | N/A | No  
Configurable port number | Yes | No | No | No  
Multicast messaging | No | Yes | No | No  
  
### NTP authentication

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Symmetric key | Yes | Yes | Yes | No  
Autokey (insecure) | No | Yes | No | No  
Network Time Security (NTS) | Yes | No | Yes | No  
MS-SNTP via Samba | Yes | Yes (may deny service to other users) | Yes (may deny service to other users) | No  
MAC hash functions | MD5, SHA-1, SHA-2, …​ | MD5, SHA-1, SHA-2, …​ | MD5, SHA-1, SHA-2, …​ | N/A  
CMAC ciphers | AES-128, AES-256 | AES-128 | AES-128, AES-256, …​ | N/A  
NTS AEADs | AES-SIV-CMAC-256, AES-128-GCM-SIV | N/A | AES-SIV-CMAC-256, AES-SIV-CMAC-384, AES-SIV-CMAC-512 | N/A  
  
### NTP pool use

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Number of used servers | 4 (configurable) | 10 (configurable) | 10 (configurable) | By DNS  
Replace unreachable | Yes | Yes | Yes | No  
Replace falsetickers | Yes | Yes | Yes | N/A  
  
### NTP poll control

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Polling interval | 64-1024 s (configurable) | 64-1024 s (configurable) | 64-1024 s (configurable) | 5-1500 s  
Minimum configurable polling interval | 1/128 s | 8 s | 1 s | N/A  
Randomization | Yes | Yes | Yes | Yes  
Burst | Yes | Yes | Yes | No  
Interval independent from other sources | Yes | Yes | Yes | No  
Aware of jitter | Yes | Yes | Yes | No  
User-controlled polling | Yes | No | No | No  
  
### NTP timestamping

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Kernel RX timestamping | Yes | Yes | Yes | Yes  
Kernel TX timestamping | Yes (Linux) | No | No | No  
Hardware RX timestamping | Yes (Linux) | No | No | No  
Hardware TX timestamping | Yes (Linux) | No | No | No  
  
### Reference clocks

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
System drivers | PPS, PTP (Linux), RTC (Linux), Timedelta (OpenBSD) | PPS | PPS | Timedelta (OpenBSD)  
Interfaces for 3rd party drivers | SHM, SOCK | SHM | SHM | None  
Number of HW-specific drivers | 0 | > 30 | 12 | 0  
Sample filtering | Yes | Yes | Yes | Yes  
  
### Real-time clock (RTC)

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Time initialization from RTC | Yes (Linux) | No | No | No  
RTC drift tracking | Yes (Linux) | No | No | No  
RTC trimming | Yes (Linux) | No | No | No  
Kernel RTC synchronization | Yes (Linux, OpenBSD, macOS) | Yes (Linux) | Yes (Linux) | Yes (Linux)  
Restore time from file w/o RTC | Yes | No | No | No  
  
### Leap seconds

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Clock correction modes | system, step, slew, ignore | system, step, ignore | system, step, ignore | ignore  
Majority of sources required to agree on leap | Yes | Yes | Yes | No  
Additional leap second source | system tzdata | leapfile | leapfile | N/A  
Server leap smear | Yes (quadratic) | Yes (cosine) | Yes (cosine) | N/A  
Accepted on | Jun 30 / Dec 31 | any day | any day | any day  
Applied on | Jun 30 / Dec 31 | last day of any month | last day of any month | N/A  
Announced on | Jun 30 / Dec 31 | last day of any month | last day of any month | any day  
  
### Runtime management

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Local monitoring | Yes | Yes | Yes | Yes  
Local configuration | Yes | Yes | Yes | No  
Remote monitoring | Yes | Yes | Yes | No  
Remote configuration | No (`chrony` >= 2.2) | Yes | Yes | No  
Restricted access | Yes | Yes | Yes | N/A  
  
### Security

| `chrony` | `ntp` | `ntpsec` | `openntpd`  
---|---|---|---|---  
Root privileges dropping (in all processes) | Yes (Linux) | Yes (Linux, NetBSD, illumos) | Yes (Linux, NetBSD) | No  
Privilege separation | Yes (FreeBSD, NetBSD, OpenBSD, macOS, illumos) | No | No | Yes  
System call filter (seccomp, pledge) | Yes (Linux, OpenBSD) | Yes (Linux) | Yes (Linux) | Yes (OpenBSD)  
Random NTP client source port | Yes | No | No | Yes  
Fully random transmit timestamp in client packets | Yes | No | Yes | Yes  
Sub-second randomization of polling interval | Yes | No | No | No  
Connected NTP client sockets | Yes | No | No | Yes  
NTP server port disabled by default | Yes | No | No | Yes  
Remote management disabled by default | N/A | No | Yes | N/A  
Remote management port separate from NTP | Yes | No | No | N/A  
No traffic amplification in management protocol | Yes | No | No | N/A  
Non-blocking response rate limiting | Yes | No | No | N/A  
  
## Performance

This is a comparison of accuracies that can be achieved when the NTP implementations are used as NTP clients in various clock and network conditions. The accuracy of the synchronized clock was measured in a [simulated Linux environment](https://gitlab.com/chrony/clknetsim). The results are mean values and standard deviations from 100 simulations. The values are in microseconds.

### Test 1: permanent network connection and stable clock

In this test with one NTP server the clients were using their default polling configuration. The clock was relatively stable (1ppb/s wander).

Network jitter | `chrony` | `ntp` | `openntpd`  
---|---|---|---  
10 μs | 35 ± 8 | 234 ± 46 | 857 ± 226  
100 μs | 109 ± 14 | 256 ± 50 | 888 ± 221  
1.0 ms | 475 ± 93 | 454 ± 94 | 980 ± 262  
10.0 ms | 1603 ± 447 | 3665 ± 651 | 2014 ± 387  
  
### Test 2: permanent network connection and less stable clock

In this test the polling interval of the clients was fixed to 64 seconds and the clock was less stable (10ppb/s wander). `openntpd` couldn't be included as its polling interval is not configurable.

Network jitter | `chrony` | `ntp` | `openntpd`  
---|---|---|---  
10 μs | 14 ± 0 | 165 ± 17 | N/A  
100 μs | 56 ± 3 | 167 ± 18 | N/A  
1.0 ms | 229 ± 15 | 217 ± 17 | N/A  
10.0 ms | 750 ± 91 | 1467 ± 100 | N/A  
  
### Test 3: intermittent network connection

In this test the network was available to the clients only for 30 continuous minutes every 24 hours. The polling interval configuration and the clock wander were the same as in the first test.

Network jitter | `chrony` | `ntp` | `openntpd`  
---|---|---|---  
10 μs | 7273 ± 1744 | 608803 ± 510468 | 170583 ± 140321  
100 μs | 9528 ± 1895 | 580679 ± 481379 | 160203 ± 112421  
1 ms | 10706 ± 2521 | 1115961 ± 733914 | 168645 ± 126309  
10 ms | 26105 ± 70408 | 897703 ± 847901 | 285437 ± 295667  
  
## Summary

### `chrony` vs `ntp` and `ntpsec`

Things `chrony` can do better than `ntp` and `ntpsec`:

  * `chrony` can perform usefully in an environment where access to the time reference is intermittent. `ntp` and `ntpsec` need regular polling of the reference to work well.

  * `chrony` can usually synchronise the clock faster and with better time accuracy.

  * `chrony` quickly adapts to sudden changes in the rate of the clock (e.g. due to changes in the temperature of the crystal oscillator). `ntp` and `ntpsec` may need a long time to settle down again.

  * `chrony` can perform well even when the network is congested for longer periods of time.

  * `chrony` in the default configuration never steps the time to not upset other running programs. `ntp` and `ntpsec` can be configured to never step the time too, but in that case it has to use a different means of adjusting the clock (daemon loop instead of kernel discipline), which may have a negative effect on accuracy of the clock.

  * `chrony` can adjust the rate of the clock in a larger range, which allows it to operate even on machines with broken or unstable clock (e.g. in some virtual machines).

  * `chrony` is smaller, it uses less memory and it wakes up the CPU only when necessary, which is better for power saving.




Things `chrony` can do that `ntp` and `ntpsec` can't:

  * `chrony` supports the Network Time Security (NTS) authentication mechanism (supported by `ntpsec`, but not `ntp`).

  * `chrony` supports hardware timestamping on Linux, which allows an extremely stable and accurate synchronisation in local network.

  * `chrony` provides support for isolated networks whether the only method of time correction is manual entry (e.g. by the administrator looking at a clock). `chrony` can look at the errors corrected at different updates to work out the rate at which the computer gains or loses time, and use this estimate to trim the computer clock subsequently.

  * `chrony` provides support to work out the gain or loss rate of the real-time clock, i.e. the clock that maintains the time when the computer is turned off. It can use this data when the system boots to set the system time from a corrected version of the real-time clock. These real-time clock facilities are only available on Linux, so far.




Things `ntpsec` can do that `chrony` can't:

  * `ntpsec` includes drivers for most current hardware reference clocks, although fewer than `ntp`.




Things `ntp` can do that `chrony` can't:

  * `ntp` supports all operating modes from RFC 5905, including broadcast, multicast, and manycast server/client. However, the broadcast and multicast modes are inherently less accurate and less secure (even with authentication) than the ordinary server/client mode, and should generally be avoided.

  * `ntp` supports the Autokey protocol (RFC 5906) to authenticate servers with public-key cryptography. Note that the protocol has been shown to be insecure and has been obsoleted by NTS (RFC 8915).

  * `ntp` has been ported to more operating systems.

  * `ntp` includes a large number of drivers for various hardware reference clocks. `chrony` requires other programs (e.g. `gpsd` or `ntp-refclock`) to provide reference time via the `SHM` or `SOCK` interface.




Last updated 2026-09-03 14:53:22 +0200 
