

    Jetson Orin Series — NVIDIA Jetson Linux Developer Guide

  Skip to main content

    Back to top

  Ctrl+K

    NVIDIA Jetson Linux Developer Guide

    NVIDIA Jetson Linux Developer Guide

  Table of Contents

  Introduction

Welcome

Quick Start

Architecture

Jetson Software Architecture

Boot Architecture
Jetson AGX Orin, Orin NX, and Orin Nano Boot Flow

Partition Configuration

Software Feature Overview

Jetson Orin Series

Software Features in Depth

Flashing Support

Emulation Flash Configurations

Root File System

Bootloader
T23x Boot Configuration Table

Pinmux and GPIO Configuration

Common Prod Configuration

Controller Product Configuration

Pad Voltage DT Binding

PMIC Configuration

Storage Device Configuration

UPHY Lane Configuration

OEM-FW Ratchet Configuration

BootROM Reset PMIC Configuration

Miscellaneous Configuration

SDRAM Configuration

DRAM-ECC

GPIO Interrupt Mapping Configuration

MB2 BCT Misc Configuration

Security Configuration

UEFI Adaptation

Update and Redundancy

Kernel
Kernel Adaptation

Kernel Customization

Installing Real-Time Kernel

Bring Your Own Kernel

Generic Timestamp Engine

BMI088 IMU Driver

Kernel Boot Time Optimization

Display Configuration and Bring-Up
Common Display configurations for all Platforms

Orin specific Display Configuration

Kernel Debugging Tools

Multimedia
Multimedia APIs

Accelerated Decode with ffmpeg

Accelerated GStreamer

Software Encode in Orin Nano

Hardware Acceleration in the WebRTC Framework

Graphics
Graphics APIs

Graphics Programming
Binary Shader Program Management

GLSLC Shader Program Compiler

OpenGL ES Programming Tips

EGLStream

EGLDevice

Sample Applications

OpenWFD

Vulkan SC

Vulkan SC Samples

Windowing Systems
Weston (Wayland)

X Window System

Camera Development
Camera Software Development Solution

Sensor Software Driver Programming

Jetson Virtual Channel with GMSL Camera Framework

Argus NvRaw Tool

Camera Driver Porting

Security
Secure Boot

Factory Secure Key and Expansion Key Provisioning

OP-TEE: Open Portable Trusted Execution Environment

Disk Encryption

Secure Storage

Firmware TPM

Rollback Protection

Memory Encryption

PVA Authentication

Communications
PCIe Endpoint Mode

Enabling Bluetooth Audio

Audio Setup and Development

Clocks

Platform Power and Performance
Jetson Orin Nano Series, Jetson Orin NX Series and Jetson AGX Orin Series

Software Packages and the Update Mechanism

Boot Time Optimization

Working With Sources

Test Plan and Validation

Hardware References

Jetson Developer Kit Setup

Jetson EEPROM Layout

Jetson Module Adaptation and Bring-Up
Jetson AGX Orin Series

Jetson Orin NX and Nano Series

Checklists

Configuring the Jetson Expansion Headers

Controller Area Network (CAN)

Applications and Tools

Board Automation

Jetson Linux Toolchain

Jetson Linux Development Tools
Debugging on Jetson Platforms

Program Trace Macrocell

Tegrastats Utility

Tegra Combined UART

How to Submit a Bug Report

Reference Material

Package Manifest

Related Documentation

Legal Information

    Jetson Orin Series

Jetson Orin Series#

NVIDIA® Jetson™ Linux supports these software features, which provide users a complete package to bring up Linux on Jetson AGX Orin™, Jetson Orin™ NX, and Jetson Orin™ Nano devices.

Bootloader#

Bootloader Binary

Feature

Notes

BPMP processor
boot binaries

Storage location

Cold boot: QSPI

RCM boot: Downloaded over USB
recovery port

Next stage storage
location

Cold boot: QSPI

RCM boot: Downloaded over USB
recovery port

Next stage

UEFI

Storage device
support

QSPI

Partition table
support

GPT (with protective MBR)

File system support

None

I/O bus support

Console UART

I2C

Toolchain#

Feature

Tool chains

Notes

Aarch64

gcc-11.3-glibc-2.35

For 64-bit kernel and user space

Kernel#

Interface

Feature

Linux kernel

Version 5.15.148

Camera Interface#

Platform

Interface

Feature

Notes

AGX Orin

Camera support (CSI
input support)

V4L2 Media-Controller
(V4L2 API bypasses ISP)

CSI0, CSI1, CSI2, CSI3,
CSI4, CSI5, CSI6, CSI7

Orin NX and
Orin Nano

Camera support (CSI
input support)

V4L2 Media-Controller
(V4L2 API bypasses ISP)

CSI0, CSI1, CSI2, CSI3

Note

We recommend using a camera with a frame rate of less than or equal to 60 FPS.

LSIO#

Module

Feature

Notes

UART

PIO mode

FIFO access using CPU.

DMA mode

FIFO access using DMA.

Hardware-/software-based flow
control

Flow control line toggling from
hardware or software.

Buffer throttling

Flow control based on data in receive
buffer.

RX and TX DMA mode selection

DMA mode transfer on RX and TX or on
only one path.

Interrupt mode

Data transfer complete handling through
interrupt.

Polling mode

Data transfer complete handling through
polling.

MCR control

Modem control access.

Baud rate/port configuration

Changing port configuration.

Baud rate adjustment

Adjusting baud rate to fall within
tolerance range.

I2C
controller

Speed mode (standard, FM, FM+)

Repeat start

Repeat start on transfer of data.

No start

No address cycle after repeat start.

Packet mode

Normal/byte mode

7-bit/10-bit addressing mode

7-bit/10-bit addressing mode.

DMA mode

APB/GPC DMA for FIFO access.

Clock gating and clock always
on

Clock control after each transfer for
power saving.

Runtime PM

Runtime power management.

Dynamic clock speed change

Change speed of the bus.

Interrupt-based

Transfer complete handling using
interrupt.

Polling

Transfer complete handling using
polling.

Bit banging for data transfer

Use GPIO APIs for data transfer.

Multiple transfer request

Multiple transfer request.

Bus clear support

Bus clear handling when bus is held by
device.

>4K on software-based split

>4K on software-based split.

>64K on software-based split

>64K on software-based split.

SPI
controller

Packed/unpacked

Data can be put on FIFO in packed or
unpacked format. Packed format reduces
the number of I/O accesses on FIFO.

Full duplex mode

Device can read and write data
simultaneously.

Least significant bit

Option to send least significant bit
first from packets.

Dual SPI

SPI PICO/POCI (previously MOSI/MISO)
can act as RX and TX.

Least significant byte first

Option to send least significant byte
first from packets.

Hardware-based CS control and
CS setup/hold time

Hardware control the CS and maintain CS
setup and hold time.

Software or hardware chip
select polarity section

Chip select can be active high or
active low based on the external device
property.

Supported modes 0–3

SPI communication support mode 0, 1, 2,
or 3.

DMA mode

Data written to and read from FIFO using
DMA mode.

PIO (non-DMA) mode

CPU has direct access to FIFO for
read and write.

GPIO-based chip select

CS line is controlled by the GPIO APIs.

SPI different clock rates

Set the interface clock speed based on
what device can support.

Prod configuration

Platform-/chip-specific configuration of
controller and interface.

Clock delay between packets

Provision for delay between packets.

Clock gating and clock always
on

Dynamic clock enable/disable for power
save.

Runtime PM

Runtime power management.

Interrupt-based

Transfer-done handling through
interrupt.

Polling

Transfer-done handling through polling.

Different packet bit length

Multiple-transfer request

Multiple SPI transfer request from a
single call.

GPIO

GPIO request/free

GPIO access permission.

Pinmux integration with GPIOS

GPIO APIs call pinmux for required pin
configuration.

Direction set/get

GPIO direction configuration.

Value set/get

GPIO value set to pin and get from pin.

Interrupt support from all
pins

Wake-up support for SC7

GPIO register dump

Support for libgpiod library
and tools

Suspend/resume

Pinmux

Function configuration

Pinmux function configuration.

Pinmux configuration

Configuration of various pinmux
properties, such as pull up/pull down,
input, and tristate.

Suspend/resume

Save and restore pinmux context.

Drive strength

Drive strength configuration of pins.

Prod setting

Static pinmux configuration

Dynamic pinmux configuration

Pinmux register dump

Pinmux configuration dumping

Pinmux-GPIO integration

APBDMA/
GPCDMA

Memory to memory

Memory to I/O

I/O to memory

Cyclic-once mode

Transfer done through
interrupt mode

Multiple transfer requests

Queue mechanism for transfer requests.

Watchdog

Watchdog framework support

System reset on CPU hang

System reset on WDT expiry.

Suspend/resume support

Suspend/resume handling.

Watchdog interrupt support

WDT reset on ISR.

Watchdog polling/ping support

WDT start/stop/pin from user space.

PWM

PWM ops

PWM registration to framework.

Prod setting

Tegra-specific controller configuration.

Clock accuracy calculation

PMC

Controlling I/O PAD voltage
(PWR_DETECT)

Pad voltage configuration by software.

I/O DPD configuration

Deep power-down configuration.

IO_NOPOWER through regulator

IO_NOPOWER configuration.

Read/write PMC registers

PMC register access interface.

PMC config for BootROM
I2C

PMC configuration for BootROM
I2C/MMIO commands.

PMC LED blink

PMC provides the control for blinking
LED, including in deep-sleep mode. The
blinking control is a PWM signal.

PMC soft LED breathing

PMC soft LED breathing to control LED
ramp up, ramp down, and on time.

Tachometer

Read RPM

BPMP I2C
controller

Speed mode (standard, FM, FM+)

Bus speed configuration.

Packet mode

I2C controller configuration
in packet mode.

Normal/byte mode

I2C controller configuration
in normal mode.

7-bit/10-bit addressing mode

DMA mode

I2C FIFO access through APB
DMA.

Bus-clear support

Bus-clear handling when bus is held by
device.

SPE-UART

PIO mode

FIFO access using CPU.

DMA mode

FIFO access using DMA.

Hardware flow control

Flow control line toggling from
hardware and software.

FIFO mode

FIFO mode of UART controller.

SPE DMA

Memory to memory

Memory to I/O

I/O to memory

Continuous mode support

Cyclic mode.

I2C
target

Normal/byte mode

I2C controller configuration
in normal mode.

FIFO mode

I2C controller configuration
in FIFO mode.

7-bit addressing

7-bit addressing.

10-bit addressing

10-bit addressing.

Repeat start

Repeat start on transfer of data.

Clock stretching

Clock line stretching.

CAN 2.0 A

CAN FD

CAN FD increases the maximum data
throughput to ~3.7 Mbps. 10 Mbps over
10 meters. Maximum signal frequency:
15 Mbps.

TX Standard CAN message

TX Standard CAN with 11-bit message
message identifiers, originally
specified to operate at a maximum
frequency of 250 Kbps. Maximum signal
frequency: 1 Mbps. Available bitrates:
125 Kbps, 250 Kbps, 500 Kbps, and 1 Mbps.

RX Standard CAN message

RX Standard CAN with bitrates of 125
Kbps, 250 Kbps, 500 Kbps, and 1 Mbps.

TX+RX Standard CAN message

TX+RX Standard CAN with bitrates of 125
Kbps, 250 Kbps, 500 Kbps, and 1 Mbps.

TX Extended CAN message

TX Extended CAN with 29-bit message
identifiers, originally specified to
operate at a maximum frequency of 1
Mbps. Available bitrates: 125 Kbps,
250 Kbps, 500 Kbps, and 1 Mbps.

RX Extended CAN message

RX Extended CAN with bitrates of 125
Kbps, 250 Kbps, 500 Kbps, and 1 Mbps.

TX+RX Extended CAN message

TX+RX Extended CAN with bitrates of 125
Kbps, 250 Kbps, 500 Kbps, and 1 Mbps.

CAN loopback

Internal controller loopback.

Standard FD (BRS/non-BRS)

Standard FD frame with bitrate switching
and non-bitrate switching.

Extended FD (BRS/non-BRS)

Extended FD frame with bitrate switching
and non-bitrate switching.

Timestamping

Timestamp generation of TX/RX frames.

Listen-only mode

Listen-only mode to monitor CAN bus.

Restart bus-off

Restart bus after CAN bus-off state.

QSPI

Interface widths

Single, Dual, and Quad supported.

SDR and DDR modes

Data rates on one or both clock edges.

Bit lengths

Packets of 8, 16, and 32 bits
supported.

Transfer modes 0 and 3

Transfer modes 0 and 3 for SDR and mode
0 for DDR.

DMA/PIO mode selection

Option to select data written to or read
from FIFO using DMA or CPU.

Packed/unpacked

Data can be put in packed or unpacked
format on FIFO. Packed format reduces
the number of I/O accesses.

Endianness LSByte and LSBit

Option to send least-significant
byte or bit first from packet.

Hardware-based chip select
control and CS setup/hold time

Hardware control of chip select pin and
CS setup and hold time.

Software or hardware chip
select polarity selection

Chip select can be active high or active
low based on the external device
property.

Combined sequence mode

CDM, ADDR, and DATA transferred in
single GO resulting in single interrupt.

Various clock rates

Set the interface clock speed based on
what device can support.

Golden Register settings

Platform-/chip-specific configuration of
controller and interface.

Active clock delay between
packets

Provision to have delay between packets.

Runtime power management

HSIO#

Module

Feature

Notes

EthernetControllerFeaturesEqos

Speed mode change through ethtool

10/100 Mbps support

1000 Mbps support

10000 Mbps support

Half-duplex support

ARP offload

IEEE 1588-2008 (PTP)

Energy-Efficient Ethernet (EEE)

Transmit checksum offload

Receive checksum offload

TCP segmentation offload

Jumbo frame support

Up to 9 KB (9018 bytes untagged or 9022
bytes tagged).

Flow control/PAUSE frame support

EAVB support

Up to 4 TX/RX queue/channels with 4 KB
size

VLAN (insertion/stripping of VLAN tag
in hardware)

VLAN tag-based filtering supported
for only one VLAN tag.

Ethernet

Ping

Remote wake-up

NFS boot

Suspend/resume support over NFS

PCIe

Controllers with x8 link width

Max x8 link width (few controllers).

Controllers with x4 link width

Max x4 link width (few controllers).

Controllers with x2 link width

Max x4 link width (few controllers).

Controllers with x1 link width

Max x1 link width (few controllers).

Legacy interrupts

Applicable to all controllers.

MSI & MSI-X interrupts

Applicable to all controllers.

128-byte maximum payload size

Applicable to all controllers.

256-byte maximum payload size

Applicable to all controllers.

Gen-1 speed

Applicable to all controllers.

Gen-2 speed

Applicable to all controllers.

Gen-3 speed

Applicable to all controllers.

Gen-4 speed (not applicable to Orin
Nano)

Applicable to all controllers.

ASPM - L0s

Applicable to all controllers (enabled
by default only on C1 controller).

ASPM - L1

ASPM - L1.1

ASPM - L1.2

Wake support

Applicable to all controllers.

Advanced error reporting (AER)

Applicable to all controllers.

DMA support in Root Port

Applicable to all controllers.

Endpoint mode support

AGX Orin: C5 and C7 controllers.

Orin NX/Nano: C4 controllers.

SDMMC

DR50

eMMC interface running in DDR mode at
50 MHz.

HS200

eMMC interface running in SDR mode at
200 MHz.

HS400

eMMC interface running in DDR mode at
200 MHz.

HS533

eMMC interface running in DDR mode at
267 MHz.

HW tuning

Supports tuning in SDMMC controller.

Packed commands

Read and write commands can be packed
in groups (either all read or all write)
that transfer data for all commands in
the group in one transfer on the bus,
to reduce overhead.

Cache

Similar to CPU cache, but implemented
in eMMC; helps improve performance.

Discard

Erases data if necessary during
background erase events.

Sanitize

Physically removes data from unmapped
user address space.

RPMB

Secure access.

BKOPS

Allows execution of background
operations when host is not being
serviced.

HPI

High-priority interrupt to stop ongoing
background operations and reliable
writes.

Power-off notification

Allows device to prepare itself to
power off properly and improve user
experience during power-on.

Sleep

Minimizes power consumption of the eMMC
device.

RTPM

Software feature to save power by
switching off clocks when no
transactions are on the bus.

Field firmware upgrade

Update eMMC firmware.

Device Life Estimation types A and B

Device Health is a mechanism to get
vital NAND flash program/erase cycles
information as a percentage of useful
flash lifespan.

Type A: SLC device health information

Type B: MLC device health information

PRE EOL information

Provides indication about device
lifetime reflected by average reserved
blocks.

Hardware command queue

Performed by SD/MMC controller.

Enhanced strobe mode (ESM) in HS400 mode

Optional for devices; indicated by
STROBE_SUPPORT[184] register of EXT_CSD.

eMMC CQ CQIC feature

Generates coalesced interrupts when
the interrupt coalescing mechanism is
enabled.

Suspend/resume and shutdown

UFS

PWM-G1

PWM-G2

PWM-G3

PWM-G4

PWM-G5

PWM-G6

UFS (m-phy) interface runs in low
performance (PWM-Gx) modes.

HS-G1

HS-G2

HS-G3

HS-G4

UFS (m-phy) interface runs in high
performance (HS-Gx) modes.

Native Command Queue support

Hibernation

Low power state.

Runtime time power management

Driver issues software hibernation
entry in runtime suspend, and
hibernation exit in runtime resume.

Auto hibernation

Hibernation triggered by controller.

PWM SLOW modes

PWM SLOW_AUTO modes

HS FAST modes

HS FAST_AUTO modes

HS RATE_A series

HS RATE_B series

USB 3.0

SuperSpeedPlus host

USB host in 3.1 Gen2 mode (10 Gbps).

SuperSpeed host

USB host in 3.0 mode (5 Gbps).

High Speed host

USB host in 2.0 mode (480 Mbps).

Full Speed host

USB host in 2.0 or 1.2 mode (12 Mbps).

Low Speed host

USB host in 2.0 or 1.2 mode (1.5 Mbps).

Auto-suspend

USB host suspends the port/connected
device if there is no activity.

Remote wake-up

USB host resumes the port/connected
device if wake-up is triggered by the
device.

Auto-resume

USB host resumes the port/connected
device if wake-up is triggered by the
host.

ELPG for xUSB High Speed partition

Engine-level power gating support for
xUSB High Speed partition.

ELPG for xUSB SuperSpeed partition

Engine-level power gating support for
xUSB SuperSpeed partition.

Lower power state (U3 state)

LPM states (U1, U2 states)

Hot plug support

USB drives can be removed and connected
while system is active.

Port multiplier support

Hub for USB.

Host mass storage

Protocol for storage devices.

Host USB video class

Protocol for camera devices.

Host USB ECM

Protocol for ethernet over USB.

Host USB audio class

Protocol for audio over USB.

Host USB modem—NCM

NCM protocol support for modem
functionality.

USB HID protocol

Human interface devices.

SuperSpeed device (xUSB)

USB device in 3.0 mode.

High Speed device (xUSB)

USB device in 2.0 mode.

BC1.2 charging support

Support for battery charging per BC1.2
spec.

Apple charger

Support for detecting Apple charger.

MTP device mode

MTP protocol support for data transfer.

ADB device mode

ADB protocol support for data transfer.

RNDIS device mode

RNDIS protocol support for data transfer.

OTG

USB host and device (cable-based
detection).

HDMI#

Feature

Details

EDID support

Read and parse EDID.

Hot-plug
detection

Hot-plug detection with HDMI® monitors and TVs.

HDMI 1.4

Support for HDMI 1.4 (480p, 720p, 1080p, and 4K
at 30 Hz).

HDMI 2.0

Support for HDMI 2.0 (4K at 30 Hz and 60 Hz).

4K at 60 Hz is not supported on Orin Nano.

Driver
suspend/resume

Driver suspend/resume for low power.

HDMI as
primary
display

Support HDMI as the primary display.

Sideband
information

Sends the sideband information, such as
infoframes and audio data, to the panel during
video refresh.

Note

The Jetson Orin HDMI passed the HDMI2.1 certificate with the GCTS2.1f version, and only the features in the preceding table are supported.

DisplayPort#

Feature

Details

EDID

Read and parse EDID.

DP hot-plug
support

Hot-plug detection with DP monitors or TV.

DP 4K at 60 Hz

4K mode in DP.

Not supported for Orin Nano.

DP 4K at 120 Hz
or 8K at 30 Hz

HBR3 with 4K at 120 Hz or 8K at 30 Hz.

Supported only on Orin AGX; not supported for Orin NX and Nano.

Enhanced
framing

Error recovery methods.

Full link
training

Handshake signaling between host and device.

HPD_IRQ event

Feedback from the panels in case of link synchronization loss.

Driver
suspend/resume

Driver suspend/resume for low power.

Primary
display

Support DP as primary display.

DP MST

Support two different multiple streams on different monitors.

Link rates
1.62, 2.7, 5.4,
and 8.1 Gbps

Link rates supported by the driver up to HBR3.

Aux link

Support DP aux link.

Sideband
information

Sends the sideband information, such as infoframes and audio
data, to the panel during video refresh.

Note

The Jetson Orin Platform does not support embedded DisplayPort (eDP).

Security Engine#

Algorithm

Notes

AES-CBC/ECB/OFB/CTR/XTS

Uses AES1 engine running on SE2
through Host1x bus.

AES-CMAC

Uses AES1 engine running on SE2
through Host1x bus.

AEAD-AES_CCM/GCM

Uses AES1 engine running on SE2
through Host1x bus.

HMAC-SHA

Uses HASH engine running on SE4
through Host1x bus.

SHA1/2/3-224/256/384/512

Uses HASH engine running on SE4
through Host1x bus.

Power Modes (Profiles)#

Features:

Reference power profiles such as 10W, 15W, and 30W are supported for the various Jetson Orin modules.

NVPModel interface for power mode selection and custom mode creation.

RTC#

Features:

Alarm.

Wake-up from SC7.

System#

Features:

Reboot support.

Shutdown support.

SC7.

cpuidle.

Wake from idle.

Wake from sleep.

CPU hotplug.

DVFS.

CPU/GPU frequency governor.

EMC bandwidth manager.

Power monitor.

Clock and thermal management.

initrd support.

System boot with ATF as secure monitor.

Experimental generic timestamping engine (GTE) support for LIC IRQ lines and AON GPIOs.

Porting to Custom Platforms#

To adapt the software to custom platforms, follow the Orin AGX Platform Adaptation and Bring-Up guide for NVIDIA® Jetson AGX Orin™ and the Jetson Orin NX and Nano Series guide for NVIDIA® Jetson Orin™ NX.

Unsupported Features#

The SDIO feature is not supported in software. For Wi-Fi or Bluetooth use cases, we recommend using PCIe.

EMMC boot devices are not supported for Orin NX series.

        previous

        Partition Configuration

        next

        Flashing Support

     On this page

Bootloader

Toolchain

Kernel

Camera Interface

LSIO

HSIO

HDMI

DisplayPort

Security Engine

Power Modes (Profiles)

RTC

System

Porting to Custom Platforms

Unsupported Features

   so the DOM is not blocked -->

  Privacy Policy
   | 

  Manage My Privacy
   | 

  Do Not Sell or Share My Data
   | 

  Terms of Service
   | 

  Accessibility
   | 

  Corporate Policies
   | 

  Product Security
   | 

  Contact

      Copyright © 2024-2026, NVIDIA Corporation.

  Last updated on Jan 16, 2026.

