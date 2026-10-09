# Orin Time Sync

Orin Time Sync has multiple components as described below in diagram. It consist of:

  * **PTP daemon** for synchronizing one of the Tegra's internal PHC clock which act as a source for all other local clock as base for synchronization/co-relation.
  * **PTP - TSC HW synchronization** for aligning TSC with PHC increment rate using 1HZ PPS signal input to TSC from MAC. Needed for camera Fsync signal generation alignment with PTP second boundary. 
  * **PTP bridge in Tgera (MAC-MAC)** \- To allow other MAC(s) to synchronize with GM time through the primary MAC (connected to GM) in order to support other Time aware client devices like Lidar etc connected through other MACs in Tegra.
  * **Time Co-relation Module** : Provides concurrent and correlated snapshot of multiple time domain like CCPLEX TSC, PTP etc. on a given event to be consumed by upper layer software. 



## PTP Support in Orin

As per the supported PTP topology, One of the Tegra MAC and its associated PHC needs to get synchronized with external Grand Master clock. Once synchronized this MAC PHC (PTP hardware counter) works as a base for:

  * All the Tegra internal clocks which needs to be synchronized or to be correlated as per the use-case.
  * All the software component requiring the current PTP time in system. 



## AVNU PTP for Development

In DRIVE OS Linux (AV+L), Linuxptp daemon i.e ptp4l is supposed to be run in client mode with automotive profile mode to synchronize Tegra's MGBE_2 mac interface from external GM. 
    
    
    ./ptp4l -f automotive_slave.cfg -i mgbe2_0 -p /dev/ptp<X> -m 
    // where:
    // /dev/ptp<X> - PTP dev node associated with the mgbe2_0
    

For device connected on EQOS another daemon instance can be started with following command in server mode. To identify `<x>`, use the `ethtool -T [Interface ID]` command. If ethtool is not installed on the host and DUT, use the `sudo apt install ethtool` command to install.
    
    
    ./ptp4l -f automotive_master.cfg -i eqos_0 -p /dev/ptp<X> -m 
    // where:
    // /dev/ptp<X> - PTP dev node associated with the eqos_0
    

## AUTOSAR TSync PTP for Production

Note: Tsync PTP implementation is for development purpose only and it must not be used for production.

AUTOSAR Ethernet Tsync profile PTP commands:
    
    
    / #Server Mode
    
    cd /opt/nvidia/drive-linux/samples/nvavb/daemons/
    sudo ./gptpTsync <interface> -F autosar_cfg.ini -INITPDELAY 0 -OPERPDELAY 0 -INITSYNC -3 -OPERSYNC -3 -S -V -GM
    
    // #client Mode
    
    cd /opt/nvidia/drive-linux/samples/nvavb/daemons/
    sudo ./gptpTsync <interface> -F autosar_cfg.ini -INITPDELAY 0 -OPERPDELAY 0 -INITSYNC -3 -OPERSYNC -3 -S -V -N -A
    
    // where:
    // <iface> : Interface name i.e eqos_0, mgbe0_0, mgbe1_0 on which PTP is supposed to be started. It could be a VLAN interface above the primary interface.
    

The following section explains the autosar specific configurable parameters from INI file for gptpTsync binary:
    
    
    /*
    # Time secure TLV related data
    # Specification for 10.2.4 EthTSynGlobalTimeFollowUpDataIDList, Also refer ECUC_EthTSyn_00030
    # Provide 16 data element values in decimal
    # _Provide value in decimal value only(not in hexadecimal)_ */
    FollowUpDataIdList = 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
    
    /* 
    # Time secure TLV CRC_Time_Flags for CRC calcuation in master mode.
    # Refer CRC_Time_Flags from SWS_EthTSyn_00065
    # Following values to be 'OR'ed to create flag as per requirement
    # BitMask 0x01 [messageLength]
    # BitMask 0x02 [domainNumber]
    # BitMask 0x04 [correctionField]
    # BitMask 0x08 [sourcePortIdentity]
    # BitMask 0x10 [sequenceId]
    # BitMask 0x20 [preciseOriginTimestamp]
    # Value provided in below config will be 'AND' with 0x3F to consider only valid bits
    # _Provide value in hex format only_ */
    EthTSynCrcTimeFlagsTxSecured = 0x37
     
    /*
    # Status TLV related data
    # if SyncToGateWay is set, SGW with SyncToSubDomain will be set. Otherwise, SyncToGTM will be set.
    # Valid values 0 and 1
    # _Provide value in decimal value only(not in hexadecimal)_ */
    SyncToGateWay = 0
    
    /* 
    # User data TLV number of bytes UserDataLength
    # this value should be 1 >= UserDataLength <= 3 */
    UserDataLength = 1
     
    /*
    # User data TLV Related data
    # Provide user byte of size uint8_t with space separated, i.e UserByte = UserByte_0  UserByte_1  UserByte_2
    # _Provide value in decimal value only(not in hexadecimal)_ */
    UserByte = 0 0 0
     
    /*
    # CRC validation mode for slave
    # CRC_IGNORED and CRC_VALIDATED */
    EthTSynRxCrcValidated = CRC_VALIDATED
    
    /* 
    # Static Pdelay setting.
    # Set to 1 to disable path delay calculation using Pdelay_req, Pdelay_resp, Pdelay_respfup.
    # and use EthTSynGlobalTimePropagationDelay as static Link delay.
    # Setting it to 1 will also make daemon to ignore path delay messages received.
     
    # Set 0 to enable path delay calculation using Pdelay_req, Pdelay_resp, Pdelay_respfup
    # and ignore EthTSynGlobalTimePropagationDelay.
    # _Provide value in decimal value only(not in hexadecimal)_ */
    UseStaticPdelay = 0
     
    /*
    # Static Delay in nanosecond will be used when UseStaticPdelay is set to 1
    # Default value of 0 will be used if UseStaticPdelay set and below field is not populated.
    # _Provide value in decimal value only(not in hexadecimal)_ */
    EthTSynGlobalTimePropagationDelay = 20000000
    
    /* 
    # Option to enable and disable processing and transmit of sub-TLV's.
    # Set values to 1 enable and 0 to disable
    # Time secure sub-TLV
    # _Provide value in decimal value only(not in hexadecimal)_ */
    EthTSynTLVFollowUpTimeSubTLV = 1
    /* # Status sub-TLV */
    EthTSynTLVFollowUpStatusSubTLV = 1
    /* # User dats sub-TLV */
    EthTSynTLVFollowUpUserDataSubTLV = 1
    

## Other PTP Profiles in Orin

DRIVE OS Linux and QNX supports IEEE 1588, IEEE 802.1AS and AUTOSAR EthTsync profiles. Support for these profiles are limited to the software stack running in Tegra/Orin i.e., no support in switches. This means these profiles can not be enabled on Ethernet MACs which are directly connected to the switches on the DRIVE Orin Development Platforms. For custom boards all the Tegra MACs can support these profiles. 

**IEEE 1588 (over IPv4) Command**
    
    
    // #Server Mode
    // Assign IP address to interface first
    
    sudo ./ptp4l -i <iface>  -p /dev/ptp<X> -m -4 -l7
    
    // #client Mode
    
    sudo ./ptp4l -i <iface>  -p /dev/ptp<X> -m -s -l7  -4 &
    
    // where:
    // <iface> : Interface name i.e eqos_0, mgbe0_0, mgbe1_0 on which PTP is supposed to be started. It could be a VLAN interface above the primary interface.
    // /dev/ptp<X> - PTP dev node associated with the interface. Use "ethtool -T" command to identify the hardware clock instance i.e <X>
    

**IEEE 802.1AS Command**

Only available in standard (non safety) and and overlay platform configurations.
    
    
    //pre-requisite
    
    slay ptpd
    slay ptpd-avb
    
    // #Server Mode
    
    // In standard build, In prod_debug and prod_debug_extra
    SOCK=/sock<X> /proc/boot/ptpd-avb -WKLCQb <iface> 
    
    
    // #client Mode
    
    // Standard build, prod_debug and prod_debug_extra
    
    SOCK=/sock<X> /proc/boot/ptpd-avb -gKLCQb <iface> 
    
    
    // where:
    // <iface> : Interface name i.e eqos_0, mgbe0_0, mgbe1_0 on which PTP is supposed to be started. It could be a VLAN interface above the primary interface.
    

## PTP Bridging in Orin

Orin SOC has 5 Ethernet MAC controllers imbedded in it. Out of which only one can act as PTP secondary interface to synchronize the Tegra with external GM. There could be use-cases where the time info coming from external GM needs to be forwarded to the devices connected to other mac controllers. This requires a PTP time aware bridge like implementation in Tegra. Starting 6.0.2 a new feature is included in lower layer to sync multiple MAC PHCs from an external GM. As of now, support is there for only 2 macs at a time.

Primary (source) and secondary (sink) MAC selection is configurable through device tree. Check the device tree binding doc in `Documentation/devicetree/bindings/platform/tegra/tegra-nvethernet.txt`.

Note: By default, the MGBE2→ EQOS sync is enabled on boot-up starting 6.0.2 release. 

## PTP-TSC Sync

Orin SOC provides a hardware assisted mechanism to align/sync the TSC (clock/counter) with one of the Tegra SOC Ethernet MAC on second boundary using the PPS signal from the MAC. This feature is useful in synchronizing camera fsync signals (used for camera frame capture/timestamping) with PTP time in order to reduce the timestamp jitter between sensors timestamped with PTP domain. This feature is also useful for co-relation of TSC and PTP time-domain timestamps (needed for sensor fusion application) as it ensures a fixed offset (no run-time drift) between PTP and TSC counters over the time. The MAC selection for TSC reference is based on the PTP topology of the platform. It must be always the MAC acting as PTP client for External GTM directly or through switch.

PTP time source MAC selection is configurable through NvPPS/NvTime2 modules configuration section.

Note: By default, the MGBE2→ TSC locking enabled on boot starting 6.0.3 release.

## NvPPS

NvPPS module is Linux kernel module/driver which provide a single point of access to all the user applications to query:

  * Current synchronized PTP time from PHC 
  * Event co-related PTP and TSC time domain timestamps 



This module is enabled by default in the DRIVE OS kernel. 

**Operating Mode**

NvPPS module has two operating modes.

  1. **GPIO/PPS mode** : The NvPPS driver takes external 1PPS signal as input from devices like GPS etc.. on Tegra GPIO pin as an event and reads the timestamps from relevant time domains for correlation. 
  2. **Timer mode:** In this mode event for co-relation is based on periodic SW timer scheduled inside NvPPS module.



Timer mode is useful when there is no external device like GPS present for 1 PPS signal generation in system design or if their is no provision in platform (e.g P3663) to receive PPS signal through Tegra GPIO. 

**Userspace exposed interfaces**

The module exposes a dev node named as "/dev/nvpps0". The dev node supports following IOCTL calls:

**SN** |  **IOCTL** |  **Description**  
---|---|---  
1 | NVPPS_GETTIMESTAMP | Get current timestamps from kernel and PTP time from primary PHC interface  
2 | NVPPS_GETEVENT | Get correlated timestamps of different time-domains (TSC, PTP etc.) at the last event.   
3 | NVPPS_GETVERSION | Get NvPPS driver and API versions.  
4 | NVPPS_GETPARAMS |  Get the current NVPPS parameters. Like Operating mode(in GPIO or Timer mode) and TSC timestamp mode(in Nanosec or Counter mode).  For the mode values to be used, please refer ioctl header file mentioned below.  
5 | NVPPS_SETPARAMS |  Set the NvPPS parameters, described in the NVPPS_GETPARAMS section. This IOCTL can be used to update NvPPS parameters in runtime The parameter settings in NvPPS module are common to all client applications.  
  
The data structures required for supporting the IOCTLs are defined in [kernel/nvidia/include/uapi/linux/nvpps_ioctl.h](https://tegra-sw-opengrok.nvidia.com/source/xref/stage-main_automotive/kernel/nvidia/include/uapi/linux/nvpps_ioctl.h)

**Kernel space exposed interface**

In addition to User space interfaces, NvPPS module exports a kernel API for other kernel modules to get the current synchronized PTP time in kernel. Please refer to [kernel/nvidia/include/linux/nvpps.h](https://tegra-sw-opengrok.nvidia.com/source/xref/stage-main_automotive/kernel/nvidia/include/linux/nvpps.h).
    
    
    /*
     * Get PTP time
     * Clients may call this API whenever PTP time is needed.
     * If PTP time source is not registered, returns -EINVAL
     *
     * This API is available irrespective of nvpps dt availablity
     * When nvpps dt node is not present, interface name will
     * default to "eth0".
     */
    int nvpps_get_ptp_ts(void *ts)
    

**NvPPS module configuration**

Following static configuration is possible through NvPPS Device tree node. Please refer to device tree binding doc [kernel/nvidia/Documentation/devicetree/bindings/nvpps/nvpps.txt](https://tegra-sw-opengrok.nvidia.com/source/xref/stage-main_automotive/kernel/nvidia/Documentation/devicetree/bindings/nvpps/nvpps.txt)

  * Primary PTP interface( prop_name: "interface" ) - The MAC interface which connects to external GM. This interface is also used as source for PTP to TSC synchronization.
  * Secondary PTP interface (prop_name: "sec_interface" ) - The MAC which synchronizes with primary PTP MAC interface using PTP bridging.
  * 1PPS input pin (prop_name: "gpios" ) - Specifies GPIO pin detail on which 1PPS signal is available. Please refer platform specific NvPPS DT node more info. 



On this page

  * PTP Support in Orin

  * AVNU PTP for Development

  * AUTOSAR TSync PTP for Production

  * Other PTP Profiles in Orin

  * PTP Bridging in Orin

  * PTP-TSC Sync

  * NvPPS




Subject to Change | For test and development only. 

Published: 01/31/2023 9:02 a.m. |  PR-10720-6.0 

Follow DRIVE OS

[ Facebook ](http://www.facebook.com/NVIDIA "Follow GeForce on Facebook") [ Twitter ](http://twitter.com/nvidiadc "Follow GeForce on Twitter") [ LinkedIn ](https://www.linkedin.com/company/nvidia) [ YouTube ](http://www.youtube.com/user/nvidia)

NVIDIA

[ USA - United States  ](https://www.nvidia.com/object/country-selector.html)

  * [Privacy Policy](https://www.nvidia.com/en-us/about-nvidia/privacy-policy/)
  * [Manage My Privacy](https://www.nvidia.com/en-us/privacy-center/)
  * [Legal](https://www.nvidia.com/en-us/about-nvidia/legal-info/)
  * [Accessibility](https://www.nvidia.com/en-us/about-nvidia/accessibility/)
  * [Product Security](https://www.nvidia.com/en-us/product-security/)
  * [Contact](https://www.nvidia.com/en-us/contact/)



Copyright © 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
