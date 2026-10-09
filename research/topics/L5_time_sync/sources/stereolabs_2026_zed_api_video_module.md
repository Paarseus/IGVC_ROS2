[ ](https://www.stereolabs.com)

  * [Documentation](https://www.stereolabs.com/docs/)
  * [API Reference](https://www.stereolabs.com/docs/api/)
  * [Samples](https://www.stereolabs.com/docs/samples/)
  * [Support](https://support.stereolabs.com/hc/en-us/)
  * [Downloads](https://www.stereolabs.com/developers/release/)
  * [ ](https://github.com/stereolabs/zed-sdk "GitHub")

|  [](javascript:searchBox.CloseResultsWindow\(\)) |  [C++](https://www.stereolabs.com/docs/api/) |  [Python](https://www.stereolabs.com/docs/api/python/) |  [C#](https://www.stereolabs.com/docs/api/csharp/) |  [C](https://www.stereolabs.com/docs/api/c/)  
---|---|---|---  
  
Classes | Enumerations | Functions

Video Module

##  Classes  
  
---  
struct  | [RecordingStatus](structsl_1_1RecordingStatus.html)  
| Structure containing information about the status of the recording. [More...](structsl_1_1RecordingStatus.html#details)  
  
struct  | [StreamingParameters](structsl_1_1StreamingParameters.html)  
| Structure containing the options used to stream with the ZED SDK. [More...](structsl_1_1StreamingParameters.html#details)  
  
struct  | [RecordingParameters](structsl_1_1RecordingParameters.html)  
| Structure containing the options used to record. [More...](structsl_1_1RecordingParameters.html#details)  
  
struct  | [DeviceProperties](structsl_1_1DeviceProperties.html)  
| Structure containing information about the properties of a camera. [More...](structsl_1_1DeviceProperties.html#details)  
  
struct  | [StreamingProperties](structsl_1_1StreamingProperties.html)  
| Structure containing information about the properties of a streaming device. [More...](structsl_1_1StreamingProperties.html#details)  
  
class  | [InitParameters](structsl_1_1InitParameters.html)  
| Class containing the options used to initialize the [sl::Camera](classsl_1_1Camera.html "This class serves as the primary interface between the camera and the various features provided by th...") object. [More...](structsl_1_1InitParameters.html#details)  
  
struct  | [EncodedStreamPacket](structsl_1_1EncodedStreamPacket.html)  
| Single encoded video packet retrieved from a [Camera](classsl_1_1Camera.html "This class serves as the primary interface between the camera and the various features provided by th...") source. [More...](structsl_1_1EncodedStreamPacket.html#details)  
  
struct  | [EncodedStreamInfo](structsl_1_1EncodedStreamInfo.html)  
| Describes one encoded video source exposed by the [Camera](classsl_1_1Camera.html "This class serves as the primary interface between the camera and the various features provided by th..."). [More...](structsl_1_1EncodedStreamInfo.html#details)  
  
struct  | [RawBufferFd](structsl_1_1RawBufferFd.html)  
| Validity-tracked handle to a native DMA buffer (dmabuf) file descriptor from the camera capture pool. [More...](structsl_1_1RawBufferFd.html#details)  
  
struct  | [RawBuffer](structsl_1_1RawBuffer.html)  
| Zero-copy wrapper for native camera capture buffers. [More...](structsl_1_1RawBuffer.html#details)  
  
class  | [Camera](classsl_1_1Camera.html)  
| This class serves as the primary interface between the camera and the various features provided by the SDK. [More...](classsl_1_1Camera.html#details)  
  
class  | [InitParametersOne](structsl_1_1InitParametersOne.html)  
| Class containing the options used to initialize the [sl::CameraOne](classsl_1_1CameraOne.html "This class serves as the primary interface between the camera and the various features provided by th...") object. [More...](structsl_1_1InitParametersOne.html#details)  
  
class  | [InputType](classsl_1_1InputType.html)  
| Class defining the input type used in the ZED SDK. [More...](classsl_1_1InputType.html#details)  
  
  
##  Enumerations  
  
---  
enum class  | [SVO_ENCODING_PRESET](group__Video__group.html#ga955243aa3c970625400d959863dfff61)  
| Lists available encoding presets for SVO recording. [More...](group__Video__group.html#ga955243aa3c970625400d959863dfff61)  
  
enum class  | [SIDE](group__Video__group.html#gaed1a25f4b15c6110f7ac1dc385827049)  
| Lists possible sides on which to get data from. [More...](group__Video__group.html#gaed1a25f4b15c6110f7ac1dc385827049)  
  
enum  | [FLIP_MODE](group__Video__group.html#gafe09520beed3f2eba620708468d0eca6) : int   
| Lists possible flip modes of the camera. [More...](group__Video__group.html#gafe09520beed3f2eba620708468d0eca6)  
  
enum class  | [RESOLUTION](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828)  
| Lists available resolutions. [More...](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828)  
  
enum class  | [VIDEO_SETTINGS](group__Video__group.html#ga7bab4c6ca4fd971055eca1fdd9f4223d)  
| Lists available camera settings for the camera (contrast, hue, saturation, gain, ...). [More...](group__Video__group.html#ga7bab4c6ca4fd971055eca1fdd9f4223d)  
  
enum class  | [VIEW](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c)  
| Lists available views. [More...](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c)  
  
enum class  | [SVO_COMPRESSION_MODE](group__Video__group.html#gae52adc3898d4dd4cb5b770567fe8031c)  
| Lists available compression modes for SVO recording. [More...](group__Video__group.html#gae52adc3898d4dd4cb5b770567fe8031c)  
  
enum class  | [RAW_BUFFER_TYPE](group__Video__group.html#ga6eebe0925312f2ae35adf9b953341e08)  
| Lists the types of native raw buffers supported by RawBuffer. [More...](group__Video__group.html#ga6eebe0925312f2ae35adf9b953341e08)  
  
enum class  | [SENSOR_STATE](group__Video__group.html#ga114514c1e848aaad7a90b2720b34e32b)  
| Lists possible sensor states. [More...](group__Video__group.html#ga114514c1e848aaad7a90b2720b34e32b)  
  
enum class  | [MODEL](group__Video__group.html#gaa29851790d3e528f42f539334e6e5887)  
| Lists ZED camera model. [More...](group__Video__group.html#gaa29851790d3e528f42f539334e6e5887)  
  
enum class  | [INPUT_TYPE](group__Video__group.html#ga4a2a702e602c466869aa447ac2760c13)  
| Lists available input types in the ZED SDK. [More...](group__Video__group.html#ga4a2a702e602c466869aa447ac2760c13)  
  
enum class  | [CAMERA_STATE](group__Video__group.html#ga3eda01e75494f556f7a4ede1a7c2d55d)  
| Lists possible camera states. [More...](group__Video__group.html#ga3eda01e75494f556f7a4ede1a7c2d55d)  
  
enum class  | [STREAMING_CODEC](group__Video__group.html#ga0361144a89b83ced3ff322f026e54370)  
| Lists the different encoding types for image streaming. [More...](group__Video__group.html#ga0361144a89b83ced3ff322f026e54370)  
  
enum class  | [ENCODED_STREAM_SOURCE](group__Video__group.html#ga943e40b1f554a2e8bb9d7265db57c00f)  
| Identifies which of the camera's encoded video paths to read from. [More...](group__Video__group.html#ga943e40b1f554a2e8bb9d7265db57c00f)  
  
enum class  | [BUS_TYPE](group__Video__group.html#ga1508df264e0751558523bc8d35c1c3f4)  
| Lists available LIVE input type in the ZED SDK. [More...](group__Video__group.html#ga1508df264e0751558523bc8d35c1c3f4)  
  
enum class  | [TIME_REFERENCE](group__Video__group.html#ga9401e0c9b9fec46d2eb300ffd2fc72c9)  
| Lists possible time references for timestamps or data. [More...](group__Video__group.html#ga9401e0c9b9fec46d2eb300ffd2fc72c9)  
  
  
##  Functions  
  
---  
[sl::Resolution](structsl_1_1Resolution.html) | [getResolution](group__Video__group.html#ga6a283d1c725e46eb2c84aedabfddf855) ([RESOLUTION](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828) resolution)  
| Gets the corresponding [sl::Resolution](structsl_1_1Resolution.html "Structure containing the width and height of an image.") from an [sl::RESOLUTION](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828 "Lists available resolutions."). [More...](group__Video__group.html#ga6a283d1c725e46eb2c84aedabfddf855)  
  
unsigned int SL_CORE_EXPORT | [generateVirtualStereoSerialNumber](group__Video__group.html#gad5763001b1143a92937066e083aa53ab) (unsigned int serial_left, unsigned int serial_right)  
| Generate a unique identifier for virtual stereo based on the serial numbers of the two ZED Ones. [More...](group__Video__group.html#gad5763001b1143a92937066e083aa53ab)  
  
  
## Enumeration Type Documentation

## ◆ SVO_ENCODING_PRESET

| enum [SVO_ENCODING_PRESET](group__Video__group.html#ga955243aa3c970625400d959863dfff61)  
---  
strong  
  
Lists available encoding presets for SVO recording. 

The preset controls the speed/quality tradeoff of the hardware encoder. 

Note
    Only applicable when [SVO_COMPRESSION_MODE](group__Video__group.html#gae52adc3898d4dd4cb5b770567fe8031c) is H264 or H265 (lossy or lossless). 
Enumerator  
---  
DEFAULT | Encoder default.  
Maps to NVENC P4 / V4L2 default.   
ULTRAFAST | Fastest encoding, lowest quality.  
Maps to NVENC P1 / V4L2 ULTRAFAST.   
FAST | Fast encoding.  
Maps to NVENC P2 / V4L2 FAST.   
MEDIUM | Balanced speed/quality.  
Maps to NVENC P3 / V4L2 MEDIUM.   
SLOW | Slow encoding, higher quality.  
Maps to NVENC P5 / V4L2 SLOW.   
  
## ◆ SIDE

| enum [SIDE](group__Video__group.html#gaed1a25f4b15c6110f7ac1dc385827049)  
---  
strong  
  
Lists possible sides on which to get data from. 

Enumerator  
---  
LEFT | Left side only.   
RIGHT | Right side only.   
BOTH | Left and right side.   
  
## ◆ FLIP_MODE

enum [FLIP_MODE](group__Video__group.html#gafe09520beed3f2eba620708468d0eca6) : int  
---  
  
Lists possible flip modes of the camera. 

Enumerator  
---  
OFF | No flip applied. Default behavior.   
ON | Images and camera sensors' data are flipped useful when your camera is mounted upside down.   
AUTO | In LIVE mode, use the camera orientation (if an IMU is available) to set the flip mode.  
In SVO mode, read the state of this enum when recorded.   
  
## ◆ RESOLUTION

| enum [RESOLUTION](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828)  
---  
strong  
  
Lists available resolutions. 

Note
    The VGA resolution does not respect the 640*480 standard to better fit the camera sensor (672*376 is used). 

Warning
    All resolutions are not available for every camera. 
     You can find the available resolutions for each camera in [our documentation](https://www.stereolabs.com/docs/video/camera-controls#selecting-a-resolution). 
Enumerator  
---  
HD4K | 3856x2180 for imx678 mono   
QHDPLUS | 3800x1800   
HD2K | 2208*1242 (x2)   
Available FPS: 15   
Only supported with ZED-M / ZED2i   
HD1536 | 1920*1536 (x2)   
Available FPS: 30   
Only supported with ZED-X HDR lineup (One/Stereo)   
HD1080 | 1920*1080 (x2)   
Available FPS: 15, 30   
HD1200 | 1920*1200 (x2)   
Available FPS: 15, 30, 60   
Only supported with ZED-X lineup (One/Stereo) and ZED-XOne 4K   
HD720 | 1280*720 (x2)   
Available FPS: 15, 30, 60   
SVGA | 960*600 (x2)   
Available FPS: 15, 30, 60, 120   
Only supported with ZED-X lineup (One/Stereo)   
VGA | 672*376 (x2)   
Available FPS: 15, 30, 60, 100   
XVGA | 960x768 (x2)   
Available FPS: 30   
Only supported with ZED-X HDR lineup (One/Stereo)   
TXGA | 640x512 (x2)   
Available FPS: 30   
Only supported with ZED-X HDR lineup (One/Stereo)   
AUTO | Select the resolution compatible with the camera: 

  * ZED X/X Mini: HD1200
  * ZED X Pro/X Pro Mini: HD1536
  * other cameras: HD720

  
  
## ◆ VIDEO_SETTINGS

| enum [VIDEO_SETTINGS](group__Video__group.html#ga7bab4c6ca4fd971055eca1fdd9f4223d)  
---  
strong  
  
Lists available camera settings for the camera (contrast, hue, saturation, gain, ...). 

Warning
    All [VIDEO_SETTINGS](group__Video__group.html#ga7bab4c6ca4fd971055eca1fdd9f4223d) are not supported for all camera models. You can find the supported [VIDEO_SETTINGS](group__Video__group.html#ga7bab4c6ca4fd971055eca1fdd9f4223d) for each ZED camera in our [documentation](https://www.stereolabs.com/docs/video/camera-controls#adjusting-camera-settings).  
  
GAIN and EXPOSURE are linked in auto/default mode (see [sl::Camera::setCameraSettings()](classsl_1_1Camera.html#a993f92ae544c7628a9f03c4504b2f2cb)). 
Enumerator  
---  
BRIGHTNESS | Brightness control   
Affected value should be between 0 and 8. 

Note
    Not available for ZED X/X Mini cameras.   
CONTRAST | Contrast control   
Affected value should be between 0 and 8. 

Note
    Not available for ZED X/X Mini cameras.   
HUE | Hue control   
Affected value should be between 0 and 11. 

Note
    Not available for ZED X/X Mini cameras.   
SATURATION | Saturation control   
Affected value should be between 0 and 8.   
SHARPNESS | Digital sharpening control   
Affected value should be between 0 and 8.   
GAMMA | ISP gamma control   
Affected value should be between 1 and 9.   
GAIN | Gain control   
Affected value should be between 0 and 100 for manual control. 

Note
    If EXPOSURE is set to -1 (automatic mode), then GAIN will be automatic as well.   
EXPOSURE | Exposure control   
Affected value should be between 0 and 100 for manual control.  
The exposition is mapped linearly in a percentage of the following max values.  
Special case for `EXPOSURE = 0` that corresponds to 0.17072ms.  
The conversion to milliseconds depends on the framerate: 

  * 15fps & `EXPOSURE = 100` -> 19.97ms
  * 30fps & `EXPOSURE = 100` -> 19.97ms
  * 60fps & `EXPOSURE = 100` -> 10.84072ms
  * 100fps & `EXPOSURE = 100` -> 10.106624ms

  
AEC_AGC | Defines if the GAIN and EXPOSURE are in automatic mode or not.  
Setting GAIN or EXPOSURE values will automatically set this value to 0.   
AEC_AGC_ROI | Defines the region of interest for automatic exposure/gain computation.  
To be used with overloaded [setCameraSettings()](classsl_1_1Camera.html#aa266794188e5b07018a6252fb5f5d78e) / [getCameraSettings()](classsl_1_1Camera.html#a617d7cb73e9e9da0bc8c8ecfb5a92a88) methods.   
WHITEBALANCE_TEMPERATURE | Color temperature control   
Affected value should be between 2800 and 6500 with a step of 100. 

Note
    Setting a value will automatically set WHITEBALANCE_AUTO to 0.   
WHITEBALANCE_AUTO | Defines if the white balance is in automatic mode or not.   
LED_STATUS | Status of the front LED of the camera.  
Set to 0 to disable the light, 1 to enable the light.  
Default value is on. 

Note
    Requires camera firmware 1523 at least.   
EXPOSURE_TIME | Exposure time of the sensor in microseconds. 

Note
    Only available for GMSL2-based camera (ZED-X, ZED-X Mini, ZED-Xone,...) .
     Use this value to get the real exposure time in us.   
ANALOG_GAIN | Analog gain (sensor) control in mDB. 

Note
    Only available for GMSL2-based camera (ZED-X, ZED-X Mini, ZED-Xone,...) . 
     Use this value to get the real analog gain in mdB.   
DIGITAL_GAIN | Digital gain (ISP) as a factor. 

Note
    Only available for GMSL2-based camera (ZED-X, ZED-X Mini, ZED-Xone,...) .   
AUTO_EXPOSURE_TIME_RANGE | Range of exposure auto control in microseconds.  
Used with [setCameraSettings()](classsl_1_1Camera.html#a995b068cdf91292c4700e3cdfdc649ed).  
Min/max range between max range defined in DTS.  
By default: [28000 - <fps_time> or 19000] us. 

Note
    Only available for ZED X/X Mini cameras.   
AUTO_ANALOG_GAIN_RANGE | Range of sensor gain in automatic control.  
Used with [setCameraSettings()](classsl_1_1Camera.html#a995b068cdf91292c4700e3cdfdc649ed).  
Min/max range between max range defined in DTS.  
By default: [1000 - 16000] mdB. 

Note
    Only available for ZED X/X Mini cameras.   
AUTO_DIGITAL_GAIN_RANGE | Range of digital ISP gain in automatic control.  
Used with [setCameraSettings()](classsl_1_1Camera.html#a995b068cdf91292c4700e3cdfdc649ed).  
Min/max range between max range defined in DTS.  
By default: [1 - 256]. 

Note
    Only available for ZED X/X Mini cameras.   
EXPOSURE_COMPENSATION | Exposure-target compensation made after auto exposure.  
Reduces the overall illumination target by factor of F-stops.  
Affected value should be between 0 and 100 (mapped between [-2.0,2.0]).  
Default value is 50, i.e. no compensation applied. 

Note
    Only available for ZED-X / ZED-X Mini / ZED-XOne GS and ZED-XOne UHD cameras.   
DENOISING | Level of denoising applied on both left and right images.  
Affected value should be between 0 and 100.  
Default value is 50. 

Note
    Only available for GMSL2-based cameras   
SCENE_ILLUMINANCE | Level of illuminance of the scene calculated by the ISP.   
Can be used to determine the level of light in the scene and adjust settings accordingly. 

Note
    Read-only control.   
Available for ZED-X/X mini and ZED-XOne GS or 4K(UHD) cameras only.   
Value provided in [0.1x]Lux for ZED-X / ZED-X Mini / ZED-XOne GS and ZED-XOne UHD cameras.   
AE_ANTIBANDING | AE anti-banding mode.  
Affected value should be between 0 and 3.  
0: OFF, 1: AUTO, 2: 50Hz, 3: 60Hz.  
Default value is 0 (OFF). 

Note
    Only available for non-HDR GMSL cameras (e.g. ZED X, ZED X Mini, ZED X One GS, etc.).   
  
## ◆ VIEW

| enum [VIEW](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c)  
---  
strong  
  
Lists available views. 

Enumerator  
---  
LEFT | Left (for stereo or default for monocular camera) BGRA image. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT | Right BGRA image. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
LEFT_GRAY | Left gray image. Each pixel contains 1 unsigned char.  
Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT_GRAY | Right gray image. Each pixel contains 1 unsigned char.  
Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
LEFT_NV12_UNRECTIFIED | Left NV12 unrectified image.   
Type: [sl::MAT_TYPE::NV12](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT_NV12_UNRECTIFIED | Right NV12 unrectified image.   
Type: [sl::MAT_TYPE::NV12](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
LEFT_UNRECTIFIED | Left BGRA unrectified image. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT_UNRECTIFIED | Right BGRA unrectified image. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
LEFT_UNRECTIFIED_GRAY | Left gray unrectified image. Each pixel contains 1 unsigned char.  
Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT_UNRECTIFIED_GRAY | Right gray unrectified image. Each pixel contains 1 unsigned char.  
Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
SIDE_BY_SIDE | Left and right image (the image width is therefore doubled). Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
DEPTH | Color rendering of the depth. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)

Note
    Use [MEASURE::DEPTH](group__Depth__group.html#ga798a8eed10c573d759ef7e5a5bcd545d) with [Camera::retrieveMeasure()](classsl_1_1Camera.html#a724dbb224bae5ae72512b3a4da77a671) to get depth values.   
CONFIDENCE | Color rendering of the depth confidence. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)

Note
    Use [MEASURE::CONFIDENCE](group__Depth__group.html#ga798a8eed10c573d759ef7e5a5bcd545d) with [Camera::retrieveMeasure()](classsl_1_1Camera.html#a724dbb224bae5ae72512b3a4da77a671) to get confidence values.   
NORMALS | Color rendering of the normals. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)

Note
    Use [MEASURE::NORMALS](group__Depth__group.html#ga798a8eed10c573d759ef7e5a5bcd545d) with [Camera::retrieveMeasure()](classsl_1_1Camera.html#a724dbb224bae5ae72512b3a4da77a671) to get normal values.   
DEPTH_RIGHT | Color rendering of the right depth mapped on right sensor. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)

Note
    Use [MEASURE::DEPTH_RIGHT](group__Depth__group.html#ga798a8eed10c573d759ef7e5a5bcd545d) with [Camera::retrieveMeasure()](classsl_1_1Camera.html#a724dbb224bae5ae72512b3a4da77a671) to get depth right values.   
NORMALS_RIGHT | Color rendering of the normals mapped on right sensor. Each pixel contains 4 unsigned char (B, G, R, A).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)

Note
    Use [MEASURE::NORMALS_RIGHT](group__Depth__group.html#ga798a8eed10c573d759ef7e5a5bcd545d) with [Camera::retrieveMeasure()](classsl_1_1Camera.html#a724dbb224bae5ae72512b3a4da77a671) to get normal right values.   
LEFT_BGRA | Alias of [sl::VIEW::LEFT](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
LEFT_BGR | Left image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT_BGRA | Alias of [sl::VIEW::RIGHT](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT_BGR | Right image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
LEFT_UNRECTIFIED_BGRA | Alias of [sl::VIEW::LEFT_UNRECTIFIED](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
LEFT_UNRECTIFIED_BGR | Left unrectified image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT_UNRECTIFIED_BGRA | Alias of [sl::VIEW::RIGHT_UNRECTIFIED](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
RIGHT_UNRECTIFIED_BGR | Right unrectified image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
SIDE_BY_SIDE_BGRA | Alias of [sl::VIEW::SIDE_BY_SIDE](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
SIDE_BY_SIDE_BGR | Side by side image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
SIDE_BY_SIDE_GRAY | Side by side image in gray scale: Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
SIDE_BY_SIDE_UNRECTIFIED_BGRA | Alias of [sl::VIEW::SIDE_BY_SIDE_UNRECTIFIED](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
SIDE_BY_SIDE_UNRECTIFIED_BGR | Side by side unrectified image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
SIDE_BY_SIDE_UNRECTIFIED_GRAY | Side by side unrectified image in gray scale: Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
DEPTH_BGRA | Alias of [sl::VIEW::DEPTH](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
DEPTH_BGR | Depth image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
DEPTH_GRAY | Depth image in gray scale: Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
CONFIDENCE_BGRA | Alias of [sl::VIEW::CONFIDENCE](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
CONFIDENCE_BGR | Confidence image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
CONFIDENCE_GRAY | Confidence image in gray scale: Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
NORMALS_BGRA | Alias of [sl::VIEW::NORMALS](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
NORMALS_BGR | Normal image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
NORMALS_GRAY | Normal image in gray scale: Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
DEPTH_RIGHT_BGRA | Alias of [sl::VIEW::DEPTH_RIGHT](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
DEPTH_RIGHT_BGR | Depth right image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
DEPTH_RIGHT_GRAY | Depth right image in gray scale: Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
NORMALS_RIGHT_BGRA | Alias of [sl::VIEW::NORMALS_RIGHT](group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c).  
Type: [sl::MAT_TYPE::U8_C4](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
NORMALS_RIGHT_BGR | Normal right image in BGR pixel format: Type: [sl::MAT_TYPE::U8_C3](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
NORMALS_RIGHT_GRAY | Normal right image in gray scale: Type: [sl::MAT_TYPE::U8_C1](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713)  
LEFT_NV12 | Left NV12 rectified image (YUV 4:2:0 semi-planar).  
Type: [sl::MAT_TYPE::NV12](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713), size must be even   
RIGHT_NV12 | Right NV12 rectified image (YUV 4:2:0 semi-planar).  
Type: [sl::MAT_TYPE::NV12](group__Core__group.html#gab12e66b9515d6772cda59cc2f7e69713), size must be even   
  
## ◆ SVO_COMPRESSION_MODE

| enum [SVO_COMPRESSION_MODE](group__Video__group.html#gae52adc3898d4dd4cb5b770567fe8031c)  
---  
strong  
  
Lists available compression modes for SVO recording. 

Note
    LOSSLESS is an improvement of previous lossless compression (used in ZED Explorer), even if size may be bigger, compression time is much faster. 
Enumerator  
---  
LOSSLESS | PNG/ZSTD (lossless) CPU based compression.  
Average size: 42% of RAW   
H264 | H264 (AVCHD) GPU based compression.  
Average size: 1% of RAW 

Note
    Requires a NVIDIA GPU.   
H265 | H265 (HEVC) GPU based compression.  
Average size: 1% of RAW 

Note
    Requires a NVIDIA GPU.   
H264_LOSSLESS | H264 Lossless GPU/Hardware based compression.  
Average size: 25% of RAW   
Provides a SSIM/PSNR result (vs RAW) >= 99.9%. 

Note
    Requires a NVIDIA GPU.   
H265_LOSSLESS | H265 Lossless GPU/Hardware based compression.  
Average size: 25% of RAW   
Provides a SSIM/PSNR result (vs RAW) >= 99.9%. 

Note
    Requires a NVIDIA GPU.   
  
## ◆ RAW_BUFFER_TYPE

| enum [RAW_BUFFER_TYPE](group__Video__group.html#ga6eebe0925312f2ae35adf9b953341e08)  
---  
strong  
  
Lists the types of native raw buffers supported by [RawBuffer](structsl_1_1RawBuffer.html "Zero-copy wrapper for native camera capture buffers."). 

Warning
    This is an advanced low-level API. Enabled by defining SL_ENABLE_ADVANCED_CAPTURE_API before including this header. Improper use can crash the Argus stack or destabilize the system. 
Enumerator  
---  
UNKNOWN | Unknown or uninitialized buffer type.   
NVBUFSURFACE | NvBufSurface (Argus native buffer)   
  
## ◆ SENSOR_STATE

| enum [SENSOR_STATE](group__Video__group.html#ga114514c1e848aaad7a90b2720b34e32b)  
---  
strong  
  
Lists possible sensor states. 

Enumerator  
---  
AVAILABLE | The sensor can be opened by the ZED SDK.   
NOT_AVAILABLE | The sensor is already opened and unavailable.   
  
## ◆ MODEL

| enum [MODEL](group__Video__group.html#gaa29851790d3e528f42f539334e6e5887)  
---  
strong  
  
Lists ZED camera model. 

Enumerator  
---  
ZED | ZED camera model   
ZED_M | ZED Mini (ZED M) camera model   
ZED2 | ZED 2 camera model   
ZED2i | ZED 2i camera model   
ZED_X | ZED X camera model (120mm baseline) with dual global shutter AR0234 sensor   
ZED_XM | ZED X Mini (50mm baseline) camera model with dual global shutter AR0234 sensor   
ZED_X_HDR | ZED X HDR camera model (120mm baseline) with dual HDR rolling shutter ISX031 sensor   
ZED_X_HDR_MINI | ZED X HDR mini camera model (50mm baseline) with dual HDR rolling shutter ISX031 sensor   
ZED_X_HDR_MAX | ZED X HDR Max camera model (170mm baseline) with dual HDR rolling shutter ISX031 sensor   
ZED_X_NANO | ZED X Nano (18mm baseline) camera model with dual global shutter AR0234 sensor   
VIRTUAL_ZED_X | Virtual ZED-X generated from 2 ZED-XOne (Using ZED Media Server)   
ZED_XONE_GS | ZED X One with global shutter AR0234 sensor   
ZED_XONE_UHD | ZED X One with 4K rolling shutter IMX678 sensor   
ZED_XONE_HDR | ZED X One with HDR rolling shutter ISX031 sensor   
ZED_XONE_CORE | ZED X One Core with global shutter AR0234 sensor, direct MIPI connection   
  
## ◆ INPUT_TYPE

| enum [INPUT_TYPE](group__Video__group.html#ga4a2a702e602c466869aa447ac2760c13)  
---  
strong  
  
Lists available input types in the ZED SDK. 

Enumerator  
---  
USB | USB input mode   
SVO | SVO file input mode   
STREAM | STREAM input mode (requires to use [enableStreaming()](classsl_1_1Camera.html#a1e597642b94bd41fe8eb8de95abc44fe) / [disableStreaming()](classsl_1_1Camera.html#aa0f6002788540a28cd8793d107c124d8)" on the "sender" side)   
GMSL | GMSL input mode (only on NVIDIA Jetson)   
MIPI | MIPI input mode, for a camera connected directly to a MIPI capture card (only on NVIDIA Jetson)   
HOLOSCAN | Holoscan Camera-over-Ethernet input mode, through a Holoscan sensor bridge (only on NVIDIA Jetson)   
  
## ◆ CAMERA_STATE

| enum [CAMERA_STATE](group__Video__group.html#ga3eda01e75494f556f7a4ede1a7c2d55d)  
---  
strong  
  
Lists possible camera states. 

Enumerator  
---  
AVAILABLE | The camera can be opened by the ZED SDK.   
NOT_AVAILABLE | The camera is already opened and unavailable.   
  
## ◆ STREAMING_CODEC

| enum [STREAMING_CODEC](group__Video__group.html#ga0361144a89b83ced3ff322f026e54370)  
---  
strong  
  
Lists the different encoding types for image streaming. 

Enumerator  
---  
H264 | AVCHD/H264 encoding   
H265 | HEVC/H265 encoding   
  
## ◆ ENCODED_STREAM_SOURCE

| enum [ENCODED_STREAM_SOURCE](group__Video__group.html#ga943e40b1f554a2e8bb9d7265db57c00f)  
---  
strong  
  
Identifies which of the camera's encoded video paths to read from. 

A single [Camera](classsl_1_1Camera.html) can produce up to three concurrent encoded H264/H265 bitstreams: the incoming stream when the camera is opened from a network sender ([ENCODED_STREAM_SOURCE::RECEIVING](namespacesl.html#ga943e40b1f554a2e8bb9d7265db57c00fa0c96084902e8f21bf3bf678fdce4f2a0)), the outgoing stream when [Camera::enableStreaming()](classsl_1_1Camera.html#a1e597642b94bd41fe8eb8de95abc44fe) is active ([ENCODED_STREAM_SOURCE::SENDING](namespacesl.html#ga943e40b1f554a2e8bb9d7265db57c00fac9c9fa46a3628497a4e7f74444ae4568)), and the SVO encoder output when [Camera::enableRecording()](classsl_1_1Camera.html#ae4d00872b1565d756de2645ee5a6de7c) is active with a video compression mode ([ENCODED_STREAM_SOURCE::RECORDING](namespacesl.html#ga943e40b1f554a2e8bb9d7265db57c00fa6106cd30b22a82df6165f121308b5366)). Each source has its own codec, bitrate and key-frame schedule. 

Enumerator  
---  
RECEIVING | Incoming stream packets ([Camera](classsl_1_1Camera.html "This class serves as the primary interface between the camera and the various features provided by th...") opened with [INPUT_TYPE::STREAM](namespacesl.html#ga4a2a702e602c466869aa447ac2760c13a2f05998d2a71cdc19b7109549bbe2646)).   
SENDING | Outgoing stream packets (enableStreaming() is active).   
RECORDING | SVO encoder output (enableRecording() with H264 / H265 compression mode).   
LAST | Sentinel.   
  
## ◆ BUS_TYPE

| enum [BUS_TYPE](group__Video__group.html#ga1508df264e0751558523bc8d35c1c3f4)  
---  
strong  
  
Lists available LIVE input type in the ZED SDK. 

Enumerator  
---  
USB | USB input mode   
GMSL | GMSL input mode 

Note
    Only on NVIDIA Jetson.   
AUTO | Automatically select the input type.  
Trying first for available USB cameras, then the cameras connected to the host.   
MIPI | MIPI input mode, for a camera connected directly to a MIPI capture card. 

Note
    Only on NVIDIA Jetson.   
HOLOSCAN | Holoscan Camera-over-Ethernet input mode, for a camera behind a Holoscan sensor bridge. 

Note
    Only on NVIDIA Jetson.   
  
## ◆ TIME_REFERENCE

| enum [TIME_REFERENCE](group__Video__group.html#ga9401e0c9b9fec46d2eb300ffd2fc72c9)  
---  
strong  
  
Lists possible time references for timestamps or data. 

Enumerator  
---  
IMAGE | The requested timestamp or data will be at the time of the frame extraction.   
CURRENT | The requested timestamp or data will be at the time of the function call.   
IMAGE_CENTER_OF_EXPOSURE | The middle of the frame's integration window: the instant the "average photon" of the frame reached the sensor. [IMAGE](namespacesl.html#ga9401e0c9b9fec46d2eb300ffd2fc72c9a23a12f67f614b5518c7f1c2465bf95e3) is anchored at the start of the sensor readout, which is the moment the frame's integration ends*; this reference therefore sits half an exposure earlier** than [IMAGE](namespacesl.html#ga9401e0c9b9fec46d2eb300ffd2fc72c9a23a12f67f614b5518c7f1c2465bf95e3). Use it to align frames with other sensors (LiDAR, IMU, robot joints) that are timestamped at the instant they measure.

Note
    Only meaningful for [Camera::getTimestamp()](classsl_1_1Camera.html#a3cd31c58aba33727f35aeae28244c82d) / [CameraOne::getTimestamp()](classsl_1_1CameraOne.html#a828659c6fb27cea26acd6b93ca7e8f84). It is rejected by [Camera::getSensorsData()](classsl_1_1Camera.html#af67fb040d2dd3bd65baada2cb743abe9) / [CameraOne::getSensorsData()](classsl_1_1CameraOne.html#ad760a36b68e1c46f35fca821fdafbbbd). 
     Requires a per-frame exposure, so it is available on ZED X, ZED X Mini, ZED X One GS and ZED X One 4K. Returns 0 on every other input: USB cameras, the HDR camera family (ZED X HDR / HDR Mini / HDR Max, ZED X One HDR), and any SVO or network stream carrying no per-frame sensor metadata. Always check for 0 before using the value. 
     Exact on global-shutter sensors (ZED X, ZED X Mini, ZED X One GS), where the whole array integrates at once. On the rolling-shutter ZED X One 4K it is the center of exposure of the first* row; later rows integrate progressively later, by up to the sensor readout time.   
  
## Function Documentation

## ◆ getResolution()

[sl::Resolution](structsl_1_1Resolution.html) sl::getResolution  | ( | [RESOLUTION](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828) | _resolution_| ) |   
---|---|---|---|---|---  
  
Gets the corresponding [sl::Resolution](structsl_1_1Resolution.html "Structure containing the width and height of an image.") from an [sl::RESOLUTION](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828 "Lists available resolutions."). 

Parameters
     resolution| : The wanted [sl::RESOLUTION](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828 "Lists available resolutions.").   
---|---  
  
Returns
    The [sl::Resolution](structsl_1_1Resolution.html "Structure containing the width and height of an image.") corresponding to [sl::RESOLUTION](group__Video__group.html#gabd0374c748530a64a72872c43b2cc828 "Lists available resolutions.") given as argument. 

## ◆ generateVirtualStereoSerialNumber()

unsigned int SL_CORE_EXPORT sl::generateVirtualStereoSerialNumber  | ( | unsigned int | _serial_left_ ,   
---|---|---|---  
|  | unsigned int | _serial_right_  
| ) | |   
  
Generate a unique identifier for virtual stereo based on the serial numbers of the two ZED Ones. 

Parameters
     serial_left| : Serial number of the left camera.   
---|---  
serial_right| : Serial number of the right camera.   
  
Returns
    A unique hash for the given pair of serial numbers, or 0 if an error occurred (e.g: same serial number). 
