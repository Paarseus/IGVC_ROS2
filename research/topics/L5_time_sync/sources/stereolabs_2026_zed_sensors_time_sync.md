# Sensors Time Synchronization

Ask a question|[Open in Claude](https://claude.ai/new?q=Read%20https%3A%2F%2Fdocs.stereolabs.com%2Fdocs%2Fdevelopment%2Fzed-sdk%2Fmodules%2Fsensors%2Ftime-synchronization.md%20so%20I%20can%20ask%20questions%20about%20it. "Ask questions about this page")|[Open in ChatGPT](https://chat.openai.com/?hint=search&q=Read%20https%3A%2F%2Fdocs.stereolabs.com%2Fdocs%2Fdevelopment%2Fzed-sdk%2Fmodules%2Fsensors%2Ftime-synchronization.md%20so%20I%20can%20ask%20questions%20about%20it. "Open in ChatGPT")|More actions

Accurate time synchronization is required for reliable integration of data from multiple sensor sources. The ZED cameras and sensors share a common and low-drift reference clock. Incoming packets are timestamped upon reception by the host machine, in Epoch time with nanosecond resolution. These timestamps can be used to:

  * Integrate multi-sensor data from a single camera such as IMU and stereo images.
  * Fuse data with external sensor sources such as GPS or LiDAR.



## Getting Time Synced Sensors Data

Sensors data is accessible by using the function `getSensorsData(sensors_data, TIME_REFERENCE)` as explained in the [Using the API](/docs/development/zed-sdk/modules/sensors/using-the-api) section.

The structure `sensors_data` is where data from the different sensors is stored. The `TIME_REFERENCE` variable is explained below.

### Time Sync Reference

While the camera sensors are synchronized and share the same time reference, they have different operating frequencies (data rates). For example, the IMU on board the ZED 2 depth camera runs at 400 Hz, while images can be captured at 100 Hz maximum.

To simplify multi-sensor data acquisition, we provide a `TIME_REFERENCE` variable in the `getSensorsData` function that lets you define if you want to retrieve data synchronized with image frames or not:

  * `TIME_REFERENCE::CURRENT` lets you retrieve the most recent sensor data available.
  * `TIME_REFERENCE::IMAGE` lets you retrieve the closest sensor data to the last image frame.



The Camera-IMU synchronization schema below illustrates what values these two settings return.

In the example above, we call the `grab()` function and `retrieveImage()` to retrieve the most recent image frame available from the camera. Then we call one of the two functions below:

  * `getSensorsData(sensors_data, TIME_REFERENCE::IMAGE)`: It returns IMU sample “1” which was captured near image frame “I0”. We try to call the same function again before a new frame is available, and it returns the same IMU sample “1”.

  * `getSensorsData(sensors_data, TIME_REFERENCE::CURRENT)`: It returns the most recent IMU sample. We call the function twice and IMU samples “2” and “4” are returned, independently from grab call and image frame time.




## Using the API

To learn how to retrieve current or image-synchronized sensor data, read the [Using Sensors API](/docs/development/zed-sdk/modules/sensors/using-the-api) section.

Was this page helpful?

YesNo

[PreviousTemperature Sensors Overview](/docs/development/zed-sdk/modules/sensors/temperature-sensors)[NextUsing the Sensors API](/docs/development/zed-sdk/modules/sensors/using-the-api)[Built with](https://buildwithfern.com/?utm_campaign=buildWith&utm_medium=docs&utm_source=docs.stereolabs.com)

* * *

[](https://www.linkedin.com/company/stereolabs)[](https://x.com/stereolabs3d)[](https://www.instagram.com/stereolabs3d)[](https://www.facebook.com/stereolabs)[](https://www.youtube.com/@Stereolabs3d)[](https://github.com/stereolabs)[](https://www.stereolabs.com/)
