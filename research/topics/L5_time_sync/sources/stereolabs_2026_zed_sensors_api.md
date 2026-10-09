# Using the Sensors API

Ask a question|[Open in Claude](https://claude.ai/new?q=Read%20https%3A%2F%2Fdocs.stereolabs.com%2Fdocs%2Fdevelopment%2Fzed-sdk%2Fmodules%2Fsensors%2Fusing-the-api.md%20so%20I%20can%20ask%20questions%20about%20it. "Ask questions about this page")|[Open in ChatGPT](https://chat.openai.com/?hint=search&q=Read%20https%3A%2F%2Fdocs.stereolabs.com%2Fdocs%2Fdevelopment%2Fzed-sdk%2Fmodules%2Fsensors%2Fusing-the-api.md%20so%20I%20can%20ask%20questions%20about%20it. "Open in ChatGPT")|More actions

The Sensors API lets you access sensors available on ZED depth cameras and perform a wide variety of sensor-related tasks explained below.

## Getting Sensors Data

The data from the different sensors are accessible with the `SensorsData` class. The class contains raw and calibrated values of each sensor.

To retrieve sensor values synchronized with the image frames, you will need to:

  * Open the camera and grab the current image.
  * Get sensor data corresponding to this image using `TIME_REFERENCE::IMAGE`.
  * Retrieve data from the different sensors.



C++PythonC#
    
    
    // Create and open the camera
    
    Camera zed;
    
    zed.open();
    
    SensorsData sensors_data;
    
    // Grab new frames and retrieve sensors data
    
    while (zed.grab() == ERROR_CODE::SUCCESS) {
    
        zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE); // Get frame synchronized sensor data
    
        // Extract multi-sensor data
    
        SensorsData::IMUData imu_data = sensors_data.imu;
    
        SensorsData::BarometerData barometer_data = sensors_data.barometer;
    
        SensorsData::MagnetometerData magnetometer_data = sensors_data.magnetometer;
    
        // Retrieve linear acceleration and angular velocity
    
        float3 linear_acceleration = imu_data.linear_acceleration;
    
        float3 angular_velocity = imu_data.angular_velocity;
    
        // Retrieve pressure and relative altitude
    
        float pressure = barometer_data.pressure;
    
        float relative_altitude = barometer_data.relative_altitude;
    
        // Retrieve magnetic field
    
        float3 magnetic_field = magnetometer_data.magnetic_field_uncalibrated;
    
    }

### Time Reference

The function `getSensorsData` can be called with a `TIME_REFERENCE`. For example, you can use `TIME_REFERENCE::CURRENT` to get the sensor data corresponding to the timestamp of the function call, or `TIME_REFERENCE::IMAGE` to get the data synchronized with the current camera image.

For more information on sensor time reference, read the [Sensors Time Synchronization](/docs/development/zed-sdk/modules/sensors/time-synchronization) section.

## Retrieve New Sensor Data

To retrieve updated sensor data, `getSensorsData` must be called at a frequency at least equal to the sensor data rate, using `TIME_REFERENCE::CURRENT`. Since sensors have different frequencies and their data is stored in the same `sensors_data` structure, some sensor data might not be updated between two calls to `getSensorsData`.

To know if a given sensor data has been updated, we compare their timestamps which are used as unique identifiers. If they are the same, then the sensor data has not been updated.

C++PythonC#
    
    
    Timestamp last_imu_ts = 0;
    
    while (!exit_app) {
    
        // Call this loop faster than the IMU rate, independently of grab()
    
        zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
    
        // Check if a new IMU sample is available
    
        if (sensors_data.imu.timestamp > last_imu_ts) {
    
            cout << "Linear Acceleration: " << sensors_data.imu.linear_acceleration << endl;
    
            cout << "Angular Velocity: " << sensors_data.imu.angular_velocity << endl;
    
            last_imu_ts = sensors_data.imu.timestamp;
    
        }
    
    }

## Accessing Raw Sensor Data

The Sensors API lets you read raw data from the depth camera’s built-in motion and position sensors. To retrieve raw data, use the `uncalibrated` values present in `sensors_data` structure.

## Identifying Sensors Capabilities

Sensors factory parameters can be accessed through the API. They cannot be changed as they are fixed in the microcontroller firmware of the cameras. You can access the following parameters:

  * Sensor Type
  * Sampling Rate
  * Range
  * Resolution
  * Noise Density
  * Random Walk (if applicable)
  * Sensor Units



C++PythonC#
    
    
    // Display camera information (model, serial number, firmware version)
    
    auto info = zed.getCameraInformation();
    
    cout << "Camera Model: " << info.camera_model << endl;
    
    cout << "Serial Number: " << info.serial_number << endl;
    
    cout << "Camera Firmware: " << info.camera_configuration.firmware_version << endl;
    
    cout << "Sensors Firmware: " << info.sensors_configuration.firmware_version << endl;
    
    // Display accelerometer sensor configuration
    
    SensorParameters& sensor_parameters = info.sensors_configuration.accelerometer_parameters;
    
    cout << "Sensor Type: " << sensor_parameters.type << endl;
    
    cout << "Sampling Rate: " << sensor_parameters.sampling_rate << endl;
    
    cout << "Range: "       << sensor_parameters.range << endl;
    
    cout << "Resolution: "  << sensor_parameters.resolution << endl;
    
    if (isfinite(sensor_parameters.noise_density)) cout << "Noise Density: " << sensor_parameters.noise_density << endl;
    
    if (isfinite(sensor_parameters.random_walk)) cout << "Random Walk: " << sensor_parameters.random_walk << endl;

## Code Example

For a code example, check out the [Getting Sensor Data](/docs/tutorials/sensor-data) tutorial.

Was this page helpful?

YesNo

[PreviousSensors Time Synchronization](/docs/development/zed-sdk/modules/sensors/time-synchronization)[NextDepth Sensing Overview](/docs/development/zed-sdk/modules/depth-sensing)[Built with](https://buildwithfern.com/?utm_campaign=buildWith&utm_medium=docs&utm_source=docs.stereolabs.com)

* * *

[](https://www.linkedin.com/company/stereolabs)[](https://x.com/stereolabs3d)[](https://www.instagram.com/stereolabs3d)[](https://www.facebook.com/stereolabs)[](https://www.youtube.com/@Stereolabs3d)[](https://github.com/stereolabs)[](https://www.stereolabs.com/)
