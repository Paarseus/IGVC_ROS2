

<!-- Source: https://docs.revrobotics.com/brushless/spark-max/parameters (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/brushless/spark-max/parameters.md).

# SPARK MAX Configuration Parameters

Below is a list of all the configurable parameters within the SPARK MAX. Parameters can be set through the CAN or USB interfaces. The parameters are saved in a different region of memory from the device firmware and persist through a firmware update.&#x20;

<table data-header-hidden><thead><tr><th width="272">Name</th><th width="64" align="center">ID</th><th width="88" align="center">Type</th><th width="93" align="center">Default</th><th width="269">Description</th></tr></thead><tbody><tr><td>Name</td><td align="center">ID</td><td align="center">Type</td><td align="center">Default</td><td>Description</td></tr><tr><td>kCanID</td><td align="center">0</td><td align="center">uint</td><td align="center">0</td><td>CAN ID<br><br>This parameter persists through a normal firmware update.</td></tr><tr><td>kInputMode</td><td align="center">1</td><td align="center">Input Mode</td><td align="center">0</td><td>Input mode, this parameter is read only and the input mode is detected by the firmware automatically.<br>0 - PWM<br>1 - CAN<br>2 - USB</td></tr><tr><td>kMotorType</td><td align="center">2</td><td align="center">Motor Type</td><td align="center">BRUSHLESS</td><td>Motor type:<br><br>0 - Brushed<br>1 - Brushless<br><br>This parameter persists through a normal firmware update.</td></tr><tr><td>Reserved</td><td align="center">3</td><td align="center">-</td><td align="center"></td><td>Reserved</td></tr><tr><td>kSensorType</td><td align="center">4</td><td align="center">Sensor Type</td><td align="center">HALL_EFFECT</td><td>Sensor type:<br>0 - No Sensor<br>1 - Hall Sensor<br>2 - Encoder<br>This parameter persists through a normal firmware update.</td></tr><tr><td>kCtrlType</td><td align="center">5</td><td align="center">Ctrl Type</td><td align="center">CTRL_DUTY_CYCLE</td><td>Control Type, this is a read only parameter of the currently active control type. The control type is changed by calling the correct API.<br>0 - Duty Cycle<br>1 - Velocity<br>2 - Voltage<br>3 - Position</td></tr><tr><td>kIdleMode</td><td align="center">6</td><td align="center">Idle Mode</td><td align="center">IDLE_COAST</td><td>State of the half bridge when the motor controller commands zero output or is disabled.<br>0 - Coast<br>1 - Brake<br>This parameter persists through a normal firmware update.</td></tr><tr><td>kInputDeadband</td><td align="center">7</td><td align="center">float32</td><td align="center">%0.05</td><td>Percent of the input which results in zero output for PWM mode.<br>This parameter persists through a normal firmware update.</td></tr><tr><td>Reserved</td><td align="center">8</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">9</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>kPolePairs</td><td align="center">10</td><td align="center">uint</td><td align="center">7</td><td>Number of pole pairs for the brushless motor. This is the number of poles/2 and can be determined by either counting the number of magnets or counting the number of windings and dividing by 3. This is an important term for speed regulation to properly calculate the speed.</td></tr><tr><td>kCurrentChop</td><td align="center">11</td><td align="center">float32</td><td align="center">115/Amps</td><td>If the half bridge detects this current limit, it will disable the motor driver for a fixed amount of time set by kCurrentChopCycles. This is a low sophistication 'current control'. Set to 0 to disable. The max value is 125.</td></tr><tr><td>kCurrentChopCycles</td><td align="center">12</td><td align="center">uint</td><td align="center">0</td><td>Number of PWM Cycles for the h-bridge to be off in the case that the current limit is set. Min = 1, multiples of PWM period (50μs). During this time the current will be recirculating through the low side MOSFETs, so instead of 'freewheeling' the diodes, the bridge will be in brake mode during this time.</td></tr><tr><td>kP_0</td><td align="center">13</td><td align="center">float32</td><td align="center">0</td><td>Proportional gain constant for gain slot 0.</td></tr><tr><td>kI_0</td><td align="center">14</td><td align="center">float32</td><td align="center">0</td><td>Integral gain constant for gain slot 0.</td></tr><tr><td>kD_0</td><td align="center">15</td><td align="center">float32</td><td align="center">0</td><td>Derivative gain constant for gain slot 0.</td></tr><tr><td>kF_0</td><td align="center">16</td><td align="center">float32</td><td align="center">0</td><td>Feed Forward gain constant for gain slot 0.</td></tr><tr><td>kIZone_0</td><td align="center">17</td><td align="center">float32</td><td align="center">0</td><td>Integrator zone constant for gain slot 0. The PIDF loop integrator will only accumulate while the setpoint is within IZone of the target.</td></tr><tr><td>kDFilter_0</td><td align="center">18</td><td align="center">float32</td><td align="center">0</td><td>PIDF derivative filter constant for gain slot 0.</td></tr><tr><td>kOutputMin_0</td><td align="center">19</td><td align="center">float32</td><td align="center">-1</td><td>Max output constant for gain slot 0. This is the max output of the controller.</td></tr><tr><td>kOutputMax_0</td><td align="center">20</td><td align="center">float32</td><td align="center">1</td><td>Min output constant for gain slot 0. This is the min output of the controller.</td></tr><tr><td>kP_1</td><td align="center">21</td><td align="center">float32</td><td align="center">0</td><td>Proportional gain constant for gain slot 1.</td></tr><tr><td>kI_1</td><td align="center">22</td><td align="center">float32</td><td align="center">0</td><td>Integral gain constant for gain slot 1.</td></tr><tr><td>kD_1</td><td align="center">23</td><td align="center">float32</td><td align="center">0</td><td>Derivative gain constant for gain slot 1.</td></tr><tr><td>kF_1</td><td align="center">24</td><td align="center">float32</td><td align="center">0</td><td>Feed Forward gain constant for gain slot 1.</td></tr><tr><td>kIZone_1</td><td align="center">25</td><td align="center">float32</td><td align="center">0</td><td>Integrator zone constant for gain slot 1. The PIDF loop integrator will only accumulate while the setpoint is within IZone of the target.</td></tr><tr><td>kDFilter_1</td><td align="center">26</td><td align="center">float32</td><td align="center">0</td><td>PIDF derivative filter constant for gain slot 1.</td></tr><tr><td>kOutputMin_1</td><td align="center">27</td><td align="center">float32</td><td align="center">-1</td><td>Max output constant for gain slot 1. This is the max output of the controller.</td></tr><tr><td>kOutputMax_1</td><td align="center">28</td><td align="center">float32</td><td align="center">1</td><td>Min output constant for gain slot 1. This is the min output of the controller.</td></tr><tr><td>kP_2</td><td align="center">29</td><td align="center">float32</td><td align="center">0</td><td>Proportional gain constant for gain slot 2.</td></tr><tr><td>kI_2</td><td align="center">30</td><td align="center">float32</td><td align="center">0</td><td>Integral gain constant for gain slot 2.</td></tr><tr><td>kD_2</td><td align="center">31</td><td align="center">float32</td><td align="center">0</td><td>Derivative gain constant for gain slot 2.</td></tr><tr><td>kF_2</td><td align="center">32</td><td align="center">float32</td><td align="center">0</td><td>Feed Forward gain constant for gain slot 2.</td></tr><tr><td>kIZone_2</td><td align="center">33</td><td align="center">float32</td><td align="center">0</td><td>Integrator zone constant for gain slot 2. The PIDF loop integrator will only accumulate while the setpoint is within IZone of the target.</td></tr><tr><td>kDFilter_2</td><td align="center">34</td><td align="center">float32</td><td align="center">0</td><td>PIDF derivative filter constant for gain slot 2.</td></tr><tr><td>kOutputMin_2</td><td align="center">35</td><td align="center">float32</td><td align="center">-1</td><td>Max output constant for gain slot 2. This is the max output of the controller.</td></tr><tr><td>kOutputMax_2</td><td align="center">36</td><td align="center">float32</td><td align="center">1</td><td>Min output constant for gain slot 2. This is the min output of the controller.</td></tr><tr><td>kP_3</td><td align="center">37</td><td align="center">float32</td><td align="center">0</td><td>Proportional gain constant for gain slot 3.</td></tr><tr><td>kI_3</td><td align="center">38</td><td align="center">float32</td><td align="center">0</td><td>Integral gain constant for gain slot 3.</td></tr><tr><td>kD_3</td><td align="center">39</td><td align="center">float32</td><td align="center">0</td><td>Derivative gain constant for gain slot 3.</td></tr><tr><td>kF_3</td><td align="center">40</td><td align="center">float32</td><td align="center">0</td><td>Feed Forward gain constant for gain slot 3.</td></tr><tr><td>kIZone_3</td><td align="center">41</td><td align="center">float32</td><td align="center">0</td><td>Integrator zone constant for gain slot 3. The PIDF loop integrator will only accumulate while the setpoint is within IZone of the target.</td></tr><tr><td>kDFilter_3</td><td align="center">42</td><td align="center">float32</td><td align="center">0</td><td>PIDF derivative filter constant for gain slot 3.</td></tr><tr><td>kOutputMin_3</td><td align="center">43</td><td align="center">float32</td><td align="center">-1</td><td>Max output constant for gain slot 3. This is the max output of the controller.</td></tr><tr><td>kOutputMax_3</td><td align="center">44</td><td align="center">float32</td><td align="center">1</td><td>Min output constant for gain slot 3. This is the min output of the controller.</td></tr><tr><td>Reserved</td><td align="center">45</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">46</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">47</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">48</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">49</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>kLimitSwitchFwdPolarity</td><td align="center">50</td><td align="center">bool</td><td align="center">0</td><td>Forward Limit Switch polarity.<br>0 - Normally Open<br>1 - Normally Closed</td></tr><tr><td>kLimitSwitchRevPolarity</td><td align="center">51</td><td align="center">bool</td><td align="center">0</td><td>Reverse Limit Switch polarity.<br>0 - Normally Open<br>1 - Normally Closed</td></tr><tr><td>kHardLimitFwdEn</td><td align="center">52</td><td align="center">bool</td><td align="center">1</td><td>Limit switch enable, enabled by default</td></tr><tr><td>kHardLimitRevEn</td><td align="center">53</td><td align="center">bool</td><td align="center">1</td><td>Limit switch enable, enabled by default</td></tr><tr><td>Reserved</td><td align="center">54</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">55</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>kRampRate</td><td align="center">56</td><td align="center">float32</td><td align="center">V/s 0</td><td>Voltage ramp rate active for all control modes in % output per second, a value of 0 disables this feature. All APIs take the reciprocal to make the unit 'time from 0 to full'.</td></tr><tr><td>kFollowerID</td><td align="center">57</td><td align="center">uint</td><td align="center">0</td><td>CAN EXTID of the message with data to follow</td></tr><tr><td>kFollowerConfig</td><td align="center">58</td><td align="center">uint</td><td align="center">0</td><td>Special configuration register for setting up to follow on a repeating message (follower mode). CFG[0] to CFG[3] where CFG[0] is the motor output start bit (LSB), CFG[1] is the motor output stop bit (MSB). CFG[0] - CFG[1] determines endianness. CFG[2] bits determine sign mode and inverted, CFG[3] sets a preconfigured controller (0x1A = REV, 0x1B = Talon/Victor style as of 2018 season)</td></tr><tr><td>kSmartCurrentStallLimit</td><td align="center">59</td><td align="center">uint</td><td align="center">80A</td><td>Smart Current Limit at stall, or any RPM less than kSmartCurrentConfig RPM.</td></tr><tr><td>kSmartCurrentFreeLimit</td><td align="center">60</td><td align="center">uint</td><td align="center">20A</td><td>Smart current limit at free speed</td></tr><tr><td>kSmartCurrentConfig</td><td align="center">61</td><td align="center">uint</td><td align="center">10000</td><td>Smart current limit RPM value to start linear reduction of current limit. Set this > free speed to disable.</td></tr><tr><td>Reserved</td><td align="center">62</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">63</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">64</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">65</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">66</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">67</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">68</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>kEncoderCountsPerRev</td><td align="center">69</td><td align="center">uint</td><td align="center">4096</td><td>Number of encoder counts in a single revolution, counting every edge on the A and B lines of a quadrature encoder. (Note: This is different than the CPR spec of the encoder which is 'Cycles per revolution'. This value is 4 * CPR.</td></tr><tr><td>kEncoderAverageDepth</td><td align="center">70</td><td align="center">uint</td><td align="center">64</td><td>Number of samples to average for velocity data based on quadrature encoder input. This value can be between 1 and 64.</td></tr><tr><td>kEncoderSampleDelta</td><td align="center">71</td><td align="center">uint</td><td align="center">200 per 500us</td><td>Delta time value for encoder velocity measurement in 500μs increments. The velocity calculation will take delta the current sample, and the sample x * 500μs behind, and divide by this the sample delta time. Can be any number between 1 and 255</td></tr><tr><td>Reserved</td><td align="center">72</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">73</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">74</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>kCompensatedNominalVoltage</td><td align="center">75</td><td align="center">float32</td><td align="center">0 V</td><td>In voltage compensation mode mode, this is the max scaled voltage.</td></tr><tr><td>kSmartMotionMaxVelocity_0</td><td align="center">76</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMaxAccel_0</td><td align="center">77</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMinVelOutput_0</td><td align="center">78</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionAllowedClosedLoopError_0</td><td align="center">79</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionAccelStrategy_0</td><td align="center">80</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMaxVelocity_1</td><td align="center">81</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMaxAccel_1</td><td align="center">82</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMinVelOutput_1</td><td align="center">83</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionAllowedClosedLoopError_1</td><td align="center">84</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionAccelStrategy_1</td><td align="center">85</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMaxVelocity_2</td><td align="center">86</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMaxAccel_2</td><td align="center">87</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMinVelOutput_2</td><td align="center">88</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionAllowedClosedLoopError_2</td><td align="center">89</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionAccelStrategy_2</td><td align="center">90</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMaxVelocity_3</td><td align="center">91</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMaxAccel_3</td><td align="center">92</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionMinVelOutput_3</td><td align="center">93</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionAllowedClosedLoopError_3</td><td align="center">94</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSmartMotionAccelStrategy_3</td><td align="center">95</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kIMaxAccum_0</td><td align="center">96</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder1_0</td><td align="center">97</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder2_0</td><td align="center">98</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder3_0</td><td align="center">99</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kIMaxAccum_1</td><td align="center">100</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder1_1</td><td align="center">101</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder2_1</td><td align="center">102</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder3_1</td><td align="center">103</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kIMaxAccum_2</td><td align="center">104</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder1_2</td><td align="center">105</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder2_2</td><td align="center">106</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder3_2</td><td align="center">107</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kIMaxAccum_3</td><td align="center">108</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder1_3</td><td align="center">109</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder2_3</td><td align="center">110</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kSlot3Placeholder3_3</td><td align="center">111</td><td align="center">float32</td><td align="center">0</td><td></td></tr><tr><td>kPositionConversionFactor</td><td align="center">112</td><td align="center">float32</td><td align="center">1</td><td></td></tr><tr><td>kVelocityConversionFactor</td><td align="center">113</td><td align="center">float32</td><td align="center">1</td><td></td></tr><tr><td>kClosedLoopRampRate</td><td align="center">114</td><td align="center">float32</td><td align="center">0 DC/sec</td><td></td></tr><tr><td>kSoftLimitFwd</td><td align="center">115</td><td align="center">float32</td><td align="center">0</td><td>Soft limit forward value</td></tr><tr><td>kSoftLimitRev</td><td align="center">116</td><td align="center">float32</td><td align="center">0</td><td>Soft limit reverse value</td></tr><tr><td>Reserved</td><td align="center">117</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">118</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>kAnalogPositionConversion</td><td align="center">119</td><td align="center">float32</td><td align="center">1 rev/volt</td><td>Conversion factor for position from analog sensor. This value is multiplied by the voltage to give an output value.</td></tr><tr><td>kAnalogVelocityConversion</td><td align="center">120</td><td align="center">float32</td><td align="center">1 vel/v/s</td><td>Conversion factor for velocity from analog sensor. This value is multiplied by the voltage to give an output value.</td></tr><tr><td>kAnalogAverageDepth</td><td align="center">121</td><td align="center">uint</td><td align="center">0</td><td>Number of samples in moving average of velocity.</td></tr><tr><td>kAnalogSensorMode</td><td align="center">122</td><td align="center">uint</td><td align="center">0</td><td>0 Absolute: In this mode the sensor position is always read as voltage * conversion factor and reads the absolute position of the sensor. In this mode setPosition() does not have an effect.<br><br>1 Relative: In this mode the voltage difference is summed to calculate a relative position.</td></tr><tr><td>kAnalogInverted</td><td align="center">123</td><td align="center">bool</td><td align="center">0</td><td>When inverted, the voltage is calculated as (ADC Full Scale - ADC Reading). This means that for absolute mode, the sensor value is 3.3V - voltage. In relative mode the direction is reversed.</td></tr><tr><td>kAnalogSampleDelta</td><td align="center">124</td><td align="center">uint</td><td align="center">0</td><td>Delta time between samples for velocity measurement</td></tr><tr><td>Reserved</td><td align="center">125</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>Reserved</td><td align="center">126</td><td align="center">-</td><td align="center">-</td><td>Reserved</td></tr><tr><td>kDataPortConfig</td><td align="center">127</td><td align="center">uint</td><td align="center">0</td><td>0: Default configuration using limit switches<br><br>1: Alternate Encoder Mode - limit switches are disabled and alternate encoder is enabled.<br>This parameter persists through a normal firmware update.</td></tr><tr><td>kAltEncoderCountsPerRev</td><td align="center">128</td><td align="center">uint</td><td align="center">4096</td><td>Number of encoder counts in a single revolution, counting every edge on the A and B lines of a quadrature encoder. (Note: This is different than the CPR spec of the encoder which is 'Cycles per revolution'. This value is 4 * CPR.</td></tr><tr><td>kAltEncoderAverageDepth</td><td align="center">129</td><td align="center">uint</td><td align="center">64</td><td>Number of samples to average for velocity data based on quadrature encoder input. This value can be between 1 and 64.</td></tr><tr><td>kAltEncoderSampleDelta</td><td align="center">130</td><td align="center">uint</td><td align="center">200</td><td>Delta time value for encoder velocity measurement in 500μs increments. The velocity calculation will take delta the current sample, and the sample x * 500μs behind, and divide by this the sample delta time. Can be any number between 1 and 255.</td></tr><tr><td>kAltEncoderInverted</td><td align="center">131</td><td align="center">bool</td><td align="center">0</td><td>Invert the phase of the encoder sensor. This is useful when the motor direction is opposite of the motor direction.</td></tr><tr><td>kAltEncoderPositionFactor</td><td align="center">132</td><td align="center">float32</td><td align="center">1</td><td>Value multiplied by the native units (rotations) of the encoder for position.</td></tr><tr><td>kAltEncoderVelocityFactor</td><td align="center">133</td><td align="center">float32</td><td align="center">1</td><td>Value multiplied by the native units (rotations) of the encoder for velocity.</td></tr></tbody></table>


<!-- Source: https://docs.revrobotics.com/brushless/spark-max/gs/basic-config (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/brushless/spark-max/gs/basic-config.md).

# Basic Configurations

SPARK MAX has many operating modes that can be configured through its CAN and USB interfaces. Additionally, the following basic operating modes can be configured with the MODE button located on the top of the SPARK MAX:&#x20;

* [Idle Behavior](/brushless/spark-max/operating-modes.md#brake-coast-mode-idle-behavior): Brake/Coast
* [Motor Type](/brushless/spark-max/operating-modes.md#brushed-brushless-mode-motor-type): Brushed/Brushless

**Mode configuration must be done with power applied to the SPARK MAX.**

{% hint style="info" %}
Configuring the idle behavior and motor type using the mode button is a quick way to set up a SPARK MAX without using a computer. This is most useful when you are controlling the SPARK MAX through its PWM interface or when testing the affect of Braking or Coasting on a mechanism.
{% endhint %}

### Idle Behavior

Whenever the SPARK MAX receives a neutral signal (no motor movement) or no signal at all (robot disabled), it can either brake the motor or let it coast. When in Brake Mode, MAX will short the motor wires to each other, electrically braking the motor. This slows the motor down very quickly if it was spinning and makes it harder, but not impossible to back-drive the motor when it is stopped.

* With power turned on, press and release the MODE button to switch between Brake and Coast Mode.
* The Status LED will indicate which idle behavior mode it is in. See the [Status LED Colors and Patterns section](/brushless/spark-max/status-led.md#standard-operation) for more information.

### Motor Type

It is very important to have the SPARK MAX configured for the appropriate motor type.&#x20;

{% hint style="danger" %}
Operating in Brushed Mode with a brushless motor connected will permanently damage the motor!
{% endhint %}

With power turned on, press and hold the MODE button for approximately 3 - 4 seconds.

* The Status LED will change and indicate which motor type is selected. See the [Status LED Colors and Patterns section](/brushless/spark-max/status-led.md#standard-operation) for more information.
* Release the MODE button.


<!-- Source: https://docs.revrobotics.com/brushless/spark-max/control-interfaces (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/brushless/spark-max/control-interfaces.md).

# SPARK MAX Control Interfaces

The SPARK MAX can be controlled by three different interfaces, servo-style PWM, controller area network (CAN), and USB. The following sections describe the operation and protocols of these interfaces. For more details on the physical connections, see [Control Connections](/brushless/spark-max/specs/control-connections.md).

## PWM Interface

The SPARK MAX can accept a standard servo-style PWM signal as a control for the output duty cycle. Even though the PWM port is shared with the CAN port, SPARK MAX will automatically detect the incoming signal type and respond accordingly. For details on how to connect a PWM cable to the SPARK MAX, see [CAN/PWM Port](/brushless/spark-max/specs/control-connections.md#can-pwm-port).

The SPARK MAX responds to a factory default pulse range of 1000µs to 2000µs. These pulses correspond to full-reverse and full-forward rotation, respectively, with 1500µs (±5% default input deadband) as the neutral position, i.e. no rotation. The input deadband is configurable with the [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client/) or the CAN interface. The table below describes how the default pulse range maps to the output behavior.

#### PWM Pulse Mapping

![](https://content.gitbook.com/content/e0CWwhMSoCEH7NLVoLhF/blobs/Dd6lRy99b9kyN8xnE4UK/PWM%20Pulse%20Mapping%20\(1\).svg)

{% hint style="warning" %}
If a valid signal isn't received within a 60ms window, the SPARK MAX will disable the motor output and either brake or coast the motor depending on the configured Idle Mode. For details on the Idle Mode, see [Idle Mode - Brake/Coast Mode](/brushless/spark-max/operating-modes.md).
{% endhint %}

## CAN Interface

The SPARK MAX can be connected to a robot CAN network. CAN is a bi-directional communications bus that enables advanced features within the SPARK MAX. SPARK MAX must be connected to a CAN network that has the appropriate termination resistors at both endpoints. Please see the FIRST Robotics Competition Robot Rules for the CAN bus wiring requirements. Even though the CAN port is shared with the PWM port, SPARK MAX will automatically detect the incoming signal type and respond accordingly. SPARK MAX uses standard CAN frames with an extended ID (29 bits), and utilizes the FRC CAN protocol for defining the bits of the extended ID:

#### CAN Packet Structure

| **ExtID \[28:24]** | **ExtID \[23:16]** | **ExtID \[15:10]** | **ExtID \[9:6]** | **ExtID \[5:0]** |
| ------------------ | ------------------ | ------------------ | ---------------- | ---------------- |
| Device Type        | Manufacturer       | API Class          | API Index        | Device ID        |

Each device on the CAN bus must be assigned a unique CAN ID number. Out of the box, SPARK MAX is assigned a device ID of 0. It is highly recommended to change all SPARK MAX CAN IDs from 0 to any unused ID from 1 to 62. CAN IDs can be changed by connecting the SPARK MAX to a Windows computer and using the [REV Hardware Client](/rev-hardware-client/home/rev-hardware-client-overview.md). For details on other SPARK MAX configuration parameters, see [Configuration Parameters](/brushless/spark-max/parameters.md).[<br>](https://github.com/REVrobotics/SPARK-MAX-Documentation/blob/master/operating-modes/broken-reference/README.md)

Additional information about the CAN accessible features and how to access them can be found in the [SPARK MAX API Information](/revlib/install.md) section.

### **Periodic Status Frames**

The SPARK MAX sends data periodically back to the roboRIO. Frequently accessed data, like motor position and temperature, can be accessed using several APIs. Data is broken up into several CAN "frames" which are sent at a periodic rate. This rate can be changed manually in code, but unlike other parameters, this setting **does not persist** through a power cycle. The rate can be set anywhere from a minimum 1ms to a maximum 32767ms period. The table below describes each status frame and its available data.

#### Periodic Status 0 - Default Rate: 10ms

| **Available Data**      | **Description**                                                                                                                                                                                                                                                                 |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Applied \*\*\*\* Output | The actual value sent to the motors from the motor controller. The frame stores this value as a 16-bit signed integer, and is converted to a floating point value between -1 and 1 by the roboRIO SDK. This value is also used by any follower controllers to set their output. |
| Faults                  | Each bit represents a different fault on the controller. These fault bits clear automatically when the fault goes away.                                                                                                                                                         |
| Sticky Faults           | The same as the Faults field, however the bits do not reset until a power cycle or a 'Clear Faults' command is sent.                                                                                                                                                            |
| Is Follower             | A single bit that is true if the controller is configured to follow another controller.                                                                                                                                                                                         |

#### Periodic Status 1 - Default Rate: 20ms

| **Available Data** | **Description**                                                                                                                                                                                                              |
| ------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Motor Velocity     | 32-bit IEEE floating-point representation of the motor velocity in RPM using the selected sensor.                                                                                                                            |
| Motor Temperature  | <p>8-bit unsigned value representing:</p><p>Firmware version 1.0.381 - Voltage of the temperature sensor with 0 = 0V and 255 = 3.3V.<br>Current firmware versions - Motor temperature in °C for the NEO Brushless Motor.</p> |
| Motor Voltage      | 12-bit fixed-point value that is converted to a floating point voltage value (in Volts) by the roboRIO SDK. This is the input voltage to the controller.                                                                     |
| Motor Current      | 12-bit fixed-point value that is converted to a floating point current value (in Amps) by the roboRIO SDK. This is the raw phase current of the motor.                                                                       |

#### Periodic Status 2 - Default Rate: 20ms

| **Available Data** | **Description**                                                               |
| ------------------ | ----------------------------------------------------------------------------- |
| Motor Position     | 32-bit IEEE floating-point representation of the motor position in rotations. |

#### Periodic Status 3 - Default Rate: 50ms

| **Available Data**     | **Description**                                                                                                                                                    |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Analog Sensor Voltage  | 10-bit fixed-point value that is converted to a floating point voltage value (in Volts) by the roboRIO SDK. This is the voltage being output by the analog sensor. |
| Analog Sensor Velocity | 22-bit fixed-point value that is converted to a floating point voltage value (in RPM) by the roboRIO SDK. This is the velocity reported by the analog sensor.      |
| Analog Sensor Position | 32-bit IEEE floating-point representation of the velocity in RPM reported by the analog sensor.                                                                    |

#### Periodic Status 4 - Default Rate: 20ms

| **Available Data**         | **Description**                                                                                  |
| -------------------------- | ------------------------------------------------------------------------------------------------ |
| Alternate Encoder Velocity | 32-bit IEEE floating-point representation of the velocity in RPM of the alternate encoder.       |
| Alternate Encoder Position | 32-bit IEEE floating-point representation of the position in rotations of the alternate encoder. |

#### Periodic Status 5 - Default Rate: 200ms

| **Available Data**                         | **Description**                                                                               |
| ------------------------------------------ | --------------------------------------------------------------------------------------------- |
| Duty Cycle Absolute Encoder Position       | 32-bit IEEE floating-point representation of the position of the duty cycle absolute encoder. |
| Duty Cycle Absolute Encoder Absolute Angle | 16-bit integer representation of the absolute angle of the duty cycle absolute encoder.       |

#### Periodic Status 6 - Default Rate: 200ms

| **Available Data**                    | **Description**                                                                                       |
| ------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| Duty Cycle Absolute Encoder Velocity  | 32-bit IEEE floating-point representation of the velocity in RPM of the duty cycle absolute encoder.  |
| Duty Cycle Absolute Encoder Frequency | 16-bit unsigned integer representation of the frequency at which the duty cycle signal is being sent. |

### **Use-case Examples**

#### **Position Control on the roboRIO**

A user wants to implement their own PID loop on the roboRIO to hold a position. They want to run this loop at 100Hz (every 10ms), but the motor position data in Periodic Status 2 is sent at 20Hz (every 50ms).

The user can change this rate to 10ms by calling:

| *Pseudocode*                                        |
| --------------------------------------------------- |
| `setPeriodicFrameRate(PeriodicFrame.kStatus2, 10);` |

#### **High CAN Utilization**

A user has many connected CAN devices and wishes to minimize the CAN bus utilization. They do not need any telemetry feedback, and have several follower devices that are only checked for faults.

The user can set the telemetry frame rates low, and set the Periodic Status 0 frame rate low on the follower devices:

| *Pseudocode*                                                                                                                                                                                                                                                                                              |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `leader.setPeriodicFrameRate(PeriodicFrame.kStatus1, 500); leader.setPeriodicFrameRate(PeriodicFrame.kStatus2, 500); follower.setPeriodicFrameRate(PeriodicFrame.kStatus0, 100); follower.setPeriodicFrameRate(PeriodicFrame.kStatus1, 500); follower.setPeriodicFrameRate(PeriodicFrame.kStatus2, 500);` |

#### **Faster Follower Bandwidth**

The user wants the follower devices to update at a faster rate: 200Hz (every 5ms).

The Periodic Status 0 frame can be increased to achieve this.

| *Pseudocode*                                              |
| --------------------------------------------------------- |
| `leader.setPeriodicFrameRate(PeriodicFrame.kStatus0, 5);` |

## USB Interface

The SPARK MAX can be configured and controlled through a USB connection to a computer running the [REV Hardware Client](/rev-hardware-client/home/rev-hardware-client-overview.md). The USB interface utilizes a standard CDC (USB to Serial) driver. The command interface is similar to CAN, using the same ID and data structure, but always sends and receives a full 12-byte packet. The CAN ID is omitted (DNC) when talking directly to the device. However, the three MSB of the ID allow selection of alternate commands:

* 0b000 - Standard command - CAN ID omitted (DNC)
* 0b001 - Extended command - USB specific

All commands sent over USB receive a response. In the case that the corresponding CAN command does not receive a response, the USB interface receives an Ack command.

#### USB Packet Structure

| **ExtID \[31:29]** | **ExtID \[28:24]** | **ExtID \[23:16]**  | **ExtID \[15:10]** | **ExtID \[9:6]** | **ExtID \[5:0]** |
| ------------------ | ------------------ | ------------------- | ------------------ | ---------------- | ---------------- |
| USB Command Type   | Device Type (2)    | Manufacturer (0x15) | API Class          | API Index        | Device ID        |

#### USB Non-Standard Commands

| **Command**                                                         | **API Class** | **API Index** |
| ------------------------------------------------------------------- | ------------- | ------------- |
| <p>Enter DFU Bootloader<br>(will also disconnect USB interface)</p> | 0             | 1             |


<!-- Source: https://docs.revrobotics.com/brushless/spark-max/operating-modes (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/brushless/spark-max/operating-modes.md).

# SPARK MAX Operating Modes

## Brushed/Brushless Mode - Motor Type

Brushed and brushless DC motors require different motor control schemes based on the differences in their technology. It is possible to damage the SPARK MAX, the motor, or both if the appropriate motor type isn't configured properly.&#x20;

Brushed or brushless motor types can be configured using the Mode Button, CAN, and USB interfaces.

### Mode Button Configuration

Follow the steps below to switch motor types with the Mode Button. It is recommended that the motor be left disconnected until the correct mode is selected.

{% hint style="info" %}
Use a small screwdriver, straightened paper clip, pen, or other small implement to press the button. Do not use any type of pencil as the pencil lead can break off inside the SPARK MAX.
{% endhint %}

1. Connect the SPARK MAX to the main power, not just USB Power.
2. The Status LED will indicate which motor type is configured by blinking yellow or blue for Brushed Mode or blinking magenta or cyan for Brushless Mode.
3. Press and hold the Mode Button for approximately 3 seconds.
4. After the button has been held for enough time, the Status LED will change and indicate the different motor configuration.
5. Release the mode button.

{% hint style="info" %}
Please see the [Status LED Patterns](/brushless/spark-max/status-led.md) guide for information on how to identify the Motor Type configuration by the color of the Status LED!
{% endhint %}

### USB Configuration

Follow the steps below to switch motor types with the USB and the REV Hardware Client application. Be sure to [download and install the REV Hardware Client](/rev-hardware-client/gs/install.md) application before continuing.

1. Connect the SPARK MAX to your computer using a USB-C cable.
2. Open the REV Hardware Client and verify that the application is connected to your SPARK MAX.
3. On the **Basic** tab, select the appropriate motor type under the **Select Motor Type** menu.
4. Click **Burn Flash** and confirm the change.

### CAN Configuration

Please see the [API Information](/revlib/install.md) for information on how to configure the SPARK MAX using the CAN interface.&#x20;

## Brake/Coast Mode - Idle Behavior

When the SPARK MAX is receiving a neutral command the idle behavior of the motor can be handled in two different ways: **Braking** or **Coasting**.&#x20;

When in **Brake Mode**, the SPARK MAX will effectively short all motor wires together. This quickly dissipates any electrical energy within the motor and brings it to a quick stop.

When in **Coast Mode**, the SPARK MAX will effectively disconnect all motor wires. This allows the motor to spin down at its own rate.

The Idle Mode can be configured using the Mode Button, CAN, and USB interfaces.

### Mode Button Configuration&#x20;

Follow the steps below to switch the Idle Mode between Brake and Coast with the Mode Button.

{% hint style="info" %}
Use a small screwdriver, straightened paper clip, pen, or other small implement to press the button. Do not use any type of pencil as the pencil lead can break off inside the SPARK MAX.
{% endhint %}

1. Connect the SPARK MAX to main power, not just USB Power.
2. The Status LED will indicate which Idle Mode is currently configured by blinking blue or cyan for Brake and yellow or magenta for Coast depending on the motor type.
3. Press and release the Mode Button
4. You should see the Status LED change to indicate the selected Idle Mode.

{% hint style="info" %}
Please see the [Status LED Patterns](/brushless/spark-max/status-led.md) guide for information on how to identify the Idle Behavior configuration by the color of the Status LED!
{% endhint %}

### USB Configuration

Follow the steps below to switch the Idle Mode between Brake and Coast with the USB and the REV Hardware Client application. Be sure to [download and install the REV Hardware Client](/rev-hardware-client/home/rev-hardware-client-overview.md) application before continuing.

1. Connect the SPARK MAX to your computer using a USB-C cable.
2. Open the REV Hardware Client application and verify that the application is connected to your SPARK MAX.
3. On the **Basic** tab, select the desired mode with the **Idle Mode** switch.
4. Click **Burn Flash** and confirm the change.

### CAN Configuration

Please see the [API Information](/revlib/install.md) for information on how to configure the SPARK MAX using the CAN interface.&#x20;
