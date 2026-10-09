# maxon: I2t Limitation & Motor winding protection

Source: maxon Support knowledge base, article 4412763373970. URL: https://support.maxongroup.com/hc/en-us/articles/4412763373970-maxon-controllers-I2t-Limitation-Motor-winding-protection . Article last updated 2026-02-06 (per Zendesk API). Retrieved 2026-09-28 via https://support.maxongroup.com/api/v2/help_center/en-us/articles/4412763373970.json; HTML tags stripped, text otherwise unchanged.

Topic:

How is a motor winding overload protection be ensured by the use of maxon controllers?

Is there an adjustable time period for an "overcurrent" as possible by some frequency inverters in case of AC motors?

Solution:
1.) Frequency converters and AC motors
Most frequency inverters offer an overload ratio of 110% or 160%. The overload time period might be configurable too. However, such a very simple protection model does not fit quite well to compact (ironless) servo motors and dynamic applications:

On the one hand, maxon DC/EC motors are often operated many times above the specified nominal data for a short period of time as the base of any highly dynamic motion demand, e.g. in case of fast acceleration or deceleration. 

On the other hand, the compact DC/EC motors have a lower heat capacity and internal heat dissipation than huge AC motors with lots of iron inside.

A simple method in which the overload is coupled to a fixed time period cannot fulfill these requirements optimally.
 
2.) Thermal model and overload protection of DC/EC motors
maxon controllers of the ESCON and EPOS product lines use an automatic so-called I2t limitation to protect the motor winding from overload and to estimate the thermal load of the winding (even without a temperature sensor).

The I2t algorithm measures the motor current during each current control cycle (EPOS4: 40us) and dynamically estimates the thermal load state of the winding on this basis.
 

The automatic critical winding overload detection is processed by the so-called I2t algorithm. The  winding's thermal load limit is based on the specified and configured "Nominal current" of the motor winding (at a defined ambient temperature) and the specified and also to be configured "Thermal time constant winding" of the motor. These two values must be configured correctly to ensure the protection of the winding!
This motor-related configuration is usually carried out during initial commissioning with the "Startup" wizard (-> "Drive System / Motor") of "EPOS Studio" and normally never needs to be adjusted during operation. Alternatively, configuration is also possible to configure the objects "Motor data / Nominal current" (0x3001/01) and "Motor data / Thermal time constant winding" (0x3001/04) by an application program of the higher-level controller.
 

When the estimated thermal limit of the motor winding is reached, the motor current is automatically limited to the configured "Nominal current" to prevent any further temperature rise and damage to the motor winding. So there is no(!) abrupt shutdown of the motor present but still a limited operation possible with reduced torque due to the motor current limitation.

The advantage of the I2t algorithm is a correct thermal evaluation also in cyclic operation and that the time period of a "too high" motor current is automatically adapted according to the actual current level. This means that if the motor current is only just above the configured "Nominal current", it will be available for a longer period of time than in case of a motor current which equals 2 or 3 times the motor's "Nominal current". Cyclic aspects with repetitive "overcurrent" and "cool-down" phases" are properly taken into account by the I2t algorithm.
The I2t algorithm's theory is explained in more detail by chapter "3.11.2 Output Current Limitation according to I2t Method" of the "EPOS4 Firmware Specification.pdf". There is also a graphic and calculation example of how long an "overcurrent" is available depending on the configured nominal current:

Example:

A motor current that is twice as high as the configured "Nominal current" is limited to the "Nominal current" level after 0.3 times of the "Thermal time constant winding".

A motor current that is four times as high as the configured "Nominal current" is limited to the "Nominal current" level after 0.1 times of the "Thermal time constant winding".

Hint:
How close the motor is already operated to its thermal limit (as a per mil value) can be read out by a higher-level controller via the EPOS4 object "Power limitation / I2t level motor" (0x3200/01) or also recorded with the EPOS Studio "Data Recorder" during operation just like the "Current actual value averaged" (0x30D1/01) and / or "Velocity actual value averaged" (0x30D3/01).
 
3.) Maximum output current
One important advantage of maxon DC/EC motors is that, in contrast to AC or iron-core DC motors, they can be overloaded for some period of time even by a multiple of the specified nominal data.

As a rule of thumb, there is typically a "Maximum output current" configured which equals 2-3 times of the motor's "Nominal current", or even the maximum output current of the output stage. Everything is possible without any problems and without any risk to damage a DC-/EC-motor as long as the base data (-> "Nominal current" and "Thermal time constant winding") of the I2t-limitation are configured correctly.

When configuring the "Max output current", it must be ensured that the resulting motor torque does not exceed the specified data of other mechanical components (e.g. gearboxes, spindles, ...) and can damage them. 

The "Max output current" is also configured either by EPOS Studio's "Startup Wizard" (-> "Controller / Limits") or by an application program writing the EPOS4 object "Output current limit" (0x3001/02).


 
4.) Additional notes

Detailed information and specification of all mentioned EPOS4 objects can be found in the "EPOS4 Firmware Specification.pdf".

All EPOS objects can also be accessed with the general "EPOS Command Library" function calls (or LabView VIs) "SetObject" and "GetObject".
 

5.) Cross reference

EPOS4 / IDX: What is the meaning and action of the I2t protection?


Attached extract of the "ESCON2 Firmware Specification":
This extract describes the I2t limitation in detail and also gives examples how to interpret the diagrams. The I2t principle is the same for the EPOS4, IDX, ESCON2 and ESCON, i.e. all information applies identical for all these maxon product lines.
 

 



 

