# VectorNav Inertial Navigation Primer, section 1.9: Heading Determination
Source: https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-heading (rendered page text, accessed 2026-09-27)

1.9 HEADING DETERMINATION

The heading, also referred to as the yaw or azimuth, is the rotation of a system about the vertical axis of the inertial reference frame (aligned to gravity). A variety of techniques for determining a system's heading utilizing inertial sensors have been discussed in this chapter, each with their own pros and cons, and are summarized here.



MAGNETOMETER (MAGNETIC COMPASS)

A detailed description of using a magnetometer for heading determination is given in Section 1.6.



THEORY OF OPERATION

A magnetometer is used to measure the magnetic field surrounding a system. This magnetic field measurement can be compared to models of Earth's magnetic field to determine the heading of a system with respect to magnetic North. Though the geographical location of magnetic North is different than that of true North, the heading can be found with respect to true North by taking into account the declination angle between the two locations.



LIMITATIONS

Using a magnetometer to accurately estimate a system's heading can prove to be quite challenging. Earth's magnetic field is weak, making magnetometers highly susceptible to magnetic disturbances, which are caused by any ferrous materials or electric currents near the magnetometer. These disturbances will bias and distort the background magnetic field, leading to increased errors in the heading estimate. Earth's magnetic field can also shift as much as 
2
∘
 from one day to the next. Due to this, even in the most ideal magnetic environments, a magnetometer can only provide a heading accuracy of 
1
∘
 to 
2
∘
 over an extended period of time.



GNSS/INS (DYNAMIC ALIGNMENT)

A detailed description of how a GNSS/INS system achieves heading determination via dynamic alignment is given in Section 1.7.



THEORY OF OPERATION

A combined GNSS/INS system determines the heading of a system through the correlation of measurements from the two systems, a process known as dynamic alignment. The accelerometer measurements from the INS solution are compared to the position and velocity measurements from the GNSS solution to determine the heading of the system to high levels of accuracy, depending on the quality of the GNSS and INS. Note this is not the same as assuming that heading equals the course over ground reported by the GNSS.



LIMITATIONS

Though a GNSS/INS system can provide an accurate and reliable estimate of the system's heading, there are a few conditions that can cause the loss of heading observability when using this system. If the combined system experiences any low dynamic or static situations, the horizontal acceleration of the system will be nearly zero, making the comparison of the INS and GNSS measurements impossible and causing the heading observability of the system to be lost. The loss of heading observability can also be caused by GNSS outages which occur from signal blockages or signal interference.



GNSS COMPASSING

A detailed description of using a GNSS compass for heading determination is given in Section 1.8.



THEORY OF OPERATION

The GNSS compassing technique uses a form of real-time kinematic (RTK) positioning known as moving-baseline RTK to determine a system's heading. Moving-baseline RTK compares the carrier phase measurements between two GNSS antennas that are fixed relative to each other at a given distance. Through this comparison, the GNSS compass can determine a relative positioning of each antenna to millimeter-level accuracy and estimate the system's heading with an accuracy that is inversely proportional to the separation distance between the two antennas.



LIMITATIONS

While GNSS compassing can provide an accurate and reliable heading estimate, there are a few limitations of using this technique. In order for the GNSS compass to estimate a system's heading using moving-baseline RTK, the two antennas must have a clear view of the sky and observe at least six of the same satellites. The GNSS compass also has a high sensitivity to multipath and has difficulty providing an accurate heading in applications subject to multipath conditions, such as urban canyons.



GYROCOMPASSING

A detailed description of the types of gyros capable of utilizing gyrocompassing for heading determination is given in Section 1.4.



THEORY OF OPERATION

Gyrocompassing is a technique that determines a system's heading without reliance on GNSS or magnetic field measurements. This method uses a gyroscope to measure the angular rate of the earth in conjunction with measurements of the gravity vector to detect the direction of North and determine the system's heading.



LIMITATIONS

Although gyrocompassing can provide a reliable heading estimate, independent of both magnetometers and GNSS, this technique is limited to using high performance gyroscopes such as fiber-optic gyroscopes (FOGs) and ring laser gyroscopes (RLGs). The size, weight, power, and cost of gyros capable of gyrocompassing is prohibitive for most applications.
