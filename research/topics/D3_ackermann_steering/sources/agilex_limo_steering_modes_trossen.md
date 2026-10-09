[  Trossen Robotics Docs Home ](https://docs.trossenrobotics.com/)

* * *

[ ![Logo](../_static/logo_limo.png) ](../index.html)

  * [Getting Started](../getting_started.html)
  * [Operation](../operation.html)
    * [Mobile App Usage](app.html)
    * Steering Modes
  * [Demos](../demos.html)
  * [Specifications](../specifications.html)
  * [Tips and Tricks](../tips_and_tricks.html)
  * [Downloads](../downloads.html)

__[AgileX LIMO Documentation](../index.html)

  * [](../index.html)
  * [Operation](../operation.html)
  * Steering Modes
  * [ Edit on GitHub](https://github.com/TrossenRobotics/agilex_limo_docs/blob/main/operation/steering_modes.rst)

* * *

# Steering Modes

## Overview

Latch Status | Indicator Color | Current Steering Mode or Status  
---|---|---  
Any | Blinking Red | Low Battery or Main Controller Alarm  
Solid Red | LIMO Stopped Due to Error  
Inserted | Yellow | Four-wheel Differential Drive or Tracked  
Blue | Mecanum  
Released | Green | Ackermann  
Ackermann | Four-wheel Differential | Tracked | Mecanum  
---|---|---|---  
![../_images/ackermann.png](../_images/ackermann.png) | ![../_images/differential.png](../_images/differential.png) | ![../_images/tracked.png](../_images/tracked.png) | ![../_images/mecanum.png](../_images/mecanum.png)  
  
  * **Ackermann:** The Ackermann steering geometry is a geometric arrangement of linkages in the steering of a car or other vehicle designed to solve the problem of wheels on the inside and outside of a turn needing to trace out circles of different radii. [[Wikipedia]](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)
  * **Four-wheel Differential:** A differential wheeled robot is a mobile robot whose movement is based on two separately driven wheels placed on either side of the robot body. It can thus change its direction by varying the relative rate of rotation of its wheels and hence does not require an additional steering motion. Robots with such a drive typically have one or more caster wheels to prevent the vehicle from tilting. [[Wikipedia]](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
  * **Tracked:** Tank steering systems allow a tank, or other continuous track vehicle, to turn. Because the tracks cannot be angled relative to the hull (in any operational design), steering must be accomplished by speeding one track up, slowing the other down (or reversing it), or a combination of both. [[Wikipedia]](https://en.wikipedia.org/wiki/Tank_steering_systems)
  * **Mecanum:** The mecanum wheel is an omnidirectional wheel design for a land-based vehicle to move in any direction. [[Wikipedia]](https://en.wikipedia.org/wiki/Mecanum_wheel)

## Switching Steering Modes

### Switching to Ackermann

Pull up the latches on both sides, turn 30 degrees clockwise to make the
longer line on both latches points to the front of the vehicle body, and then
they will be stuck. When the vehicle light turns solid green, the robot is in
Ackermann steering mode.

![../_images/ackermann_1.png](../_images/ackermann_1.png) | ![../_images/ackermann_2.png](../_images/ackermann_2.png)  
---|---  
  
### Switching to Differential

Pull up the latches on both sides, turn 30 degrees clockwise to make the
shorter line on the two latches points to the front of the vehicle body. At
this point, it is in insertion state. Fine-tune the tire angle to align the
hole so that the latch is inserted. When the vehicle light turns solid yellow,
the the robot is in Four-wheel Differential steering mode.

![../_images/differential_1.png](../_images/differential_1.png) | ![../_images/differential_2.png](../_images/differential_2.png)  
---|---  
  
### Switching to Tracked

With the robot in four-wheel differential mode, the track can be put on
directly. It is recommended to put the track on the rear wheel with small
space first.

Warning

When using Tracked mode, please lift the doors on both sides to prevent
scratches.

![../_images/tracked_1.png](../_images/tracked_1.png)

### Switching to Mecanum

First remove the hubcaps and tires, leaving only the hub motors. Then,
ensuring that the small roller of each Mecanum wheel is facing the center of
the body, install the Mecanum wheel with the included M3*5 screws.

![../_images/mecanum_1.png](../_images/mecanum_1.png) | ![../_images/mecanum_2.png](../_images/mecanum_2.png) | ![../_images/mecanum_3.png](../_images/mecanum_3.png)  
---|---|---  
  
Note

When switching to the Mecanum steering mode, make sure that each Mecanum wheel
is installed at the angle shown above in the third picture.

[ Previous](app.html "Mobile App Usage") [Next ](../demos.html "Demos")

* * *

(C) Copyright 2022, Trossen Robotics.

Built with [Sphinx](https://www.sphinx-doc.org/) using a
[theme](https://github.com/readthedocs/sphinx_rtd_theme) provided by [Read the
Docs](https://readthedocs.org).

