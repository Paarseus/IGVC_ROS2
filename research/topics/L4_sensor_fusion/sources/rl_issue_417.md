# Issue #417: Feature request: discard erroneous measurements
URL: https://github.com/cra-ros-pkg/robot_localization/issues/417
Author: rokusottervanger  Created: 2018-09-17T15:53:04Z

Using statistical methods, it is possible to draw conclusions about the validity of the combination of a measurement and the predicted state. If a measurement (including its covariance) is not consistent with the predicted state estimation (and its respective uncertainty), it implies that either the physical model or the measurement is incorrect. I would assume that in most cases, the measurement would be incorrect. An example of this is the occurrence of multipath effects in GPS and UWB. Using this conclusion about the validity of the measurement, one can choose to use the measurement or discard it. 

A reference of this method (I know, it's not very recent): https://www.stats.ox.ac.uk/~caron/Publications/J_Information_Fusion_2004.pdf 

I'm going to experiment with this method for a project I'm working on. I'm not saying this will be the holy grail. I already see a problem where measurements are discarded for some time, the covariance of the prediction grows, enabling faulty measurements to be fused. So it should definitely be an option that can be disabled. But if I were to implement this, would you be interested in a PR?

---
## Comment by ayrton04 at 2018-09-17T16:05:40Z

This is the reason for the `*_rejection_threshold` parameters. They are specified as Mahalanobis distances, and they will use the EKF's current covariance estimate (along with the covariance measurement from the sensor) to determine whether to reject the measurement:

https://github.com/cra-ros-pkg/robot_localization/blob/kinetic-devel/src/ekf.cpp#L185
https://github.com/cra-ros-pkg/robot_localization/blob/kinetic-devel/src/filter_base.cpp#L380

EDIT: let me know if this doesn't solve your use case, though.

---
## Comment by rokusottervanger at 2018-09-17T18:37:33Z

Ah that's nice. I must admit, it's been a while since I looked at the code,
so I didn't know it was in there. We'll have a look, try it, and get back
to you if it doesn't solve the case. Thank you!

On Mon, Sep 17, 2018, 18:09 Tom Moore <notifications@github.com> wrote:

> This is the reason for the *_rejection_threshold parameters. They are
> specified as Mahalanobis distances, and they will use the EKF's current
> covariance estimate (along with the covariance measurement from the sensor)
> to determine whether to reject the measurement:
>
>
> https://github.com/cra-ros-pkg/robot_localization/blob/kinetic-devel/src/ekf.cpp#L185
>
> https://github.com/cra-ros-pkg/robot_localization/blob/kinetic-devel/src/filter_base.cpp#L380
>
> EDIT: let me know if this doesn't solve your use case, though.
>
> —
> You are receiving this because you authored the thread.
> Reply to this email directly, view it on GitHub
> <https://github.com/cra-ros-pkg/robot_localization/issues/417#issuecomment-422074989>,
> or mute the thread
> <https://github.com/notifications/unsubscribe-auth/AIlLBxwkKMtLCUAheOFDx5wPLvAVak-Eks5ub8kygaJpZM4WsNuE>
> .
>

