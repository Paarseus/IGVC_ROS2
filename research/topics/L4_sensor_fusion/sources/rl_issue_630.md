# Issue #630: pose_rejection_threshold documention unclear
URL: https://github.com/cra-ros-pkg/robot_localization/issues/630
Author: jola6897  Created: 2021-03-04T05:15:28Z

## Add more details to the documentation of pose_rejection_threshold calibration parameters.
If I understand the code correctly  *_pose_rejection_threshold squared shall be the chi-square distribution quantile for a given degree of freedom in a chosen confidence.

#### Feature description
Given an input with a degree of freedom of 1 e.g. measured x-position, what confidence are you choosing? 0.05 [(chi-square quantiles table)](https://en.wikipedia.org/wiki/Chi-square_distribution)? With 1 DoF and 0.05 this would mean this is 3.84 which results in *_pose_rejection_threshold to be 1.96 approximately. 

Could you maybe elaborate on one example from the [template](https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/params/ukf_template.yaml) how you came up with the proposed values? Since I also do not see a test using the a rejection threshold that is not the numeric maximum.

I would be very glad if you could shed some light into this topic. Best regards.


---
## Comment by ayrton04 at 2021-03-09T14:15:44Z

The rejection threshold is a [Mahlanobis distance](https://github.com/cra-ros-pkg/robot_localization/blob/a2da33bb3aaa4b10358de391221fe0ed97b16f49/params/ekf_template.yaml#L117). The original PR is [here](https://github.com/cra-ros-pkg/robot_localization/pull/146).

In a 1D problem (we'll continue to use X in this example), it's effectively just an unsigned Z-score. We're measuring how many standard deviations from the mean a given measurement is:

```
distance = sqrt( (x_measured - x_state)^2 / x_variance )
```

If distance > threshold, we reject the measurement (don't fuse it). This was just a way to reject outlier measurements before they had the chance to destroy the state estimate.

The values in the template are just examples. The template doesn't refer to any specific state estimation implementation, so the values are arbitrary.


---
## Comment by jola6897 at 2021-03-09T18:47:43Z

Thanks you a lot for the explanation.

---
## Comment by nyxrobotics at 2022-09-29T11:54:07Z

I want to adjust the parameters, is there a way to publish the current Mahalanobis distance as debugging information?

---
## Comment by nyxrobotics at 2022-10-27T13:35:46Z

This issue has not been resolved.
Can you reopen?

---
## Comment by ayrton04 at 2022-10-27T15:24:01Z

I'll reopen it if someone would be willing to PR the fix. :)
