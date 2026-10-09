Source: https://www.itl.nist.gov/div898/handbook/prc/section2/prc263.htm
Accessed: 2026-09-28

  
7. [Product and Process Comparisons](../prc.htm)   
7.2. [Comparisons based on data from one process](prc2.htm)   
7.2.6. [What intervals contain a fixed percentage of the population values?](prc26.htm)   
  
| 

## 7.2.6.3.

| 

## Tolerance intervals for a normal distribution  
  
---|---  
_Definition of a tolerance interval_ |  A confidence interval covers a population parameter with a stated confidence, that is, a certain proportion of the time. There is also a way to cover a fixed proportion of the population with a stated confidence. Such an interval is called a _tolerance interval._ The endpoints of a tolerance interval are called _tolerance limits_. An application of tolerance intervals to manufacturing involves comparing specification limits prescribed by the client with tolerance limits that cover a specified proportion of the population.   
_Difference between confidence and tolerance intervals_ |  Confidence limits are limits within which we expect a given population parameter, such as the mean, to lie. Statistical tolerance limits are limits within which we expect a stated proportion of the population to lie.   
_Not related to engineering tolerances_ |  Statistical tolerance intervals have a probabilistic interpretation. _Engineering tolerances_ are specified outer limits of acceptability which are usually prescribed by a design engineer and do not necessarily reflect a characteristic of the actual measurements.   
_Three types of tolerance intervals_ |  Three types of questions can be addressed by tolerance intervals. Question (1) leads to a two-sided interval; questions (2) and (3) lead to one-sided intervals. 

  1. What interval will contain \\(p\\)  percent of the population measurements?
  2. What interval guarantees that \\(p\\)  percent of population measurements will not fall below a lower limit?
  3. What interval guarantees that \\(p\\)  percent of population measurements will not exceed an upper limit?

  
_Tolerance intervals for measurements from a normal distribution_ |  For the questions above, the corresponding tolerance intervals are defined by lower (L) and upper (U) tolerance limits which are computed from a series of measurements \\(Y_1, \, \ldots, \, Y_N\\): 

  1. \\(Y_L = \bar{Y} - k_2 s; \,\,\, Y_U = \bar{Y} + k_2 s\\) 
  2. \\(Y_L = \bar{Y} - k_1 s\\) 
  3. \\(Y_U = \bar{Y} + k_1 s\\) 

where the \\(k\\)  factors are determined so that the intervals cover at least a proportion \\(p\\)  of the population with confidence, \\(\alpha\\).  The value of \\(p\\) is often referred to as the coverage factor.   
_Calculation of \\(k\\) factor for a two-sided tolerance limit for a normal distribution_ |  If the data are from a normally distributed population, an approximate value for the \\(k_2\\)  factor as a function of \\(p\\)  and \\(\alpha\\)  for a two-sided tolerance interval [(Howe, 1969)](../section5/prc5.htm#Howe, W. G. \(1969\)) is $$ k_2 = z_{(1+p)/2} \sqrt{\frac{\nu \left(1 + \frac{1}{N}\right) \, }{\chi^2_{1-\alpha, \, \nu}}} \, , $$  where \\(\chi_{1-\alpha, \, \nu}^2\\)  is the [critical value of the chi-square distribution](../../eda/section3/eda3674.htm) with degrees of freedom \\(\nu\\)  that is exceeded with probability \\(\alpha\\),  and \\(z_{(1+p)/2}\\)  is the [critical value of the normal distribution](../../eda/section3/eda3671.htm) associated with cummulative probability \\((1+p)/2\\).  The quantity \\(\nu\\)  represents the degrees of freedom used to estimate the standard deviation. Most of the time the same sample will be used to estimate both the mean and standard deviation so that \\(\nu = N-1\\),  but the formula allows for other possible values of \\(\nu\\).   
_Guenther's correction to \\(k_2\\)_ |  [Guenther (1977)](../section5/prc5.htm#Guenther) recommends the following correction to Howe's approximation $$ k_2^{*} = wk_2 $$ where $$ w = \sqrt{ 1 + \frac{N -3 - \chi^2_{N-1,1 - \alpha} } {2(N+1)^2}} $$ For reasonably large values of \\(N\\), this correction factor should be close to 1. For example, for \\(N\\) = 40 and \\(\alpha\\) = 0.95, the correction factor is 0.9972.   
_Example of calculation_ |  For example, suppose that we take a sample of \\(N\\)  = 43 silicon wafers from a lot and measure their thicknesses in order to find tolerance limits within which a proportion \\(p\\)  = 0.90 of the wafers in the lot fall with confidence \\(\alpha\\)  = 0.99. Since the standard deviation, \\(s\\),  is computed from the sample of 43 wafers, the degrees of freedom are \\(\nu = N-1\\).  The reader can download the data as a [text file](../../datasets/MPC62.DAT).   
_Use of tables in calculating two-sided tolerance intervals_ |  Values of the \\(k_2\\)  factor as a function of \\(p\\) and \\(\alpha\\)  are tabulated in some textbooks, such as [Dixon and Massey (1969)](../section5/prc5.htm#Dixon, W.J and Massey, F.J). To use the normal and chi-square tables in this handbook to approximate the \\(k_2\\)  factor, follow the steps outlined below. 

  1. Calculate: \\((1+p)/2 = (1+0.9)/2 = 0.95 \\)  and \\(\nu = N-1 = 43 - 1 = 42\\). 
  2. Go to the page describing [critical values of the normal distribution](../../eda/section3/eda3671.htm). In the summary table under the column labeled 0.95, find  \\(z_{(1+p)/2} = z_{0.95} = 1.645\\). 
  3. Go to the table of [lower critical values of the chi-square distribution](../../eda/section3/eda3674.htm). Under the column labeled 0.01 in the row labeled degrees of freedom = 42, find  \\(\chi_{1-\alpha, \, \nu}^2 = \chi_{0.01, \, 42}^2 = 23.650\\). 
  4. Calculate $$ k_2 = z_{(1+p)/2} \sqrt{\frac{\nu \left(1 + \frac{1}{N}\right) \, }{\chi^2_{1-\alpha, \, \nu}}} = 1.645 \sqrt{\frac{42\left(\frac{44}{43}\right)}{23.650}} = 2.217 \, . $$ 

The tolerance limits are then computed from the sample mean, \\(\bar{Y}\\),  and standard deviation, \\(s\\),  according to [case(1)](prc263.htm#cases).   
_Important notes_ |  The notation for the critical value of the chi-square distribution can be confusing. Values as tabulated are, in a sense, already squared; whereas the critical value for the normal distribution must be squared in the formula above.  Some software is capable of computing a tolerance intervals for a given set of data so that the user does not need to perform all the calculations. All the tolerance intervals shown in this section can be computed using both [Dataplot code](prc263.dp) and [R code](prc263.r). In addition, R software is capable of computing an exact value of the \\(k_2\\)  factor thus replacing the approximation given above. R and Dataplot examples include the case where a tolerance interval is computed automatically from a data set.   
_Calculation of a one-sided tolerance interval for a normal distribution_ |  The calculation of an approximate \\(k\\)  factor for one-sided tolerance intervals comes directly from the following set of formulas [(Natrella, 1963)](../../mpc/section7/mpc7.htm#Natrella): $$ \begin{eqnarray} k_{1} & = & \frac{z_{p} + \sqrt{z_{p}^2 - ab}} {a} \\\ & & \\\ a & = & 1 - \frac{z_{\alpha}^2}{2(N-1)} \\\ & & \\\ b & = & z_{p}^2 - \frac{ z_{\alpha}^2}{N} \, . \end{eqnarray} $$   
_A one-sided tolerance interval example_ |  For the example above, it may also be of interest to guarantee with 0.99 probability (or 99 % confidence) that 90 % of the wafers have thicknesses less than an upper tolerance limit. This problem falls under [case (3)](prc253.htm#cases). The calculations for the \\(k_1\\)  factor for a one-sided tolerance interval are: $$ \begin{eqnarray} a & = & 1 - \frac{1}{2(43-1)} \, (2.3263)^2 = 0.9356\\\ & & \\\ b & = & (1.2816)^2 - \frac{1}{43} \, (2.3263)^2 = 1.5165\\\ & & \\\ k_{1} & = & \frac{1.2816 + \sqrt{(1.2816)^2 - (0.9356)(1.5165)}} {0.9356} = 1.8752 \, . \end{eqnarray} $$   
_Tolerance factor based on the non-central \\(t\\) distribution_ |  The value of \\(k_1\\)  can also be computed using the inverse cumulative distribution function for the non-central \\(t\\)  distribution. This method may give more accurate results for small values of \\(N\\).  The value of \\(k_1\\)  using the non-central \\(t\\)  distribution (using the same example as above) is: $$ \begin{eqnarray} \delta & = & z_{p} \sqrt{N} = 1.2816 \sqrt{43} = 8.4037 \\\ & & \\\ k_1 & = & \frac{ t_{\alpha, \, N-1, \, \delta} }{ \sqrt{N} } = \frac{12.28834}{\sqrt{43}} = 1.8740 \, , \end{eqnarray} $$  where \\(\delta\\)  is the non-centrality parameter.  In this case, the difference between the two computations is negligible (1.8752 versus 1.8740). However, the difference becomes more pronounced as the value of \\(N\\)  gets smaller (in particular, for \\(N \le\\)  10). For example, if \\(N\\)  = 43 is replaced with \\(N\\)  = 6, the non-central \\(t\\)  method returns a value of 4.4111 for \\(k_1\\)  while the method based on the Natrella formulas returns a value of 5.2808.  The disadvantage of the non-central \\(t\\)  method is that it depends on the inverse cumulative distribution function for the non-central \\(t\\)  distribution. This function is not available in many statistical and spreadsheet software programs, but it is available in Dataplot and R (see [Dataplot code](prc263.dp) and [R code](prc263.r)). In addition, the inverse of the non-central t function may lose accuracy for large sample sizes. The Natrella formulas only depend on the inverse cumulative distribution function for the normal distribution (which is available in just about all statistical and spreadsheet software programs). Unless you have small samples (say \\(N \le\\)  10), the difference in the methods should not have much practical effect. 

