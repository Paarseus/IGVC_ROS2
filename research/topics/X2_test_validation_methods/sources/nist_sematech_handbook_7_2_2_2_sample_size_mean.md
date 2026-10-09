Source: https://www.itl.nist.gov/div898/handbook/prc/section2/prc222.htm
Accessed: 2026-09-28

  
7. [Product and Process Comparisons](../prc.htm)   
7.2. [Comparisons based on data from one process](prc2.htm)   
7.2.2. [Are the data consistent with the assumed process mean?](prc22.htm)   
  
| 

## 7.2.2.2.

| 

## Sample sizes required  
  
---|---  
_The computation of sample sizes depends on many things, some of which have to be assumed in advance_ |  Perhaps one of the most frequent questions asked of a statistician is, 

"How many measurements should be included in the sample?" 
Unfortunately, there is no correct answer without additional information (or assumptions). The sample size required for an experiment designed to investigate the behavior of an unknown population mean will be influenced by the following: 
  * value selected for \\(\alpha\\),  the risk of rejecting a true hypothesis, 
  * value of \\(\beta\\),  the risk of accepting a false null hypothesis when a particular value of the alternative hypothesis is true, 
  * value of the population standard deviation. 
  
_Application - estimating a minimum sample size, \\(N\\), for limiting the error in the estimate of the mean_ |  For example, suppose that we wish to estimate the average daily yield, \\(\mu\\),  of a chemical process by the mean of a sample, \\(Y_1, \, \ldots, \, Y_N\\),  such that the error of estimation is less than \\(\delta\\)  with a probability of 95 %. This means that a 95 % confidence interval centered at the sample mean should be $$ \bar{Y} - \delta \le \mu \le \bar{Y} + \delta \, , $$  and if the standard deviation is known, $$ \delta = \frac{\sigma}{\sqrt{N}} \, z_{1 - 0.025} \, . $$  The [critical value from the normal distribution](../../eda/section3/eda3671.htm) for 1 - \\(\alpha\\)  /2 = 0.975 is 1.96. Therefore, $$ N \ge \left( \frac{1.96}{\delta} \right)^2 \sigma^2 \, . $$   
_Limitation and interpretation_ |  A restriction is that the standard deviation must be known. Lacking an exact value for the standard deviation requires some accommodation, perhaps the best estimate available from a previous experiment.   
_Controlling the risk of accepting a false hypothesis_ |  To control the risk of accepting a false hypothesis, we set not only \\(\alpha\\),  the probability of rejecting the null hypothesis when it is true, but also \\(\beta\\),  the probability of accepting the null hypothesis when in fact the population mean is \\(\mu + \delta\\)  where \\(\delta\\)  is the difference or shift we want to detect.   
_Standard deviation assumed to be known_ |  The minimum sample size, \\(N\\),  is shown below for two- and one-sided tests of hypotheses with \\(\sigma\\)  assumed to be known. $$ \begin{eqnarray} N = (z_{1-\alpha/2} + z_{1-\beta})^2 \left( \frac{\sigma}{\delta} \right)^2 \rightarrow two-sided \,\, test \\\ N = (z_{1-\alpha} + z_{1-\beta})^2 \left( \frac{\sigma}{\delta} \right)^2 \rightarrow one-sided \,\, test\\\ \end{eqnarray} $$  The quantities \\(z_{1-\alpha/2}\\) and \\(z_{1-\beta}\\)  are critical values from the [normal distribution](../../eda/section3/eda3671.htm).  Note that it is usual to state the shift, \\(\delta\\),  in units of the standard deviation, thereby simplifying the calculation.   
_Example where the shift is stated in terms of the standard deviation_ |  For a one-sided hypothesis test where we wish to detect an increase in the population mean of one standard deviation, the following information is required: \\(\alpha\\),  the significance level of the test, and \\(\beta\\),  the probability of failing to detect a shift of one standard deviation. For a test with \\(\alpha\\)  = 0.05 and \\(\beta\\)  = 0.10, the minimum sample size required for the test is $$ N = (1.645 + 1.282)^2 = 8.567 \approx 9 \, . $$   
_More often we must compute the sample size with the population standard deviation being unknown_ |  The procedures for computing sample sizes when the standard deviation is not known are similar to, but more complex, than when the standard deviation is known. The formulation depends on the _t_ distribution where the minimum sample size is given by $$ \begin{eqnarray} N = (t_{1-\alpha/2} + t_{1-\beta})^2 \left( \frac{s}{\delta} \right)^2 \rightarrow two-sided \,\, test \\\ N = (t_{1-\alpha} + t_{1-\beta})^2 \left( \frac{s}{\delta} \right)^2 \rightarrow one-sided \,\, test\\\ \end{eqnarray} $$  The drawback is that [critical values of the _t_ distribution](../../eda/section3/eda3672.htm) depend on known degrees of freedom, which in turn depend upon the sample size which we are trying to estimate.   
_Iterate on the initial estimate using critical values from the \\(t\\) table_ |  Therefore, the best procedure is to start with an intial estimate based on a sample standard deviation and iterate. Take the example discussed above where the the minimum sample size is computed to be \\(N\\)  = 9. This estimate is low. Now use the formula above with degrees of freedom \\(N\\)  \- 1 = 8 which gives a second estimate of $$ N = (1.860 + 1.397)^2 = 10.6 \approx 11 \, . $$  It is possible to apply another iteration using degrees of freedom 10, but in practice one iteration is usually sufficient. For the purpose of this example, results have been rounded to the closest integer; however, computer programs for finding critical values from the \\(t\\)  distribution allow non-integer degrees of freedom.   
_Table showing minimum sample sizes for a two-sided test_ |  The table below gives sample sizes for a two-sided test of hypothesis that the mean is a given value, with the shift to be detected a multiple of the standard deviation. For a one-sided test at significance level \\(\alpha\\),  look under the value of 2\\(\alpha\\)  in column 1. Note that this table is based on the normal approximation (i.e., the standard deviation is known).  **Sample Size Table for Two-Sided Tests** |  \\(\alpha\\)  |  \\(\beta\\)  |  \\(\delta = 0.5 \sigma\\)  |  \\(\delta = 1.0 \sigma\\)  |  \\(\delta = 1.5 \sigma\\)   
---|---|---|---|---  
  
* * *  
  
0.01 | 0.01 | 98 | 25 | 11  
0.01 | 0.05 | 73 | 18 | 8  
0.01 | 0.10 | 61 | 15 | 7  
0.01 | 0.20 | 47 | 12 | 6  
0.01 | 0.50 | 27 | 7 | 3  
0.05 | 0.01 | 75 | 19 | 9  
0.05 | 0.05 | 53 | 13 | 6  
0.05 | 0.10 | 43 | 11 | 5  
0.05 | 0.20 | 33 | 8 | 4  
0.05 | 0.50 | 16 | 4 | 3  
0.10 | 0.01 | 65 | 16 | 8  
0.10 | 0.05 | 45 | 11 | 5  
0.10 | 0.10 | 35 | 9 | 4  
0.10 | 0.20 | 25 | 7 | 3  
0.10 | 0.50 | 11 | 3 | 3  
0.20 | 0.01 | 53 | 14 | 6  
0.20 | 0.05 | 35 | 9 | 4  
0.20 | 0.10 | 27 | 7 | 3  
0.20 | 0.20 | 19 | 5 | 3  
0.20 | 0.50 | 7 | 3 | 3

