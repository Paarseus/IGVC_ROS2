Source: https://www.itl.nist.gov/div898/handbook/prc/section2/prc241.htm
Accessed: 2026-09-28

  
7. [Product and Process Comparisons](../prc.htm)  
7.2. [Comparisons based on data from one process](prc2.htm)  
7.2.4. [Does the proportion of defectives meet requirements?](prc24.htm)  
  


## 7.2.4.1. Confidence intervals   
  
---  
_Confidence intervals using the method of Agresti and Coull_ |  The Wilson method for calculating confidence intervals for proportions (introduced by Wilson (1927), recommended by [Brown, Cai and DasGupta (2001)](../section5/prc5.htm#Brown) and [Agresti and Coull (1998)](../section5/prc5.htm#Agresti)) is based on inverting the hypothesis test given in [Section 7.2.4](prc24.htm). That is, solve for the two values of \\(p_0\\)  (say, \\(p_{upper}\\) and \\(p_{lower}\\))  that result from setting \\(z = z_{1-\alpha/2}\\)  and solving for \\(p_0 = p_{upper}\\),  and then setting \\(z = z_{\alpha/2}\\)  and solving for \\(p_0 = p_{lower}\\).  (Here, as in Section 7.2.4, \\(z_{\alpha/2}\\)  denotes the variate value from the [standard normal distribution](../../eda/section3/eda3661.htm) such that the area to the left of the value is \\(\alpha/2\\).)  Although solving for the two values of \\(p_0\\)  might sound complicated, the appropriate expressions can be obtained by straightforward but slightly tedious algebra. Such algebraic manipulation isn't necessary, however, as the appropriate expressions are given in various sources. Specifically, we have   
_Formulas for the confidence intervals_ |  $$ \large \begin{eqnarray} \mbox{U.L. } & = & \frac{\hat{p} + \frac{z^2_{1-\alpha/2}}{2n} + z_{1-\alpha/2} \sqrt{ \frac{\hat{p}(1-\hat{p})}{n} + \frac{z^2_{1-\alpha/2}}{4n^2} }} {1 + \frac{z^2_{1-\alpha/2}}{n}} \\\ & & \\\ & & \\\ \mbox{L.L. } & = & \frac{\hat{p} + \frac{z^2_{\alpha/2}}{2n} + z_{\alpha/2} \sqrt{ \frac{\hat{p}(1-\hat{p})}{n} + \frac{z^2_{\alpha/2}}{4n^2} }} {1 + \frac{z^2_{\alpha/2}}{n}} \, . \end{eqnarray} $$   
_Procedure does not strongly depend on values of \\(p\\) and \\(n\\)_ |  This approach can be substantiated on the grounds that it is the exact algebraic counterpart to the (large-sample) hypothesis test given in section 7.2.4 and is also supported by the research of Agresti and Coull. One advantage of this procedure is that its worth does not strongly depend upon the value of \\(n\\)  and/or \\(p\\),  and indeed was recommended by Agresti and Coull for virtually all combinations of \\(n\\) and \\(p\\).   
_Another advantage is that the lower limit cannot be negative_ |  Another advantage is that the lower limit cannot be negative. That is not true for the confidence expression most frequently used: $$ \hat{p} \pm z_{1-\alpha/2}\sqrt{\frac{\hat{p}(1-\hat{p})}{n} } \, . $$  A confidence limit approach that produces a lower limit which is an impossible value for the parameter for which the interval is constructed is an inferior approach. This also applies to limits for the control charts that are discussed in Chapter 6.   
_One-sided confidence intervals_ |  A one-sided confidence interval can also be constructed simply by replacing each \\(z_{\alpha/2}\\)  by \\(z_{\alpha}\\)  in the expression for the lower or upper limit, whichever is desired. The 95 % one-sided interval for \\(p\\)  for the example in the preceding section is:   
_Example_ |  $$ \large \begin{eqnarray} p & \ge & \mbox{lower limit} \\\ & \\\ p & \ge & \frac{\hat{p} + \frac{z^2_{\alpha}}{2n} + z_{\alpha} \sqrt{ \frac{\hat{p}(1-\hat{p})}{n} + \frac{z^2_{\alpha}}{4n^2} }} {1 + \frac{z^2_{\alpha}}{n}} \\\ & & \\\ & & \\\ p & \ge & \frac{0.13 + \frac{(-1.645)^2}{2(200)} -1.645 \sqrt{ \frac{0.13(1-0.13)}{200} + \frac{(-1.645)^2}{4(200)^2} }} {1 + \frac{(-1.645)^2}{200}} \\\ & & \\\ p & \ge & 0.09577 \, . \end{eqnarray} $$   
_Conclusion from the example_ |  Since the lower bound does not exceed 0.10, in which case it would exceed the hypothesized value, the null hypothesis that the proportion defective is at most 0.10, which was given in the preceding section, would not be rejected if we used the confidence interval to test the hypothesis. Of course a confidence interval has value in its own right and does not have to be used for hypothesis testing.   
__ |  **Exact Intervals for Small Numbers of Failures and/or Small Sample Sizes**  
_Constrution of exact two-sided confidence intervals based on the binomial distribution_ |  If the number of failures is very small or if the sample size \\(N\\)  is very small, symmetical confidence limits that are approximated using the normal distribution may not be accurate enough for some applications. An _exact method_ based on the binomial distribution is shown next. To construct a two-sided confidence interval at the \\(100(1-\alpha)\\) %  confidence level for the true proportion defective \\(p\\)  where \\(N_d\\)  defects are found in a sample of size \\(N\\)  follow the steps below. 

  1. Solve the equation, $$ \sum_{k=0}^{N_d} \left( \begin{array}{c} N \\\ k \end{array} \right) p_{U}^k (1-p_{U})^{N-k} = \alpha/2 \, , $$  for \\(p_U\\)  to obtain the upper \\(100(1-\alpha)\\) %  limit for \\(p\\). 
  2. Next solve the equation, $$ \sum_{k=0}^{N_d-1} \left( \begin{array}{c} N \\\ k \end{array} \right) p_{L}^k (1-p_{L})^{N-k} = 1 - \alpha/2 \, , $$  for \\(p_L\\)  to obtain the lower \\(100(1-\alpha)\\) %  limit for \\(p\\). 

  
_Note_ |  The interval \\((p_L, \, p_U)\\)  is an exact \\(100(1-\alpha)\\) %  confidence interval for \\(p\\).  However, it is not symmetric about the observed proportion defective, \\(\hat{p} = N_d/N\\).   
_Binomial confidence interval example_ |  The equations above that determine \\(p_L\\) and \\(p_U\\)  can be solved using readily available functions. Take as an example the situation where twenty units are sampled from a continuous production line and four items are found to be defective. The proportion defective is estimated to be \\(\hat{p}\\)  = 4/20 = 0.20. The steps for calculating a 90 % confidence interval for the true proportion defective, \\(p\\)  follow. 
    
    
      1. Initalize constants.
         alpha = 0.10
         Nd = 4
         N = 20
    
      2. Define a function for upper limit (fu) and a function 
         for the lower limit (fl).
         fu = _F_(Nd,pu,20) - alpha/2
         fl = _F_(Nd-1,pl,20) - (1-alpha/2)
    
         _F_ is the cumulative density function for the 
         binominal distribution.
      
      3. Find the value of pu that corresponds to fu = 0 and
         the value of pl that corresponds to fl = 0 using software
         to find the roots of a function.
    

The values of \\(p_U\\) and \\(p_L\\) for our example are: 
    
    
         pu = 0.401029
         pl = 0.071354
    

Thus, a 90 % confidence interval for the proportion defective, \\(p\\),  is (0.071, 0.400). Whether or not the interval is truly "exact" depends on the software.  The calculations used in this example can be performed using both [Dataplot code](prc241.dp) and [R code](prc241.r).   
_Terminology Note_ |  Previous versions of the Handbook referred to the method described here as the Agresti-Coull method. However, common practice in the statistics literature is to refer to the method given here as the Wilson method and a similar, but different, method described in Brown, Cai, and DasGupta as the Agresti-Coull method (the Agresti-Coull paper refers to this as the "adjusted Wald" method). We have modified our terminology to be consistent with common practice in the statistical literature. 

