Source: https://www.itl.nist.gov/div898/handbook/mpc/section4/mpc441.htm
Accessed: 2026-09-28

  
2. [Measurement Process Characterization](../mpc.htm)   
2.4. [Gauge R & R studies](mpc4.htm)   
2.4.4. [Analysis of variability](mpc44.htm)   
  
| 

## 2.4.4.1.

| 

## Analysis of repeatability  
  
---|---  
_Case study:[Resistivity probes_](../section6/mpc61.htm) |  The repeatability quantifies the basic precision for the gauge. A level-1 [repeatability standard deviation](mpc44.htm) is computed for each group of _J_ repetitions, and a graphical analysis is recommended for deciding if repeatability is dependent on the check standard, the operator, or the gauge. Two graphs are recommended. These should show: 

  * [Plot of repeatability standard deviations versus check standard with day coded](../section6/mpc612.htm)
  * [Plot of repeatability standard deviations versus check standard with gauge coded](../section6/mpc613.htm) 
Typically, we expect the standard deviation to be gauge dependent -- in which case there should be a separate standard deviation for each gauge. If the gauges are all at the same level of precision, the values can be combined over all gauges.   
_Repeatability standard deviations can be pooled over operators, runs, and check standards_ |  A repeatability standard deviation from _J_ repetitions is not a reliable estimate of the precision of the gauge. Fortunately, these standard deviations can be pooled over days; runs; and check standards, if appropriate, to produce a more reliable precision measure. The table below shows a mechanism for pooling. The pooled repeatability standard deviation, \\( {\large s}_1 \\),  has _LK_(_J_ \- 1) degrees of freedom for measurements taken over: 

  * _J_ repetitions 
  * _K_ days 
  * _L_ runs 
  
_Basic pooling rules_ |  The table below gives the mechanism for pooling repeatability standard deviations over days and runs. The pooled value is an average of weighted variances and is shown as the last entry in the right-hand column of the table. The pooling can also cover check standards, if appropriate.   
_[View of entire dataset from the nested design](../section6/mpc6111.htm)_ |  To illustrate the calculations, a [subset of data](../../datasets/MPC441.DAT) collected in a nested design for one check standard (#140) and one probe (#2362) are shown below. The measurements are resistivity (ohm.cm) readings with six repetitions per day. The individual level-1 standard deviations from the six repetitions and degrees of freedom are recorded in the last two columns of the database. 
    
    
    **Run  Wafer  Probe  Month  Day  Op  Temp    Average  Stddev  df**
     1    140    2362    3    15   1   23.08   96.0771  0.1024  5
     1    140    2362    3    17   1   23.00   95.9976  0.0943  5
     1    140    2362    3    18   1   23.01   96.0148  0.0622  5
     1    140    2362    3    22   1   23.27   96.0397  0.0702  5
     1    140    2362    3    23   2   23.24   96.0407  0.0627  5
     1    140    2362    3    24   2   23.13   96.0445  0.0622  5
     
     2    140    2362    4    12   1   22.88   96.0793  0.0996  5
     2    140    2362    4    18   2   22.76   96.1115  0.0533  5
     2    140    2362    4    19   2   22.79   96.0803  0.0364  5
     2    140    2362    4    19   1   22.71   96.0411  0.0768  5
     2    140    2362    4    20   2   22.84   96.0988  0.1042  5
     2    140    2362    4    21   1   22.94   96.0482  0.0868  5
      
  
|  **Pooled repeatability standard deviations over days, runs** | Source of Variability | Degrees of Freedom | Standard Deviations | Sum of Squares (SS)  
---|---|---|---  
Probe 2362** |  \\( \nu_i\\)  |  \\( {\large s}_{1i} \\)  |  \\( SS_i= \nu_i {\large s}_{1i}^2 \\)   
      
    
    
    run 1 - day 1
    
    run 1 - day 2
    
    run 1 - day 3
    
    run 1 - day 4
    
    run 1 - day 5
    
    run 1 - day 6
    
    run 2 - day 1
    
    run 2 - day 2
    
    run 2 - day 3
    
    run 2 - day 4
    
    run 2 - day 5
    
    run 2 - day 6
    

| 
    
    
    
          5
    
          5
    
          5
    
          5
    
          5
    
          5
    
          5
    
          5
    
          5
    
          5
    
          5
    
          5
    

| 
    
    
    
         0.1024
    
         0.0943
    
         0.0622
    
         0.0702
    
         0.0627
    
         0.0622
    
         0.0996
     
         0.0533
    
         0.0364
    
         0.0768
    
         0.1042
    
         0.0868
    

| 
    
    
    
       0.05243
    
       0.04446
    
       0.01934
    
       0.02464
    
       0.01966
    
       0.01934
    
       0.04960
    
       0.01420
    
       0.00662
    
       0.02949
    
       0.05429
    
       0.03767
      
  
\\( \nu = \sum \nu_i \longrightarrow \\)    
gives the total degrees of freedom for \\( {\large s}_1 \\)  | 
    
    
         **60**

|  \\( SS = \sum SS_i \longrightarrow \\)    
gives the total sum of squares for \\( {\large s}_1 \\)  | 
    
    
       **0.37176**  
  
The pooled value of \\( {\large s}_1 \\)  is given by \\( {\large s}_1 = \sqrt{SS/\nu} \longrightarrow \\)  | 
    
    
       **0.07871**
    
    
    
      
    
    
    
    
    
    
    The calculations displayed in the table above can be 
    generated using both [Dataplot code](mpc441.dp) 
    and [R code](mpc441.r).
    The  reader can download the
    [data as a text file](../../datasets/MPC441.DAT).
    
      
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
       
    
    
    
    

