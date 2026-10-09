Source: https://www.itl.nist.gov/div898/handbook/eda/section2/eda251.htm
Accessed: 2026-09-28

  
1. [Exploratory Data Analysis](../eda.htm)   
1.2. [EDA Assumptions](eda2.htm)   
1.2.5. [Consequences](eda25.htm)   
  
| 

## 1.2.5.1.

| 

## Consequences of Non-Randomness  
  
---|---  
_Randomness Assumption_ |  There are four underlying assumptions: 

  1. randomness; 
  2. fixed location; 
  3. fixed variation; and 
  4. fixed distribution. 
The randomness assumption is the most critical but the least tested.   
_Consequeces of Non-Randomness_ |  If the randomness assumption does not hold, then 

  1. All of the usual statistical tests are invalid. 
  2. The calculated uncertainties for commonly used statistics become meaningless. 
  3. The calculated minimal sample size required for a pre-specified tolerance becomes meaningless. 
  4. The simple model: y = constant + error becomes invalid. 
  5. The parameter estimates become suspect and non-supportable. 
  
_Non-Randomness Due to Autocorrelation_ |  One specific and common type of non-randomness is autocorrelation. Autocorrelation is the correlation between _Y t_ and _Y t-k_, where _k_ is an integer that defines the lag for the autocorrelation. That is, autocorrelation is a time dependent non-randomness. This means that the value of the current point is highly dependent on the previous point if _k_ = 1 (or _k_ points ago if _k_ is not 1). Autocorrelation is typically detected via an [autocorrelation plot](../section3/autocopl.htm) or a [lag plot](../section3/lagplot.htm).  If the data are not random due to autocorrelation, then 

  1. Adjacent data values may be related. 
  2. There may not be _n_ independent snapshots of the phenomenon under study. 
  3. There may be undetected "junk"-outliers. 
  4. There may be undetected "information-rich"-outliers. 


