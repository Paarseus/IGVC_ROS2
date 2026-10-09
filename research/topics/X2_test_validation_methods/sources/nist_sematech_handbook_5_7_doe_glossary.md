Source: https://www.itl.nist.gov/div898/handbook/pri/section7/pri7.htm
Accessed: 2026-09-28

  
5. [Process Improvement](../pri.htm)   
  
| 

## 5.7.

| 

## A Glossary of DOE Terminology  
  
---|---  
_Definitions for key DOE terms_ | This page gives definitions and information for many of the basic terms used in DOE. 

> * **_Alias_** : When the estimate of an effect also includes the influence of one or more other effects (usually high order interactions) the effects are said to be _aliased_(see _confounding_). For example, if the estimate of effect D in a four factor experiment actually estimates (D + ABC), then the main effect D is aliased with the 3-way interaction ABC. **Note** : This causes no difficulty when the higher order interaction is either non-existent or insignificant.
> * **_Analysis of Variance (ANOVA)_** : A mathematical process for separating the variability of a group of observations into assignable causes and setting up various significance tests.
> * **_Balanced Design_** : An experimental design where all cells (i.e. treatment combinations) have the same number of observations.
> * **_Blocking_** : A schedule for conducting treatment combinations in an experimental study such that any effects on the experimental results due to a known change in raw materials, operators, machines, etc., become concentrated in the levels of the blocking variable. **Note** : the reason for blocking is to isolate a systematic effect and prevent it from obscuring the main effects. Blocking is achieved by restricting randomization.
> * **_Center Points_** : Points at the center value of all factor ranges.
>   
> **_Coding Factor Levels_ : **Transforming the scale of measurement for a factor so that the high value becomes +1 and the low value becomes -1 (see _scaling_). After coding all factors in a 2-level full factorial experiment, the design matrix has all orthogonal columns.  Coding  is a simple linear transformation of the original measurement scale. If the "high" value is Xh and the "low" value is XL (in the original scale), then the scaling transformation takes any original X value and converts it to (X - a)/b, where   
> a = (Xh \+ XL)/2 and b = ( Xh -X L)/2.   
> To go back to the original measurement scale, just take the coded value and multiply it by "b" and add "a" or, X = b(coded value) + a. As an example, if the factor is temperature and the high setting is 65oC and the low setting is 55oC, then a = (65 + 55)/2 = 60 and b = (65 - 55)/2 = 5. The center point (where the coded value is 0) has a temperature of 5(0) + 60 =  60oC.     
> * **_Comparative Designs_** : A design aimed at making conclusions about one a priori important factor, possibly in the presence of one or more other "nuisance" factors.
> * **_Confounding_** :  A confounding design is one where some treatment effects (main or interactions) are estimated by the same linear combination of the experimental observations as some blocking effects. In this case, the treatment effect and the blocking effect are said to be _confounded_. Confounding is also used as a general term to indicate that the value of a main effect estimate comes from both the main effect itself and also contamination or bias from higher order interactions. Note: Confounding designs naturally arise when [full factorial designs](../section3/pri331.htm) have to be run in blocks and the block size is smaller than the number of different treatment combinations. They also occur whenever a [fractional factorial design](../section3/pri332.htm) is chosen instead of a full factorial design. 
> * **_Crossed Factors:_** See _factors below_.
> * **_Design_** : A set of experimental runs which allows you to fit a particular model and estimate your desired effects.
> * **_Design Matrix_** : A [matrix description](../section3/pri32.htm#The Design) of an experiment that is useful for constructing and analyzing experiments.
> * _**Effect** :_ How changing the settings of a factor changes the response. The effect of a single factor is also called a _main effect_. **Note:** For a factor A with two levels, scaled so that low = -1 and high = +1, the effect of A is estimated by subtracting the average response when A is -1 from the average response when A = +1 and dividing the result by 2 (division by 2 is needed because the -1 level is 2 scaled units away from the +1 level).
> * **_Error_** : Unexplained variation in a collection of observations. **Note** : DOE's typically require understanding of both random error and lack of fit error. 
> * **_Experimental Unit_** : The entity to which a specific treatment combination is applied. **Note** : an experimental unit can be a
> 
>   * PC board
>   * silicon wafer
>   * tray of components simultaneously treated
>   * individual agricultural plants
>   * plot of land
>   * automotive transmissions
>   * etc.
> 

> * _**Factors** : Process_ _inputs_ an investigator manipulates to cause a change in the output. Some factors cannot be controlled by the experimenter but may effect the responses. If their effect is significant, these _uncontrolled factors_ should be measured and used in the data analysis. **Note** : The inputs can be discrete or continuous.
> 
>   * **_Crossed Factors_** : Two factors are _crossed_ if every level of one occurs with every level of the other in the experiment. 
>   * **_Nested Factors_** : A factor "A" is nested within another factor "B" if the levels or values of "A" are different for every level or value of "B". **Note** : Nested factors or effects have a hierarchical relationship.
> 

> * **_Fixed Effect_** : An effect associated with an input variable that has a limited number of levels or in which only a limited number of levels are of interest to the experimenter.
> * **_Interactions_** :__ Occurs when the effect of one factor on a response depends on the level of another factor(s). 
> * **_Lack of Fit Error_** :__ Error that occurs when the analysis omits one or more important terms or factors from the process model. **Note** : Including replication in a DOE allows separation of experimental error into its components: lack of fit and random (pure) error.
> * **_Model_** :__ Mathematical relationship which relates changes in a given response to changes in one or more factors. 
> * **_Nested Factors_** : See _factors_ above.
> * **_Orthogonality_** : Two vectors of the same length are orthogonal if the sum of the products of their corresponding elements is 0. **Note** : An experimental design is orthogonal if the effects of any factor balance out (sum to zero) across the effects of the other factors.
> * **_Random Effect_** : An effect associated with input variables chosen at random from a population having a large or infinite number of possible values. 
> * **_Random error_** :__ Error that occurs due to natural variation in the process. Note: Random error is typically [assumed ](../section2/pri24.htm)to be normally distributed with zero mean and a constant variance. **Note** : Random error is also called experimental error.
> * **_Randomization_** :__ A schedule for allocating treatment material and for conducting treatment combinations in a DOE such that the conditions in one run neither depend on the conditions of the previous run nor predict the conditions in the subsequent runs. **Note** : The importance of randomization cannot be over stressed. Randomization is necessary for conclusions drawn from the experiment to be correct, unambiguous and defensible. 
> * **_Replication_** :__ Performing the same treatment combination more than once. **Note** : Including replication allows an estimate of the random error independent of any lack of fit error. 
> * **_Resolution_** : A term which describes the degree to which estimated main effects are aliased (or confounded) with estimated 2-level interactions, 3-level interactions, etc. In general, the resolution of a design is one more than the smallest order interaction that some main effect is confounded (aliased) with. If some main effects are confounded with some 2-level interactions, the resolution is 3. **Note** :[ Full factorial](../section3/pri331.htm) designs have no confounding and are said to have resolution "infinity". For most practical purposes, a resolution 5 design is excellent and a resolution 4 design may be adequate. Resolution 3 designs are useful as economical screening designs. 
> * **_Responses_** :__ The output(s) of a process. Sometimes called dependent variable(s).
> * **_Response Surface Designs_** : A DOE that fully explores the process window and models the responses. **Note** : These designs are most effective when there are less than 5 factors. Quadratic models are used for response surface designs and at least three levels of every factor are needed in the design.
> * **_Rotatability_** : A design is _rotatable_ if the variance of the predicted response at any point **x** depends only on the distance of **x** from the design center point. A design with this property can be rotated around its center point without changing the prediction variance at **x**. **Note** : Rotatability is a desirable property for response surface designs (i.e. quadratic model designs).
> * **_Scaling Factor Levels_** : [Transforming](../section3/pri32.htm#Definition of) factor levels so that the high value becomes +1 and the low value becomes -1. 
> * **_Screening Designs_** :__ A DOE that identifies which of many factors have a significant effect on the response. **Note** : Typically screening designs have more than 5 factors.
> * **_Treatment_** : A treatment is a specific combination of factor levels whose effect is to be compared with other treatments.
> * **_Treatment Combination_** :__ The combination of the settings of several factors in a given experimental trial. Also known as a _run_.
> * **_Variance Components_** :__ Partitioning of the overall variation into assignable components.

  
 

