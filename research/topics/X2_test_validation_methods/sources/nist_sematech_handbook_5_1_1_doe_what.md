Source: https://www.itl.nist.gov/div898/handbook/pri/section1/pri11.htm
Accessed: 2026-09-28

  
5. [Process Improvement](../pri.htm)   
5.1. [Introduction](pri1.htm)   
  
| 

## 5.1.1.

| 

## What is experimental design?  
  
---|---  
_Experimental Design (or DOE) economically maximizes information_ |  In an experiment, we deliberately change one or more process variables (or factors) in order to observe the effect the changes have on one or more response variables. The (statistical) design of experiments (_DOE_) is an efficient procedure for planning experiments so that the data obtained can be analyzed to yield valid and objective conclusions.  DOE begins with determining the [objectives](../section3/pri31.htm) of an experiment and selecting the [process factors](../section3/pri32.htm) for the study. An _Experimental Design_ is the laying out of a detailed experimental plan in advance of doing the experiment. Well chosen experimental designs maximize the amount of "information" that can be obtained for a given amount of experimental effort.  The statistical theory underlying DOE generally begins with the concept of _process models_.   
__ |  **Process Models for DOE**  
_Black box process model_ |  It is common to begin with a process [model](pri12.htm#Model:) of the `black box' type, with several discrete or continuous input [factors](pri12.htm#Factors:) that can be controlled--that is, varied at will by the experimenter--and one or more measured output [responses](pri12.htm#Responses). The output responses are assumed continuous. Experimental data are used to derive an empirical (approximation) model linking the outputs and inputs. These empirical models generally contain [first and second-order terms](../section2/pri23.htm).  Often the experiment has to account for a number of uncontrolled factors that may be discrete, such as different machines or operators, and/or continuous such as ambient temperature or humidity. Figure 1.1 illustrates this situation.   
_Schematic for a typical process with controlled inputs, outputs, discrete uncontrolled factors and continuous uncontrolled factors_ |    
**FIGURE 1.1  ** **A `Black Box' Process Model Schematic**  
_Models for DOE's_ |  The most common empirical models fit to the experimental data take either a _linear_ form or _quadratic_ form.   
_Linear model_ |  A linear model with two factors, _X_ 1 and _X_ 2, can be written as 

\\( Y = \beta_{0} + \beta_{1}X_{1} + \beta_{2}X_{2} + \beta_{12}X_{1}X_{2} + \mbox{experimental error} \\) 
Here, _Y_ is the response for given levels of the [main effects](pri12.htm#Effect:) _X_ 1 and _X_ 2 and the _X_ 1 _X_ 2 term is included to account for a possible [interaction](pri12.htm#Interactions:) effect between _X_ 1 and _X_ 2. The constant \\( \beta_{0} \\)  is the response of _Y_ when both main effects are 0.  For a more complicated example, a linear model with three factors _X_ 1, _X_ 2, _X_ 3 and one response, _Y_ , would look like (if all possible terms were included in the model) 

> \\( Y = \beta_{0} + \beta_{1}X_{1} + \beta_{2}X_{2} + \beta_{3}X_{3} + \beta_{12}X_{1}X_{2} + \\\ \beta_{13}X_{1}X_{3} + \beta_{23}X_{2}X_{3} + \beta_{123}X_{1}X_{2}X_{3} + \\\ \mbox{experimental error} \\) 

The three terms with single "_X_ 's" are the _main[effects](pri12.htm#Effect:)_ terms. There are _k_(_k_ -1)/2 = 3*2/2 = 3 _two-way[interaction](pri12.htm#Interactions:)_ terms and 1 _three-way_ interaction term (which is often omitted, for simplicity). When the experimental data are analyzed, all the unknown "\\( \beta \\)"  parameters are estimated and the coefficients of the "_X_ " terms are tested to see which ones are significantly different from 0.   
_Quadratic model_ |  A second-order (quadratic) model (typically used in [_response surface_](pri12.htm#Response Surface) DOE's with suspected curvature) does not include the three-way interaction term but adds three more terms to the linear model, namely 

> \\( \beta_{11}X_{1}^{2} + \beta_{22}X_{2}^{2} + \beta_{33}X_{3}^{2} \\) 

**Note:** Clearly, a full model could include many cross-product (or interaction) terms involving squared X's. However, in general these terms are not needed and most DOE software defaults to leaving them out of the model. 

