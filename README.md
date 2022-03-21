# Generate Origami DataSet Using SWOMPS Package

This set of codes show how to build origami performance database using the SWOMPS simulation package (in MATLAB). 
This repostory conatins three datasets regarding the performanc of origami canopy, active origami gripper, and origami arch for shape fitting. 

## Origami canopy

<p align="center">
<img align="center" src="https://github.com/zzhuyii/GenerateOrigamiDataSet/blob/main/Figures_ReadMe/CanopyDetails.png" width="80%" >
</p>


In this database, we study the stiffness performance of an origami canopy. 
The canopy can be made using two patterns and they are a Miura origami pattern (p=1) or a TMP pattern (p=2). 
Two additional categorical variables are used to determine the number of unit cells in the pattern and they are m={24,30,36} and n={6,9,12}. 
Other design features include the thickness of panels (tp), the thickness of creases (tc), and the width of the creases (W).
The stiffness of the canopy under axial loading and bending loading are computed, and these stiffness values are measured at 30%, 60%, and 90% extension. 
The extension ratio is measured using the folded length of the pattern to the original flat length of the system. 

## Active origami gripper 

<p align="center">
<img align="center" src="https://github.com/zzhuyii/GenerateOrigamiDataSet/blob/main/Figures_ReadMe/GripperDetails.png" width="80%" >
</p>


In this database, we study the performance of an active origami gripper. 
Three different patterns are studied and the performance of the gripper include: 
(1) fundamental frequency (freq) of the gripper, 
(2) input heating power (Q) needed to close its gripping arm,
(3) maximum crease temperature (T) during the gripping motion, and
(4) stiffness (k) of the gripper in resisting loads applied to pry it open. 
Design features of the gripper include the length (L1) and width (L2) of the gripper arm, the location of the first hinge from the base (measured as a ratio Ra of the outer arm compared to the total arm length), the thickness of the two layers in the actuator design (t1 and t2), and the width of the actuator creases.


## Origami Arch for shape fitting

<p align="center">
<img align="center" src="https://github.com/zzhuyii/GenerateOrigamiDataSet/blob/main/Figures_ReadMe/ShapeFitDetails.png" width="80%" >
</p>


In this database we study the interaction between origami shape fitting and mechanical behaviors of produced origami structure. 
A Miura origami strip is built to fit the a half circle curve with a radius of 2m. 
Shape fitting parameters including number of segments m, the offset length lo, the strip width Ws, and the extrude dimension le are varied to change the produced origami strip. 
Other design features such as the thicknesses of panels (tp) and creases (tc) and the width of creases (W) are also varied when building the database because they affects the stiffness of the arch.
The performance indices include the stiffness in X-direction (kx) and Z-direction (kz), the error of shape fitting (e), and whether the structure will snap under a 5 N load applied vertically (Sz).

