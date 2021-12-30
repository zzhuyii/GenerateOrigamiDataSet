%%%%% Sequentially Working Origami Multi-Physics Simulator (SWOMPS)  %%%%%%
%
% Authors: Yi Zhu, and Evgueni T. Filipov
%
% Discription: This code package implement a bar and hinge model based 
% simulator for active origami structures with multi-physics based 
% actuation mechanisms. The code package can capture both the mechanical 
% behavior and the heat transfer aspect. The implementation is versatile
% and has the following features:
%
% (1) Provides 5 different loading solvers of active origami. They are: 
%     Newton-Raphson method, displacement controlled method, modified 
%     generazlied displacement controlled method, self-stress folding, and
%     thermal folding method.
% (2) Allows users to create arbitrary number and sequence of the five
%     loading methods. Users can stop the solver at specified increments
%     and switch between different solvers or edit origami systems during 
%     within the increment easily.
% (3) Simulate electro-thermo-mechanically coupled actuation of origami.
% (4) Simulate inter-panel contact of origami systems.
% (5) Simulate the compliant creases explicitly with novel bar and hinge
%     model meshing schemes.
%
% Acknowledgement: We would like to acknowledge the prior works from
% Ke Liu and Glaucio H. Paulino for establishing shared versions of
% nonrigid origami simulators. Their works paved the way for the new
% origami simulator, the origami contact, compliant crease, electro-thermal
% model presented in this package. 
%
% Reference:
% [1] Y. Zhu, E. T. Filipov (2021). 'Sequentially Working Origami Multi-
%     Physics Simulator (SWOMPS): A Versatile Implementation' (submitted)
% [2] Y. Zhu, E. T. Filipov (2021). 'Rapid Multi-Physic Simulation for 
%     Electro-Thermal Origami Robotic Systems' (submitted)
% [3] Y. Zhu, E. T. Filipov (2020). 'A Bar and Hinge Model for Simulating 
%     Bistability in Origami Structures with Compliant Creases' Journal of 
%     Mechanisms and Robotics, 021110-1. 
% [4] Y. Zhu, E. T. Filipov (2019). 'An Efficient Numerical Approach for 
%     Simulating Contact in Origami Assemblages.' Proc. R. Soc. A, 475: 
%     20190366.       
% [5] Y. Zhu, E. T. Filipov (2019). 'Simulating compliant crease origami 
%     with a bar and hinge model.' IDETC/CIE 2019. 97119. 
% [6] K. Liu, G. H. Paulino (2018). 'Highly efficient nonlinear        
%     structural analysis of origami assemblages using the MERLIN2      
%     software.' Origami^7. 
% [7] K. Liu, G. H. Paulino (2017). 'Nonlinear mechanics of non-rigid   
%     origami - An efficient computational approach.' Proc. R. Soc. A 473: 
%     20170348. 
% [8] K. Liu, G. H. Paulino (2016). 'MERLIN: A MATLAB implementation to   
%     capture highly nonlinear behavior of non-rigid origami.'           
%     Proceedings of IASS Annual Symposium 2016. 
%
%%%%% Sequentially Working Origami Multi-Physics Simulator (SWOMPS)  %%%%%%

%% Initialize the solver

clear;clc;close all;
rng('shuffle')

% set up the solver class for simulation
ori=OrigamiSolver;        
ori.showNumber=0;
ori.faceColorNumbering='yellow';
ori.faceAlphaNumbering=1;
plotFlag=1; % determine if we need plotting

    
%% Properties that are features
% Target 1
m=24;
tcrease=0.7*10^-3; % crease of origami
tpanel=1.5*10^-3; % thickness of panel
W=3*10^-3; % width of crease
offset=0.25;
stripeWidth=0.02;
extrude=0.15;

% Target 2
% m=8;
% tcrease=0.7*10^-3; % crease of origami
% tpanel=5*10^-3; % thickness of panel
% W=3*10^-3; % width of crease
% offset=0.18;
% stripeWidth=0.035;
% extrude=0.1;

% Target 3
% m=12;
% tcrease=0.8*10^-3; % crease of origami 
% tpanel=4*10^-3; % thickness of panel
% W=2.5*10^-3; % width of crease
% offset=0.11;
% stripeWidth=0.03;
% extrude=0.15;



[ori.node0,ori.panel0,error]=CurveFit_MiuraStrip(m,offset,stripeWidth,extrude);

% Analyze the original pattern before proceeding to the next step
ori.Mesh_AnalyzeOriginalPattern();

% Plot the results for inspection
ori.viewAngle1=0;
ori.viewAngle2=0;
ori.displayRange=5; % plotting range
ori.displayRangeRatio=0.1; % plotting range in the negative axis
% ori.Plot_UnmeshedOrigami();

%% Meshing of the origami model
% find the crease lines that are not on the edge
creaseIndex=ori.oldCreaseType~=1;

% Define the crease width 
ori.creaseWidthVec=zeros(ori.oldCreaseNum,1);
ori.creaseWidthVec(creaseIndex)=W;
% set the creases that are not at the edge to have width W

% Compute the meshed geometry
ori.compliantCreaseOpen=0;
ori.flag2D3D=3;
ori.Mesh_Mesh()
ori.Plot_MeshedOrigami();

%% Assign Mechanical Properties
% set up the mechanical parameters
ori.panelE=2*10^9; 
ori.creaseE=2*10^9; 
ori.panelPoisson=0.3;
ori.creasePoisson=0.3; 

A=size(ori.panel0);
panelNum=A(2);
ori.panelThickVec=tpanel*ones(panelNum,1); 
ori.panelW=W;

% set up the thickness of creases
ori.creaseThickVec=zeros(ori.oldCreaseNum,1);
ori.creaseThickVec(creaseIndex)=tcrease;


%% Set the plotting information
ori.x0=500;
ori.y0=500;
ori.width=500;
ori.height=500;    
ori.deformEdgeShow=1;

%% Apply a NR loading to determine the z direction stiffness
nrZ=ControllerNRLoading;

nrZ.videoOpen=0;
nrZ.plotOpen=plotFlag;

nrZ.increStep=5;
nrZ.tol=1*10^-7; 

nrZ.supp=[1 1 1 1;
          2*m+1 1 1 1;
          2*m+1+1 1 1 1;
          4*m+2 1 1 1;
          4*m+2+1 1 1 1;
          6*m+3 1 1 1;];
nrZ.load=[m+1, 0, 0, -1];

ori.loadingController{1}={"NR",nrZ};
ori.Solver_Solve();    

disp= abs(nrZ.Uhis(nrZ.increStep,m+1,3));    
Zstiff=nrZ.increStep*1/disp;  

Zsnap=0;
if disp>1
    Zsnap=1;
end

%% Apply a NR loading to determine the X direction stiffness
nrX=ControllerNRLoading;

nrX.videoOpen=0;
nrX.plotOpen=plotFlag;

nrX.increStep=5;
nrX.tol=1*10^-7; 

nrX.supp=[1 1 1 1;
          2*m+1 1 1 1;
          2*m+1+1 1 1 1;
          4*m+2 1 1 1;
          4*m+2+1 1 1 1;
          6*m+3 1 1 1;];
nrX.load=[m+1, 1, 0, 0];

ori.loadingController{1}={"NR",nrX};
ori.Solver_Solve();    

disp= abs(nrX.Uhis(nrX.increStep,m+1,1));    
Xstiff=nrX.increStep*1/disp;   




