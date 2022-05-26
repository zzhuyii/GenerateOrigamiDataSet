%% check the rigid-foldability of the generated the Miura strip

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
tcrease=0.02*10^-3; % crease of origami
tpanel=4*10^-3; % thickness of panel
W=1*10^-3; % width of crease

m=8;
offset=0.3;
stripeWidth=0.2;
extrude=0.2;


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
ori.mesh2D3D=3;
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


%% Check developability
nrX=ControllerNRLoading;

nrX.videoOpen=0;
nrX.plotOpen=plotFlag;
nrX.detailFigOpen=1;

nrX.increStep=5;
nrX.tol=1*10^-7; 

nrX.supp=[1 1 1 1;
          2*m+1 0 1 1;
          m+1 0 1 0];
          
nrX.load=[2*m+1+1, 0, 0.01, 0;
          4*m+2+1, 0, -0.01, 0;];

ori.loadingController{1}={"NR",nrX};
ori.Solver_Solve();    

disp= abs(nrX.Uhis(nrX.increStep,m+1,1));    
Xstiff=nrX.increStep*1/disp;   




