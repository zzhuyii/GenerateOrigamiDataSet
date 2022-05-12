%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Generate Two Panel Gripper Dataset
%  Yi Zhu & Evgueni T. Filipov
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear all;
rng('shuffle')

%% Initialize the solver
ori=OrigamiSolver;

%% Properties that are constants
plotFlag=1; % This will diactivate the plotting
rho=1200; % density of the material
qload=9; % loading power per step input
range=2*10^-3;
deltaAlpha=50*10^(-6); % difference in thermal expansion
gripCloseThreshold=1*10^-3;


%% Properties that are features
L1=(2+3)*10^-3; % arm length
L2=(1+2)*10^-3; % arm width
t1=(0.05+0.15)*10^-6; % thickness of two layer
t2=(0.2+0.6)*10^-6; % thickness of two layer
tpanel=(10+10)*10^-6; % thickness of panel
W=(100+100)*10^-6; % width of crease
rate=(0.3+0.2);


%% Information to control the ploting
plotFlag=1; % determine if we need plotting
ori.x0=500;
ori.y0=500;
ori.width=800;
ori.height=800;   

ori.viewAngle1=45;
ori.viewAngle2=45;
ori.displayRange=7*10^(-3); % plotting range
ori.displayRangeRatio=1; % plotting range in the negative axis

%% Generate origami patterns
ori.node0=[0 -range/2 0;
           L2 -range/2 0;
           0 range/2 0;
           L2 range/2 0;
           0 range/2+(1-rate)*L1 0;
           L2 range/2+(1-rate)*L1 0;
           0 range/2+L1 0;
           L2 range/2+L1 0;
           0 -range/2-(1-rate)*L1 0;
           L2 -range/2-(1-rate)*L1 0;
           0 -range/2-L1 0;
           L2 -range/2-L1 0;];

ori.panel0{1}=[1 2 4 3];
ori.panel0{2}=[3 4 6 5];
ori.panel0{3}=[5 6 8 7];
ori.panel0{4}=[9 10 2 1];
ori.panel0{5}=[11 12 10 9];

% Analyze the original pattern before proceeding to the next step
ori.Mesh_AnalyzeOriginalPattern();


% Plot the results for inspection
% Plot the unmeshed origami for inspection;
% ori.Plot_UnmeshedOrigami(); 

%% Meshing of the origami model

% Define the crease width 
ori.creaseWidthVec=zeros(ori.oldCreaseNum,1);
ori.creaseWidthVec(2)=W;
ori.creaseWidthVec(4)=W;
ori.creaseWidthVec(7)=W;
ori.creaseWidthVec(12)=W;

% Compute the meshed geometry
ori.Mesh_Mesh()
% Plot the meshed origami for inspection;
% ori.Plot_MeshedOrigami(); 


%% Assign Mechanical Properties

ori.panelE=2*10^9; 
ori.creaseE=2*10^9; 
ori.panelPoisson=0.3;
ori.creasePoisson=0.3; 
ori.panelThickVec=[tpanel;
    tpanel;
    tpanel;
    tpanel;
    tpanel]; 
ori.panelW=W;

% set up the diagonal rate to be large to suppress crease torsion
ori.diagonalRate=100;

ori.creaseThickVec=zeros(ori.oldCreaseNum,1);
ori.creaseThickVec(2)=(t1+t2);
ori.creaseThickVec(4)=(t1+t2);
ori.creaseThickVec(7)=(t1+t2);
ori.creaseThickVec(12)=(t1+t2);

%% setup panel contact information

ori.contactOpen=0;
ori.ke=0.0001;
ori.d0edge=40*(10^(-6));
ori.d0center=40*(10^(-6));


%% Assign Thermal Properties

ori.panelThermalConductVec = [0.3;
    0.3;0.3;0.3;0.3]; 
ori.creaseThermalConduct=0.3;
ori.envThermalConduct=0.026;

% thickness of the submerged environment at RT
ori.t2RT=L2*1.5;


%% Define Density of Origami
ori.densityCrease=rho;
ori.densityPanel=rho;


%% Thermal loading step
maxLoadingStep=300;

newNodeNum=size(ori.newNode);
newNodeNum=newNodeNum(1);

tempHisAssemble=zeros(newNodeNum,maxLoadingStep);
UhisAssemble=zeros(maxLoadingStep,newNodeNum,3);

finalQ=0;
Tmax=0;

thermal=ControllerElectroThermalFolding;
thermal.thermalStep=100;
thermal.tol=5*10^-7; 

thermal.supp=[1,1,1,1;
              2,1,1,1;
              3,1,1,1;
              4,1,1,1;];  

thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);
thermal.deltaAlpha(2)=deltaAlpha;
thermal.deltaAlpha(4)=deltaAlpha;
thermal.deltaAlpha(7)=deltaAlpha;
thermal.deltaAlpha(12)=deltaAlpha;

thermal.Emat1=2*10^9;
thermal.Emat2=2*10^9;
thermal.tmat1=t1;
thermal.tmat2=t2;
thermal.videoOpen=1; % close the animation
thermal.plotOpen=0; % close the plot

% the target loading of crease heating
thermal.targetCreaseHeating=[2,qload*L2;
    4,qload*L2;
    7,qload*L2;
    12,qload*L2];

ori.loadingController{1}={"ThermalLoading",thermal};
ori.Solver_Solve();

