%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Generate Splitted Miura Pattern Gripper (Single Sample)
%  This file replace the specified corrupted data
%  Yi Zhu & Evgueni T. Filipov
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize the solver
tic
clear;clc;close all;
rng('shuffle')

ori=OrigamiSolver;    
    
%% properties that are constant
rho=1200; % determine density
qload=0.1*110; % loading power per step input (W)
range=1*10^-3;
deltaAlpha=50*10^(-6); % difference in thermal expansion
gripCloseThreshold=1*10^-3;


%% Properties that are features
L1=(2+3)*10^-3; % arm length
L2=(1+2)*10^-3; % arm width
t1=(0.05+0.15)*10^-6; % thickness of two layer
t2=(0.2+0.6)*10^-6; % thickness of two layer
tpanel=(10+10)*10^-6; % thickness of panel
W=(100+100)*10^-6; % width of crease
rate=0.5;


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


%% Define the Geometry of origami
% This section of code is used to generate the geometry of the origami

ori.node0=[-L2/4 -range/2 0;
           L2/4 -range/2 0;
           L2/4 range/2 0;
           -L2/4 range/2 0;
           -L2/2 -range/2-L2/4 0;
           L2/2 -range/2-L2/4 0;
           L2/2 range/2+L2/4 0;
           -L2/2 range/2+L2/4 0;
           -L2/2 range/2+(1-rate)*L1-L2/4 0;
           -L2/4 range/2+(1-rate)*L1 0;
           L2/4 range/2+(1-rate)*L1 0;
           L2/2 range/2+(1-rate)*L1-L2/4 0;
           -L2/2 range/2+L1-L2/4 0;
           -L2/4 range/2+L1 0;
           L2/4 range/2+L1 0;
           L2/2 range/2+L1-L2/4 0;
           -L2/2 -range/2-(1-rate)*L1+L2/4 0;
           -L2/4 -range/2-(1-rate)*L1 0;
           L2/4 -range/2-(1-rate)*L1 0;
           L2/2 -range/2-(1-rate)*L1+L2/4 0;
           -L2/2 -range/2-L1+L2/4 0;
           -L2/4 -range/2-L1 0;
           L2/4 -range/2-L1 0;
           L2/2 -range/2-L1+L2/4 0;];
  
ori.panel0{1}=[1 2 3 4];
ori.panel0{2}=[5 1 4 8];
ori.panel0{3}=[2 6 7 3];
ori.panel0{4}=[3 7 12 11];
ori.panel0{5}=[4 3 11 10];
ori.panel0{6}=[8 4 10 9];
ori.panel0{7}=[9 10 14 13];
ori.panel0{8}=[10 11 15 14];
ori.panel0{9}=[11 12 16 15];
ori.panel0{10}=[5 17 18 1];
ori.panel0{11}=[18 19 2 1];
ori.panel0{12}=[19 20 6 2];
ori.panel0{13}=[21 22 18 17];
ori.panel0{14}=[22 23 19 18];
ori.panel0{15}=[23 24 20 19];

% Analyze the original pattern before proceeding to the next step
ori.Mesh_AnalyzeOriginalPattern();


%% Meshing of the origami model

% Define the crease width 
ori.creaseWidthVec=zeros(ori.oldCreaseNum,1);
ori.creaseWidthVec(1)=W;
ori.creaseWidthVec(2)=W;
ori.creaseWidthVec(3)=W;
ori.creaseWidthVec(4)=W;
ori.creaseWidthVec(6)=W;
ori.creaseWidthVec(7)=W;
ori.creaseWidthVec(8)=W;
ori.creaseWidthVec(10)=W;
ori.creaseWidthVec(11)=W;
ori.creaseWidthVec(14)=W;
ori.creaseWidthVec(13)=W;
ori.creaseWidthVec(15)=W;
ori.creaseWidthVec(17)=W;
ori.creaseWidthVec(19)=W;
ori.creaseWidthVec(21)=W;

ori.creaseWidthVec(27)=W;
ori.creaseWidthVec(29)=W;
ori.creaseWidthVec(26)=W;
ori.creaseWidthVec(28)=W;
ori.creaseWidthVec(30)=W;
ori.creaseWidthVec(34)=W;
ori.creaseWidthVec(36)=W;

% Compute the meshed geometry
ori.Mesh_Mesh()

% Plot the results for inspection
% ori.Plot_UnmeshedOrigami(); % Plot the unmeshed origami for inspection;
% ori.Plot_MeshedOrigami(); % Plot the meshed origami for inspection;


%% Assign Mechanical Properties

ori.panelE=2*10^9; 
ori.creaseE=2*10^9; 
ori.panelPoisson=0.3;
ori.creasePoisson=0.3; 

ori.panelThickVec=tpanel*ones(15,1); 
ori.panelW=W;

% set up the diagonal rate to be large to suppress crease torsion
% ori.diagonalRate=1000;

ori.creaseThickVec=zeros(ori.oldCreaseNum,1);

ori.creaseThickVec(1)=t1+t2;
ori.creaseThickVec(2)=t1+t2;
ori.creaseThickVec(3)=t1+t2;
ori.creaseThickVec(4)=t1+t2;

ori.creaseThickVec(6)=t1+t2;
ori.creaseThickVec(7)=t1+t2;
ori.creaseThickVec(8)=t1+t2;
ori.creaseThickVec(10)=t1+t2;
ori.creaseThickVec(11)=t1+t2;
ori.creaseThickVec(14)=t1+t2;
ori.creaseThickVec(13)=t1+t2;
ori.creaseThickVec(15)=t1+t2;
ori.creaseThickVec(17)=t1+t2;
ori.creaseThickVec(19)=t1+t2;
ori.creaseThickVec(21)=t1+t2;

ori.creaseThickVec(27)=t1+t2;
ori.creaseThickVec(29)=t1+t2;
ori.creaseThickVec(26)=t1+t2;
ori.creaseThickVec(28)=t1+t2;
ori.creaseThickVec(30)=t1+t2;
ori.creaseThickVec(34)=t1+t2;
ori.creaseThickVec(36)=t1+t2;


%% Assign Thermal Properties

ori.panelThermalConductVec = 0.3*ones(15,1); 

ori.creaseThermalConduct=0.3;
ori.envThermalConduct=0.026;

% thickness of the submerged environment at RT
ori.t2RT=(4*L1+2*sqrt(2)*L2+2*L2+2*range)*1.5/22; 


%% Define Density of Origami
ori.densityCrease=rho;
ori.densityPanel=rho;


%% Apply a NR loading to prebias the fold
nr2=ControllerNRLoading;

nr2.videoOpen=0;
nr2.plotOpen=0;

nr2.increStep=10;
nr2.tol=5*10^-6; 

nr2.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;];
  
force=0.1*10^-9;
  
nr2.load=[
         15,0,0,-force;
         24,0,0,-force
         31,0,0,force;
         32,0,0,force;
         46,0,0,-force;
         38,0,0,-force;
         53,0,0,force;
         54,0,0,force];

ori.loadingController{1}={"NR",nr2};
ori.Solver_Solve();  
ori.continuingLoading=1;

%% Thermal Loading step
MaximumLoadingStep=400;

thermal=ControllerThermalLoading;

thermal.thermalStep=120;
thermal.tol=5*10^-6; 

thermal.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;];

thermal.thermalBoundaryPanelVec=[];
thermal.roomTempNode=[];

thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);

thermal.deltaAlpha(1)= deltaAlpha;
thermal.deltaAlpha(3)= deltaAlpha;

thermal.deltaAlpha(2)= deltaAlpha;
thermal.deltaAlpha(4)= deltaAlpha;
thermal.deltaAlpha(15)= deltaAlpha;
thermal.deltaAlpha(28)= deltaAlpha;

thermal.deltaAlpha(6)= deltaAlpha;
thermal.deltaAlpha(7)= deltaAlpha;
thermal.deltaAlpha(8)= deltaAlpha;
thermal.deltaAlpha(10)= deltaAlpha;
thermal.deltaAlpha(13)= deltaAlpha;
thermal.deltaAlpha(17)= deltaAlpha;
thermal.deltaAlpha(26)= deltaAlpha;
thermal.deltaAlpha(30)= deltaAlpha;

thermal.deltaAlpha(11)= deltaAlpha;
thermal.deltaAlpha(14)= deltaAlpha;
thermal.deltaAlpha(27)= -deltaAlpha;
thermal.deltaAlpha(29)= -deltaAlpha;

thermal.deltaAlpha(19)= deltaAlpha;
thermal.deltaAlpha(21)= deltaAlpha;
thermal.deltaAlpha(34)= deltaAlpha;
thermal.deltaAlpha(36)= deltaAlpha;



thermal.Emat1=2*10^9; 
thermal.Emat2=2*10^9;
thermal.tmat1=t1;
thermal.tmat2=t2;
thermal.videoOpen=1;
thermal.plotOpen=0;

% the target loading of crease heating
thermal.targetCreaseHeating=[
    1,range*qload;    
    3,range*qload;
    
    2,L2/2*qload;
    4,L2/2*qload;
    15,L2/2*qload;
    28,L2/2*qload;    
    
    6,sqrt(2)*L2/4*qload;
    7,sqrt(2)*L2/4*qload;
    8,sqrt(2)*L2/4*qload;
    10,sqrt(2)*L2/4*qload;
    13,sqrt(2)*L2/4*qload;
    17,sqrt(2)*L2/4*qload;
    26,sqrt(2)*L2/4*qload;
    30,sqrt(2)*L2/4*qload;    
    
    11,(1-rate)*L1*qload;
    14,(1-rate)*L1*qload;
    27,(1-rate)*L1*qload;
    29,(1-rate)*L1*qload;    
    
    19,rate*L1*qload;
    21,rate*L1*qload;    
    34,rate*L1*qload;
    36,rate*L1*qload];

newNodeNum=size(ori.newNode);
newNodeNum=newNodeNum(1);

tempHisAssemble=zeros(newNodeNum,MaximumLoadingStep);
UhisAssemble=zeros(MaximumLoadingStep,newNodeNum,3);


% applying the thermal loading
ori.loadingController{1}={"ThermalLoading",thermal};
ori.Solver_Solve();    


