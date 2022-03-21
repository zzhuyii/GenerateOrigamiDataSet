%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Generate Miura Pattern Gripper Dataset
%  This file recalculate the frequency of the data
%  It is only used to fixed a wrong input value for mass
%  The original code is already fixed so this code is not used now
%  Yi Zhu & Evgueni T. Filipov
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize the solver
clear;clc;close all;
output=readmatrix('Miura Based.txt'); 
freqFinal=zeros(2000,1);

for t=1:2000

ori=OrigamiSolver;

%% Properties that are constant
rho=1200; % density of material
qload=0.1; % loading power per step (W)
range=2*10^-3;
deltaAlpha=50*10^(-6); % difference in thermal expansion
gripCloseThreshold=1*10^-3;


%% properties that are features
L1=output(t,2);
L2=output(t,3); % arm width
t1=output(t,4); % thickness of two layer
t2=output(t,5); % thickness of two layer
tpanel=output(t,6); % thickness of panel
rate=output(t,7);
W=output(t,8); % width of crease



%% Information to control the ploting
plotFlag=1; % determine if we need plotting
ori.x0=500;
ori.y0=500;
ori.width=800;
ori.height=800;   

ori.viewAngle1=135;
ori.viewAngle2=45;
ori.displayRange=7*10^(-3); % plotting range
ori.displayRangeRatio=1; % plotting range in the negative axis


%% Define the Geometry of origami
% This section of code is used to generate the geometry of the origami
% pattern before meshing; 
ori.node0=[-range/2 0 0;
           range/2 0 0;
           0 -L2/2 0;
           0 L2/2 0;
           (range/2+(1-rate)*L1) -L2/2 0;
           (range/2+(1-rate)*L1-1/2*L2) 0 0;
           (range/2+(1-rate)*L1) L2/2 0;
           (range/2+L1) 0 0;
           -(range/2+(1-rate)*L1) -L2/2 0;
           -(range/2+(1-rate)*L1-1/2*L2) 0 0;
           -(range/2+(1-rate)*L1) L2/2 0;
           -(range/2+L1) 0 0;
           ];
  
ori.panel0{1}=[1 3 2];
ori.panel0{2}=[1 2 4];
ori.panel0{3}=[2 3 5 6];
ori.panel0{4}=[2 6 7 4];
ori.panel0{5}=[5 8 6];
ori.panel0{6}=[6 8 7];

ori.panel0{7}=[1 10 9 3];
ori.panel0{8}=[10 1 4 11];
ori.panel0{9}=[12 9 10];
ori.panel0{10}=[12 10 11];


% Analyze the original pattern before proceeding to the next step
ori.Mesh_AnalyzeOriginalPattern();

% Plot the unmeshed origami for inspection;
% ori.Plot_UnmeshedOrigami(); 


%% Meshing of the origami model

% Define the crease width 
ori.creaseWidthVec=zeros(ori.oldCreaseNum,1);
ori.creaseWidthVec(1)=W;
ori.creaseWidthVec(2)=W;
ori.creaseWidthVec(3)=W;
ori.creaseWidthVec(4)=W;
ori.creaseWidthVec(5)=W;

ori.creaseWidthVec(6)=W;
ori.creaseWidthVec(8)=W;
ori.creaseWidthVec(9)=W;
ori.creaseWidthVec(12)=W;

ori.creaseWidthVec(14)=W;
ori.creaseWidthVec(15)=W;
ori.creaseWidthVec(17)=W;
ori.creaseWidthVec(19)=W;

% Compute the meshed geometry
ori.Mesh_Mesh()

% Plot the meshed origami for inspection;
% ori.Plot_MeshedOrigami(); 


%% Assign Mechanical Properties

ori.panelE=2*10^9; 
ori.creaseE=2*10^9; 
ori.panelPoisson=0.3;
ori.creasePoisson=0.3; 

ori.panelThickVec=tpanel*ones(11,1); 
ori.panelW=W;

% set up the diagonal rate to be large to suppress crease torsion
% ori.diagonalRate=1000;

ori.creaseThickVec=zeros(ori.oldCreaseNum,1);

ori.creaseThickVec(1)=t1+t2;
ori.creaseThickVec(2)=t1+t2;
ori.creaseThickVec(3)=t1+t2;
ori.creaseThickVec(4)=t1+t2;
ori.creaseThickVec(5)=t1+t2;

ori.creaseThickVec(6)=t1+t2;
ori.creaseThickVec(8)=t1+t2;
ori.creaseThickVec(9)=t1+t2;
ori.creaseThickVec(12)=t1+t2;

ori.creaseThickVec(14)=t1+t2;
ori.creaseThickVec(15)=t1+t2;
ori.creaseThickVec(17)=t1+t2;
ori.creaseThickVec(19)=t1+t2;

%% Assign Thermal Properties

ori.panelThermalConductVec = 0.3*ones(11,1); 

ori.creaseThermalConduct=0.3;
ori.envThermalConduct=0.026;

% thickness of the submerged environment at RT
ori.t2RT=(range+L1*2+sqrt(range^2+L2^2)+sqrt(2)*2*L2)/13*1.5; 


%% Define Density of Origami
ori.densityCrease=rho;
ori.densityPanel=rho;


%% Initialize the origami system
ori.Solver_Solve();
ori.continuingLoading=1;


%% First solve the fundamental frequency of the gripper
frequency=ControllerFrequencyAnalysis;
frequency.supp=[1,1,1,1;
              3,1,1,1;
              4,1,1,1;
              5,1,1,1;
              74,1,1,1;
              75,1,1,1]; 

[frequencySquared,Umode]=ori.Dynamic_FrequencyAnalysis(frequency);
frequencySquared=diag(frequencySquared);
[freq,index]=min(abs(frequencySquared));
freqFinal(t)=sqrt(freq);

UmodeBase=Umode(:,index);
UmodeBase=reshape(UmodeBase,3,83)'/10000;

% ori.Plot_DeformedShape(ori.newNode+ori.currentU,...
%     ori.newNode+ori.currentU+UmodeBase);


%% Output of the results
t

end
output(:,9)=freqFinal;
writematrix(output,'Miura Based.txt')



