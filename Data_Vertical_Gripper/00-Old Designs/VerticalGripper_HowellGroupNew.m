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
sampleNum=1;
tempSampleNum=1;
output=zeros(sampleNum,11);

for t=1:sampleNum

ori=OrigamiSolver;

%% Properties that are constant
rho=1200; % density of material
qload=0.1; % loading power per step (W)
range=2*10^-3;
deltaAlpha=50*10^(-6); % difference in thermal expansion
gripCloseThreshold=1*10^-3;


%% properties that are features
L1=(rand*4+3)*10^-3; % arm length
L2=(rand*2+2)*10^-3; % arm width
t1=(rand*0.1+0.15)*10^-6; % thickness of two layer
t2=(rand*0.4+0.6)*10^-6; % thickness of two layer
tpanel=(rand*20+10)*10^-6; % thickness of panel
rate=(rand*0.4+0.2);
W=(rand*200+100)*10^-6; % width of crease



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
% pattern before meshing; 
ori.node0=[-range/2 -L2/6 0;
           range/2 -L2/6 0;
           range/2 L2/6 0;
           -range/2 L2/6 0;
           -(range/2-1/6*L2) L2/2 0;
           (range/2-1/6*L2) L2/2 0;
           -(range/2-1/6*L2) -L2/2 0;
           (range/2-1/6*L2) -L2/2 0;
           range/2+L1*rate L2/6 0;
           range/2+L1*rate -L2/6 0;
           -range/2-L1*rate L2/6 0;
           -range/2-L1*rate -L2/6 0;
           range/2+L1 L2/6 0;
           range/2+L1 -L2/6 0;
           -range/2-L1 L2/6 0;
           -range/2-L1 -L2/6 0;
           ];
  
ori.panel0{1}=[1 2 3 4];
ori.panel0{2}=[3 4 5 6];
ori.panel0{3}=[1 2 8 7];
ori.panel0{4}=[2 3 9 10];
ori.panel0{5}=[1 4 11 12];

ori.panel0{6}=[3 6 9];
ori.panel0{7}=[2 8 10];
ori.panel0{8}=[1 7 12];
ori.panel0{9}=[4 5 11];

ori.panel0{10}=[12 11 15 16];
ori.panel0{11}=[10 14 13 9];


% Analyze the original pattern before proceeding to the next step
ori.Mesh_AnalyzeOriginalPattern();


% Plot the results for inspection
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

ori.creaseWidthVec(11)=W;
ori.creaseWidthVec(12)=W;
ori.creaseWidthVec(14)=W;
ori.creaseWidthVec(15)=W;

ori.creaseWidthVec(13)=W;
ori.creaseWidthVec(16)=W;

% Compute the meshed geometry
ori.Mesh_Mesh()

% Plot the meshed origami for inspection;
%ori.Plot_MeshedOrigami(); 


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

ori.creaseThickVec(11)=t1+t2;
ori.creaseThickVec(12)=t1+t2;
ori.creaseThickVec(14)=t1+t2;
ori.creaseThickVec(15)=t1+t2;

ori.creaseThickVec(13)=t1+t2;
ori.creaseThickVec(16)=t1+t2;

%% Assign Thermal Properties

ori.panelThermalConductVec = 0.3*ones(11,1); 

ori.creaseThermalConduct=0.3;
ori.envThermalConduct=0.026;

% thickness of the submerged environment at RT
ori.t2RT=(8/3/sqrt(3)*L2+4*L1*rate+2*range+2/3*L2+2/3*L2)/14*1.5; 


%% Define Density of Origami
ori.densityCrease=rho;
ori.densityPanel=rho;


%% Initialize the origami system
ori.Solver_Solve();
ori.continuingLoading=1;


%% First solve the fundamental frequency of the gripper
frequency=ControllerFrequencyAnalysis;
frequency.supp=[1,1,1,1;
              2,1,1,1;
              3,1,1,1;
              4,1,1,1;];
 

[frequencySquared,Umode]=ori.Dynamic_FrequencyAnalysis(frequency);
frequencySquared=diag(frequencySquared);
[freq,index]=min(abs(frequencySquared));
freqFinal=sqrt(freq);


UmodeBase=Umode(:,index);
UmodeBase=reshape(UmodeBase,3,93)'/10000;

% ori.Plot_DeformedShape(ori.newNode+ori.currentU,...
%     ori.newNode+ori.currentU+UmodeBase);



%% Setup the loading controller
% Try setting up a NR loading to prepare the shape:

nr=ControllerNRLoading;
nr.increStep=10;
nr.tol=5*10^-6;

loadVal=0.1*10^-9;
nr.load=[15 0 0 loadVal;
    16 0 0 loadVal;
    19 0 0 loadVal;
    20 0 0 loadVal;
    11 0 0 -loadVal;
    12 0 0 -loadVal; 
    7 0 0 -loadVal; 
    8 0 0 -loadVal;];

nr.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;];
nr.videoOpen=0;
nr.plotOpen=0;

ori.loadingController{1}={"NR",nr};
ori.Solver_Solve();


%% Thermal loading step
MaximumLoadingStep=300;

thermal=ControllerThermalLoading;

thermal.thermalStep=1;
thermal.tol=5*10^-6; 

thermal.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;];

thermal.thermalBoundaryPanelVec=[];
thermal.roomTempNode=[];

thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);

thermal.deltaAlpha(1)= -deltaAlpha;
thermal.deltaAlpha(2)= -deltaAlpha;
thermal.deltaAlpha(3)= deltaAlpha;
thermal.deltaAlpha(4)= -deltaAlpha;

thermal.deltaAlpha(5)= deltaAlpha;
thermal.deltaAlpha(6)= -deltaAlpha;
thermal.deltaAlpha(8)= deltaAlpha;
thermal.deltaAlpha(9)= -deltaAlpha;

thermal.deltaAlpha(11)= deltaAlpha;
thermal.deltaAlpha(12)= -deltaAlpha;
thermal.deltaAlpha(14)= -deltaAlpha;
thermal.deltaAlpha(15)= deltaAlpha;

thermal.deltaAlpha(13)= -deltaAlpha;
thermal.deltaAlpha(16)= deltaAlpha;


thermal.Emat1=2*10^9; 
thermal.Emat2=2*10^9;
thermal.tmat1=t1;
thermal.tmat2=t2;
thermal.videoOpen=0;
thermal.plotOpen=0;

% the target loading of crease heating
stepHeating=[
    1,1/3*L2*qload;    
    3,1/3*L2*qload;
    
    2,range*qload;
    4,range*qload;
    
    5,2/3/sqrt(3)*L2*qload;
    6,2/3/sqrt(3)*L2*qload;
    8,2/3/sqrt(3)*L2*qload;
    9,2/3/sqrt(3)*L2*qload;
    
    11,L1*qload;
    12,L1*qload;
    14,L1*qload;
    15,L1*qload;
    
    13,L1*qload;
    16,L1*qload;];

stepHeatingStart=stepHeating;
stepHeatingStart(:,2)=1/10*stepHeating(:,2);

stepHeatingStart2=stepHeating;
stepHeatingStart2(:,2)=1/5*stepHeating(:,2);

newNodeNum=size(ori.newNode);
newNodeNum=newNodeNum(1);

tempHisAssemble=zeros(newNodeNum,MaximumLoadingStep);
UhisAssemble=zeros(MaximumLoadingStep,newNodeNum,3);

for i=1:MaximumLoadingStep

    if i<=10     
        thermal.targetCreaseHeating=stepHeatingStart;
    elseif i<=20
        thermal.targetCreaseHeating=stepHeatingStart2;
    else
        thermal.targetCreaseHeating=stepHeating;
    end
    
    % applying the thermal loading
    ori.loadingController{1}={"ThermalLoading",thermal};
    ori.Solver_Solve();
    
    
    UhisAssemble(i,:,:)=thermal.Uhis(1,:,:);
    tempHisAssemble(:,i)=thermal.temperatureHis(:,1);
    
        % Solve the distance to stop folding
    node1=squeeze(ori.newNode(35,:))'+squeeze(UhisAssemble(i,35,:));
    node2=squeeze(ori.newNode(39,:))'+squeeze(UhisAssemble(i,39,:));
    distance=norm(node1-node2);
    % Solve the average temperature of crease
    Tmax=max(tempHisAssemble(:,i));
    finalQ=(8/3/sqrt(3)*L2+4*L1*rate+2/3*L2+2*range+2/3*L2)*qload*(i-17);      
    finalStep=i;
    
    if distance<gripCloseThreshold                 
        break
    end  
end

if plotFlag==1
    % plot the loaded structure after the assemble step
    ori.Plot_DeformedShapeTemp(thermal,...
        ori.newNode, ori.currentU+ori.newNode, ...
        squeeze(tempHisAssemble(:,finalStep)));
end


%% Apply a NR loading to determine the stiffness
nr1=ControllerNRLoading;

nr1.videoOpen=0;
nr1.plotOpen=plotFlag;

nr1.increStep=50;
nr1.tol=5*10^-6; 

nr1.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;];
  
force=0.1*10^-9;
  
nr1.load=[15,force,0,0;
         16,force,0,0;
         19,-force,0,0;
         20,-force,0,0];

ori.loadingController{1}={"NR",nr1};
ori.Solver_Solve();    

Ubefore = nr1.Uhis(1,16,1);
Uafter=nr1.Uhis(nr1.increStep,16,1);

disp=abs(Uafter-Ubefore);
stiff=(nr1.increStep-1)*force/disp;

%% Output of the results
outputVec=[L1 L2 t1 t2 tpanel rate W freqFinal finalQ Tmax stiff];

if finalStep~=MaximumLoadingStep
    output(tempSampleNum,:)=outputVec;
    tempSampleNum=tempSampleNum+1;
end
end
