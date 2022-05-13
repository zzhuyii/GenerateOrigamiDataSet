%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Generate Miura Pattern Gripper (Single Sample)
%  Yi Zhu & Evgueni T. Filipov
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize the solver
clear;clc;close all;
rng('shuffle')
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
           (range/2+(1-rate)*L1+1/2*L2) -L2/2 0;
           (range/2+(1-rate)*L1) 0 0;
           (range/2+(1-rate)*L1+1/2*L2) L2/2 0;
           (range/2+L1) 0 0;
           -(range/2+(1-rate)*L1+1/2*L2) -L2/2 0;
           -(range/2+(1-rate)*L1) 0 0;
           -(range/2+(1-rate)*L1+1/2*L2) L2/2 0;
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
frequency.supp=[1,1,1,0;
              4,1,1,0;
              3,1,0,0;
              5,1,0,0;
              74,0,0,1;
              75,0,0,1];
 

[frequencySquared,Umode]=ori.Dynamic_FrequencyAnalysis(frequency);
frequencySquared=diag(frequencySquared);
[freq,index]=min(abs(frequencySquared));
freqFinal=sqrt(freq);


UmodeBase=Umode(:,index);
UmodeBase=reshape(UmodeBase,3,83)'/10000;

% ori.Plot_DeformedShape(ori.newNode+ori.currentU,...
%     ori.newNode+ori.currentU+UmodeBase);



%% Setup the loading controller
% Try setting up a NR loading to prepare the shape:

nr=ControllerNRLoading;
nr.increStep=10;
nr.tol=5*10^-6;

loadVal=0.1*10^-9;
nr.load=[2 0 0 -loadVal;
    6 0 0 -loadVal;
    14 0 0 loadVal;
    27 0 0 loadVal;
    24 0 0 -loadVal;
    8 0 0 -loadVal;
];

%     60 0 0 loadVal;
%     61 0 0 loadVal;
%     59 0 0 loadVal;
%     58 0 0 -loadVal;
%     57 0 0 -loadVal;
%     56 0 0 -loadVal;
%     54 0 0 -loadVal;
%     55 0 0 -loadVal;
%     53 0 0 -loadVal;
%     71 0 0 loadVal;
%     72 0 0 loadVal;
%     73 0 0 loadVal;
%     69 0 0 -loadVal;
%     70 0 0 -loadVal;
%     68 0 0 -loadVal;
%     66 0 0 -loadVal;
%     65 0 0 -loadVal;
%     67 0 0 -loadVal;

nr.supp=[1,1,1,0;
              4,1,1,0;
              3,1,0,0;
              5,1,0,0;
              74,0,0,1;
              75,0,0,1];
          
nr.videoOpen=0;
nr.plotOpen=0;

ori.loadingController{1}={"NR",nr};
ori.Solver_Solve();


%% Thermal loading step

MaximumLoadingStep=400;

thermal=ControllerElectroThermalFolding;

thermal.thermalStep=1;
thermal.tol=5*10^-6; 

thermal.supp=[1,1,1,0;
              4,1,1,0;
              3,1,0,0;
              5,1,0,0;
              74,0,0,1;
              75,0,0,1];

thermal.thermalBoundaryPanelVec=[];
thermal.roomTempNode=[];

thermal.deltaAlpha=zeros(ori.oldCreaseNum,1);

thermal.deltaAlpha(1)= deltaAlpha;
thermal.deltaAlpha(2)= deltaAlpha;
thermal.deltaAlpha(3)= deltaAlpha;
thermal.deltaAlpha(4)= -deltaAlpha;
thermal.deltaAlpha(5)= deltaAlpha;

thermal.deltaAlpha(6)= -deltaAlpha;
thermal.deltaAlpha(8)= deltaAlpha;
thermal.deltaAlpha(9)= deltaAlpha;
thermal.deltaAlpha(12)= -deltaAlpha;

thermal.deltaAlpha(14)= deltaAlpha;
thermal.deltaAlpha(15)= deltaAlpha;
thermal.deltaAlpha(17)= -deltaAlpha;
thermal.deltaAlpha(19)= deltaAlpha;


thermal.Emat1=2*10^9; 
thermal.Emat2=2*10^9;
thermal.tmat1=t1;
thermal.tmat2=t2;
thermal.videoOpen=0;
thermal.plotOpen=0;

% the target loading of crease heating
stepHeating=[
    1,range*qload;   
    
    2,1/2*sqrt(range^2+L2^2)*qload;    
    3,1/2*sqrt(range^2+L2^2)*qload; 
    4,1/2*sqrt(range^2+L2^2)*qload;    
    5,1/2*sqrt(range^2+L2^2)*qload; 
    
    6,rate*L1*qload;
    8,sqrt(2)*L2/2*qload;
    9,sqrt(2)*L2/2*qload;   
    12,L1*(1-rate)*qload;
    
    14,rate*L1*qload;
    15,sqrt(2)*L2/2*qload;  
    17,sqrt(2)*L2/2*qload;
    19,L1*(1-rate)*qload;];

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
        thermal.tol=1*10^-7; 
    elseif i<=20
        thermal.targetCreaseHeating=stepHeatingStart2;
        thermal.tol=2*10^-6; 
    else
        thermal.targetCreaseHeating=stepHeating;
        thermal.tol=5*10^-6; 
    end
    
    % applying the thermal loading
    ori.loadingController{1}={"ElectroThermal",thermal};
    ori.Solver_Solve();
    
    
    UhisAssemble(i,:,:)=thermal.Uhis(1,:,:);
    tempHisAssemble(:,i)=thermal.temperatureHis(:,1);
    
    % Solve the distance to stop folding
    node1=squeeze(ori.newNode(59,:))'+squeeze(UhisAssemble(i,59,:));
    node2=squeeze(ori.newNode(71,:))'+squeeze(UhisAssemble(i,71,:));
    distance=norm(node1-node2);
    
    % Solve the average temperature of crease
    Tmax=max(tempHisAssemble(:,i));
    finalQ=(range+L1*2+sqrt(range^2+L2^2)+sqrt(2)*2*L2)*qload*(i-17);      
    finalStep=i;
    
    if distance<gripCloseThreshold                 
        break
    end  
end


% plot the loaded structure after the assemble step
ori.Plot_DeformedShapeTemp(thermal,...
    ori.newNode, ori.currentU+ori.newNode, ...
    squeeze(tempHisAssemble(:,finalStep)));
          


%% Apply a NR loading to determine the stiffness
nr1=ControllerNRLoading;

nr1.videoOpen=0;
nr1.plotOpen=plotFlag;

nr1.increStep=50;
nr1.tol=5*10^-6; 

nr1.supp=[1,1,1,0;
              4,1,1,0;
              3,1,0,0;
              5,1,0,0;
              74,0,0,1;
              75,0,0,1];
  
force=0.1*10^-9;
  
nr1.load=[29,force,0,0;
         32,force,0,0;
         16,-force,0,0;
         19,-force,0,0];

ori.loadingController{1}={"NR",nr1};
ori.Solver_Solve();    



% Calculate stiffness
Ubefore = nr1.Uhis(1,16,1);
Uafter=nr1.Uhis(nr1.increStep,16,1);

disp=abs(Uafter-Ubefore);
stiff=(nr1.increStep-1)*force/disp;

%% Output of the results
outputVec=[1 L1 L2 t1 t2 tpanel rate W freqFinal finalQ Tmax stiff];




