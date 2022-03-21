%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Generate Two Panel Origami Gripper (Single Sample)
%  Yi Zhu & Evgueni T. Filipov
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize the solver
close all; clear all;
rng('shuffle')
ori=OrigamiSolver;

%% Properties that are constants
plotFlag=0; % This will diactivate the plotting
rho=1200; % density of the material
qload=0.1; % loading power per step input
range=2*10^-3;
deltaAlpha=50*10^(-6); % difference in thermal expansion
gripCloseThreshold=1*10^-3;


%% Properties that are features
L1=(rand*4+3)*10^-3; % arm length
L2=(rand*2+2)*10^-3; % arm width
t1=(rand*0.1+0.15)*10^-6; % thickness of two layer
t2=(rand*0.4+0.6)*10^-6; % thickness of two layer
tpanel=(rand*20+10)*10^-6; % thickness of panel
W=(rand*200+100)*10^-6; % width of crease
rate=(rand*0.4+0.2);


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
UmodeBase=reshape(UmodeBase,3,37)'/10000;

% ori.Plot_DeformedShape(ori.newNode+ori.currentU,...
%     ori.newNode+ori.currentU+UmodeBase);




%% Thermal loading step
maxLoadingStep=300;

newNodeNum=size(ori.newNode);
newNodeNum=newNodeNum(1);

tempHisAssemble=zeros(newNodeNum,maxLoadingStep);
UhisAssemble=zeros(maxLoadingStep,newNodeNum,3);

finalQ=0;
Tmax=0;

thermal=ControllerThermalLoading;
thermal.thermalStep=1;
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
thermal.videoOpen=0; % close the animation
thermal.plotOpen=0; % close the plot

% the target loading of crease heating
thermal.targetCreaseHeating=[2,qload*L2;
    4,qload*L2;
    7,qload*L2;
    12,qload*L2];

for i=1:maxLoadingStep

    ori.loadingController{1}={"ThermalLoading",thermal};
    ori.Solver_Solve();
    % we perform the countinuing loading step by step and check the
    % rotation angle, the loading stops after reaching 90 degree;
    
    UhisAssemble(i,:,:)=thermal.Uhis(1,:,:);
    tempHisAssemble(:,i)=thermal.temperatureHis(:,1);
    
    % Solve the distance to stop folding
    node1=squeeze(ori.newNode(12,:))'+squeeze(UhisAssemble(i,12,:));
    node2=squeeze(ori.newNode(17,:))'+squeeze(UhisAssemble(i,17,:));
    distance=norm(node1-node2);
    
    finalStep=i;
    finalQ=(qload*4*L2)*i;
    Tmax=max(tempHisAssemble(:,i));
    
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

nr1.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;];
  
force=0.1*10^-9;
  
nr1.load=[7,0,force,0;
         8,0,force,0;
         11,0,-force,0;
         12,0,-force,0];

ori.loadingController{1}={"NR",nr1};
ori.Solver_Solve();    


% calculate the stiffness
Ubefore = nr1.Uhis(1,7,2);
Uafter=nr1.Uhis(nr1.increStep,7,2);

disp=abs(Uafter-Ubefore);
stiff=(nr1.increStep-1)*force/disp;


%% Output of the results
outputVec=[3 L1 L2 t1 t2 tpanel rate W freqFinal finalQ Tmax stiff];

