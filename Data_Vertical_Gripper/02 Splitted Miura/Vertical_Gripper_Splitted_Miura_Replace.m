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
tic
clear;clc;close all;
rng('shuffle')

output=readmatrix('Splitted Miura.txt');
tempSampleNum=1956;

ori=OrigamiSolver;    
    
%% properties that are constant
rho=1200; % determine density
qload=0.1; % loading power per step input (W)
range=1*10^-3;
deltaAlpha=50*10^(-6); % difference in thermal expansion
gripCloseThreshold=1*10^-3;


%% Properties that are features
L1=(rand*4+3)*10^-3; % arm length
L2=(rand*2+2)*10^-3; % arm width
t1=(rand*0.1+0.15)*10^-6; % thickness of two layer
t2=(rand*0.4+0.6)*10^-6; % thickness of two layer
tpanel=(rand*20+10)*10^-6; % thickness of panel
W=(rand*200+100)*10^-6; % width of crease

for k=1:10        
    rate=(rand*0.4+0.2);
    if (1-rate)*L1 > (L2/2 + 1.5*W)
        break
    end
end

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
UmodeBase=reshape(UmodeBase,3,141)'/10000;

% ori.Plot_DeformedShape(ori.newNode+ori.currentU,...
%     ori.newNode+ori.currentU+UmodeBase);

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


%% Thermal Loading step

MaximumLoadingStep=400;

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
thermal.videoOpen=0;
thermal.plotOpen=0;

% the target loading of crease heating
stepHeating=[
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

stepHeatingStart=stepHeating;
stepHeatingStart(:,2)=1/20*stepHeating(:,2);

stepHeatingStart2=stepHeating;
stepHeatingStart2(:,2)=1/10*stepHeating(:,2);

stepHeatingStart3=stepHeating;
stepHeatingStart3(:,2)=1/5*stepHeating(:,2);

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
        thermal.tol=1*10^-6; 
    elseif i<=30
        thermal.targetCreaseHeating=stepHeatingStart3; 
        thermal.tol=2*10^-6; 
    else
        thermal.targetCreaseHeating=stepHeating;
        thermal.tol=2*10^-6; 
    end
    
    % applying the thermal loading
    ori.loadingController{1}={"ThermalLoading",thermal};
    ori.Solver_Solve();    
    
    UhisAssemble(i,:,:)=thermal.Uhis(1,:,:);
    tempHisAssemble(:,i)=thermal.temperatureHis(:,1);
    
        % Solve the distance to stop folding
    node1=squeeze(ori.newNode(32,:))'+squeeze(UhisAssemble(i,32,:));
    node2=squeeze(ori.newNode(53,:))'+squeeze(UhisAssemble(i,53,:));
    distance=norm(node1-node2);
    % Solve the average temperature of crease
    Tmax=max(tempHisAssemble(:,i));
    finalQ=(4*L1+2*sqrt(2)*L2+2*L2+...
        2*range)*qload*(i-26.5);      
    finalStepBase=i;
    
    if distance<gripCloseThreshold               
        break
    end  
end

if plotFlag==1
    % plot the folded structure after the assemble step
    ori.Plot_DeformedShapeTemp(thermal,...
        ori.newNode, ori.currentU+ori.newNode, ...
        squeeze(tempHisAssemble(:,finalStepBase)));
    
    
    F = getframe(gcf);
    [X, Map] = frame2im(F); 
    name=append('Sample',string(tempSampleNum),' folded structure','.png');
    imwrite(X,name)
    
end

close;

%% Apply a NR loading to determine the stiffness
% % use one small loading to prepare the simulation
nr1=ControllerNRLoading;
nr1.videoOpen=0;
nr1.plotOpen=0;
nr1.increStep=3;
nr1.tol=1*10^-7; 
nr1.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;];  
force=0.1*10^-9;  
nr1.load=[31,0,force,0;
         32,0,force,0;
         53,0,-force,0;
         54,0,-force,0];

ori.loadingController{1}={"NR",nr1};
ori.Solver_Solve();    

% Store the loaded geometry
% F = getframe(gcf);
% [X, Map] = frame2im(F); 
% name=append('Sample',string(tempSampleNum),' loading prepare','.png');
% imwrite(X,name)
% close;


% Perform the actual loading
nr2=ControllerNRLoading;
nr2.videoOpen=0;
nr2.plotOpen=plotFlag;
nr2.increStep=3;
nr2.tol=1*10^-7; 
nr2.supp=[1,1,1,1;
      2,1,1,1;
      3,1,1,1;
      4,1,1,1;];  
force=10*10^-9;  
nr2.load=[31,0,force,0;
         32,0,force,0;
         53,0,-force,0;
         54,0,-force,0];

     
Ubefore = ori.currentU(31,2);
ori.loadingController{1}={"NR",nr2};
ori.Solver_Solve();  

% Store the loaded geometry
F = getframe(gcf);
[X, Map] = frame2im(F); 
name=append('Sample',string(tempSampleNum),' loaded structure','.png');
imwrite(X,name)
close;

% Calculate the stiffness of the gripper
Uafter=nr2.Uhis(nr2.increStep,31,2);

disp=abs(Uafter-Ubefore);
stiff=(nr2.increStep)*2*force/disp;


%% Output of the results
outputVec=[2 L1 L2 t1 t2 tpanel rate W freqFinal finalQ Tmax stiff];

if finalStepBase~=MaximumLoadingStep
    output(tempSampleNum,:)=outputVec;
end
writematrix(output,'Splitted Miura.txt')
toc
