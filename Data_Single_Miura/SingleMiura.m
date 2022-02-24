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
sampleNum=500;

output=readmatrix('SingleMiura.txt');
A=size(output);
existingSample=A(1);
tempSampleNum=existingSample+1;

for t=(existingSample+1):(existingSample+sampleNum)

ori=OrigamiSolver;       
ori.showNumber=0;
ori.faceColorNumbering='yellow';
ori.faceAlphaNumbering=1;

    
%% properties that are constant

rho=1200; % determine density
plotFlag=1; % determine if we need plotting
L=0.04; % assume the material is from a sheet with 0.04m*0.04m size 

%% Properties that are features

tcrease=(rand*0.5+0.5)*10^-3; % crease of origami
tpanel=(rand*5+1)*10^-3; % thickness of panel
W=(rand*3+1)*10^-3; % width of crease
gama=(rand*30+50)*pi/180;


%% Define the Geometry of origami
% This section of code is used to generate the geometry of the origami

m=2;
n=2;

bendstiff=[0 0 0];
axialstiff=[0 0 0];

Extset=[0.3,0.6,0.9];

for k=1:3

    Ext=Extset(k);

    b=L/m;
    a=(L-b*cot(gama))/n;

    [ori.node0,ori.panel0]=GenerateMiuraSheet(a,b,gama,m,n,Ext);


    % Analyze the original pattern before proceeding to the next step
    ori.Mesh_AnalyzeOriginalPattern();

    % Plot the results for inspection
    ori.viewAngle1=45;
    ori.viewAngle2=45;
    ori.displayRange=L; % plotting range
    ori.displayRangeRatio=0.1; % plotting range in the negative axis

    if plotFlag==1
        ori.Plot_UnmeshedOrigami(); % Plot the unmeshed origami for inspection;
        close;
    end
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

    if plotFlag==1
        ori.Plot_MeshedOrigami(); % Plot the meshed origami for inspection;
        close;
    end

    %% Assign Mechanical Properties

    ori.panelE=2*10^9; 
    ori.creaseE=2*10^9; 
    ori.panelPoisson=0.3;
    ori.creasePoisson=0.3; 

    A=size(ori.panel0);
    panelNum=A(2);
    ori.panelThickVec=tpanel*ones(panelNum,1); 
    ori.panelW=W;

    % set up the diagonal rate to be large to suppress crease torsion
    ori.creaseThickVec=zeros(ori.oldCreaseNum,1);
    ori.creaseThickVec(creaseIndex)=tcrease;


    %% Define Density of Origami
    ori.densityCrease=rho;
    ori.densityPanel=rho;

      
    %% Apply a NR loading to determine the axial stiffness
    nraxial=ControllerNRLoading;

    nraxial.videoOpen=0;
    nraxial.plotOpen=plotFlag;

    nraxial.increStep=3;
    
    nraxial.tol=1*10^-7; 
    
    supNode=find(ori.newNode(:,1)==0);

    nodeMax=ori.newNode(:,1);
    nodeMax=max(nodeMax);
    
    forceNode=find(ori.newNode(:,1)==nodeMax);
    
    length(supNode)
    nraxial.supp=horzcat(supNode,ones(length(supNode),3));
    
    force=1/length(forceNode);

    nraxial.load=horzcat(horzcat(forceNode,force*ones(length(forceNode),1)),...
        zeros(length(forceNode),2));


    ori.loadingController{1}={"NR",nraxial};
    ori.Solver_Solve();    

    disp= mean(abs(nraxial.Uhis(nraxial.increStep,forceNode,1)));
    axialstiff(k)=nraxial.increStep*1/disp;
    
    % Store the loaded geometry
    F = getframe(gcf);
    [X, Map] = frame2im(F); 

    name=append('Sample',string(tempSampleNum),' Axial-Deployment',string(Ext),'.png');
    imwrite(X,name)
    close;

end

%% Output of the results
outputVec=[tcrease tpanel W gama axialstiff];
output(tempSampleNum,:)=outputVec;
tempSampleNum=tempSampleNum+1;

writematrix(output,'SingleMiura.txt')

end
toc
