%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Generate TMP Canopy (Single Sample)
%  Yi Zhu & Evgueni T. Filipov
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize the solver

clear;clc;close all;

ori=OrigamiSolver;  
ori.showNumber=0;
ori.faceColorNumbering='yellow';
ori.faceAlphaNumbering=1;

%% properties that are constant

rho=1200; % determine density
plotFlag=1; % determine if we need plotting
L=0.2; % assume the material is from a sheet with 0.2m*0.2m size 

%% Properties that are features

% design 1
% m=36;
% n=9;
% tcrease=0.7*10^-3; % crease of origami
% tpanel=4*10^-3; % thickness of panel
% W=3*10^-3; % width of crease
% creaseE=2.0*10^9; % Young's modulus of creases
% panelE=3.0*10^9; % Young's modulus of panels

% design 2 
m=30;
n=9;
tcrease=0.7*10^-3; % crease of origami
tpanel=5*10^-3; % thickness of panel
W=3*10^-3; % width of crease
creaseE=2.0*10^9; % Young's modulus of creases
panelE=3.0*10^9; % Young's modulus of panels


bendstiff=[0 0 0];
axialstiff=[0 0 0];
Extset=[0.3,0.6,0.9];

for k=1:3

    Ext=Extset(k) ;

    b=L/m;
    a=L/n;

    [ori.node0,ori.panel0]=GenerateTMPSheet(a,b,m,n,Ext);


    % Analyze the original pattern before proceeding to the next step
    ori.Mesh_AnalyzeOriginalPattern();

    % Plot the results for inspection
    ori.viewAngle1=45;
    ori.viewAngle2=45;
    ori.displayRange=L; % plotting range
    ori.displayRangeRatio=0.1; % plotting range in the negative axis
    ori.deformEdgeShow=0;
    
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
    ori.mesh2D3D=3;
    ori.Mesh_Mesh()

    if plotFlag==1
        ori.Plot_MeshedOrigami(); % Plot the meshed origami for inspection;
        close;
    end

    %% Assign Mechanical Properties

    ori.panelE=panelE; 
    ori.creaseE=creaseE; 
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

    
    %% Apply a NR loading to determine the bending stiffness
    nrbend=ControllerNRLoading;

    nrbend.videoOpen=0;
    nrbend.plotOpen=plotFlag;

    nrbend.increStep=3;
    
    nrbend.tol=1*10^-7; 
    
    
    supNode=find(ori.newNode(:,1)==min(ori.newNode(:,1)));

    nodeMax=ori.newNode(:,1);
    nodeMax=max(nodeMax);
    
    forceNode=find(ori.newNode(:,1)==nodeMax);
    
    length(supNode)
    nrbend.supp=horzcat(supNode,ones(length(supNode),3));
    
    force=1/length(forceNode);

    nrbend.load=horzcat(horzcat(forceNode,zeros(length(forceNode),2)),...
        -force*ones(length(forceNode),1));


    ori.loadingController{1}={"NR",nrbend};
    ori.Solver_Solve();    

    disp= mean(abs(nrbend.Uhis(nrbend.increStep,forceNode,3)));
    bendstiff(k)=nrbend.increStep*1/disp;
    
    
    %% Apply a NR loading to determine the axial stiffness
    nraxial=ControllerNRLoading;

    nraxial.videoOpen=0;
    nraxial.plotOpen=plotFlag;

    nraxial.increStep=3;
    
    nraxial.tol=1*10^-7; 
    
    supNode=find(ori.newNode(:,1)==min(ori.newNode(:,1)));

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
    

end
bendstiff(2)
axialstiff(3)
bendstiff(3)
