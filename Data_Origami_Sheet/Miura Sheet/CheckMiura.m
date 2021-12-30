% Yes the Yoshimura pattern generation code works fine
% Though it may not be suitable for making the metamaterial sheet
% We need something else.

ori=OrigamiSolver;   

plotFlag=1;
Ext=0.9;
m=6;
n=6;
L=0.2;

gamma=70*pi/180;

b=L/m;
a=L/n;

[ori.node0,ori.panel0]=GenerateMiuraSheet(a,b,gamma,m,n,Ext);


% Analyze the original pattern before proceeding to the next step
ori.Mesh_AnalyzeOriginalPattern();

   
ori.showNumber=0;
ori.faceColorNumbering='yellow';
ori.faceAlphaNumbering=1;

% Plot the results for inspection
ori.viewAngle1=45;
ori.viewAngle2=45;
ori.displayRange=L; % plotting range
ori.displayRangeRatio=0.1; % plotting range in the negative axis

if plotFlag==1
    ori.Plot_UnmeshedOrigami(); % Plot the unmeshed origami for inspection;
end

node=ori.node0;


norm(node(12,:)-node(21,:))
sqrt(a*a/4+b*b)


norm(node(20,:)-node(21,:))
a