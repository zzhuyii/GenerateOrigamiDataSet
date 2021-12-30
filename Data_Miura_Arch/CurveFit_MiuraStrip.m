%% Fit a curve using the Miura Strip

function [node, panel, error] = CurveFit_MiuraStrip(m,offset,w,extrude)

    % the curve we want to fit 
    % t in (0,1)
    t=[0:1:(2*m)]'/2/m;
    xcord=X(t);
    zcord=Z(t);
    
%     plot(xcord,zcord)

    % create the zigzag center line
    for i =1:m

        tang=([xcord(2*i+1);zcord(2*i+1);]-[xcord(2*i-1);zcord(2*i-1);]);
        tang=tang/norm(tang);        
        vec=[-tang(2);tang(1)];
        vec=vec/norm(vec);

        xcord(2*i)=xcord(2*i)+vec(1)*offset;
        zcord(2*i)=zcord(2*i)+vec(2)*offset;   
    end
% 
%     plot(xcord,zcord)
%     set(gca,'DataAspectRatio',[1 1 1])
%     hold on

    xcord1=xcord;
    zcord1=zcord;

    % Compute the direction vector for finding the angle bisector
    for i=1:(2*m-1)
        vec1=([xcord(i+1);zcord(i+1);]-[xcord(i);zcord(i);]);
        vec2=([xcord(i+2);zcord(i+2);]-[xcord(i+1);zcord(i+1);]);
        vec1=vec1/norm(vec1);
        vec2=vec2/norm(vec2);

        vec=(vec1+vec2)/2;

        bisec=acos(dot(vec,vec2)/norm(vec)/norm(vec2));

        h=w/sin(bisec);
        xcord1(i+1)=xcord1(i+1)+h*vec(1);
        zcord1(i+1)=zcord1(i+1)+h*vec(2);   

    end

    % finde the first and final node
    vechead=[xcord(2);zcord(2);]-[xcord(1);zcord(1);];
    vechead2(2)=-vechead(1);
    vechead2(1)=vechead(2);
    vechead2=vechead2/norm(vechead2);

    xcord1(1)=xcord1(1)+w*vechead2(1);
    zcord1(1)=zcord1(1)+w*vechead2(2);   

    vectail=[xcord(2*m+1);zcord(2*m+1);]-[xcord(2*m);zcord(2*m);];
    vectail2(2)=vectail(1);
    vectail2(1)=-vectail(2);
    vectail2=vectail2/norm(vectail2);

    xcord1(2*m+1)=xcord1(2*m+1)+w*vectail2(1);
    zcord1(2*m+1)=zcord1(2*m+1)+w*vectail2(2);   

%     plot(xcord1,zcord1)
%     hold off

    % Generate teh nodal coordinates
    xcord2=xcord1;
    zcord2=zcord1;

    ycord=zeros(2*m+1,1);
    ycord1=-extrude*ones(2*m+1,1);
    ycord2=extrude*ones(2*m+1,1);

    x=[xcord;xcord1;xcord2];
    y=[ycord;ycord1;ycord2];
    z=[zcord;zcord1;zcord2];

    node=[x,y,z];

%     figure
%     scatter3(x,y,z);
%     set(gca,'DataAspectRatio',[1 1 1])

    % set up the panel connectivity
    tempPanelNum=1;
    panel={};
    for i=1:2*m
        panel{tempPanelNum}=[i,2*m+1+i,2*m+1+i+1,i+1];
        tempPanelNum=tempPanelNum+1;
        panel{tempPanelNum}=[i,4*m+2+i,4*m+2+i+1,i+1];
        tempPanelNum=tempPanelNum+1;    
    end
    
    
    % caculate the error between the target shape and the origami
    seg=10;
    t=[1:seg*2*m]/(seg*2*m);
    targetX=X(t);
    targetZ=Z(t);
    
    targetX=targetX';
    targetZ=targetZ';
    
    oriFitX=zeros(seg*2*m,1);
    oriFitZ=zeros(seg*2*m,1);
    
    for i=1:2*m
        node1=[xcord(i);zcord(i)];
        node2=[xcord(i+1);zcord(i+1)];
        
        for j=1:seg
            oriFitX((i-1)*seg+j)=node1(1)*(seg-j)/seg+node2(1)*(j)/seg;
            oriFitZ((i-1)*seg+j)=node1(2)*(seg-j)/seg+node2(2)*(j)/seg;
        end
        
    end
    
    error=(oriFitX-targetX).^2+(oriFitZ-targetZ).^2;
    error=sqrt(error);
    error=mean(error,'all');

end


%% the curve we want to fit
% arch shape
function x=X(t)
    x=2-2*cos(t*pi);
end

function z=Z(t)
    z=2*sin(t*pi);
end

% Sin function
% function x=X(t)
%     x=t*4;
% end
% 
% function z=Z(t)
%     z=sin(6*t);
% end

% saw function
% function x=X(t)
%     x=t*4;
% end
% 
% function z=Z(t)
%     z=2-2*abs(t-0.5);
% end
