function [A,B]=return_A(points_x,points_y,x,y,j)
        xa=points_x(1);ya=points_y(1);
        xb=points_x(2);yb=points_y(2);
        d1=sqrt((xa-x(j))^2+(ya-y(j))^2);
        d2=sqrt((xb-x(j))^2+(yb-y(j))^2);
        [mind,index]=min([d1;d2]);
        if index==1
            A=[xa;ya];
            B=[xb;yb];
        else
            A=[xb;yb];
            B=[xa;ya];
        end