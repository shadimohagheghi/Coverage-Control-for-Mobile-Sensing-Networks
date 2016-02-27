function F1=estimate_integral(points_x,points_y,th1,r)

        [L,B]=line_equation_v2(points_x,points_y);...
        
        dq=2*pi*r(1)/360; %ston kuklo exw dtheta=1 moira to opoio se mikos metfrazetai ws 1moira*2*pi*r/360
        %dq=0.01  palia
        if abs(L)<=0
        %L=0 ==> y=stathero
            min1=min(points_x);max1=points_x;    
            X=min1:dq:max1;
            Y(1:length(X))=points_y(1);...
        elseif isinf(L)
        %L=inf ==> x=stathero
            min1=min(points_y);max1=points_y;    
            Y=min1:dq:max1;
            X(1:length(Y))=points_x(1);...
        
        else
            min1=min(points_x);max1=points_x;    
            X=min1:dq:max1;...
            Y=L*X+B;...
        end
    
        t=length(X);...
        F1=0+0*1i;...
        for g=1:t
            xk=X(g)+1*cos(th1);yk=Y(g)+1*sin(th1);... %prepei na allaksw to 1
            Ni=(xk-X(g))+(yk-Y(g))*1i;...%Ni=-ni sto shmeio g
            F1=F1+Ni;
        end
     


return