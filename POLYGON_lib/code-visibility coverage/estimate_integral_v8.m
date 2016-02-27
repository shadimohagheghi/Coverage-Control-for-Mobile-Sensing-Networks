function F1=estimate_integral_v8(points_x,points_y,th1,r,A,B,j,x,y)
        
 
        M=sqrt((A(1)-x(j))^2+(A(2)-y(j))^2);%length of edge AB 
        [L,B]=line_equation_v2(points_x,points_y);...
        
        dq=2*pi*r(j)/360; %ston kuklo exw dtheta=1 moira to opoio se mikos metfrazetai ws 1moira*2*pi*r/360
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
            min1=min(points_x);max1=max(points_x);    
            X=min1:dq:max1;...
            Y=L*X+B;...
        end
    
        t=length(X);...
        if length(t)<=3
        dq=dq/10;
        X=min1:dq:max1;...
        Y=L*X+B;...
         t=length(X);...
        end
        F1=0+0*1i;...
        for g=1:t
            d(g)=sqrt((X(g)-A(1))^2+(Y(g)-A(2))^2);
            xk=X(g)+(d(g)/(M^2))*cos(th1);yk=Y(g)+(d(g)/(M^2))*sin(th1);... %prepei na allaksw to 1
            Ni=(xk-X(g))+(yk-Y(g))*1i;...%Ni=-ni sto shmeio g
            F1=F1+Ni;
        end
     


return