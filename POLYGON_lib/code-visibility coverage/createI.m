function I=createI(j,N)
     if abs(j-1)<=0
         I(1:N-1)=(j+1):N;
     elseif abs(j-N)<=0
         I(1:N-1)=1:(N-1);
     else
         I1=1:j-1;
         I2=j+1:N;
         I=[I1 I2];
     end
     if abs(N-1)<=0
         I=[];
     end
     return