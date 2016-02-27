function same=vectorSame(vectx,vect)
%function same=vectorSame(vectx,vect)
%The function returns the elements of vector vectx that are same in vect.

i=1;
vectx=unique(vectx);
vect=unique(vect);
l1=length(vect);
l2=length(vectx);
same=[];%Τα όμοια στοιχεία
while i<=l1
    j=1;
    while j<=l2
       if vectx(j)==vect(i)
           same=[same vectx(j)];
       end
       j=j+1;
    end
    i=i+1;
end
