function [vor,r_lim]=power_visibility_Voronoi_v3(p,pol,r)
%[visVor]=visibilityVoronoi_r(p,pol)
%Ypologizei ta visibility-based voronoi twn kombwn p sto polygwno pol

snap_distance=0.001;
epsilon=1e-8;
[polx,poly]=poly2ccw(pol(:,1),pol(:,2));
lp=size(p,1);
vis=cell(lp,1);
%Gemizw to cell vis me tis perioxes oratothtas olwn ton kombwn
for i=1:lp
     vis(i)={visibility_polygon(p(i,:),{[polx poly]},epsilon,snap_distance)};
end
% 
% temp=vis{1};
% plot(polx,poly,'g')
% hold on
% plot(temp(:,1),temp(:,2))

%Briskw tous geitones delaunay twn kombwn kai apothikeuw 1 sth thesh DN(i,j)
%an oi komboi i,j einai geitonikoi.
DN=ones(lp);
if lp>2
    DT=DelaunayTri([p]);
    tri=DT.Triangulation;
    %Briskw apo thn trigwnopoihsh poioi komboi einai geitonikoi metaksi
    %tous.
    for i=1:size(tri,1)
        DN(tri(i,1),tri(i,2))=1;
        DN(tri(i,2),tri(i,3))=1;
        DN(tri(i,1),tri(i,3))=1;
    end            
else
    DN(1,2)=1;
end
DN=DN'|DN; %Thetw DN(i,j)==DN(j,i).


%H domh visIntersection periexei ana dyo tis tomes twn perioxwn oratothtas
%twn geitonikwn kombwn.
visIntersection=[];
for i=1:lp
    for j=i+1:lp
        vis_i=vis{i};
        vis_j=vis{j};
        if DN(i,j)==1 %An einai delaunay geitones...
            ind=sort([i,j]);
            %Elegxw an yparxei hdh sth bash
            found=false;
            if ~isempty(visIntersection)
                for k=1:length(visIntersection)
                    if all(visIntersection(k).ind==ind)
                        found=true;
                    end
                end
            end
            if found==false
                [visx,visy]=polybool('intersection',vis_i(:,1),vis_i(:,2),vis_j(:,1),vis_j(:,2));
                temp.ind=ind;
                temp.int=[visx,visy];
                visIntersection=[visIntersection;temp];
            end
        end
    end
end
% plot(polx,poly,'g')
% hold on
% for i=1:length(visIntersection)
%     col=rand(3,1);
%     ind=visIntersection(i).ind;
%     int=visIntersection(i).int;
%     if ~isempty(int)
%     patch(int(:,1),int(:,2),[col+0.9*(1-col)]')
%
%     plot(p(ind,1),p(ind,2),'*','Color',col)
%
%     end
% end

%Gia kathe stoixeio tou visIntersection ypologizw ta kelia voronoi toy kathenos 
%apo ta dio kentra ind, mesa sto polygwno int.
for i=1:length(visIntersection)
    if ~isempty(visIntersection(i).int)
        inters=visIntersection(i).int;
        vcell=power_voronoi_limited_2centers_v2(p(visIntersection(i).ind,:),pol,r,p);
        v1=vcell{1};v2=vcell{2};
        if isempty(v1)
            vor1x=[];vor1y=[];
        elseif isempty(v2)
            vor2x=[];vor2y=[];
        elseif isempty(v1) && isempty(v2)
            vor1x=[];vor1y=[];vor2x=[];vor2y=[];
        else
        [vor1x,vor1y]=polybool('intersection',inters(:,1),inters(:,2),v1(:,1),v1(:,2));
        [vor2x,vor2y]=polybool('intersection',inters(:,1),inters(:,2),v2(:,1),v2(:,2));
        end
        visIntersection(i).vor1={[vor1x,vor1y]};
        visIntersection(i).vor2={[vor2x,vor2y]};
    else
        visIntersection(i).vor1={[]};
        visIntersection(i).vor2={[]}; 
    end
end

%Oi perioxes twn kombwn
for i=1:lp
    vvor=vis{i};
    vvorx=vvor(:,1);vvory=vvor(:,2);
    for j=1:length(visIntersection)
        index=visIntersection(j).ind;
        if i==index(1)
            temp=visIntersection(j).vor2; 
        elseif i==index(2)
            temp=visIntersection(j).vor1;
        else
            temp={[]};
        end
        temp=cell2mat(temp);
        if ~isempty(temp)
            [vvorx,vvory]=polybool('subtraction',vvorx,vvory,temp(:,1),temp(:,2));
        end
    end  
    visVor(i)={[vvorx vvory]};
end

k1=1:360;
for t1=1:lp
    temp=visVor{t1}; 
    if ~isempty(temp)
    vor{t1,1}=temp(:,1);vor{t1,2}=temp(:,2);
    
    c2=r(t1)*cosd(k1)+p(t1,1);
    d2=r(t1)*sind(k1)+p(t1,2);
    [xr,yr]=polybool('&',vor{t1,1},vor{t1,2},c2,d2);
    else
        xr=[];yr=[];
    end
    r_lim{t1,1}=xr;
    r_lim{t1,2}=yr;
end













%afairw apo tis perioxes visibility-based voronoi ta mikra polygwna pou
%bgazei h polybool
% min_area=1e-3;
% for i=1:lp
%     pols={[]};
%     temp=visVor{i};
%     %arxika xwrizw to polygwno me bash ta nan
%     tempx=temp(:,1);
%     tempy=temp(:,2);
%     ind=find(isnan(tempx))';
%     ind=[0 ind length(tempx)+1];
%     for j=1:length(ind)-1;
%         pols(j)={[[tempx(ind(j)+1:ind(j+1)-1)] [tempy(ind(j)+1:ind(j+1)-1)]]};
%     end   
%     %sth analoga me to an to kathe polygwno einai mesa se kapoio allo prosthetw
%     %h afairw thn perioxh tou
%     lps=length(pols);
%     j=1;
%     while j<=lps
%         temp1=pols{j};
%         if polyarea(temp1(:,1),temp1(:,2))<min_area
%             pols(j)=[];
%             j=j-1;
%             lps=lps-1;
%         end
%         j=j+1;
%     end
%     
%     %Bazw ta kommatia ksana sth seira, kai ta xwrizw me NaN.
%     temp2=[];
%     for j=1:lps
%         temp2=[temp2;pols{j};[NaN NaN]];
%     end
%     temp2(end,:)=[];
%     visVor(i)={temp2};
% end

%
% plot(polx,poly,'g')
% axis equal
% hold on
% for i=4:4%lp
%     col=rand(3,1);
%     temp=visVor{i};
%     plot(temp(:,1),temp(:,2),'Color',col)
%     plot(p(i,1),p(i,2),'+','Color',col)
% end
% plot(p(:,1),p(:,2),'r+')
% 
%         vv1=cell2mat(visIntersection(j).vor1);
%         vv2=cell2mat(visIntersection(j).vor2);


