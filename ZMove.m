%weird ass bug, iets met machining precision, dus moest overal round
%gebruiken, heb daarom ook mijn eigen functie vind gemaakt. Hij gaf aan dat
%twee getallen niet hetzelfde waren, wat ook waar was want ze verschilden
%met 10^-18


function [xnew,znew] = ZMove(pos,x,z)
load('T')
if t>0
   xnew=pos(1)+0.02;
   t=t-1;
else
xnew=pos(1)+0.01;
end
znew=pos(2);
zsensorplus=vind(round(x(1,:),2),round(pos(1,1),2));
xplussensor=vind(round(x(1,:),2),round(pos(1,1)+0.20,2));
xminussensor=vind(round(z(1,:),2),round(pos(2,1),2));
tussen=-x(1,xminussensor)+pos(1,1);

val1=(0.20>=tussen);
val2=(tussen>0);
val=val1 & val2;

xminus=min(tussen(val));
zsensor=max(pos(2,1)-z(1,zsensorplus));
if isempty(zsensorplus)
    xnew=pos(1,1);
    znew=pos(2,1);
else
    if (zsensor<=0.45)&&(zsensor>=0.35)
        znew=pos(2)+0.4-zsensor;
    end
    if zsensor>0.45
        znew=pos(2,1)-0.05;
    end
    if zsensor<0.35
        znew=pos(2,1)+0.05;
    end
    if isempty(xplussensor)
    else
        for i=1:length(xplussensor)
            if round(z(xplussensor(i)),2)==round(pos(2,1),2)
                load('T');
                t=t+1;
                xnew=pos(1,1);
                znew=pos(2,1)+0.05;
                if isempty(find(round(z(xplussensor),2)>round(pos(2,1)+0.01,2)))
                    time=21;
                    save('Time','time');
                end
            end
        end
    end
    
    if isempty(xminussensor)
    else
       
            if round(xminus,2)<=0.18
                xnew=pos(1,1)+0.02;
                
               
            end
            if round(xminus,2)==0.19
                xnew=pos(1,1)+0.01;
            end
            if round(xminus,2)==0.2
                xnew=pos(1,1);
                load('T');
                t=t+1;
                if round(zsensor,2)==0.4
                    xnew=pos(1,1)+0.01;
                    t=t-1;
                end
            end
        
                
    end
end
save('T','t');
load('Time')
if time>0
    time=time-1;
    save('Time','time');
    xnew=pos(1,1)+0.01;
    znew=pos(2,1)+0.02;
end