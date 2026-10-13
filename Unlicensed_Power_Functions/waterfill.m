function p=waterfill(g,Pmax,caps)
% Correctly bracketed KKT solution for sum log2(1+g_i*p_i).
g=max(g(:),0);caps=max(caps(:),0);p=zeros(size(g));
if isempty(g)||Pmax<=0,return;end
if sum(caps)<=Pmax,p=caps;return;end
if ~any(g>0),return;end
hi=max(1,max(1./max(g,realmin)))+Pmax;lo=0;
for it=1:100
    mu=(lo+hi)/2;p=min(caps,max(0,mu-1./max(g,realmin)));
    if sum(p)>Pmax,hi=mu;else,lo=mu;end
end
p=min(caps,max(0,lo-1./max(g,realmin)));
if sum(p)>Pmax,p=p*(Pmax/sum(p));end
end

