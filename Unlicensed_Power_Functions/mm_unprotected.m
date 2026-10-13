function v=mm_unprotected(h,iter)
v=ones(size(h))/sqrt(numel(h));B=h*h';
for k=1:max(1,iter)
    vNext=B*v; vNext=vNext/max(norm(vNext),realmin);
    if norm(vNext-v)<1e-8,v=vNext;break;end
    v=vNext;
end
end

