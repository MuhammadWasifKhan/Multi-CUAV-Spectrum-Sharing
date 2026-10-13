function Q=initial_path(starts,hover,N,step,area,Dmin)
U=size(starts,1); Q=zeros(U,N,2);
for u=1:U
    a=starts(u,:); d=hover(u,:)-a;
    maxOut=0.45*(N-1)*step; d=d*min(1,maxOut/max(norm(d),1e-12));
    for n=1:N
        v=(n-1)/(N-1); scale=min([1,2*v,2*(1-v)]);
        Q(u,n,:)=a+scale*d;
    end
end
Q=repair_path(Q,starts,step,Dmin,area);
end

