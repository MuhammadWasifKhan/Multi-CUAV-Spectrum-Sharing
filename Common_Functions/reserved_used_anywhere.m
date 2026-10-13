function tf=reserved_used_anywhere(tx,sc,b)
tf=false;
for u=1:numel(tx)
    if b==1,idx=tx(u).idxLic;else,idx=tx(u).idxUn;end
    if any(idx(:,2)==sc),tf=true;return;end
end
end
