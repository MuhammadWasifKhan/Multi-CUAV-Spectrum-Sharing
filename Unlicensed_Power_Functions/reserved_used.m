function used=reserved_used(tx,sc,band)
used=false;
for u=1:numel(tx)
    [idx]=band_data(tx,u,band);
    if any(idx(:,2)==sc),used=true;return;end
end
end
