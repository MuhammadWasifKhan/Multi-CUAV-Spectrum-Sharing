function v = mrt_unit(h)
nh = norm(h);
if nh < 1e-30
    v = ones(numel(h),1)/sqrt(numel(h));
else
    v = h/nh;
end
end
