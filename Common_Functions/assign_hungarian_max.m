function pairs = assign_hungarian_max(score)
if isempty(score)
    pairs = zeros(0,2);
    return;
end
if exist('matchpairs','file') == 2
    cost = -score;
    bigM = 1e12;
    pairs = matchpairs(cost,bigM,'min');
    pairs = sortrows(pairs,1);
else
    pairs = assign_bruteforce(-score);
end
end
