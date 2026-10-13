function swap_verify_assignment(tx,raw)
% All initial SU associations, per-band user coverage and per-UAV SC sets
% remain unchanged. Swaps are a permutation of existing SC assignments.
for u=1:numel(raw)
    assert(isequal(tx(u).users,raw(u).users),'Association changed');
    assert(isequal(tx(u).idxLic(:,1),raw(u).idxLic(:,1)), ...
        'Licensed user assignments changed');
    assert(isequal(tx(u).idxUn(:,1),raw(u).idxUn(:,1)), ...
        'Unlicensed user assignments changed');
    assert(isequal(sort(tx(u).idxLic(:,2)),sort(raw(u).idxLic(:,2))), ...
        'Licensed channel set changed');
    assert(isequal(sort(tx(u).idxUn(:,2)),sort(raw(u).idxUn(:,2))), ...
        'Unlicensed channel set changed');
end
end


%% =========== FOUR-CASE RESERVED SUBCHANNEL CONTROLLERS ===========
