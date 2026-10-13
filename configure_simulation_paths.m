function configure_simulation_paths(projectRoot, mode)
% Configure only the selected experiment's dependencies, without resetting CVX.
% MATLAB resolves duplicate function names in path order. Convergence overrides
% must precede Common_Functions; the other experiments must not see overrides.
if nargin<2, mode='common'; end
names={'Common_Functions','Convergence_Overrides','Cases_Functions','Unlicensed_Power_Functions','Original_Scripts'};
for k=1:numel(names)
    d=fullfile(projectRoot,names{k});
    if isfolder(d)
        pathParts=strsplit(path,pathsep);
        if any(strcmpi(pathParts,d))
            rmpath(d);
        end
    end
end
switch lower(mode)
    case 'unlicensed'
        selected={'Unlicensed_Power_Functions'};
    case 'convergence'
        selected={'Convergence_Overrides','Common_Functions'};
    case 'cases'
        selected={'Cases_Functions','Common_Functions'};
    otherwise
        selected={'Common_Functions'};
end
% addpath(...,'-begin') prepends entries: iterate backwards.
for k=numel(selected):-1:1
    d=fullfile(projectRoot,selected{k});
    assert(isfolder(d),'Required folder missing: %s',d);
    addpath(d,'-begin');
end
if exist('cvx_begin','file')~=2
    warning('CVX is not on the MATLAB path. Initialize C:\cvx\cvx_setup.m before running CVX experiments.');
end
end
