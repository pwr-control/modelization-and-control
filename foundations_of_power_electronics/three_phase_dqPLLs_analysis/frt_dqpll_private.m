function prv = frt_dqpll_private()
%FRT_DQPLL_PRIVATE  Handles to the functions of ./private, for the live script init_model.m.
%
%   prv = frt_dqpll_private()   one function handle per file of ./private: prv.library_setup,
%                               prv.timing_param, prv.grid_param, prv.frt_param, prv.afe_hw_param, ...
%
%   Same file as mpc_psm/mpc_psm_private.m and ps_25V_12kA/ps_25V_12kA_private.m. The Live Editor runs a live
%   script from a temporary copy outside this folder, so ./private is out of scope there ("Unrecognized function
%   or variable 'timing_param'"). A function of this folder does see ./private, and a function handle keeps the
%   scope in which it was made: init_model.m calls prv.<name>(...) and runs the same in the Live Editor and from
%   the Command Window. A section run on its own needs prv in the workspace (the first section makes it). This
%   file stays outside ./private on purpose: the Live Editor finds it in the current folder or on the path, as it
%   finds the model.

    files = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
    prv = struct();
    for k = 1:numel(files)
        [~, name] = fileparts(files(k).name);
        prv.(name) = eval(['@' name]);      % made in this file: the handle points to ./private/<name>.m
    end
end
