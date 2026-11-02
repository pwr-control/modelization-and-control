function prv = single_phase_dab_private()
%SINGLE_PHASE_DAB_PRIVATE  Handles to the functions of ./private, for the live scripts of this folder.
%
%   prv = single_phase_dab_private()   one function handle per file of ./private: prv.library_setup,
%                                      prv.timing_param, prv.dab_hw_param, prv.device_mosfet_param, ...
%
%   Same idea as ../three_phase_dab/dab3_private.m. The Live Editor runs a live script (init_model.m) from a
%   temporary copy outside this folder, so ./private is out of scope there ("Unrecognized function or variable
%   'library_setup'"). A function of this folder does see ./private, and a function handle keeps the scope in
%   which it was made: init_model.m calls prv.<name>(...) and runs the same in the Live Editor and from the
%   Command Window; a section run on its own needs prv in the workspace (the first section makes it). This file
%   stays outside ./private on purpose: the Live Editor finds it in the current folder or on the path, as it
%   finds the model.

    files = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
    prv = struct();
    for k = 1:numel(files)
        [~, name] = fileparts(files(k).name);
        prv.(name) = eval(['@' name]);      % made in this file: the handle points to ./private/<name>.m
    end
end
