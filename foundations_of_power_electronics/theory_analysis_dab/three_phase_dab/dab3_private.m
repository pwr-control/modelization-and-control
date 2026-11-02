function prv = dab3_private()
%DAB3_PRIVATE  Handles to the functions of ./private, for the live scripts of this folder.
%
%   prv = dab3_private()   one function handle per file of ./private: prv.library_setup, prv.dab3_param,
%                          prv.dab3_ideal_op, prv.dab3_harmonic_op, prv.dab3_edges, prv.dab3_isw, ...
%
%   The Live Editor runs a live script (init_model.m, dab3_analysis.m, dab3_run_check.m) from a temporary copy
%   outside this folder, so ./private is out of scope there: "Unrecognized function or variable
%   'library_setup'". A function of this folder does see ./private, and a function handle keeps the scope in
%   which it was made: the scripts call prv.<name>(...) and run the same in the Live Editor and from the
%   Command Window; a section run on its own needs prv in the workspace (the first section makes it). This file
%   stays outside ./private on purpose: the Live Editor finds it in the current folder or on the path, as it
%   finds the model.
%   A function of the library that must be frozen or changed for this model only goes in ./private with the
%   same name and the scripts call it as prv.<name>(...).

    files = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
    prv = struct();
    for k = 1:numel(files)
        [~, name] = fileparts(files(k).name);
        prv.(name) = eval(['@' name]);      % made in this file: the handle points to ./private/<name>.m
    end
end
