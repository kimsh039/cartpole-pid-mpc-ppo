function paths = project_paths()
% PROJECT_PATHS Resolve and create the repository's standard directories.

paths.Matlab = fileparts(mfilename("fullpath"));
paths.Root = fileparts(paths.Matlab);
paths.Models = fullfile(paths.Root,"models");
paths.Results = fullfile(paths.Root,"results");
paths.Trajectories = fullfile(paths.Results,"trajectories");
paths.Tables = fullfile(paths.Results,"tables");
paths.Images = fullfile(paths.Root,"docs","images");
paths.Media = fullfile(paths.Root,"docs","media");

folders = [ ...
    string(paths.Models), ...
    string(paths.Trajectories), ...
    string(paths.Tables), ...
    string(paths.Images), ...
    string(paths.Media) ...
];

for folder = folders
    if ~isfolder(folder)
        mkdir(folder);
    end
end
end
