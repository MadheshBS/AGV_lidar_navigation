% startup.m
% Automatic MATLAB environment configuration for AGV project

% Configure Simulink Data Inspector (SDI) to D:\MATLAB_TEMP
% to avoid consuming limited space on C:
temp_dir = 'D:\MATLAB_TEMP';
if ~isfolder(temp_dir)
    try
        mkdir(temp_dir);
    catch
    end
end

if isfolder(temp_dir)
    try
        Simulink.sdi.setStorageLocation(temp_dir);
        Simulink.sdi.setArchiveRunLimit(0);
        Simulink.sdi.clear;
    catch
    end
end
