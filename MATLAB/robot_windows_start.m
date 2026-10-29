function robot_windows_start(readyFile, errorFile, stopFile)
% Dedicated process entrypoint. Errors propagate to the WSL launcher.
setappdata(0, 'RobotSimulationReadyFile', readyFile);
setappdata(0, 'RobotSimulationStopFile', stopFile);
try
    % The existing script clears its base workspace. Bootstrap state stays
    % here and in process-local root appdata, rather than that workspace.
    evalin('base', 'can_robot_visualizer');
catch err
    fid = fopen(errorFile, 'w');
    if fid ~= -1
        fprintf(fid, '%s', getReport(err, 'extended', 'hyperlinks', 'off'));
        fclose(fid);
    end
    rethrow(err);
end
end
