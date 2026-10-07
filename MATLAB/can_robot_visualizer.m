%% LIVE CANOPEN / AVATAR VISUALIZER
%
% Receives the versioned Supervisor/HMI status packet from the
% FreeRTOS CANopen / AVATAR simulation over UDP and displays the robot
% using the project's existing UR5 visual mesh model.

clear;
clc;
close all;

%% ========================================================================
%  1. ADD PROJECT ROOT TO MATLAB PATH
% =========================================================================

thisFileFolder = fileparts(mfilename("fullpath"));
repoRoot = fileparts(thisFileFolder);
addpath(repoRoot);

fprintf("Repository root:\n%s\n\n", repoRoot);

%% ========================================================================
%  2. UDP RECEIVER
% =========================================================================

PORT = 5005;

udp = udpport( ...
    "datagram", ...
    "IPV4", ...
    "LocalPort", PORT);

fprintf( ...
    "Listening for CANopen Supervisor telemetry on UDP port %d...\n", ...
    PORT);

%% ========================================================================
%  3. LOAD PROJECT ROBOT CONFIGURATION
% =========================================================================

robot = config.UR5();

%% ========================================================================
%  4. BUILD UR5 VISUAL ROBOT
% =========================================================================

visualRobot = robotmodel.buildUR5geometry(robot);

%% ========================================================================
%  5. INITIAL JOINT CONFIGURATION
% =========================================================================

q = zeros(robot.dof, 1);

%% ========================================================================
%  6. CREATE 3-D FIGURE
% =========================================================================

fig = figure( ...
    "Name", "Live CANopen / AVATAR Robot", ...
    "NumberTitle", "off", ...
    "Color", [0.12 0.12 0.12], ...
    "Renderer", "opengl");

ax = axes("Parent", fig);

hold(ax, "on");
grid(ax, "on");
axis(ax, "equal");
axis(ax, "vis3d");

ax.Projection = "perspective";
view(ax, 135, 25);

ax.Color = [0.12 0.12 0.12];
ax.XColor = [0.85 0.85 0.85];
ax.YColor = [0.85 0.85 0.85];
ax.ZColor = [0.85 0.85 0.85];
ax.GridColor = [0.65 0.65 0.65];
ax.GridAlpha = 0.30;

xlabel(ax, "X [m]");
ylabel(ax, "Y [m]");
zlabel(ax, "Z [m]");

rotate3d(fig, "on");

%% ========================================================================
%  7. INITIAL ROBOT DRAW
% =========================================================================

show( ...
    visualRobot, ...
    q, ...
    "Parent", ax, ...
    "Frames", "off", ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "PreservePlot", false);

title( ...
    ax, ...
    "Waiting for CANopen / AVATAR feedback...", ...
    "Color", [0.95 0.95 0.95]);

%% ========================================================================
%  8. IMPROVE 3-D MESH RENDERING
% =========================================================================

meshObjects = findobj(ax, "Type", "Patch");

for k = 1:numel(meshObjects)
    meshObjects(k).EdgeColor = "none";
    meshObjects(k).FaceLighting = "gouraud";
end

camlight(ax, "headlight");
camlight(ax, "right");
lighting(ax, "gouraud");
material(ax, "dull");

xlim(ax, [-1.0  0.4]);
ylim(ax, [-0.8  0.8]);
zlim(ax, [-0.5  1.1]);

drawnow;

%% ========================================================================
%  9. CAN SUPERVISOR STATUS PACKET
% =========================================================================
%
% The controller sends the same HMI status packet to MATLAB:
%
%       57 x uint32 = 228 bytes
%
% Network byte order is used.
%
% Important MATLAB 1-based word positions:
%
%       1       magic = 0x53544D33 ("STM3")
%       2       protocol version = 3
%       3       sequence
%       4       robot state
%       35:37   TCP X/Y/Z as IEEE-754 float bits
%       38:43   q1...q6 as IEEE-754 float bits [rad]
%       44      CAN ready nodes
%       45      CAN expected nodes
%

PACKET_WORDS = 57;
PACKET_BYTES = PACKET_WORDS * 4;
STATUS_MAGIC = uint32(hex2dec("53544D33"));
PROTOCOL_VERSION = uint32(3);

stateNames = [ ...
    "BOOT", ...
    "HOMING", ...
    "IDLE", ...
    "TEACHING", ...
    "PATH VALIDATION", ...
    "APPROACH", ...
    "PATH EXECUTION", ...
    "ARC STABILIZING", ...
    "WELDING", ...
    "PAUSED", ...
    "RETRACTING", ...
    "FAULT", ...
    "EMERGENCY STOP" ...
];

%% ========================================================================
%  10. LIVE TELEMETRY LOOP
% =========================================================================

fprintf("\n");
fprintf("========================================\n");
fprintf("LIVE CANOPEN / AVATAR VISUALIZATION\n");
fprintf("========================================\n");
fprintf("Waiting for Supervisor status packets...\n");
fprintf("Close the figure to stop visualization.\n");
fprintf("========================================\n\n");

while isvalid(fig)

    if udp.NumDatagramsAvailable > 0

        packets = read( ...
            udp, ...
            udp.NumDatagramsAvailable, ...
            "uint8");

        bytes = uint8(packets(end).Data);

        if numel(bytes) ~= PACKET_BYTES
            fprintf( ...
                "Unexpected packet size: %d bytes (expected %d)\n", ...
                numel(bytes), ...
                PACKET_BYTES);
            continue;
        end

        %% ----------------------------------------------------------------
        % NETWORK BYTES -> 57 uint32 WORDS
        % -----------------------------------------------------------------

        byteMatrix = reshape(bytes(:), 4, []);
        byteMatrix = flipud(byteMatrix);

        words = typecast( ...
            byteMatrix(:), ...
            "uint32");

        if ...
            words(1) ~= STATUS_MAGIC || ...
            words(2) ~= PROTOCOL_VERSION

            fprintf("Ignored packet with wrong magic/version.\n");
            continue;
        end

        sequence = words(3);
        robotState = double(words(4));

        %% ----------------------------------------------------------------
        % EXTRACT TCP AND JOINT FEEDBACK
        % -----------------------------------------------------------------

        tcp = double( ...
            typecast( ...
                uint32(words(35:37)), ...
                "single"));

        q = double( ...
            typecast( ...
                uint32(words(38:43)), ...
                "single"));

        q = q(:);

        canReady = double(words(44));
        canExpected = double(words(45));

        %% ----------------------------------------------------------------
        % UPDATE ROBOT
        % -----------------------------------------------------------------

        show( ...
            visualRobot, ...
            q, ...
            "Parent", ax, ...
            "Frames", "off", ...
            "Visuals", "on", ...
            "Collisions", "off", ...
            "FastUpdate", true, ...
            "PreservePlot", false);

        %% ----------------------------------------------------------------
        % STATUS TITLE
        % -----------------------------------------------------------------

        qDeg = rad2deg(q);

        if ...
            robotState >= 0 && ...
            robotState < numel(stateNames)

            stateText = stateNames(robotState + 1);
        else
            stateText = "UNKNOWN";
        end

        titleText = sprintf( ...
            ['CANOPEN / AVATAR FEEDBACK\n' ...
             'State: %s   CAN: %d/%d nodes   Seq: %u\n' ...
             'TCP [%.3f %.3f %.3f] m\n' ...
             'J1 %.2f°   J2 %.2f°   J3 %.2f°   ' ...
             'J4 %.2f°   J5 %.2f°   J6 %.2f°'], ...
            stateText, ...
            canReady, ...
            canExpected, ...
            sequence, ...
            tcp(1), ...
            tcp(2), ...
            tcp(3), ...
            qDeg(1), ...
            qDeg(2), ...
            qDeg(3), ...
            qDeg(4), ...
            qDeg(5), ...
            qDeg(6));

        title( ...
            ax, ...
            titleText, ...
            "Color", [0.95 0.95 0.95]);

        fprintf( ...
            ['%-15s CAN=%d/%d  ' ...
             'J=[%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f] deg\n'], ...
            stateText, ...
            canReady, ...
            canExpected, ...
            qDeg(1), ...
            qDeg(2), ...
            qDeg(3), ...
            qDeg(4), ...
            qDeg(5), ...
            qDeg(6));

        drawnow limitrate;

    else
        pause(0.005);
    end
end

%% ========================================================================
%  11. CLEANUP
% =========================================================================

clear udp;

fprintf("\nVisualizer closed.\n");
