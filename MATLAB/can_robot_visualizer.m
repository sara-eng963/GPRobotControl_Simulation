function can_robot_visualizer()
%CAN_ROBOT_VISUALIZER Live AVATAR/CANopen robot visualization.
%
% Listens to the CAN Supervisor simulation status stream on UDP port 5005.
% The packet is the same versioned HMI status packet used by the operator UI.
%
% Run this file in MATLAB, then start supervisor_mock_sim in WSL with
% MATLAB_IP set to the Windows host address.

PORT = 5005;
PACKET_WORDS = 57;
PACKET_BYTES = 4 * PACKET_WORDS;
MAGIC = uint32(hex2dec('53544D33')); % "STM3"
VERSION = uint32(3);

if exist("udpport","file") ~= 2
    error(["udpport() is unavailable. This visualizer requires a MATLAB " ...
           "release/toolbox that provides udpport."]);
end

u = udpport("byte","IPV4","LocalPort",PORT,"Timeout",0.05);
cleanupUdp = onCleanup(@() deleteUdp(u)); %#ok<NASGU>

fprintf("CAN robot visualizer listening on UDP %d...\n", PORT);

stateNames = [ ...
    "BOOT","HOMING","IDLE","TEACHING","PATH VALIDATION","APPROACH", ...
    "PATH EXECUTION","ARC STABILIZING","WELDING","PAUSED", ...
    "RETRACTING","FAULT","EMERGENCY STOP"];

% Standard-DH reference model used by ControlCore robot_config_init_ur5().
a = [0, -0.425, -0.39225, 0, 0, 0];
d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

fig = figure( ...
    "Name","CANopen / AVATAR Whole-Pipeline Visualizer", ...
    "NumberTitle","off", ...
    "Color","w");

layout = tiledlayout(fig,2,2,"TileSpacing","compact","Padding","compact");

axRobot = nexttile(layout,[2 1]);
axis(axRobot,"equal");
grid(axRobot,"on");
hold(axRobot,"on");
view(axRobot,135,25);
xlabel(axRobot,"X (m)");
ylabel(axRobot,"Y (m)");
zlabel(axRobot,"Z (m)");
xlim(axRobot,[-1 1]);
ylim(axRobot,[-1 1]);
zlim(axRobot,[-0.2 1.2]);
title(axRobot,"Robot pose");

robotLine = plot3(axRobot,nan,nan,nan,"-o","LineWidth",2,"MarkerSize",5);
tcpTrail = animatedline(axRobot,"LineWidth",1.5);

axJoints = nexttile(layout);
grid(axJoints,"on");
hold(axJoints,"on");
xlabel(axJoints,"Time (s)");
ylabel(axJoints,"Joint angle (deg)");
title(axJoints,"Joint feedback");
jointLines = gobjects(1,6);
for k = 1:6
    jointLines(k) = animatedline(axJoints,"DisplayName",sprintf("J%d",k));
end
legend(axJoints,"Location","eastoutside");

axTcp = nexttile(layout);
grid(axTcp,"on");
hold(axTcp,"on");
xlabel(axTcp,"Time (s)");
ylabel(axTcp,"TCP position (m)");
title(axTcp,"TCP feedback");
tcpLines = gobjects(1,3);
labels = ["X","Y","Z"];
for k = 1:3
    tcpLines(k) = animatedline(axTcp,"DisplayName",labels(k));
end
legend(axTcp,"Location","eastoutside");

t0 = tic;
lastSequence = uint32(0);

while isvalid(fig)
    while u.NumBytesAvailable >= PACKET_BYTES
        % If MATLAB fell behind, keep only the newest complete packet.
        packetCount = floor(u.NumBytesAvailable / PACKET_BYTES);
        raw = read(u,PACKET_BYTES * packetCount,"uint8");
        raw = uint8(raw(end-PACKET_BYTES+1:end));

        bytes = reshape(raw,4,[]);
        bytes = flipud(bytes);
        words = typecast(bytes(:),"uint32");

        if words(1) ~= MAGIC || words(2) ~= VERSION
            continue;
        end

        sequence = words(3);
        if sequence == lastSequence
            continue;
        end
        lastSequence = sequence;

        stateId = double(words(4));
        canReady = double(words(44));
        canExpected = double(words(45));

        q = double(typecast(uint32(words(38:43)),"single"));
        tcp = double(typecast(uint32(words(35:37)),"single"));

        t = toc(t0);

        points = fkPoints(q,a,d,alpha);
        set(robotLine, ...
            "XData",points(1,:), ...
            "YData",points(2,:), ...
            "ZData",points(3,:));

        addpoints(tcpTrail,tcp(1),tcp(2),tcp(3));

        for k = 1:6
            addpoints(jointLines(k),t,rad2deg(q(k)));
        end

        for k = 1:3
            addpoints(tcpLines(k),t,tcp(k));
        end

        if stateId >= 0 && stateId < numel(stateNames)
            stateText = stateNames(stateId+1);
        else
            stateText = "UNKNOWN";
        end

        title(axRobot,sprintf( ...
            "State: %s   |   CAN: %d/%d nodes   |   seq %u", ...
            stateText,canReady,canExpected,sequence));

        % Keep time plots readable during long runs.
        if t > 20
            xlim(axJoints,[t-20,t]);
            xlim(axTcp,[t-20,t]);
        end

        drawnow limitrate;
    end

    pause(0.01);
end
end


function points = fkPoints(q,a,d,alpha)
T = eye(4);
points = zeros(3,7);

for i = 1:6
    th = q(i);
    ct = cos(th);
    st = sin(th);
    ca = cos(alpha(i));
    sa = sin(alpha(i));

    A = [ ...
        ct, -st*ca,  st*sa, a(i)*ct; ...
        st,  ct*ca, -ct*sa, a(i)*st; ...
         0,     sa,     ca,      d(i); ...
         0,      0,      0,         1];

    T = T * A;
    points(:,i+1) = T(1:3,4);
end
end


function deleteUdp(u)
try
    delete(u);
catch
end
end
