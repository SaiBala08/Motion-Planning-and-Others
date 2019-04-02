%% Load and display robot

robo = importrobot('/chaitanya_model/Assem Baxter Arm/urdf/Assem Baxter Arm (1).urdf')
axes = show(robo,'Frames','off','Visuals',"off");
axes.CameraPositionMode = 'auto';
axis([-2,2,-2,2,-2,2])


%% Create a smooth curve from the waypoints to serve as trajectory

%wayPoints = [0.4 0.4 0.4; 0.4 0.4 0;0.4 -0.4 0;0.4 -0.4 0.4;0.4 0.4 0.4];
wayPoints = [-0.5 -0.8 -0.2 ; -0.4 -0.8 -0.1 ; -0.3 -0.8 0 ; -0.1 -0.8 0.1 ; 0.1 -0.8 0.2 ; 0.4 -0.8 0.3];

trajectory = cscvn(wayPoints');

% Plot trajectory spline and waypoints
hold on
fnplt(trajectory,'r',2);

No.ofPoints = 30;
% % Evaluate trajectory to create a vector of end-effector positions
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),No.ofPoints));

%% Perform Forward Kinematics for a point in space

%Add end effector frame

eeBody = robotics.RigidBody('Link_7');
setFixedTransform(eeBody.Joint,trvec2tform([0.35 0.30 0]));
addBody(robo,eeBody,'Link_6');

% Declairing IK variable.
ik = robotics.InverseKinematics('RigidBodyTree',robo);

EE_Position = [0.400000000000000 -0.493416192542374 0.163391951226152];
initialguess = robo.homeConfiguration;
Homo_Transform = trvec2tform(EE_Position);
weights = [0 0 0 1 1 1];


for i = 1:size(eePositions,2)
    Homo_Transform = trvec2tform(eePositions(:,i)');
    Joint_Values(i,:) = ik('Link_7',Homo_Transform,weights,initialguess);
    initialguess = Joint_Values(i,:);
end


%% Visualize robot configurations

axis([-2,2,-2,2,-2,2])
for i = 1:size(eePositions,2)
    show(robo,Joint_Values(i,:), 'PreservePlot', false,'Frames',"on");
    pause(0.1)
end

