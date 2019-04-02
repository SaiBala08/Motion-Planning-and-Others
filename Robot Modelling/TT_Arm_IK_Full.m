%% Load and display robot

robo = importrobot('/chaitanya_model/Assem Baxter Arm/urdf/Assem Baxter Arm (1).urdf')
axes = show(robo,'Frames','off','Visuals',"off");
axes.CameraPositionMode = 'auto';
axis([-1,1,-1,1,-1,1])
%% Create a smooth curve from the waypoints to serve as trajectory

wayPoints = [0.3 0.4 0.7; 0.3 0.4 0.3;0.3 -0.4 0.3;0.3 -0.4 0.7;0.3 0.4 0.7];
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
initialguess = robo.homeConfiguration;
weights = [0 0 0 1 1 1];

axis([-1,1,-1,1,-1,1])
for i = 1:size(eePositions,2)
    Homo_Transform = trvec2tform(eePositions(:,i)');
    Joint_Values = ik('Link_7',Homo_Transform,weights,initialguess);
    show(robo,Joint_Values, 'PreservePlot', false,'Frames',"off");
    initialguess = Joint_Values;
    pause(0.1)
end
hold off
