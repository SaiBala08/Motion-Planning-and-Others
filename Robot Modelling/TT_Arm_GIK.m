%% Load and display robot

robo = importrobot('/chaitanya_model/Assem Baxter Arm/urdf/Assem Baxter Arm Final.urdf')
axes = show(robo,'Frames',"off",'Visuals','off');
axes.CameraPositionMode = 'auto';
axis([-1,1,-1,1,-1,1])
%% Create a smooth curve from the waypoints to serve as trajectory

wayPoints = [0.3 0.4 0.7; 0.3 0.4 0.3;0.3 -0.4 0.3;0.3 -0.4 0.7;0.3 0.4 0.7];
%wayPoints = [-0.5 -0.8 -0.2 ; -0.4 -0.8 -0.1 ; -0.3 -0.8 0 ; -0.1 -0.8 0.1 ; 0.1 -0.8 0.2 ; 0.4 -0.8 0.3];
trajectory = cscvn(wayPoints');

hold on
fnplt(trajectory,'r',2);
No.ofPoints = 30;
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),No.ofPoints));
%% Perform Inverse Kinematics for a point in space
%Add end effector frame

eeBody = robotics.RigidBody('Link_7');
setFixedTransform(eeBody.Joint,trvec2tform([0.35 0.30 0]));
addBody(robo,eeBody,'Link_6');

% Declairing Generalized IK variable.
gik = robotics.GeneralizedInverseKinematics('RigidBodyTree',robo);
gik.ConstraintInputs = {'position','aiming'};

posTgt = robotics.PositionTarget('Link_7');
aimCon = robotics.AimingConstraint('Link_7');
initialguess = robo.homeConfiguration;
 

for i = 1:size(eePositions,2)
    posTgt.TargetPosition = (eePositions(:,i)');
    aimCon.TargetPoint = posTgt.TargetPosition + [1.2 0.0 0.0];
    %aimCon.TargetPoint = [0.4 0.0 0.0]
    q(i,:) = gik(initialguess,posTgt,aimCon);
    initialguess = q(i,:);
end
%% Visualize robot configurations

axis([-1,1,-1,1,-1,1])
for i = 1:size(eePositions,2)
    show(robo,q(i,:), 'PreservePlot', false,'Frames',"on");
    pause(0.1)
end
