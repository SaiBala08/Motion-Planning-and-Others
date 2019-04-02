%% Importing  and Displaying Robot

robo = importrobot('/chaitanya_model/Assem Baxter Arm/urdf/Assem Baxter Arm (1).urdf')
axes = show(robo);
axes.CameraPositionMode = 'auto';
axis([-1,1,-1,1,-1,1])

%% Adding end effector frame

eeBody = robotics.RigidBody('Link_7');
setFixedTransform(eeBody.Joint,trvec2tform([0.35 0.30 0]));
addBody(robo,eeBody,'Link_6');

%% Perform Forward Kinematics for a point in space
% Given specific Joint Angles(J_Values) the End Effector 
% will reach a point Coordinates_EE.

J_Values = {pi/4,0,0,0,0,0};
initialguess = struct('JointName',{'Rev_1','Rev_2','Rev_3','Rev_4','Rev_5','Rev_6',},'JointPosition',J_Values);
H = getTransform(robo,initialguess,'Link_7','base_link');
Coordinates_EE = tform2trvec(H)

%% Displaying Robot with the given Joint Angles

axes = show(robo,initialguess,"Frames","on");
axes.CameraPositionMode = 'auto';
axis([-1,1,-1,1,-1,1])
