clear; 
close;
clc;
% ROS Setup
rosinit;
vel_cmd=rospublisher("/cmd_vel","DataFormat","struct");
velocity=rosmessage(vel_cmd);

diff_drv = differentialDriveKinematics("TrackWidth",0.16,"WheelRadius", 0.066, "VehicleInputs", "VehicleSpeedHeadingRate");
r=0.066;
d=0.16;

% JointStates = rossubscriber('/joint_states');
poseSub=rossubscriber('/odom');
% joint_data=receive(JointStates);  

% 
% freq=
% t=0:pi/24:pi/2;
% x=1.1 + 2*sin(freq*t);
% y = 0.9 + 0.7*sin(2*freq*t);
% path=[x;y]';

tim=0:pi/24:2*pi;
path_y =7.*sin(tim);
path=[tim;path_y]';


init = path(1,:);
destn = path(end,:);
tolerance = 0.5;
dist = norm(init - destn);

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 2.84;
controller.LookaheadDistance = 1.7;

poseStates=receive(poseSub,10);

quart = [poseStates.Pose.Pose.Orientation.X,poseStates.Pose.Pose.Orientation.Y,poseStates.Pose.Pose.Orientation.Z, poseStates.Pose.Pose.Orientation.W];
eul=quat2eul(quart);
pose_curr=[poseStates.Pose.Pose.Position.X,poseStates.Pose.Pose.Position.Y,eul(3)]';
pose=pose_curr;

robotCurrent_x=[pose_curr(1)];
robotCurrent_y=[pose_curr(2)];
robotDesir_x=[pose(1)];
robotDesir_y=[pose(2)];
time=1;
time_stamp=[0];

ld = controller.LookaheadDistance;
rate = 0.1;
viz = rateControl(1/rate);
frame = diff_drv.TrackWidth/0.5;
RST(1,:) = 0;
LST(1,:) = 0;

tic;
t=0;

while (dist > tolerance)

    hold off
    [v, omega, lookaheadpoint] = controller(pose); % to get the lookahead point
    clear v omega;
    dir = atan2((lookaheadpoint(2)-pose(2)),(lookaheadpoint(1)-pose(1)));
    alpha = -pose(3) + dir;
    delta = atan2(2*sin(alpha),ld);

    LS = controller.DesiredLinearVelocity + d*tan(delta);
    RS = controller.DesiredLinearVelocity - d*tan(delta);
    RST(end+1,:) = RS;
    LST(end+1,:) = LS;
%     pose(1:2) = pose(1:2) + 20*[cos(pose(3));sin(pose(3))]*rate;
%     td = (1/3)*(-RS + LS);
%     pose(3) = pose(3) + td*rate;
    dist = norm(pose(1:2)'-destn(1:2));
    
    v=r*(LS+RS)/2;
    omega=r*(RS-LS)/(2*d);
    
    velocity.Linear.X = v;   
    velocity.Angular.Z = omega; 
    send(vel_cmd,velocity)
    
    poseStates=receive(poseSub,10);
%     pose_curr=[poseStates.Pose.Pose.Position.X,poseStates.Pose.Pose.Position.Y,poseStates.Pose.Pose.Orientation.W]';
%     pose=pose_curr;
    quart= [poseStates.Pose.Pose.Orientation.X,poseStates.Pose.Pose.Orientation.Y,poseStates.Pose.Pose.Orientation.Z, poseStates.Pose.Pose.Orientation.W];
    eul=quat2eul(quart);
    pose_curr=[poseStates.Pose.Pose.Position.X,poseStates.Pose.Pose.Position.Y,eul(3)]';
    pose=pose_curr;

    plotTrVec = [pose(1:2); 0];
    show(mapp)
    hold on
    plot(path(:,1),path(:,2))
    hold on
    plotRot = axang2quat([0 0 1 pose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize,'MeshColor','r');

    xdir = lookaheadpoint(1) - pose(1);
    ydir = lookaheadpoint(2) - pose(2);
    plt = [pose(1),pose(2);lookaheadpoint(1),lookaheadpoint(2);lookaheadpoint(1)+ 10*xdir, lookaheadpoint(2)+ 10*ydir];
    plot(plt(:,1),plt(:,2),'LineWidth',2,'Color','g');
    hold off

    xlim([100 700])
    ylim([100 700])
    title('Inflated WPI Map');
    waitfor(viz);
end

velocity.Linear.X = 0;   
velocity.Angular.X = 0;
velocity.Angular.Z = 0;
send(vel_cmd,velocity)

rosshutdown

figure(1);
subplot(2,2,1)
plot(time_stamp,robotCurrent_y ); 
title('Robots current y co-ordinate Vs Time');
xlabel('Time t (s)');
ylabel('Position Y of the robot');
legend('Pose_y gazebo');

subplot(2,2,2)
plot(path(:,1),path(:,2),'-r');
title('Desired path y=7sin(x) Vs X ');
xlabel('X axis');
ylabel('Yaxis');
legend('The desired path ');

subplot(2,2,3)
plot(robotCurrent_x,robotCurrent_y,'-'); 
title('Path followed by robot Vs time');
xlabel('X axis of the robot');
ylabel('Y axis of the robot');
legend('Actual path gazebo');