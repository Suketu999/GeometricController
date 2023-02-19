diff_drv = differentialDriveKinematics("TrackWidth", 3, "VehicleInputs", "VehicleSpeedHeadingRate");

% prm = robotics.PRM(mapp);
% prm.NumNodes = 8000;
% prm.ConnectionDistance = 40;
% startLocation = [200 300];
% endLocation = [580 300];
% % path1 = findpath(prm, startLocation, endLocation);
% show(prm)
% hold on
% startLocation = [580 300];
% endLocation = [650 600];
% path2 = findpath(prm, startLocation, endLocation);
% path = [path1;path2(3:end,:)];
% show(prm)
% hold on
% plot(path(:,1),path(:,2),'color','r');
% xlim([100 700])
% ylim([100 700])

load('path_smth.mat');

path = path(1:17,:);
plot(path(:,1),path(:,2))
show (mapp)

init = path(1,:);
destn = path(end,:);
pose = [init 0]';
tolerance = 10;
dist = norm(init - destn);

lookaheadFinder = controllerPurePursuit;
lookaheadFinder.Waypoints = path;
lookaheadFinder.DesiredLinearVelocity = 20;
lookaheadFinder.MaxAngularVelocity = 20;
lookaheadFinder.LookaheadDistance = 3;

ld = lookaheadFinder.LookaheadDistance;
rate = 0.1;
viz = rateControl(1/rate);
frame = diff_drv.TrackWidth/0.5;
% RST(1,:) = 0;
% LST(1,:) = 0;
lookaheadpoint_prev = pose(1:2)';
delta_prev = 0;
velo = 20;
% del = 0;
% po = 0;

while (dist > tolerance)

    hold off
    [v, omega, lookaheadpoint] = lookaheadFinder(pose); % to get the lookahead point
    clear v omega;
    dir = atan2((lookaheadpoint(2)-pose(2)),(lookaheadpoint(1)-pose(1)));
%     alpha = -pose(3) + dir;
%     delta = atan2(2*sin(alpha),ld);
%     lookaheadpoint

    a = [lookaheadpoint,0] - [lookaheadpoint_prev,0];
    b = [pose(1:2)',0] - [lookaheadpoint_prev,0];
    crosstrack = -((cross(a,b))/norm(a));

%     if isnan(crosstrack(3)) & crosstrack_prev(3)>0
%         crosstrack(3) = 500;
%     elseif isnan(crosstrack(3)) & crosstrack_prev(3)<0
%         crosstrack(3) = -500;
%     end
%     crosstrack(3)
%     if crosstrack(3)>15
%         crosstrack(3) = 15;
%     elseif crosstrack(3)<-15
%         crosstrack(3) = -15;
%     end

    head = atan2(lookaheadpoint(2)-lookaheadpoint_prev(2),lookaheadpoint(1)-lookaheadpoint_prev(1));
    (head-delta_prev)*180/pi;
%     atan2(crosstrack(3),(velo));
%     crosstrack(3);

    delta = (head-delta_prev) + atan2(crosstrack(3)*1.5,(velo));
%     del(end+1) = delta;

    LS = velo + diff_drv.TrackWidth*tan(delta);
    RS = velo - diff_drv.TrackWidth*tan(delta);
%     RST(end+1,:) = RS;
%     LST(end+1,:) = LS;
    pose(1:2) = pose(1:2) + velo*[cos(pose(3));sin(pose(3))]*rate;
    theta_dot = (1/diff_drv.TrackWidth)*(-RS + LS);
    pose(3) = pose(3) + theta_dot*rate;
    dist = norm(pose(1:2)'-destn(1:2));
%     po(end+1) = pose(3);

    plotTrVec = [pose(1:2); 0];
    show(mapp)
    hold on
    plot(path(:,1),path(:,2))
    hold on
    plotRot = axang2quat([0 0 1 pose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frame,'MeshColor','r');

%     xdir = lookaheadpoint(1) - pose(1);
%     ydir = lookaheadpoint(2) - pose(2);
%     plt = [pose(1),pose(2);lookaheadpoint(1),lookaheadpoint(2);lookaheadpoint(1)+ 10*xdir, lookaheadpoint(2)+ 10*ydir];
%     plot(plt(:,1),plt(:,2),'LineWidth',2,'Color','g');
    
    hold off

    xlim([100 700])
    ylim([100 700])
    title('Inflated WPI Map - Stanley Controller');
    waitfor(viz);
    lookaheadpoint_prev = lookaheadpoint;
    delta_prev = delta;
    crosstrack_prev = crosstrack;
end