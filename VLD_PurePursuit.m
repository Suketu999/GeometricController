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

% load('path_smth.mat');

plot(path(:,1),path(:,2))
show (mapp)

ang(1) = 0;
for i = 2:length(path)
    x(i-1,1) = path(i,1)-path(i-1,1);
    x(i-1,2) = path(i,2)-path(i-1,2);
    ang(i) = (180/pi)*atan2(x(i-1,2),x(i-1,1));
    diff_ang(i) = ang(i)-ang(i-1);
end
diff_ang(end+1:end+2) = 0;

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
RST(1,:) = 0;
LST(1,:) = 0;
i=1;

while (dist > tolerance)

    hold off
    lookaheadFinder.LookaheadDistance = 8 - 1.3*(6.5*abs(diff_ang(floor(i/12)+1))/90);
%     ld = 8 - 1.3*(6.5*abs(diff_ang(floor(i/12)+1))/90);
    [v, omega, lookaheadpoint] = lookaheadFinder(pose); % to get the lookahead point
    clear v omega;
    dir = atan2((lookaheadpoint(2)-pose(2)),(lookaheadpoint(1)-pose(1)));
    alpha = -pose(3) + dir;
    delta = atan2(2*sin(alpha),ld);

    LS = 20 + 3*tan(delta);
    RS = 20 - 3*tan(delta);
    RST(end+1,:) = RS;
    LST(end+1,:) = LS;
    pose(1:2) = pose(1:2) + 20*[cos(pose(3));sin(pose(3))]*rate;
    td = (1/3)*(-RS + LS);
    pose(3) = pose(3) + td*rate;
    dist = norm(pose(1:2)'-destn(1:2));

    plotTrVec = [pose(1:2); 0];
    show(mapp)
    hold on
    plot(path(:,1),path(:,2))
    hold on
    plotRot = axang2quat([0 0 1 pose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frame,'MeshColor','r');

    xdir = lookaheadpoint(1) - pose(1);
    ydir = lookaheadpoint(2) - pose(2);
    plt = [pose(1),pose(2);lookaheadpoint(1),lookaheadpoint(2);lookaheadpoint(1)+ 10*xdir, lookaheadpoint(2)+ 10*ydir];
    plot(plt(:,1),plt(:,2),'LineWidth',2,'Color','g');
    hold on

    xlim([100 700])
    ylim([100 700])
    title('Inflated WPI Map - PP with Variable l_d');
    i = i+1;
    waitfor(viz);

end