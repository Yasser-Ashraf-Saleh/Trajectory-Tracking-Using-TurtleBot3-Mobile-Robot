%ipaddress = 'http://192.168.43.235:11311';
%rosinit(ipaddress)

rostopic list
odom = rossubscriber('/odom');

XA=[];
YA=[];
i=0;
kp_d=20;
kd_d=0;
ki_d=16;
kp_p=3;
kd_p=0;
ki_p=0;
PDE=0;PPE=0;
IIP=0;IID=0;
for TT=1:length(XR)
    i=i+1;
    dt=1;
    %Gettin the position
    odomdata = receive(odom, 3); % timout is 3s
    pose = odomdata.Pose.Pose;
    x1 = pose.Position.X
    y1 = pose.Position.Y
   
    odomdata = receive(odom, 3); % timout is 3s
    pose = odomdata.Pose.Pose;
    x2 = pose.Position.X
    y2 = pose.Position.Y
    XA=[XA,x2];
    YA=[YA,y2];
    theta=atan2(y2-y1,x2-x1);
    %z = pose.Position.Z
    %X(i,1:3)=[x,y,z];
    %Desired position
    %Line trajec: Y=X
    xr=XR(1,TT);
    yr=YR(1,TT);
    %error
    xe=xr-x2;
    ye=yr-y2;
    
    D=sqrt(xe^2+ye^2)
    cp=cross([cos(theta),sin(theta),0],[xe,ye,0]);
    sgn=sign(cp(1,3));
    phe=sgn*acos((xe*cos(theta)+ye*sin(theta))/D);
    %Controller
    DDD=(D-PDE)/dt;
    IID=IID+D*dt;
    dp=kp_d*D;
    dd=kd_d*DDD;
    di=ki_d*IID;
    velc=dp+dd+di;
    DDP=(phe-PPE)/dt;
    IIP=IIP+phe*dt;
    pp=kp_p*phe;
    pd=kd_p*phe;
    pi=ki_p*phe;
    angu=pp+pd+pi;
    %Setting velocities and angular velocities
    velocity = velc;     
    robot = rospublisher('/cmd_vel') ;
    velmsg = rosmessage(robot);
    w = angu;
    velmsg.Linear.X= velocity;
    velmsg.Angular.Z = w;
    send(robot, velmsg);qqq
    PDE=D;PPE=phe;
    pause(dt/4);

end
% to stop the robot
velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(robot, velmsg);
plot(XA,YA);
hold on
plot(XR,YR);