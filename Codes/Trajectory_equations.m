%LINE
XR=linspace(0,10,100);
YR=linspace(0,20,100);
%CIRCLE
XR=[];YR=[];
for theta=-pi:0.01:pi
    XR=[XR,cos(theta)];
    YR=[YR,sin(theta)];
    
end
%SINE
XR=[];YR=[];
for theta=-pi:0.01:pi
    XR=[XR,theta];
    YR=[YR,sin(theta)];
    
end
%Lemitscate
XR=[];YR=[];
A=5;
for theta=-pi:0.01:pi
    r=1+(sin(theta))^2;
    XR=[XR,A*(cos(theta))/r];
    YR=[YR,A*sin(theta)*cos(theta)/r];
end
