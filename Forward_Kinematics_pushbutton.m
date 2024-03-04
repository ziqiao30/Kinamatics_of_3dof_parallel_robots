
%------------- Forward Kinematics of 'pushbutton' ----------------

% input: the angle of each leg
% ouput: the center postion and the normal vector of the top plane

function [pos,N]=Forward_Kinematics_pushbutton(phi1,phi2,phi3)
    % r the radius of the hexagon's inscribed circle
    r=0.04406;%m
    % the length of the leg
    l=0.0594; %m
    % the center location of the base
    Ob=[0,0,0];
    
    % the location of the waterbomb center bi
    theta1=0;
    theta2=2*pi/3;
    theta3=4*pi/3;
    
    % the position of yhe waterbomb center
    b1=[cos(theta1)*(r+l*cos(phi1)),sin(theta1)*(r+l*cos(phi1)),l*sin(phi1)];
    b2=[cos(theta2)*(r+l*cos(phi2)),sin(theta2)*(r+l*cos(phi2)),l*sin(phi2)];
    b3=[cos(theta3)*(r+l*cos(phi3)),sin(theta3)*(r+l*cos(phi3)),l*sin(phi3)];
    
    % orthogonal vectorNto the virtualplane
    N=cross((b1-b2),(b1-b3));
    
    % distance fromObthe virtual plane
    d=dot((b1-Ob),N)/norm(N);
    % posistion of Op
    pos=2*d*N/norm(N);
    fileID = fopen('exp.txt','w');
    fprintf('%4.4f  %4.4f %4.4f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f\r\n',pos(1),pos(2),pos(3), N(1),N(2),N(3),phi1,phi2,phi3);
    fclose(fileID);
end
