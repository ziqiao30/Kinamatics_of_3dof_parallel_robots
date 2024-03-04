
%the angle of the leg
phi1=pi/6:pi/30:pi/2;
phi2=pi/6:pi/30:pi/2;
phi3=pi/6:pi/30:pi/2;
maxRows=round((pi/2-pi/6)/pi/60);
rowIndex = 1;
dataMatrix = zeros(maxRows, 3);

for x=phi1
    for y=phi2
        for z=phi3
            [p,N]=Forward_Kinematics_pushbutton(x,y,z);
            
            dataMatrix(rowIndex, :) = p.';
            rowIndex=rowIndex+1;
            scatter3(p(1),p(2),p(3),'blue','*','LineWidth',4);
%             [azimuth,elevation,r] =cart2sph(N(1),N(2),N(3));
%             azimuth= rad2deg(azimuth);
%             elevation= rad2deg(elevation)*2;
%             scatter3(elevation, azimuth,r,  4, 'g');

            hold on
         
            % myfunction(p,N)
            %hold on
            xlabel('x[m]')
            ylabel('y[m]')
            zlabel('z[m]')
        end
    end
end




grid on 
axis square 
%