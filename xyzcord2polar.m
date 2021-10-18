function [polar_vec] = xyzcord2polar(xyz_cord, dt)
    
    column_wise = num2cell(transpose(xyz_cord));
    [rx, ry, rz, vx, vy, vz] = column_wise{:};
    % [rx, ry, rz, vx, vy, vz] = column_wise{:};
    Rx = rx + vx*dt;%  + 1/2*vx*dt^2;
    Ry = ry + vy*dt;%  + 1/2*vy*dt^2;
    Rz = rz + vz*dt;%  + 1/2*vz*dt^2;
    
    Rr =sqrt(Rx^2 + Ry^2 + Rz^2);
    phi=atan(Rz / sqrt(Rx^2 + Ry^2));
    theta=atan(Ry / Rx);
    
    if((Rx>=0 & Ry>=0) | (Rx>=0 & Ry<0))
        theta=theta;
    else
        theta=theta+pi;
    end
    
%     if ((rx^2+ry^2)^2) < 0.0001  % Threshold = 0.0001
%         Rr= 0.0;
%         theta = 0.0;
%         phi = 0.0;
%     end
    
    polar_vec = [Rr; theta; phi];
end

