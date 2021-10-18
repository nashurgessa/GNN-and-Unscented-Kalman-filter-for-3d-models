function [ polar_vector_circle, vector_circle_xyz ] = fun_gen_circle...
    ( Rx0, Ry0, Rz0, v, w, N, T )
%产生圆形航迹
%v为目标速度，w为目标围绕其圆心的角速度

% for k=1:N
%     tmp_Rx(k)=Rx0+v/w*cos(w*(k-1)*T);%目标在x轴上的真实运动轨迹
%     tmp_Ry(k)=Ry0+v/w*sin(w*(k-1)*T);%目标在y轴上的真实运动轨迹
%     tmp_Rz(k)=Rz0;%目标在z轴上的真实运动轨迹
%     Rr(k)=sqrt(tmp_Rx(k)^2+tmp_Ry(k)^2+tmp_Rz(k)^2);%将目标运动轨迹转换为极坐标系下的值，目标的径向距离
%     phi(k)=atan(tmp_Rz(k)/sqrt(tmp_Rx(k)^2+tmp_Ry(k)^2));%目标的俯仰角
%     theta(k)=atan(tmp_Ry(k)/tmp_Rx(k));%目标的方位角
%     if((tmp_Rx(k)>=0 & tmp_Ry(k)>=0) | (tmp_Rx(k)>=0 & tmp_Ry(k)<0))%进行坐标系转换时需考虑象限的问题，当位于二三象限时，方位角需要加上一个pi
%         theta(k)=theta(k);
%     else theta(k)=theta(k)+pi;
%     end
% end

for k=1:N
    tmp_Rx(k)=Rx0+v/w*cos(w*(k-1)*T);%目标在x轴上的真实运动轨迹
    tmp_Ry(k)=Ry0+v/w*sin(w*(k-1)*T);%目标在y轴上的真实运动轨迹
    tmp_Rz(k)=Rz0;%目标在z轴上的真实运动轨迹
    vx(k) = 0;
    vy(k) = 0;
    vz(k) = 0;
%     ax_(k) = 0;
%     ay_(k) = 0;
%     az_(k) = 0;
    [theta(k),Rr(k)] = cart2pol(tmp_Rx(k),tmp_Ry(k));
    phi(k) = atan(tmp_Rz(k)/sqrt(tmp_Rx(k)^2+tmp_Ry(k)^2));
    R_v(k) = 0;
    theta_v(k) = 0;
    phi_v(k) = 0;
%     R_a(k) = 0;
%     theta_a(k) = 0;
%     phi_a(k) = 0;
end

polar_vector_circle = [Rr; theta; phi; R_v; theta_v; phi_v];
vector_circle_xyz = [tmp_Rx; tmp_Ry; tmp_Rz; vx; vy; vz];


end %end function