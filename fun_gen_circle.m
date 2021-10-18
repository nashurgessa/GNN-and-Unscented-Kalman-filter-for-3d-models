function [ polar_vector_circle, vector_circle_xyz ] = fun_gen_circle...
    ( Rx0, Ry0, Rz0, v, w, N, T )
%����Բ�κ���
%vΪĿ���ٶȣ�wΪĿ��Χ����Բ�ĵĽ��ٶ�

% for k=1:N
%     tmp_Rx(k)=Rx0+v/w*cos(w*(k-1)*T);%Ŀ����x���ϵ���ʵ�˶��켣
%     tmp_Ry(k)=Ry0+v/w*sin(w*(k-1)*T);%Ŀ����y���ϵ���ʵ�˶��켣
%     tmp_Rz(k)=Rz0;%Ŀ����z���ϵ���ʵ�˶��켣
%     Rr(k)=sqrt(tmp_Rx(k)^2+tmp_Ry(k)^2+tmp_Rz(k)^2);%��Ŀ���˶��켣ת��Ϊ������ϵ�µ�ֵ��Ŀ��ľ������
%     phi(k)=atan(tmp_Rz(k)/sqrt(tmp_Rx(k)^2+tmp_Ry(k)^2));%Ŀ��ĸ�����
%     theta(k)=atan(tmp_Ry(k)/tmp_Rx(k));%Ŀ��ķ�λ��
%     if((tmp_Rx(k)>=0 & tmp_Ry(k)>=0) | (tmp_Rx(k)>=0 & tmp_Ry(k)<0))%��������ϵת��ʱ�迼�����޵����⣬��λ�ڶ�������ʱ����λ����Ҫ����һ��pi
%         theta(k)=theta(k);
%     else theta(k)=theta(k)+pi;
%     end
% end

for k=1:N
    tmp_Rx(k)=Rx0+v/w*cos(w*(k-1)*T);%Ŀ����x���ϵ���ʵ�˶��켣
    tmp_Ry(k)=Ry0+v/w*sin(w*(k-1)*T);%Ŀ����y���ϵ���ʵ�˶��켣
    tmp_Rz(k)=Rz0;%Ŀ����z���ϵ���ʵ�˶��켣
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