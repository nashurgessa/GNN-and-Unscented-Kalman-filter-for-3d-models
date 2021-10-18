function [ vector_polar, vector_line_xyz ] = fun_gen_line ...
    ( Rx0, Ry0, Rz0, vx, vy, vz, Ax, Ay, Az, N, T )
%����ֱ�ߺ���


for k=1:N
    tmp_Rx(k)=Rx0+vx*(k-1)*T+0.5*(k-1)*Ax*T^2;%Ŀ����x���ϵ���ʵ�˶��켣
    tmp_Ry(k)=Ry0+vy*(k-1)*T+0.5*(k-1)*Ay*T^2;%Ŀ����y���ϵ���ʵ�˶��켣
    tmp_Rz(k)=Rz0+vz*(k-1)*T+0.5*(k-1)*Az*T^2;%Ŀ����z���ϵ���ʵ�˶��켣
%     vx_(k) = 0;
%     vy_(k) = 0;
%     vz_(k) = 0;
    tmp_vx(k) = vx + (k-1) * Ax* T;
    tmp_vy(k) = vy + (k-1) * Ay* T;
    tmp_vz(k) = vz + (k-1) * Az* T;
%     ax_(k) = Ax;
%     ay_(k) = Ay;
%     az_(k) = Az;
    Rr(k)=sqrt(tmp_Rx(k)^2+tmp_Ry(k)^2+tmp_Rz(k)^2);%��Ŀ���˶��켣ת��Ϊ������ϵ�µ�ֵ��Ŀ��ľ������
    phi(k)=atan(tmp_Rz(k)/sqrt(tmp_Rx(k)^2+tmp_Ry(k)^2));%Ŀ��ĸ�����
    theta(k)=atan(tmp_Ry(k)/tmp_Rx(k));%Ŀ��ķ�λ��
    if((tmp_Rx(k)>=0 & tmp_Ry(k)>=0) | (tmp_Rx(k)>=0 & tmp_Ry(k)<0))%��������ϵת��ʱ�迼�����޵����⣬��λ�ڶ�������ʱ����λ����Ҫ����һ��pi
        theta(k)=theta(k);
    else theta(k)=theta(k)+pi;
    end
end

for k=1:N
    tmp_Rx(k)=Rx0+vx*(k-1)*T+0.5*(k-1)*Ax*T^2;%Ŀ����x���ϵ���ʵ�˶��켣
    tmp_Ry(k)=Ry0+vy*(k-1)*T+0.5*(k-1)*Ay*T^2;%Ŀ����y���ϵ���ʵ�˶��켣
    tmp_Rz(k)=Rz0+vz*(k-1)*T+0.5*(k-1)*Az*T^2;%Ŀ����z���ϵ���ʵ�˶��켣
    [theta(k),Rr(k)] = cart2pol(tmp_Rx(k),tmp_Ry(k));
    phi(k) = atan(tmp_Rz(k)/sqrt(tmp_Rx(k)^2+tmp_Ry(k)^2));
    Rr_v(k) =0;
    theta_v(k) =0;
    phi_v(k) =0;
%     Rr_a(k) = 0;
%     theta_a(k) = 0;
%     phi_a(k) = 0;
end


vector_polar=[Rr; theta; phi; Rr_v; theta_v; phi_v];
vector_line_xyz = [tmp_Rx; tmp_Ry; tmp_Rz; tmp_vx; tmp_vy; tmp_vz] ;
 
end