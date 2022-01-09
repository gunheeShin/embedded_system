

    close all; clear ;

    load('data.mat');

%% 변수 설정    
   % 매트랩에서 3차원 배열은 row X column X page로 구성
    t = double(pickle_data.t); % timestamps [s], t는 uint16이므로 double형 변수와의 계산을 위해 double로 변환
    x_init = pickle_data.x_init; % initial x position [m]
    y_init = pickle_data.y_init; % initial y position [m]
    th_init = pickle_data.th_init; % initial theta position [rad]
    % input signal
    v = pickle_data.v; % translational velocity input [m/s]
    om = pickle_data.om; % rotational velocity input [rad/s]
    
    % bearing and range measurements, LIDAR constants
    b = pickle_data.b; % bearing to each landmarks center in the frame ,!attached to the laser [rad]
    r = pickle_data.r; % range measurements [m]
    l = double(pickle_data.l); % x,y positions of landmarks [m] l은 uint16이므로 double 변수와의 계산을 위해 double로 변환
    d = double(pickle_data.d); % distance between robot center and laser rangefinder ,![m] d는 0(uint8)이므로 변수와의 계산을 위해 double로 변환

    v_var = 0.01; % translation velocity variance
    om_var = 0.01; % rotational velocity variance
    % allowed to tune these values
    % r_var = 0.1 % range measurements variance
    r_var = 0.01;
    % b_var = 0.1 % bearing measurement variance
    b_var = 10;

    Q_km=diag([v_var, om_var]);
    cov_y=diag([r_var,b_var]);

    x_est= zeros(1,3,size(v,2));   % size(v,2)=501 v 배열의 열의 갯수
    P_est= zeros(3,3,size(v,2));

    x_est(1,:,1)=[x_init,y_init,th_init];   % 1페이지의 1행 전체
    P_est(:,:,1)=diag([1,1,0.1]);           % 1페이지 전체

    %% Main Filter Loop

    % Set the initial values

    P_check=P_est(:,:,1);
    x_check=reshape(x_est(1,:,1),[3,1]);


    for k=2:501
    delta_t=t(k)-t(k-1); % time stamp
    theta=wraptopi(x_check(3));

    % Update state with odometry readings
    F=[cos(theta),0; sin(theta),0; 0,1];
    inp= [v(k-1);om(k-1)];
    

    % Prediction
    x_check=x_check+F*inp*delta_t;
    x_check(3)=wraptopi(x_check(3));

    % Motion Model Jacobian with respect to last state
    F_km=zeros(3,3);
    F_km=[1,0,-sin(theta)*delta_t*v(k-1);
        0,1,cos(theta)*delta_t*v(k-1);
        0,0,1];

    % Motion Model jacobian with respect to noise
    L_km=zeros(3,2);
    L_km=[cos(theta)*delta_t,0;
        sin(theta)*delta_t,0;
        0,1];

    % Propagate uncertainty
    P_check= (F_km*P_check*F_km.'+L_km*Q_km*L_km.');

    % Update state estimate using available landmark measurements
    for i=1:8
        [x_check,P_check]=measurement_update(l(i,:),r(k,i),b(k,i),P_check,x_check,d,cov_y);
    end

    % Set final state predcitions for timestep
    x_est(1,1,k)=x_check(1);
    x_est(1,2,k)=x_check(2);
    x_est(1,3,k)=x_check(3);
    P_est(:,:,k)=P_check;

    end

%--------------------------------------------------------
% Plot을 위해 1X3X501 배열의 각 요소인 1X1X501을 1X501로 변환
%--------------------------------------------------------
    x_est_mod= reshape(x_est(:,1,:),1,size(x_est,3)); 
    y_est_mod= reshape(x_est(:,2,:),1,size(x_est,3));
    theta_mode=reshape(x_est(:,3,:),1,size(x_est,3));

    figure
    subplot(2,1,1);
    plot(x_est_mod,y_est_mod,'b');
    title('Estimated trajectory');
    xlabel('x [m]');
    ylabel('y [m]');

    subplot(2,1,2);
    plot(t(:),theta_mode,'g');
    title('Estimated trajectory');
    xlabel('Time [s]');
    ylabel('theta [rad]');


%% theta를 -pi~pi내로 유지해 주는 사용자 정의 함수
function theta_mod =wraptopi(theta)

if theta>pi
    theta_mod=theta-floor(theta/(2*pi)+1)*2*pi;
elseif theta<-pi
    theta_mod=theta+floor(theta/(-2*pi)+1)*2*pi;
else 
    theta_mod=theta;
end
end

%% Prediction의 결과와 Measurement 결과를 통해 Correction 해주는 사용자 정의 함수

function [x_check,P_check]=measurement_update(lk,rk,bk,P_check,x_check,d,cov_y)
    x_k=x_check(1);
    y_k=x_check(2);
    theta_k=wraptopi(x_check(3));

    x_l=lk(1);
    y_l=lk(2);
    
    d_x=x_l-x_k-d*cos(theta_k);
    d_y=y_l-y_k-d*sin(theta_k);
    
    r = sqrt(d_x.^2+d_y.^2);
    phi = atan2(d_y,d_x)-theta_k;

    %Compute measurement Jacobian
    H_k=zeros(2,3);
    H_k(1,1) = -d_x/r;
    H_k(1,2) = -d_y/r;
    H_k(1,3) = d*(d_x*sin(theta_k) - d_y*cos(theta_k))/r;
    H_k(2,1) = d_y/r.^2;
    H_k(2,2) = -d_x/r.^2;
    H_k(2,3) = -1-d*(d_y*sin(theta_k) + d_x*cos(theta_k))/r.^2;
    H_K=double(H_k);
    
    M_k=eye(2,2);
    y_out=[r;wraptopi(phi)];
    y_mes=[rk;wraptopi(bk)];

    %Compute Kalman Gain
    K_k=(P_check*H_k.')/(H_k*P_check*H_k.'+M_k*cov_y*M_k.');

    %Correct predicted state
    x_check=x_check+K_k*(y_mes-y_out);
    x_check(3)= wraptopi(x_check(3));

    %Correct covariance
    P_check=(eye(3,3)-K_k*H_k)*P_check;

end %return x_check,P_check


