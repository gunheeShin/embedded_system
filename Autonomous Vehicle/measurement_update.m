%% Prediction의 결과와 Measurement 결과를 통해 Correction
function [x_check,P_check]=measurement_update(lk,rk,bk,P_check,x_check,d,cov_y)
    x_k=x_check(1);
    y_k=x_check(2);
    theta_k=wraptopi(x_check(3));

    x_l=double(lk(1));
    y_l=double(lk(2));
    

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
    
    M_k=[1,0.0001;0.0001,1];
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