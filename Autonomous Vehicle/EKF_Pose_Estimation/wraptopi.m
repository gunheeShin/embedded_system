%% Keep theta in -pi~pi
function theta_mod =wraptopi(theta)

if theta>3.14
    theta_mod=theta-floor(theta/(2*pi)+1)*2*pi;
elseif theta<-pi
    theta_mod=theta+floor(theta/(-2*pi)+1)*2*pi;
else 
    theta_mod=theta;
end
end