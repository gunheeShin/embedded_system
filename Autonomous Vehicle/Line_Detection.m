
clear; close all;


img = imread('image3.jpg');

% Add noise into the image
img_noise = imnoise(img, 'speckle', 0.05);

H=1/9*[[1,1,1];[1,1,1];[1,1,1]]; %Average Filter

img_denoise_avg=imfilter(img_noise,H,'conv');   % Noise removal using an average filter.
img_denoise_gau=imgaussfilt(img_noise);         % Noise removal using an Gaussian filter.
img_noise_2D=img_noise(:,:,1);                  % Transform 3D image to 2D image.
img_denoise_med=medfilt2(img_noise_2D);         % Noise removal using Median filter

figure;
subplot(1,3,1);
imshow(img_denoise_avg); title('Average filter');
subplot(1,3,2);
imshow(img_denoise_gau); title('Gaussian filter');
subplot(1,3,3);
imshow(img_denoise_med); title('Median filter');


%% 1) Apply Filter 
H1=[[-1,-1,-1];[-1,9,-1];[-1,-1,-1]];   % Gaussian Filter
H2=[[0,0,0];[0,2,0];[0,0,0]]-H1;        % Sharpening Filter


img_filtered_gau=imfilter(img,H1,'conv');   % Filtering the original image using Gaussian Filter
img_filtered_sha=imfilter(img,H2,'conv');   % Filtering the original image using Sharpening Filter
% Sharpening Filter expressed Edge better than Gaussian Filter

img_filtered_avr_gau=imfilter(img_denoise_avg,H1,'conv');   % Filtering the denoised image(by Averager filter) using Gaussian Filter
img_filtered_avr_sha=imfilter(img_denoise_avg,H2,'conv');   % Filtering the denoised image(by Averager filter) using Sharpening Filter

img_filtered_gau_gau=imfilter(img_denoise_gau,H1,'conv');   % Filtering the denoised image(by Gaussian filter) using Gaussian Filter
img_filtered_gau_sha=imfilter(img_denoise_gau,H2,'conv');   % Filtering the denoised image(by Gaussian filter) using Sharpening Filter

img_filtered_med_gau=imfilter(img_denoise_med,H1,'conv');   % Filtering the denoised image(by Median filter) using Sharpening Filter
img_filtered_med_sha=imfilter(img_denoise_med,H2,'conv');   % Filtering the denoised image(by Median filter) using Sharpening Filter


figure;
subplot(1,2,1);
imshow(img_filtered_gau); title('Gaussian filter');
subplot(1,2,2);
imshow(img_filtered_sha); title('Sharpening filter');

figure;
subplot(1,2,1);
imshow(img_filtered_avr_gau); title('Gaussian filter avr');
subplot(1,2,2);
imshow(img_filtered_avr_sha); title('Sharpening filter avr');

figure;
subplot(1,2,1);
imshow(img_filtered_gau_gau); title('Gaussian filter gau');
subplot(1,2,2);
imshow(img_filtered_gau_sha); title('Sharpening filter gau');

figure;
subplot(1,2,1);
imshow(img_filtered_med_gau); title('Gaussian filter med');
subplot(1,2,2);
imshow(img_filtered_med_sha); title('Sharpening filter med');

% As a result of filtering, i proceeded a procces with a image which is denoised by Gaussian filter and also edge detected by Gausian Filter 

%% 2-3) Apply Edge detection and Hough Transform to find lines

img_2D=img_filtered_avr_gau(:,:,1); % Change the 3D image to the 2D image
img_Edge=edge(img_2D,'canny');      %canny Edge detection

[H,T,R] = hough(img_Edge,'Theta',20:89);            %Hough Tranform, 
                                                    % By limitng theta to 20~89, all lines close to vertical near streetlights were removed
P=houghpeaks(H,3,'threshold',ceil(0.3*max(H(:))));  % To detect lines in the image, the number of peaks should be more than 3

imshow(H,[],'XData',T,'YData',R,...
            'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
plot(T(P(:,2)),R(P(:,1)),'s','color',"green"); % In the Hough graph, peak values are expressed in a green box. 

%% 4) From detected edges estimate the lines associated to the true lane. Then compute angle of the lane 

lines = houghlines(img_Edge,T,R,P,'FillGap',5,'MinLength',150); % Detected edge due to noise was signigicantly removed from the fillgap of 5.                                                                
                                                                %Long-running noise was removed from the MinLength 150.

figure, imshow(img_denoise_avg), hold on
for k = 1:length(lines)
   road_lane = [lines(k).point1; lines(k).point2]; 
   
   plot(road_lane(:,1),road_lane(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(road_lane(1,1),road_lane(1,2),'x','LineWidth',2,'Color','yellow');
   plot(road_lane(2,1),road_lane(2,2),'x','LineWidth',2,'Color','red');
end

%% Visualization
figure; subplot(1, 3, 1);
imshow(img_noise); title('noise image');
subplot(1,3,2);
imshow(H,[],'XData',T,'YData',R,...
            'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
plot(T(P(:,2)),R(P(:,1)),'s','color',"green");
subplot(1,3,3);
imshow(img_denoise_avg), hold on
max_len = 0;
for k = 1:length(lines)
   road_lane = [lines(k).point1; lines(k).point2];
   
   plot(road_lane(:,1),road_lane(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(road_lane(1,1),road_lane(1,2),'x','LineWidth',2,'Color','yellow');
   plot(road_lane(2,1),road_lane(2,2),'x','LineWidth',2,'Color','red');
end
title('line detection');
