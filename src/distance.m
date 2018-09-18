Im1 = imread('robo1.png');
Im2 = imread('robo2.png');
%plotting of them
subplot(1,2,1);
imshow(Im1);
subplot(1,2,2);
imshow(Im2);
Im1=rgb2gray(Im1);
Im2=rgb2gray(Im2);

%the code for conversion of image to its normalized histogram

hn1 = imhist(Im1)./numel(Im1);
hn2 = imhist(Im2)./numel(Im2);

% Calculate the Euclidean distance
f = sum(sqrt(hn1 - hn2).^2)
%f=norm(hn1,hn2);