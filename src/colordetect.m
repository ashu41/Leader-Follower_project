tic;
clc;
clearvars;
OI=imread('pep.png');
OI=rgb2gray(OI);
imshow(OI)
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
drawnow
figure
[pC, graylevel]=imhist(OI);
bar(pC);
grid on;
t=100;
BI=OI>t;
BI=imfill(BI,'holes');
maxYValue = ylim;
line([t, t], maxYValue, 'Color', 'r');
annotationText = sprintf('Thresholded at %d gray levels', t);
text(double(t + 5), double(0.5 * maxYValue(2)), annotationText, 'FontSize', 10, 'Color', [0 .5 0]);
text(double(t - 70), double(0.94 * maxYValue(2)), 'Background', 'FontSize', 10, 'Color', [0 0 .5]);
text(double(t + 50), double(0.94 * maxYValue(2)), 'Foreground', 'FontSize', 10, 'Color', [0 0 .5]);
labeledImage = bwlabel(BI); 
figure
imshow(labeledImage, []); 