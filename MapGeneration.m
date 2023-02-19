I = imread('wpi_map.jpeg');
I = rgb2gray(I);
BW = imbinarize(I);
% imshow(BW);
BW = imcomplement(BW);
mapp = binaryOccupancyMap(BW);
% figure
% show(mapp);
inflate(mapp,2);
% figure(2)
% show(mapp)
% title('Inflated WPI Map')
% xlim([100 700])
% ylim([100 700])