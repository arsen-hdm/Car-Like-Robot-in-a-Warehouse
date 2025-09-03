clc; clear all; close all;

l = 0.175;          %m distance between the front and rear wheels
phi_lim = 0.69;   %rad maximum steering angle
vlim = 2;        %m/s maximum heading velocity
wheelR = 0.1;    %m radius of the wheels
wlim = 1.25;       %rad/s maximum steering velocity

% change the name of the saved .mat file to distinguish between different
% models

save('MiR250_short_l.mat', 'l', 'phi_lim','vlim','wheelR','wlim');