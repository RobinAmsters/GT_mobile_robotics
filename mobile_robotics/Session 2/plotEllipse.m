% Returns x and y coordinates of ellipse based on center points, long and
% short axis and and angle relative to the x-axis.
%
% INPUTS
% a = axis of ellipse parallel to x-axis
% b = axis of ellipse parallel to y-axis
% x0 = x-coordinate of center of ellipse
% y0 = y-coordinate of center of ellipse
% thetax = tilting angle of ellipse
%
% OUTPUTS
% xE = x-coordinates of ellipse
% yE = y-coordinates of ellipse
%
% Robin Amsters
% Céderic Ketelbuters
% 2015 - 2016

function [xE, yE] = plotEllipse(a,b,x0,y0,thetax)
tE = linspace(0,2*pi,100);

xE = x0 + a*sin(tE+thetax);
yE = y0 + b*cos(tE);
end