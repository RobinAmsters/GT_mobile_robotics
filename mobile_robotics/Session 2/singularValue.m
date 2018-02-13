% Calculate singular values and vectors of [x,y]-part of covariance matrix
%
% INPUTS
% Pcov = covariance matrix
%
% OUTPUTS
% S = singular values of covariance matrix (= abs(eigenvalues))
% normV = normalized eigenvectors of covariance matrix
% 
% Robin Amsters
% Céderic Ketelbuters
% 2015 - 2016

function [S, normV] = singularValue(Pcov)
%% INITIALIZATION
cov_sub = ([Pcov([1,2],1),Pcov([1,2],2)]); % select only relevant elements for [x,y]-plot from covariance matrix

%% SVD
% Get singular value decomposition of input, U is identical to V for symmetric matrices
[~,S,V] = svd(cov_sub); 

%% NORMALIZE
S = [S([1,2],1).*norm([V(1),V(2)]),S([1,2],2).*norm([V(3),V(4)])]; % multiply S = singular values by norm of columns V
normV = [V([1,2],1)./norm([V(1),V(2)]),V([1,2],2)./norm([V(3),V(4)])]; % get Columns of V = eigenvectors and normalize
end
