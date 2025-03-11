%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rla()
% Plot the root locus of the TF with ASSYMPTOTES
%
% Syntax: 
%   [R K] = rla(GH)
%   [R K] = rla(GH, k)
%   [R K] = rla(GH, k, lw)
%   [R K] = rla(GH, k, lw, zoom)
%
% Input:
%   GH    = O-L Transfer Function (system)
%
%   Optional Inputs
%   k     = vector of gains     def = nan (let Matlab decide)
%   lw    = rl line width       def = nan (lw = 4)
%   zoom  = amt to zoom OUT     def = 1
%
% Output:
%   R = matrix of roots corresponding to each gain value
%   K = vector of gain values
%
% Example:
%   To change line width to 2 but use the default k vector
%   rla(GH, nan, 2);
%
% Written by: Leo Stocco
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [R K] = rla(GH, varargin)

  % Params
  if nargin == 1
    [R K] = rlaHelper(GH);
  elseif nargin == 2
    [R K] = rlaHelper(GH, varargin{1});
  elseif nargin == 3
    [R K] = rlaHelper(GH, varargin{1}, varargin{2});
  elseif nargin == 4
    [R K] = rlaHelper(GH, varargin{1}, varargin{2}, varargin{3});
  else
    display('Wrong number of input parameters.');
  end

end % function
