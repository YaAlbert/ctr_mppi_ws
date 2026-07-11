function [Pseg, idx_start, idx_end] = ExtractLocalSegment(Pcurve, s_uniform, idx_center, targetLength)
% EXTRACTLOCALSEGMENT
% ------------------------------------------------------------
% Purpose:
%   Extract a local segment from a centerline around a specified center
%   index, with a desired approximate total arc length.
%
% Inputs:
%   Pcurve       - Nx3 centerline points
%   s_uniform    - Nx1 arc-length parameter associated with Pcurve
%   idx_center   - index around which the segment is extracted
%   targetLength - desired total segment length
%
% Outputs:
%   Pseg         - extracted local segment points
%   idx_start    - start index of the segment
%   idx_end      - end index of the segment
%
% Notes:
%   - This is a utility function. You should use it to isolate local
%     vessel segments near high-curvature regions.
%   - This file is not the main research focus of the project.
% ------------------------------------------------------------

    if idx_center < 1 || idx_center > size(Pcurve,1)
        error('ExtractLocalSegment:IndexOutOfRange', ...
              'idx_center is out of range.');
    end

    halfLength = targetLength / 2;

    s_center = s_uniform(idx_center);
    s_min = s_center - halfLength;
    s_max = s_center + halfLength;

    idx_start = find(s_uniform >= s_min, 1, 'first');
    idx_end   = find(s_uniform <= s_max, 1, 'last');

    if isempty(idx_start)
        idx_start = 1;
    end
    if isempty(idx_end)
        idx_end = size(Pcurve,1);
    end

    Pseg = Pcurve(idx_start:idx_end, :);
end