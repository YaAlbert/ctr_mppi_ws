function R = rotation_from_vectors(a, b)
% ROTATION_FROM_VECTORS
% ------------------------------------------------------------
% Purpose:
%   Compute a rotation matrix R such that:
%       R * a = b
%
% Inputs:
%   a - 3x1 or 1x3 source direction vector
%   b - 3x1 or 1x3 target direction vector
%
% Output:
%   R - 3x3 rotation matrix
%
% Notes:
%   - This function is useful for aligning the initial tangent of a CTR
%     trajectory to the initial tangent of a vessel segment.
%   - This file is a geometry helper and is not the main project focus.
% ------------------------------------------------------------

    a = a(:) / norm(a);
    b = b(:) / norm(b);

    v = cross(a, b);
    c = dot(a, b);
    s = norm(v);

    if s < 1e-12
        if c > 0
            R = eye(3);
        else
            % 180-degree rotation: choose any orthogonal axis
            tmp = null(a.');
            axis = tmp(:,1);
            K = [0 -axis(3) axis(2);
                 axis(3) 0 -axis(1);
                 -axis(2) axis(1) 0];
            R = eye(3) + 2*K*K;
        end
        return;
    end

    K = [0 -v(3) v(2);
         v(3) 0 -v(1);
         -v(2) v(1) 0];

    R = eye(3) + K + K*K*((1-c)/(s^2));
end