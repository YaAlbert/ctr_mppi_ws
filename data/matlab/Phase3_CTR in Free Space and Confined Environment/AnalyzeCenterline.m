function result = AnalyzeCenterline(filename, numSamples, windowSize, margin, showPlots)
% ANALYZECENTERLINE
% ------------------------------------------------------------
% Purpose:
%   Read a 3D Slicer .fcsv centerline file, resample it by arc length,
%   smooth it, compute curvature, and identify the maximum-curvature point.
%
% Inputs:
%   filename   - path to a .fcsv centerline file exported from 3D Slicer
%   numSamples - number of resampled points (default: 200)
%   windowSize - smoothing window for sgolay filter (default: 11)
%   margin     - number of endpoint samples ignored when searching for the
%                maximum curvature point (default: 8)
%   showPlots  - true/false flag for diagnostic figures (default: true)
%
% Output:
%   result - structure with fields:
%       .P_raw        : original imported centerline points
%       .P_resampled  : uniformly resampled centerline points
%       .P_smooth     : smoothed centerline points
%       .s_raw        : raw arc length parameter
%       .s_uniform    : uniform arc length parameter
%       .kappa        : curvature profile
%       .kappa_max    : maximum curvature value
%       .idx_max      : index of selected maximum-curvature point
%       .p_maxcurv    : coordinates of selected maximum-curvature point
%
% Notes:
%   - This file is provided as a utility. You should use it as a tool
%     for centerline preparation and curvature analysis.
%   - This file is not the main research target of the project.
% ------------------------------------------------------------

    if nargin < 2 || isempty(numSamples)
        numSamples = 200;
    end
    if nargin < 3 || isempty(windowSize)
        windowSize = 11;
    end
    if nargin < 4 || isempty(margin)
        margin = 8;
    end
    if nargin < 5 || isempty(showPlots)
        showPlots = true;
    end

    if mod(windowSize,2) == 0
        windowSize = windowSize + 1;
    end

    % --------------------------------------------------------
    % Step 1: Count header lines beginning with '#'
    % --------------------------------------------------------
    fid = fopen(filename, 'r');
    if fid == -1
        error('AnalyzeCenterline:FileOpenError', ...
              'Cannot open file: %s', filename);
    end

    headerLines = 0;
    while true
        pos = ftell(fid);
        line = fgetl(fid);

        if ~ischar(line)
            fclose(fid);
            error('AnalyzeCenterline:NoDataRows', ...
                  'File ended before any data rows were found.');
        end

        if startsWith(strtrim(line), '#')
            headerLines = headerLines + 1;
        else
            fseek(fid, pos, 'bof');
            break;
        end
    end
    fclose(fid);

    % --------------------------------------------------------
    % Step 2: Read data table
    % Standard .fcsv from Slicer often uses:
    % col1=id, col2=x, col3=y, col4=z, ...
    % --------------------------------------------------------
    tbl = readtable(filename, ...
        'FileType', 'text', ...
        'Delimiter', ',', ...
        'ReadVariableNames', false, ...
        'NumHeaderLines', headerLines);

    P = [tbl{:,2}, tbl{:,3}, tbl{:,4}];
    P = P(all(~isnan(P),2), :);

    if size(P,1) < 5
        error('AnalyzeCenterline:TooFewPoints', ...
              'Too few valid centerline points were imported.');
    end

    % --------------------------------------------------------
    % Step 3: Arc-length parameterization
    % --------------------------------------------------------
    dP = diff(P,1,1);
    ds = sqrt(sum(dP.^2,2));
    s_raw = [0; cumsum(ds)];

    % --------------------------------------------------------
    % Step 4: Uniform resampling
    % --------------------------------------------------------
    s_uniform = linspace(0, s_raw(end), numSamples).';
    Px = interp1(s_raw, P(:,1), s_uniform, 'spline');
    Py = interp1(s_raw, P(:,2), s_uniform, 'spline');
    Pz = interp1(s_raw, P(:,3), s_uniform, 'spline');
    P_resampled = [Px, Py, Pz];

    % --------------------------------------------------------
    % Step 5: Smooth centerline
    % --------------------------------------------------------
    Px_s = smoothdata(P_resampled(:,1), 'sgolay', windowSize);
    Py_s = smoothdata(P_resampled(:,2), 'sgolay', windowSize);
    Pz_s = smoothdata(P_resampled(:,3), 'sgolay', windowSize);
    P_smooth = [Px_s, Py_s, Pz_s];

    % --------------------------------------------------------
    % Step 6: Compute curvature with respect to arc length
    % --------------------------------------------------------
    d1 = zeros(size(P_smooth));
    d2 = zeros(size(P_smooth));

    for dim = 1:3
        d1(:,dim) = gradient(P_smooth(:,dim), s_uniform);
        d2(:,dim) = gradient(d1(:,dim), s_uniform);
    end

    kappa = zeros(size(P_smooth,1),1);
    for i = 1:size(P_smooth,1)
        num = norm(cross(d1(i,:), d2(i,:)));
        den = norm(d1(i,:))^3 + 1e-12;
        kappa(i) = num / den;
    end

    if numSamples <= 2*margin
        error('AnalyzeCenterline:InvalidMargin', ...
              'numSamples must be larger than 2*margin.');
    end

    [kappa_max, idx_local] = max(kappa(margin:end-margin));
    idx_max = idx_local + margin - 1;
    p_maxcurv = P_smooth(idx_max,:);

    % --------------------------------------------------------
    % Step 7: Package outputs
    % --------------------------------------------------------
    result = struct();
    result.P_raw = P;
    result.P_resampled = P_resampled;
    result.P_smooth = P_smooth;
    result.s_raw = s_raw;
    result.s_uniform = s_uniform;
    result.kappa = kappa;
    result.kappa_max = kappa_max;
    result.idx_max = idx_max;
    result.p_maxcurv = p_maxcurv;

    % --------------------------------------------------------
    % Step 8: Visualization
    % --------------------------------------------------------
    if ~showPlots
        return;
    end

    figure;
    plot3(P(:,1), P(:,2), P(:,3), 'o-', 'LineWidth', 1.2, 'MarkerSize', 4);
    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Original imported centerline');

    figure;
    plot3(P_resampled(:,1), P_resampled(:,2), P_resampled(:,3), ...
        '.-', 'LineWidth', 1.2, 'MarkerSize', 10);
    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Uniformly resampled centerline');

    figure;
    plot3(P_resampled(:,1), P_resampled(:,2), P_resampled(:,3), ...
        '--', 'LineWidth', 1); hold on;
    plot3(P_smooth(:,1), P_smooth(:,2), P_smooth(:,3), ...
        'b-', 'LineWidth', 2);
    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Resampled vs Smoothed centerline');
    legend('Resampled', 'Smoothed');

    figure;
    plot(s_uniform, kappa, 'LineWidth', 1.5); hold on;
    plot(s_uniform(idx_max), kappa_max, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    xlabel('Arc length');
    ylabel('Curvature');
    title('Vessel centerline curvature profile');
    grid on;
    legend('Curvature', 'Selected max curvature');

    figure;
    plot3(P_smooth(:,1), P_smooth(:,2), P_smooth(:,3), ...
        'b-', 'LineWidth', 2); hold on;
    plot3(p_maxcurv(1), p_maxcurv(2), p_maxcurv(3), ...
        'ro', 'MarkerSize', 10, 'LineWidth', 2);
    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Maximum-curvature point');
    legend('Centerline', 'Max-curvature point');
end
