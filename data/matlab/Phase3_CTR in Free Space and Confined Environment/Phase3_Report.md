# Geometry-Aware Analysis of 3-Tube CTR

## 1. Simplified CTR Model

The study uses the same simplified piecewise constant-curvature CTR model as the previous phase. Tube transition points are defined from each tube insertion value and curved length. Each resulting link is modeled as a constant-curvature segment, and the full CTR backbone is formed by chaining link transforms in `Robot.m`.

The missing `Tube.m` class was added so the provided scripts can construct tube geometry, precurvature, elastic modulus, and second moment of area.

## 2. Free-Space Workspace Analysis

Implemented in `WorkspaceStudy.m`.

Sampling convention:

- `q = [rho1 rho2 rho3 theta1 theta2 theta3]`
- translations are in mm
- rotations are in degrees
- `rho1 = 0` mm fixed
- `rho2 = 0:20:60` mm
- `rho3 = 0:20:80` mm
- `theta1`, `theta2`, `theta3 = -90:45:90` degrees

Metrics:

- most straight: smallest RMS lateral backbone deviation from the base z-axis
- most bent: largest maximum lateral backbone deviation from the base z-axis

Representative configurations from the validation run:

| Case | q = [rho1 rho2 rho3 theta1 theta2 theta3] | Metric | Tip position (m) |
|---|---:|---:|---:|
| Most straight | `[0 0 0 -90 90 90]` | RMS lateral `8.7852e-05` m | `[1.5387e-18, -0.00019644, 0.049999]` |
| Most bent | `[0 0 80 -45 -45 -45]` | Max lateral `0.087165` m | `[0.061635, -0.061635, 0.065972]` |

The script saves `WorkspaceSamples.mat` and plots the tip workspace plus representative backbones using `sample_ctr_centerline.m`.

## 3. Vessel Centerline Analysis

Implemented in `CurvatureFeasibilityStudy.m`.

The script locates the centerline `.fcsv` file by pattern, allowing for the current filename `Centerline curve_3 (0).fcsv`. It uses `AnalyzeCenterline.m` to read, resample, smooth, and compute the curvature profile. It also loads the quantification CSV, schema CSV, endpoints FCSV, and vessel STL when available.

Maximum vessel curvature from the validation run:

- `0.054729 1/mm`
- `54.729 1/m`

The highest-curvature location is marked on a 3D vessel centerline plot, and curvature versus arc length is plotted.

## 4. Curvature Feasibility

The CTR curvature capability was estimated by a computational scan:

- `rho1 = 0` mm
- `rho2 = 0:20:60` mm
- `rho3 = 0:20:80` mm
- `theta1`, `theta2`, `theta3 = -180:45:180` degrees

The scanned metric was:

```text
max_q max_j kappa_j(q)
```

Validation-run result:

- maximum CTR coupled curvature: `29 1/m`
- maximum vessel curvature: `54.729 1/m`
- ratio `R = 0.52988`

Interpretation: under this simplified geometric model and this data scale, the CTR is unlikely to match the strongest local vessel curvature.

## 5. Local Segment Fitting

Implemented in `FitLocalSegment.m`.

The fitted vessel segment is centered at the maximum-curvature point from Part B. Segment length is `90 mm`, chosen to match the shortest tube curved length. The script:

- extracts the local segment using `ExtractLocalSegment.m`
- generates CTR backbones using `sample_ctr_centerline.m`
- aligns the initial CTR tangent to the initial vessel tangent using `rotation_from_vectors.m`
- minimizes the pointwise mean squared trajectory error by coarse grid search

Fitting cost:

```text
J(q) = (1/N) sum_k ||p_CTR(k; q) - p_vessel(k)||^2
```

Best validation-run result:

| qbest | MSE (mm^2) | Mean error (mm) | Max error (mm) |
|---:|---:|---:|---:|
| `[0 0 0 0 0 0]` | `49.223` | `5.1518` | `14.442` |

The result suggests the simple free-space CTR backbone cannot closely fit the selected high-curvature vessel segment.

## 6. Limitations

This is a geometric analysis only. It does not model contact, friction, torsion-aware mechanics, tissue interaction, vessel deformation, collision, or clinical feasibility. The vessel coordinates are in mm while the CTR model uses meters, so scripts explicitly convert units for fitting and curvature comparison.

## Provided Utility Usage

- `AnalyzeCenterline.m`: centerline loading, smoothing, arc-length parameterization, curvature computation
- `ExtractLocalSegment.m`: local segment extraction near maximum curvature
- `sample_ctr_centerline.m`: full CTR backbone generation
- `build_single_link_transform.m`: link transform helper used by backbone sampling
- `rotation_from_vectors.m`: tangent alignment for local fitting
- `Robot.m`: CTR forward kinematics and coupled link curvature computation
