# Localization
## Building files
Ensure vcpkg is installed already.
```bash
external/vcpkg/vcpkg install
```

To compile (first time or after adding dependencies in `vcpkg.json`)
```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=${PWD}/external/vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_FEATURE_FLAGS=manifests \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Build the package
```bash
cmake --build build -j
```

## Adding New Package
Add your package name in the dependencies array
```json
{
  "name": "autoronto-localization",
  "version-string": "0.1.0",
  "dependencies": [
    "opencv4",
    "eigen3"
  ]
}
```

Then, add the dependency inside the `CMAKELists.txt` for static type checkers.
```
find_package(Eigen3 CONFIG REQUIRED)

target_link_libraries(app PRIVATE Eigen3::Eigen)
```

After that, don't forget to rebuild the `vkpkg` & `cmake`.

## Testing
Run the python test file to generate testing images & corresponding csv.
```bash
chmod +x test.sh
chmod +x test_gcc.sh
```

For testing with clang
```bash
./test.sh
```

For testing with gcc
```bash
./test_gcc.sh
```

### Test Outputs
| ID | Geometry (mask vs prior) | What it validates | Typical args | Expected result | Debug image |
|---|---|---|---|---|---|
| **01_good_line** | Straight line; prior matches | Happy path: ROI rasterization + dilation + LS fit with high R² | `--roi 10 --horizon_frac 0.67` | **Pass** (numeric `offset_m`, `heading_rad`, `r2≈1.0`) | Yes |
| **02_steeper_line** | Straighter but steeper slope; prior matches | Same as 01 with different slope; regression stability | `--roi 10 --horizon_frac 0.67` | **Pass** (high R²) | Yes |
| **03_few_points_horizon** | Same as 01, but most pixels cropped by horizon | “< 20 points” guard in pipeline (after ROI∩mask∩horizon) | `--roi 10 --horizon_frac 0.98` | **Fail** (NaNs or low R²; sometimes prints `r2=0.0` if early exit) | No |
| **04_vertical_line** | Vertical lane (constant x); prior matches | R² computation guard when `SS_tot≈0` (no x variance) | `--roi 10 --horizon_frac 0.67` | **Fail** (R²→0 → rejected → NaNs) | No |
| **05_noisy_low_r2** | Straight line with heavy x-noise; prior is clean | Low-R² rejection path in `fitLineXY` | `--roi 10 --horizon_frac 0.67` | **Fail** (R² < 0.90 → NaNs) | No |
| **06a_needs_dilation** | Mask shifted +12 px; prior unshifted | Dilation width matters: narrow tube misses overlap | `--roi 3 --horizon_frac 0.67` | **Fail** (few/zero points or low R²) | No |
| **06b_needs_dilation** | Same as 06a | Dilation recovers overlap with wider tube | `--roi 10 --horizon_frac 0.67` | **Pass** (high R²) | Yes |
| **07_wrong_prior** | Mask and prior far apart (±100 px) | Detects “no overlap” with ROI even if wide | `--roi 20 --horizon_frac 0.67` | **Fail** (often `<20` points → `r2=0.0` or NaNs) | No |
| **08_curved_lane** | Quadratic (curved) lane; straight prior | Model mismatch → LS line fit gives low R² | `--roi 10 --horizon_frac 0.67` | **Fail** (R² < 0.90 → NaNs) | No |


### Artifacts per case
`mask.png` — binary ground-truth lane pixels (uint8; 0/255).

`prior.csv` — sparse prior polyline: rows u,v (x,y in pixels).

`debug.png` — only written for pass runs when --debug is provided: shows mask (gray), ROI tube (blue), fitted line (yellow), centerline (green), and the evaluation point (red).
