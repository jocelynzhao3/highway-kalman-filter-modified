# Highway Kalman Filter (Modified)

This project is based on [moorissa/highway-kalman-filter](https://github.com/moorissa/highway-kalman-filter).

## Improvements

1. **Optimized Type System** (`optimization_config.h`)
   - Introduced fixed-size Eigen types for state and measurement vectors
   - Pre-allocated matrices with compile-time dimensions for performance optimizations
   - Reduced total highway computation time from 1.0668s to 1.02373s (~4% improvement) under the same compiler optimizations

2. **UKF bug fix** (`ukf.cpp`)
   - Line 245, original code overwrite sigma matrix too early, new code updates correctly (line 322)
   - Fixing the overwriting of sigma initialization causes major RMSE improvement (~70-90%):

    Before:
    ```
    Average RMSE: [14.6389, 1.65034, 5.35352, 0.773409]  // [x, y, vx, vy]
    ```
    After:
    ```
    Average RMSE: [0.454338, 0.14265, 1.7057, 0.662087]  // [x, y, vx, vy]
    ```

## Usage

```bash
# Build
cd highway-kalman-filter
mkdir -p build && cd build
cmake ..
make

# Run
./ukf_highway
```
