# nav2_lyapunov_stable_controller

## Example Configuration

```yaml
FollowPath:
    plugin: "nav2_lyapunov_stable_controller::LyapunovStableController"
    desired_linear_vel: 3.0
    max_angular_vel: 1.0
    transform_tolerance: 1.0
    lookahead_dist: 0.6
    max_angular_drift: 0.3
    use_collision_detection: true
    max_allowed_time_to_collision: 1.0
    k_linear: 1.0
    k_angular: 3.0
```