
namespace nav2_lyapunov_stable_controller {
template <typename T>
int sign(T value) {
    return (value > 0) - (value < 0);
}
}