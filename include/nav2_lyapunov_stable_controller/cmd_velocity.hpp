// Copyright (c) 2024 LonelyPaprika
// Author (s) Théo Combelles <theo.combelles@gmail.com>
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_LYAPUNOV_STABLE_CONTROLLER__CMD_VELOCITY_HPP_
#define NAV2_LYAPUNOV_STABLE_CONTROLLER__CMD_VELOCITY_HPP_

namespace nav2_lyapunov_stable_controller {

struct CmdVelocity {
    double linear_vel;
    double angular_vel;
};
}  // namespace nav2_lyapunov_stable_controller
#endif  // NAV2_LYAPUNOV_STABLE_CONTROLLER__CMD_VELOCITY_HPP_
