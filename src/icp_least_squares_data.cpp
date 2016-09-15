// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "icp_least_squares_data.h"

#include "Eigen/Dense"

void Solve(const ICPLeastSquaresData& system, float x[6]) {
  Eigen::Matrix<float, 6, 6> a;
  Eigen::Matrix<float, 6, 1> b;
  int k = 0;
  for (int i = 0; i < 6; ++i) {
    for (int j = i; j < 6; ++j) {
      a(i, j) = system.a[k];
      a(j, i) = system.a[k];
      ++k;
    }
    b(i, 0) = system.b[i];
  }

  Eigen::Map<Eigen::Matrix<float, 6, 1>> x_map(x);
  x_map = a.colPivHouseholderQr().solve(b);
}
