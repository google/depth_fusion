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
#ifndef ICP_LEAST_SQUARES_DATA_H
#define ICP_LEAST_SQUARES_DATA_H

struct ICPLeastSquaresData {
  // Packed storage for the symmetric covariance matrix.
  float a[21];

  // Column major storage for the right hand side vector.
  float b[6];

  // Aka the L2 energy.
  float squared_residual;

  // The number of samples accumulated to form a and b.
  int num_samples;
};

void Solve(const ICPLeastSquaresData& system, float x[6]);

#endif // ICP_LEAST_SQUARES_DATA_H
