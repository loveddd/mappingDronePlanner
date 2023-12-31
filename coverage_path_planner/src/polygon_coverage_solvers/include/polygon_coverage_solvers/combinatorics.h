/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef POLYGON_COVERAGE_SOLVERS_COMBINATORICS_H_
#define POLYGON_COVERAGE_SOLVERS_COMBINATORICS_H_

#include <cstddef>
#include <set>
#include <vector>
#include "ros_interface.h"

namespace polygon_coverage_planning {

// Calculate the factorial
inline int factorial(int n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

// Calculate the binomial coefficient "n choose k".
inline int nChooseK(int n, int k) {
  return factorial(n) / (factorial(k) * factorial(n - k));
}

// Pick all of the k-element combinations from an ordered set.
void getAllCombinationsOfKElementsFromN(
    const std::vector<size_t>& sorted_elements, int k,
    std::vector<std::set<size_t>>* result);
// The recursive call.
void getAllCombinationsOfKElementsFromN(
    const std::vector<size_t>& sorted_elements, int k, int start_pos,
    std::vector<size_t>* combination, std::vector<std::set<size_t>>* result);

}  // namespace polygon_coverage_planning

#endif /* POLYGON_COVERAGE_SOLVERS_COMBINATORICS_H_ */
