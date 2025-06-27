// Minimal Hungarian (Munkres) algorithm implementation for rectangular cost matrices
// MIT License
#pragma once
#include <vector>
#include <limits>
#include <algorithm>

// Adapted for square/rectangular matrices, cost-minimizing
// assignment[i] = index of assigned column for row i, or -1 if unassigned
inline void hungarian(const std::vector<std::vector<double>>& cost,
                     std::vector<int>& assignment) {
    int nRows = cost.size();
    int nCols = nRows ? cost[0].size() : 0;
    int dim = std::max(nRows, nCols);
    std::vector<std::vector<double>> mat(dim, std::vector<double>(dim, 0));
    for (int i = 0; i < nRows; ++i)
        for (int j = 0; j < nCols; ++j)
            mat[i][j] = cost[i][j];
    // Pad with large values
    for (int i = nRows; i < dim; ++i)
        for (int j = 0; j < dim; ++j)
            mat[i][j] = 1e10;
    for (int i = 0; i < dim; ++i)
        for (int j = nCols; j < dim; ++j)
            mat[i][j] = 1e10;
    std::vector<double> u(dim), v(dim);
    std::vector<int> p(dim), way(dim);
    for (int i = 1; i < dim; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(dim, std::numeric_limits<double>::max());
        std::vector<char> used(dim);
        do {
            used[j0] = true;
            int i0 = p[j0], j1;
            double delta = std::numeric_limits<double>::max();
            for (int j = 1; j < dim; ++j) if (!used[j]) {
                double cur = mat[i0][j] - u[i0] - v[j];
                if (cur < minv[j]) minv[j] = cur, way[j] = j0;
                if (minv[j] < delta) delta = minv[j], j1 = j;
            }
            for (int j = 0; j < dim; ++j) {
                if (used[j]) u[p[j]] += delta, v[j] -= delta;
                else minv[j] -= delta;
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    assignment.assign(nRows, -1);
    for (int j = 1; j < dim; ++j) {
        if (p[j] > 0 && p[j] < nRows && j < nCols) // skip dummy row 0
            assignment[p[j]] = j;
    }
}
