#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Node {
    int x, y;
    float cost, heuristic;

    bool operator<(const Node& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

bool isValid(int x, int y, const std::vector<std::vector<int>>& grid, int buffer = 1) {
    int rows = grid.size(), cols = grid[0].size();
    if (x < 0 || y < 0 || x >= rows || y >= cols || grid[x][y] != 0) return false;

    // Collision buffer around obstacles
    for (int i = -buffer; i <= buffer; ++i)
        for (int j = -buffer; j <= buffer; ++j)
            if (x + i >= 0 && y + j >= 0 && x + i < rows && y + j < cols && grid[x + i][y + j] != 0)
                return false;
    return true;
}

std::vector<std::pair<int, int>> getNeighbors(int x, int y) {
    return {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1},
            {x + 1, y + 1}, {x - 1, y - 1}, {x + 1, y - 1}, {x - 1, y + 1}};
}

float calculateHeuristic(int x, int y, int goalX, int goalY) {
    // Weighted Euclidean heuristic
    float dx = std::abs(x - goalX);
    float dy = std::abs(y - goalY);
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::pair<int, int>> smoothPath(const std::vector<std::pair<int, int>>& path) {
    if (path.size() <= 2) return path; // No smoothing needed for short paths.

    std::vector<std::pair<int, int>> smoothPath = {path[0]};
    for (size_t i = 1; i < path.size() - 1; ++i) {
        int prevX = smoothPath.back().first, prevY = smoothPath.back().second;
        int currX = path[i].first, currY = path[i].second;
        int nextX = path[i + 1].first, nextY = path[i + 1].second;

        // Bézier smoothing
        int smoothedX = (prevX + 2 * currX + nextX) / 4;
        int smoothedY = (prevY + 2 * currY + nextY) / 4;
        smoothPath.push_back({smoothedX, smoothedY});
    }
    smoothPath.push_back(path.back());
    return smoothPath;
}

std::vector<std::pair<int, int>> aStarPathPlanning(
    const std::vector<std::vector<int>>& grid,
    std::pair<int, int> start,
    std::pair<int, int> goal) 
{
    std::priority_queue<Node> openList;
    std::vector<std::vector<bool>> visited(grid.size(), std::vector<bool>(grid[0].size(), false));
    std::vector<std::vector<std::pair<int, int>>> parent(grid.size(), std::vector<std::pair<int, int>>(grid[0].size(), {-1, -1}));

    openList.push({start.first, start.second, 0, calculateHeuristic(start.first, start.second, goal.first, goal.second)});
    visited[start.first][start.second] = true;

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        if (current.x == goal.first && current.y == goal.second) {
            std::vector<std::pair<int, int>> path;
            for (std::pair<int, int> at = goal; at != std::make_pair(-1, -1); at = parent[at.first][at.second])
                path.push_back(at);
            std::reverse(path.begin(), path.end());
            return smoothPath(path);
        }

        for (const auto& [nx, ny] : getNeighbors(current.x, current.y)) {
            if (isValid(nx, ny, grid, 1) && !visited[nx][ny]) {
                visited[nx][ny] = true;
                parent[nx][ny] = {current.x, current.y};
                float newCost = current.cost + (nx == current.x || ny == current.y ? 1.0f : 1.414f); // Diagonal cost adjustment
                openList.push({nx, ny, newCost, calculateHeuristic(nx, ny, goal.first, goal.second)});
            }
        }
    }

    return {}; // Return empty path if no path is found.
}

int main() {
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 1},
        {0, 1, 1, 0, 1},
        {0, 0, 0, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 0, 0, 0}
    };

    std::pair<int, int> start = {0, 0};
    std::pair<int, int> goal = {4, 4};

    auto path = aStarPathPlanning(grid, start, goal);

    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (const auto& [x, y] : path)
            std::cout << "(" << x << ", " << y << ") ";
        std::cout << "\n";
    } else {
        std::cout << "No path found.\n";
    }

    return 0;
}
