#include <iostream>
#include <vector>
#include <cmath>

//TODO: anti floudering mechanisms
/*
Warning
any incidence of seemingly unexplained integers (eg. 49, 24 etc)
are due to the size of target matrix
and must be changed with target matrix size
*/

/*
This code is self explanatory
but has a lot of stuff inherent to the running platform
and some things that are constatnts when they should not be
*/

/*
this planner implements a "shit particle system"
that essentially marks a travelled area with a milder repulsive field
*/

// Constants for potential field
const double ATTRACTIVE_CONSTANT = 100.0;
const double REPULSIVE_CONSTANT = 150.0;
const double INFLUENCE_RADIUS = 5.0; // Radius around obstacles for repulsive force
const double SHIT_INFLUENCE = 0.7;

// Struct to represent a 2D point
struct Point
{
    int x, y;
    Point(int _x, int _y) : x(_x), y(_y) {}
};

// Function to calculate the attractive potential
//can be changed
double calculateAttractivePotential(const Point &pos, const Point &goal)
{
    //goal only
    double distance = std::sqrt((goal.x - pos.x) * (goal.x - pos.x) + (goal.y - pos.y) * (goal.y - pos.y));
    return 0.5 * ATTRACTIVE_CONSTANT *distance ;
}

//can be changed
// Function to calculate the repulsive potential
double calculateRepulsivePotential(const Point &pos, const std::vector<std::vector<int>> &grid)
{
    double repulsivePotential = 0.0;

    for (int i = 0; i < grid.size(); ++i)
    {
        for (int j = 0; j < grid[0].size(); ++j)
        {

            if (grid[i][j] == 100 or grid[i][j] == 2)
            { // obstacle or SHIT
                double distance = std::sqrt((i - pos.x) * (i - pos.x) + (j - pos.y) * (j - pos.y));

                if (distance <= INFLUENCE_RADIUS)
                {
                    double influence = (1.0 / distance) - (1.0 / INFLUENCE_RADIUS);
                    repulsivePotential += 0.5 * REPULSIVE_CONSTANT * influence * influence;
                }
            }
        }
    }

    return repulsivePotential;
}

// Function to calculate the total potential at a given position
double calculateTotalPotential(const Point &pos, const Point &goal, const std::vector<std::vector<int>> &grid)
{
    double attractive = calculateAttractivePotential(pos, goal);
    double repulsive = calculateRepulsivePotential(pos, grid);
    double boost_val = 1.0;
    if (grid[pos.x][pos.y] == 2)
    {
        boost_val = SHIT_INFLUENCE;
    }
    return boost_val * (attractive + repulsive);
}

bool isLineOfSight(const std::vector<std::vector<int>>& grid, const std::vector<int>& point1, const std::vector<int>& point2) {
    int x1 = point1[0];
    int y1 = point1[1];
    int x2 = point2[0];
    int y2 = point2[1];

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    std::cout<<"meoaw"<<std::endl;

    while (x1 != x2 || y1 != y2) {
        if (grid[x1][y1] == 100 or grid[x1-1][y1] == 100 or grid[x1+1][y1] == 100 or grid[x1][y1-1] == 100 or grid[x1][y1+1] == 100 or \
        grid[x1-1][y1-1] == 100 or grid[x1+1][y1+1] == 100 or grid[x1+1][y1-1] == 100 or grid[x1-1][y1+1] == 100) {  // If obstacle, return false
            return false;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    return (grid[x2][y2] == 0 or grid[x2][y2] == 2 or grid[x2][y2] == 3);  // Ensure the endpoint is not an obstacle
}

// Function to smooth the path by removing unnecessary points
std::vector<std::vector<int>> smoothPath(const std::vector<std::vector<int>>& path, std::vector<std::vector<int>>& grid) {
    if (path.size() <= 2) {
        return path;  // No simplification needed for small paths
    }

    std::vector<std::vector<int>> smoothedPath;
    smoothedPath.push_back(path[0]);  // Start with the first point

    std::vector<int> lastPoint = path[0];

    for (size_t i = 2; i < path.size(); ++i) {
        if (!isLineOfSight(grid, lastPoint, path[i])) {
            smoothedPath.push_back(path[i - 1]);
            lastPoint = path[i - 1];
            // grid[path[i][0]][path[i][1]]=0;
        }else{
            std::cout<<"Annhilated point ("<<path[i][0]<<" "<<path[i][1]<<")"<<std::endl;
            //TODO :: BETTER VISUZLIZATION
            grid[path[i][0]][path[i][1]]=3;
        }
    }
    for(int i=0; i<smoothedPath.size();i++){
        std::cout<<"Smooth point ("<<smoothedPath[i][0]<<" "<<smoothedPath[i][1]<<")"<<std::endl;
    }
    smoothedPath.push_back(path.back());  // Add the last point
    return smoothedPath;
}


// Function to get the next step in the path using gradient descent
//**equivalent to gradient descent 
Point getNextStep(const Point &currentPos, const Point &goal, std::vector<std::vector<int>> &grid)
{
    Point nextStep = currentPos;
    double minPotential = calculateTotalPotential(currentPos, goal, grid);

    // Check neighboring cells for the one with the lowest potential
    std::vector<Point> neighbors = {
        {currentPos.x + 1, currentPos.y},
        {currentPos.x - 1, currentPos.y},
        {currentPos.x, currentPos.y + 1},
        {currentPos.x, currentPos.y - 1},
        {currentPos.x + 1, currentPos.y + 1},
        {currentPos.x + 1, currentPos.y - 1},
        {currentPos.x - 1, currentPos.y + 1},
        {currentPos.x - 1, currentPos.y - 1}

    };

    for (const Point &neighbor : neighbors)
    {
        if (neighbor.x >= 0 && neighbor.x < grid.size() && neighbor.y >= 0 && neighbor.y < grid[0].size() && grid[neighbor.x][neighbor.y] != 1)
        {
            double potential = calculateTotalPotential(neighbor, goal, grid);
            if (potential < minPotential)
            {
                minPotential = potential;
                nextStep = neighbor;
            }
        }
    }
    grid[currentPos.x][currentPos.y] = 2;
    return nextStep;
}


std::vector<std::vector<int>> createMap(int width, int height)
{
    return std::vector<std::vector<int>>(height, std::vector<int>(width, 0));
}

// Function to plot a point on the map
void plotPoint(std::vector<std::vector<int>> &map, const Point &p)
{
    int x = static_cast<int>(std::round(p.x));
    int y = static_cast<int>(std::round(p.y));

    if (x >= 0 && x < map[0].size() && y >= 0 && y < map.size())
    {
        map[x][y] = 1; // Mark the point on the map
    }
}

// Function to calculate a Bezier curve point at parameter t (0 <= t <= 1)
Point calculateBezierPoint(double t, const std::vector<Point> &controlPoints)
{
    int n = controlPoints.size() - 1;
    Point result(0.0, 0.0);

    // Calculate the Bernstein polynomial
    for (int i = 0; i <= n; ++i)
    {
        double binomialCoeff = std::tgamma(n + 1) / (std::tgamma(i + 1) * std::tgamma(n - i + 1));
        double term = binomialCoeff * std::pow(1 - t, n - i) * std::pow(t, i);
        result.x += term * controlPoints[i].x;
        result.y += term * controlPoints[i].y;
    }

    return result;
}

// Function to plot a Bezier curve on the map
void plotBezierCurve(std::vector<std::vector<int>> &map, const std::vector<Point> &controlPoints, int numSamples)
{
    for (int i = 0; i <= numSamples; ++i)
    {
        double t = static_cast<double>(i) / numSamples;
        Point bezierPoint = calculateBezierPoint(t, controlPoints);
        plotPoint(map, bezierPoint);
    }
}

// Function to print the map
//Can be modded
//WARNING PRINT ONLY SINGLE DIGIT PIXELS


// Main function to plan the path using Predictive Potential Field (PPF)
std::vector<std::vector<int>> predictivePotentialFieldPlanner(std::vector<std::vector<int>> &grid, Point start, Point goal, int max_iter,float resolution,int smoothPasses)
{
    Point currentPos = start;
    std::vector<std::vector<int>> path;
    std::cout << "Starting at: (" << start.x << ", " << start.y << ")\n";
    std::cout << "Goal: (" << goal.x << ", " << goal.y << ")\n";
    int i = 0;
    while (currentPos.x != goal.x || currentPos.y != goal.y)
    {
        currentPos = getNextStep(currentPos, goal, grid);
        std::cout << "Moving to: (" << currentPos.x << ", " << currentPos.y << ")\n";
        path.push_back({currentPos.x, currentPos.y});
        i++;
        if (i > max_iter)
        {
            break;
        }
    }

    std::cout << "Reached the goal!\n";
    std::vector<std::vector<int>> smoothed_path;
    for(int i=0;i<smoothPasses;i++){
        smoothed_path=smoothPath(path,grid);
    }
    for(int i=0;i<smoothed_path.size();i++){
        grid[smoothed_path[i][0]][smoothed_path[i][1]]=7;
    }
    return smoothed_path;
}

// int main() {
//     // Define the grid (0 = free space, 1 = obstacle)
//     std::vector<std::vector<int>> grid = {
//         {0, 0, 0, 0, 0},
//         {0, 1, 0, 1, 0},
//         {0, 1, 0, 1, 0},
//         {0, 0, 0, 1, 0},
//         {0, 0, 0, 0, 0}
//     };
//     std::vector<std::vector<int>> matrix(100, std::vector<int>(100, 0));

//     Point start(10, 0);  // Start point
//     Point goal(50, 50);   // Goal point

//     std::vector<Point> controlPoints = {
//         Point(2, 2),
//         Point(50, 50),
//         Point(90, 80)
//     };
//      std::vector<Point> controlPoints1 = {
//         Point(2, 25),
//         Point(50, 75),
//         Point(60, 60)
//     };

//     // Plot a Bezier curve on the map with 100 samples
//     int numSamples = 1000;
//     plotBezierCurve(matrix, controlPoints, numSamples);
//     plotBezierCurve(matrix, controlPoints1, numSamples);
//     // Print the map with the plotted Bezier curve
//     // Run the Predictive Potential Field Planner
//     std::vector<std::vector<int>> path = predictivePotentialFieldPlanner(matrix, start, goal);
//     matrix[start.x][start.y] = 9;
//     matrix[goal.x][goal.y] = 9;
//     printMap(matrix);
//     return 0;
// }


// int main() {
//     // Example grid: 0 is free space, 1 is an obstacle
//     std::vector< std::vector<int>> grid = {
//         {0, 0, 1, 0, 0},
//         {0, 0, 1, 0, 0},
//         {0, 0, 1, 0, 0},
//         {0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0}
//     };

//     // Example path represented as vector of vector<int>
//      std::vector< std::vector<int>> path = {{0, 0}, {1, 2}, {2, 3}, {3, 3}, {4, 4}};

//     // Smooth the path
//      std::vector< std::vector<int>> smoothedPath = smoothPath(path, grid);

//     // Print the smoothed path
//      std::cout << "Original Path:" <<  std::endl;
//     for (const auto& p : path) {
//         std::cout << "(" << p[0] << ", " << p[1] << ") ";
//     }
//      std::cout <<  std::endl;

//      std::cout << "Smoothed Path:" <<  std::endl;
//     for (const auto& p : smoothedPath) {
//         std::cout << "(" << p[0] << ", " << p[1] << ") ";
//     }
//     std::cout <<  std::endl;

//     return 0;
// }