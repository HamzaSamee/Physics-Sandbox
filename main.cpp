#include "raylib.h"
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <stack>
#include <cmath>

using std::string;
using std::vector;
using std::unique_ptr;
using std::make_unique;
using std::unordered_map;
using std::stack;

// ============================================================================
// CONSTANTS & COLORS
// ============================================================================
const Color PrimaryDark = { 28, 40, 51, 255 };
const Color SecondaryGold = { 255, 184, 0, 255 };
const Color LightBackground = { 240, 240, 240, 255 };
const Color AccentRed = { 231, 76, 60, 255 };
const Color AccentBlue = { 52, 152, 219, 255 };
const Color AccentGreen = { 46, 204, 113, 255 };
const Color TextGray = { 80, 80, 80, 255 };

const int screenWidth = 1300;
const int screenHeight = 720;
const int canvasWidth = 850;
const int panelX = canvasWidth + 1;
const int panelWidth = screenWidth - canvasWidth - 1;

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct DataPoint {
    double time{ 0.0 };
    double position_x{ 0.0 };
    double position_y{ 0.0 };
    double velocity_x{ 0.0 };
    double velocity_y{ 0.0 };
    double kinetic_energy{ 0.0 };
    double potential_energy{ 0.0 };
    double pressure{ 0.0 };
};

struct Ball {
    double x{ 0.0 }, y{ 0.0 };
    double vx{ 0.0 }, vy{ 0.0 };
    double radius{ 0.5 }, mass{ 1.0 };
    int id{ 0 };
};

struct ExperimentParameters {
    string experiment_type = "CollisionBalls";
    double gravity = 9.81;
    double air_resistance = 0.05;
    double mass = 2.0;
    double length = 2.0;
    double initial_angle = 45.0;
    double spring_constant = 50.0;
    double ball_masses[2] = { 2.0, 1.5 };
    double ball_radii[2] = { 0.6, 0.5 };
    double initial_velocities_x[2] = { 4.0, -3.0 };
    double initial_velocities_y[2] = { 0.0, 0.0 };
    double fluid_density = 1000.0;
    double pipe_diameter1 = 0.15;
    double pipe_diameter2 = 0.06;
    double fluid_velocity = 2.0;
    double static_pressure = 101325.0;
    double projectile_speed = 25.0;
    double projectile_angle = 45.0;
};


// ============================================================================
// GRAPH (for collision tracking)
// ============================================================================
struct GraphNode
{
    int ball_id;
    int collision_count = 0;
    vector<int> neighbors;
};

class CollisionGraph
{
private:
    vector<GraphNode> nodes;

public:
    void setup(int count)
    {
        nodes.clear();
        for (int i = 0; i < count; ++i)
            nodes.push_back({i, 0, {}});
    }

    void registerCollision(int id1, int id2)
    {
        if (id1 == id2)
            return;

        bool connected = false;
        for (int n : nodes[id1].neighbors)
        {
            if (n == id2)
            {
                connected = true;
                break;
            }
        }

        if (!connected)
        {
            nodes[id1].neighbors.push_back(id2);
            nodes[id2].neighbors.push_back(id1);
        }

        nodes[id1].collision_count++;
        nodes[id2].collision_count++;
    }

    const vector<GraphNode> &getNodes() const { return nodes; }
};

// ============================================================================
// BST (Binary Search Tree for energy tracking)
// ============================================================================
struct BSTNode
{
    double key, value_time;
    BSTNode *left;
    BSTNode *right;
    BSTNode(double k, double t) : key(k), value_time(t), left(nullptr), right(nullptr) {}
};

class EnergyBST
{
private:
    BSTNode *root;
    int node_count;

    BSTNode *insertRecursive(BSTNode *node, double key, double time)
    {
        if (!node)
        {
            node_count++;
            return new BSTNode(key, time);
        }
        if (key < node->key)
            node->left = insertRecursive(node->left, key, time);
        else if (key > node->key)
            node->right = insertRecursive(node->right, key, time);
        return node;
    }

    void destroyRecursive(BSTNode *node)
    {
        if (node)
        {
            destroyRecursive(node->left);
            destroyRecursive(node->right);
            delete node;
        }
    }

public:
    EnergyBST() : root(nullptr), node_count(0) {}
    ~EnergyBST() { destroyRecursive(root); }

    void insert(double key, double time)
    {
        root = insertRecursive(root, key, time);
    }

    void clear()
    {
        destroyRecursive(root);
        root = nullptr;
        node_count = 0;
    }

    int getNodeCount() const { return node_count; }
};

// ============================================================================
// HASHMAP (for analysis data)
// ============================================================================
class AnalysisData
{
public:
    unordered_map<string, double> max_metrics;

    AnalysisData()
    {
        max_metrics["Max_KE"] = 0.0;
        max_metrics["Max_Velocity"] = 0.0;
    }

    void updateMaxKE(double ke)
    {
        if (ke > max_metrics["Max_KE"])
            max_metrics["Max_KE"] = ke;
    }

    void updateMaxVelocity(double v)
    {
        if (v > max_metrics["Max_Velocity"])
            max_metrics["Max_Velocity"] = v;
    }
};
