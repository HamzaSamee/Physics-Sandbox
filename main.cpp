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

// ============================================================================
// PHYSICS UTILITIES
// ============================================================================
inline double degToRad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

inline double calcKE(double m, double vx, double vy) {
    return m > 0 ? 0.5 * m * (vx * vx + vy * vy) : 0;
}

inline double calcPE(double m, double g, double h) {
    return m > 0 ? m * g * h : 0;
}

// ============================================================================
// EXPERIMENT INTERFACE
// ============================================================================
class IExperiment {
public:
    virtual ~IExperiment() = default;
    virtual void setup(const ExperimentParameters& params) = 0;
    virtual void update(double dt) = 0;
    virtual double time() const = 0;
    virtual const vector<DataPoint>& dataLog() const = 0;
    virtual vector<string> getActiveDataStructures() const = 0;
    virtual vector<string> getCurrentPhysicsInfo() const = 0;
    virtual string getExperimentName() const = 0;
};

// ============================================================================
// FREEFALL EXPERIMENT
// ============================================================================
class FreeFall : public IExperiment {
private:
    ExperimentParameters p;
    double t{ 0 };
    static const int count = 3;
    double h[count], v[count];
    vector<DataPoint> log;

public:
    void setup(const ExperimentParameters& params) override {
        p = params;
        t = 0;
        log.clear();
        for (int i = 0; i < count; ++i) {
            h[i] = 30.0 - i * 5.0;
            v[i] = 0;
        }
    }

    void update(double dt) override {
        if (dt <= 0) return;
        t += dt;

        for (int i = 0; i < count; ++i) {
            double a = p.gravity - p.air_resistance * v[i];
            v[i] += a * dt;
            h[i] -= v[i] * dt;

            if (h[i] < 0) {
                h[i] = 0;
                v[i] = -v[i] * 0.75;
            }

            if (i == 0) {
                double ke = calcKE(p.mass, 0, v[i]);
                double pe = calcPE(p.mass, p.gravity, h[i]);
                log.push_back({ t, 0, h[i], 0, v[i], ke, pe });
            }
        }
    }

    vector<string> getActiveDataStructures() const override {
        return { "Array (h[3], v[3])", "Vector (DataPoints)", "BST (Energy)", "HashMap (Max)" };
    }

    vector<string> getCurrentPhysicsInfo() const override {
        return {
            "Mass: " + std::to_string(p.mass) + " kg",
            "Gravity: " + std::to_string(p.gravity) + " m/s²",
            "Height: " + std::to_string(h[0]) + " m",
            "Velocity: " + std::to_string(v[0]) + " m/s",
            "Air Resist: " + std::to_string(p.air_resistance)
        };
    }

    string getExperimentName() const override { return "Free Fall - Three Balls"; }
    double time() const override { return t; }
    const vector<DataPoint>& dataLog() const override { return log; }
    int getCount() const { return count; }
    double getHeight(int i) const { return (i >= 0 && i < count) ? h[i] : 0; }
    double getVelocity(int i) const { return (i >= 0 && i < count) ? v[i] : 0; }
    ExperimentParameters& getParams() { return p; }
};

// ============================================================================
// PENDULUM EXPERIMENT
// ============================================================================
class Pendulum : public IExperiment {
private:
    ExperimentParameters p;
    double t{ 0 };
    double angle{ 0 }, w{ 0 };
    vector<DataPoint> log;

public:
    void setup(const ExperimentParameters& params) override {
        p = params;
        t = 0;
        angle = degToRad(p.initial_angle);
        w = 0;
        log.clear();
    }

    void update(double dt) override {
        if (dt <= 0) return;
        t += dt;

        double a = -(p.gravity / p.length) * sin(angle) - p.air_resistance * w;
        w += a * dt;
        angle += w * dt;

        double bx = p.length * sin(angle);
        double by = p.length * cos(angle);
        double vx = w * p.length * cos(angle);
        double vy = -w * p.length * sin(angle);

        double ke = calcKE(p.mass, vx, vy);
        double pe = calcPE(p.mass, p.gravity, p.length - p.length * cos(angle));

        log.push_back({ t, bx, by, vx, vy, ke, pe });
    }

    vector<string> getActiveDataStructures() const override {
        return { "Variables (angle, w)", "Vector (DataPoints)", "BST (Energy)", "HashMap (Max)" };
    }

    vector<string> getCurrentPhysicsInfo() const override {
        return {
            "Mass: " + std::to_string(p.mass) + " kg",
            "Length: " + std::to_string(p.length) + " m",
            "Angle: " + std::to_string(angle * 180 / 3.14159) + "°",
            "Angular Vel: " + std::to_string(w) + " rad/s",
            "Gravity: " + std::to_string(p.gravity) + " m/s²"
        };
    }

    string getExperimentName() const override { return "Simple Pendulum"; }
    double time() const override { return t; }
    const vector<DataPoint>& dataLog() const override { return log; }
    double getBobX() const { return p.length * sin(angle); }
    double getBobY() const { return p.length * cos(angle); }
    ExperimentParameters& getParams() { return p; }
};


// ============================================================================
// SPRING SYSTEM EXPERIMENT
// ============================================================================
class SpringSystem : public IExperiment
{
private:
    ExperimentParameters p;
    double t{0};
    double x{2.0}, v{0};
    vector<DataPoint> log;

public:
    void setup(const ExperimentParameters &params) override
    {
        p = params;
        t = 0;
        x = 2.0;
        v = 0;
        log.clear();
    }

    void update(double dt) override
    {
        if (dt <= 0)
            return;
        t += dt;

        double f = -p.spring_constant * x - p.air_resistance * v * 5.0;
        double a = f / p.mass;
        v += a * dt;
        x += v * dt;

        double ke = 0.5 * p.mass * v * v;
        double pe = 0.5 * p.spring_constant * x * x;

        log.push_back({t, x, 0, v, 0, ke, pe});
    }

    vector<string> getActiveDataStructures() const override
    {
        return {"Variables (x, v)", "Vector (DataPoints)", "BST (Energy)", "HashMap (Max)"};
    }

    vector<string> getCurrentPhysicsInfo() const override
    {
        return {
            "Mass: " + std::to_string(p.mass) + " kg",
            "Spring k: " + std::to_string(p.spring_constant) + " N/m",
            "Displacement: " + std::to_string(x) + " m",
            "Velocity: " + std::to_string(v) + " m/s",
            "Force: " + std::to_string(-p.spring_constant * x) + " N"};
    }

    string getExperimentName() const override { return "Spring-Mass System"; }
    double time() const override { return t; }
    const vector<DataPoint> &dataLog() const override { return log; }
    double getDisplacement() const { return x; }
    ExperimentParameters &getParams() { return p; }
};

// ============================================================================
// COLLISION BALLS EXPERIMENT
// ============================================================================
class CollisionBalls : public IExperiment
{
private:
    ExperimentParameters p;
    double t{0};
    static const int count = 2;
    Ball balls[count];
    vector<DataPoint> log;
    CollisionGraph graph;

    void handleCollisions()
    {
        double dx = balls[0].x - balls[1].x;
        double dy = balls[0].y - balls[1].y;
        double dist = sqrt(dx * dx + dy * dy);
        double minD = balls[0].radius + balls[1].radius;

        if (dist < minD && dist > 0)
        {
            double m1 = balls[0].mass, m2 = balls[1].mass;
            double v1 = balls[0].vx, v2 = balls[1].vx;

            balls[0].vx = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
            balls[1].vx = ((m2 - m1) * v2 + 2 * m1 * v1) / (m1 + m2);

            double overlap = minD - dist;
            if (overlap > 0)
            {
                if (balls[0].x < balls[1].x)
                {
                    balls[0].x -= overlap / 2;
                    balls[1].x += overlap / 2;
                }
                else
                {
                    balls[0].x += overlap / 2;
                    balls[1].x -= overlap / 2;
                }
            }

            graph.registerCollision(0, 1);
        }
    }

public:
    void setup(const ExperimentParameters &params) override
    {
        p = params;
        t = 0;
        log.clear();

        for (int i = 0; i < count; ++i)
        {
            balls[i] = {
                5.0 + i * 5.0, 5.0,
                p.initial_velocities_x[i], 0,
                p.ball_radii[i], p.ball_masses[i], i};
        }

        graph.setup(count);
    }

    void update(double dt) override
    {
        if (dt <= 0)
            return;
        t += dt;

        for (int i = 0; i < count; ++i)
        {
            balls[i].x += balls[i].vx * dt;

            if (balls[i].x < balls[i].radius)
            {
                balls[i].x = balls[i].radius;
                balls[i].vx *= -1;
            }
            if (balls[i].x > 15 - balls[i].radius)
            {
                balls[i].x = 15 - balls[i].radius;
                balls[i].vx *= -1;
            }
        }

        handleCollisions();

        double ke = calcKE(balls[0].mass, balls[0].vx, balls[0].vy);
        log.push_back({t, balls[0].x, balls[0].y, balls[0].vx, balls[0].vy, ke, 0});
    }

    vector<string> getActiveDataStructures() const override
    {
        return {"Array (Ball[2])", "Graph (Collisions)", "Vector (DataPoints)", "BST (Energy)", "HashMap (Max)"};
    }

    vector<string> getCurrentPhysicsInfo() const override
    {
        int total = 0;
        for (const auto &n : graph.getNodes())
            total += n.collision_count;

        return {
            "Ball 1 Mass: " + std::to_string(balls[0].mass) + " kg",
            "Ball 2 Mass: " + std::to_string(balls[1].mass) + " kg",
            "Ball 1 Vel: " + std::to_string(balls[0].vx) + " m/s",
            "Ball 2 Vel: " + std::to_string(balls[1].vx) + " m/s",
            "Collisions: " + std::to_string(total / 2)};
    }

    string getExperimentName() const override { return "Elastic Collision - Two Balls"; }
    double time() const override { return t; }
    const vector<DataPoint> &dataLog() const override { return log; }
    const CollisionGraph &getGraph() const { return graph; }
    int getCount() const { return count; }
    const Ball *getBalls() const { return balls; }
    Ball *getBallsMutable() { return balls; }
    ExperimentParameters &getParams() { return p; }
};


