// physics_sandbox_enhanced.cpp
// Refactored and enhanced physics simulation with live controls
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
// UI CONSTANTS
// ============================================================================
const Color COLOR_PRIMARY_DARK = { 28, 40, 51, 255 };
const Color COLOR_SECONDARY_GOLD = { 255, 184, 0, 255 };
const Color COLOR_LIGHT_BG = { 240, 240, 240, 255 };
const Color COLOR_ACCENT_RED = { 231, 76, 60, 255 };
const Color COLOR_ACCENT_BLUE = { 52, 152, 219, 255 };
const Color COLOR_ACCENT_GREEN = { 46, 204, 113, 255 };
const Color COLOR_TEXT_GRAY = { 80, 80, 80, 255 };

const int SCREEN_WIDTH = 1300;
const int SCREEN_HEIGHT = 720;
const int CANVAS_WIDTH = 850;
const int PANEL_X = CANVAS_WIDTH + 1;
const int PANEL_WIDTH = SCREEN_WIDTH - CANVAS_WIDTH - 1;

// Physics constants
const double GRAVITY_DEFAULT = 9.81;
const double PROJECTILE_OPTIMAL_ANGLE = 38.0; // Adjusted to land near boundary

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
    double gravity = GRAVITY_DEFAULT;
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
    double projectile_angle = PROJECTILE_OPTIMAL_ANGLE;
};

// ============================================================================
// COLLISION GRAPH (for tracking ball collisions)
// ============================================================================
struct GraphNode {
    int ball_id;
    int collision_count = 0;
    vector<int> neighbors;
};

class CollisionGraph {
private:
    vector<GraphNode> nodes;

public:
    void setup(int ball_count) {
        nodes.clear();
        for (int i = 0; i < ball_count; ++i) {
            nodes.push_back({ i, 0, {} });
        }
    }

    void registerCollision(int ball_id_1, int ball_id_2) {
        if (ball_id_1 == ball_id_2) return;

        // Check if connection already exists
        bool already_connected = false;
        for (int neighbor : nodes[ball_id_1].neighbors) {
            if (neighbor == ball_id_2) {
                already_connected = true;
                break;
            }
        }

        // Add bidirectional edge if not connected
        if (!already_connected) {
            nodes[ball_id_1].neighbors.push_back(ball_id_2);
            nodes[ball_id_2].neighbors.push_back(ball_id_1);
        }

        nodes[ball_id_1].collision_count++;
        nodes[ball_id_2].collision_count++;
    }

    const vector<GraphNode>& getNodes() const { return nodes; }
};

// ============================================================================
// BINARY SEARCH TREE (for energy tracking)
// ============================================================================
struct BSTNode {
    double key;
    double value_time;
    BSTNode* left;
    BSTNode* right;

    BSTNode(double k, double t) : key(k), value_time(t), left(nullptr), right(nullptr) {}
};

class EnergyBST {
private:
    BSTNode* root;
    int node_count;

    BSTNode* insertRecursive(BSTNode* node, double key, double time) {
        if (!node) {
            node_count++;
            return new BSTNode(key, time);
        }
        if (key < node->key) {
            node->left = insertRecursive(node->left, key, time);
        }
        else if (key > node->key) {
            node->right = insertRecursive(node->right, key, time);
        }
        return node;
    }

    void destroyRecursive(BSTNode* node) {
        if (node) {
            destroyRecursive(node->left);
            destroyRecursive(node->right);
            delete node;
        }
    }

public:
    EnergyBST() : root(nullptr), node_count(0) {}
    ~EnergyBST() { destroyRecursive(root); }

    void insert(double key, double time) {
        root = insertRecursive(root, key, time);
    }

    void clear() {
        destroyRecursive(root);
        root = nullptr;
        node_count = 0;
    }

    int getNodeCount() const { return node_count; }
};

// ============================================================================
// HASH MAP (for analysis metrics)
// ============================================================================
class AnalysisData {
public:
    unordered_map<string, double> max_metrics;

    AnalysisData() {
        max_metrics["Max_KE"] = 0.0;
        max_metrics["Max_Velocity"] = 0.0;
    }

    void updateMaxKE(double kinetic_energy) {
        if (kinetic_energy > max_metrics["Max_KE"]) {
            max_metrics["Max_KE"] = kinetic_energy;
        }
    }

    void updateMaxVelocity(double velocity) {
        if (velocity > max_metrics["Max_Velocity"]) {
            max_metrics["Max_Velocity"] = velocity;
        }
    }
};

// ============================================================================
// PHYSICS HELPER FUNCTIONS
// ============================================================================
inline double degreesToRadians(double degrees) {
    return degrees * 3.14159265358979323846 / 180.0;
}

inline double calculateKineticEnergy(double mass, double vel_x, double vel_y) {
    return mass > 0 ? 0.5 * mass * (vel_x * vel_x + vel_y * vel_y) : 0.0;
}

inline double calculatePotentialEnergy(double mass, double gravity, double height) {
    return mass > 0 ? mass * gravity * height : 0.0;
}

// ============================================================================
// EXPERIMENT INTERFACE
// ============================================================================
class IExperiment {
public:
    virtual ~IExperiment() = default;
    virtual void setup(const ExperimentParameters& params) = 0;
    virtual void update(double delta_time) = 0;
    virtual double getTime() const = 0;
    virtual const vector<DataPoint>& getDataLog() const = 0;
    virtual vector<string> getActiveDataStructures() const = 0;
    virtual vector<string> getCurrentPhysicsInfo() const = 0;
    virtual string getExperimentName() const = 0;
};

// ============================================================================
// EXPERIMENT 1: FREE FALL
// ============================================================================
class FreeFall : public IExperiment {
private:
    ExperimentParameters params;
    double time_elapsed{ 0.0 };
    static const int BALL_COUNT = 3;
    double heights[BALL_COUNT];
    double velocities[BALL_COUNT];
    vector<DataPoint> data_log;

public:
    void setup(const ExperimentParameters& p) override {
        params = p;
        time_elapsed = 0.0;
        data_log.clear();

        for (int i = 0; i < BALL_COUNT; ++i) {
            heights[i] = 30.0 - i * 5.0;
            velocities[i] = 0.0;
        }
    }

    void update(double delta_time) override {
        if (delta_time <= 0) return;
        time_elapsed += delta_time;

        for (int i = 0; i < BALL_COUNT; ++i) {
            double acceleration = params.gravity - params.air_resistance * velocities[i];
            velocities[i] += acceleration * delta_time;
            heights[i] -= velocities[i] * delta_time;

            // Ground collision with bounce
            if (heights[i] < 0) {
                heights[i] = 0;
                velocities[i] = -velocities[i] * 0.75; // 75% energy retained
            }

            // Log first ball data
            if (i == 0) {
                double ke = calculateKineticEnergy(params.mass, 0, velocities[i]);
                double pe = calculatePotentialEnergy(params.mass, params.gravity, heights[i]);
                data_log.push_back({ time_elapsed, 0, heights[i], 0, velocities[i], ke, pe });
            }
        }
    }

    vector<string> getActiveDataStructures() const override {
        return { "Array (heights, velocities)", "Vector (DataPoints)",
                 "BST (Energy)", "HashMap (Metrics)" };
    }

    vector<string> getCurrentPhysicsInfo() const override {
        return {
            "Mass: " + std::to_string(params.mass) + " kg",
            "Gravity: " + std::to_string(params.gravity) + " m/s²",
            "Height: " + std::to_string(heights[0]) + " m",
            "Velocity: " + std::to_string(velocities[0]) + " m/s",
            "Air Resistance: " + std::to_string(params.air_resistance)
        };
    }

    string getExperimentName() const override { return "Free Fall - Three Balls"; }
    double getTime() const override { return time_elapsed; }
    const vector<DataPoint>& getDataLog() const override { return data_log; }
    int getBallCount() const { return BALL_COUNT; }
    double getHeight(int index) const { return (index >= 0 && index < BALL_COUNT) ? heights[index] : 0.0; }
    double getVelocity(int index) const { return (index >= 0 && index < BALL_COUNT) ? velocities[index] : 0.0; }
    ExperimentParameters& getParams() { return params; }
};

// ============================================================================
// EXPERIMENT 2: PENDULUM
// ============================================================================
class Pendulum : public IExperiment {
private:
    ExperimentParameters params;
    double time_elapsed{ 0.0 };
    double angle{ 0.0 };
    double angular_velocity{ 0.0 };
    vector<DataPoint> data_log;

public:
    void setup(const ExperimentParameters& p) override {
        params = p;
        time_elapsed = 0.0;
        angle = degreesToRadians(params.initial_angle);
        angular_velocity = 0.0;
        data_log.clear();
    }

    void update(double delta_time) override {
        if (delta_time <= 0) return;
        time_elapsed += delta_time;

        double angular_accel = -(params.gravity / params.length) * sin(angle)
            - params.air_resistance * angular_velocity;
        angular_velocity += angular_accel * delta_time;
        angle += angular_velocity * delta_time;

        // Calculate bob position and velocity
        double bob_x = params.length * sin(angle);
        double bob_y = params.length * cos(angle);
        double vel_x = angular_velocity * params.length * cos(angle);
        double vel_y = -angular_velocity * params.length * sin(angle);

        double ke = calculateKineticEnergy(params.mass, vel_x, vel_y);
        double pe = calculatePotentialEnergy(params.mass, params.gravity,
            params.length - params.length * cos(angle));

        data_log.push_back({ time_elapsed, bob_x, bob_y, vel_x, vel_y, ke, pe });
    }

    vector<string> getActiveDataStructures() const override {
        return { "Variables (angle, angular_vel)", "Vector (DataPoints)",
                 "BST (Energy)", "HashMap (Metrics)" };
    }

    vector<string> getCurrentPhysicsInfo() const override {
        return {
            "Mass: " + std::to_string(params.mass) + " kg",
            "Length: " + std::to_string(params.length) + " m",
            "Angle: " + std::to_string(angle * 180 / 3.14159) + "°",
            "Angular Velocity: " + std::to_string(angular_velocity) + " rad/s",
            "Gravity: " + std::to_string(params.gravity) + " m/s²"
        };
    }

    string getExperimentName() const override { return "Simple Pendulum"; }
    double getTime() const override { return time_elapsed; }
    const vector<DataPoint>& getDataLog() const override { return data_log; }
    double getBobX() const { return params.length * sin(angle); }
    double getBobY() const { return params.length * cos(angle); }
    ExperimentParameters& getParams() { return params; }
};

// ============================================================================
// EXPERIMENT 3: SPRING SYSTEM
// ============================================================================
class SpringSystem : public IExperiment {
private:
    ExperimentParameters params;
    double time_elapsed{ 0.0 };
    double displacement{ 2.0 };
    double velocity{ 0.0 };
    vector<DataPoint> data_log;

public:
    void setup(const ExperimentParameters& p) override {
        params = p;
        time_elapsed = 0.0;
        displacement = 2.0;
        velocity = 0.0;
        data_log.clear();
    }

    void update(double delta_time) override {
        if (delta_time <= 0) return;
        time_elapsed += delta_time;

        double spring_force = -params.spring_constant * displacement;
        double damping_force = -params.air_resistance * velocity * 5.0;
        double total_force = spring_force + damping_force;
        double acceleration = total_force / params.mass;

        velocity += acceleration * delta_time;
        displacement += velocity * delta_time;

        double ke = 0.5 * params.mass * velocity * velocity;
        double pe = 0.5 * params.spring_constant * displacement * displacement;

        data_log.push_back({ time_elapsed, displacement, 0, velocity, 0, ke, pe });
    }

    vector<string> getActiveDataStructures() const override {
        return { "Variables (displacement, velocity)", "Vector (DataPoints)",
                 "BST (Energy)", "HashMap (Metrics)" };
    }

    vector<string> getCurrentPhysicsInfo() const override {
        return {
            "Mass: " + std::to_string(params.mass) + " kg",
            "Spring Constant: " + std::to_string(params.spring_constant) + " N/m",
            "Displacement: " + std::to_string(displacement) + " m",
            "Velocity: " + std::to_string(velocity) + " m/s",
            "Force: " + std::to_string(-params.spring_constant * displacement) + " N"
        };
    }

    string getExperimentName() const override { return "Spring-Mass System"; }
    double getTime() const override { return time_elapsed; }
    const vector<DataPoint>& getDataLog() const override { return data_log; }
    double getDisplacement() const { return displacement; }
    ExperimentParameters& getParams() { return params; }
};

// ============================================================================
// EXPERIMENT 4: COLLISION BALLS
// ============================================================================
class CollisionBalls : public IExperiment {
private:
    ExperimentParameters params;
    double time_elapsed{ 0.0 };
    static const int BALL_COUNT = 2;
    Ball balls[BALL_COUNT];
    vector<DataPoint> data_log;
    CollisionGraph collision_graph;

    void handleCollisions() {
        double dx = balls[0].x - balls[1].x;
        double dy = balls[0].y - balls[1].y;
        double distance = sqrt(dx * dx + dy * dy);
        double min_distance = balls[0].radius + balls[1].radius;

        if (distance < min_distance && distance > 0) {
            // Elastic collision physics
            double mass1 = balls[0].mass;
            double mass2 = balls[1].mass;
            double vel1 = balls[0].vx;
            double vel2 = balls[1].vx;

            balls[0].vx = ((mass1 - mass2) * vel1 + 2 * mass2 * vel2) / (mass1 + mass2);
            balls[1].vx = ((mass2 - mass1) * vel2 + 2 * mass1 * vel1) / (mass1 + mass2);

            // Separate overlapping balls
            double overlap = min_distance - distance;
            if (overlap > 0) {
                if (balls[0].x < balls[1].x) {
                    balls[0].x -= overlap / 2;
                    balls[1].x += overlap / 2;
                }
                else {
                    balls[0].x += overlap / 2;
                    balls[1].x -= overlap / 2;
                }
            }

            collision_graph.registerCollision(0, 1);
        }
    }

public:
    void setup(const ExperimentParameters& p) override {
        params = p;
        time_elapsed = 0.0;
        data_log.clear();

        for (int i = 0; i < BALL_COUNT; ++i) {
            balls[i] = {
                5.0 + i * 5.0, 5.0,
                params.initial_velocities_x[i], 0,
                params.ball_radii[i], params.ball_masses[i], i
            };
        }

        collision_graph.setup(BALL_COUNT);
    }

    void applyLiveParams() {
        for (int i = 0; i < BALL_COUNT; i++) {
            balls[i].mass = params.ball_masses[i];
        }
    }

    void update(double delta_time) override {
        applyLiveParams();
        if (delta_time <= 0) return;
        time_elapsed += delta_time;

        for (int i = 0; i < BALL_COUNT; ++i) {
            balls[i].x += balls[i].vx * delta_time;

            // Wall collisions
            if (balls[i].x < balls[i].radius) {
                balls[i].x = balls[i].radius;
                balls[i].vx *= -1;
            }
            if (balls[i].x > 15 - balls[i].radius) {
                balls[i].x = 15 - balls[i].radius;
                balls[i].vx *= -1;
            }
        }


        handleCollisions();

        double ke = calculateKineticEnergy(balls[0].mass, balls[0].vx, balls[0].vy);
        data_log.push_back({ time_elapsed, balls[0].x, balls[0].y, balls[0].vx, balls[0].vy, ke, 0 });
    }
   


    vector<string> getActiveDataStructures() const override {
        return { "Array (Ball[2])", "Graph (Collisions)", "Vector (DataPoints)",
                 "BST (Energy)", "HashMap (Metrics)" };
    }

    vector<string> getCurrentPhysicsInfo() const override {
        int total_collisions = 0;
        for (const auto& node : collision_graph.getNodes()) {
            total_collisions += node.collision_count;
        }

        return {
            "Ball 1 Mass: " + std::to_string(balls[0].mass) + " kg",
            "Ball 2 Mass: " + std::to_string(balls[1].mass) + " kg",
            "Ball 1 Velocity: " + std::to_string(balls[0].vx) + " m/s",
            "Ball 2 Velocity: " + std::to_string(balls[1].vx) + " m/s",
            "Collisions: " + std::to_string(total_collisions / 2)
        };
    }

    string getExperimentName() const override { return "Elastic Collision - Two Balls"; }
    double getTime() const override { return time_elapsed; }
    const vector<DataPoint>& getDataLog() const override { return data_log; }
    const CollisionGraph& getGraph() const { return collision_graph; }
    int getBallCount() const { return BALL_COUNT; }
    const Ball* getBalls() const { return balls; }
    Ball* getBallsMutable() { return balls; }
    ExperimentParameters& getParams() { return params; }
};

// ============================================================================
// EXPERIMENT 5: BERNOULLI FLOW
// ============================================================================
class BernoulliFlow : public IExperiment {
private:
    struct FluidParticle {
        double x, y, vx;
        int section; // 0 = wide pipe, 1 = narrow pipe
    };

    ExperimentParameters params;
    double time_elapsed{ 0.0 };
    double velocity1{ 0.0 }, velocity2{ 0.0 };
    double pressure1{ 0.0 }, pressure2{ 0.0 };
    vector<DataPoint> data_log;
    vector<FluidParticle> particles;

    void computeFlowProperties() {
        double area1 = 3.14159 * pow(params.pipe_diameter1 / 2, 2);
        double area2 = 3.14159 * pow(params.pipe_diameter2 / 2, 2);

        // Pulsating flow simulation
        velocity1 = params.fluid_velocity * (1.0 + 0.15 * sin(time_elapsed * 1.5));
        velocity2 = area2 != 0 ? (area1 * velocity1) / area2 : velocity1;

        // Bernoulli's equation: P + (1/2)ρv² = constant
        double dynamic_pressure1 = 0.5 * params.fluid_density * velocity1 * velocity1;
        double dynamic_pressure2 = 0.5 * params.fluid_density * velocity2 * velocity2;

        pressure1 = params.static_pressure;
        double total_pressure = pressure1 + dynamic_pressure1;
        pressure2 = total_pressure - dynamic_pressure2;
    }

public:
    void setup(const ExperimentParameters& p) override {
        params = p;
        time_elapsed = 0.0;
        computeFlowProperties();
        data_log.clear();
        particles.clear();

        // Initialize fluid particles
        for (int i = 0; i < 25; i++) {
            particles.push_back({
                0.5 + i * 0.2,
                5.0 + (rand() % 100) / 100.0,
                velocity1, 0
                });
        }
    }

    void update(double delta_time) override {
        if (delta_time <= 0) return;
        time_elapsed += delta_time;
        computeFlowProperties();

        // Update particle positions
        for (auto& particle : particles) {
            if (particle.section == 0) {
                particle.x += velocity1 * delta_time * 0.5;
                particle.vx = velocity1;
                if (particle.x > 7.5) particle.section = 1;
            }
            else {
                particle.x += velocity2 * delta_time * 0.5;
                particle.vx = velocity2;
                if (particle.x > 15) {
                    particle.x = 0.5;
                    particle.section = 0;
                }
            }
        }

        data_log.push_back({ time_elapsed, velocity1, velocity2, 0, 0, 0, 0, pressure2 });
    }

    vector<string> getActiveDataStructures() const override {
        return { "Vector (Particles)", "Vector (DataPoints)",
                 "Variables (velocity, pressure)", "BST (Energy)", "HashMap (Metrics)" };
    }

    vector<string> getCurrentPhysicsInfo() const override {
        return {
            "Velocity 1: " + std::to_string(velocity1) + " m/s",
            "Velocity 2: " + std::to_string(velocity2) + " m/s",
            "Pressure 1: " + std::to_string(pressure1 / 1000) + " kPa",
            "Pressure 2: " + std::to_string(pressure2 / 1000) + " kPa",
            "Density: " + std::to_string(params.fluid_density) + " kg/m³"
        };
    }

    string getExperimentName() const override { return "Bernoulli's Principle - Fluid Flow"; }
    double getTime() const override { return time_elapsed; }
    const vector<DataPoint>& getDataLog() const override { return data_log; }
    double getVelocity1() const { return velocity1; }
    double getVelocity2() const { return velocity2; }
    double getPressure1() const { return pressure1; }
    double getPressure2() const { return pressure2; }
    const vector<FluidParticle>& getParticles() const { return particles; }
    ExperimentParameters& getParams() { return params; }
};

// ============================================================================
// EXPERIMENT 6: PROJECTILE MOTION
// ============================================================================
class ProjectileMotion : public IExperiment {
private:
    ExperimentParameters params;
    double time_elapsed{ 0.0 };
    double position_x{ 0.0 }, position_y{ 0.0 };
    double velocity_x{ 0.0 }, velocity_y{ 0.0 };
    vector<DataPoint> data_log;
    vector<Vector2> trajectory;
    bool has_landed{ false };

public:
    void recomputeVelocity() {
        double angle_rad = degreesToRadians(params.projectile_angle);
        velocity_x = params.projectile_speed * cos(angle_rad);
        velocity_y = params.projectile_speed * sin(angle_rad);
    }

    void setup(const ExperimentParameters& p) override {
        params = p;
        time_elapsed = 0.0;
        position_x = 0.0;
        position_y = 0.0;
        has_landed = false;

        double angle_rad = degreesToRadians(params.projectile_angle);
        velocity_x = params.projectile_speed * cos(angle_rad);
        velocity_y = params.projectile_speed * sin(angle_rad);

        data_log.clear();
        trajectory.clear();
    }

    void update(double delta_time) override {
 
        if (delta_time <= 0 || has_landed) return;
        time_elapsed += delta_time;

        double speed = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
        double drag_force_x = -params.air_resistance * velocity_x * speed;
        double drag_force_y = -params.air_resistance * velocity_y * speed;

        velocity_x += drag_force_x * delta_time;
        velocity_y += (drag_force_y - params.gravity) * delta_time;

        position_x += velocity_x * delta_time;
        position_y += velocity_y * delta_time;

        // Ground collision
        if (position_y < 0) {
            position_y = 0;
            has_landed = true;
        }

        trajectory.push_back({ (float)position_x, (float)position_y });

        double ke = calculateKineticEnergy(params.mass, velocity_x, velocity_y);
        double pe = calculatePotentialEnergy(params.mass, params.gravity, position_y);
        data_log.push_back({ time_elapsed, position_x, position_y, velocity_x, velocity_y, ke, pe });
    }

    vector<string> getActiveDataStructures() const override {
        return { "Variables (x, y, vx, vy)", "Vector (Trajectory)", "Vector (DataPoints)",
                 "BST (Energy)", "HashMap (Metrics)" };
    }

    vector<string> getCurrentPhysicsInfo() const override {
        double speed = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
        double angle_deg = atan2(velocity_y, velocity_x) * 180.0 / 3.14159;

        return {
            "Position: (" + std::to_string(position_x) + ", " + std::to_string(position_y) + ") m",
            "Velocity: " + std::to_string(speed) + " m/s",
            "Angle: " + std::to_string(angle_deg) + "°",
            "Height: " + std::to_string(position_y) + " m",
            "Range: " + std::to_string(position_x) + " m"
        };
    }

    string getExperimentName() const override { return "Projectile Motion"; }
    double getTime() const override { return time_elapsed; }
    const vector<DataPoint>& getDataLog() const override { return data_log; }
    double getPositionX() const { return position_x; }
    double getPositionY() const { return position_y; }
    const vector<Vector2>& getTrajectory() const { return trajectory; }
    bool hasLanded() const { return has_landed; }
    ExperimentParameters& getParams() { return params; }
};

// ============================================================================
// PHYSICS ENGINE MANAGER
// ============================================================================
class PhysicsEngine {
private:
    ExperimentParameters params;
    unique_ptr<IExperiment> current_experiment;
    vector<DataPoint> empty_log;
    EnergyBST energy_tree;
    AnalysisData analysis_data;
    stack<ExperimentParameters> undo_stack;
    bool is_paused = false;

    void createExperiment() {
        if (params.experiment_type == "FreeFall")
            current_experiment = make_unique<FreeFall>();
        else if (params.experiment_type == "Pendulum")
            current_experiment = make_unique<Pendulum>();
        else if (params.experiment_type == "SpringSystem")
            current_experiment = make_unique<SpringSystem>();
        else if (params.experiment_type == "CollisionBalls")
            current_experiment = make_unique<CollisionBalls>();
        else if (params.experiment_type == "Bernoulli")
            current_experiment = make_unique<BernoulliFlow>();
        else if (params.experiment_type == "Projectile")
            current_experiment = make_unique<ProjectileMotion>();
    }

public:
    void setParameters(const ExperimentParameters& new_params) {
        undo_stack.push(params);
        params = new_params;
        createExperiment();
        current_experiment->setup(params);
        energy_tree.clear();
        analysis_data = AnalysisData();
        is_paused = false;
    }

    void updateSimulation(double delta_time) {
        if (current_experiment && delta_time > 0 && !is_paused) {
            current_experiment->update(delta_time);
        }

        // Update analysis data
        if (!getDataLog().empty()) {
            const auto& latest_data = getDataLog().back();
            energy_tree.insert(latest_data.kinetic_energy, latest_data.time);
            analysis_data.updateMaxKE(latest_data.kinetic_energy);

            double speed = sqrt(latest_data.velocity_x * latest_data.velocity_x +
                latest_data.velocity_y * latest_data.velocity_y);
            analysis_data.updateMaxVelocity(speed);
        }
    }

    void togglePause() { is_paused = !is_paused; }
    bool isPaused() const { return is_paused; }

    void sortDataLog() {
        if (!current_experiment) return;
        vector<DataPoint>& log = const_cast<vector<DataPoint>&>(current_experiment->getDataLog());

        // Insertion sort by kinetic energy
        for (size_t i = 1; i < log.size(); ++i) {
            DataPoint key = log[i];
            int j = i - 1;
            while (j >= 0 && log[j].kinetic_energy > key.kinetic_energy) {
                log[j + 1] = log[j];
                j--;
            }
            log[j + 1] = key;
        }
    }

    long long factorial(int n) {
        if (n <= 0) return (n == 0) ? 1 : 0;
        return (long long)n * factorial(n - 1);
    }

    ExperimentParameters& getParams() { return params; }
    const vector<DataPoint>& getDataLog() const {
        return current_experiment ? current_experiment->getDataLog() : empty_log;
    }
    const EnergyBST& getEnergyTree() const { return energy_tree; }
    const AnalysisData& getAnalysis() const { return analysis_data; }
    IExperiment* getCurrentExperiment() { return current_experiment.get(); }

    // Type-safe experiment accessors
    FreeFall* asFreeFall() { return dynamic_cast<FreeFall*>(current_experiment.get()); }
    Pendulum* asPendulum() { return dynamic_cast<Pendulum*>(current_experiment.get()); }
    SpringSystem* asSpring() { return dynamic_cast<SpringSystem*>(current_experiment.get()); }
    CollisionBalls* asCollision() { return dynamic_cast<CollisionBalls*>(current_experiment.get()); }
    BernoulliFlow* asBernoulli() { return dynamic_cast<BernoulliFlow*>(current_experiment.get()); }
    ProjectileMotion* asProjectile() { return dynamic_cast<ProjectileMotion*>(current_experiment.get()); }
};

// ============================================================================
// DRAWING UTILITIES
// ============================================================================
void DrawSpring(Vector2 start, Vector2 end, int coil_count, float width, Color color) {
    Vector2 direction = { end.x - start.x, end.y - start.y };
    float length = sqrt(direction.x * direction.x + direction.y * direction.y);
    Vector2 dir_normalized = { direction.x / length, direction.y / length };
    Vector2 perpendicular = { -dir_normalized.y, dir_normalized.x };
    float step = length / coil_count;
    Vector2 current = start;

    for (int i = 0; i < coil_count; i++) {
        Vector2 next = {
            start.x + dir_normalized.x * step * (i + 1),
            start.y + dir_normalized.y * step * (i + 1)
        };
        Vector2 point1 = { current.x + perpendicular.x * width, current.y + perpendicular.y * width };
        Vector2 point2 = { next.x - perpendicular.x * width, next.y - perpendicular.y * width };
        DrawLineEx(current, point1, 2, color);
        DrawLineEx(point1, point2, 2, color);
        current = next;
    }
    DrawLineEx(current, end, 2, color);
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================
int main() {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Sandbox - Enhanced Edition");
    SetTargetFPS(60);

    PhysicsEngine engine;
    ExperimentParameters params;
    params.experiment_type = "CollisionBalls";
    engine.setParameters(params);

    int factorial_n = 5;

    while (!WindowShouldClose()) {
        float delta_time = GetFrameTime();
        engine.updateSimulation(delta_time);

        // ====================================================================
        // INPUT HANDLING
        // ====================================================================

        // Experiment selection
        if (IsKeyPressed(KEY_ONE)) {
            params.experiment_type = "FreeFall";
            engine.setParameters(params);
        }
        if (IsKeyPressed(KEY_TWO)) {
            params.experiment_type = "Pendulum";
            engine.setParameters(params);
        }
        if (IsKeyPressed(KEY_THREE)) {
            params.experiment_type = "SpringSystem";
            engine.setParameters(params);
        }
        if (IsKeyPressed(KEY_FOUR)) {
            params.experiment_type = "CollisionBalls";
            engine.setParameters(params);
        }
        if (IsKeyPressed(KEY_FIVE)) {
            params.experiment_type = "Bernoulli";
            engine.setParameters(params);
        }
        if (IsKeyPressed(KEY_SIX)) {
            params.experiment_type = "Projectile";
            engine.setParameters(params);
        }

        // General controls
        if (IsKeyPressed(KEY_SPACE)) engine.togglePause();
        if (IsKeyPressed(KEY_S)) engine.sortDataLog();
        if (IsKeyPressed(KEY_R)) engine.setParameters(params);
        if (IsKeyPressed(KEY_UP) && !IsKeyDown(KEY_LEFT_CONTROL)) factorial_n = (factorial_n < 10) ? factorial_n + 1 : 10;
        if (IsKeyPressed(KEY_DOWN) && !IsKeyDown(KEY_LEFT_CONTROL)) factorial_n = (factorial_n > 0) ? factorial_n - 1 : 0;

        // Live parameter adjustments (Ctrl + Key combinations)
        string experiment_type = params.experiment_type;
        bool ctrl_held = IsKeyDown(KEY_LEFT_CONTROL);

        if (experiment_type == "FreeFall") {
            auto freefall = engine.asFreeFall();
            if (freefall && ctrl_held) {
                if (IsKeyPressed(KEY_G)) freefall->getParams().gravity += 1.0;
                if (IsKeyPressed(KEY_H)) freefall->getParams().gravity =
                    (freefall->getParams().gravity > 1.0) ? freefall->getParams().gravity - 1.0 : 1.0;
                if (IsKeyPressed(KEY_M)) freefall->getParams().mass += 0.5;
                if (IsKeyPressed(KEY_N)) freefall->getParams().mass =
                    (freefall->getParams().mass > 0.5) ? freefall->getParams().mass - 0.5 : 0.5;
            }
        }
        else if (experiment_type == "Pendulum") {
            auto pendulum = engine.asPendulum();
            if (pendulum && ctrl_held) {
                if (IsKeyPressed(KEY_G)) pendulum->getParams().gravity += 1.0;
                if (IsKeyPressed(KEY_H)) pendulum->getParams().gravity =
                    (pendulum->getParams().gravity > 1.0) ? pendulum->getParams().gravity - 1.0 : 1.0;
                if (IsKeyPressed(KEY_L)) pendulum->getParams().length += 0.5;
                if (IsKeyPressed(KEY_K)) pendulum->getParams().length =
                    (pendulum->getParams().length > 0.5) ? pendulum->getParams().length - 0.5 : 0.5;
            }
        }
        else if (experiment_type == "SpringSystem") {
            auto spring = engine.asSpring();
            if (spring && ctrl_held) {
                if (IsKeyPressed(KEY_K)) spring->getParams().spring_constant += 10.0;
                if (IsKeyPressed(KEY_J)) spring->getParams().spring_constant =
                    (spring->getParams().spring_constant > 10.0) ? spring->getParams().spring_constant - 10.0 : 10.0;
                if (IsKeyPressed(KEY_M)) spring->getParams().mass += 0.5;
                if (IsKeyPressed(KEY_N)) spring->getParams().mass =
                    (spring->getParams().mass > 0.5) ? spring->getParams().mass - 0.5 : 0.5;
            }
        }
        else if (experiment_type == "CollisionBalls") {
            auto collision = engine.asCollision();
            if (collision && ctrl_held) {
                if (IsKeyPressed(KEY_Q)) collision->getParams().ball_masses[0] += 0.5;
                if (IsKeyPressed(KEY_A)) collision->getParams().ball_masses[0] =
                    (collision->getParams().ball_masses[0] > 0.5) ? collision->getParams().ball_masses[0] - 0.5 : 0.5;
                if (IsKeyPressed(KEY_W)) collision->getParams().ball_masses[1] += 0.5;
                if (IsKeyPressed(KEY_S)) collision->getParams().ball_masses[1] =
                    (collision->getParams().ball_masses[1] > 0.5) ? collision->getParams().ball_masses[1] - 0.5 : 0.5;
                if (IsKeyPressed(KEY_E)) collision->getParams().initial_velocities_x[0] += 1.0;
                if (IsKeyPressed(KEY_D)) collision->getParams().initial_velocities_x[0] -= 1.0;
            }
        }
        else if (experiment_type == "Bernoulli") {
            auto bernoulli = engine.asBernoulli();
            if (bernoulli && ctrl_held) {
                if (IsKeyPressed(KEY_V)) bernoulli->getParams().fluid_velocity += 0.5;
                if (IsKeyPressed(KEY_C)) bernoulli->getParams().fluid_velocity =
                    (bernoulli->getParams().fluid_velocity > 0.5) ? bernoulli->getParams().fluid_velocity - 0.5 : 0.5;
                if (IsKeyPressed(KEY_D)) bernoulli->getParams().pipe_diameter2 += 0.01;
                if (IsKeyPressed(KEY_X)) bernoulli->getParams().pipe_diameter2 =
                    (bernoulli->getParams().pipe_diameter2 > 0.02) ? bernoulli->getParams().pipe_diameter2 - 0.01 : 0.02;
            }
        }
        else if (experiment_type == "Projectile") {
            auto projectile = engine.asProjectile();
            if (projectile && ctrl_held) {
                if (IsKeyPressed(KEY_UP)) projectile->getParams().projectile_angle += 5.0;
                if (IsKeyPressed(KEY_DOWN)) projectile->getParams().projectile_angle =
                    (projectile->getParams().projectile_angle > 5.0) ? projectile->getParams().projectile_angle - 5.0 : 5.0;
                if (IsKeyPressed(KEY_RIGHT)) projectile->getParams().projectile_speed += 2.0;
                if (IsKeyPressed(KEY_LEFT)) projectile->getParams().projectile_speed =
                    (projectile->getParams().projectile_speed > 2.0) ? projectile->getParams().projectile_speed - 2.0 : 2.0;
            }
        }

        // ====================================================================
        // RENDERING
        // ====================================================================
        BeginDrawing();
        ClearBackground(COLOR_LIGHT_BG);

        // Main canvas area
        DrawRectangle(0, 0, CANVAS_WIDTH, SCREEN_HEIGHT, RAYWHITE);
        DrawRectangleLines(0, 0, CANVAS_WIDTH, SCREEN_HEIGHT, COLOR_PRIMARY_DARK);
        DrawText("Keys: 1-6 Switch | SPACE-Pause | R-Reset | S-Sort | Ctrl+Keys for live adjustments",
            10, 8, 10, COLOR_TEXT_GRAY);

        // Pause indicator
        if (engine.isPaused()) {
            DrawRectangle(CANVAS_WIDTH / 2 - 80, 40, 160, 40, Fade(COLOR_ACCENT_RED, 0.8f));
            DrawText("|| PAUSED ||", CANVAS_WIDTH / 2 - 55, 50, 20, WHITE);
        }

        // ====================================================================
        // EXPERIMENT VISUALIZATION
        // ====================================================================
        if (experiment_type == "FreeFall") {
            auto freefall = engine.asFreeFall();
            if (freefall) {
                // Ground
                DrawRectangle(50, 600, 700, 10, COLOR_PRIMARY_DARK);

                // Height scale
                DrawLine(100, 50, 100, 600, COLOR_TEXT_GRAY);
                for (int i = 0; i <= 10; i++) {
                    int y_pos = 600 - i * 50;
                    DrawLine(90, y_pos, 100, y_pos, COLOR_TEXT_GRAY);
                    DrawText(TextFormat("%dm", i * 3), 60, y_pos - 5, 10, COLOR_TEXT_GRAY);
                }

                // Balls
                for (int i = 0; i < freefall->getBallCount(); i++) {
                    float x = 250 + i * 180;
                    float y = 600 - (float)freefall->getHeight(i) * 15;
                    DrawCircle(x, y, 18, COLOR_ACCENT_RED);
                    DrawText(TextFormat("Ball %d", i + 1), x - 20, y - 35, 10, COLOR_PRIMARY_DARK);
                    DrawText(TextFormat("h=%.1fm", freefall->getHeight(i)), x - 25, y + 25, 9, COLOR_TEXT_GRAY);
                    DrawText(TextFormat("v=%.1f m/s", freefall->getVelocity(i)), x - 28, y + 38, 8, Fade(COLOR_TEXT_GRAY, 0.7f));
                }
            }
        }
        else if (experiment_type == "Pendulum") {
            auto pendulum = engine.asPendulum();
            if (pendulum) {
                float pivot_x = CANVAS_WIDTH / 2;
                float pivot_y = 80;
                float bob_x = pivot_x + (float)pendulum->getBobX() * 200;
                float bob_y = pivot_y + (float)pendulum->getBobY() * 200;

                // Pivot mount
                DrawLineEx({ pivot_x - 60, pivot_y }, { pivot_x + 60, pivot_y }, 5, COLOR_PRIMARY_DARK);

                // Pendulum rod
                DrawLineEx({ pivot_x, pivot_y }, { bob_x, bob_y }, 3, COLOR_TEXT_GRAY);

                // Bob
                DrawCircle(bob_x, bob_y, 28, COLOR_ACCENT_RED);
                DrawCircle(pivot_x, pivot_y, 6, COLOR_PRIMARY_DARK);

                // Labels
                DrawText(TextFormat("m=%.1fkg", pendulum->getParams().mass), bob_x - 30, bob_y + 35, 10, COLOR_PRIMARY_DARK);
                DrawText(TextFormat("L=%.1fm", pendulum->getParams().length), pivot_x + 10, pivot_y + 10, 9, COLOR_TEXT_GRAY);
            }
        }
        else if (experiment_type == "SpringSystem") {
            auto spring = engine.asSpring();
            if (spring) {
                float anchor_x = CANVAS_WIDTH / 2;
                float anchor_y = 120;
                float spring_length = 120 + (float)spring->getDisplacement() * 90;

                // Fixed ceiling
                DrawRectangle(anchor_x - 70, anchor_y - 12, 140, 12, COLOR_PRIMARY_DARK);

                // Spring
                DrawSpring({ anchor_x, anchor_y }, { anchor_x, anchor_y + spring_length }, 14, 18, COLOR_TEXT_GRAY);

                // Mass block
                DrawRectangle(anchor_x - 30, anchor_y + spring_length, 60, 60, COLOR_ACCENT_BLUE);

                // Labels
                DrawText(TextFormat("m=%.1fkg", spring->getParams().mass),
                    anchor_x - 25, anchor_y + spring_length + 20, 10, WHITE);
                DrawText(TextFormat("x=%.2fm", spring->getDisplacement()),
                    anchor_x - 25, anchor_y + spring_length + 70, 10, COLOR_PRIMARY_DARK);
                DrawText(TextFormat("k=%.0f N/m", spring->getParams().spring_constant),
                    anchor_x - 35, anchor_y - 30, 10, COLOR_TEXT_GRAY);
            }
        }
        else if (experiment_type == "CollisionBalls") {
            auto collision = engine.asCollision();
            if (collision) {
                // Track
                DrawRectangle(50, 380, 750, 8, COLOR_PRIMARY_DARK);

                // Balls
                for (int i = 0; i < collision->getBallCount(); i++) {
                    const Ball* ball = &collision->getBalls()[i];
                    float x = 120 + (float)ball->x * 45;
                    float y = 350 - (float)ball->radius * 45;
                    Color ball_color = i == 0 ? COLOR_ACCENT_BLUE : COLOR_ACCENT_RED;

                    DrawCircle(x, y, (float)ball->radius * 45, ball_color);
                    DrawLineEx({ x, y }, { x + (float)ball->vx * 22, y }, 2.5f, COLOR_PRIMARY_DARK);

                    // Ball info
                    DrawText(TextFormat("B%d", i + 1), x - 8, y - 5, 12, WHITE);
                    DrawText(TextFormat("m=%.1f kg", ball->mass), x - 25, y + (float)ball->radius * 45 + 8, 9, COLOR_PRIMARY_DARK);
                    DrawText(TextFormat("v=%.1f m/s", ball->vx), x - 28, y + (float)ball->radius * 45 + 22, 8, COLOR_TEXT_GRAY);
                }

                // Collision graph visualization
                const auto& graph_nodes = collision->getGraph().getNodes();
                DrawText("Collision Graph", 100, 500, 14, COLOR_PRIMARY_DARK);

                for (size_t i = 0; i < graph_nodes.size(); i++) {
                    int circle_x = 180 + i * 200;
                    Color node_color = graph_nodes[i].collision_count > 0 ? COLOR_ACCENT_RED : Fade(COLOR_TEXT_GRAY, 0.5f);
                    DrawCircle(circle_x, 550, 22, node_color);
                    DrawText(TextFormat("Ball %d", (int)i + 1), circle_x - 18, 580, 11, COLOR_TEXT_GRAY);
                    DrawText(TextFormat("Hits: %d", graph_nodes[i].collision_count), circle_x - 20, 600, 10, COLOR_PRIMARY_DARK);

                    // Draw connection edge
                    if (graph_nodes.size() > 1 && i == 0 && graph_nodes[0].collision_count > 0) {
                        DrawLineEx({ (float)circle_x + 22, 550 }, { (float)(circle_x + 200 - 22), 550 }, 3, COLOR_SECONDARY_GOLD);
                    }
                }
            }
        }
        else if (experiment_type == "Bernoulli") {
            auto bernoulli = engine.asBernoulli();
            if (bernoulli) {
                float start_x = 120;
                float start_y = 300;
                float pipe_length = 550;
                float diameter1 = 90;
                float diameter2 = 35;

                // Wide pipe section
                DrawRectangle(start_x, start_y - diameter1 / 2, pipe_length / 2, diameter1, Fade(COLOR_ACCENT_BLUE, 0.2f));
                DrawRectangleLines(start_x, start_y - diameter1 / 2, pipe_length / 2, diameter1, COLOR_ACCENT_BLUE);

                // Narrow pipe section
                DrawRectangle(start_x + pipe_length / 2, start_y - diameter2 / 2, pipe_length / 2, diameter2, Fade(COLOR_ACCENT_RED, 0.2f));
                DrawRectangleLines(start_x + pipe_length / 2, start_y - diameter2 / 2, pipe_length / 2, diameter2, COLOR_ACCENT_RED);

                // Transition visualization
                DrawTriangle(
                    { start_x + pipe_length / 2, start_y - diameter1 / 2 },
                    { start_x + pipe_length / 2, start_y - diameter2 / 2 },
                    { start_x + pipe_length / 2 + 25, start_y - diameter2 / 2 },
                    Fade(COLOR_TEXT_GRAY, 0.4f)
                );
                DrawTriangle(
                    { start_x + pipe_length / 2, start_y + diameter1 / 2 },
                    { start_x + pipe_length / 2, start_y + diameter2 / 2 },
                    { start_x + pipe_length / 2 + 25, start_y + diameter2 / 2 },
                    Fade(COLOR_TEXT_GRAY, 0.4f)
                );

                // Fluid particles
                const auto& particles = bernoulli->getParticles();
                for (const auto& particle : particles) {
                    float px = start_x + (float)particle.x * 40;
                    float py = start_y + (float)(particle.y - 5) * 8;
                    Color particle_color = particle.section == 0 ? COLOR_ACCENT_BLUE : COLOR_ACCENT_RED;
                    DrawCircle(px, py, 4, particle_color);
                    DrawLineEx({ px, py }, { px - (float)particle.vx * 5, py }, 2, Fade(particle_color, 0.4f));
                }

                // Flow measurements
                DrawText(TextFormat("V1: %.2f m/s", bernoulli->getVelocity1()), start_x + 60, start_y - 60, 13, COLOR_ACCENT_BLUE);
                DrawText(TextFormat("P1: %.1f kPa", bernoulli->getPressure1() / 1000), start_x + 60, start_y - 45, 11, COLOR_TEXT_GRAY);
                DrawText(TextFormat("V2: %.2f m/s", bernoulli->getVelocity2()), start_x + pipe_length / 2 + 60, start_y - 40, 13, COLOR_ACCENT_RED);
                DrawText(TextFormat("P2: %.1f kPa", bernoulli->getPressure2() / 1000), start_x + pipe_length / 2 + 60, start_y - 25, 11, COLOR_TEXT_GRAY);
                DrawText("Bernoulli: P + 1/2pv² = const", start_x + 100, start_y + 80, 12, COLOR_PRIMARY_DARK);
            }
        }
        else if (experiment_type == "Projectile") {

            auto projectile = engine.asProjectile();
            bool changed = false;

            if (IsKeyPressed(KEY_UP)) { projectile->getParams().projectile_angle += 5.0; changed = true; }
            if (IsKeyPressed(KEY_DOWN)) { projectile->getParams().projectile_angle -= 5.0; changed = true; }
            if (IsKeyPressed(KEY_RIGHT)) { projectile->getParams().projectile_speed += 2.0; changed = true; }
            if (IsKeyPressed(KEY_LEFT)) { projectile->getParams().projectile_speed -= 2.0; changed = true; }

            if (changed) {
                projectile->setup(projectile->getParams());
            }

            if (projectile) {
                // Ground
                DrawRectangle(50, 550, 750, 5, COLOR_PRIMARY_DARK);

                float scale_x = 40.0f;
                float scale_y = 40.0f;
                float offset_x = 100.0f;
                float offset_y = 550.0f;

                // Trajectory path
                const auto& trajectory = projectile->getTrajectory();
                for (size_t i = 1; i < trajectory.size(); i++) {
                    Vector2 point1 = { offset_x + trajectory[i - 1].x * scale_x, offset_y - trajectory[i - 1].y * scale_y };
                    Vector2 point2 = { offset_x + trajectory[i].x * scale_x, offset_y - trajectory[i].y * scale_y };
                    DrawLineEx(point1, point2, 2, Fade(COLOR_ACCENT_BLUE, 0.4f));
                }

                // Projectile or landing marker
                if (!projectile->hasLanded()) {
                    float proj_x = offset_x + (float)projectile->getPositionX() * scale_x;
                    float proj_y = offset_y - (float)projectile->getPositionY() * scale_y;
                    DrawCircle(proj_x, proj_y, 12, COLOR_ACCENT_RED);

                    // Velocity vector
                    const auto& log = projectile->getDataLog();
                    if (!log.empty()) {
                        float vel_x = (float)log.back().velocity_x * 3;
                        float vel_y = -(float)log.back().velocity_y * 3;
                        DrawLineEx({ proj_x, proj_y }, { proj_x + vel_x, proj_y + vel_y }, 2.5f, COLOR_SECONDARY_GOLD);

                        // Info labels
                        DrawText(TextFormat("h=%.1fm", projectile->getPositionY()), proj_x + 15, proj_y - 10, 9, COLOR_PRIMARY_DARK);
                        double speed = sqrt(log.back().velocity_x * log.back().velocity_x +
                            log.back().velocity_y * log.back().velocity_y);
                        DrawText(TextFormat("v=%.1f m/s", speed), proj_x + 15, proj_y + 2, 8, COLOR_TEXT_GRAY);
                    }
                }
                else {
                    float landing_x = offset_x + (float)projectile->getPositionX() * scale_x;
                    DrawCircle(landing_x, offset_y, 12, Fade(COLOR_ACCENT_RED, 0.5f));
                    DrawText("LANDED", landing_x - 25, offset_y + 20, 12, COLOR_ACCENT_GREEN);
                    DrawText(TextFormat("Range: %.1fm", projectile->getPositionX()), landing_x - 35, offset_y + 35, 10, COLOR_PRIMARY_DARK);
                }

                // Launch point
                DrawCircle(offset_x, offset_y, 8, COLOR_PRIMARY_DARK);
                DrawText("Launch", offset_x - 20, offset_y + 15, 10, COLOR_TEXT_GRAY);

                // Distance markers
                for (int i = 0; i <= 15; i++) {
                    float grid_x = offset_x + i * scale_x * 2;
                    DrawLine(grid_x, offset_y, grid_x, offset_y + 5, Fade(COLOR_TEXT_GRAY, 0.3f));
                    if (i % 5 == 0) {
                        DrawText(TextFormat("%dm", i * 2), grid_x - 10, offset_y + 10, 9, COLOR_TEXT_GRAY);
                    }
                }
            }
        }

        // Experiment title bar
        if (engine.getCurrentExperiment()) {
            string exp_name = engine.getCurrentExperiment()->getExperimentName();
            int text_width = MeasureText(exp_name.c_str(), 16);
            DrawRectangle(CANVAS_WIDTH / 2 - text_width / 2 - 20, SCREEN_HEIGHT - 45, text_width + 40, 35, Fade(COLOR_PRIMARY_DARK, 0.9f));
            DrawText(exp_name.c_str(), CANVAS_WIDTH / 2 - text_width / 2, SCREEN_HEIGHT - 35, 16, COLOR_SECONDARY_GOLD);
        }

        // ====================================================================
        // SIDE PANEL
        // ====================================================================

        DrawRectangle(PANEL_X, 0, PANEL_WIDTH, SCREEN_HEIGHT, COLOR_LIGHT_BG);
        DrawLine(PANEL_X, 0, PANEL_X, SCREEN_HEIGHT, COLOR_PRIMARY_DARK);

        int py = 20;
        DrawText("PHYSICS DATA", PANEL_X + 20, py, 18, COLOR_PRIMARY_DARK);
        py += 35;

        // Current measurements
        DrawRectangle(PANEL_X + 15, py, PANEL_WIDTH - 30, 140, WHITE);
        DrawRectangleLines(PANEL_X + 15, py, PANEL_WIDTH - 30, 140, COLOR_ACCENT_BLUE);
        DrawText("Current Measurements:", PANEL_X + 25, py + 10, 13, COLOR_PRIMARY_DARK);
        py += 30;

        if (engine.getCurrentExperiment()) {
            auto info = engine.getCurrentExperiment()->getCurrentPhysicsInfo();
            for (size_t i = 0; i < info.size() && i < 5; i++) {
                DrawText(info[i].c_str(), PANEL_X + 25, py, 10, COLOR_TEXT_GRAY);
                py += 20;
            }
        }
        py += 20;

        // Max values
        DrawText("Maximum Values:", PANEL_X + 20, py, 14, COLOR_PRIMARY_DARK);
        py += 25;
        DrawRectangle(PANEL_X + 15, py, PANEL_WIDTH - 30, 60, WHITE);
        DrawRectangleLines(PANEL_X + 15, py, PANEL_WIDTH - 30, 60, COLOR_TEXT_GRAY);
        DrawText(TextFormat("Max KE: %.2f J", engine.getAnalysis().max_metrics.at("Max_KE")),
            PANEL_X + 25, py + 15, 12, COLOR_ACCENT_RED);
        DrawText(TextFormat("Max Vel: %.2f m/s", engine.getAnalysis().max_metrics.at("Max_Velocity")),
            PANEL_X + 25, py + 35, 11, COLOR_TEXT_GRAY);
        py += 75;

        // Live Controls Guide
        DrawText("LIVE CONTROLS (Hold Ctrl):", PANEL_X + 20, py, 13, COLOR_ACCENT_GREEN);
        py += 20;
        DrawRectangle(PANEL_X + 15, py, PANEL_WIDTH - 30, 100, WHITE);
        DrawRectangleLines(PANEL_X + 15, py, PANEL_WIDTH - 30, 100, COLOR_ACCENT_GREEN);

        if (experiment_type == "CollisionBalls") {
            DrawText("Q/A: Ball 1 Mass ±", PANEL_X + 25, py + 10, 9, COLOR_TEXT_GRAY);
            DrawText("W/S: Ball 2 Mass ±", PANEL_X + 25, py + 25, 9, COLOR_TEXT_GRAY);
        }
        else if (experiment_type == "Pendulum") {
            DrawText("G/H: Gravity ±", PANEL_X + 25, py + 10, 9, COLOR_TEXT_GRAY);
            DrawText("L/K: Length ±", PANEL_X + 25, py + 25, 9, COLOR_TEXT_GRAY);
        }
        else if (experiment_type == "SpringSystem") {
            DrawText("K/J: Spring k ±", PANEL_X + 25, py + 10, 9, COLOR_TEXT_GRAY);
        }
        else if (experiment_type == "FreeFall") {
            DrawText("G/H: Gravity ±", PANEL_X + 25, py + 10, 9, COLOR_TEXT_GRAY);
        }
        else if (experiment_type == "Projectile") {
            DrawText("↑/↓: Angle ±", PANEL_X + 25, py + 10, 9, COLOR_TEXT_GRAY);
            DrawText("→/←: Speed ±", PANEL_X + 25, py + 25, 9, COLOR_TEXT_GRAY);
            DrawText("(Reset to apply)", PANEL_X + 25, py + 40, 8, Fade(COLOR_TEXT_GRAY, 0.7f));
        }
        py += 110;

        // Data structures
        DrawText("DATA STRUCTURES:", PANEL_X + 20, py, 14, COLOR_ACCENT_GREEN);
        py += 25;
        DrawRectangle(PANEL_X + 15, py, PANEL_WIDTH - 30, 120, WHITE);
        DrawRectangleLines(PANEL_X + 15, py, PANEL_WIDTH - 30, 120, COLOR_ACCENT_GREEN);

        if (engine.getCurrentExperiment()) {
            auto dsa = engine.getCurrentExperiment()->getActiveDataStructures();
            for (size_t i = 0; i < dsa.size() && i < 6; i++) {
                DrawText(TextFormat("✓ %s", dsa[i].c_str()),
                    PANEL_X + 25, py + 10 + i * 18, 9, COLOR_TEXT_GRAY);
            }
        }
        py += 130;

        // BST (Energy tree)
        DrawText(TextFormat("BST Nodes: %d", engine.getEnergyTree().getNodeCount()),
            PANEL_X + 20, py, 11, Fade(COLOR_TEXT_GRAY, 0.7f));
        DrawText("(energy tracking)", PANEL_X + 20, py + 15, 8, Fade(COLOR_TEXT_GRAY, 0.5f));
        py += 40;

        // Recursion demo
        DrawText("Recursion: Factorial", PANEL_X + 20, py, 13, COLOR_PRIMARY_DARK);
        py += 20;
        DrawRectangle(PANEL_X + 15, py, PANEL_WIDTH - 30, 45, WHITE);
        DrawRectangleLines(PANEL_X + 15, py, PANEL_WIDTH - 30, 45, COLOR_TEXT_GRAY);
        DrawText(TextFormat("%d! = %lld", factorial_n, engine.factorial(factorial_n)),
            PANEL_X + 25, py + 12, 15, COLOR_ACCENT_BLUE);
        py += 60;

        // Time
        DrawText(TextFormat("Time: %.2f s",
            engine.getDataLog().empty() ? 0 : engine.getDataLog().back().time),
            PANEL_X + 20, py, 10, Fade(COLOR_TEXT_GRAY, 0.7f));

        EndDrawing();
    }

    CloseWindow();
    return 0;
}