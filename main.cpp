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
// MAIN LOOP
// ============================================================================
int main()
{
    InitWindow(screenWidth, screenHeight, "Physics Sandbox - Enhanced with Controls");
    SetTargetFPS(60);

    PhysicsEngine engine;
    ExperimentParameters params;
    params.experiment_type = "CollisionBalls";
    engine.setParameters(params);

    int factN = 5;

    while (!WindowShouldClose())
    {
         float dt = GetFrameTime();
         engine.updateSimulation(dt);
        
         // ====================================================================
         // INPUT HANDLING
         // ====================================================================
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
        
         if (IsKeyPressed(KEY_SPACE)) engine.togglePause();
         if (IsKeyPressed(KEY_S)) engine.sortDataLog();
         if (IsKeyPressed(KEY_R)) engine.setParameters(params);
         if (IsKeyPressed(KEY_UP)) factN = (factN < 10) ? factN + 1 : 10;
         if (IsKeyPressed(KEY_DOWN)) factN = (factN > 0) ? factN - 1 : 0;
        
         // Live parameter adjustments
         string type = params.experiment_type;
         if (type == "CollisionBalls") {
             auto c = engine.asCollision();
             if (c && IsKeyDown(KEY_LEFT_CONTROL)) {
                 if (IsKeyPressed(KEY_Q)) c->getParams().ball_masses[0] += 0.5;
                 if (IsKeyPressed(KEY_A)) c->getParams().ball_masses[0] = (c->getParams().ball_masses[0] > 0.5) ? c->getParams().ball_masses[0] - 0.5 : 0.5;
                 if (IsKeyPressed(KEY_W)) c->getParams().ball_masses[1] += 0.5;
                 if (IsKeyPressed(KEY_S)) c->getParams().ball_masses[1] = (c->getParams().ball_masses[1] > 0.5) ? c->getParams().ball_masses[1] - 0.5 : 0.5;
             }
         }
         else if (type == "Pendulum") {
             auto p = engine.asPendulum();
             if (p && IsKeyDown(KEY_LEFT_CONTROL)) {
                 if (IsKeyPressed(KEY_G)) p->getParams().gravity += 1.0;
                 if (IsKeyPressed(KEY_H)) p->getParams().gravity = (p->getParams().gravity > 1.0) ? p->getParams().gravity - 1.0 : 1.0;
                 if (IsKeyPressed(KEY_L)) p->getParams().length += 0.5;
                 if (IsKeyPressed(KEY_K)) p->getParams().length = (p->getParams().length > 0.5) ? p->getParams().length - 0.5 : 0.5;
             }
         }
         else if (type == "SpringSystem") {
             auto s = engine.asSpring();
             if (s && IsKeyDown(KEY_LEFT_CONTROL)) {
                 if (IsKeyPressed(KEY_K)) s->getParams().spring_constant += 10.0;
                 if (IsKeyPressed(KEY_J)) s->getParams().spring_constant = (s->getParams().spring_constant > 10.0) ? s->getParams().spring_constant - 10.0 : 10.0;
             }
         }
         else if (type == "FreeFall") {
             auto f = engine.asFreeFall();
             if (f && IsKeyDown(KEY_LEFT_CONTROL)) {
                 if (IsKeyPressed(KEY_G)) f->getParams().gravity += 1.0;
                 if (IsKeyPressed(KEY_H)) f->getParams().gravity = (f->getParams().gravity > 1.0) ? f->getParams().gravity - 1.0 : 1.0;
             }
         }
         else if (type == "Projectile") {
             auto pr = engine.asProjectile();
             if (pr && IsKeyDown(KEY_LEFT_CONTROL)) {
                 if (IsKeyPressed(KEY_UP)) pr->getParams().projectile_angle += 5.0;
                 if (IsKeyPressed(KEY_DOWN)) pr->getParams().projectile_angle = (pr->getParams().projectile_angle > 5.0) ? pr->getParams().projectile_angle - 5.0 : 5.0;
                 if (IsKeyPressed(KEY_RIGHT)) pr->getParams().projectile_speed += 2.0;
                 if (IsKeyPressed(KEY_LEFT)) pr->getParams().projectile_speed = (pr->getParams().projectile_speed > 2.0) ? pr->getParams().projectile_speed - 2.0 : 2.0;
             }
         }


        
        // ====================================================================
        // RENDERING
        // ====================================================================
        BeginDrawing();
        ClearBackground(LightBackground);

        // Canvas area
        DrawRectangle(0, 0, canvasWidth, screenHeight, RAYWHITE);
        DrawRectangleLines(0, 0, canvasWidth, screenHeight, PrimaryDark);
        DrawText("Keys: 1-6 Switch | SPACE-Pause | R-Reset | S-Sort | Ctrl+Keys for live adjustments", 10, 8, 10, TextGray);

        // Pause indicator
        if (engine.isPaused())
        {
            DrawRectangle(canvasWidth / 2 - 80, 40, 160, 40, Fade(AccentRed, 0.8f));
            DrawText("|| PAUSED ||", canvasWidth / 2 - 55, 50, 20, WHITE);
        }

        // ====================================================================
        // EXPERIMENT RENDERING
        // ====================================================================
        if (type == "FreeFall")
        {
            auto f = engine.asFreeFall();
            if (f)
            {
                DrawRectangle(50, 600, 700, 10, PrimaryDark);
                DrawLine(100, 50, 100, 600, TextGray);

                for (int i = 0; i <= 10; i++)
                {
                    DrawLine(90, 600 - i * 50, 100, 600 - i * 50, TextGray);
                    DrawText(TextFormat("%dm", i * 3), 60, 595 - i * 50, 10, TextGray);
                }

                for (int i = 0; i < f->getCount(); i++)
                {
                    float x = 250 + i * 180;
                    float y = 600 - (float)f->getHeight(i) * 15;
                    DrawCircle(x, y, 18, AccentRed);

                    // Ball labels
                    DrawText(TextFormat("Ball %d", i + 1), x - 20, y - 35, 10, PrimaryDark);
                    DrawText(TextFormat("h=%.1fm", f->getHeight(i)), x - 25, y + 25, 9, TextGray);
                    DrawText(TextFormat("v=%.1f", f->getVelocity(i)), x - 22, y + 38, 8, Fade(TextGray, 0.7f));
                }
            }
        }
        else if (type == "Pendulum")
        {
            auto p = engine.asPendulum();
            if (p)
            {
                float px = canvasWidth / 2, py = 80;
                float bx = px + (float)p->getBobX() * 200;
                float by = py + (float)p->getBobY() * 200;

                DrawLineEx({px - 60, py}, {px + 60, py}, 5, PrimaryDark);
                DrawLineEx({px, py}, {bx, by}, 3, TextGray);
                DrawCircle(bx, by, 28, AccentRed);
                DrawCircle(px, py, 6, PrimaryDark);

                // Bob label
                DrawText(TextFormat("m=%.1fkg", p->getParams().mass), bx - 30, by + 35, 10, PrimaryDark);
                DrawText(TextFormat("L=%.1fm", p->getParams().length), px + 10, py + 10, 9, TextGray);
            }
        }
        else if (type == "SpringSystem")
        {
            auto s = engine.asSpring();
            if (s)
            {
                float ax = canvasWidth / 2, ay = 120;
                float len = 120 + (float)s->getDisplacement() * 90;

                DrawRectangle(ax - 70, ay - 12, 140, 12, PrimaryDark);
                DrawSpring({ax, ay}, {ax, ay + len}, 14, 18, TextGray);
                DrawRectangle(ax - 30, ay + len, 60, 60, AccentBlue);

                // Mass label
                DrawText(TextFormat("m=%.1fkg", s->getParams().mass), ax - 25, ay + len + 20, 10, WHITE);
                DrawText(TextFormat("x=%.2fm", s->getDisplacement()), ax - 25, ay + len + 70, 10, PrimaryDark);
                DrawText(TextFormat("k=%.0f N/m", s->getParams().spring_constant), ax - 35, ay - 30, 10, TextGray);
            }
        }
        else if (type == "CollisionBalls")
        {
            auto c = engine.asCollision();
            if (c)
            {
                DrawRectangle(50, 380, 750, 8, PrimaryDark);

                for (int i = 0; i < c->getCount(); i++)
                {
                    const Ball *b = &c->getBalls()[i];
                    float x = 120 + (float)b->x * 45;
                    float y = 350 - (float)b->radius * 45;
                    DrawCircle(x, y, (float)b->radius * 45, i == 0 ? AccentBlue : AccentRed);
                    DrawLineEx({x, y}, {x + (float)b->vx * 22, y}, 2.5f, PrimaryDark);

                    // Ball labels
                    DrawText(TextFormat("B%d", i + 1), x - 8, y - 5, 12, WHITE);
                    DrawText(TextFormat("m=%.1f", b->mass), x - 20, y + (float)b->radius * 45 + 8, 9, PrimaryDark);
                    DrawText(TextFormat("v=%.1f", b->vx), x - 18, y + (float)b->radius * 45 + 22, 8, TextGray);
                }

                const auto &nodes = c->getGraph().getNodes();
                DrawText("Collision Graph", 100, 500, 14, PrimaryDark);

                for (size_t i = 0; i < nodes.size(); i++)
                {
                    int cx = 180 + i * 200;
                    DrawCircle(cx, 550, 22, nodes[i].collision_count > 0 ? AccentRed : Fade(TextGray, 0.5f));
                    DrawText(TextFormat("Ball %d", (int)i + 1), cx - 18, 580, 11, TextGray);
                    DrawText(TextFormat("Hits: %d", nodes[i].collision_count), cx - 20, 600, 10, PrimaryDark);

                    if (nodes.size() > 1 && i == 0 && nodes[0].collision_count > 0)
                        DrawLineEx({(float)cx + 22, 550}, {(float)(cx + 200 - 22), 550}, 3, SecondaryGold);
                }
            }
        }
        else if (type == "Bernoulli")
        {
            auto b = engine.asBernoulli();
            if (b)
            {
                float sx = 120, sy = 300, len = 550;
                float d1 = 90, d2 = 35;

                DrawRectangle(sx, sy - d1 / 2, len / 2, d1, Fade(AccentBlue, 0.2f));
                DrawRectangle(sx + len / 2, sy - d2 / 2, len / 2, d2, Fade(AccentRed, 0.2f));
                DrawRectangleLines(sx, sy - d1 / 2, len / 2, d1, AccentBlue);
                DrawRectangleLines(sx + len / 2, sy - d2 / 2, len / 2, d2, AccentRed);

                DrawTriangle({sx + len / 2, sy - d1 / 2}, {sx + len / 2, sy - d2 / 2},
                             {sx + len / 2 + 25, sy - d2 / 2}, Fade(TextGray, 0.4f));
                DrawTriangle({sx + len / 2, sy + d1 / 2}, {sx + len / 2, sy + d2 / 2},
                             {sx + len / 2 + 25, sy + d2 / 2}, Fade(TextGray, 0.4f));

                const auto &pts = b->getParticles();
                for (const auto &pt : pts)
                {
                    float px = sx + (float)pt.x * 40;
                    float py = sy + (float)(pt.y - 5) * 8;
                    Color pc = pt.sec == 0 ? AccentBlue : AccentRed;
                    DrawCircle(px, py, 4, pc);
                    DrawLineEx({px, py}, {px - (float)pt.vx * 5, py}, 2, Fade(pc, 0.4f));
                }

                DrawText(TextFormat("V1: %.2f m/s", b->getV1()), sx + 60, sy - 60, 13, AccentBlue);
                DrawText(TextFormat("P1: %.1f kPa", b->getP1() / 1000), sx + 60, sy - 45, 11, TextGray);
                DrawText(TextFormat("V2: %.2f m/s", b->getV2()), sx + len / 2 + 60, sy - 40, 13, AccentRed);
                DrawText(TextFormat("P2: %.1f kPa", b->getP2() / 1000), sx + len / 2 + 60, sy - 25, 11, TextGray);
                DrawText("Bernoulli: P + 1/2pv² = const", sx + 100, sy + 80, 12, PrimaryDark);
            }
        }
        else if (type == "Projectile")
        {
            auto proj = engine.asProjectile();
            if (proj)
            {
                DrawRectangle(50, 550, 750, 5, PrimaryDark);

                float scaleX = 40.0f;
                float scaleY = 40.0f;
                float offsetX = 100.0f;
                float offsetY = 550.0f;

                const auto &traj = proj->getTrajectory();
                for (size_t i = 1; i < traj.size(); i++)
                {
                    Vector2 p1 = {offsetX + traj[i - 1].x * scaleX, offsetY - traj[i - 1].y * scaleY};
                    Vector2 p2 = {offsetX + traj[i].x * scaleX, offsetY - traj[i].y * scaleY};
                    DrawLineEx(p1, p2, 2, Fade(AccentBlue, 0.4f));
                }

                if (!proj->hasLanded())
                {
                    float px = offsetX + (float)proj->getX() * scaleX;
                    float py = offsetY - (float)proj->getY() * scaleY;
                    DrawCircle(px, py, 12, AccentRed);

                    const auto &log = proj->dataLog();
                    if (!log.empty())
                    {
                        float vx = (float)log.back().velocity_x * 3;
                        float vy = -(float)log.back().velocity_y * 3;
                        DrawLineEx({px, py}, {px + vx, py + vy}, 2.5f, SecondaryGold);

                        // Projectile label
                        DrawText(TextFormat("h=%.1fm", proj->getY()), px + 15, py - 10, 9, PrimaryDark);
                        DrawText(TextFormat("v=%.1f", sqrt(log.back().velocity_x * log.back().velocity_x + log.back().velocity_y * log.back().velocity_y)), px + 15, py + 2, 8, TextGray);
                    }
                }
                else
                {
                    float px = offsetX + (float)proj->getX() * scaleX;
                    DrawCircle(px, offsetY, 12, Fade(AccentRed, 0.5f));
                    DrawText("LANDED", px - 25, offsetY + 20, 12, AccentGreen);
                    DrawText(TextFormat("Range: %.1fm", proj->getX()), px - 35, offsetY + 35, 10, PrimaryDark);
                }

                DrawCircle(offsetX, offsetY, 8, PrimaryDark);
                DrawText("Launch", offsetX - 20, offsetY + 15, 10, TextGray);

                for (int i = 0; i <= 15; i++)
                {
                    float gx = offsetX + i * scaleX * 2;
                    DrawLine(gx, offsetY, gx, offsetY + 5, Fade(TextGray, 0.3f));
                    if (i % 5 == 0)
                        DrawText(TextFormat("%dm", i * 2), gx - 10, offsetY + 10, 9, TextGray);
                }
            }
        }

        // Experiment heading at bottom
        if (engine.getExp())
        {
            string expName = engine.getExp()->getExperimentName();
            int textWidth = MeasureText(expName.c_str(), 16);
            DrawRectangle(canvasWidth / 2 - textWidth / 2 - 20, screenHeight - 45, textWidth + 40, 35, Fade(PrimaryDark, 0.9f));
            DrawText(expName.c_str(), canvasWidth / 2 - textWidth / 2, screenHeight - 35, 16, SecondaryGold);
        }

        
        // ====================================================================
        // SIDE PANEL
        // ====================================================================
        DrawRectangle(panelX, 0, panelWidth, screenHeight, LightBackground);
        DrawLine(panelX, 0, panelX, screenHeight, PrimaryDark);

        int py = 20;
        DrawText("PHYSICS DATA", panelX + 20, py, 18, PrimaryDark);
        py += 35;

        // Current measurements
        DrawRectangle(panelX + 15, py, panelWidth - 30, 140, WHITE);
        DrawRectangleLines(panelX + 15, py, panelWidth - 30, 140, AccentBlue);
        DrawText("Current Measurements:", panelX + 25, py + 10, 13, PrimaryDark);
        py += 30;

        if (engine.getExp()) {
            auto info = engine.getExp()->getCurrentPhysicsInfo();
            for (size_t i = 0; i < info.size() && i < 5; i++) {
                DrawText(info[i].c_str(), panelX + 25, py, 10, TextGray);
                py += 20;
            }
        }
        py += 20;

        // Max values
        DrawText("Maximum Values:", panelX + 20, py, 14, PrimaryDark);
        py += 25;
        DrawRectangle(panelX + 15, py, panelWidth - 30, 60, WHITE);
        DrawRectangleLines(panelX + 15, py, panelWidth - 30, 60, TextGray);
        DrawText(TextFormat("Max KE: %.2f J", engine.getAnalysis().max_metrics.at("Max_KE")),
            panelX + 25, py + 15, 12, AccentRed);
        DrawText(TextFormat("Max Vel: %.2f m/s", engine.getAnalysis().max_metrics.at("Max_Velocity")),
            panelX + 25, py + 35, 11, TextGray);
        py += 75;

        // Live Controls Guide
        DrawText("LIVE CONTROLS (Hold Ctrl):", panelX + 20, py, 13, AccentGreen);
        py += 20;
        DrawRectangle(panelX + 15, py, panelWidth - 30, 100, WHITE);
        DrawRectangleLines(panelX + 15, py, panelWidth - 30, 100, AccentGreen);

        if (type == "CollisionBalls") {
            DrawText("Q/A: Ball 1 Mass ±", panelX + 25, py + 10, 9, TextGray);
            DrawText("W/S: Ball 2 Mass ±", panelX + 25, py + 25, 9, TextGray);
        }
        else if (type == "Pendulum") {
            DrawText("G/H: Gravity ±", panelX + 25, py + 10, 9, TextGray);
            DrawText("L/K: Length ±", panelX + 25, py + 25, 9, TextGray);
        }
        else if (type == "SpringSystem") {
            DrawText("K/J: Spring k ±", panelX + 25, py + 10, 9, TextGray);
        }
        else if (type == "FreeFall") {
            DrawText("G/H: Gravity ±", panelX + 25, py + 10, 9, TextGray);
        }
        else if (type == "Projectile") {
            DrawText("↑/↓: Angle ±", panelX + 25, py + 10, 9, TextGray);
            DrawText("→/←: Speed ±", panelX + 25, py + 25, 9, TextGray);
            DrawText("(Reset to apply)", panelX + 25, py + 40, 8, Fade(TextGray, 0.7f));
        }
        py += 110;

        // Data structures
        DrawText("DATA STRUCTURES:", panelX + 20, py, 14, AccentGreen);
        py += 25;
        DrawRectangle(panelX + 15, py, panelWidth - 30, 120, WHITE);
        DrawRectangleLines(panelX + 15, py, panelWidth - 30, 120, AccentGreen);

        if (engine.getExp()) {
            auto dsa = engine.getExp()->getActiveDataStructures();
            for (size_t i = 0; i < dsa.size() && i < 6; i++) {
                DrawText(TextFormat("✓ %s", dsa[i].c_str()), panelX + 25, py + 10 + i * 18, 9, TextGray);
            }
        }
        py += 130;

        // BST
        DrawText(TextFormat("BST Nodes: %d", engine.getBST().getNodeCount()),
            panelX + 20, py, 11, Fade(TextGray, 0.7f));
        DrawText("(energy tracking)", panelX + 20, py + 15, 8, Fade(TextGray, 0.5f));
        py += 40;

        // Recursion demo
        DrawText("Recursion: Factorial", panelX + 20, py, 13, PrimaryDark);
        py += 20;
        DrawRectangle(panelX + 15, py, panelWidth - 30, 45, WHITE);
        DrawRectangleLines(panelX + 15, py, panelWidth - 30, 45, TextGray);
        DrawText(TextFormat("%d! = %lld", factN, engine.factorial(factN)),
            panelX + 25, py + 12, 15, AccentBlue);
        py += 60;

        // Time
        DrawText(TextFormat("Time: %.2f s",
            engine.getDataLog().empty() ? 0 : engine.getDataLog().back().time),
            panelX + 20, py, 10, Fade(TextGray, 0.7f));

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
