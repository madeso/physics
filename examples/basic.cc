#include <memory>
#include <iostream>

#include "raylib.h"
#include "rlgl.h"

#include "physics/physics.h"



template<class> inline constexpr bool always_false_v = false;

void draw_shape(const physics::Shape& shape_arg, Color color)
{
	std::visit([&](const auto& shape)
    {
        using T = std::decay_t<decltype(shape)>;
        if constexpr (std::is_same_v<T, physics::Sphere>)
        {
            const auto p = shape.center;
            DrawSphereWires({p.x, p.y, p.z}, shape.radius, 5, 5, color);
        }
        else if constexpr (std::is_same_v<T, physics::Plane>)
        {
            // todo(Gustav): fix normal
            constexpr float size = 100.0f;
            DrawPlane({0,0,0}, {size, size}, color);
            // DrawCubeWires(Vector3{0,0,0}, size, 0.1f, size, color);
        }
        else if constexpr (std::is_same_v<T, physics::NullShape>)
        {
            // nop
        }
        else
        {
            static_assert(always_false_v<T>, "non-exhaustive visitor!");
        }
    }, shape_arg);
}


void draw_object(const physics::Object& obj, Color color)
{
    const auto p = obj.transform.position;
    const auto r = obj.transform.rotation;
    const auto s = obj.transform.scale;

    rlPushMatrix();
    // std::cout << p.x << " " << p.y << " " << p.z << "\n";
    rlTranslatef(p.x, p.y, p.z);
    // rlRotatef(float angle, float x, float y, float z);
    rlScalef(s, s,s);
    draw_shape(obj.shape, color);
    rlPopMatrix();
}

struct SimObject
{
    Color color = GREEN;
    physics::Object object;

    SimObject& with_color(Color c)
    {
        color = c;
        return *this;
    }

    SimObject& with_shape(physics::Shape s)
    {
        object.shape = s;
        return *this;
    }

    SimObject& with_mass(float m)
    {
        object.body = physics::Rigidbody{};
        object.body->mass = m;
        return *this;
    }

    SimObject& with_position(const glm::vec3& p)
    {
        object.transform.position = p;
        return *this;
    }
};

struct Demo
{
    std::vector<std::unique_ptr<SimObject>> objects;
    physics::World world;

    Demo()
    {
        world.solvers.emplace_back(physics::PositionSolver);
    }

    SimObject& add()
    {
        auto ptr = std::make_unique<SimObject>();
        auto ret = ptr.get();
        objects.emplace_back(std::move(ptr));
        world.objects.emplace_back(&ret->object);
        return *ret;
    }

    void step(float dt)
    {
        Step(&world, dt);
    }

    void draw()
    {
        for(auto& o: objects)
        {
            draw_object(o->object, o->color);
        }
    }
};

int main(void)
{
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "physics demo");

    Camera camera;
    camera.position = { 0.0f, 10.0f, 10.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    SetCameraMode(camera, CAMERA_FREE);

    Demo demo;

    demo.add()
        .with_shape(physics::Plane{glm::vec3{0, 1, 0}, 0.0f})
        .with_position({0, -3, 0})
        ;
    demo.add()
        .with_color(RED)
        .with_shape(physics::Sphere{glm::vec3{0, 0, 0}, 1.0f})
        .with_mass(1.0f)
        ;

    constexpr int fps = 60;
    SetTargetFPS(fps);
    while (!WindowShouldClose())
    {
        demo.step(1.0f / fps);

        UpdateCamera(&camera);
        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                demo.draw();
                DrawGrid(10, 1.0f);
            EndMode3D();
        EndDrawing();
    }

    CloseWindow();

    return 0;
}

