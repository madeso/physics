#include "raylib.h"

#include "physics/physics.h"


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

    SetTargetFPS(60);
    while (!WindowShouldClose())
    {
        UpdateCamera(&camera);
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                DrawCube((Vector3){-4.0f, 0.0f, 2.0f}, 2.0f, 5.0f, 2.0f, RED);
                DrawCubeWires((Vector3){-4.0f, 0.0f, 2.0f}, 2.0f, 5.0f, 2.0f, GOLD);
                DrawCubeWires((Vector3){-4.0f, 0.0f, -2.0f}, 3.0f, 6.0f, 2.0f, MAROON);

                DrawSphere((Vector3){-1.0f, 0.0f, -2.0f}, 1.0f, GREEN);
                DrawSphereWires((Vector3){1.0f, 0.0f, 2.0f}, 2.0f, 16, 16, LIME);

                DrawCylinder((Vector3){4.0f, 0.0f, -2.0f}, 1.0f, 2.0f, 3.0f, 4, SKYBLUE);
                DrawCylinderWires((Vector3){4.0f, 0.0f, -2.0f}, 1.0f, 2.0f, 3.0f, 4, DARKBLUE);
                DrawCylinderWires((Vector3){4.5f, -1.0f, 2.0f}, 1.0f, 1.0f, 2.0f, 6, BROWN);

                DrawCylinder((Vector3){1.0f, 0.0f, -4.0f}, 0.0f, 1.5f, 3.0f, 8, GOLD);
                DrawCylinderWires((Vector3){1.0f, 0.0f, -4.0f}, 0.0f, 1.5f, 3.0f, 8, PINK);

                DrawGrid(10, 1.0f);

            EndMode3D();

            DrawFPS(10, 10);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}

