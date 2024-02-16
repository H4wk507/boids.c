#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "raylib.h"
#include "raymath.h"

// TODO: 3d
// TODO: triangles with moving points to direction

#define PROTECTED_RANGE 0.051
#define AVOID_FACTOR 0.0025

#define MATCHING_FACTOR 0.001
#define VISIBLE_RANGE 0.502
#define CENTERING_FACTOR 0.001

#define TURN_FACTOR 0.0010
#define BIASVAL 0.0001

#define MIN_X -1
#define MAX_X 1
#define MIN_Y -1
#define MAX_Y 1
#define LEFT_MARGIN 0.1
#define RIGHT_MARGIN 0.9
#define BOTTOM_MARGIN 0.9
#define TOP_MARGIN 0.1

#define INIT_VX 0.0001
#define INIT_VY 0.0001
#define MAX_SPEED 0.01
#define MIN_SPEED 0.005

#define NBOIDS 100
#define RADIUS 5

typedef struct {
    Vector2 pos;
    Vector2 velocity;
    int radius;
} Boid;

/* Points have position in <0; 1> range, and we project them to our screen dimensions */
Vector2 project_point_to_screen(Vector2 point) {
    return CLITERAL(Vector2) {
        .x = point.x*GetScreenWidth(),
        .y = point.y*GetScreenHeight()
    };
}

/* linearly interpolate point from <0; 1> to <start; end> */
double lerp(double point, double start, double end) {
    return point*(end-start)+start;
}

/* Random double between 0 and 1. */
double rand_double() {
    return (double)rand()/RAND_MAX;
}

void generate_random_boids(Boid *boids, int n, double radius, Vector2 velocity) {
    for (int i = 0; i < n; i++) {
        boids[i] = (Boid) { 
            CLITERAL(Vector2) { 
                .x = rand_double(),
                .y = rand_double()
            },
            .velocity = velocity,
            .radius = radius,
        };
     }
}

void separate(Boid *boids, int n) {
    for (int i = 0; i < n; i++) {
        double close_dx = 0;
        double close_dy = 0;
        for (int j = 0; j < n; j++) {
            if (i == j)
                continue;

            double dx = boids[i].pos.x - boids[j].pos.x;
            double dy = boids[i].pos.y - boids[j].pos.y;
            double dist = sqrt(pow(dx, 2) + pow(dy, 2));
            if (dist < PROTECTED_RANGE) {
                close_dx += dx;
                close_dy += dy;
            }
        }
        Vector2 dv = CLITERAL(Vector2) {
            .y = close_dy * AVOID_FACTOR,
            .x = close_dx * AVOID_FACTOR
        };
        boids[i].velocity = Vector2Add(boids[i].velocity, dv);
    }
}

void align(Boid *boids, int n) {
    for (int i = 0; i < n; i++) {
        double xvel_avg = 0;
        double yvel_avg = 0;
        int neighboring_boids = 0;
        for (int j = 0; j < n; j++) {
            if (i == j)
                continue;

            double dx = boids[i].pos.x - boids[j].pos.x;
            double dy = boids[i].pos.y - boids[j].pos.y;
            double dist = sqrt(pow(dx, 2) + pow(dy, 2));
            if (dist < VISIBLE_RANGE) {
                xvel_avg += boids[j].velocity.x;
                yvel_avg += boids[j].velocity.y;
                neighboring_boids++;
            }
        }
        if (neighboring_boids > 0) {
            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;
        }
        Vector2 dv = CLITERAL(Vector2) {
            .y = (yvel_avg - boids[i].velocity.y) * MATCHING_FACTOR,
            .x = (xvel_avg - boids[i].velocity.x) * MATCHING_FACTOR
        };
        boids[i].velocity = Vector2Add(boids[i].velocity, dv);
    }
}

void cohere(Boid *boids, int n) {
    for (int i = 0; i < n; i++) {
        double xpos_avg = 0;
        double ypos_avg = 0;
        int neighboring_boids = 0;
        for (int j = 0; j < n; j++) {
            if (i == j)
                continue;
            
            double dx = boids[i].pos.x - boids[j].pos.x;
            double dy = boids[i].pos.y - boids[j].pos.y;
            double dist = sqrt(pow(dx, 2) + pow(dy, 2));
            if (dist < VISIBLE_RANGE) {
                xpos_avg += boids[j].pos.x;
                ypos_avg += boids[j].pos.y;
                neighboring_boids++;
            }
        }
        if (neighboring_boids > 0) {
            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;
        }
        Vector2 dv = CLITERAL(Vector2) {
            .x = (xpos_avg - boids[i].pos.x) * CENTERING_FACTOR,
            .y = (ypos_avg - boids[i].pos.y) * CENTERING_FACTOR
        };
        boids[i].velocity = Vector2Add(boids[i].velocity, dv);
    }
}

// bias to make two left and right groups
void bias(Boid *boids, int n) {
    for (int i = 0; i < n; i++) {
        if (i < n/2) {
            boids[i].velocity.x = (1-BIASVAL)*boids[i].velocity.x + (BIASVAL*1);
        } else {
            boids[i].velocity.x = (1-BIASVAL)*boids[i].velocity.x + (BIASVAL*-1);
        }
    }
}

void turn(Boid *boids, int n) {
    for (int i = 0; i < n; i++) {
        if (boids[i].pos.x < LEFT_MARGIN)   boids[i].velocity.x = boids[i].velocity.x + TURN_FACTOR;
        if (boids[i].pos.x > RIGHT_MARGIN)  boids[i].velocity.x = boids[i].velocity.x - TURN_FACTOR;
        if (boids[i].pos.y > BOTTOM_MARGIN) boids[i].velocity.y = boids[i].velocity.y - TURN_FACTOR;
        if (boids[i].pos.y < TOP_MARGIN)    boids[i].velocity.y = boids[i].velocity.y + TURN_FACTOR;
    }
}

void update_boids_positions(Boid *boids, int n) {
    for (int i = 0; i < n; i++) {
        boids[i].pos = Vector2Add(boids[i].pos, boids[i].velocity);
    }
}

void limit_speed(Boid *boids, int n) {
    for (int i = 0; i < n; i++) {
        double speed = sqrt(pow(boids[i].velocity.x, 2)+pow(boids[i].velocity.y, 2));
        if (speed > MAX_SPEED) {
            boids[i].velocity.x = (boids[i].velocity.x/speed)*MAX_SPEED;
            boids[i].velocity.y = (boids[i].velocity.y/speed)*MAX_SPEED;
        }
        if (speed < MIN_SPEED) {
            boids[i].velocity.x = (boids[i].velocity.x/speed)*MIN_SPEED;
            boids[i].velocity.y = (boids[i].velocity.y/speed)*MIN_SPEED;
        }
    }
}

void update_boids(Boid *boids, int n) {
    separate(boids, n);
    align(boids, n);
    cohere(boids, n);
    bias(boids, n);
    turn(boids, n);
    limit_speed(boids, n);
    update_boids_positions(boids, n);
}

void draw_triangle(Boid *boid) {
    Vector2 a = CLITERAL(Vector2) {
        .x = boid->pos.x,
        .y = boid->pos.y + boid->radius
    };
    Vector2 b = CLITERAL(Vector2) {
        .x = boid->pos.x + boid->radius*sqrtf(3)/2.0f,
        .y = boid->pos.y - boid->radius/2.0f
    };
    Vector2 c = CLITERAL(Vector2) {
        .x = boid->pos.x - boid->radius*sqrtf(3)/2.0f,
        .y = boid->pos.y - boid->radius/2.0f
    };
    printf("%f, %f\n", project_point_to_screen(a).x, project_point_to_screen(a).y);
    printf("==========\n");
    printf("%f, %f, %f, %f\n", a.x, a.y, b.x, b.y);
    DrawTriangle(project_point_to_screen(a), project_point_to_screen(b), project_point_to_screen(c), RED);
}

int main(void) {
    srand(time(NULL));
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(800, 600, "boids");

    Vector2 velocity = CLITERAL(Vector2) { .x = INIT_VX, .y = INIT_VY };
    Boid boids[NBOIDS] = { 0 };
    generate_random_boids(boids, NBOIDS, RADIUS, velocity);
    while (!WindowShouldClose()) {
        if (GetKeyPressed() == KEY_SPACE) {
            generate_random_boids(boids, NBOIDS, RADIUS, velocity);
        }
        BeginDrawing();
        SetTargetFPS(60);
        ClearBackground(GetColor(0x18181800));
        update_boids(boids, NBOIDS);
        for (int i = 0; i < NBOIDS; i++) {
            //draw_triangle(&boids[i]);
            DrawCircleV(project_point_to_screen(boids[i].pos), boids[i].radius, RED);
        }
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
