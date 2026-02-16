#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

// 3D向量结构
typedef struct { double x, y, z; } Vec3;

// 4x4矩阵结构（列优先）
typedef struct { double m[4][4]; } Matrix;

// 投影矩阵参数
typedef struct { double l, r, b, t, n, f; } Frustum;

// 赛道点结构
typedef struct { Vec3 center; Vec3 left; Vec3 right; } TrackSegment;

// ==================== 向量运算 ====================
Vec3 vec3_add(Vec3 a, Vec3 b) { return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 vec3_sub(Vec3 a, Vec3 b) { return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 vec3_scale(Vec3 v, double s) { return (Vec3){v.x * s, v.y * s, v.z * s}; }
double vec3_dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 vec3_cross(Vec3 a, Vec3 b) { return (Vec3){ a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x }; }
Vec3 vec3_normalize(Vec3 v) { double len = sqrt(vec3_dot(v, v)); if (len < 1e-6) return v; return vec3_scale(v, 1.0/len); }

// ==================== 矩阵运算 ====================
Vec3 transform_point(const Vec3* v, const Matrix* mat) {
    Vec3 res;
    double w = mat->m[3][0] * v->x + mat->m[3][1] * v->y + mat->m[3][2] * v->z + mat->m[3][3];
    if (fabs(w) > 1e-6) {
        res.x = (mat->m[0][0] * v->x + mat->m[0][1] * v->y + mat->m[0][2] * v->z + mat->m[0][3]) / w;
        res.y = (mat->m[1][0] * v->x + mat->m[1][1] * v->y + mat->m[1][2] * v->z + mat->m[1][3]) / w;
        res.z = (mat->m[2][0] * v->x + mat->m[2][1] * v->y + mat->m[2][2] * v->z + mat->m[2][3]) / w;
    } else res.x = res.y = res.z = 0;
    return res;
}

Matrix matrix_multiply(const Matrix* a, const Matrix* b) {
    Matrix res;
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
        res.m[i][j] = 0;
        for (int k = 0; k < 4; k++) res.m[i][j] += a->m[i][k] * b->m[k][j];
    }
    return res;
}

Matrix projection_matrix(const Frustum* frust) {
    Matrix mat = {0};
    double rl = frust->r - frust->l, tb = frust->t - frust->b, fn = frust->f - frust->n;
    mat.m[0][0] = 2 * frust->n / rl;
    mat.m[0][2] = (frust->r + frust->l) / rl;
    mat.m[1][1] = 2 * frust->n / tb;
    mat.m[1][2] = (frust->t + frust->b) / tb;
    mat.m[2][2] = -(frust->f + frust->n) / fn;
    mat.m[2][3] = -2 * frust->f * frust->n / fn;
    mat.m[3][2] = -1;
    return mat;
}

Matrix look_at_matrix(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 z = vec3_normalize(vec3_sub(eye, target));
    Vec3 x = vec3_normalize(vec3_cross(up, z));
    Vec3 y = vec3_cross(z, x);
    Matrix mat = {0};
    mat.m[0][0] = x.x; mat.m[0][1] = x.y; mat.m[0][2] = x.z; mat.m[0][3] = -vec3_dot(x, eye);
    mat.m[1][0] = y.x; mat.m[1][1] = y.y; mat.m[1][2] = y.z; mat.m[1][3] = -vec3_dot(y, eye);
    mat.m[2][0] = z.x; mat.m[2][1] = z.y; mat.m[2][2] = z.z; mat.m[2][3] = -vec3_dot(z, eye);
    mat.m[3][3] = 1;
    return mat;
}

// ==================== 赛道生成 ====================
TrackSegment* generate_track(int num_segments, double track_width) {
    TrackSegment* track = malloc(num_segments * sizeof(TrackSegment));
    double z = 0, y = 0, x = 0, angle = 0, step = 2.0;
    for (int i = 0; i < num_segments; i++) {
        if (i % 10 == 0) angle += (rand() % 100 - 50) / 500.0;
        y += (rand() % 200 - 100) / 500.0; // 高度变化
        if (y < -1) y = -1; if (y > 1) y = 1;
        x += step * sin(angle);
        z += step * cos(angle);
        track[i].center = (Vec3){x, y, z};
        Vec3 dir = {sin(angle), 0, cos(angle)};
        Vec3 left_dir = {cos(angle), 0, -sin(angle)};
        track[i].left  = (Vec3){x + left_dir.x * track_width, y, z + left_dir.z * track_width};
        track[i].right = (Vec3){x - left_dir.x * track_width, y, z - left_dir.z * track_width};
    }
    return track;
}

// ==================== 渲染 ====================
int project_point(const Vec3* world, const Matrix* vp, int* sx, int* sy) {
    Vec3 clip = transform_point(world, vp);
    if (clip.z < -1 || clip.z > 1 || clip.x < -1 || clip.x > 1 || clip.y < -1 || clip.y > 1) return 0;
    int max_y, max_x;
    getmaxyx(stdscr, max_y, max_x);
    *sx = (int)((clip.x + 1) * 0.5 * (max_x - 1));
    *sy = (int)((1 - (clip.y + 1) * 0.5) * (max_y - 1));
    return 1;
}

void draw_line(const Vec3* a, const Vec3* b, const Matrix* vp, char ch) {
    int x1, y1, x2, y2;
    if (!project_point(a, vp, &x1, &y1) || !project_point(b, vp, &x2, &y2)) return;
    int dx = abs(x2 - x1), dy = abs(y2 - y1);
    int steps = dx > dy ? dx : dy; if (steps < 1) steps = 1;
    for (int i = 0; i <= steps; i++) {
        double t = (double)i / steps;
        int x = x1 + (int)(t * (x2 - x1));
        int y = y1 + (int)(t * (y2 - y1));
        if (x >= 0 && x < COLS && y >= 0 && y < LINES) mvaddch(y, x, ch);
    }
}

// ==================== 游戏状态 ====================
typedef struct { Vec3 position; double yaw; double speed; } Car;

double get_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

int main() {
    initscr(); cbreak(); noecho(); keypad(stdscr, TRUE); nodelay(stdscr, TRUE); curs_set(0);
    srand(time(NULL));

    int num_segments = 800;
    double track_width = 3.0;
    TrackSegment* track = generate_track(num_segments, track_width);

    int max_y, max_x;
    getmaxyx(stdscr, max_y, max_x);
    double aspect = (double)max_x / max_y;
    Frustum frust = { -aspect * 0.4, aspect * 0.4, -0.3, 0.3, 0.5, 200.0 };
    Matrix proj = projection_matrix(&frust);

    Car car = { {0, 0.5, 0}, 0, 0 };

    // 控制参数
    const double ACCELERATION = 5.0;      // 加速度 m/s² (增大以更快响应)
    const double TURN_SPEED = 1.0;        // 转向速度 rad/s (减小使转向更柔和)
    const double MAX_SPEED = 15.0;
    const double FRICTION = 1.5;          // 摩擦减速度 m/s² (减小使滑行更久)
    const double KEY_HOLD_TIME = 0.12;     // 按键保持时间(秒)

    double last_time = get_time();
    double last_w_time = -KEY_HOLD_TIME, last_s_time = -KEY_HOLD_TIME;
    double last_a_time = -KEY_HOLD_TIME, last_d_time = -KEY_HOLD_TIME;
    int running = 1;

    while (running) {
        double current_time = get_time();
        double delta_time = current_time - last_time;
        last_time = current_time;
        if (delta_time > 0.1) delta_time = 0.1;

        // 读取所有等待的输入，更新按键时间
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'w': last_w_time = current_time; break;
                case 's': last_s_time = current_time; break;
                case 'a': last_a_time = current_time; break;
                case 'd': last_d_time = current_time; break;
                case 'q': running = 0; break;
            }
        }

        // 判断按键是否处于保持状态
        int accel_fwd = (current_time - last_w_time) < KEY_HOLD_TIME;
        int accel_bwd = (current_time - last_s_time) < KEY_HOLD_TIME;
        int turn_left = (current_time - last_a_time) < KEY_HOLD_TIME;
        int turn_right = (current_time - last_d_time) < KEY_HOLD_TIME;

        // 速度计算
        double target_speed = car.speed;
        if (accel_fwd) {
            target_speed = car.speed + ACCELERATION * delta_time;
        } else if (accel_bwd) {
            target_speed = car.speed - ACCELERATION * delta_time;
        } else {
            if (car.speed > 0) {
                target_speed = car.speed - FRICTION * delta_time;
                if (target_speed < 0) target_speed = 0;
            } else if (car.speed < 0) {
                target_speed = car.speed + FRICTION * delta_time;
                if (target_speed > 0) target_speed = 0;
            }
        }
        if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
        if (target_speed < -MAX_SPEED/2) target_speed = -MAX_SPEED/2;
        car.speed = target_speed;

        // 转向计算（速度越快转向越迟钝）
        double turn = 0.0;
        if (turn_left) turn = TURN_SPEED * delta_time;
        if (turn_right) turn = -TURN_SPEED * delta_time;
        double turn_factor = 1.0 / (1.0 + fabs(car.speed) * 0.2); // 速度影响系数
        car.yaw += turn * turn_factor;

        // 更新位置
        car.position.x += car.speed * delta_time * sin(car.yaw);
        car.position.z += car.speed * delta_time * cos(car.yaw);
        int nearest = (int)(car.position.z / 2.0);
        if (nearest >= 0 && nearest < num_segments)
            car.position.y = track[nearest].center.y + 0.3;

        // 相机设置
        Vec3 eye = { car.position.x - 2.5 * sin(car.yaw), car.position.y + 2.0, car.position.z - 2.5 * cos(car.yaw) };
        Vec3 target = { car.position.x + 6 * sin(car.yaw), car.position.y, car.position.z + 6 * cos(car.yaw) };
        Vec3 up = {0, 1, 0};
        Matrix view = look_at_matrix(eye, target, up);
        Matrix vp = matrix_multiply(&proj, &view);

        erase();

        // 绘制赛道
        for (int i = 1; i < num_segments; i++) {
            draw_line(&track[i-1].center, &track[i].center, &vp, '.');
            draw_line(&track[i-1].left, &track[i].left, &vp, '#');
            draw_line(&track[i-1].right, &track[i].right, &vp, '#');
            if (i % 5 == 0) draw_line(&track[i].left, &track[i].right, &vp, '-');
        }

        // 绘制赛车
        int sx, sy;
        if (project_point(&car.position, &vp, &sx, &sy)) mvaddch(sy, sx, '@');

        // 显示FPS和状态
        double fps = 1.0 / delta_time;
        mvprintw(0, 0, "FPS: %.1f | Speed: %.2f m/s | Yaw: %.2f rad | Pos: (%.1f, %.1f, %.1f)",
                 fps, car.speed, car.yaw, car.position.x, car.position.y, car.position.z);
        mvprintw(1, 0, "Hold W/S to accelerate, A/D to turn. Q to quit.");

        refresh();
        usleep(15000); // ~66 FPS，降低帧率使重复字符更容易跟上
    }

    free(track);
    endwin();
    return 0;
}
