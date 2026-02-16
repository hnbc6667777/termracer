#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

// ==================== 宏定义（可调参数） ====================
// 游戏控制
#define ACCELERATION       6.0     // 加速度 (m/s²)
#define TURN_SPEED          0.9     // 转向速度 (rad/s)
#define MAX_SPEED          16.0     // 最大前进速度 (m/s)
#define MAX_REV_SPEED       8.0     // 最大倒车速度 (m/s)
#define FRICTION            1.2     // 摩擦减速度 (m/s²)
#define KEY_HOLD_TIME       0.1     // 转向键保持判定时间 (秒)

// 赛道生成
#define NUM_SEGMENTS      800      // 赛道总段数
#define TRACK_WIDTH        6.0     // 赛道半宽 (实际宽度为 2*TRACK_WIDTH)
#define SEGMENT_STEP        2.0     // 每段前进距离
#define HEIGHT_VARIATION    1     // 高度变化幅度 (±0.2)
#define TURN_VARIATION      0.1     // 方向变化幅度 (±0.1 rad)

// 投影矩阵（视锥体）
#define FRUSTUM_L_FACTOR    0.4     // left = -aspect * FRUSTUM_L_FACTOR
#define FRUSTUM_R_FACTOR    0.4     // right = aspect * FRUSTUM_R_FACTOR
#define FRUSTUM_B          -0.3     // bottom
#define FRUSTUM_T           0.3     // top
#define FRUSTUM_N           0.5     // near
#define FRUSTUM_F         200.0     // far

// 相机参数
#define CAMERA_DIST         2.5     // 相机在车后的距离
#define CAMERA_HEIGHT       2.0     // 相机高度
#define TARGET_DIST         6.0     // 相机注视点超前距离

// ==================== 数据结构 ====================
typedef struct { double x, y, z; } Vec3;
typedef struct { double m[4][4]; } Matrix;
typedef struct { double l, r, b, t, n, f; } Frustum;
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
    double z = 0, y = 0, x = 0, angle = 0;
    for (int i = 0; i < num_segments; i++) {
        if (i % 10 == 0) angle += ((rand() % 200) - 100) / 1000.0 * TURN_VARIATION * 10; // 随机转向
        y += ((rand() % 200) - 100) / 500.0 * HEIGHT_VARIATION; // 高度变化
        if (y < -1) y = -1; if (y > 1) y = 1;
        x += SEGMENT_STEP * sin(angle);
        z += SEGMENT_STEP * cos(angle);
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

    TrackSegment* track = generate_track(NUM_SEGMENTS, TRACK_WIDTH);

    int max_y, max_x;
    getmaxyx(stdscr, max_y, max_x);
    double aspect = (double)max_x / max_y;
    Frustum frust = { -aspect * FRUSTUM_L_FACTOR, aspect * FRUSTUM_R_FACTOR, FRUSTUM_B, FRUSTUM_T, FRUSTUM_N, FRUSTUM_F };
    Matrix proj = projection_matrix(&frust);

    Car car = { {0, 0.5, 0}, 0, 0 };

    int accel_fwd_on = 0;
    int accel_bwd_on = 0;

    double last_time = get_time();
    double last_a_time = -KEY_HOLD_TIME, last_d_time = -KEY_HOLD_TIME;
    int running = 1;

    while (running) {
        double current_time = get_time();
        double delta_time = current_time - last_time;
        last_time = current_time;
        if (delta_time > 0.1) delta_time = 0.1;

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'w':
                    accel_fwd_on = !accel_fwd_on;
                    if (accel_fwd_on) accel_bwd_on = 0;
                    break;
                case 's':
                    accel_bwd_on = !accel_bwd_on;
                    if (accel_bwd_on) accel_fwd_on = 0;
                    break;
                case 'a': last_a_time = current_time; break;
                case 'd': last_d_time = current_time; break;
                case 'q': running = 0; break;
            }
        }

        // 速度计算
        double target_speed = car.speed;
        if (accel_fwd_on) {
            target_speed = car.speed + ACCELERATION * delta_time;
        } else if (accel_bwd_on) {
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
        if (target_speed < -MAX_REV_SPEED) target_speed = -MAX_REV_SPEED;
        car.speed = target_speed;

        // 转向
        int turn_left = (current_time - last_a_time) < KEY_HOLD_TIME;
        int turn_right = (current_time - last_d_time) < KEY_HOLD_TIME;
        double turn = 0.0;
        if (turn_left) turn = TURN_SPEED * delta_time;
        if (turn_right) turn = -TURN_SPEED * delta_time;
        double turn_factor = 1.0 / (1.0 + fabs(car.speed) * 0.15);
        car.yaw += turn * turn_factor;

        // 更新位置
        car.position.x += car.speed * delta_time * sin(car.yaw);
        car.position.z += car.speed * delta_time * cos(car.yaw);
        int nearest = (int)(car.position.z / SEGMENT_STEP);
        if (nearest >= 0 && nearest < NUM_SEGMENTS)
            car.position.y = track[nearest].center.y + 0.3;

        // 相机
        Vec3 eye = { car.position.x - CAMERA_DIST * sin(car.yaw), car.position.y + CAMERA_HEIGHT, car.position.z - CAMERA_DIST * cos(car.yaw) };
        Vec3 target = { car.position.x + TARGET_DIST * sin(car.yaw), car.position.y, car.position.z + TARGET_DIST * cos(car.yaw) };
        Vec3 up = {0, 1, 0};
        Matrix view = look_at_matrix(eye, target, up);
        Matrix vp = matrix_multiply(&proj, &view);

        erase();

        // 绘制赛道
        for (int i = 1; i < NUM_SEGMENTS; i++) {
            draw_line(&track[i-1].center, &track[i].center, &vp, '.');
            draw_line(&track[i-1].left, &track[i].left, &vp, '#');
            draw_line(&track[i-1].right, &track[i].right, &vp, '#');
            if (i % 5 == 0) draw_line(&track[i].left, &track[i].right, &vp, '-');
        }

        int sx, sy;
        if (project_point(&car.position, &vp, &sx, &sy)) mvaddch(sy, sx, '@');

        double fps = 1.0 / delta_time;
        const char* throttle_state = "N";
        if (accel_fwd_on) throttle_state = "FWD";
        else if (accel_bwd_on) throttle_state = "REV";
        mvprintw(0, 0, "FPS: %.1f | Throttle: %s | Speed: %.2f m/s | Yaw: %.2f rad",
                 fps, throttle_state, car.speed, car.yaw);
        mvprintw(1, 0, "W: toggle forward, S: toggle reverse, A/D: hold to turn, Q: quit");

        refresh();
        usleep(15000);
    }

    free(track);
    endwin();
    return 0;
}
