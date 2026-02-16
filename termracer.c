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
    Vec3 res; double w = mat->m[3][0] * v->x + mat->m[3][1] * v->y + mat->m[3][2] * v->z + mat->m[3][3];
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
    mat.m[0][0] = 2 * frust->n / rl; mat.m[0][2] = (frust->r + frust->l) / rl;
    mat.m[1][1] = 2 * frust->n / tb; mat.m[1][2] = (frust->t + frust->b) / tb;
    mat.m[2][2] = -(frust->f + frust->n) / fn; mat.m[2][3] = -2 * frust->f * frust->n / fn;
    mat.m[3][2] = -1; return mat;
}

Matrix look_at_matrix(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 z = vec3_normalize(vec3_sub(eye, target));
    Vec3 x = vec3_normalize(vec3_cross(up, z));
    Vec3 y = vec3_cross(z, x);
    Matrix mat = {0};
    mat.m[0][0] = x.x; mat.m[0][1] = x.y; mat.m[0][2] = x.z; mat.m[0][3] = -vec3_dot(x, eye);
    mat.m[1][0] = y.x; mat.m[1][1] = y.y; mat.m[1][2] = y.z; mat.m[1][3] = -vec3_dot(y, eye);
    mat.m[2][0] = z.x; mat.m[2][1] = z.y; mat.m[2][2] = z.z; mat.m[2][3] = -vec3_dot(z, eye);
    mat.m[3][3] = 1; return mat;
}

// ==================== 赛道生成 ====================
TrackSegment* generate_track(int num_segments, double track_width) {
    TrackSegment* track = malloc(num_segments * sizeof(TrackSegment));
    double z = 0, y = 0, x = 0, angle = 0, step = 2.0;
    for (int i = 0; i < num_segments; i++) {
        if (i % 10 == 0) angle += (rand() % 100 - 50) / 500.0;
        y += (rand() % 200 - 100) / 500.0;
        if (y < -1) y = -1; if (y > 1) y = 1;
        x += step * sin(angle); z += step * cos(angle);
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
    int max_y, max_x; getmaxyx(stdscr, max_y, max_x);
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

// ==================== 游戏状态 & Kitty 协议处理 ====================
typedef struct { Vec3 position; double yaw; double speed; } Car;

// 按键状态
typedef enum { KEY_STATE_RELEASED, KEY_STATE_PRESSED } KeyState;
KeyState key_w = KEY_STATE_RELEASED;
KeyState key_s = KEY_STATE_RELEASED;
KeyState key_a = KEY_STATE_RELEASED;
KeyState key_d = KEY_STATE_RELEASED;

double get_time() { struct timeval tv; gettimeofday(&tv, NULL); return tv.tv_sec + tv.tv_usec / 1000000.0; }

// Kitty协议解析函数
void handle_kitty_input(const char *buf, int len) {
    // 寻找CSI序列: \x1b[
    for (int i = 0; i < len - 1; i++) {
        if (buf[i] == 0x1b && buf[i+1] == '[') {
            // 提取参数，格式: 键码;修饰符;事件类型 u
            int keycode = 0, mods = 0, event = 0;
            char *endptr;
            // 简单解析: 从 \x1b[ 后开始
            const char *ptr = buf + i + 2;
            keycode = strtol(ptr, &endptr, 10);
            if (endptr && *endptr == ';') {
                ptr = endptr + 1;
                mods = strtol(ptr, &endptr, 10);
                if (endptr && *endptr == ';') {
                    ptr = endptr + 1;
                    event = strtol(ptr, &endptr, 10);
                    if (endptr && *endptr == 'u') {
                        // 成功解析Kitty事件
                        // 将键码映射到游戏按键 (这里以字母键为例)
                        if (keycode == 'w' || keycode == 'W') {
                            key_w = (event == 1 || event == 2) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED; // press/repeat视为按下
                        } else if (keycode == 's' || keycode == 'S') {
                            key_s = (event == 1 || event == 2) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
                        } else if (keycode == 'a' || keycode == 'A') {
                            key_a = (event == 1 || event == 2) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
                        } else if (keycode == 'd' || keycode == 'D') {
                            key_d = (event == 1 || event == 2) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
                        } else if (keycode == 'q' || keycode == 'Q') {
                            // 处理退出键，这里只处理按下事件以避免重复退出
                            if (event == 1) {
                                // 设置退出标志，这里简单处理，需要从主循环访问，所以先跳过，在主循环处理q键
                            }
                        }
                        // 忽略其他键
                    }
                }
            }
        }
    }
}

int main() {
    initscr(); cbreak(); noecho(); keypad(stdscr, TRUE); nodelay(stdscr, TRUE); curs_set(0);
    srand(time(NULL));

    // 启用Kitty键盘协议
    printf("\x1b[>1u"); // 启用协议 [citation:1]
    fflush(stdout);

    int num_segments = 800;
    double track_width = 3.0;
    TrackSegment* track = generate_track(num_segments, track_width);

    int max_y, max_x; getmaxyx(stdscr, max_y, max_x);
    double aspect = (double)max_x / max_y;
    Frustum frust = { -aspect * 0.4, aspect * 0.4, -0.3, 0.3, 0.5, 200.0 };
    Matrix proj = projection_matrix(&frust);

    Car car = { {0, 0.5, 0}, 0, 0 };

    const double ACCELERATION = 5.0;
    const double TURN_SPEED = 1.0;
    const double MAX_SPEED = 15.0;
    const double FRICTION = 1.2;

    double last_time = get_time();
    int running = 1;

    while (running) {
        double current_time = get_time();
        double delta_time = current_time - last_time;
        last_time = current_time;
        if (delta_time > 0.1) delta_time = 0.1;

        // 读取所有输入
        int ch;
        char input_buf[256];
        int pos = 0;
        while ((ch = getch()) != ERR && pos < 255) {
            input_buf[pos++] = ch;
        }
        if (pos > 0) {
            input_buf[pos] = '\0';
            // 检查是否是Kitty协议序列 (以ESC开头)
            if (input_buf[0] == 0x1b) {
                handle_kitty_input(input_buf, pos);
            } else {
                // 处理普通字符（作为后备）
                for (int i = 0; i < pos; i++) {
                    switch (input_buf[i]) {
                        case 'w': key_w = KEY_STATE_PRESSED; break;
                        case 's': key_s = KEY_STATE_PRESSED; break;
                        case 'a': key_a = KEY_STATE_PRESSED; break;
                        case 'd': key_d = KEY_STATE_PRESSED; break;
                        case 'q': running = 0; break;
                    }
                }
            }
        }

        // 根据按键状态更新速度和转向
        double target_speed = car.speed;
        if (key_w == KEY_STATE_PRESSED) {
            target_speed = car.speed + ACCELERATION * delta_time;
        } else if (key_s == KEY_STATE_PRESSED) {
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

        double turn = 0.0;
        if (key_a == KEY_STATE_PRESSED) turn = TURN_SPEED * delta_time;
        if (key_d == KEY_STATE_PRESSED) turn = -TURN_SPEED * delta_time;
        double turn_factor = 1.0 / (1.0 + fabs(car.speed) * 0.2);
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
        mvprintw(0, 0, "FPS: %.1f | Speed: %.2f m/s | Yaw: %.2f rad | W:%s S:%s A:%s D:%s",
                 fps, car.speed, car.yaw,
                 key_w == KEY_STATE_PRESSED ? "1" : "0",
                 key_s == KEY_STATE_PRESSED ? "1" : "0",
                 key_a == KEY_STATE_PRESSED ? "1" : "0",
                 key_d == KEY_STATE_PRESSED ? "1" : "0");

        refresh();
        usleep(15000);
    }

    // 恢复终端键盘模式
    printf("\x1b[<u");
    fflush(stdout);

    free(track);
    endwin();
    return 0;
}
