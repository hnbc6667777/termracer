#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>   // \u7528\u4e8e\u9ad8\u7cbe\u5ea6\u8ba1\u65f6

// 3D\u5411\u91cf\u7ed3\u6784
typedef struct {
    double x, y, z;
} Vec3;

// 4x4\u77e9\u9635\u7ed3\u6784\uff08\u5217\u4f18\u5148\uff09
typedef struct {
    double m[4][4];
} Matrix;

// \u6295\u5f71\u77e9\u9635\u53c2\u6570
typedef struct {
    double l, r, b, t, n, f;
} Frustum;

// \u8d5b\u9053\u70b9\u7ed3\u6784
typedef struct {
    Vec3 center;
    Vec3 left;
    Vec3 right;
} TrackSegment;

// ==================== \u5411\u91cf\u8fd0\u7b97 ====================
Vec3 vec3_add(Vec3 a, Vec3 b) {
    return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 vec3_sub(Vec3 a, Vec3 b) {
    return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 vec3_scale(Vec3 v, double s) {
    return (Vec3){v.x * s, v.y * s, v.z * s};
}
double vec3_dot(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return (Vec3){
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}
Vec3 vec3_normalize(Vec3 v) {
    double len = sqrt(vec3_dot(v, v));
    if (len < 1e-6) return v;
    return vec3_scale(v, 1.0/len);
}

// ==================== \u77e9\u9635\u8fd0\u7b97 ====================
// \u5411\u91cf\u4e0e\u77e9\u9635\u4e58\u6cd5
Vec3 transform_point(const Vec3* v, const Matrix* mat) {
    Vec3 res;
    double w = mat->m[3][0] * v->x + mat->m[3][1] * v->y + mat->m[3][2] * v->z + mat->m[3][3];
    if (fabs(w) > 1e-6) {
        res.x = (mat->m[0][0] * v->x + mat->m[0][1] * v->y + mat->m[0][2] * v->z + mat->m[0][3]) / w;
        res.y = (mat->m[1][0] * v->x + mat->m[1][1] * v->y + mat->m[1][2] * v->z + mat->m[1][3]) / w;
        res.z = (mat->m[2][0] * v->x + mat->m[2][1] * v->y + mat->m[2][2] * v->z + mat->m[2][3]) / w;
    } else {
        res.x = res.y = res.z = 0;
    }
    return res;
}

// \u4e24\u4e2a\u77e9\u9635\u76f8\u4e58
Matrix matrix_multiply(const Matrix* a, const Matrix* b) {
    Matrix res;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res.m[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                res.m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
    return res;
}

// \u8bbe\u7f6e\u900f\u89c6\u6295\u5f71\u77e9\u9635\uff08\u6839\u636e\u7ed9\u5b9a\u516c\u5f0f\uff09
Matrix projection_matrix(const Frustum* frust) {
    Matrix mat = {0};
    double rl = frust->r - frust->l;
    double tb = frust->t - frust->b;
    double fn = frust->f - frust->n;

    mat.m[0][0] = 2 * frust->n / rl;
    mat.m[0][2] = (frust->r + frust->l) / rl;
    mat.m[1][1] = 2 * frust->n / tb;
    mat.m[1][2] = (frust->t + frust->b) / tb;
    mat.m[2][2] = -(frust->f + frust->n) / fn;
    mat.m[2][3] = -2 * frust->f * frust->n / fn;
    mat.m[3][2] = -1;
    return mat;
}

// LookAt\u77e9\u9635\uff08\u76f8\u673a\u4f4d\u7f6e\uff0c\u76ee\u6807\u70b9\uff0c\u4e0a\u5411\u91cf\uff09
Matrix look_at_matrix(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 z = vec3_normalize(vec3_sub(eye, target)); // \u76f8\u673a\u6307\u5411\u7269\u4f53\u7684\u65b9\u5411\uff08OpenGL\u4e2d\u89c6\u7ebf\u65b9\u5411\u662f -z\uff09
    Vec3 x = vec3_normalize(vec3_cross(up, z));
    Vec3 y = vec3_cross(z, x);

    Matrix mat = {0};
    mat.m[0][0] = x.x; mat.m[0][1] = x.y; mat.m[0][2] = x.z; mat.m[0][3] = -vec3_dot(x, eye);
    mat.m[1][0] = y.x; mat.m[1][1] = y.y; mat.m[1][2] = y.z; mat.m[1][3] = -vec3_dot(y, eye);
    mat.m[2][0] = z.x; mat.m[2][1] = z.y; mat.m[2][2] = z.z; mat.m[2][3] = -vec3_dot(z, eye);
    mat.m[3][3] = 1;
    return mat;
}

// ==================== \u8d5b\u9053\u751f\u6210 ====================
// \u968f\u673a\u751f\u6210\u5e26\u6709\u8f7b\u5fae\u9ad8\u5ea6\u53d8\u5316\u7684\u8d5b\u9053
TrackSegment* generate_track(int num_segments, double track_width) {
    TrackSegment* track = malloc(num_segments * sizeof(TrackSegment));
    double z = 0;
    double y = 0;
    double x = 0;
    double angle = 0;
    double step = 2.0;

    for (int i = 0; i < num_segments; i++) {
        // \u968f\u673a\u6539\u53d8\u65b9\u5411
        if (i % 10 == 0) {
            angle += (rand() % 100 - 50) / 500.0;
        }
        // \u968f\u673a\u9ad8\u5ea6\u53d8\u5316
        double height_change = (rand() % 200 - 100) / 500.0; // -0.2 ~ 0.2
        y += height_change;
        if (y < -1) y = -1;
        if (y > 1) y = 1;

        x += step * sin(angle);
        z += step * cos(angle);

        track[i].center.x = x;
        track[i].center.y = y;
        track[i].center.z = z;

        // \u5de6\u53f3\u8fb9\u754c
        Vec3 dir = {sin(angle), 0, cos(angle)};
        Vec3 left_dir = {cos(angle), 0, -sin(angle)};
        track[i].left.x = x + left_dir.x * track_width;
        track[i].left.y = y;
        track[i].left.z = z + left_dir.z * track_width;
        track[i].right.x = x - left_dir.x * track_width;
        track[i].right.y = y;
        track[i].right.z = z - left_dir.z * track_width;
    }
    return track;
}

// ==================== \u6e32\u67d3 ====================
int project_point(const Vec3* world, const Matrix* vp, int* sx, int* sy) {
    Vec3 clip = transform_point(world, vp);
    if (clip.z < -1 || clip.z > 1 || clip.x < -1 || clip.x > 1 || clip.y < -1 || clip.y > 1)
        return 0;
    int max_y, max_x;
    getmaxyx(stdscr, max_y, max_x);
    *sx = (int)((clip.x + 1) * 0.5 * (max_x - 1));
    *sy = (int)((1 - (clip.y + 1) * 0.5) * (max_y - 1));
    return 1;
}

// \u753b\u7ebf\uff08\u7ebf\u6027\u63d2\u503c\uff09
void draw_line(const Vec3* a, const Vec3* b, const Matrix* vp, char ch) {
    int x1, y1, x2, y2;
    if (!project_point(a, vp, &x1, &y1) || !project_point(b, vp, &x2, &y2))
        return;

    int dx = abs(x2 - x1), dy = abs(y2 - y1);
    int steps = dx > dy ? dx : dy;
    if (steps < 1) steps = 1;
    for (int i = 0; i <= steps; i++) {
        double t = (double)i / steps;
        int x = x1 + (int)(t * (x2 - x1));
        int y = y1 + (int)(t * (y2 - y1));
        if (x >= 0 && x < COLS && y >= 0 && y < LINES)
            mvaddch(y, x, ch);
    }
}

// ==================== \u6e38\u620f\u72b6\u6001 ====================
typedef struct {
    Vec3 position;
    double yaw;      // \u5f27\u5ea6
    double speed;    // \u7c73/\u79d2
} Car;

// \u83b7\u53d6\u5f53\u524d\u65f6\u95f4\uff08\u79d2\uff09
double get_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

int main() {
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);

    srand(time(NULL));

    // \u751f\u6210\u8d5b\u9053\uff08\u589e\u52a0\u6bb5\u6570\u4ee5\u663e\u793a\u66f4\u8fdc\uff09
    int num_segments = 800;
    double track_width = 3.0;   // \u52a0\u5bbd\u8d5b\u9053
    TrackSegment* track = generate_track(num_segments, track_width);

    int max_y, max_x;
    getmaxyx(stdscr, max_y, max_x);
    double aspect = (double)max_x / max_y;

    // \u6295\u5f71\u77e9\u9635\uff08\u9002\u5f53\u7f29\u5c0f\u53ef\u89c6\u8303\u56f4\u4ee5\u653e\u5927\u8d5b\u9053\uff09
    Frustum frust = {
        .l = -aspect * 0.4,
        .r =  aspect * 0.4,
        .b = -0.3,
        .t =  0.3,
        .n = 0.5,
        .f = 200.0
    };
    Matrix proj = projection_matrix(&frust);

    // \u521d\u59cb\u5316\u8d5b\u8f66
    Car car;
    car.position.x = 0;
    car.position.y = 0.5;
    car.position.z = 0;
    car.yaw = 0;
    car.speed = 0;

    // \u63a7\u5236\u53c2\u6570\uff08\u57fa\u4e8e\u65f6\u95f4\uff09
    const double ACCELERATION = 3.0;      // \u52a0\u901f\u5ea6 m/s²
    const double TURN_SPEED = 1.2;        // \u8f6c\u5411\u901f\u5ea6 rad/s
    const double MAX_SPEED = 10.0;         // \u6700\u5927\u901f\u5ea6 m/s
    const double FRICTION = 2.0;           // \u6469\u64e6\u51cf\u901f\u5ea6 m/s²\uff08\u4e0d\u52a0\u901f\u65f6\uff09

    double last_time = get_time();
    int running = 1;

    while (running) {
        double current_time = get_time();
        double delta_time = current_time - last_time;
        last_time = current_time;
        if (delta_time > 0.1) delta_time = 0.1; // \u9632\u6b62\u5361\u987f\u65f6\u8df3\u8dc3\u592a\u5927

        // \u5904\u7406\u8f93\u5165\uff08\u57fa\u4e8e\u65f6\u95f4\u7684\u589e\u91cf\uff09
        int ch = getch();
        double target_speed = car.speed;
        double turn = 0.0;

        switch (ch) {
            case 'w': target_speed = car.speed + ACCELERATION * delta_time; break;
            case 's': target_speed = car.speed - ACCELERATION * delta_time; break;
            case 'a': turn = TURN_SPEED * delta_time; break;  // \u6b63\u8f6c\u5411\u5de6
            case 'd': turn = -TURN_SPEED * delta_time; break;
            case 'q': running = 0; break;
        }

        // \u5e94\u7528\u6469\u64e6\uff08\u4e0d\u52a0\u901f\u65f6\u51cf\u901f\uff09
        if (ch != 'w' && ch != 's') {
            if (car.speed > 0) {
                target_speed = car.speed - FRICTION * delta_time;
                if (target_speed < 0) target_speed = 0;
            } else if (car.speed < 0) {
                target_speed = car.speed + FRICTION * delta_time;
                if (target_speed > 0) target_speed = 0;
            }
        }

        // \u9650\u5236\u901f\u5ea6\u8303\u56f4
        if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
        if (target_speed < -MAX_SPEED/2) target_speed = -MAX_SPEED/2; // \u5012\u8f66\u9650\u901f

        car.speed = target_speed;

        // \u5e94\u7528\u8f6c\u5411\uff08\u57fa\u4e8e\u901f\u5ea6\uff0c\u901f\u5ea6\u8d8a\u5feb\u8f6c\u5411\u6548\u679c\u8d8a\u5f31\uff0c\u6a21\u62df\u771f\u5b9e\u611f\uff09
        double turn_factor = 1.0 / (1.0 + fabs(car.speed) * 0.3); // \u901f\u5ea6\u8d8a\u5feb\u8f6c\u5411\u8d8a\u8fdf\u949d
        car.yaw += turn * turn_factor;

        // \u66f4\u65b0\u8d5b\u8f66\u4f4d\u7f6e\uff08\u57fa\u4e8e\u901f\u5ea6\u548c\u65f6\u95f4\uff09
        car.position.x += car.speed * delta_time * sin(car.yaw);
        car.position.z += car.speed * delta_time * cos(car.yaw);

        // \u9ad8\u5ea6\u8ddf\u968f\uff08\u53d6\u6700\u8fd1\u8d5b\u9053\u6bb5\u7684\u9ad8\u5ea6\uff09
        int nearest = (int)(car.position.z / 2.0);
        if (nearest >= 0 && nearest < num_segments) {
            car.position.y = track[nearest].center.y + 0.3;
        }

        // \u8bbe\u7f6e\u76f8\u673a\uff1a\u4f4d\u4e8e\u8d5b\u8f66\u540e\u65b9\u4e0a\u65b9\uff0c\u770b\u5411\u8d5b\u8f66\u524d\u65b9
        Vec3 eye = {
            car.position.x - 2.5 * sin(car.yaw),
            car.position.y + 2.0,
            car.position.z - 2.5 * cos(car.yaw)
        };
        Vec3 target = {
            car.position.x + 6 * sin(car.yaw),
            car.position.y,
            car.position.z + 6 * cos(car.yaw)
        };
        Vec3 up = {0, 1, 0};

        Matrix view = look_at_matrix(eye, target, up);
        Matrix vp = matrix_multiply(&proj, &view);

        erase();

        // \u7ed8\u5236\u8d5b\u9053
        for (int i = 1; i < num_segments; i++) {
            draw_line(&track[i-1].center, &track[i].center, &vp, '.');
            draw_line(&track[i-1].left, &track[i].left, &vp, '#');
            draw_line(&track[i-1].right, &track[i].right, &vp, '#');
            if (i % 5 == 0)
                draw_line(&track[i].left, &track[i].right, &vp, '-');
        }

        // \u7ed8\u5236\u8d5b\u8f66
        int sx, sy;
        if (project_point(&car.position, &vp, &sx, &sy)) {
            mvaddch(sy, sx, '@');
        }

        // \u8ba1\u7b97\u5e76\u663e\u793aFPS
        double fps = 1.0 / delta_time;
        mvprintw(0, 0, "FPS: %.1f | Speed: %.2f m/s | Yaw: %.2f rad | Pos: (%.1f, %.1f, %.1f)",
                 fps, car.speed, car.yaw, car.position.x, car.position.y, car.position.z);
        mvprintw(1, 0, "Controls: W/S (accelerate), A/D (turn), Q (quit) | Turn speed: %.1f rad/s", TURN_SPEED);

        refresh();
        usleep(10000); // \u7ea6100 FPS\u4e0a\u9650\uff0c\u907f\u514d\u5360\u7528\u8fc7\u591aCPU
    }

    free(track);
    endwin();
    return 0;
}
