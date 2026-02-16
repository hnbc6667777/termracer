#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>   // for gettimeofday

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
        if (i % 10 == 0) {
            angle += (rand() % 100 - 50) / 500.0;
        }
        double height_change = (rand() % 200 - 100) / 500.0;
        y += height_change;
        if (y < -1) y = -1;
        if (y > 1) y = 1;

        x += step * sin(angle);
        z += step * cos(angle);

        track[i].center.x = x;
        track[i].center.y = y;
        track[i].center.z = z;

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

int main() {
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);

    srand(time(NULL));

    int num_segments = 800;
    double track_width = 3.0;   // \u52a0\u5bbd
    TrackSegment* track = generate_track(num_segments, track_width);

    int max_y, max_x;
    getmaxyx(stdscr, max_y, max_x);
    double aspect = (double)max_x / max_y;

    // \u6295\u5f71\u77e9\u9635\uff0c\u7f29\u5c0f\u53ef\u89c6\u8303\u56f4\u4f7f\u7269\u4f53\u663e\u5f97\u66f4\u5927
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

    // \u65f6\u95f4\u76f8\u5173\u53d8\u91cf
    struct timeval prev_time, current_time;
    gettimeofday(&prev_time, NULL);
    double delta_time = 0.016; // \u521d\u59cb\u9ed8\u8ba4\u503c\u7ea660FPS
    double fps = 0.0;
    int frame_count = 0;
    double fps_timer = 0.0;

    int running = 1;
    while (running) {
        // \u8ba1\u7b97\u65f6\u95f4\u5dee
        gettimeofday(&current_time, NULL);
        delta_time = (current_time.tv_sec - prev_time.tv_sec) + 
                     (current_time.tv_usec - prev_time.tv_usec) / 1000000.0;
        // \u9650\u5236\u6700\u5927delta_time\u9632\u6b62\u8df3\u8dc3\u8fc7\u5927
        if (delta_time > 0.1) delta_time = 0.1;
        prev_time = current_time;

        // \u7b80\u5355\u7684FPS\u8ba1\u7b97\uff08\u6bcf\u79d2\u66f4\u65b0\uff09
        fps_timer += delta_time;
        frame_count++;
        if (fps_timer >= 1.0) {
            fps = frame_count / fps_timer;
            frame_count = 0;
            fps_timer = 0.0;
        }

        // \u5904\u7406\u8f93\u5165
        int ch = getch();
        switch (ch) {
            case 'w': car.speed += 5.0; break;      // \u589e\u52a0 5 m/s\uff08\u77ac\u95f4\u52a0\u901f\uff09
            case 's': car.speed -= 5.0; break;      // \u51cf\u901f 5 m/s
            case 'a': car.yaw += 1.0; break;        // \u8f6c\u5411 1 \u5f27\u5ea6\uff08\u7ea657\u5ea6\uff09
            case 'd': car.yaw -= 1.0; break;
            case 'q': running = 0; break;
        }
        // \u9650\u5236\u901f\u5ea6\u8303\u56f4
        if (car.speed > 30.0) car.speed = 30.0;
        if (car.speed < -10.0) car.speed = -10.0;

        // \u66f4\u65b0\u8d5b\u8f66\u4f4d\u7f6e\uff08\u57fa\u4e8e\u901f\u5ea6\u548c\u65f6\u95f4\uff09
        car.position.x += car.speed * sin(car.yaw) * delta_time;
        car.position.z += car.speed * cos(car.yaw) * delta_time;

        // \u9ad8\u5ea6\u8ddf\u968f\uff08\u53d6\u6700\u8fd1\u8d5b\u9053\u6bb5\uff09
        int nearest = (int)(car.position.z / 2.0);
        if (nearest < 0) nearest = 0;
        if (nearest >= num_segments) nearest = num_segments - 1;
        car.position.y = track[nearest].center.y + 0.3;

        // \u8bbe\u7f6e\u76f8\u673a\uff08\u62c9\u8fd1\u5e76\u964d\u4f4e\u9ad8\u5ea6\uff09
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

        // \u663e\u793a\u4fe1\u606f\uff08\u5e26\u5355\u4f4d\uff09
        mvprintw(0, 0, "FPS: %.1f", fps);
        mvprintw(1, 0, "Speed: %.2f m/s  Yaw: %.2f rad  Pos: (%.1f, %.1f, %.1f)", 
                 car.speed, car.yaw, car.position.x, car.position.y, car.position.z);
        mvprintw(2, 0, "Controls: W/S (accelerate/brake +-5 m/s per press), A/D (turn), Q quit");
        mvprintw(3, 0, "Camera height: 2.0 units, distance: 2.5 units");

        refresh();
        usleep(10000); // \u5c0f\u5e45\u7761\u7720\u907f\u514dCPU\u8fc7\u9ad8\uff0c\u4f46\u4e3b\u8981\u9760\u65f6\u95f4\u5dee\u63a7\u5236\u8fd0\u52a8
    }

    free(track);
    endwin();
    return 0;
}
