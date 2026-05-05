// simple_logger_linux.c
// 빌드: gcc -O2 -pthread -DLOGGER_DEMO simple_logger_linux.c -o logger_demo
// 사용: 라이브러리로 쓸 땐 LOGGER_DEMO 제외하고 컴파일 후 log_init()/log_xxx() 호출

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

#ifndef LOG_MAX_BYTES
#define LOG_MAX_BYTES (1024*1024)  // 기본 1MB 초과 시 회전
#endif

typedef enum {
    LOG_TRACE = 0,
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR
} log_level_t;

static const char* LEVEL_STR[] = {"TRACE","DEBUG","INFO","WARN","ERROR"};

typedef struct {
    FILE* fp;
    char  path[1024];
    log_level_t level;
    pthread_mutex_t mu;
    int rotation_enabled;
    size_t max_bytes;
} logger_t;

static logger_t G; // 전역 로거(간단 사용 목적)

static int file_size_bytes(const char* path, size_t* out) {
    struct stat st;
    if (stat(path, &st) != 0) return -1;
    *out = (size_t)st.st_size;
    return 0;
}

static void now_yyyymmdd_hhmmss(char* buf, size_t n) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    time_t t = ts.tv_sec;
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(buf, n, "%Y-%m-%d %H:%M:%S", &tmv);
}

static int rotate_if_needed() {
    if (!G.rotation_enabled || !G.fp || G.path[0] == '\0') return 0;

    size_t sz = 0;
    if (file_size_bytes(G.path, &sz) != 0) return 0;
    if (sz < G.max_bytes) return 0;

    // 현재 파일 닫고 path -> path.1 로 교체
    fclose(G.fp);
    G.fp = NULL;

    char bak[1152];
    snprintf(bak, sizeof(bak), "%s.1", G.path);
    // 기존 백업 삭제(있어도 무시)
    remove(bak);

    // rename 실패해도 새 파일 열어서 진행
    (void)rename(G.path, bak);

    G.fp = fopen(G.path, "a");
    if (!G.fp) return -1;
    return 0;
}

int log_init(const char* path, log_level_t level) {
    memset(&G, 0, sizeof(G));
    G.level = level;
    G.rotation_enabled = 1;
    G.max_bytes = LOG_MAX_BYTES;

    if (!path || !*path) path = "app.log";
    snprintf(G.path, sizeof(G.path), "%s", path);

    G.fp = fopen(G.path, "a");
    if (!G.fp) return -1;

    if (pthread_mutex_init(&G.mu, NULL) != 0) {
        fclose(G.fp);
        G.fp = NULL;
        return -1;
    }
    return 0;
}

void log_set_level(log_level_t level) { G.level = level; }

void log_set_rotation(int enabled, size_t max_bytes) {
    G.rotation_enabled = enabled;
    if (max_bytes > 0) G.max_bytes = max_bytes;
}

static void vlog_write(log_level_t lv, const char* tag, const char* fmt, va_list ap) {
    if (!G.fp) return;

    char tbuf[32];
    now_yyyymmdd_hhmmss(tbuf, sizeof(tbuf));

    char line[4096];
    int off = snprintf(line, sizeof(line), "%s [%s]", tbuf, LEVEL_STR[lv]);
    if (tag && *tag) off += snprintf(line + off, sizeof(line) - off, " <%s>", tag);
    off += snprintf(line + off, sizeof(line) - off, " ");
    vsnprintf(line + off, sizeof(line) - off, fmt, ap);

    size_t len = strnlen(line, sizeof(line));
    if (len == 0 || line[len-1] != '\n') {
        if (len < sizeof(line)-1) {
            line[len++] = '\n';
            line[len] = '\0';
        }
    }

    pthread_mutex_lock(&G.mu);
    rotate_if_needed();
    fputs(line, G.fp);
    fflush(G.fp);
    pthread_mutex_unlock(&G.mu);
}

void log_write(log_level_t lv, const char* tag, const char* fmt, ...) {
    if (lv < G.level) return;
    va_list ap; va_start(ap, fmt);
    vlog_write(lv, tag, fmt, ap);
    va_end(ap);
}

void log_trace(const char* fmt, ...) {
    if (LOG_TRACE < G.level) return;
    va_list ap; va_start(ap, fmt);
    vlog_write(LOG_TRACE, NULL, fmt, ap);
    va_end(ap);
}
void log_debug(const char* fmt, ...) {
    if (LOG_DEBUG < G.level) return;
    va_list ap; va_start(ap, fmt);
    vlog_write(LOG_DEBUG, NULL, fmt, ap);
    va_end(ap);
}
void log_info(const char* fmt, ...) {
    if (LOG_INFO < G.level) return;
    va_list ap; va_start(ap, fmt);
    vlog_write(LOG_INFO, NULL, fmt, ap);
    va_end(ap);
}
void log_warn(const char* fmt, ...) {
    if (LOG_WARN < G.level) return;
    va_list ap; va_start(ap, fmt);
    vlog_write(LOG_WARN, NULL, fmt, ap);
    va_end(ap);
}
void log_error(const char* fmt, ...) {
    if (LOG_ERROR < G.level) return;
    va_list ap; va_start(ap, fmt);
    vlog_write(LOG_ERROR, NULL, fmt, ap);
    va_end(ap);
}

void log_close(void) {
    if (G.fp) { fflush(G.fp); fclose(G.fp); G.fp = NULL; }
    pthread_mutex_destroy(&G.mu);
}

/* ------------------ 데모용 main (라이브러리로 쓰면 LOGGER_DEMO 제거) ------------------ */
#ifdef LOGGER_DEMO
int main(void) {
    if (log_init("app.log", LOG_DEBUG) != 0) {
        fprintf(stderr, "log_init failed: %s\n", strerror(errno));
        return 1;
    }
    log_set_rotation(1, 256*1024); // 256KB 넘으면 app.log.1 로 회전

    log_info("프로그램 시작: pid=%d", (int)getpid());
    log_debug("디버그 메시지: x=%d", 42);
    log_warn("경고: 기본 설정으로 동작합니다.");
    log_error("에러 예시: %s", "리소스 없음");

    for (int i = 0; i < 5000; ++i) {
        log_write(LOG_TRACE, "loop", "i=%d, rnd=%d", i, rand());
    }

    log_close();
    return 0;
}
#endif
