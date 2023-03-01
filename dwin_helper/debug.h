#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdio.h>
#include <syslog.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

extern int debug_to_syslog;
extern FILE * log_f;
void debug_output(int level, int errno_flag, const char *fmt, ...);

#endif

