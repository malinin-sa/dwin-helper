#include "debug.h"

int debug_to_syslog = 0;
FILE * log_f = NULL;  // Logfile

void debug_output(int level, int errno_flag, const char *fmt, ...)
{
    int errno_save, n;
    char buf[1024];
//#if (!defined(BSD)) && (!(_POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600) && ! _GNU_SOURCE)
    char msg_buf[100];
//#endif
    va_list ap;

    /*
     * First we save the current 'error' value.  This is required because
     * the subsequent calls to vsnprintf could conceivably change it!
     */
    errno_save = errno;

    /* Add the user's message */
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);

    /* If errno_flag is set, add on the library error message */
    if (errno_flag) {
        strncat(buf, ": ", 1024 - strlen(buf));
        n += 2;
        /*
         * this is bad - apparently gcc/libc wants to use the non-standard GNU
         * version of strerroundeclaredr_r, which doesn't actually put the message into
         * my buffer :-(.  I have put in a 'hack' to get around this.
         */
#if (defined(BSD))
        strerror_r(errno_save, buf + n, sizeof(buf) - n);    /* 2 for the ': ' */
#elif (_POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600) && ! _GNU_SOURCE
        strerror_r(errno_save, buf + n, sizeof(buf) - n);
#else
        strncat(buf, strerror_r(errno_save, msg_buf, sizeof(msg_buf)), 1024 - strlen(buf));
#endif
    }
    /* If 'level' is not negative, send the message to the syslog */
    if (debug_to_syslog > 0)
        syslog(level, "%s", buf);
    else
    {
    	/* For printing to stderr we need to add a newline */
    	if (log_f == NULL)
    	{
    		strcat(buf, "\n");
    		fputs(buf, stderr);
    		fflush(stderr);
    	}
    	else
    	{
    		strcat(buf, "\n");
    		fputs(buf, log_f);
    		fflush(log_f);
    	}
    }

    /* Clean up the argument list routine */
    va_end(ap);
}
