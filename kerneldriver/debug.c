/*
 * Jaldi debugging
 */

#include "jaldi.h"

void jaldi_print(int level, const char *fmt, ...)
{
	va_list args;

	if (JALDI_DEBUG_ON && level <= JALDI_DEBUG_LEVEL) {
		va_start(args, fmt);
		printk(KERN_EMERG "jaldi [%d]: ", level);
		vprintk(fmt, args);
		va_end(args);
	}
}
