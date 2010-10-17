/*
 * Jaldi debugging
 */

#include "jaldi.h"
#include "debug.h"

bool debug = 1;

void jaldi_print(int level; const char *fmt, ...)
{
	va_list args;

	if (likely(!(debug)))
		return;

	va_start(args, fmt);
	printk(KERN_DEBUG "jaldi: ");
	vprintk(fmt, args);
	va_end(args);
}
