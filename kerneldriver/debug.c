/*
 * Jaldi debugging
 */

#include "jaldi.h"

void jaldi_print(int level, const char *fmt, ...)
{
	va_list args;

	if (likely(!(JALDI_DEBUG_ON)))
		return;

	va_start(args, fmt);
	printk(KERN_EMERG "jaldi: ");
	vprintk(fmt, args);
	va_end(args);
}
