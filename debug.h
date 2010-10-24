/*
 * Jaldi Debugging
 * 
 * Todo: 
 * - Currently just a wrapper around printk; what more do we need?
 * - Should be controlled with a config statement, not a bit in the actual 
 *   function call...
 * 
 * Originally based on ath's debug.
 */

#ifndef JALDI_DEBUG_H
#define JALDI_DEBUG_H

#include "jaldi.h"

enum jaldi_debug_level {
	JALDI_DEBUG,
	JALDI_INFO,
	JALDI_ALERT,
	JALDI_FATAL,
}

void jaldi_print(int level, const char *fmt, ...)
	__attribute__ ((format (printf, 3, 4))); // FIXME: reflect adding in the level indicator

#endif /* JALDI_DEBUG_H */
