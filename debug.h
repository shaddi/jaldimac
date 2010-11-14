/*
 * Jaldi Debugging
 * 
 * Todo: 
 * - Currently just a wrapper around printk; need to implement some debug level control
 * 
 * Originally based on ath's debug.
 */

#ifndef JALDI_DEBUG_H
#define JALDI_DEBUG_H

#include "jaldi.h"

#define JALDI_DEBUG_ON 1
#define WHERESTR "[file %s, line %d]: "
#define WHEREARG __FILE__, __LINE__
#define DBG_START_MSG jaldi_print(JALDI_DEBUG,"---> Entering '%s' [file %s, line %d]\n", __FUNCTION__, WHEREARG)
#define	OHAI jaldi_print(JALDI_DEBUG,"%s [file %s, line %d]\n", __FUNCTION__, WHEREARG)

enum JALDI_DEBUG_LEVEL {
	JALDI_DEBUG,
	JALDI_INFO,
	JALDI_ALERT,
	JALDI_WARN,
	JALDI_FATAL,
};

void jaldi_print(int level, const char *fmt, ...)
	__attribute__ ((format (printf, 2, 3)));

#endif /* JALDI_DEBUG_H */
