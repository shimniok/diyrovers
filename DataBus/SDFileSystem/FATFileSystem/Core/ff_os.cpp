/*
 * ff_os.cpp
 *
 * Description: CoOS-specific re-entrant sync implementation
 *
 *  Created on: Dec 29, 2013
 *      Author: mes
 */

#include "ff.h"
#include "ffconf.h"
#include <CoOS.h>
#include "integer.h"

/* Create a sync object */
int ff_cre_syncobj (BYTE, _SYNC_t *sync) {
	return 1;
}

/* Delete a sync object */
int ff_del_syncobj (_SYNC_t sync) {
	return 1;
}

/* Lock sync object */
int ff_req_grant (_SYNC_t sync) {
	return 1;
}

/* Unlock sync object */
void ff_rel_grant (_SYNC_t sync) {
	return;
}
