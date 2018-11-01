/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

#include "lmic.h"

// RUNTIME STATE
static struct {
	osjob_t* scheduledjobs;
	osjob_t* runnablejobs;
} OS;

void os_init(lmicApi_t lmicApi) {
	memset(&OS, 0x00, sizeof(OS));
	lmic_hal_init(lmicApi);
	radio_init();
	LMIC_init();
}

ostime_t os_getTime() {
	return lmic_hal_ticks();
}

static u1_t unlinkjob(osjob_t** pnext, osjob_t* job) {
	for (; *pnext; pnext = &((*pnext)->next)) {
		if (*pnext == job) { // unlink
			*pnext = job->next;
			return 1;
		}
	}
	return 0;
}

// clear scheduled job
void os_clearCallback(osjob_t* job) {
	lmic_hal_disableIRQs();
	unlinkjob(&OS.scheduledjobs, job) || unlinkjob(&OS.runnablejobs, job);
	lmic_hal_enableIRQs();
}

// schedule immediately runnable job
void os_setCallback(osjob_t* job, osjobcb_t cb) {
	osjob_t** pnext;
	lmic_hal_disableIRQs();
	// remove if job was already queued
	os_clearCallback(job);
	// fill-in job
	job->deadline = 0;
	job->func = cb;
	job->next = NULL;
	// add to end of run queue
	for (pnext = &OS.runnablejobs; *pnext; pnext = &((*pnext)->next))
		;
	*pnext = job;
	lmic_hal_enableIRQs();
}

// schedule timed job
void os_setTimedCallback(osjob_t* job, ostime_t time, osjobcb_t cb) {
	osjob_t** pnext;
	lmic_hal_disableIRQs();
	// remove if job was already queued
	os_clearCallback(job);
	// fill-in job
	job->deadline = time;
	job->func = cb;
	job->next = NULL;
	// insert into schedule
	for (pnext = &OS.scheduledjobs; *pnext; pnext = &((*pnext)->next)) {
		if ((*pnext)->deadline - time > 0) { // (cmp diff, not abs!)
			// enqueue before next element and stop
			job->next = *pnext;
			break;
		}
	}
	*pnext = job;
	lmic_hal_enableIRQs();
}

osjob_t* os_nextJob() {
	if (OS.runnablejobs) {
		return OS.runnablejobs;
	} else if (OS.scheduledjobs) { // check for expired timed jobs
		return OS.scheduledjobs;
	}
	return NULL;
}

// execute jobs from timer and from run queue
void os_runloop(bit_t loopForever) {
	while (1) {
		osjob_t* j = NULL;
		lmic_hal_disableIRQs();

		// check for runnable jobs
		if (OS.runnablejobs) {
			j = OS.runnablejobs;
			OS.runnablejobs = j->next;
		} else if (OS.scheduledjobs && lmic_hal_checkTimer(OS.scheduledjobs->deadline)) { // check for expired timed jobs
			j = OS.scheduledjobs;
			OS.scheduledjobs = j->next;
		} else { // nothing pending
			lmic_hal_sleep(); // wake by irq (timer already restarted)
		}
		lmic_hal_enableIRQs();

		if (j) { // run job callback
			j->func(j);
		}

		if (!loopForever) {
			break;
		}
	}
}

