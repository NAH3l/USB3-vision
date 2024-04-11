#ifndef _U3V_3VENT_H_
#define _U3V_3VENT_H_

#include "u3v.h"
#define U3V_EVENT_PREFIX 0x45563355

struct u3v_event {
	struct u3v_device *u3v_dev;
	//struct mutex event_lock;
	bool started;
	bool wait_in_progress;
	//struct list_head event_queue;
	int queue_depth;
	//struct completion event_complete;
	_Uint64t overwritten_event_count;
	int order; /* number of pages needed for buffers */
	//struct usb_anchor event_anchor;
};

#endif /*  _U3V_3VENT_H_ */
