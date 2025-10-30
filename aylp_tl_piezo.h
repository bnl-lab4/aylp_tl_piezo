#ifndef AYLP_TL_PIEZO_H_
#define AYLP_TL_PIEZO_H_

#include "anyloop.h"
#include "libserialport.h"

#define LEN_CMD_BUF 64
#define V_MAX 150
#define CHECK_SP(call, msg) do { if (call != SP_OK) { log_error("%s", msg); sp_close(data->port); sp_free_port(data->port); data->port = NULL; return 1; } } while(0)

enum { AXIS_X=0, AXIS_Y=1, AXIS_Z=2, AXIS_COUNT=3 };

struct aylp_tl_piezo_data {
	struct sp_port *port;
	int mask[AXIS_COUNT];
	int map[AXIS_COUNT];
};

int aylp_tl_piezo_init(struct aylp_device *self);

int aylp_tl_piezo_process(struct aylp_device *self, struct aylp_state *state);

int aylp_tl_piezo_close(struct aylp_device *self);

#endif
