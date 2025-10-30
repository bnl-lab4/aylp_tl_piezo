#include <string.h>
#include "anyloop.h"
#include "xalloc.h"
#include "logging.h"
#include "aylp_tl_piezo.h"
#include "libserialport.h"


int aylp_tl_piezo_init(struct aylp_device *self) {
	// Allocate the device data struct
	self->device_data = xcalloc(1, sizeof(struct aylp_tl_piezo_data));
	struct aylp_tl_piezo_data *data = self->device_data;

	// These configuration options are required
	bool is_dev = false, is_map = false, is_mask = false;
	const char *dev;

	// Parse the JSON config
	json_object_object_foreach(self->params, k, v) {
		if (k[0] == '_') {
			// keys starting with underscores are comments

		} else if (!strcmp(k, "dev")) {
			// The config option is the device name (e.g., "/dev/ttyACM0")

			if(is_dev) {
				log_warn("The \"dev\" configuration option appears more than once.");
				continue;
			}

			is_dev = true;
			
			if (!json_object_is_type(v, json_type_string)) {
				log_error("The \"dev\" JSON object must be a string.");
				return 1;
			}

			// Get the device name
			dev = json_object_get_string(v);

		} else if (!strcmp(k, "map")) {
			// The config option maps inputs to outputs with repetition

			if(is_map) {
				log_warn("The \"map\" configuration option appears more than once.");
				continue;
			}

			is_map = true;

			if (!json_object_is_type(v, json_type_array)) {
				log_error("The \"map\" object must be an array.");
				return 1;
			}

			if (json_object_array_length(v) != AXIS_COUNT) {
				log_error("The \"map\" array must be %d elements.", AXIS_COUNT);
				return 1;
			}

			// Copy the array to the device data struct
			for (size_t i = 0; i < AXIS_COUNT; i++) {
				struct json_object *element = json_object_array_get_idx(v, i);
				
				if (!element) {
					log_error("An error occurred parsing the \"map\" array.");
					return 1;
				}

				if (!json_object_is_type(element, json_type_int)) {
					log_error("Each element in the \"map\" array must be an integer.");
					return 1;
				}

				data->map[i] = json_object_get_int(element);

				if(data->map[i] < 0 || data->map[i] >= AXIS_COUNT) {
					log_error("Each integer in the \"map\" array must be in [0, %d].", AXIS_COUNT - 1);
					return 1;
				}
			}

			log_trace("map = [%d, %d, %d]", data->map[0], data->map[1], data->map[2]);
			
		} else if (!strcmp(k, "mask")) {
			// The config option toggle outputs

			if(is_mask) {
				log_warn("The \"mask\" configuration option appears more than once.");
				continue;
			}

			is_mask = true;

			if (!json_object_is_type(v, json_type_array)) {
				log_error("The \"mask\" object must be an array.");
				return 1;
			}

			if (json_object_array_length(v) != AXIS_COUNT) {
				log_error("The \"mask\" array must be %d elements.", AXIS_COUNT);
				return 1;
			}

			// Copy the array to the device data struct
			for (size_t i = 0; i < AXIS_COUNT; i++) {
				struct json_object *element = json_object_array_get_idx(v, i);
				
				if (!element) {
					log_error("An error occurred parsing the \"mask\" array.");
					return 1;
				}

				if (!json_object_is_type(element, json_type_boolean) && !json_object_is_type(element, json_type_int)) {
					log_error("Each element in the \"mask\" array must be a boolean or integer.");
					return 1;
				}

				data->mask[i] = json_object_get_int(element);
			}

			log_trace("mask = [%d, %d, %d]", data->mask[0], data->mask[1], data->mask[2]);

		} else {
			log_warn("An unknown configuration option was ignored: \"%s\"", k);
		}
	}

	// Check the required configuration flags
	if (!is_dev || !is_map || !is_mask) {
		log_error("A required configuration option (\"dev\", \"map\", or \"mask\") is missing.");
		return 1;
	}

	// Get the piezo controller port
	if (sp_get_port_by_name(dev, &data->port) != SP_OK) {
		log_error("The piezo controller could not be found: %s", dev);
		return 1;
	}

	// Open the piezo controller
	if (sp_open(data->port, SP_MODE_READ_WRITE) != SP_OK) {
		// Release a partial resource
		sp_free_port(data->port);
		data->port = NULL;

		log_error("The piezo controller could not be opened: %s", dev);
		return 1;
	}

	log_trace("The piezo controller was opened successfully: %s", dev);

	// Set the serial settings
    CHECK_SP(sp_set_baudrate(data->port, 115200), "The baud rate could not be set on the serial port.");
	CHECK_SP(sp_set_bits(data->port, 8), "The number of data bits could not be set on the serial port.");
	CHECK_SP(sp_set_parity(data->port, SP_PARITY_NONE), "The parity could not be set on the serial port.");
	CHECK_SP(sp_set_stopbits(data->port, 1), "The number of stop bits could not be set on the serial port.");
	CHECK_SP(sp_set_flowcontrol(data->port, SP_FLOWCONTROL_NONE), "The flow control could not be set on the serial port.");

	// Flush all buffers
	CHECK_SP(sp_flush(data->port, SP_BUF_BOTH), "The buffers could not be flushed.");

	// Attach methods
	self->proc = &aylp_tl_piezo_process;
	self->fini = &aylp_tl_piezo_close;

	// Set types and units
	self->type_in = AYLP_T_VECTOR;
	self->units_in = AYLP_U_V;
	self->type_out = AYLP_T_UNCHANGED;
	self->units_out = AYLP_U_UNCHANGED;

	return 0;
}


int aylp_tl_piezo_process(struct aylp_device *self, struct aylp_state *state) {
	// Get the device data
	struct aylp_tl_piezo_data *data = self->device_data;

	// Allocate the message buffer
	char buf[LEN_CMD_BUF];

	// Define the default voltage for each axis.
	double v_x = 0, v_y = 0, v_z = 0;

	// Write the X voltage to the controller
	if (data->mask[AXIS_X]) {
		// Map an input voltage to the X axis
		switch(data->map[AXIS_X]) { 
			case(AXIS_X): v_x = gsl_vector_get(state->vector, AXIS_X); break; 
			case(AXIS_Y): v_x = gsl_vector_get(state->vector, AXIS_Y); break;
			case(AXIS_Z): v_x = gsl_vector_get(state->vector, AXIS_Z); break; 
			default: log_error("An invalid axis index was given: %d", data->map[AXIS_X]); 
		}

		if (0 <= v_x && v_x <= V_MAX) {
			snprintf(buf, LEN_CMD_BUF, "xvoltage=%.2f\r\n", v_x);
			if (sp_blocking_write(data->port, buf, strlen(buf), 0) < 0)
				log_error("An error occurred while writing the X voltage to the piezo controller.");
		
		} else {
			log_error("An invalid voltage was provided for the X axis.");
		}
	}

	// Write the Y voltage to the controller
	if (data->mask[AXIS_Y]) {
		// Map the input voltage to the Y axis
		switch(data->map[AXIS_Y]) { 
			case(AXIS_X): v_y = gsl_vector_get(state->vector, AXIS_X); break; 
			case(AXIS_Y): v_y = gsl_vector_get(state->vector, AXIS_Y); break;
			case(AXIS_Z): v_y = gsl_vector_get(state->vector, AXIS_Z); break; 
			default: log_error("An invalid axis index was given: %d", data->map[AXIS_Y]); 
		}

		if (0 <= v_y && v_y <= V_MAX) {
			snprintf(buf, LEN_CMD_BUF, "yvoltage=%.2f\r\n", v_y);
			if (sp_blocking_write(data->port, buf, strlen(buf), 0) < 0)
				log_error("An error occurred while writing the Y voltage to the piezo controller.");
		
		} else {
			log_error("An invalid voltage was provided for the Y axis.");
		}
	}

	// Write the Z voltage to the controller
	if (data->mask[AXIS_Z]) {
		// Map the input voltage to the Z axis
		switch(data->map[AXIS_Z]) { 
			case(AXIS_X): v_z = gsl_vector_get(state->vector, AXIS_X); break; 
			case(AXIS_Y): v_z = gsl_vector_get(state->vector, AXIS_Y); break;
			case(AXIS_Z): v_z = gsl_vector_get(state->vector, AXIS_Z); break; 
			default: log_error("An invalid axis index was given: %d", data->map[AXIS_Z]); 
		}

		if (0 <= v_z && v_z <= V_MAX) {
			snprintf(buf, LEN_CMD_BUF, "zvoltage=%.2f\r\n", v_z);
			if (sp_blocking_write(data->port, buf, strlen(buf), 0) < 0)
				log_error("An error occurred while writing the Z voltage to the piezo controller.");
		
		} else {
			log_error("An invalid voltage was provided for the Z axis.");
		}
	}

	// Log the voltages
	snprintf(buf, LEN_CMD_BUF, "xvoltage=%.2f yvoltage=%.2f zvoltage=%.2f", v_x, v_y, v_z);
	log_info(buf);

	// We should wait for commands to be transmitted to the controller
	if (sp_drain(data->port) != SP_OK)
		log_error("An error occurred while draining the output buffer.");

	// We do not want the response from the controller
	if (sp_flush(data->port, SP_BUF_INPUT))
		log_error("An error occurred while flushing the input buffer.");

	return 0;
}


int aylp_tl_piezo_close(struct aylp_device *self) {
	// Get the device data
	struct aylp_tl_piezo_data *data = self->device_data;

	// Close handles to the piezo controller
	if (data->port) {
		sp_close(data->port);
		sp_free_port(data->port);
	}

	// Free the device data struct
	xfree(data);

	return 0;
}

