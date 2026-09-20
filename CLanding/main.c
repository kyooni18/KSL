#include "landing.h"

#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    pthread_mutex_t mutex;
} Output;

static void emit_writer(Output *out, JsonWriter *w) {
    if (w->failed || !w->data) return;
    pthread_mutex_lock(&out->mutex);
    fwrite(w->data, 1, w->length, stdout);
    fputc('\n', stdout);
    fflush(stdout);
    pthread_mutex_unlock(&out->mutex);
}

static void snapshot_callback(const LandingSnapshot *snapshot, void *context) {
    Output *out = context;
    JsonWriter w;
    jw_init(&w);
    jw_raw(&w, "{\"type\":\"snapshot\",\"snapshot\":");
    snapshot_json(&w, snapshot);
    jw_char(&w, '}');
    emit_writer(out, &w);
    jw_free(&w);
}

static void emit_ready(Output *out, const LandingConfiguration *configuration) {
    LandingSnapshot snapshot;
    landing_snapshot_init(&snapshot, &configuration->vehicle);
    JsonWriter w;
    jw_init(&w);
    jw_raw(&w, "{\"type\":\"ready\",\"protocolVersion\":1,\"configuration\":");
    landing_configuration_json(&w, configuration);
    jw_raw(&w, ",\"snapshot\":");
    snapshot_json(&w, &snapshot);
    jw_char(&w, '}');
    emit_writer(out, &w);
    jw_free(&w);
    trajectory_clear(&snapshot.actual_trajectory);
    trajectory_clear(&snapshot.reference_trajectory);
}

static void emit_response(Output *out, const char *id, bool configuration_result, const LandingConfiguration *configuration) {
    JsonWriter w;
    jw_init(&w);
    jw_raw(&w, "{\"type\":\"response\",\"id\":");
    jw_string(&w, id ? id : "");
    jw_raw(&w, ",\"ok\":true,\"result\":");
    if (configuration_result) {
        jw_raw(&w, "{\"configuration\":");
        landing_configuration_json(&w, configuration);
        jw_char(&w, '}');
    } else {
        jw_raw(&w, "{}");
    }
    jw_char(&w, '}');
    emit_writer(out, &w);
    jw_free(&w);
}

static void emit_error(Output *out, const char *id, const char *message) {
    JsonWriter w;
    jw_init(&w);
    jw_raw(&w, "{\"type\":\"response\",\"id\":");
    if (id) jw_string(&w, id); else jw_null(&w);
    jw_raw(&w, ",\"ok\":false,\"error\":");
    jw_string(&w, message ? message : "Unknown backend error");
    jw_char(&w, '}');
    emit_writer(out, &w);
    jw_free(&w);
}

int main(void) {
    Output output;
    pthread_mutex_init(&output.mutex, NULL);
    LandingConfiguration configuration = landing_configuration_default();
    landing_configuration_normalize(&configuration);
    LandingController *controller = landing_controller_create(&configuration, snapshot_callback, &output);
    if (!controller) {
        fprintf(stderr, "Failed to allocate C landing controller.\n");
        return EXIT_FAILURE;
    }
    emit_ready(&output, &configuration);

    char *line = NULL;
    size_t line_capacity = 0;
    while (getline(&line, &line_capacity, stdin) >= 0) {
        JsonToken tokens[8192];
        JsonDoc doc;
        if (json_parse(line, tokens, 8192, &doc) < 1 || doc.tokens[0].type != JSON_OBJECT) {
            emit_error(&output, NULL, "Invalid backend command JSON");
            continue;
        }
        char id[128] = "";
        char method[96] = "";
        int id_index = json_object_get(&doc, 0, "id");
        int method_index = json_object_get(&doc, 0, "method");
        if (id_index >= 0) json_string(&doc, id_index, id, sizeof(id));
        if (method_index < 0 || !json_string(&doc, method_index, method, sizeof(method))) {
            emit_error(&output, id[0] ? id : NULL, "Backend command requires method");
            continue;
        }

        int config_index = json_object_get(&doc, 0, "configuration");
        if (config_index >= 0) {
            LandingConfiguration requested = configuration;
            if (!landing_configuration_from_json(&requested, &doc, config_index)) {
                emit_error(&output, id[0] ? id : NULL, "Invalid landing configuration");
                continue;
            }
            configuration = requested;
            landing_controller_update_configuration(controller, &configuration);
        }

        if (!strcmp(method, "getConfiguration")) {
            configuration = landing_controller_configuration(controller);
            emit_response(&output, id, true, &configuration);
        } else if (!strcmp(method, "updateConfiguration")) {
            if (config_index < 0) {
                emit_error(&output, id, "updateConfiguration requires configuration");
                continue;
            }
            configuration = landing_controller_configuration(controller);
            emit_response(&output, id, true, &configuration);
        } else if (!strcmp(method, "connect")) {
            landing_controller_connect(controller);
            /* landing_controller_connect() starts the live control thread.
               Do not immediately re-enter the controller mutex just to copy
               configuration for the protocol response: the first telemetry
               tick can legitimately hold that mutex while kRPC rebuilds
               streams after a quickload, which used to delay this response
               long enough for headless clients to report a false timeout.
               The local configuration is already the normalized request used
               to create the connection. */
            emit_response(&output, id, true, &configuration);
        } else if (!strcmp(method, "disconnect")) {
            landing_controller_disconnect(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "createPlan")) {
            landing_controller_create_plan(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "engage")) {
            landing_controller_engage(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "engageReentry")) {
            landing_controller_engage_reentry(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "engageHACTest")) {
            landing_controller_engage_hac_test(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "startCalibration")) {
            landing_controller_start_calibration(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "stopCalibration")) {
            bool apply = json_boolean(&doc, json_object_get(&doc, 0, "applyResults"), false);
            landing_controller_stop_calibration(controller, apply);
            if (apply) configuration = landing_controller_configuration(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "resetCalibration")) {
            landing_controller_reset_calibration(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "setPaused")) {
            bool value = json_boolean(&doc, json_object_get(&doc, 0, "value"), false);
            landing_controller_set_paused(controller, value);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "abort")) {
            landing_controller_abort(controller);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "setGear")) {
            bool value = json_boolean(&doc, json_object_get(&doc, 0, "value"), false);
            landing_controller_set_gear(controller, value);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "setBrakes")) {
            bool value = json_boolean(&doc, json_object_get(&doc, 0, "value"), false);
            landing_controller_set_brakes(controller, value);
            emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "saveCheckpoint")) {
            char name[128] = {0};
            json_string(&doc, json_object_get(&doc, 0, "name"), name, sizeof(name));
            char error[512] = {0};
            if (!landing_controller_save_checkpoint(controller, name, error, sizeof(error)))
                emit_error(&output, id[0] ? id : NULL, error[0] ? error : "Could not save KSP checkpoint");
            else emit_response(&output, id, false, &configuration);
        } else if (!strcmp(method, "shutdown")) {
            landing_controller_shutdown(controller);
            emit_response(&output, id, false, &configuration);
            break;
        } else {
            char message[192];
            snprintf(message, sizeof(message), "Unknown backend method: %s", method);
            emit_error(&output, id[0] ? id : NULL, message);
        }
    }

    free(line);
    landing_controller_destroy(controller);
    pthread_mutex_destroy(&output.mutex);
    return EXIT_SUCCESS;
}
