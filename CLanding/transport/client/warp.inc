static bool warp_decision_from_rates(double remaining_ut,
                                     double response_horizon_seconds,
                                     int maximum_factor,
                                     const double *rates,
                                     size_t rate_count,
                                     KrpcCNanoWarpDecision *decision) {
    if (!decision || !rates || rate_count == 0 ||
        !isfinite(remaining_ut) || remaining_ut < 0.0 ||
        !isfinite(response_horizon_seconds) || response_horizon_seconds <= 0.0 ||
        maximum_factor < 0 || (size_t)maximum_factor >= rate_count)
        return false;

    *decision = (KrpcCNanoWarpDecision){
        .factor = 0,
        .predicted_rate = rates[0],
        .response_horizon_seconds = response_horizon_seconds,
        .stopping_margin_ut = remaining_ut
    };
    for (int factor = maximum_factor; factor >= 1; --factor) {
        double rate = rates[factor];
        if (!isfinite(rate) || rate <= 0.0) return false;
        double margin = remaining_ut - rate * response_horizon_seconds;
        if (margin >= 0.0) {
            decision->factor = factor;
            decision->predicted_rate = rate;
            decision->stopping_margin_ut = margin;
            break;
        }
    }
    return true;
}

bool krpc_cnano_client_warp_decision(double remaining_ut,
                                     double response_horizon_seconds,
                                     int maximum_factor,
                                     KrpcCNanoWarpDecision *decision) {
    return warp_decision_from_rates(
        remaining_ut, response_horizon_seconds, maximum_factor,
        k_stock_rails_rate_by_factor,
        sizeof(k_stock_rails_rate_by_factor) /
            sizeof(k_stock_rails_rate_by_factor[0]),
        decision);
}

static bool set_rails_warp_factor_observed(KrpcCNanoClient *client,
                                           int32_t requested,
                                           int32_t *observed,
                                           double *latency_seconds,
                                           char *error,
                                           size_t error_size) {
    if (!client || !observed || !latency_seconds) return false;
    if (!krpc_cnano_transport_begin_deadline(client->connection)) {
        set_error(error, error_size,
                  "Could not start the configured rails-warp response deadline");
        return false;
    }

    double started = monotonic_seconds();
    krpc_error_t set_result =
        krpc_SpaceCenter_set_RailsWarpFactor(client->connection, requested);
    if (rpc_failed(client, set_result, "SpaceCenter.RailsWarpFactor(set)",
                   error, error_size)) {
        krpc_cnano_transport_end_deadline(client->connection);
        return false;
    }

    krpc_error_t read_result =
        krpc_SpaceCenter_RailsWarpFactor(client->connection, observed);
    *latency_seconds = monotonic_seconds() - started;
    krpc_cnano_transport_end_deadline(client->connection);
    if (rpc_failed(client, read_result,
                   "SpaceCenter.RailsWarpFactor(readback)",
                   error, error_size))
        return false;

    size_t rate_count = sizeof(k_stock_rails_rate_by_factor) /
                        sizeof(k_stock_rails_rate_by_factor[0]);
    if (*observed < 0 || (size_t)*observed >= rate_count ||
        (requested == 0 && *observed != 0) ||
        (requested > 0 && *observed > requested)) {
        set_error(error, error_size,
                  "KSP returned invalid rails-warp factor %d for request %d",
                  (int)*observed, (int)requested);
        return false;
    }
    if (!isfinite(*latency_seconds) || *latency_seconds <= 0.0) {
        set_error(error, error_size,
                  "Could not measure rails-warp command/response latency");
        return false;
    }
    return true;
}

typedef struct {
    double ut;
    int32_t factor;
    double rate;
    double latency_seconds;
} CNanoWarpState;

static bool read_warp_state(KrpcCNanoClient *client,
                            CNanoWarpState *state,
                            char *error,
                            size_t error_size) {
    if (!client || !state) return false;
    if (!krpc_cnano_transport_begin_deadline(client->connection)) {
        set_error(error, error_size,
                  "Could not start the configured rails-warp state deadline");
        return false;
    }

    double started = monotonic_seconds();
    float rate = NAN;
    krpc_error_t code =
        krpc_SpaceCenter_UT(client->connection, &state->ut);
    if (rpc_failed(client, code, "SpaceCenter.UT(warp)",
                   error, error_size)) {
        krpc_cnano_transport_end_deadline(client->connection);
        return false;
    }
    code = krpc_SpaceCenter_RailsWarpFactor(
        client->connection, &state->factor);
    if (rpc_failed(client, code, "SpaceCenter.RailsWarpFactor(warp-state)",
                   error, error_size)) {
        krpc_cnano_transport_end_deadline(client->connection);
        return false;
    }
    code = krpc_SpaceCenter_WarpRate(client->connection, &rate);
    state->latency_seconds = monotonic_seconds() - started;
    krpc_cnano_transport_end_deadline(client->connection);
    if (rpc_failed(client, code, "SpaceCenter.WarpRate",
                   error, error_size))
        return false;

    size_t rate_count = sizeof(k_stock_rails_rate_by_factor) /
                        sizeof(k_stock_rails_rate_by_factor[0]);
    state->rate = (double)rate;
    if (!isfinite(state->ut) ||
        state->factor < 0 || (size_t)state->factor >= rate_count ||
        !isfinite(state->rate) || state->rate <= 0.0 ||
        !isfinite(state->latency_seconds) ||
        state->latency_seconds <= 0.0) {
        set_error(error, error_size,
                  "KSP returned invalid rails-warp state "
                  "(ut=%.9g factor=%d rate=%.9g latency=%.9g)",
                  state->ut, (int)state->factor, state->rate,
                  state->latency_seconds);
        return false;
    }
    return true;
}

static bool read_maximum_rails_factor(KrpcCNanoClient *client,
                                      int32_t *maximum_factor,
                                      char *error,
                                      size_t error_size) {
    if (!krpc_cnano_transport_begin_deadline(client->connection)) {
        set_error(error, error_size,
                  "Could not start the maximum-rails-factor response deadline");
        return false;
    }
    krpc_error_t code = krpc_SpaceCenter_MaximumRailsWarpFactor(
        client->connection, maximum_factor);
    krpc_cnano_transport_end_deadline(client->connection);
    if (rpc_failed(client, code, "SpaceCenter.MaximumRailsWarpFactor",
                   error, error_size))
        return false;

    size_t rate_count = sizeof(k_stock_rails_rate_by_factor) /
                        sizeof(k_stock_rails_rate_by_factor[0]);
    if (*maximum_factor < 0 || (size_t)*maximum_factor >= rate_count) {
        set_error(error, error_size,
                  "KSP returned unsupported maximum rails-warp factor %d",
                  (int)*maximum_factor);
        return false;
    }
    return true;
}

static double ut_resolution(double ut) {
    double adjacent = nextafter(ut, INFINITY);
    double resolution = adjacent - ut;
    return isfinite(resolution) && resolution > 0.0 ? resolution : DBL_EPSILON;
}

bool krpc_cnano_client_warp(KrpcCNanoClient *client, double ut,
                            char *error, size_t error_size) {
    if (!client || !client->connected || !isfinite(ut)) return false;

    /* SpaceCenter.WarpTo is intentionally not used on the serial protocol:
       it withholds its RPC response until the target UT, so a serial timeout
       leaves the server-side warp coroutine alive without a request channel
       available to stop it. This controller uses short, deadline-bounded RPCs
       and chooses a rail factor from a finite-response stopping margin. */
    int timeout_ms = krpc_cnano_transport_timeout_ms(client->connection);
    if (timeout_ms <= 0) {
        set_error(error, error_size,
                  "Rails warp requires a positive configured transport timeout");
        return false;
    }
    /* decision-literal: mathematical-numerical-requirement | Convert configured milliseconds to seconds for monotonic wall-time comparison. */
    double progress_timeout = (double)timeout_ms / 1000.0;

    int32_t observed_factor = -1;
    double stop_latency = 0.0;
    if (!set_rails_warp_factor_observed(
            client, 0, &observed_factor, &stop_latency,
            error, error_size))
        return false;
    double factor_command_latency = stop_latency;

    int32_t maximum_factor = 0;
    if (!read_maximum_rails_factor(
            client, &maximum_factor, error, error_size))
        return false;
    if (maximum_factor < 1) {
        set_error(error, error_size,
                  "KSP reports that on-rails time warp is unavailable");
        return false;
    }

    CNanoWarpState state = {0};
    if (!read_warp_state(client, &state, error, error_size))
        return false;
    if (ut <= state.ut) {
        reset_rate_filters(client);
        return true;
    }

    double rates[sizeof(k_stock_rails_rate_by_factor) /
                 sizeof(k_stock_rails_rate_by_factor[0])];
    memcpy(rates, k_stock_rails_rate_by_factor, sizeof(rates));
    if ((size_t)state.factor < sizeof(rates) / sizeof(rates[0]))
        rates[state.factor] = state.rate;

    double last_progress_ut = state.ut;
    double last_progress_wall = monotonic_seconds();

    for (;;) {
        double remaining = ut - state.ut;
        if (remaining <= 0.0) {
            if (state.factor != 0) {
                double latency = 0.0;
                if (!set_rails_warp_factor_observed(
                        client, 0, &observed_factor, &latency,
                        error, error_size))
                    return false;
            }
            reset_rate_filters(client);
            return true;
        }

        if ((size_t)state.factor < sizeof(rates) / sizeof(rates[0]))
            rates[state.factor] = state.rate;
        /* Finite-response margin includes one factor command/readback,
           one state observation, and the eventual factor-zero command. */
        double response_horizon =
            factor_command_latency + state.latency_seconds + stop_latency;
        KrpcCNanoWarpDecision decision = {0};
        if (!warp_decision_from_rates(
                remaining, response_horizon, maximum_factor,
                rates, sizeof(rates) / sizeof(rates[0]), &decision)) {
            set_error(error, error_size,
                      "Could not derive a rails-warp factor from "
                      "remaining UT %.9g and response horizon %.9g",
                      remaining, response_horizon);
            return false;
        }

        if (decision.factor == 0) {
            if (state.factor != 0) {
                double latency = 0.0;
                if (!set_rails_warp_factor_observed(
                        client, 0, &observed_factor, &latency,
                        error, error_size))
                    return false;
                if (latency > stop_latency) stop_latency = latency;
            }
            reset_rate_filters(client);
            return true;
        }

        if (state.factor != decision.factor) {
            double latency = 0.0;
            if (!set_rails_warp_factor_observed(
                    client, decision.factor, &observed_factor, &latency,
                    error, error_size))
                return false;
            if (latency > factor_command_latency)
                factor_command_latency = latency;
            if (observed_factor == 0) {
                set_error(error, error_size,
                          "KSP declined requested rails-warp factor %d",
                          decision.factor);
                return false;
            }
            if (observed_factor < decision.factor)
                maximum_factor = observed_factor;
        }

        CNanoWarpState next = {0};
        if (!read_warp_state(client, &next, error, error_size)) {
            (void)krpc_SpaceCenter_set_RailsWarpFactor(
                client->connection, 0);
            return false;
        }

        double wall = monotonic_seconds();
        double resolution = ut_resolution(last_progress_ut);
        if (next.ut > last_progress_ut + resolution) {
            last_progress_ut = next.ut;
            last_progress_wall = wall;
        } else if (wall - last_progress_wall >= progress_timeout) {
            (void)krpc_SpaceCenter_set_RailsWarpFactor(
                client->connection, 0);
            set_error(error, error_size,
                      "Rails warp made no measurable UT progress within "
                      "the configured %.3f s transport timeout",
                      progress_timeout);
            return false;
        }

        state = next;
    }
}
