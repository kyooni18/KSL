static bool build_source_candidates(
        const HistoryRow *rows,
        size_t count,
        SourceCandidate **out,
        size_t *out_count,
        char *error,
        size_t error_size) {
    *out = NULL;
    *out_count = 0;
    if (count == 0) return true;

    bool *used = calloc(count, sizeof(*used));
    SourceCandidate *candidates = calloc(count, sizeof(*candidates));
    size_t *indices = malloc(count * sizeof(*indices));
    if (!used || !candidates || !indices) {
        free(used);
        free(candidates);
        free(indices);
        set_error(error, error_size,
            "Out of memory reducing physics history");
        return false;
    }

    for (size_t i = 0; i < count; ++i) {
        if (used[i]) continue;
        CellKey key = row_cell(&rows[i]);
        size_t n = 0;
        for (size_t j = i; j < count; ++j) {
            if (!used[j] &&
                cell_equal(key, row_cell(&rows[j])) &&
                strcmp(rows[i].source, rows[j].source) == 0) {
                used[j] = true;
                indices[n++] = j;
            }
        }

        size_t chosen = choose_medoid(rows, indices, n, false);
        SourceCandidate *candidate = &candidates[(*out_count)++];
        candidate->key = key;
        candidate->row = rows[chosen];
        copy_string(candidate->source, sizeof(candidate->source),
            rows[i].source);
    }

    free(used);
    free(indices);
    *out = candidates;
    return true;
}

static bool build_selected_cells(
        const SourceCandidate *candidates,
        size_t count,
        SelectedCell **out,
        size_t *out_count,
        char *error,
        size_t error_size) {
    *out = NULL;
    *out_count = 0;
    if (count == 0) return true;

    bool *used = calloc(count, sizeof(*used));
    SelectedCell *selected = calloc(count, sizeof(*selected));
    size_t *group = malloc(count * sizeof(*group));
    HistoryRow *group_rows = malloc(count * sizeof(*group_rows));
    size_t *local = malloc(count * sizeof(*local));
    if (!used || !selected || !group || !group_rows || !local) {
        free(used);
        free(selected);
        free(group);
        free(group_rows);
        free(local);
        set_error(error, error_size,
            "Out of memory aggregating independent flights");
        return false;
    }

    for (size_t i = 0; i < count; ++i) {
        if (used[i]) continue;
        size_t n = 0;
        for (size_t j = i; j < count; ++j) {
            if (!used[j] &&
                cell_equal(candidates[i].key, candidates[j].key)) {
                used[j] = true;
                group[n++] = j;
            }
        }

        for (size_t j = 0; j < n; ++j) {
            group_rows[j] = candidates[group[j]].row;
            local[j] = j;
        }
        size_t chosen_local =
            choose_medoid(group_rows, local, n, true);
        SelectedCell *cell = &selected[(*out_count)++];
        cell->key = candidates[i].key;
        cell->row = group_rows[chosen_local];
        cell->independent = (unsigned)n;
    }

    free(used);
    free(group);
    free(group_rows);
    free(local);
    *out = selected;
    return true;
}

/*
 * These boundaries are the midpoints of the predictor's Mach anchors
 * {0.35, 1.05, 2.6, 6.0}; they classify stored coverage into the same
 * interpolation regions rather than encoding separate flight behavior.
 */
static int certified_regime_for_store(double mach) {
    static const double anchors[] = {0.35, 1.05, 2.6, 6.0};
    for (int i = 0; i < 3; ++i)
        if (mach < 0.5 * (anchors[i] + anchors[i + 1])) return i;
    return 3;
}

static double cell_distance(CellKey a, CellKey b) {
    if (a.gear != b.gear || a.brakes != b.brakes ||
        a.airbrakes != b.airbrakes)
        return INFINITY;

    /*
     * Cell coordinates are already dimensionless model-space indices.
     * AoA/beta cells are 4 deg wide; dividing their index separation by
     * 2.5 expresses the existing 10 deg aerodynamic lookup neighborhood.
     */
    double dq = (double)(a.q - b.q);
    double dm = (double)(a.mach - b.mach);
    double da = (double)(a.aoa - b.aoa) / 2.5;
    double db = (double)(a.beta - b.beta) / 2.5;
    return sqrt(dq * dq + dm * dm + da * da + db * db);
}

static bool better_support(
        const SelectedCell *candidate,
        const SelectedCell *current) {
    if (candidate->independent != current->independent)
        return candidate->independent > current->independent;
    return candidate->row.id > current->row.id;
}

static size_t reduce_selected_cells(
        const SelectedCell *selected,
        size_t count,
        size_t maximum,
        size_t *indices) {
    if (count <= maximum) {
        for (size_t i = 0; i < count; ++i) indices[i] = i;
        return count;
    }

    bool *retained = calloc(count, sizeof(*retained));
    if (!retained) return 0;
    size_t n = 0;

    /*
     * First preserve every represented device/Mach-region topology. Within a
     * topology, prefer the state supported by more independent flights.
     */
    for (size_t i = 0; i < count && n < maximum; ++i) {
        int regime = certified_regime_for_store(selected[i].row.mach);
        bool group_seen = false;
        for (size_t j = 0; j < n; ++j) {
            const SelectedCell *existing = &selected[indices[j]];
            if (existing->key.gear == selected[i].key.gear &&
                existing->key.brakes == selected[i].key.brakes &&
                existing->key.airbrakes == selected[i].key.airbrakes &&
                certified_regime_for_store(existing->row.mach) == regime) {
                group_seen = true;
                break;
            }
        }
        if (group_seen) continue;

        size_t best = i;
        for (size_t j = i + 1; j < count; ++j) {
            if (selected[j].key.gear != selected[i].key.gear ||
                selected[j].key.brakes != selected[i].key.brakes ||
                selected[j].key.airbrakes != selected[i].key.airbrakes ||
                certified_regime_for_store(selected[j].row.mach) != regime)
                continue;
            if (better_support(&selected[j], &selected[best]))
                best = j;
        }

        if (!retained[best]) {
            retained[best] = true;
            indices[n++] = best;
        }
    }

    /*
     * Fill the remaining budget by farthest-point sampling in the model
     * coordinates. This maximizes retained coverage directly; support count
     * and recency only break numerical ties.
     */
    while (n < maximum) {
        size_t best = SIZE_MAX;
        double best_distance = -INFINITY;

        for (size_t i = 0; i < count; ++i) {
            if (retained[i]) continue;
            double nearest = INFINITY;
            for (size_t j = 0; j < n; ++j)
                nearest = fmin(nearest,
                    cell_distance(selected[i].key,
                        selected[indices[j]].key));

            bool tie = best != SIZE_MAX &&
                numerical_tie(nearest, best_distance);
            if (best == SIZE_MAX ||
                nearest > best_distance ||
                (tie && better_support(&selected[i], &selected[best]))) {
                best = i;
                best_distance = nearest;
            }
        }

        if (best == SIZE_MAX) break;
        retained[best] = true;
        indices[n++] = best;
    }

    free(retained);
    return n;
}

