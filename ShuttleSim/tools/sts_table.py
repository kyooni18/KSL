#!/usr/bin/env python3
"""Compact per-case table for one or more campaign result directories."""
import glob, json, sys
for camp in sys.argv[1:]:
    print("==", camp)
    for f in sorted(glob.glob(camp.rstrip('/') + '/results/*.json')):
        r = json.load(open(f)); a = r.get('analysis') or {}
        cl = a.get('closestApproachToRunway') or {}; ei = a.get('entryInterface') or {}; bp = a.get('burnVsPlan') or {}
        fin = a.get('final') or {}
        print(r['id'][:20], r['outcome']['class'][:22], 'dv %4.1f/%4.1f' % (bp.get('planned_dv_mps') or 0, bp.get('achieved_engine_dv_mps') or 0),
              'nrm %3.0f' % (bp.get('planned_normal_angle_deg') or 0), 'Pe %4.1f/%4.1f' % ((bp.get('planned_pe_alt_m') or 0) / 1000, (bp.get('achieved_pe_alt_m') or 0) / 1000),
              'EIv %4.0f' % ei.get('surface_mps', 0), 'closest %4.0fkm v%5.0f h%3.0fkm' % (cl.get('range_m', 0) / 1000, cl.get('surface_mps', 0), cl.get('alt_m', 0) / 1000),
              'taem' if a.get('mm305Acquired') else '', 'end along %5.0f cross %5.0f' % (fin.get('along_m', 0) / 1000, fin.get('cross_m', 0) / 1000))
