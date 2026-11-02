# Supported STS CLI integration — 2026-11-02

The normal STS mission profiles now select the Swift operator backend and
Configuration/sts-taem.json. The native counterpart preserves the successful
20261102T024801Z checkpoint flight configuration. Operator mapping preserves
entry and landing settings, including the separate 1.5 s approach interval.

The backend uses the shipped aerodynamic polar by default, matching direct
flight; historical archives require explicit configuration or environment.
TAEM takeover no longer requires admission by the obsolete terminal planner;
its atmospheric/airborne checks, IO arming, and Final capture check remain.
RPC timeout defaults to ten seconds. The wrapper resolves its default profile
outside the repository, and ready output exposes mapped mission configuration.

Validation: four focused OperatorConfigurationTests passed, release build
passed, normal CLI dry-run and configuration validation passed. Offline backend
startup from /tmp, configuration update and shutdown passed without connect or
engage. See adjacent artifacts. No KSP command was sent and the active archived
flight executable was not changed. Full-chain orbital validation remains open;
this promotion does not imply every mission profile has flown successfully.
