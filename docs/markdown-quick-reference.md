# Markdown Quick Reference

This file is a fast index of the repository's main markdown documents.

It focuses on project-authored docs and workflow docs first. Vendor and third-party library docs under `lib/` are listed at the end for reference.

## Core Project Docs

- [README.md](../README.md)
  - top-level project overview and build environment notes

- [AGENTS.md](../AGENTS.md)
  - local agent instructions for working in this repository

- [CLAUDE.md](../CLAUDE.md)
  - project overview, build/test expectations, and repository conventions

- [GEMINI.md](../GEMINI.md)
  - additional local agent/workflow guidance

## Docs Folder

- [docs/README.md](./README.md)
  - general docs landing page

- [docs/codebase-structure-review.md](./codebase-structure-review.md)
  - architecture and refactor review of the current codebase

- [docs/antenna-delay-self-calibration.md](./antenna-delay-self-calibration.md)
  - notes on antenna delay self-calibration

- [docs/drone-position-timeout-investigation-report.md](./drone-position-timeout-investigation-report.md)
  - investigation into drone position timeout behavior

- [docs/esp32s3-dw1000-update-rate-instability-report.md](./esp32s3-dw1000-update-rate-instability-report.md)
  - investigation into ESP32S3 and DW1000 update-rate instability

- [docs/uwb-update-rate-firmware-investigation.md](./uwb-update-rate-firmware-investigation.md)
  - firmware-focused investigation of UWB update rate issues

- [docs/tdoa-distance-constraint-correction/README.md](./tdoa-distance-constraint-correction/README.md)
  - notes and rationale for TDoA distance-constraint correction work

- [docs/backlog/TODO.md](./backlog/TODO.md)
  - backlog and open TODO items

## Scripts and Tooling Docs

- [scripts/README.md](../scripts/README.md)
  - overview of project scripts

- [scripts/README_DISCOVERY.md](../scripts/README_DISCOVERY.md)
  - documentation for discovery-related scripts/workflows

- [scripts/sim_vehicle/README.md](../scripts/sim_vehicle/README.md)
  - simulation vehicle helper notes

- [tools/rtls-link-cli/README.md](../tools/rtls-link-cli/README.md)
  - CLI usage, including discovery and OTA flows

- [tools/rtls-link-manager/README.md](../tools/rtls-link-manager/README.md)
  - desktop manager overview and development commands

- [tools/rtls-link-manager/CLAUDE.md](../tools/rtls-link-manager/CLAUDE.md)
  - local instructions specific to the desktop manager submodule

## Library and Third-Party Reference Docs

- [lib/DW1000_mf/README.md](../lib/DW1000_mf/README.md)
- [lib/DW1000_mf/LICENSE.md](../lib/DW1000_mf/LICENSE.md)
- [lib/Eigen/README.md](../lib/Eigen/README.md)
- [lib/Trilat/README.md](../lib/Trilat/README.md)
- [lib/libdw1000/README.md](../lib/libdw1000/README.md)
- [lib/libdw1000/LICENSE.md](../lib/libdw1000/LICENSE.md)
- [lib/libdw1000/vendor/cmock/README.md](../lib/libdw1000/vendor/cmock/README.md)
- [lib/libdw1000/vendor/cmock/docs/CMock_Summary.md](../lib/libdw1000/vendor/cmock/docs/CMock_Summary.md)
- [lib/libdw1000/vendor/cmock/vendor/c_exception/README.md](../lib/libdw1000/vendor/cmock/vendor/c_exception/README.md)
- [lib/libdw1000/vendor/cmock/vendor/unity/README.md](../lib/libdw1000/vendor/cmock/vendor/unity/README.md)
- [lib/libdw1000/vendor/unity/README.md](../lib/libdw1000/vendor/unity/README.md)

## Notes

- The `docs/` directory already contains user-created untracked files in this worktree. This index was added without modifying those files.
- If you want, this file can be narrowed later to only "project docs that matter day-to-day" and exclude vendor docs entirely.
