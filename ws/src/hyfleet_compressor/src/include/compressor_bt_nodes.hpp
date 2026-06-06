#pragma once

// ==============================================================================
// Stage 3 — Coordinator BT nodes
//
// RosActionNode wrappers for calling booster action servers:
//   BoostLow   — RosActionNode → /low_booster/control_booster
//   BoostHigh  — RosActionNode → /high_booster/control_booster
//
// Routing (LOW / HIGH / SYNC) is handled in the XML trees via Parallel and
// subtree patterns — not in C++ switch logic.
//
// Nothing is registered here yet. Add nodes here as Stage 3 progresses.
// ==============================================================================
