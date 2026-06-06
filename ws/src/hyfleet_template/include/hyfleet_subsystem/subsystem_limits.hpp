#pragma once

// TODO: rename namespace to hyfleet_<name>
namespace hyfleet_subsystem
{
    // TODO: add hardware constexpr limits here — values that are universal and
    // immutable for this hardware type (e.g. speed ceilings, CPM ranges).
    //
    // Rule: differs per instance → param. Universal + immutable → constexpr here.
    // See architecture.md §5 (Parameters & limits) for the classification rule.
    //
    // Example:
    //   constexpr double speed_min = 0.0;
    //   constexpr double speed_max = 1600.0;

} // namespace hyfleet_subsystem
