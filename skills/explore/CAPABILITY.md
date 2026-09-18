---
description: Explore through reachable known-free approaches to live map frontiers.
---

# Explore

Start `robonix/skill/explore/explore` once and retain the returned `run_id`.
Poll `robonix/skill/explore/explore/status`; stop with
`robonix/skill/explore/explore/cancel`. The existing request and response
schemas are unchanged. Status detail includes completed navigation leg count.

The skill selects footprint-clear approaches connected to the robot through
known-free occupancy cells. It never navigates to a cluster mean inside unknown
space or obstacles. Failed goal neighbourhoods are temporarily excluded; goals
are recomputed from fresh map observations. No reachable approach means waiting
for updates, not successful exploration.

Mapping supplies occupancy, Navigation executes motion, and Scene interprets
the front RGB-D observations acquired along the route. All business calls use
the existing Atlas-resolved map and navigation capabilities. No scene XML,
fixed room coordinates or simulator object inventory enters goal selection.

Use a bounded timeout and max_speed_m_s. The skill applies that cap through
Navigation's existing speed-limit capabilities without raising a lower cap.
It conditionally restores the previous limit after confirmed navigation stop.
Complete or cancel Explore before starting another motion task. Cancellation
is asynchronous: poll the same run until terminal. The worker waits for the
exact navigation run to stop before completing cancellation or another leg.
A failed cancellation is reported as task failure, not successful stopping.
