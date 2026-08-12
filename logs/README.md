# Goal selection records

`goal_selections.csv` is appended whenever `goal_point` is received. Merely
starting FAR without selecting a goal does not add a row. Each row contains:

- a globally increasing sequence and a per-process session ID/sequence;
- wall-clock, ROS, odometry-message and semantic-map timestamps;
- robot position/heading, source goal and transformed world-frame goal;
- odometry/map wall-clock ages at the instant of selection;
- graph node and current static/dynamic point counts;
- graph/planner state and the previously published waypoint.

A goal retained while the graph is initializing is recorded once, not again
when it is later applied. Existing legacy rows are kept and their new fields
are left empty when FAR automatically upgrades the CSV header. The global
sequence resumes from the largest sequence already present after a restart.

Set private parameter `goal_record_file` to redirect the log. An empty value
uses this directory.

After starting the simulation, mapping and FAR nodes, replay the latest row:

```bash
rosrun far_planner replay_goal_selection.py
```

Use `--sequence N` for a specific row. The script compares current odometry
with the recorded start and refuses a mismatched replay unless `--force` is
given.
