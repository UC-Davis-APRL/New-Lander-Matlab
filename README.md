# 2026 APRL fullscale control system
program for our rockets control system architecture.

# Features
trajectory planning via ILQR
Simulation and graphical data representation
Cascade control system with sensor fusion

# Usage
first create a reference trajectory by editing the nodes and running Reference Trajectory.m
node example:
```
segments{end+1} = seg_move(0.0,  10.0, [0;0;pd_from_alt(0)],   [5;0;pd_from_alt(50)]);
segments{end+1} = seg_move(10.0,20.0,  [5;0;pd_from_alt(50)], [5;0;pd_from_alt(0)]);
segments{end+1} = seg_move(20.0,25.0,  [5;0;pd_from_alt(00)], [0;0;pd_from_alt(0)]);
segments{end+1} = seg_hold(25.0,35.0,  [0;0;pd_from_alt(0)])
```

then run TrajectoryPlanningilqr.m to create a more accurate and followable trajectory with all the feedforwards required by the control system.

then run simulate.m and you should be good to go.