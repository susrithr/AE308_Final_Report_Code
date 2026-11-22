GROUP 18 — CONTROL SYSTEMS PROJECT
===================================

Archive contents
----------------
matlab/run_all.m
    Single top-level MATLAB script for generating the project figures and
    numerical results.

How to run
----------
1. Extract the archive while preserving the folder structure.
2. Open MATLAB.
3. Set the Current Folder to the extracted archive's root folder.
4. Run:
       run('matlab/run_all.m')

Generated outputs
-----------------
The script creates the following outputs beside the matlab folder:

    results.txt
    figs/*.pdf

All plot titles include the group number, which is set once near the
beginning of matlab/run_all.m:

    groupNo = 18;

Change that value if the group number needs to be updated.

Requirements
------------
MATLAB with Control System Toolbox.

Notes
-----
The actuator saturation limit in Section 8 is illustrative and normalized
(plus/minus 1). Replace it with the actual actuator limit if one is specified
by the project.

The Nyquist plot is zoomed around the critical point (-1, 0) for visibility.
This changes only the displayed axis limits, not the computed response.
