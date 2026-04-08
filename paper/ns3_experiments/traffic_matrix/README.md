# Reproduction of our two-shell TE experiment

**Explanation**

The network topology used in our simulation experiments consists of two LEO constellations: “Amazon Leo” and Telesat, totaling 1,507 satellites. The ground nodes include 198 Starlink gateway stations and 1,000 UTs distributed across the 100 most populous cities worldwide. The total simulation duration is set to 200 seconds, with a granular time step of 0.1 seconds.

For traffic generation, we evaluate scenarios with 100, 200, and 300 TCP flows. To simulate realistic network loading, these flows are initiated sequentially at 0.1-second intervals.

The primary performance metrics evaluated in this study include average TCP flow throughput, end-to-end distance, and hop count.

**Commands**

Run and analyze by executing:

```
python two_shell_step_1.py
python general_step_2.py
python run_all_analyses.py
```
