# Inter-shell Routing with Traffic Engineering for Multi-shell LEO Satellite Networks

This project extends the Hypatia Low-Earth-Orbit Satellite Networks (LEOSN) simulator to support Software-Defined Networking (SDN) capabilities. This extension enables the controller to use a TE-based flow routing algorithm for per-flow traffic engineering.

## Getting started

1. Our System setup
   - Python version: Python 3.8.20
   - Operating System: Ubuntu 18.04

2. Detailed installation instructions of Hypatia
   please refer to <a href="https://github.com/snkas/hypatia">Hypatia</a>.

3. `kuiper_630_telesat_1015_isls_plus_grid_ground_stations_298_algorithm_free_one_only_over_isls.tar.gz`
   Move it into `<hypatia>/paper/satellite_networks_state/gen_data` and extract.
   
4. Reproduction of the two-shell TE experiments
   Please navigate to `paper/traffic_matrix/README.md`.