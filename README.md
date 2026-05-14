# Inter-shell Routing with Traffic Engineering for Multi-shell LEO Satellite Networks

This project extends the Hypatia Low-Earth-Orbit Satellite Network (LEOSN) simulator with **Software-Defined Networking (SDN)** capabilities. With this extension, the controller can run a **Traffic Engineering (TE)** flow-routing algorithm to perform **per-flow traffic engineering**, enabling research and experiment reproduction for inter-shell routing in LEO satellite networks.

## Requirements (verified)
- OS: Ubuntu 18.04
- Python: 3.8.20

> Other Ubuntu/Python versions may work, but the steps in this README are validated on the versions above.

---

## Getting Started

### 1) Install Hypatia
First, install Hypatia by following its official instructions:
- Hypatia repository: <a href="https://github.com/snkas/hypatia">https://github.com/snkas/hypatia</a>

### 2) Prepare the dataset (gen_data)
Move the following file:

- `kuiper_630_telesat_1015_isls_plus_grid_ground_stations_298_algorithm_free_one_only_over_isls.tar.gz`

into:

- `<hypatia>/paper/satellite_networks_state/gen_data/`

Then extract it in that directory, for example:
```bash
cd <hypatia>/paper/satellite_networks_state/gen_data
tar -xzf kuiper_630_telesat_1015_isls_plus_grid_ground_stations_298_algorithm_free_one_only_over_isls.tar.gz
```

---

## Reproducing Experiments

### Two-shell TE experiments
Follow the instructions in:
- `paper/traffic_matrix/README.md`

---

## Notes
- This repository is an **extension of Hypatia**, so data paths, data generation workflows, and experiment execution largely follow Hypatia’s conventions.