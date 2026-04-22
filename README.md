# Task-Coordinated Path Optimization for Grouped USV Formations

Official MATLAB implementation of the paper: **"Task-Coordinated Path Optimisation for Grouped Unmanned Surface Vehicle Formations"**.

---

## 🚀 Key Highlights
- **Incremental Re-optimization**: Achieved a **74.96%** improvement in computational efficiency compared to global re-optimization.
- **High Robustness**: Maintained a **99.2%** obstacle avoidance success rate under 0.12 m/s ocean current disturbances.
- **Physical Feasibility**: Strictly adheres to the **30° steering angle constraint**, ensuring paths are compatible with USV kinematics.
- **Scalability**: Supports real-time task allocation and path generation for swarms of up to 32 agents.

## 📂 Project Structure
All core source files are located in the `/Github` directory:
- **`init_env.m`**: Script to initialize the MATLAB environment and search paths.
- **`01_Path_Planning/`**: Contains core SCA-APF algorithms and single-USV benchmark tests.
- **`02_Task_Allocation/`**: Implementation of the improved Hungarian algorithm for multi-target assignment.
- **`03_Cooperative_Mechanism/`**: **Core Contribution.** Includes coordination mechanism validation, efficiency benchmarking, and sensitivity analysis.

## 🛠️ Getting Started
1. **Clone/Download** this repository to your local machine.
2. Open the project root directory in **MATLAB**.
3. Run **`Github/init_env.m`** to initialize the search paths.
4. Execute **`Github/03_Cooperative_Mechanism/Main_System_Academic_Final.m`** to reproduce the full-system simulation results presented in the paper.

## 📊 Citation
If you find this code or research useful, please cite our paper:

```bibtex
@article{GW2026USV,
  title={Task-Coordinated Path Optimisation for Grouped Unmanned Surface Vehicle Formations},
  author={G.W., et al.},
  journal={Your Journal Name},
  year={2026},
  doi={Your DOI}
}
