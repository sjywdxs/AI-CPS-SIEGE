# AI-CPS-SIEGE

Artifact evaluation for TSE submission "SIEGE: A Semantics-Guided Safety Enhancement Framework for AI-enabled Cyber-Physical Systems" by Jiayang Song, Xuan Xie and Lei Ma.

## System requirement

- Operating system: Linux / MacOS / Windows;
- Matlab (Simulink/Stateflow) version: >= 2022a; (Matlab license needed)
- MATLAB toolboxes dependency
  1. [Simulink](https://www.mathworks.com/products/simulink.html)
  2. [Stateflow](https://www.mathworks.com/products/stateflow.html)
  3. [Model Predictive Control Toolbox](https://www.mathworks.com/products/model-predictive-control.html)
  4. [Deep Learning Toolbox](https://www.mathworks.com/products/deep-learning.html)
  5. [Reinforcement Learning Toolbox](https://www.mathworks.com/products/reinforcement-learning.html)
  6. [Automated Driving Toolbox](https://www.mathworks.com/products/automated-driving.html)
  
## Code Structure

The directory `AI-CPS-SIEGE/` contains 5 sub-directory `ACC/`, `AFC/`, `APV/`, `BBC/`, and `LKA/`. Each of them stores the indicated AI-CPS introduced in Experimental Systems, and we take the directory `LKA/` as an example.

`LKA/` involves the following sub-directories:

- `abstraction/` stores the scripts for the abstraction process. 

- `data_generation/` has one sub-directory: `generated_date/`.This folder `generated_date/` stores the simulated data from each DRL controller. And the script `LKA_data_generator.mlx` initiates the simulation with selected controllers.

- `DRL_training/` stores the scripts for training new DRL agents for the AI-CPS. This folder contains three scripts, `LKA_DDPG_training.mlx`, `LKA_TD3_training.mlx`, `LKA_SAC_training.mlx`, and one sub-directory `trained_agent/`. The former scripts train DRL agents with specified algorithms, and the latter collected the pre-trained agents. 

- `ensemble/`  stores the script `LKA_AC_ensemble_training.mlx` for training the "higher-level" coordinator DRL agent. And a sub-directory `trained_agent/` contains the pre-trained coordinator agents.

- `evaluation/` stores the.mat files that record the evaluation results for each type of controller.

- `model/` stores all the Simulink `.slx` that are used for system simulations.


## Installation

- Clone the repository `git clone https://github.com/sjywdxs/AI-CPS-SIEGE.git`
- It is recommended to install the latest version of MATLAB to handle the simulink models with reinforcement learning controllers.

 ## How to run the script 
- Details can be found [AI-CPS-SIEGE Replication-Instruction](https://sites.google.com/view/ai-cps-siege/replication-package)
