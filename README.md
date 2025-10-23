# EARN

This is the code of **E**dge **A**ccelerated **R**obot **N**avigation With Collaborative Motion Planning.

![Image](https://github.com/user-attachments/assets/2e347949-2c45-40ba-8180-0ae5f13b658e)

## Preqrequisite

Please install the following libraries and packages first.

- Python >= 3.8
- [CVXPY](https://www.cvxpy.org/)
- [ROS Noetic](https://wiki.ros.org/noetic)
- [Carla](https://carla.org/)
- [Carla-ROS-bridge](https://github.com/carla-simulator/ros-bridge)

> [!NOTE]
> *We recommend using Python 3.8 and Conda to manage Python environments, as examples are based on CARLA.*

## Test Environment 

- Ubuntu 20.04

## Code Use

You can firstly install the package by running the commands below:

```
bash ./setup.sh
```

For standalone navigation, you can run the commands below:
```
cd $CARLA_ROOT
bash ./CarlaUE4.sh
```
```
./pdd_demo.sh
```

For multi-vehicle navigation, you can run the commands below:
```
cd $CARLA_ROOT
bash ./CarlaUE4.sh
```
```
./earn_demo.sh
```

## Demo Video

https://github.com/user-attachments/assets/69c7f4a9-a04b-4646-81fa-2ae797dac053


> [!NOTE]
EARN is developed based on RDA-planner and rda_ros. You may want to see more information in [RDA-planner](https://github.com/hanruihua/RDA-planner) and [rda_ros](https://github.com/hanruihua/rda_ros)


## Parameters

#### PDD Configuration

| Parameter               | Type    | Default Value | Description                                                |
| ----------------------- | ------- | ------------- | ---------------------------------------------------------- |
| `~receding`             | `int`   | `15`          | Receding horizon parameter for MPC.                        |
| `~iter_num`             | `int`   | `3`           | Number of iterations for the MPC solver.                   |
| `~enable_reverse`       | `bool`  | `False`       | Enables reverse movement if set to `True`.                 |
| `~sample_time`          | `float` | `0.3`         | Sampling time interval for the MPC.                        |
| `~process_num`          | `int`   | `4`           | Number of parallel processes for MPC computation.          |
| `~accelerated`          | `bool`  | `True`        | Enables accelerated computation in MPC.                    |
| `~time_print`           | `bool`  | `False`       | Enables time logging for MPC operations.                   |
| `~obstacle_order`       | `bool`  | `True`        | Determines if obstacle ordering by distance is applied.    |
| `~max_edge_num`         | `int`   | `4`           | Maximum number of edges to consider for obstacles.         |
| `~max_obs_num`          | `int`   | `4`           | Maximum number of obstacles to consider.                   |
| `~goal_index_threshold` | `int`   | `1`           | Threshold for goal index determination.                    |
| `~iter_threshold`       | `float` | `0.2`         | Iteration threshold for convergence in MPC.                |
| `~slack_gain`           | `float` | `10`           | Gain for slack variables in MPC constraints.               |
| `~max_sd`               | `float` | `1.0`         | Maximum safety distance.                                   |
| `~min_sd`               | `float` | `0.1`         | Minimum safety distance.                                   |
| `~ws`                   | `float` | `0.2`         | Weight for the state in the cost function.                 |
| `~wu`                   | `float` | `1.4`         | Weight for the control inputs in the cost function.        |
| `~ro1`                  | `float` | `0.1`         | Weight parameter for the first term in the cost function.  |
| `~ro2`                  | `float` | `0.1`         | Weight parameter for the second term in the cost function. |
| `~pdd_en`               | `bool` | `True`         | Enables penalty dual decomposition.  |
| `~edge_accelerate`      | `bool` | `True`         | Enables edge collaboration.  |


#### MPS Configuration

| Parameter               | Type    | Default Value | Description                                                |
| ----------------------- | ------- | ------------- | ---------------------------------------------------------- |
| `~edge_position`             | `list`   | `[255, 172]`          | Position of edge server.                        |
| `~coverage`             | `int`   | `500`          | Communication coverage of edge server.                        |
| `~Cth`             | `int`   | `30`           | Maximum computation load in ms for the edge solver.                   |
| `~receding`             | `int`   | `15`          | Receding horizon parameter for MPC.                        |
| `~max_edge_num`         | `int`   | `4`           | Maximum number of edges to consider for obstacles.         |
| `~max_obs_num`          | `int`   | `4`           | Maximum number of obstacles to consider.                   |



## Paper Video

https://github.com/GuoliangLI1998/EARN/assets/107024891/21dcf0b7-ad05-4bb3-bbad-3f255fed22bc

## Citation

If you find our work helpful in your research, please consider **starring** the repository and **citing** our work using the following BibTeX entries:

```
@ARTICLE{10601554,
  author={Li, Guoliang and Han, Ruihua and Wang, Shuai and Gao, Fei and Eldar, Yonina C. and Xu, Chengzhong},
  journal={IEEE/ASME Transactions on Mechatronics}, 
  title={Edge Accelerated Robot Navigation With Collaborative Motion Planning}, 
  year={2025},
  volume={30},
  number={2},
  pages={1166-1178},
  doi={10.1109/TMECH.2024.3419436}}
```

```
  @ARTICLE{10036019,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Zhang, Qianru and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Robotics and Automation Letters}, 
  title={RDA: An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments}, 
  year={2023},
  volume={8},
  number={3},
  pages={1715-1722},
  doi={10.1109/LRA.2023.3242138}}

```

## Acknowledgement

We would like to thank the authors and developers of the following projects. This project is built upon these great open-sourced projects.
- [RDA-planner](https://github.com/hanruihua/RDA-planner)
- [rda_ros](https://github.com/hanruihua/rda_ros)
- [Carla](https://carla.org/)
- [Carla-ROS-bridge](https://github.com/carla-simulator/ros-bridge)

## Authors

[Guoliang Li](https://github.com/GuoliangLI1998)

[Ruihua Han](https://github.com/hanruihua)

[Shuai Wang](https://github.com/bearswang)
