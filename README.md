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


> [!NOTE]
EARN is developed based on RDA-planner and rda_ros. You may want to see more information in [RDA-planner](https://github.com/hanruihua/RDA-planner) and [rda_ros](https://github.com/hanruihua/rda_ros)

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
