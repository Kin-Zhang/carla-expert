# CARLA-expert

This repo collects the expert in open-source for CARLA leaderboard. As we all known that the expert is important for end-to-end learning driving. The expert is also the upper bound of learning agent performance, smart and efficient expert sometimes may more important than your agent network. 

对于一个端到端驾驶来说，第一步就是收集数据，在真实世界，收集数据时都是以人驾驶车为主；而在仿真世界中 一般采用全局视角写一个专家策略；以此作为收集数据时的策略。此repo主要提供一些已开源 并且在e2e agent上训练使用的一些experts

Here are scores that four experts run in the testing route xml provided from CARLA official leaderboard branch and with `public_scenario`



**!!! TODO RESULTS TABLE !!!**



## Setup

please remember to clone with `--recurse-submodules`

```bash
git clone --recurse-submodules https://github.com/Kin-Zhang/carla-expert
```

For people who don't have CARLA [在内地的同学可以打开scripts换一下函数 走镜像下载更快点.. ]

```bash
./run/setup_carla.sh
# input version
11
# auto download now ...
```

Set the correct the absolute path and CARLA run path!!!

```bash
conda activate py37
export CODE_FOLDER=/home/kin/workspace/carla-expert
export CARLA_ROOT=/home/kin/CARLA_0.9.10.1
export SCENARIO_RUNNER_ROOT=${CODE_FOLDER}/scenario_runner
export LEADERBOARD_ROOT=${CODE_FOLDER}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg":"${CODE_FOLDER}/team_code":${PYTHONPATH}
```

## Running
the default one is mmfn, please remember to change the path!
```bash
python run_collect.py carla_sh_path=${CARLA_ROOT}/CarlaUE4.sh absolute_path=${CODE_FOLDER}
```
### A. [Roach](https://github.com/zhejz/carla-roach)
This expert used the trained RL for expert. This part of jobs is done by Kait from this repo: [https://github.com/Kait0/carla-roach](https://github.com/Kait0/carla-roach)



Questions inside 已知问题：

- Testing on Town01, finding that the velocity is quite high and the brake distance on traffic light may exceed the traffic light and lead the light may not shown at camera images. Town01测试时，红绿灯路口由于速度较高，停下时已经超过红绿灯了 导致红绿灯在图片中不可见

### B. [MMFN](https://github.com/Kin-Zhang/mmfn)

This expert is from [https://github.com/Kin-Zhang/mmfn](https://github.com/Kin-Zhang/mmfn)



已知问题：

- because of TTC, the brake will be in advance which the reason may not appear if you are using just one camera for trainning agent. 因为内置的time to collision，有时候刹车会较为提前，如果用一个相机可能刹车原因并不会出现在照片中

### C. [SEED](https://github.com/autonomousvision/transfuser/tree/2022/team_code_autopilot)

This expert is from [https://github.com/autonomousvision/transfuser/tree/2022/team_code_autopilot](https://github.com/autonomousvision/transfuser/tree/2022/team_code_autopilot)



Questions inside 已知问题：

- change lane may have collision according to their new paper. 根据新paper来看，换道情况并未考虑进入

### D. [AUTO](https://github.com/carla-simulator/carla/blob/master/PythonAPI/carla/agents/navigation/local_planner.py)

This expert if from CARLA self for local planner with walker event evolved from mmfn repo by Kin.



Questions inside 已知问题：

- only using the distance, efficient but not smart in long term. 仅以距离为因素进行刹停，有效但是长远来看并不smart

## Plan

- [ ] Involved the e2e agent also. Big hope... Since different e2e agent need different data form
- [ ] Experiemtns all the e2e agent in same expert and test their performance, since in the leaderboard, their agent has train in different expert and different dataset, it should be control the suitation in the experiment. 最好做一次完整的实验，控制所有的变量（数据 专家的一致性） 来评判e2e agent的有效性，从leaderboard看来大家的expert不一致，训练数据大小也各不相同，直接从榜单结果看谁的方法更好并不完全有效


Welcome to contribute!

## Acknowledgements

GITHUB code repo:
- [zhejz/carla-roach](https://github.com/zhejz/carla-roach)
- [autonomousvision/transfuser](https://github.com/autonomousvision/transfuser)
- [dotchen/WorldOnRails](https://github.com/dotchen/WorldOnRails)
- [Kin-Zhang/mmfn](https://github.com/Kin-Zhang/mmfn)

Here are other discussion on CARLA leaderboard: [https://github.com/Kin-Zhang/LAV/discussions](https://github.com/Kin-Zhang/LAV/discussions)

