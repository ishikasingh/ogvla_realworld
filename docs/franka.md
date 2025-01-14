# franka

franka arm + franka gripper

## 依赖

- [fork](https://github.com/hesic73/deoxys_control) of [deoxys](https://github.com/UT-Austin-RPL/deoxys_control)


## Quickstart

### workstation（装real-time kernel控制franka的那台机器）

在两个terminal启动arm和gripper的控制进程：

```bash
bash scripts/auto_gripper.sh
```

```bash
bash scripts/auto_gripper.sh
```

然后启动server：

```bash
# 这个环境最主要就是装deoxys
conda activate hsc
python run_server.py
```

### client

可以参考 examples里面的代码


```bash
conda activate deploy
python examples/franka/demo_client.py
```


## API

参考`arm_hand_deployment/franka/proto/service.proto`的注释和`examples/franka`中的用法

