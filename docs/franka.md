# Franka  

Franka Arm + Franka Gripper  

## Dependencies  

- [Fork](https://github.com/hesic73/deoxys_control) of [Deoxys](https://github.com/UT-Austin-RPL/deoxys_control)  

## Quickstart  

### Workstation (the machine running the real-time kernel for controlling Franka)  

Start the control processes for the arm and gripper in two terminals:  

```bash
bash scripts/franka/auto_arm.sh
```

```bash
bash scripts/franka/auto_gripper.sh
```

Then start the server:  

```bash
# This environment mainly installs Deoxys
conda activate hsc
python run_server.py
```

### Client  

Refer to the example code in the `examples` directory:  

```bash
conda activate deploy
python examples/franka/demo_client.py
```

## API  

Refer to the comments in `arm_hand_deployment/franka/proto/service.proto` and usage examples in `examples/franka`.