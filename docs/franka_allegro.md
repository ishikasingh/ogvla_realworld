# Franka  

Franka Arm + Allegro Hand

## Dependencies  

- [Fork](https://github.com/hesic73/deoxys_control) of [Deoxys](https://github.com/UT-Austin-RPL/deoxys_control)  

- [Allegro Hand Controller - Noetic](https://github.com/NYU-robot-learning/Allegro-Hand-Controller-DIME)

## Quickstart  

### Workstation (the machine running the real-time kernel for controlling Franka)  

Start the control processes for the arm and hand in two terminals:  

```bash
bash scripts/franka_allegro/auto_arm.sh
```

```bash
bash scripts/franka_allegro/allegro_ros.sh
```

Then start the server:  

```bash
# This environment mainly installs Deoxys
conda activate hsc
bash scripts/franka_allegro/run_server.sh
```

### Client  

Refer to the example code in the `examples` directory:  

```bash
conda activate deploy
python examples/franka_allegro/demo_client.py
```

## API  

Refer to the comments in `arm_hand_deployment/franka_allegro/proto/service.proto` and usage examples in `examples/franka_allegro`.