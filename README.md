# MAE 506 Self-Balancing Robot
This MATLAB script models and simulates a self-balancing robot as a cart with an inverted pendulum. It checks system properties, designs a state-feedback controller, simulates the robot’s response, and creates plots and an animation video.

## Model
The system is linearized around the upright position. Its state vector is

\[
z = [x,\ \dot{x},\ \theta,\ \dot{\theta}]^T
\]

where \(x\) is cart position and \(\theta\) is the pendulum angle. The input is the horizontal force applied to the cart, and the output is cart position.

The model parameters used in the simulation are:

| Parameter | Value | Description |
|---|---:|---|
| \(M\) | 0.50 kg | Cart mass |
| \(m\) | 0.20 kg | Pendulum mass |
| \(l\) | 0.30 m | Pivot-to-center-of-mass distance |
| \(I\) | 0.006 kg·m² | Pendulum moment of inertia |
| \(b\) | 0.10 N·s/m | Cart damping |
| \(g\) | 9.81 m/s² | Gravitational acceleration |

## Controller and simulation
The script uses pole placement to design a state-feedback controller with desired closed-loop poles at \(-2\), \(-2.5\), \(-3\), and \(-3.5\). It simulates the robot for 8 seconds with:

- An initial pendulum angle of \(8^\circ\)
- A 1 N external push from 2.0 to 2.1 seconds
- A cart position command of 0.10 m starting at 4 seconds

The model assumes all four states are available to the controller. Motor force limits and sensor noise are not included.

## Requirements and how to run
The script requires MATLAB and the **Control System Toolbox** for `ctrb`, `obsv`, and `place`.

1. Save the code as `self_balancing_robot.m` from upper right hand corner.
2. Open the file in MATLAB.
3. Click Run or enter `self_balancing_robot` in the Command Window.

## Outputs
The script displays the controllability and observability ranks, open-loop poles, controller gain, reference gain, closed-loop poles, and simulation performance measures.
It also creates plots for cart position, robot tilt, and drive force. The animation is saved as `robot_animation.mp4` in MATLAB’s current folder.

## Results
From the Command Window:

- Controllability rank: 4 of 4
- Observability rank: 4 of 4
- Settling time after the position command: 3.440 s, using a 0.002 m error band
- Final position error: −0.000723 m
- Final pendulum angle: −0.0156°
- Maximum absolute cart position: 0.2315 m
- Maximum absolute pendulum angle: 8.000°
- Maximum applied force: 2.399 N
- Will also have figure that pops up with animation!
