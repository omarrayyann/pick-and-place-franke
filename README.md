# Pick & Place Manipulation

In this project, I Implemented a controller that builds a tower of 3 cubes using pose impedance control. The controller can be seen in the figure below.

![rob-controls-4](https://github.com/omarrayyann/pick-and-place-franke/assets/77675540/f7c700f7-35f6-4855-8ade-2e9af4b3f6cc)

<img width="300" alt="Screenshot 2024-05-03 at 10 35 37 PM" src="https://github.com/omarrayyann/pick-and-place-franke/assets/77675540/bb81a6ce-7599-4901-96d9-7ba769243a2d">

where:
- $\boldsymbol{\tau} \in \mathbb{R}^7$ is the vector of joint torques
- $\mathbf{J} \in \mathbb{R}^{6 \times 7}$ is the Jacobian matrix of the robot
- $\mathbf{P} \in \mathbb{R}^{6 \times 6}$ is the proportional gain matrix
- $\mathbf{D} \in \mathbb{R}^{6 \times 6}$ is the derivative gain matrix
- $\mathbf{T}_{\text{current}}, \mathbf{T}_{\text{desired}} \in \text{SE}(3)$ are the current and desired pose transformation matrices in $\text{SE}(3)$
- $\text{log}(\cdot)$ computes the matrix logarithm
- $(\cdot)^\vee$ denotes the vee map, converting a skew-symmetric matrix to a twist vector
- $\Delta \boldsymbol{V} \in \mathbb{R}^6$ is the twist error vector
