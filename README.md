
# Hexapod Robot Control Software

This repository contains the complete firmware for a six-legged hexapod robot. The software enables the robot to perform various movements, including walking, in-place rotation, and body tilting, all controlled via serial commands. The architecture is built on an Arduino framework and uses inverse kinematics to calculate precise joint angles for smooth and coordinated leg movements.

-----

## Key Features 🤖

  * **State-Driven Control**: A robust state machine manages the robot's mode (e.g., `STAND`, `WALK`, `TILT`).
  * **Tripod Gait**: Implements a stable tripod gait for walking and rotation.
  * **Inverse Kinematics**: Accurately calculates all 18 servo angles to achieve desired foot positions.
  * **Dynamic Gaits**: Leg trajectories are generated using an elliptical path with quintic easing for exceptionally smooth acceleration and deceleration.
  * **Serial Control**: The robot is fully controlled by simple character commands sent over a serial connection.
  * **Body Manipulation**: Supports height adjustment and tilting the body in any direction.

-----

### Leg & Servo Mapping

The robot's six legs are numbered 1 through 6, starting from the front-left and moving counter-clockwise. Each leg has three servos: **Coxa** (hip horizontal), **Femur** (hip vertical), and **Tibia** (knee).

The `baseIDs` array in `hexapod.ino` maps the physical legs to their corresponding base servo IDs. Each base ID is for the Coxa servo, with the Femur and Tibia servos having IDs of `baseID - 1` and `baseID - 2`, respectively.

| Leg Number | Position | Base Servo ID | Coxa ID | Femur ID | Tibia ID |
| :--------: | :----------: | :-----------: | :-----: | :------: | :------: |
| 1 | Front-Left | 3 | 3 | 2 | 1 |
| 2 | Mid-Left | 6 | 6 | 5 | 4 |
| 3 | Rear-Left | 9 | 9 | 8 | 7 |
| 4 | Rear-Right | 12 | 12 | 11 | 10 |
| 5 | Mid-Right | 15 | 15 | 14 | 13 |
| 6 | Front-Right| 18 | 18 | 17 | 16 |

### Serial Command Handling

The robot is controlled by sending specific characters and values over the serial port. The `handleSerialInput()` function in `hexapod.ino` parses these commands to change the robot's state and parameters.

| Command | Argument | Description |
|:---:|:---:|---|
| `s` | - | Enters `STAND` mode, bringing all legs to a neutral standing position. |
| `o` | - | Halts any ongoing `WALK` or `ROTATE` sequence, returning to `STAND` mode. |
| `h` | - | Increases the robot's height (body moves up). Only works in `STAND` mode. |
| `d` | - | Decreases the robot's height (body moves down). Only works in `STAND` mode. |
| `w` | `float` | Enters `WALK` mode. The float value specifies the walking angle in degrees (0 = forward, 90 = left, etc.). |
| `r` | `float` | Enters `ROTATE` mode. The float value specifies the rotation angle (positive = counter-clockwise). |
| `t` | `float` | Enters `TILT` mode. The float value specifies the direction of tilt in degrees. |

**Pseudo-code for `handleSerialInput()`:**

```cpp
// from hexapod.ino
void handleSerialInput() {
    if (Serial has data) {
        char command = read first character;
        // If command is 'w', 'r', or 't', parse the following float value
        if (command == 'w' || command == 'r' || command == 't') {
            float value = Serial.parseFloat();
            if (command == 'w') {
                currentMode = WALK;
                Angle = value;
            } else if (command == 'r') {
                currentMode = ROTATE;
                RotateAngle = value;
            } else { // 't'
                currentMode = TILT;
                TiltAngle = value;
            }
        } else {
            // Handle single-character commands like 's', 'o', 'h', 'd'
            switch (command) {
                case 's': currentMode = STAND; break;
                // ... other cases
            }
        }
    }
}
```

-----

## Core Algorithms 🧠

### Inverse Kinematics (IK)

IK is used to find the required joint angles ($$\theta_1, \theta_2, \theta_3$$) for a leg's coxa, femur, and tibia, given a target 3D coordinate (x, y, z) for the foot. The calculations are performed in `inverse_kinematics.cpp`.

1.  **Top-Down View (Coxa Angle $$\theta_1$$)**: The coxa angle is found by projecting the leg onto the XY-plane.

    $$
    $$$$\\theta\_1 = \\text{atan2}(y, x)

    $$
    $$$$
    $$
2.  **Side View (Femur $$\theta_2$$ and Tibia $$\theta_3$$)**: The problem is reduced to a 2D triangle in the vertical plane defined by the leg. We use the law of cosines to find the internal angles $$\alpha$$ and $$\beta$$.

    $$
    $$$$r = \\sqrt{x^2 + y^2} - L\_1
    $$   $$
    d = \\sqrt{r^2 + z^2}
    $$    Where$$L\_1, L\_2, L\_3$$ are the lengths of the coxa, femur, and tibia links.

    $$
    $$$$\\alpha = \\text{acos}\\left(\\frac{L\_2^2 + d^2 - L\_3^2}{2 L\_2 d}\\right)
    $$   $$
    \\beta = \\text{acos}\\left(\\frac{L\_2^2 + L\_3^2 - d^2}{2 L\_2 L\_3}\\right)

    $$
    $$$$
    $$
3.  **Final Joint Angles**: The final angles are then calculated from these intermediate values.

    $$
    $$$$\\phi = \\text{atan2}(z, r)
    $$   $$
    \\theta\_2 = -(\\phi + \\alpha)
    $$   $$
    \\theta\_3 = \\pi - \\beta

    $$
    $$$$
    $$
### Gait Generation

Gaits are created by generating a sequence of points for the feet to follow. For walking and rotating, the legs in the air (swing phase) follow a smooth elliptical path, while the legs on the ground (stance phase) move in a straight line to propel the body.

#### Easing Function for Smooth Motion

To avoid jerky movements, the points along the trajectory are spaced using a **quintic easing function**. This provides very smooth acceleration and deceleration. This is implemented in `easedLinspace`.

$$\text{ease} = t^3 (t(6t - 15) + 10)$$

Where $$t$$ is the normalized time from 0 to 1.

```cpp
// from ellipse_generation.cpp
void easedLinspace(float start, float end, int num, float output[]) {
    for (int i = 0; i < num; ++i) {
        float t = (float)i / (num - 1); // normalized [0, 1]

        // Quintic ease-in-out for even smoother acceleration and deceleration
        float ease = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);

        output[i] = start + (end - start) * ease;
    }
}
```

#### Tripod Gait Logic

The tripod gait is the most common and stable hexapod gait. Two groups of three legs move in alternation.

  * **Group 1 (Swing)**: Legs 1, 3, 5 (FL, RL, MR)
  * **Group 2 (Stance)**: Legs 2, 4, 6 (ML, RR, FR)

The `moveLegWalk` function orchestrates this by iterating through the `NUM_POINTS` of the trajectory. First, it moves Group 1 along the elliptical (swing) path while Group 2 moves along the linear (stance) path. Then, they swap roles.

**Pseudo-code for `moveLegWalk()`:**

```cpp
// from hexapod.ino
void moveLegWalk(float jointAngles[...], float jointAnglesLine[...]) {
    // Phase 1: Group 1 swings, Group 2 stands/pushes
    for (step = 0 to NUM_POINTS-1) {
        for (each leg i from 1 to 6) {
            if (leg i is in Group 1) {
                angles = jointAngles[i][step]; // Ellipse path
            } else { // Leg is in Group 2
                angles = jointAnglesLine[i][step]; // Line path
            }
            mapServoAngles(..., angles, ...);
            sc.RegWritePos(...);
        }
        sc.RegWriteAction(); // Execute move for this step
        delay(100);
    }

    // Phase 2: Group 2 swings, Group 1 stands/pushes
    for (step = 0 to NUM_POINTS-1) {
        for (each leg i from 1 to 6) {
            if (leg i is in Group 2) {
                angles = jointAngles[i][step]; // Ellipse path
            } else { // Leg is in Group 1
                angles = jointAnglesLine[i][step]; // Line path
            }
            // ... map and write servo positions
        }
        sc.RegWriteAction();
        delay(100);
    }
}
```

### Servo Angle Mapping

The angles calculated by the IK solver are in degrees and need to be converted to a position value that the servos understand (typically 0-1023). The `mapServoAngles` function handles this conversion, while `getBaseAngleOffset` corrects for the physical orientation of each coxa servo.

```cpp
// from servo_mapping.cpp
void mapServoAngles(int baseServoID, float jointAngles[3], float jointAngles_mapped[3]) {
    for (int i = 0; i < 3; i++) {
        if (i == 0) {  // Coxa
            float offsetAngle = getBaseAngleOffset(baseServoID, jointAngles[i]);
            // pos = 512 * cos(angle) + 512
            jointAngles_mapped[i] = 512.0 * cos(PI * offsetAngle / 180.0) + 512.0;
        }
        else if (i == 1) {  // Femur
            // pos = 512 * sin(angle) + 512
            jointAngles_mapped[i] = 512.0 + 512.0 * sin(jointAngles[i] * PI / 180.0);
        }
        else if (i == 2) {  // Tibia
            // pos = 512 * cos(180 - angle) + 512 - offset
            jointAngles_mapped[i] = 512.0 + 512.0 * cos((180.0 - jointAngles[i]) * PI / 180.0) - 100.0;
        }
    }
}
```

-----
