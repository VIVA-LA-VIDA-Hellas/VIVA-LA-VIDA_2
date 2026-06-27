
## Differential – [Differential Model / Type]
The robot uses a **[differential model / type]** to split power from the drivetrain into independent left and right wheel motion. This differential allows the robot to turn smoothly by letting the wheels rotate at different speeds while still sharing a common input. As a result, it improves maneuverability on corners and reduces drivetrain stress compared to forcing the left and right sides to rotate at the same speed.

<img width="412" height="273" alt="image" src="https://github.com/user-attachments/assets/ae87d359-5e80-4ba8-a7b6-2fae191879cf" />


### 1 Differential Technical Specifications

**Mechanical Characteristics**
- Differential type: **[open / locked / limited-slip / gear differential / belt differential]**
- Input method: **[gear input / belt input / axle input]**
- Output method: **left/right outputs**
- Gear ratio: **[X:Y or X.xx:1]** (if applicable)
- Backlash / play: **[note, if known]**
- Intended wheel count/layout: **[e.g., 2-wheel output / 4-wheel layout]**

**Performance Characteristics**
- Expected turning behavior: **smooth differential turn**
- Torque transfer: **shared input → independent wheel speed**
- Benefits under load: **maintains traction while turning**

### 2 Why This Differential Was Chosen
1. **Smooth, controlled turning**  
   A differential lets the left and right sides rotate at different speeds during turns. This prevents one side from “fighting” the other, so the robot can arc through corners rather than scrubbing traction.

2. **Reduced drivetrain stress**  
   Because the wheels aren’t forced to match speed during steering, mechanical loads are lower. This helps reduce wear on drivetrain components during repeated cornering.

3. **Better traction and repeatability**  
   The robot maintains more consistent grip while following the controller’s intended path. That improves repeatability across runs, especially when turning under variable field friction.

4. **Straightforward integration**  
   The differential architecture is compatible with common drivetrain layouts:
   - shared input (from the motor)
   - separate left/right output paths to the wheel axles

### 3 Drivetrain and Power Integration
The differential is connected as follows:
- Motor output shaft → **differential input**
- Differential left output → **left wheel axle**
- Differential right output → **right wheel axle**

Advantages of this setup:
- Keeps power transfer centralized at the drivetrain input
- Allows turning behavior to come naturally from drivetrain kinematics (left/right speed divergence)
- Simplifies system tuning since turning is handled mechanically rather than relying entirely on software while the wheels fight each other

### 4 Alternative Drivetrain Approaches Considered
1. **Locked left/right drive (no differential)**  
   Pros: simpler mechanism  
   Cons: increases wheel scrub and mechanical stress; less smooth cornering

2. **Independent motors without differential (direct left/right control)**  
   Pros: software can command left/right speed explicitly  
   Cons: more tuning complexity; wheel slip can make behavior less predictable

3. **Chain/belt split without differential behavior**  
   Pros: packaging simplicity  
   Cons: still forces constraints between wheels, causing drag and uneven traction in turns

### Summary
The **[differential model/type]** provides the best combination of smooth cornering, reduced mechanical stress, and reliable drivetrain behavior. It enables correct turning by allowing left and right wheel speeds to diverge when required, producing more repeatable motion under field conditions.
