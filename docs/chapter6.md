---
sidebar_position: 7
---

# Chapter 6: Control Systems and Locomotion

Controlling a humanoid robot to walk, balance, manipulate objects, and interact safely with humans requires sophisticated algorithms coordinating dozens of actuators in real-time. This chapter explores the control strategies that enable stable locomotion, dexterous manipulation, and natural human-robot interaction.

## Bipedal Walking and Balance

Walking on two legs is inherently unstable - unlike four-legged animals or wheeled robots, humanoids must actively maintain balance. This presents unique control challenges.

### The Challenge of Bipedal Stability

Humans balance by constantly making small adjustments, but robots need explicit control strategies:

**Static vs. Dynamic Stability**:
- **Static**: Center of mass always above support polygon (slow, stable)
- **Dynamic**: Uses momentum, allows temporary instability (faster, efficient)

Walking is inherently dynamic - there are phases where the robot is supported by only one foot.

### Zero Moment Point (ZMP) Control

ZMP is the point on the ground where the sum of all horizontal forces equals zero. If the ZMP lies within the support polygon (the convex hull of contact points), the robot won't tip over.

**ZMP-based Walking**:
1. Plan footstep locations
2. Generate trajectories ensuring ZMP stays inside support
3. Execute planned motion
4. Adjust based on feedback

This approach ensures stability but produces relatively slow, conservative walking. Honda's ASIMO used ZMP control extensively.

**Limitations**:
- Requires flat, known terrain
- Cannot handle disturbances well
- Prohibits dynamic movements like running
- Computationally intensive trajectory optimization

### Capture Point Control

A more modern approach considers where the robot needs to step to arrest its falling motion.

**Capture Point**: The point on the ground where the robot must step to come to a complete stop.

**Linear Inverted Pendulum Model (LIPM)**: Simplifies robot dynamics to single point mass on inverted pendulum, enabling analytical solutions.

Capture point methods enable:
- Faster walking speeds
- Better disturbance rejection
- More natural gait patterns
- Reasoning about where to step next

### Model Predictive Control (MPC)

MPC optimizes control actions over a future time horizon:

1. Predict robot state evolution under candidate actions
2. Evaluate cost function (tracking error, control effort, constraints)
3. Execute first action from optimal sequence
4. Repeat at next time step (receding horizon)

**Advantages**:
- Naturally handles constraints (joint limits, friction, torque bounds)
- Optimizes over multiple steps ahead
- Adapts to changing objectives

**Challenges**:
- Computationally expensive (optimization at 10-100 Hz)
- Requires accurate dynamics model
- Tuning cost functions is difficult

Modern controllers often use simplified models in MPC for speed while compensating for model errors through feedback.

### Learning-Based Control

Reinforcement Learning has achieved impressive walking controllers:

**Training Approach**:
1. Simulate millions of walking attempts
2. Reward forward progress, penalize falls
3. Neural network learns from experience
4. Deploy on real robot with fine-tuning

**Advantages**:
- Handles complex dynamics without explicit modeling
- Discovers efficient gaits
- Naturally robust to disturbances

**Challenges**:
- Requires massive simulation (millions of steps)
- Sim-to-real transfer is imperfect
- Difficult to provide safety guarantees
- Can be unpredictable in novel situations

Hybrid approaches combining model-based and learning-based methods show promise.

## Whole-Body Control

Walking is just one aspect - robots must coordinate all degrees of freedom to accomplish tasks.

### Hierarchical Control

Typical architecture has multiple layers:

**High-Level Planning** (0.1-1 Hz):
- Task decomposition
- Motion planning
- Footstep planning

**Mid-Level Control** (10-100 Hz):
- Whole-body coordination
- Balance maintenance
- Trajectory tracking

**Low-Level Control** (100-1000 Hz):
- Joint position/torque control
- Motor commutation
- Sensor filtering

Each layer operates at appropriate timescale with faster layers ensuring stability while slower layers provide intelligence.

### Quadratic Programming (QP) Control

Whole-body control formulated as optimization:

**Minimize**: `||Ax - b||²` (tracking desired motion)

**Subject to**:
- Joint position limits
- Joint velocity limits
- Joint torque limits
- Contact force constraints (friction cones)
- Non-penetration constraints
- Balance constraints

Solving this QP at 100-500 Hz produces joint commands that:
- Track desired motion as closely as possible
- Respect all physical constraints
- Maintain balance and contact

This approach elegantly handles redundancy (more DOFs than strictly needed) and conflicting objectives.

### Impedance Control

Rather than tracking position rigidly, impedance control modulates the mechanical impedance (relationship between force and position):

**Stiff Control**: High impedance, precise positioning (pick-and-place)
**Compliant Control**: Low impedance, gentle interaction (wiping table)
**Variable Impedance**: Adjust based on task requirements

Implementation:
- Measure position error and velocity
- Compute desired force: `F = K_p * error + K_d * velocity`
- Execute force command through current control

Impedance control enables safe physical interaction and handles contact uncertainty.

### Operational Space Control

Developed by Oussama Khatib, operational space control directly controls task-relevant coordinates (hand position, center of mass) rather than joint angles.

**Key Idea**: 
- Define desired behavior in task space (Cartesian coordinates)
- Compute forces in task space
- Map to joint torques through robot Jacobian

**Benefits**:
- Intuitive task specification
- Natural handling of redundancy
- Decouples control of different tasks

Modern whole-body controllers often use operational space formulations.

## Manipulation and Dexterity

Grasping and manipulating objects requires coordinated hand-arm control.

### Grasp Planning

Determining how to hold an object stably:

**Grasp Synthesis**:
- Identify potential contact points on object surface
- Evaluate grasp quality (force closure, resistance to disturbances)
- Select optimal grasp configuration

**Metrics**:
- **Force Closure**: Can apply arbitrary forces/torques
- **Form Closure**: Geometry alone prevents motion
- **Grasp Wrench Space**: Set of forces/torques achievable

### Force Control

Regulating forces during manipulation:

**Hybrid Position-Force Control**:
- Control position in unconstrained directions
- Control force in constrained directions (e.g., pushing on surface)

**Example**: Inserting peg into hole
- Control downward force (push into hole)
- Control lateral position (align with hole)

This enables assembly, polishing, and other contact-rich tasks.

### In-Hand Manipulation

Adjusting grasp without releasing object:

**Finger Gaiting**: Sequential repositioning of fingers
**Rolling**: Using differential finger motion to rotate object
**Sliding**: Controlled slip between fingers and object

This remains extremely challenging - requiring:
- Tactile feedback
- Precise force control
- Real-time adaptation

Recent learning-based approaches show promise but aren't yet reliable.

### Tool Use

Using tools requires understanding:
- Tool affordances (how tool can be used)
- Grasp points for effective use
- Interaction forces and trajectories

Large language models combined with vision are enabling more flexible tool use by providing high-level reasoning about tool properties and usage.

## Human-Robot Interaction (HRI)

Safe and effective collaboration with humans requires special considerations.

### Safety Requirements

**ISO 13482** standard defines safety requirements for personal care robots:

**Power and Force Limiting**: 
- Limit maximum forces in collisions
- Use soft materials on contact surfaces
- Emergency stop systems

**Safe Speed and Separation Monitoring**:
- Maintain distance from humans
- Slow down when humans approach
- Stop if collision imminent

**Hand Guiding**:
- Allow human to physically move robot
- Robot provides gentle resistance
- Return to task when released

### Intent Recognition

Predicting human actions enables proactive collaboration:

**Gaze Tracking**: Where is human looking?
**Gesture Recognition**: Hand motions indicating requests
**Activity Recognition**: What task is human performing?
**Trajectory Prediction**: Where will human move next?

These enable robots to anticipate needs and avoid interfering.

### Legibility

Robot motions should clearly communicate intent:

**Exaggerated Motions**: Slightly overshoot to show direction
**Preparatory Movements**: Telegraph upcoming actions
**Pacing**: Match human speed expectations

Legible behavior reduces human cognitive load and improves trust.

### Physical Collaboration

Working together on shared tasks:

**Leader-Follower**: Human guides, robot assists
**Role Division**: Each agent handles different aspects
**Shared Control**: Both contribute to same action (e.g., jointly lifting heavy object)

Successful collaboration requires:
- Understanding shared task goal
- Coordinating actions in real-time
- Adapting to human variations
- Communicating through motion

### Social Cues

Non-verbal communication:

**Body Language**: Posture indicating engagement or retreat
**Gaze**: Eye contact (if robot has expressive face)
**Proxemics**: Respecting personal space
**Gestures**: Nodding, pointing, waving

While not strictly necessary for function, social cues make interaction more natural and comfortable.

## Robustness and Adaptation

Real-world deployment requires handling unexpected situations.

### Disturbance Rejection

Maintaining performance despite:
- **External forces**: Pushes, pulls, collisions
- **Terrain variations**: Slopes, steps, uneven ground
- **Sensor noise**: Measurement errors and dropouts
- **Model uncertainty**: Reality differs from assumed dynamics

**Control Strategies**:
- **High-gain feedback**: Quickly correct errors
- **State estimation**: Filter noisy measurements
- **Adaptive control**: Update model online
- **Robust control**: Guarantee performance despite bounded uncertainty

### Fall Prevention and Recovery

Falls are expensive (damage) and dangerous (to robot and nearby humans):

**Prevention**:
- Conservative balance margins
- Detect impending falls early
- Take corrective steps quickly

**Recovery**:
- Controlled falling (minimize impact)
- Protective reflexes (arms absorb impact)
- Safe shutdown of systems
- Getting up from ground (if possible)

Some advanced humanoids can fall, roll, and stand back up autonomously.

### Online Adaptation

Adjusting to changing conditions:

**Parameter Adaptation**: Update controller gains based on performance
**Model Learning**: Improve dynamics model from experience
**Contact Detection**: Recognize and adapt to unexpected contacts

Adaptation enables operation in uncertain environments without extensive pre-programming.

## Advanced Locomotion

Beyond basic walking, advanced humanoids are achieving:

### Running

Dynamic running requires:
- Flight phase (both feet off ground)
- High impact forces at landing
- Precise timing and coordination

Boston Dynamics' Atlas demonstrates running and jumping capabilities.

### Rough Terrain

Navigating uneven ground requires:
- 3D terrain perception
- Adaptive footstep planning
- Continuous balance adjustment
- Robust contact handling

This remains challenging but improving rapidly.

### Climbing

Stairs, ladders, and obstacles:
- Requires reaching with hands
- High centers of mass
- Large changes in support configuration

Some humanoids can climb stairs and ladders reliably.

## Integration Example: Walking Pipeline

Typical humanoid walking involves:

1. **Perception** (50 Hz): Process camera/LiDAR to build terrain map
2. **Footstep Planning** (1 Hz): Decide where to step next 2-4 steps ahead
3. **Trajectory Generation** (10 Hz): Compute desired motion
4. **Whole-Body Control** (200 Hz): Generate joint torques
5. **Low-Level Control** (1000 Hz): Motor controllers execute commands

This pipeline coordinates multiple algorithms at different rates, with faster layers compensating for limitations of slower layers.

## Conclusion

Control systems bridge the gap between AI intelligence and physical capability. Bipedal walking requires carefully designed controllers that maintain stability while achieving efficient motion. Whole-body coordination enables simultaneous locomotion and manipulation. Human-robot interaction demands safety, legibility, and adaptation.

Modern control combines classical techniques (ZMP, MPC, impedance control) with learning-based approaches. As algorithms improve and computational power increases, humanoid robots will achieve more natural, robust, and capable movement.

The next chapters explore how perception and learning enable these control systems to operate intelligently in complex, changing environments.

---

**Next Chapter**: [Perception in Humanoid Robots →](/chapter7)