---
sidebar_position: 9
---

# Chapter 8: Learning and Adaptation

Physical AI systems must learn from experience and adapt to changing conditions. This chapter explores how robots acquire skills through demonstration, self-supervised learning, and continuous adaptation in deployment.

## Learning from Demonstration

Humans are expert manipulators with decades of experience. Learning from human demonstrations dramatically accelerates robot skill acquisition.

### Behavioral Cloning

The simplest approach treats demonstration learning as supervised learning:

**Process**:
1. Human demonstrates task multiple times
2. Record state-action pairs: (observation, action taken)
3. Train neural network to predict actions from observations
4. Deploy learned policy on robot

**Example**: Grasping objects
- Record 1000 grasps with different objects
- Train network: image → grasp pose
- Robot imitates learned behavior

**Advantages**:
- Sample efficient compared to reinforcement learning
- No reward engineering needed
- Leverages human expertise
- Works with relatively few demonstrations (10-1000)

**Challenges**:
- **Distribution shift**: Learner encounters states not in demonstrations
- **Compounding errors**: Small mistakes lead to unfamiliar states
- **Suboptimal demonstrations**: Learns human mistakes too
- **Ambiguity**: Same state might require different actions in different contexts

### DAgger (Dataset Aggregation)

Addresses distribution shift by iteratively collecting corrections:

**Algorithm**:
1. Train initial policy from expert demonstrations
2. Deploy policy, let it control robot
3. Expert watches and provides corrections when policy makes mistakes
4. Add expert corrections to training dataset
5. Retrain policy on expanded dataset
6. Repeat until satisfactory performance

This exposes the expert to states the policy actually encounters, teaching recovery behaviors.

**Success Stories**:
- Autonomous helicopter flying
- Off-road driving
- Manipulation tasks

### Kinesthetic Teaching

Physical guidance where human moves robot through desired motions:

**Method**:
- Robot enters compliant mode (low impedance)
- Human physically guides robot arm
- Robot records joint trajectories
- Playback or generalize from demonstrations

**Advantages**:
- Intuitive for non-experts
- Captures precise timing and forces
- No teleoperation equipment needed

**Limitations**:
- Requires physical access to robot
- Difficult for high-speed or dynamic motions
- Human must support robot weight

### Video Demonstrations

Learning from videos of humans performing tasks:

**Challenges**:
- **Embodiment mismatch**: Human hands differ from robot grippers
- **Viewpoint**: Video from different perspective than robot
- **Correspondence**: Mapping human motions to robot actions

**Recent Approaches**:
- Foundation models understand tasks from video
- Retargeting algorithms map human motion to robot kinematics
- Learning object affordances rather than exact motions

This unlocks internet-scale demonstration data (YouTube cooking videos, assembly instructions, etc.).

## Self-Supervised Learning

Learning representations from unlabeled data reduces reliance on human annotation.

### Contrastive Learning

Learn representations by comparing similar and dissimilar examples:

**SimCLR approach**:
1. Take image from robot camera
2. Apply two different augmentations (crop, color jitter, blur)
3. Train network so augmented versions have similar representations
4. Push representations of different images apart

**Result**: Representations capture meaningful visual features useful for downstream tasks.

**Applications**:
- Pre-training vision encoders for manipulation
- Learning object-centric representations
- Cross-modal learning (vision + touch)

### Predictive Learning

Predict future observations from current state and action:

**World Models**: Learn dynamics of environment
- Given current observation and action
- Predict next observation
- Implicitly learns physics and object behavior

**Benefits**:
- No labeling required (just record robot operation)
- Captures task-relevant features
- Enables planning in learned model

**Challenges**:
- High-dimensional predictions (images) are difficult
- Accumulating errors over long horizons
- Distinguishing important from irrelevant details

### Autoencoders and Representation Learning

Compress high-dimensional observations into low-dimensional codes:

**Variational Autoencoders (VAE)**:
- Encode observations to latent space
- Decode latent codes back to observations
- Latent space captures essential information

**Applications**:
- State estimation (latent = true state)
- Skill discovery (cluster latent space)
- Transfer learning (pre-trained encoders)

**Beta-VAE**: Encourages disentangled representations where each latent dimension controls independent factor of variation.

## Continuous Learning in Real World

Deployed robots must improve from ongoing experience.

### Online Adaptation

Adjusting behavior based on immediate feedback:

**Parameter Adaptation**:
- Update controller gains based on tracking error
- Adjust grasping forces based on slip detection
- Tune locomotion for different terrains

**Example**: Walking on sand vs concrete
- Detect increased slippage on sand
- Increase foot forces and reduce speed
- Update gait pattern for better stability

**Methods**:
- Gradient descent on recent experiences
- Bayesian updating of parameter distributions
- Meta-learning for fast adaptation

### Lifelong Learning

Accumulating knowledge over extended operation:

**Catastrophic Forgetting Problem**:
- Neural networks forget old tasks when trained on new ones
- Weights optimized for task A get overwritten learning task B

**Solutions**:
- **Elastic Weight Consolidation**: Protect important weights from change
- **Progressive Neural Networks**: Add new capacity for new tasks
- **Experience Replay**: Rehearse old tasks while learning new ones
- **Modular Networks**: Separate modules for different tasks

**Goal**: Robot that gets better over weeks/months/years of deployment.

### Curriculum Learning

Structuring learning from easy to hard:

**Natural Curriculum**:
- Start in controlled environment
- Gradually increase difficulty
- Add distractions and variations
- Eventually deploy in full complexity

**Example**: Grasping training
1. Week 1: Static objects, good lighting
2. Week 2: Moving objects on conveyor
3. Week 3: Cluttered scenes with occlusions
4. Week 4: Variable lighting and fast motion

This mirrors how humans learn - crawl before walking before running.

### Active Learning

Robot decides what to learn next:

**Uncertainty Sampling**:
- Identify situations where robot is uncertain
- Request human demonstration for those cases
- Focus learning on most informative examples

**Curiosity-Driven Exploration**:
- Seek novel or surprising experiences
- Explore areas of state space not yet understood
- Intrinsic motivation for learning

Reduces labeling burden by querying human only when needed.

## Transfer Learning Across Tasks

Leveraging knowledge from one task to accelerate learning on related tasks.

### Multi-Task Learning

Train single network on multiple tasks simultaneously:

**Shared Representation**:
- Common visual encoder for all tasks
- Task-specific output heads
- Shared features benefit all tasks

**Example**: Manipulation skills
- Grasping, pushing, pulling share visual processing
- Different action decoders for each skill
- Learning one task improves others

### Zero-Shot Transfer

Perform new tasks without additional training:

**Through Language Grounding**:
- Foundation models understand new instructions
- "Pick up the blue block" works even if never trained
- Compositional generalization from primitives

**Example**: RT-2 robot
- Trained on diverse tasks with language labels
- Can follow novel instructions like "move banana to pot"
- Combines understanding of "move", "banana", "pot"

### Few-Shot Adaptation

Quickly learn from handful of examples:

**Meta-Learning (MAML)**:
- Train on distribution of tasks
- Learn initialization that adapts quickly
- 5-10 examples sufficient for new task

**Applications**:
- Adapting to new object categories
- Adjusting to different robot embodiments
- Personalizing to user preferences

## Domain Adaptation

Transferring knowledge across different environments or embodiments.

### Sim-to-Real Adaptation

Fine-tuning simulation-trained policies:

**Residual Learning**:
- Keep simulation policy
- Learn additive correction in real world
- Preserves simulation knowledge while fixing discrepancies

**Approaches**:
- Online RL fine-tuning (1-2 hours real-world data)
- Supervised learning from human corrections
- Automatic sim parameter tuning based on real data

### Cross-Robot Transfer

Sharing skills across different robot platforms:

**Universal Policy**:
- Train on multiple robot types
- Condition on robot morphology
- Single policy controls different embodiments

**Challenges**:
- Different kinematics and dynamics
- Varied sensor configurations
- Capability differences (3 vs 7 DOF arms)

Foundation models may enable better transfer through abstract task understanding.

### Environmental Adaptation

Adjusting to new locations or conditions:

**Domain Randomization Benefits**:
- Training with varied environments
- Robust to appearance changes
- Works in new locations without retraining

**Online Calibration**:
- Quick tuning upon deployment
- Adjust to local lighting, surfaces
- Adapt to specific operating conditions

## Learning Efficiency Strategies

Maximizing learning from limited data:

### Data Augmentation

Artificially expanding training dataset:

**Image Augmentations**:
- Crops, rotations, color jittering
- Simulated occlusions
- Synthetic variations

**Trajectory Augmentations**:
- Time warping
- Noise injection
- Trajectory reversal (for symmetric tasks)

Augmentation provides implicit regularization and improves generalization.

### Hindsight Experience Replay

Learning from failures by relabeling goals:

**Idea**: Even failed attempts achieved something
- Tried to pick cup, knocked it over
- Relabel: "Goal was to knock over cup"
- Learn from accidental achievements

Dramatically improves sample efficiency in sparse-reward tasks.

### Model-Based Learning

Learning dynamics model to simulate experience:

**Dyna Architecture**:
- Learn environment model from real experience
- Generate synthetic experience with model
- Train policy on both real and synthetic data

**Benefits**:
- Extract more learning from limited real data
- Can plan using learned model
- Safer exploration (simulate risky actions)

**Challenges**:
- Model errors compound over planning horizon
- Difficult to model complex contact dynamics

## Measuring Learning Progress

Evaluating whether robots are improving:

### Performance Metrics

**Task Success Rate**: Percentage of successful task completions
**Completion Time**: How quickly task is accomplished
**Sample Efficiency**: Amount of data required to learn
**Generalization**: Performance on unseen variations

### Learning Curves

Plotting performance vs. experience:
- Steep initial improvement
- Diminishing returns over time
- Plateaus indicate learning limits

Compare different approaches on same tasks.

### Robustness Testing

Evaluation beyond nominal conditions:
- Vary lighting, object positions
- Add distractor objects
- Introduce sensor noise
- Test edge cases and failures

Robust systems maintain performance across variations.

## Conclusion

Learning enables robots to acquire skills beyond what can be explicitly programmed. Demonstration learning leverages human expertise. Self-supervised learning extracts knowledge from unlabeled data. Continuous adaptation allows improvement through deployment. Transfer learning shares knowledge across tasks and embodiments.

The combination of these approaches - learning from demonstrations, self-supervision, and online adaptation - creates robots that improve with experience rather than remaining static after deployment.

Next, we explore how language and multi-modal AI enable more natural human-robot interaction and reasoning.

---

**Next Chapter**: [Natural Language and Multimodal AI →](/chapter9)