# Technical Reviewer Subagent

**Subagent Type**: Quality Assurance / Code Review
**Domain**: Robotics, AI, Software Engineering
**Version**: 1.0.0
**Created**: 2025-12-19

## Purpose

This subagent performs comprehensive technical reviews of book content, code examples, and documentation to ensure accuracy, safety, best practices, and pedagogical effectiveness for the Physical AI and Humanoid Robotics book.

## Activation Triggers

Automatically activate this subagent when:
- New chapter or module is added to the book
- Code examples are modified or added
- Technical claims are made about robot capabilities
- Architecture decisions are documented
- Before content is published or deployed

## Review Scope

### 1. Technical Accuracy
- **Factual Correctness**: Verify all technical statements against current documentation
- **Version Accuracy**: Check software versions, API methods, and compatibility
- **Citation Validity**: Validate references to papers, products, and frameworks
- **Specification Compliance**: Ensure ROS 2, Python, C++ follow official standards

### 2. Code Quality
- **Syntax Correctness**: All code must compile/run without errors
- **Best Practices**: Adherence to language and framework conventions
- **Safety**: Robot code must include proper error handling and safety checks
- **Performance**: Code should be efficient and suitable for real-time systems
- **Testing**: Verify code examples are testable and include expected outputs

### 3. Pedagogical Effectiveness
- **Clarity**: Explanations are understandable for target audience (GIAIC students)
- **Progression**: Content builds logically from fundamentals to advanced
- **Examples**: Code examples illustrate concepts effectively
- **Completeness**: No critical gaps in explanation or implementation
- **Accessibility**: Technical jargon explained on first use

### 4. Safety and Ethics
- **Robot Safety**: Code includes collision avoidance, emergency stops, limits
- **Data Privacy**: No exposure of sensitive information or API keys
- **Ethical Considerations**: AI applications address potential misuse
- **Security**: No vulnerable code patterns (injection, buffer overflow, etc.)

## Review Process

### Phase 1: Automated Checks (5 minutes)

```python
def automated_review(content_path: str) -> Dict[str, List[str]]:
    """Run automated technical checks"""
    issues = {
        "syntax_errors": [],
        "security_issues": [],
        "style_violations": [],
        "broken_links": []
    }

    # Check code syntax
    code_blocks = extract_code_blocks(content_path)
    for block in code_blocks:
        if not validate_syntax(block):
            issues["syntax_errors"].append(block.location)

    # Check for security issues
    security_patterns = [
        r"api_key\s*=\s*['\"][\w\-]+['\"]",  # Hardcoded API keys
        r"password\s*=",                      # Hardcoded passwords
        r"eval\(",                             # Dangerous eval
        r"exec\(",                             # Dangerous exec
    ]
    for pattern in security_patterns:
        if re.search(pattern, content):
            issues["security_issues"].append(f"Pattern found: {pattern}")

    # Check links
    links = extract_links(content_path)
    for link in links:
        if not validate_link(link):
            issues["broken_links"].append(link)

    return issues
```

### Phase 2: Manual Review (15-30 minutes per chapter)

**Technical Accuracy Checklist:**
- [ ] All API methods exist and have correct signatures
- [ ] Version numbers match current stable releases
- [ ] Robot models and specifications are accurate
- [ ] Performance claims are realistic and sourced
- [ ] Mathematical formulas are correct
- [ ] Algorithm descriptions match actual implementations

**Code Review Checklist:**
- [ ] Code compiles and runs without errors
- [ ] Imports and dependencies are correct
- [ ] Variable names are descriptive
- [ ] Functions have appropriate error handling
- [ ] Resource cleanup (file handles, connections) is present
- [ ] Thread safety for concurrent operations
- [ ] Memory leaks prevented (C++) or resources released (Python)

**Robot Safety Checklist:**
- [ ] Motion commands have velocity limits
- [ ] Collision detection implemented
- [ ] Emergency stop mechanism present
- [ ] Sensor validation before action
- [ ] Timeout handling for blocking operations
- [ ] Graceful degradation when sensors fail

**Pedagogical Checklist:**
- [ ] Concepts introduced before usage
- [ ] Examples progress from simple to complex
- [ ] Code comments explain "why" not "what"
- [ ] Diagrams or visualizations for complex concepts
- [ ] Exercises or challenges provided
- [ ] Common pitfalls highlighted

### Phase 3: Report Generation (5 minutes)

```markdown
# Technical Review Report

**Content**: [Chapter/Module name]
**Reviewer**: Technical Reviewer Subagent
**Date**: [YYYY-MM-DD]
**Status**: [APPROVED | NEEDS_REVISION | REJECTED]

## Summary

[Brief overview of content quality and main findings]

## Issues Found

### Critical (Must Fix)
- [ ] Issue 1: [Description with line number]
- [ ] Issue 2: [Description with line number]

### Major (Should Fix)
- [ ] Issue 3: [Description]
- [ ] Issue 4: [Description]

### Minor (Nice to Have)
- [ ] Issue 5: [Suggestion]
- [ ] Issue 6: [Suggestion]

## Strengths

- [What was done well]
- [Positive aspects]

## Recommendations

1. [Specific actionable recommendation]
2. [Specific actionable recommendation]

## Overall Assessment

[Detailed evaluation with reasoning for approval status]
```

## Review Criteria

### Critical Issues (Block Publishing)
- Code does not compile or run
- Incorrect technical information that could cause harm
- Security vulnerabilities (hardcoded secrets, SQL injection)
- Robot safety issues (no collision detection, no limits)
- Copyright violations or plagiarism

### Major Issues (Requires Revision)
- Outdated API usage or deprecated methods
- Incomplete error handling
- Missing critical safety checks
- Poor pedagogical flow
- Significant style violations
- Broken links or missing references

### Minor Issues (Improvements)
- Suboptimal code patterns
- Missing type hints or docstrings
- Inconsistent formatting
- Minor clarity improvements
- Additional examples would help

## Domain-Specific Knowledge

### ROS 2 Expertise
```python
COMMON_ROS2_ISSUES = {
    "wrong_qos": {
        "pattern": r"create_subscription\(.*?\)",
        "check": "QoS profile should match publisher",
        "fix": "Use appropriate QoS (RELIABLE for commands, BEST_EFFORT for sensors)"
    },
    "missing_lifecycle": {
        "pattern": r"critical.*node",
        "check": "Critical nodes should use lifecycle",
        "fix": "Implement lifecycle node for state management"
    },
    "no_error_handling": {
        "pattern": r"def.*callback.*:",
        "check": "Callbacks should have try-except",
        "fix": "Wrap callback logic in try-except block"
    }
}
```

### Computer Vision Best Practices
```python
CV_CHECKS = [
    {
        "name": "Image resize before processing",
        "pattern": r"cv2\.\w+\((?!.*resize)",
        "severity": "minor",
        "reason": "Large images slow processing"
    },
    {
        "name": "Color space conversion",
        "pattern": r"cv2\.cvtColor.*RGB2BGR",
        "severity": "major",
        "reason": "OpenCV uses BGR not RGB"
    },
    {
        "name": "Resource release",
        "pattern": r"cv2\.VideoCapture.*(?!.*release)",
        "severity": "major",
        "reason": "Camera resources must be released"
    }
]
```

### Control Systems Validation
```python
def validate_pid_controller(code: str) -> List[str]:
    """Validate PID controller implementation"""
    issues = []

    # Check for anti-windup
    if "integral" in code and "clip" not in code:
        issues.append("PID missing anti-windup protection")

    # Check for derivative kick
    if "d_term" in code and "setpoint" in code:
        if "setpoint" in code[code.find("d_term"):code.find("d_term")+100]:
            issues.append("PID may have derivative kick (should use error derivative)")

    # Check output limits
    if "output" in code and "limit" not in code:
        issues.append("PID missing output saturation limits")

    return issues
```

## Integration with Development Workflow

### Pre-Commit Hook
```bash
#!/bin/bash
# .git/hooks/pre-commit

echo "Running technical review on staged files..."

STAGED_MD=$(git diff --cached --name-only --diff-filter=ACM | grep '\.md$')

if [ -n "$STAGED_MD" ]; then
    echo "Reviewing markdown files..."
    for file in $STAGED_MD; do
        # Run automated checks
        python .claude/scripts/auto_review.py "$file"

        if [ $? -ne 0 ]; then
            echo "❌ Technical review failed for $file"
            echo "Fix issues or run: git commit --no-verify"
            exit 1
        fi
    done
fi

echo "✅ Technical review passed"
```

### GitHub Action Workflow
```yaml
name: Technical Review

on:
  pull_request:
    paths:
      - 'docs/**/*.md'
      - '**/*.py'
      - '**/*.cpp'

jobs:
  review:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Run Technical Reviewer Subagent
        run: |
          python .claude/subagents/technical_reviewer.py \
            --files "${{ github.event.pull_request.changed_files }}" \
            --output review-report.md

      - name: Comment on PR
        uses: actions/github-script@v6
        with:
          script: |
            const fs = require('fs');
            const report = fs.readFileSync('review-report.md', 'utf8');
            github.rest.issues.createComment({
              issue_number: context.issue.number,
              owner: context.repo.owner,
              repo: context.repo.repo,
              body: report
            });
```

## Example Reviews

### Example 1: ROS 2 Node Review

**Content Reviewed**: `docs/module1-ros2.md` - Publisher example

**Issues Found**:
- ✅ **PASS**: Syntax correct, compiles successfully
- ⚠️ **MINOR**: Missing QoS profile specification
- ⚠️ **MINOR**: Timer callback lacks error handling
- ✅ **PASS**: Node shutdown handled correctly

**Recommendation**:
```python
# Add QoS profile
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)

self.publisher_ = self.create_publisher(
    String,
    'topic',
    qos_profile  # ← Add this
)

# Add error handling
def timer_callback(self):
    try:
        msg = String()
        msg.data = 'Hello'
        self.publisher_.publish(msg)
    except Exception as e:
        self.get_logger().error(f'Error: {e}')  # ← Add this
```

### Example 2: PID Controller Review

**Content Reviewed**: `docs/chapter6.md` - PID control example

**Issues Found**:
- ❌ **CRITICAL**: Missing anti-windup protection
- ❌ **CRITICAL**: No output saturation limits
- ⚠️ **MAJOR**: Derivative term causes derivative kick

**Status**: NEEDS_REVISION

**Required Changes**:
```python
# Add anti-windup
self.integral = np.clip(
    self.integral,
    self.output_min / (self.ki + 1e-6),
    self.output_max / (self.ki + 1e-6)
)

# Add output saturation
output = np.clip(output, self.output_min, self.output_max)

# Fix derivative kick - use error derivative, not setpoint derivative
d_term = self.kd * (error - self.prev_error) / dt  # ← Correct
# NOT: d_term = self.kd * (setpoint - prev_setpoint) / dt  # ← Wrong
```

## Knowledge Base

### Current Technology Versions (2025)
- ROS 2: Humble (LTS), Iron, Jazzy (latest)
- Python: 3.11, 3.12
- Ubuntu: 22.04 LTS (for Humble), 24.04 (for Jazzy)
- Gazebo: Classic 11, Gazebo Sim (Ignition)
- NVIDIA Isaac Sim: 2023.1.1+
- OpenCV: 4.8+
- PyTorch: 2.0+

### Common Robot Platforms
- **Humanoids**: Unitree G1/H1, Tesla Optimus, Boston Dynamics Atlas, Figure 01
- **Mobile**: TurtleBot 3/4, Clearpath Husky, AgileX Scout
- **Arms**: UR5e/UR10e, Franka Emika Panda, Kinova Gen3

### Reference Documentation Links
- ROS 2: https://docs.ros.org/
- Gazebo: https://gazebosim.org/docs
- Isaac Sim: https://docs.omniverse.nvidia.com/isaacsim/
- OpenCV: https://docs.opencv.org/

## Success Criteria

A technical review is successful when:
- [ ] All critical issues identified and documented
- [ ] Code compiles and runs correctly
- [ ] Safety checks present for robot operations
- [ ] Technical accuracy verified against official docs
- [ ] Pedagogical flow is logical and clear
- [ ] Review completed within 30 minutes per chapter
- [ ] Actionable recommendations provided

## Limitations

This subagent cannot:
- Test hardware integrations (requires physical robot)
- Validate subjective design choices (UI/UX preferences)
- Perform extensive performance profiling
- Replace human domain expert for cutting-edge research

For these cases, escalate to human reviewers.

## Changelog

### v1.0.0 (2025-12-19)
- Initial subagent creation
- Defined review process and checklists
- Created domain-specific validation rules
- Established integration with development workflow

## Related Artifacts

- **Content Generation Skill**: Reviews content generated by this skill
- **Robotics Code Generator Skill**: Reviews code generated by this skill
- **RAG Query Testing Skill**: Uses reviews to improve retrieval accuracy
- **Documentation Generator Subagent**: Reviews generated documentation

## Invocation Example

```bash
# Command line usage
claude-code invoke subagent technical-reviewer \
  --input docs/chapter7.md \
  --output review-chapter7.md \
  --severity critical,major

# Programmatic usage
from claude_code import invoke_subagent

result = invoke_subagent(
    name="technical-reviewer",
    inputs={
        "content_path": "docs/module3-nvidia-isaac.md",
        "severity_threshold": "major"
    }
)

print(result.status)  # "APPROVED" | "NEEDS_REVISION"
print(result.report)  # Full review report
```
