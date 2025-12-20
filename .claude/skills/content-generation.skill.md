# Content Generation Skill

**Skill Type**: Content Creation
**Domain**: Educational Technical Writing
**Version**: 1.0.0
**Created**: 2025-12-19

## Purpose

This skill generates educational content for the Physical AI and Humanoid Robotics book following established style guidelines, technical accuracy standards, and pedagogical best practices.

## When to Use This Skill

- Adding new chapters or sections to the book
- Expanding existing content with more details
- Creating tutorial modules with code examples
- Writing technical explanations of robotics concepts
- Generating comparative analyses of technologies

## Input Requirements

```yaml
topic: string              # Main topic or concept to cover
depth: enum                # "introductory" | "intermediate" | "advanced"
word_count: number         # Target word count (default: 2000)
include_code: boolean      # Whether to include code examples (default: true)
target_audience: string    # e.g., "GIAIC AI students", "robotics beginners"
related_chapters: array    # List of related chapter numbers for cross-referencing
```

## Output Format

The skill produces markdown content with:
- Clear heading hierarchy (H2, H3, H4)
- Technical accuracy with current (2024-2025) information
- Code examples in Python, C++, or relevant languages
- Real-world applications and use cases
- Cross-references to related chapters
- Consistent terminology with existing book content

## Style Guidelines

### Writing Style
- **Clarity**: Technical but accessible language
- **Structure**: Logical progression from concepts to implementation
- **Examples**: Concrete, working code examples
- **Tone**: Educational and encouraging, not condescending
- **Length**: Paragraphs of 3-5 sentences, code blocks with comments

### Technical Standards
- Use current versions (ROS 2 Humble/Iron, Python 3.11+, Ubuntu 22.04)
- Cite specific robot models (Unitree G1, Tesla Optimus, Boston Dynamics Atlas)
- Reference actual frameworks (Isaac Sim, Gazebo, Unity ML-Agents)
- Include installation commands and dependency information
- Show complete, runnable code (not snippets)

### Content Structure
```markdown
# [Chapter/Section Title]

[Opening paragraph: Why this topic matters]

## Core Concepts

[Fundamental principles explained clearly]

## Technical Implementation

[How it works in practice]

### Code Example

[Working code with comments]

## Real-World Applications

[Companies, products, use cases]

## Challenges and Considerations

[Limitations, tradeoffs, future directions]

## Summary

[Key takeaways in bullet points]
```

## Example Usage

### Input
```yaml
topic: "Vision Transformers for Robot Perception"
depth: "intermediate"
word_count: 2500
include_code: true
target_audience: "GIAIC AI students with basic ML knowledge"
related_chapters: [7, 9]
```

### Expected Output
A 2500-word section covering:
1. What Vision Transformers are and why they matter for robotics
2. Architecture overview (patches, attention, positional encoding)
3. Python code example using timm or transformers library
4. Comparison with CNNs for robot vision tasks
5. Real-world applications (Tesla FSD, Unitree robots)
6. Challenges (computational cost, data requirements)
7. Cross-references to Chapter 7 (Perception) and Chapter 9 (Multimodal AI)

## Quality Checklist

Before considering content complete, verify:

- [ ] Technical accuracy: All facts, versions, and code are current
- [ ] Code validity: All code examples are syntactically correct and runnable
- [ ] Consistency: Terminology matches existing chapters
- [ ] Completeness: All required sections present (concepts, implementation, examples, applications)
- [ ] Readability: Appropriate for target audience, clear explanations
- [ ] References: Cross-links to related chapters included
- [ ] Practicality: Includes actionable information (installation, usage, commands)

## Integration with Book

### Terminology Consistency
Always use these standardized terms:
- "Physical AI" (not "embodied AI" or "physical intelligence")
- "Humanoid robots" (not "humanoid robotics" as noun)
- "Vision-Language-Action models" or "VLA" (not "VLM for robotics")
- "ROS 2" (not "ROS2" or "ROS 2.0")
- "Digital twin" (not "digital replica" or "virtual twin")

### Code Standards
- Python: PEP 8 style, type hints for function signatures
- C++: Modern C++17/20, snake_case for variables
- ROS 2: rclpy for Python examples, rclcpp for C++
- Comments: Explain "why" not "what"
- Imports: Grouped (standard library, third-party, local)

### Citation Format
When referencing other chapters:
```markdown
As discussed in [Chapter 7: Perception](/docs/chapter7), vision systems...

For more on digital twins, see [Module 2: Digital Twins](/docs/module2-digital-twin).
```

## Common Patterns

### Introducing New Concepts
```markdown
**[Concept Name]** is [concise definition]. In robotics, this enables [practical capability].

For example, [concrete example with numbers/specifics].
```

### Code Example Pattern
```python
# [Brief description of what this code does]
import relevant_library

def example_function(param: Type) -> ReturnType:
    """
    [Docstring explaining purpose]

    Args:
        param: [Parameter description]

    Returns:
        [Return value description]
    """
    # [Comment explaining key step]
    result = perform_operation(param)
    return result

# Usage example
output = example_function(input_value)
print(f"Result: {output}")
```

### Real-World Application Pattern
```markdown
**Company/Product Name**: [Brief description of application]
- **Technology Used**: [Specific frameworks/algorithms]
- **Impact**: [Quantifiable results or capabilities]
- **Current Status**: [Deployment stage, availability]
```

## Error Handling

If the skill encounters unclear requirements:
1. Generate content based on best interpretation
2. Mark uncertain sections with `<!-- TODO: Clarify [aspect] -->`
3. Provide alternative formulations in comments
4. Flag for human review

## Versioning and Updates

When book content standards change:
1. Update this skill document version number
2. Document changes in "Changelog" section
3. Regenerate affected content with new standards
4. Validate consistency across all chapters

## Changelog

### v1.0.0 (2025-12-19)
- Initial skill creation
- Established style guidelines for Physical AI book
- Defined input/output formats
- Created quality checklist

## Related Skills

- **RAG Query Testing Skill**: Validates content is properly indexed and retrievable
- **Code Example Generation Skill**: Specialized for complex robotics code
- **Technical Review Subagent**: Reviews generated content for accuracy

## Success Metrics

Content generated by this skill should achieve:
- ✅ 95%+ technical accuracy (verified by domain experts)
- ✅ Readability score of 10-12th grade level (Flesch-Kincaid)
- ✅ 100% working code examples (tested and validated)
- ✅ Consistent terminology across all sections
- ✅ Positive user feedback from GIAIC students
