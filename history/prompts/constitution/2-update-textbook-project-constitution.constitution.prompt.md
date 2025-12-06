---
id: 2
title: Update-textbook-project-constitution
stage: constitution
date: 2025-12-06
model: Claude 3.5 Sonnet
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, update, textbook, ai, robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
  - specs/1-textbook-project-structure/plan.md
tests: []
---

# Update Textbook Project Constitution

## User Input

```
Project: AI/Spec-Driven Book on Physical AI & Humanoid Robotics
Platform: Docusaurus → GitHub Pages
Tools: Spec-Kit Plus + Claude Code

Core Principles:
- Educational clarity (beginner → advanced)
- Technical accuracy (AI, robotics, cloud-native)
- Practical outcomes (hands-on examples)
- Ethical responsibility (safety & responsible AI)

Standards:
- Content must be original and source-traceable
- Code examples must run successfully
- Robotics concepts must align with ROS2, URDF, and real control systems
- Agentic AI content must reflect current production practices
- Citations: official docs or reputable research
- Tone: mentor → student, respectful, direct

Structure:
- Each chapter starts from a specification
- Include objectives, examples, steps, and a final exercise
- Every technical module includes:
  - Inputs, outputs, architecture, code, failure modes, safety notes

Constraints:
- Minimum total length: 20,000-35,000 words
- At least 1 working demo (simulation acceptable)
- Images/figures must include alt text

Success Criteria:
- A student can follow and produce a working robotics or AI system
- Book deploys to GitHub Pages with no build errors
- All code reproducible and free of hallucinations
```

## Response

The project constitution has been successfully updated for the Physical AI & Humanoid Robotics textbook project, with all dependent templates synchronized.

### Constitution Updates
- Version updated from 1.1.0 to 1.2.0 (MINOR: new educational focus and requirements)
- Core principles updated to reflect educational textbook focus:
  - Educational Clarity (beginner → advanced)
  - Technical Accuracy (AI, robotics, cloud-native)
  - Practical Outcomes (hands-on examples)
  - Ethical Responsibility (safety & responsible AI)
  - Reproducible Content
  - Open Scientific Communication
- Added new sections: Educational Standards, Content Quality Requirements, Technical Requirements
- Updated project scope and constraints to match textbook requirements

### Template Synchronization
- **plan-template.md**: Updated Constitution Check section to align with new educational principles
- **spec-template.md**: Updated functional requirements and success criteria to match textbook requirements
- **tasks-template.md**: Updated all phases to reflect textbook/chapter structure rather than user story structure
- **specs/1-textbook-project-structure/plan.md**: Updated constitution check section to align with new principles

### Key Changes to Templates
- Changed all "User Story" references to "Chapter" in tasks template
- Updated validation requirements to focus on educational content quality
- Modified success criteria to emphasize reproducible learning outcomes
- Ensured all templates support Docusaurus-based textbook development

## Outcome

The constitution and templates now properly reflect the specific requirements for an educational textbook project focused on Physical AI & Humanoid Robotics, emphasizing educational clarity, technical accuracy, practical outcomes, and ethical responsibility. All templates have been updated to support specification-driven textbook development with the required structure and quality standards.