---
name: Task
about: Template for creating tasks
title: ''
labels: ''
assignees: ''
body:
- type: dropdown
    id: dropdown-0
    attributes:
      label: Testing Criteria
      description: What must be completed for this to be merged in?
      options:
        - No testing required - Not Robot Code
        - Tested in Sim
        - Tested on the Real Robot
        - Other (Write Below)
      default: 1
---

**Description**
A clear and concise description of the task.

**Steps to Complete**
- [ ] Step 1
- [ ] Step 2
- [ ] Step 3
