---
name: Task
about: Template for creating tasks
title: ''
labels: ''
assignees: ''

---

**Description**
A clear and concise description of the task.

**Steps to Complete**
- [ ] Step 1
- [ ] Step 2
- [ ] Step 3

**Completion Criteria**
- type: dropdown
    id: dropdown-0
    attributes:
      label: Completion Criteria
      description: What must be completed for this to be merged in?
      options:
        - No testing required
        - Tested in Sim
        - Tested on the Real Robot
        - Other (Write Below)
      default: 1
