---
name: implement
description: "Implement a piece of work based on a spec or set of tickets."
disable-model-invocation: true
---

Implement the work described by the user's confirmed Plan, spec, or agent-ready ticket. This Skill is owned by the `execute` Agent.

## Before implementation

- Read the confirmed Plan or ticket and record the Git fixed point.
- Inspect the worktree and preserve unrelated user changes.
- Classify the work as D1-D5. If D3-D5 has no confirmed Plan, stop and route to `planner` rather than guessing.
- State the public interface or test seam. The seam is the observation surface for callers and tests.

## Delegation

Delegate only bounded work that does not require re-planning. Every Worker task must include:

```text
objective
difficulty
allowed_files
forbidden_changes
test_seam
targeted_commands
done_when
```

- Use `worker-routine` for D1-D2 work.
- Use `worker-expert` for D3-D5 work after Execute settles the interface and semantics.
- Use `explore` and `scout` for facts only.
- Do not overlap Worker file scopes. Inspect every Worker's actual diff.

## TDD and verification

Use `/tdd` where possible, at the confirmed seam. Run C++ compilation or typechecking regularly, targeted tests during implementation, and the full test suite once at the end. For this ROS 2 package, the normal final commands are:

```bash
colcon build --packages-select detector --symlink-install
colcon test --packages-select detector
colcon test-result --verbose
```

If HighGUI needs a display, report the environment constraint explicitly instead of hiding it with a workaround.

## Review and commit

After final verification, inspect the complete diff and commit the implementation to the current branch. Then use `/code-review`, which launches parallel Standards and Spec reviews through the read-only `review` Agent. Fix actionable findings, run relevant regression checks, and create a separate follow-up fix commit when needed.

Never push or use destructive Git operations such as `git reset --hard`, `git clean`, or forceful checkout/restore.
