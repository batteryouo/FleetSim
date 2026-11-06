## Code Style

All repository content must be in English — not just code. This includes comments, identifiers,
docstrings, commit messages, and any documentation/notes written into the repo. User-facing UI
strings may be localized when appropriate; do not use Chinese in repository content merely because
the discussion that produced it was in Chinese.

### General
- Compact style: avoid unnecessary blank lines
- Blank lines only between logically distinct sections
- No trailing whitespace

### C / C++
- Indent: 2 spaces (no tabs)
- Keep control-flow braces on the same line and keep `else` beside the preceding closing brace:
  `if (condition) { ... } else { ... }`
- No blank line after opening brace or before closing brace

### Python
- Indent: 2 spaces (no tabs, overrides PEP8 default)
- No blank lines between short related functions
- Docstrings only for public APIs; skip for obvious internal helpers

## Documentation & Comments

Documentation should preserve knowledge a future developer needs to make correct decisions —
not record the author's implementation history or reasoning process. Put each piece of
knowledge at the narrowest scope that remains useful, and no closer:

```
local, easy-to-break knowledge -> inline comment
regression risk                -> test
concrete future work           -> TODO (actionable, with a removal condition)
cross-cutting design decision  -> architecture doc / ADR
implementation chronology      -> git history
```

**Docstrings describe the current contract only** — purpose, inputs, outputs, side effects,
exceptions, invariants. A docstring is not a git log and not a diary of the conversation that
produced the code.

Do **not** put in a docstring or comment:
- Dates, "as of YYYY-MM-DD", changelogs, "replaces X which replaced Y"
- References to chat/discussion sessions ("see chat discussion", "per this session's...")
- Design rationale or tradeoffs spanning multiple files/components
- More than one or two cross-references to other functions/files/docs
- Bug/incident history, speculative future work

Bad:
```python
"""Real-Gazebo validation for MPPIPlanner (see docs/project-brief.md, 2026-07-20 architecture
pivot). Supersedes intent_dwa_ros_test.py, which validated IntentDWAPlanner -- the whole
exhaustive-grid + fuzzy-intent-scoring design that class implemented has been retired..."""
```
Good:
```python
"""Drives a real scenario and logs per-tick planning time for MPPIPlanner."""
```

Inline comments explain **why**, not **what** — narrating what a line does in English prose is
noise; a comment earns its place only when the code would otherwise look wrong, invite an
incorrect "simplification," or hide a non-obvious invariant. Before adding one, ask: if this
comment were removed, could a competent reader still understand the code correctly? If yes,
skip it. A warning comment ("DO NOT CHANGE") must state the invariant, what breaks, and when
it's safe to change — not just express alarm.

When a behavior exists because of a previously discovered bug: prefer a regression test over a
comment. Add a short inline comment only if the fixed code would otherwise look wrong; don't
write the incident's history into it.

This applies to all comments/docstrings written into this codebase, regardless of whether the
change originated from a chat session, an AI assistant, or manual editing — don't preserve or
add historical/dated narration just because earlier code in the file already has it.
