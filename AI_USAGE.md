# Use of AI
Due to unexpected personal commitments, I had only a few days to complete this task, so I relied on AI assistance (Claude) more than I originally planned. I used it as a tutor to understand new concepts, to challenge and validate my own ideas and to write parts of the code, as listed in the table below.
Every piece of generated code was reviewed, built, and tested by me before being committed.

Despite the AI assistance, I know the codebase thoroughly: I can walk through any module and justify each design choice.

| Tool | What it was used for |
|---|---|
| Claude (claude.ai) | **Approach.** Suggested using VoxelGrid and planar ICP to align the scans. I noticed that ICP from scan to scan drifted too much and decided to align each scan against a sliding window of keyframes instead. It then suggested starting ICP from a constant-velocity prediction, so that it starts close to the solution.|
| Claude | **Development environment.** Helped me write compose.yaml and set up WSL2 on Windows 10. |
| Claude | **Code.** Helped me with syntax, library methods and code structure, to keep the project clean, it wrote parts of the C++ code and the Python scripts to launch the node and to analyse the timing statistics.|
| Claude | **Debugging.** Explained the errors I ran into and proposed fixes. |
| Claude | **Concepts and maths.** Explained the maths behind each stage, in particular ICP, rotations and quaternions, coordinate frames and TF, and why the quaternion stopped being normalized. |
| Claude | **Tuning and experiments.** It helped me interpret the test results and suggested repeating the same run multiple times to tell whether the slow scans came from the host or from the algorithm|
| Claude | **Refactoring.** Helped me by proposing the refactoring. I reviewed it and decided which changes to make; Claude implemented them. I then built, tested and committed each one. |
| Claude | **Git workflow.** I was not used to committing this often: Claude showed me how to split the work into small commits, each doing one thing, and drafted the commit messages, which I reviewed before committing. |
| Claude | **README and this file.** Drafted the README and a first version of this table, which I then reviewed and revised. |
| Claude | **Report.** Wrote an outline of the report. |
