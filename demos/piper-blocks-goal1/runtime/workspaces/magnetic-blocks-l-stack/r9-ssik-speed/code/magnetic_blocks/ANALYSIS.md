# v0.2.2 offline implementation and limits

See README.md for the implemented observation reuse, qualitative support predicates,
optional vision, compact reports, timing instrumentation and green endpoint recovery.
Previous analysis is archived in ANALYSIS-v0.2.1.md and ANALYSIS-v0.2.md.
TARGET_IMAGE_ANALYSIS.md remains analysis only; this controller retains the three-block L task.

The five R8 image replay results are ORDINARY-REPLAY.json in the verification bundle.
They are historical observations, not fresh physical evidence. Green release and retreat
have synthetic recovery tests, but no new physical qualification in this change.
Initial empty transitions remain separate because combining the first two would exceed
the existing 60 degree joint-step guard (88.833 degrees). No guard is relaxed.
Timing fields are measurements of the software/receipt chain, not direct motor telemetry.
No live speedup or new-layout success rate is claimed before a new physical test.
