# r9-ssik-speed-r2-transition

Base: original r2 commit b77c9986424cbe2e84af9f67ac0895c93cb3f1c3.

Only two behavioral changes:
1. After confirmed green release, combine green empty retreat and yellow hover into one checked blended waypoint wire. Keep yellow entry and all jaw boundaries. Native collision includes all placed objects; individual phase geometry checks use the actual wire. Failed preview may use original checked segments, never redispatch ambiguous motion.
2. Prefer complete final yellow retreat. If the finite original direct/routed withdrawal portfolio has no valid path under full IK, accept checked placement plus checked release and stop open in place. Green never skips retreat. Timeout, transport, placement or release failures remain failures. Report released_in_place explicitly; no automatic Prepare.

r2 IK search, candidate scheduling, backend, speed, tolerance and budget are unchanged. No r3 preflight optimizations or multi-pose IK interface. Original r2 and frozen sources remain unchanged. Offline tests are not live acceptance.
