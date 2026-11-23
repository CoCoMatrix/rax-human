# C-speed0.2.4: robust initial block center depth

Only perception.py and scene_checks.py change task behavior. C-speed0.2.3 and existing frozen versions are preserved; host motion/collision code and all speed/tolerance settings are unchanged.

Source: web-main-20261113-181732-276acd, frame e033c1d295a94668a4425d7d0759e229. Original same-frame RGB/native depth replay reproduces Surface z spread0.0232m on RED center. Colored red top is78.043..79.122mm, center has one103.227mm sample. This was not a missing color, model, network or motion failure.

Normal consistent center measurements retain the old result. For inconsistent center windows, validate the identity/calibration/samples of ALL original measurements first, then accept a strict majority (at least3) agreeing with measured colored top within existing8mm agreement and12mm spread. Height combines actual accepted center and colored top samples by median. Record discarded pixels and raw spread. These samples are local/overlapping, not proof of independent calibration accuracy.

Without such a center majority, reuse a same-frame flat colored top patch (existing4mm spread and10px coverage). If that patch is insufficient, one bounded re-selection from valid colored top-depth windows in the SAME frame is allowed; actual pixel service measurements must again establish flatness. No filled or invented depth. RGB center-ray/top-height intersection remains explicitly a geometric estimate, not center depth measurement. Identity/staleness/calibration/invalid-point errors never enter numeric outlier recovery. No mandatory Gemini or new observation.

Original failure now completes all3 colors in6 pixel calls (normal complete-scene count), no extra model/image query: red79.122mm, orange78.670mm, green82.044mm. Recorded-frame clock is pinned ONLY in offline replay. Raw depth and RGB are unchanged, native Cameras.pixel is used, socket access forbidden. New tests cover high/low outlier, unchanged normal result, no-majority, reuse, reselection/refusal, frame/calibration/sample/pixel/nonfinite failures.568 tests+52subtests pass. No robot motion started. Offline evidence is not physical task qualification.
