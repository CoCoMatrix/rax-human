# Adaptive orange entry
Source v0.2.10 remains unchanged. Failure occurred before grouped entry: nominal top+215mm downward pose had no IK; fixed65% joint split also exceeded60deg for this scene.
Generate hover candidates from measured block top, installed tip length and workspace;215mm remains a preference, not required. Search down in10mm numerical increments within the derived interval. Tilted target is20mm above each hover. Actual joint delta determines split fraction;60deg cap remains. Alternate heights must pass empty envelope, table/blocks, nativeCAD/workspace and full insertion path. No intermediate images or model calls added.
Recorded failure scene now selects a feasible hover;198 regression tests passed. No live motion performed by repair.50% speed unchanged.
