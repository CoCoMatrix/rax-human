# Color candidate repair

Independent child of r9-ssik-speed-r2-transition-a15 (8284854e64bb5d628ea4d7d37b283f165992bf29).

Only perception and version identity changed. Wider green proposals include hue32..34; aggregate saturation-weighted surface hue scores assign green/yellow using absolute evidence, with an unknown band. Pixels assigned to a confident green region are excluded from yellow candidates. No forced color quota or relative-color ranking. White/low-saturation decoration is excluded from color identity.

Color fill is an outline/solidity/fill score term rather than an independent 45% veto. Existing area/size/aspect bounds, depth-height candidate filter, interior sampling, native depth binding and task/motion checks remain. HSV_RANGES consumers outside the initial component detector remain unchanged.

Failure frame f4593832d70648b599e945763db732f1 reproduced old green-candidate failure, then passed all four color selections plus native saved-depth scene localization. 25 targeted color/depth tests passed. RGB timing median 11.46ms vs old rejected-frame 6.92ms; native saved-depth scene replay 0.152s. These are offline measurements, not hardware task success or complete path preflight. No Gemini or robot motion used. Current source and frozen versions preserved.
