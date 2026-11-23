# C-speed 0.1

Independent runtime from C034; SDK80%, nominal joint cap32deg/s with0.85 tracking reserve gives27.2deg/s; acceleration40deg/s2 unchanged. Per-call600mm and per-joint120deg scope; workspace, joint limits, contact/rest/feedback tolerances unchanged. Prepare profile unchanged. Prior runtime/controller/frozen archives untouched.

20 complete task geometry/native/batch replay cases;421 regression tests+52 subtests. Offline production bridge/profile comparison:60deg6.83s to4.37s;10deg unchanged1.43s due acceleration.120deg accepted by trajectory compiler,121deg refused. No hardware motion. Live speed remains to be measured.
