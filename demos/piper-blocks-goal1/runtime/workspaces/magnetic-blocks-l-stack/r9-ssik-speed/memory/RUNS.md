# 探索记录

| Run | 结果 | 主模型 | 辅助视觉 | 总耗时 |
|---|---|---:|---:|---:|
| R1 | 动作前上游错误 | 1 | 0 | 30.024s |
| R2 | 动作前上游错误 | 3 | 0 | 56.955s |
| R3 | 受监督成功；最终现场确认 | 120/120完成 | 0 | 1797.952s |

R3首次底部并排已在约871s记录；绿色松爪退至300mm完成于1565.018s；现场确认在约1751s到达；最终新观测O67约1758.330s。全部以证据中的实际累计计时为准。
主模型请求耗时合计1522.717s，中位数9.267s，最大70.605s；这是请求墙钟之和，不等同纯推理时间。
最后API样式429是本地120次调用预算耗尽。原始RESULT仍保留agent_exit_code=1；物理成功依据现场确认、最终观测及Stop，与进程退出码分开记录。
受检查的跨会话library merge最终成功，commit 9c6cb4be81a94c47493f5462a190151a44b51df7，只贡献手动证据，没有晋升verified自动技能。

R1逐函数演练：909.492s，48次主请求（47完成，停止时最后一次未完成），Gemini0；8次开爪RPC28.051s，0次位姿/关节运动。张爪返回141.080s中112.488s为检查等待；用户停止，不计成功演练。见references/rehearsal-r1/REHEARSAL-REVIEW.json。

R2: user stopped after207.079s,11 main requests,0 Gemini,0 physical task actions. Remaining issue: repeated startup skill/source reads. Requested left-only closure before R3:8 native gripper calls,24.040s,final0.3mm; all arm joints0,right jaw2mm. Not a rehearsal success.


## R4 historical replay files (not live observations)
- references/rehearsal-r4/r4-approach-hover-01.json
- references/rehearsal-r4/r4-approach-hover-02.json
- references/rehearsal-r4/r4-approach-hover-inputs.json
- references/rehearsal-r4/r4-approach-mid-01.json
- references/rehearsal-r4/r4-approach-mid-inputs.json
- references/rehearsal-r4/r4-close-01.json
- references/rehearsal-r4/r4-close-inputs.json
- references/rehearsal-r4/r4-historical-orientation-analysis.json
- references/rehearsal-r4/r4-hover-01.json
- references/rehearsal-r4/r4-hover-02.json
- references/rehearsal-r4/r4-hover-03.json
- references/rehearsal-r4/r4-hover-04.json
- references/rehearsal-r4/r4-hover-05.json
- references/rehearsal-r4/r4-hover-inputs.json
- references/rehearsal-r4/r4-ik-01.json
- references/rehearsal-r4/r4-ik-02.json
- references/rehearsal-r4/r4-ik-03.json
- references/rehearsal-r4/r4-ik-04.json
- references/rehearsal-r4/r4-ik-05.json
- references/rehearsal-r4/r4-insert-01.json
- references/rehearsal-r4/r4-insert-inputs.json
- references/rehearsal-r4/r4-open-01.json
- references/rehearsal-r4/r4-open-inputs.json
- references/rehearsal-r4/r4-opening-preflight.json
- references/rehearsal-r4/r4-scene-01.json
- references/rehearsal-r4/r4-transition-01-inputs.json
- references/rehearsal-r4/r4-transition-01.json
- references/rehearsal-r4/r4-transition-02-inputs.json
- references/rehearsal-r4/r4-transition-02.json
- references/rehearsal-r4/r4-transition-plan.json
- references/rehearsal-r4/r4-trial-01.json
- references/rehearsal-r4/r4-trial-02.json
- references/rehearsal-r4/r4-trial-inputs.json
- references/rehearsal-r4/r4-verify-01.json
- references/rehearsal-r4/r4-verify-inputs.json
- references/rehearsal-r4/r4-wrist-depth-01.json
- runtime/frames/067340a2f76f4e65add5b680b2e23da3/head.jpg
- runtime/frames/067340a2f76f4e65add5b680b2e23da3/head.png
- runtime/frames/067340a2f76f4e65add5b680b2e23da3/left.jpg
- runtime/frames/067340a2f76f4e65add5b680b2e23da3/left.png
- runtime/frames/067340a2f76f4e65add5b680b2e23da3/meta.json
- runtime/frames/067340a2f76f4e65add5b680b2e23da3/right.jpg
- runtime/frames/067340a2f76f4e65add5b680b2e23da3/right.png
- runtime/frames/072c02d78643428b80779c051651294b/head.jpg
- runtime/frames/072c02d78643428b80779c051651294b/head.png
- runtime/frames/072c02d78643428b80779c051651294b/left.jpg
- runtime/frames/072c02d78643428b80779c051651294b/left.png
- runtime/frames/072c02d78643428b80779c051651294b/meta.json
- runtime/frames/072c02d78643428b80779c051651294b/right.jpg
- runtime/frames/072c02d78643428b80779c051651294b/right.png
- runtime/frames/1600d4b5aac840299a18431a3efd80f7/head.jpg
- runtime/frames/1600d4b5aac840299a18431a3efd80f7/head.png
- runtime/frames/1600d4b5aac840299a18431a3efd80f7/left.jpg
- runtime/frames/1600d4b5aac840299a18431a3efd80f7/left.png
- runtime/frames/1600d4b5aac840299a18431a3efd80f7/meta.json
- runtime/frames/1600d4b5aac840299a18431a3efd80f7/right.jpg
- runtime/frames/1600d4b5aac840299a18431a3efd80f7/right.png
- runtime/frames/1bf3ce52a8244001bd18698918f36cab/head.jpg
- runtime/frames/1bf3ce52a8244001bd18698918f36cab/head.png
- runtime/frames/1bf3ce52a8244001bd18698918f36cab/left.jpg
- runtime/frames/1bf3ce52a8244001bd18698918f36cab/left.png
- runtime/frames/1bf3ce52a8244001bd18698918f36cab/meta.json
- runtime/frames/1bf3ce52a8244001bd18698918f36cab/right.jpg
- runtime/frames/1bf3ce52a8244001bd18698918f36cab/right.png
- runtime/frames/25186a33097b435ab7cd6ed25b235616/head.jpg
- runtime/frames/25186a33097b435ab7cd6ed25b235616/head.png
- runtime/frames/25186a33097b435ab7cd6ed25b235616/left.jpg
- runtime/frames/25186a33097b435ab7cd6ed25b235616/left.png
- runtime/frames/25186a33097b435ab7cd6ed25b235616/meta.json
- runtime/frames/25186a33097b435ab7cd6ed25b235616/right.jpg
- runtime/frames/25186a33097b435ab7cd6ed25b235616/right.png
- runtime/frames/2acd8865dbc54d458bbf75110b7fa513/head.jpg
- runtime/frames/2acd8865dbc54d458bbf75110b7fa513/head.png
- runtime/frames/2acd8865dbc54d458bbf75110b7fa513/left.jpg
- runtime/frames/2acd8865dbc54d458bbf75110b7fa513/left.png
- runtime/frames/2acd8865dbc54d458bbf75110b7fa513/meta.json
- runtime/frames/2acd8865dbc54d458bbf75110b7fa513/right.jpg
- runtime/frames/2acd8865dbc54d458bbf75110b7fa513/right.png
- runtime/frames/2da08a44d99742be9d86614763e37449/head.jpg
- runtime/frames/2da08a44d99742be9d86614763e37449/head.png
- runtime/frames/2da08a44d99742be9d86614763e37449/left.jpg
- runtime/frames/2da08a44d99742be9d86614763e37449/left.png
- runtime/frames/2da08a44d99742be9d86614763e37449/meta.json
- runtime/frames/2da08a44d99742be9d86614763e37449/right.jpg
- runtime/frames/2da08a44d99742be9d86614763e37449/right.png
- runtime/frames/2f02aff27a2845058df86f20c6122a79/head.jpg
- runtime/frames/2f02aff27a2845058df86f20c6122a79/head.png
- runtime/frames/2f02aff27a2845058df86f20c6122a79/left.jpg
- runtime/frames/2f02aff27a2845058df86f20c6122a79/left.png
- runtime/frames/2f02aff27a2845058df86f20c6122a79/meta.json
- runtime/frames/2f02aff27a2845058df86f20c6122a79/right.jpg
- runtime/frames/2f02aff27a2845058df86f20c6122a79/right.png
- runtime/frames/3416ddeb272944778da7dfa780d52dcc/head.jpg
- runtime/frames/3416ddeb272944778da7dfa780d52dcc/left.jpg
- runtime/frames/3416ddeb272944778da7dfa780d52dcc/meta.json
- runtime/frames/3416ddeb272944778da7dfa780d52dcc/right.jpg
- runtime/frames/3c792887b1ca426792ce7a868d0d0e60/head.jpg
- runtime/frames/3c792887b1ca426792ce7a868d0d0e60/head.png
- runtime/frames/3c792887b1ca426792ce7a868d0d0e60/left.jpg
- runtime/frames/3c792887b1ca426792ce7a868d0d0e60/left.png
- runtime/frames/3c792887b1ca426792ce7a868d0d0e60/meta.json
- runtime/frames/3c792887b1ca426792ce7a868d0d0e60/right.jpg
- runtime/frames/3c792887b1ca426792ce7a868d0d0e60/right.png
- runtime/frames/3ea2fa2750bd454f818c793e18d728e6/head.jpg
- runtime/frames/3ea2fa2750bd454f818c793e18d728e6/head.png
- runtime/frames/3ea2fa2750bd454f818c793e18d728e6/left.jpg
- runtime/frames/3ea2fa2750bd454f818c793e18d728e6/left.png
- runtime/frames/3ea2fa2750bd454f818c793e18d728e6/meta.json
- runtime/frames/3ea2fa2750bd454f818c793e18d728e6/right.jpg
- runtime/frames/3ea2fa2750bd454f818c793e18d728e6/right.png
- runtime/frames/41b3c3edcf31464f8e5870506d42193a/head.jpg
- runtime/frames/41b3c3edcf31464f8e5870506d42193a/head.png
- runtime/frames/41b3c3edcf31464f8e5870506d42193a/left.jpg
- runtime/frames/41b3c3edcf31464f8e5870506d42193a/left.png
- runtime/frames/41b3c3edcf31464f8e5870506d42193a/meta.json
- runtime/frames/41b3c3edcf31464f8e5870506d42193a/right.jpg
- runtime/frames/41b3c3edcf31464f8e5870506d42193a/right.png
- runtime/frames/428c1a54fa1543de835f568bb6346589/head.jpg
- runtime/frames/428c1a54fa1543de835f568bb6346589/head.png
- runtime/frames/428c1a54fa1543de835f568bb6346589/left.jpg
- runtime/frames/428c1a54fa1543de835f568bb6346589/left.png
- runtime/frames/428c1a54fa1543de835f568bb6346589/meta.json
- runtime/frames/428c1a54fa1543de835f568bb6346589/right.jpg
- runtime/frames/428c1a54fa1543de835f568bb6346589/right.png
- runtime/frames/532aff006d214521a9889d3a74c8ed4b/head.jpg
- runtime/frames/532aff006d214521a9889d3a74c8ed4b/head.png
- runtime/frames/532aff006d214521a9889d3a74c8ed4b/left.jpg
- runtime/frames/532aff006d214521a9889d3a74c8ed4b/left.png
- runtime/frames/532aff006d214521a9889d3a74c8ed4b/meta.json
- runtime/frames/532aff006d214521a9889d3a74c8ed4b/right.jpg
- runtime/frames/532aff006d214521a9889d3a74c8ed4b/right.png
- runtime/frames/5ef3f2838a124bf98900a562dc141748/head.jpg
- runtime/frames/5ef3f2838a124bf98900a562dc141748/head.png
- runtime/frames/5ef3f2838a124bf98900a562dc141748/left.jpg
- runtime/frames/5ef3f2838a124bf98900a562dc141748/left.png
- runtime/frames/5ef3f2838a124bf98900a562dc141748/meta.json
- runtime/frames/5ef3f2838a124bf98900a562dc141748/right.jpg
- runtime/frames/5ef3f2838a124bf98900a562dc141748/right.png
- runtime/frames/623afe4d6c9a49ff85b2e9e00de4dd83/head.jpg
- runtime/frames/623afe4d6c9a49ff85b2e9e00de4dd83/head.png
- runtime/frames/623afe4d6c9a49ff85b2e9e00de4dd83/left.jpg
- runtime/frames/623afe4d6c9a49ff85b2e9e00de4dd83/left.png
- runtime/frames/623afe4d6c9a49ff85b2e9e00de4dd83/meta.json
- runtime/frames/623afe4d6c9a49ff85b2e9e00de4dd83/right.jpg
- runtime/frames/623afe4d6c9a49ff85b2e9e00de4dd83/right.png
- runtime/frames/650328e5da724a42b21176668e41ad20/head.jpg
- runtime/frames/650328e5da724a42b21176668e41ad20/head.png
- runtime/frames/650328e5da724a42b21176668e41ad20/left.jpg
- runtime/frames/650328e5da724a42b21176668e41ad20/left.png
- runtime/frames/650328e5da724a42b21176668e41ad20/meta.json
- runtime/frames/650328e5da724a42b21176668e41ad20/right.jpg
- runtime/frames/650328e5da724a42b21176668e41ad20/right.png
- runtime/frames/6c62aa0ae26a4efbaec60c9c0b3e88b0/head.jpg
- runtime/frames/6c62aa0ae26a4efbaec60c9c0b3e88b0/head.png
- runtime/frames/6c62aa0ae26a4efbaec60c9c0b3e88b0/left.jpg
- runtime/frames/6c62aa0ae26a4efbaec60c9c0b3e88b0/left.png
- runtime/frames/6c62aa0ae26a4efbaec60c9c0b3e88b0/meta.json
- runtime/frames/6c62aa0ae26a4efbaec60c9c0b3e88b0/right.jpg
- runtime/frames/6c62aa0ae26a4efbaec60c9c0b3e88b0/right.png
- runtime/frames/8d60f55b84b0416999bc61f7ea1ed25c/head.jpg
- runtime/frames/8d60f55b84b0416999bc61f7ea1ed25c/head.png
- runtime/frames/8d60f55b84b0416999bc61f7ea1ed25c/left.jpg
- runtime/frames/8d60f55b84b0416999bc61f7ea1ed25c/left.png
- runtime/frames/8d60f55b84b0416999bc61f7ea1ed25c/meta.json
- runtime/frames/8d60f55b84b0416999bc61f7ea1ed25c/right.jpg
- runtime/frames/8d60f55b84b0416999bc61f7ea1ed25c/right.png
- runtime/frames/a70b0922ea4b406b9ccf0799284439ce/head.jpg
- runtime/frames/a70b0922ea4b406b9ccf0799284439ce/head.png
- runtime/frames/a70b0922ea4b406b9ccf0799284439ce/left.jpg
- runtime/frames/a70b0922ea4b406b9ccf0799284439ce/left.png
- runtime/frames/a70b0922ea4b406b9ccf0799284439ce/meta.json
- runtime/frames/a70b0922ea4b406b9ccf0799284439ce/right.jpg
- runtime/frames/a70b0922ea4b406b9ccf0799284439ce/right.png
- runtime/frames/a75e0f2b03e24c43881353216d6baf0f/head.jpg
- runtime/frames/a75e0f2b03e24c43881353216d6baf0f/head.png
- runtime/frames/a75e0f2b03e24c43881353216d6baf0f/left.jpg
- runtime/frames/a75e0f2b03e24c43881353216d6baf0f/left.png
- runtime/frames/a75e0f2b03e24c43881353216d6baf0f/meta.json
- runtime/frames/a75e0f2b03e24c43881353216d6baf0f/right.jpg
- runtime/frames/a75e0f2b03e24c43881353216d6baf0f/right.png
- runtime/frames/aad27e008024418cadd391fb06d3a029/head.jpg
- runtime/frames/aad27e008024418cadd391fb06d3a029/head.png
- runtime/frames/aad27e008024418cadd391fb06d3a029/left.jpg
- runtime/frames/aad27e008024418cadd391fb06d3a029/left.png
- runtime/frames/aad27e008024418cadd391fb06d3a029/meta.json
- runtime/frames/aad27e008024418cadd391fb06d3a029/right.jpg
- runtime/frames/aad27e008024418cadd391fb06d3a029/right.png
- runtime/frames/ae0ef202d2da49279910cfe07ce82d11/head.jpg
- runtime/frames/ae0ef202d2da49279910cfe07ce82d11/head.png
- runtime/frames/ae0ef202d2da49279910cfe07ce82d11/left.jpg
- runtime/frames/ae0ef202d2da49279910cfe07ce82d11/left.png
- runtime/frames/ae0ef202d2da49279910cfe07ce82d11/meta.json
- runtime/frames/ae0ef202d2da49279910cfe07ce82d11/right.jpg
- runtime/frames/ae0ef202d2da49279910cfe07ce82d11/right.png
- runtime/frames/b298753e93904f17a643573faa0eb54c/head.jpg
- runtime/frames/b298753e93904f17a643573faa0eb54c/head.png
- runtime/frames/b298753e93904f17a643573faa0eb54c/left.jpg
- runtime/frames/b298753e93904f17a643573faa0eb54c/left.png
- runtime/frames/b298753e93904f17a643573faa0eb54c/meta.json
- runtime/frames/b298753e93904f17a643573faa0eb54c/right.jpg
- runtime/frames/b298753e93904f17a643573faa0eb54c/right.png
- runtime/frames/b6fb4dc66b7d42b0b108dab07a57b05d/head.jpg
- runtime/frames/b6fb4dc66b7d42b0b108dab07a57b05d/head.png
- runtime/frames/b6fb4dc66b7d42b0b108dab07a57b05d/left.jpg
- runtime/frames/b6fb4dc66b7d42b0b108dab07a57b05d/left.png
- runtime/frames/b6fb4dc66b7d42b0b108dab07a57b05d/meta.json
- runtime/frames/b6fb4dc66b7d42b0b108dab07a57b05d/right.jpg
- runtime/frames/b6fb4dc66b7d42b0b108dab07a57b05d/right.png
- runtime/frames/c49ab9887c22499991b013bcdf00110f/head.jpg
- runtime/frames/c49ab9887c22499991b013bcdf00110f/head.png
- runtime/frames/c49ab9887c22499991b013bcdf00110f/left.jpg
- runtime/frames/c49ab9887c22499991b013bcdf00110f/left.png
- runtime/frames/c49ab9887c22499991b013bcdf00110f/meta.json
- runtime/frames/c49ab9887c22499991b013bcdf00110f/right.jpg
- runtime/frames/c49ab9887c22499991b013bcdf00110f/right.png
- runtime/frames/d6410751db054c3e971d7ba153f928ed/head.jpg
- runtime/frames/d6410751db054c3e971d7ba153f928ed/head.png
- runtime/frames/d6410751db054c3e971d7ba153f928ed/left.jpg
- runtime/frames/d6410751db054c3e971d7ba153f928ed/left.png
- runtime/frames/d6410751db054c3e971d7ba153f928ed/meta.json
- runtime/frames/d6410751db054c3e971d7ba153f928ed/right.jpg
- runtime/frames/d6410751db054c3e971d7ba153f928ed/right.png
- runtime/frames/de3845b2af27468f94becdf09f0baefb/head.jpg
- runtime/frames/de3845b2af27468f94becdf09f0baefb/head.png
- runtime/frames/de3845b2af27468f94becdf09f0baefb/left.jpg
- runtime/frames/de3845b2af27468f94becdf09f0baefb/left.png
- runtime/frames/de3845b2af27468f94becdf09f0baefb/meta.json
- runtime/frames/de3845b2af27468f94becdf09f0baefb/right.jpg
- runtime/frames/de3845b2af27468f94becdf09f0baefb/right.png
- runtime/frames/e5f12f371c104fc1bc59fe58aa9565b2/head.jpg
- runtime/frames/e5f12f371c104fc1bc59fe58aa9565b2/head.png
- runtime/frames/e5f12f371c104fc1bc59fe58aa9565b2/left.jpg
- runtime/frames/e5f12f371c104fc1bc59fe58aa9565b2/left.png
- runtime/frames/e5f12f371c104fc1bc59fe58aa9565b2/meta.json
- runtime/frames/e5f12f371c104fc1bc59fe58aa9565b2/right.jpg
- runtime/frames/e5f12f371c104fc1bc59fe58aa9565b2/right.png
- runtime/frames/e8872eefcea64bd0aa5607058bd4bf24/head.jpg
- runtime/frames/e8872eefcea64bd0aa5607058bd4bf24/head.png
- runtime/frames/e8872eefcea64bd0aa5607058bd4bf24/left.jpg
- runtime/frames/e8872eefcea64bd0aa5607058bd4bf24/left.png
- runtime/frames/e8872eefcea64bd0aa5607058bd4bf24/meta.json
- runtime/frames/f0e8a38b51224452b830923e8b32547a/head.jpg
- runtime/frames/f0e8a38b51224452b830923e8b32547a/head.png
- runtime/frames/f0e8a38b51224452b830923e8b32547a/left.jpg
- runtime/frames/f0e8a38b51224452b830923e8b32547a/left.png
- runtime/frames/f0e8a38b51224452b830923e8b32547a/meta.json
- runtime/frames/f0e8a38b51224452b830923e8b32547a/right.jpg
- runtime/frames/f0e8a38b51224452b830923e8b32547a/right.png

### Referenced measurement-source frames
- runtime/frames/274698d9681043bcb9fed67bd0ad398d/head.jpg
- runtime/frames/347a0a825ffe47b7ae36f90a0ad72210/head.jpg
- runtime/frames/347a0a825ffe47b7ae36f90a0ad72210/head.png
- runtime/frames/347a0a825ffe47b7ae36f90a0ad72210/left.jpg
- runtime/frames/347a0a825ffe47b7ae36f90a0ad72210/left.png
- runtime/frames/347a0a825ffe47b7ae36f90a0ad72210/meta.json
- runtime/frames/347a0a825ffe47b7ae36f90a0ad72210/right.jpg
- runtime/frames/347a0a825ffe47b7ae36f90a0ad72210/right.png
- runtime/frames/3c39f36099b348a8b100062b8c1fd513/head.jpg
- runtime/frames/3c39f36099b348a8b100062b8c1fd513/left.jpg
- runtime/frames/3c39f36099b348a8b100062b8c1fd513/meta.json
- runtime/frames/3c39f36099b348a8b100062b8c1fd513/right.jpg
- runtime/frames/422fddd5a1814fada9d5aab526bab68c/head.jpg
- runtime/frames/5513f0b0eb9840a396f41df26c136f97/head.jpg
- runtime/frames/63d843b4c9bf4db7b3c16b1899f95a37/head.jpg
- runtime/frames/65736d4a12c44f349d952fd922fe9695/head.jpg
- runtime/frames/65736d4a12c44f349d952fd922fe9695/left.jpg
- runtime/frames/65736d4a12c44f349d952fd922fe9695/meta.json
- runtime/frames/65736d4a12c44f349d952fd922fe9695/right.jpg
- runtime/frames/75abd4859eef4025a241fd8ac2ad0ae0/head.jpg
- runtime/frames/8ddece8dc4e64cb5bf98c9676a836aca/head.jpg
- runtime/frames/8ddece8dc4e64cb5bf98c9676a836aca/left.jpg
- runtime/frames/8ddece8dc4e64cb5bf98c9676a836aca/meta.json
- runtime/frames/8ddece8dc4e64cb5bf98c9676a836aca/right.jpg
- runtime/frames/8edf92f876084dbebee7dcca0a801758/head.jpg
- runtime/frames/8edf92f876084dbebee7dcca0a801758/head.png
- runtime/frames/8edf92f876084dbebee7dcca0a801758/left.jpg
- runtime/frames/8edf92f876084dbebee7dcca0a801758/left.png
- runtime/frames/8edf92f876084dbebee7dcca0a801758/meta.json
- runtime/frames/8edf92f876084dbebee7dcca0a801758/right.jpg
- runtime/frames/8edf92f876084dbebee7dcca0a801758/right.png
- runtime/frames/910ef64b39b1446badf42d8bfc00e15d/head.jpg
- runtime/frames/92d371d4e45047478d916717fdf5937a/head.jpg
- runtime/frames/d7266aef313e424287fc35c141abd9b8/head.jpg
- runtime/frames/d7266aef313e424287fc35c141abd9b8/head.png
- runtime/frames/d7266aef313e424287fc35c141abd9b8/left.jpg
- runtime/frames/d7266aef313e424287fc35c141abd9b8/left.png
- runtime/frames/d7266aef313e424287fc35c141abd9b8/meta.json
- runtime/frames/d7266aef313e424287fc35c141abd9b8/right.jpg
- runtime/frames/d7266aef313e424287fc35c141abd9b8/right.png
- runtime/frames/da4ec2d2be364967a6a6efac40a5052a/head.jpg
- runtime/frames/df512971d5ca48478e18f50e4d2f4ebe/head.jpg
- runtime/frames/df512971d5ca48478e18f50e4d2f4ebe/head.png
- runtime/frames/df512971d5ca48478e18f50e4d2f4ebe/left.jpg
- runtime/frames/df512971d5ca48478e18f50e4d2f4ebe/left.png
- runtime/frames/df512971d5ca48478e18f50e4d2f4ebe/meta.json
- runtime/frames/df512971d5ca48478e18f50e4d2f4ebe/right.jpg
- runtime/frames/df512971d5ca48478e18f50e4d2f4ebe/right.png
- runtime/frames/fdbea0cfd20c48d3b7405f7eb58f0c5a/head.jpg
- runtime/frames/fdbea0cfd20c48d3b7405f7eb58f0c5a/left.jpg
- runtime/frames/fdbea0cfd20c48d3b7405f7eb58f0c5a/meta.json
- runtime/frames/fdbea0cfd20c48d3b7405f7eb58f0c5a/right.jpg

### Immutable measured depth arrays
- runtime/frames/067340a2f76f4e65add5b680b2e23da3/head-depth.npz
- runtime/frames/072c02d78643428b80779c051651294b/head-depth.npz
- runtime/frames/1600d4b5aac840299a18431a3efd80f7/head-depth.npz
- runtime/frames/1bf3ce52a8244001bd18698918f36cab/head-depth.npz
- runtime/frames/25186a33097b435ab7cd6ed25b235616/head-depth.npz
- runtime/frames/2acd8865dbc54d458bbf75110b7fa513/head-depth.npz
- runtime/frames/2da08a44d99742be9d86614763e37449/head-depth.npz
- runtime/frames/2f02aff27a2845058df86f20c6122a79/head-depth.npz
- runtime/frames/347a0a825ffe47b7ae36f90a0ad72210/head-depth.npz
- runtime/frames/3c792887b1ca426792ce7a868d0d0e60/head-depth.npz
- runtime/frames/3ea2fa2750bd454f818c793e18d728e6/head-depth.npz
- runtime/frames/41b3c3edcf31464f8e5870506d42193a/head-depth.npz
- runtime/frames/428c1a54fa1543de835f568bb6346589/head-depth.npz
- runtime/frames/532aff006d214521a9889d3a74c8ed4b/head-depth.npz
- runtime/frames/623afe4d6c9a49ff85b2e9e00de4dd83/head-depth.npz
- runtime/frames/650328e5da724a42b21176668e41ad20/head-depth.npz
- runtime/frames/6c62aa0ae26a4efbaec60c9c0b3e88b0/head-depth.npz
- runtime/frames/8d60f55b84b0416999bc61f7ea1ed25c/head-depth.npz
- runtime/frames/8edf92f876084dbebee7dcca0a801758/head-depth.npz
- runtime/frames/a70b0922ea4b406b9ccf0799284439ce/head-depth.npz
- runtime/frames/a75e0f2b03e24c43881353216d6baf0f/head-depth.npz
- runtime/frames/aad27e008024418cadd391fb06d3a029/head-depth.npz
- runtime/frames/ae0ef202d2da49279910cfe07ce82d11/head-depth.npz
- runtime/frames/b298753e93904f17a643573faa0eb54c/head-depth.npz
- runtime/frames/b6fb4dc66b7d42b0b108dab07a57b05d/head-depth.npz
- runtime/frames/c49ab9887c22499991b013bcdf00110f/head-depth.npz
- runtime/frames/d6410751db054c3e971d7ba153f928ed/head-depth.npz
- runtime/frames/d7266aef313e424287fc35c141abd9b8/head-depth.npz
- runtime/frames/de3845b2af27468f94becdf09f0baefb/head-depth.npz
- runtime/frames/df512971d5ca48478e18f50e4d2f4ebe/head-depth.npz
- runtime/frames/e5f12f371c104fc1bc59fe58aa9565b2/head-depth.npz
- runtime/frames/e8872eefcea64bd0aa5607058bd4bf24/head-depth.npz
- runtime/frames/e8872eefcea64bd0aa5607058bd4bf24/left-depth.npz
- runtime/frames/f0e8a38b51224452b830923e8b32547a/head-depth.npz

## R6 function rehearsal, 2026-11-11 (active)
- open_empty_gripper: success27.228s,8 jaw steps,69mm requested/68.5mm measured;0 auxiliary calls. Fresh scene/provisional table checks. Report output/rehearsal/r6-open-01.json.
- empty_joint_waypoint first half: success17.938s; nativeCAD/workspace and41sample finger/table/top sweep passed. Endpoint b7138ca41a9f46ac8d5bae9c5ecffe4a.
- second half attempt r6-transition-02: no dispatch,1.409s; RGB observation lacks projected finger corner arrays (geometry identity available). Added FK reconstruction from the source pinned geometry at unchanged jaw width; current identity/calibration/jaw checks remain. Not depth synthesis or guard relaxation.

- R6 transition-03 passed17.866s after geometry repair; approach-01 3.366s, approach-02 9.531s/3segments, insert-01 3.308s, close-01 9.733s/50mm request50.9mm feedback.
- R6 trial-01 passed2.988s,actual9.334mm upward,wrist correlation.9785 shift[0,0],jaw51mm. Additional85mm upward transport r6-lift-01 passed7.185s. Both live checks and Agent images support retention.0auxiliary/0operator assistance.

- R6 lateral-01 passed6.258s; intermediate lower-01 passed6.327s; final lower-02 executed4.521s, endpoint needs_review (red occluded wrist template; support hidden). Native operator fact9864a2c233f8 positive support/untrapped fingers at unchanged pose. release-01 passed9.202s,69mm command68.6mm feedback. retreat-01 executed8.693s, endpoint area heuristic failed on clipping. Agent images show released pair; fixed width-ratio check and read-only verify r6-retreat-verify passed0.575s. No motion replay. Base pair completion is supervised,1operator factual assistance,0Gemini. Green untouched.

- R7 r7-open-01: open_empty_gripper, ok=True, executed=True; 26.812s; 8 motion receipts; auxiliary visual calls=0; main-agent function invocation=1, visual inspection after return; no operator assistance; remaining budget at endpoint=5891.131; report output/rehearsal/r7-open-01.json. Supervised function rehearsal, not autonomous task success.

- R7 r7-transition-01: empty_joint_waypoint, ok=True, executed=True; 17.751s; 1 motion receipts; auxiliary visual calls=0; main-agent function invocation=1, visual inspection after return; no operator assistance; remaining budget at endpoint=5781.334; report output/rehearsal/r7-transition-01.json. Supervised function rehearsal, not autonomous task success.

## R7 current function ledger

Supervised single-function rehearsals, no unattended main. Each row is one main-agent invocation, inspected between functions. Exact total model turns are in host transcript, not measured by this runner. No operator assistance unless explicitly recorded.

| Function/report | seconds | ok | executed/receipts | auxiliary vision | remaining seconds |
|---|---:|---|---|---:|---:|
| r7-open-01 / open_empty_gripper | 26.812 | True | True/8 | 0 | 5891.131 |
| r7-transition-01 / empty_joint_waypoint | 17.751 | True | True/1 | 0 | 5781.334 |
| r7-transition-02 / empty_joint_waypoint | 15.044 | True | True/1 | 0 | 5686.021 |
| r7-transition-03 / empty_joint_waypoint | 8.739 | True | True/1 | 0 | 5640.921 |
| r7-approach-01 / approach | 3.515 | True | True/1 | 0 | 5610.801 |
| r7-approach-02 / approach | 0.519 | False | False/0 | 0 | 5582.806 |
| r7-approach-02b / approach | 3.421 | True | True/1 | 0 | 5519.934 |
| r7-approach-03 / approach | 3.731 | True | True/1 | 0 | 5484.806 |
| r7-approach-04 / approach | 3.422 | True | True/1 | 0 | 5454.72 |
| r7-approach-05 / approach | 3.276 | True | True/1 | 0 | 5420.109 |
| r7-close-01 / close_grasp | 9.034 | True | True/3 | 0 | 5385.048 |
| r7-trial-01 / trial_lift | 3.442 | True | True/1 | 0 | 5357.423 |
| r7-raise-01 / transport | 7.191 | True | True/1 | 0 | 5317.745 |
| r7-transfer-01 / transport | 5.849 | True | True/1 | 0 | 5277.722 |
| r7-lower-01 / lower_to_support | 5.571 | True | True/1 | 0 | 5210.044 |
| r7-lower-02 / lower_to_support | 1.245 | False | False/0 | 0 | 5173.412 |
| r7-raise-recovery / transport | 0.981 | False | False/0 | 0 | 4963.764 |
| r7-raise-recovery-02 / transport | 5.457 | True | True/1 | 0 | 4877.459 |
| r7-transfer-02 / transport | 3.993 | True | True/1 | 0 | 4837.888 |
| r7-lower-03 / lower_to_support | 5.596 | True | True/1 | 0 | 4798.822 |
| r7-lower-04 / lower_to_support | 4.885 | False | True/1 | 0 | 4754.706 |
| r7-support-verify / verify_completed_action | 21.34 | True | False/0 | 1 | 4648.884 |
| r7-release-01 / release | 20.73 | True | True/3 | 1 | 4561.197 |
| r7-retreat-01 / retreat | 13.829 | True | True/1 | 1 | 4518.052 |
| r7-inspection-01 / empty_inspection | 6.653 | True | True/1 | 0 | 4407.831 |
| r7-inspection-02 / empty_inspection | 1.171 | False | False/0 | 0 | 4275.452 |
| r7-inspection-02b / empty_inspection | 6.31 | True | True/1 | 0 | 4146.201 |
| r7-green-transition-01 / empty_inspection | 11.675 | True | True/1 | 0 | 3808.236 |
| r7-green-transition-02 / empty_inspection | 12.483 | True | True/1 | 0 | 3710.394 |
| r7-green-pregrasp / green_pregrasp | 0.524 | False | False/0 | 0 | 3552.291 |
| r7-green-pregrasp-02 / green_pregrasp | 1.38 | False | False/0 | 0 | 3509.634 |

## R7 software closeout verification

No new physical runs. RAX main Agent checkpoint `2101a65c71618fa902101452b9e2496c67dd2262` in `code/magnetic_blocks`.
69 offline tests passed (15 new R7 regressions), with live Context and socket access blocked;
compile and nonconnecting CLI preview passed. Commands, exit codes and logs:
`output/closeout/SUMMARY.json`. The saved-frame depth replay is an offline adapter,
not a native API or hardware pass. Initial regression errors exposed an additional
center-depth hole; corrected by explicitly labeled geometric inference from real flat
top samples. No original physical result was changed;70 original report/log hashes match.
Exact main-model counts and full software-phase duration are unavailable. No physical
time budget carried forward. Software robot/Gemini calls:0/0. Library merge pending.

# R8 live rehearsal

Supervised single-function invocations; not unattended runs. Primary Agent decisions between invocations; primary-model call count is not instrumented by the runner. Auxiliary calls and duration are measured. Report code_hashes identify each exact implementation.

| Report | Function | OK | Motions | Seconds | Auxiliary calls |
|---|---|---|---|---|---|
| r8-open-01.json | open_empty_gripper | True | 8 | 26.576 | 0 |
| r8-transition-01.json | empty_joint_waypoint | True | 1 | 18.046 | 0 |
| r8-transition-02.json | empty_joint_waypoint | True | 1 | 10.573 | 0 |
| r8-transition-03.json | empty_joint_waypoint | True | 1 | 10.167 | 0 |
| r8-approach-01.json | approach | True | 5 | 15.262 | 0 |
| r8-close-01.json | close_grasp | True | 3 | 8.926 | 0 |
| r8-trial-01.json | trial_lift | True | 1 | 3.091 | 0 |
| r8-raise-01.json | transport | True | 1 | 6.692 | 0 |
| r8-base-plan-01.json | plan_base_placement | True | 0 | 1.162 | 0 |
| r8-transfer-01.json | transport | True | 1 | 5.267 | 0 |
| r8-lower-01.json | lower_to_support | True | 1 | 4.647 | 0 |
| r8-lower-02.json | lower_to_support | True | 1 | 12.803 | 1 |
| r8-release-01.json | release | True | 3 | 19.276 | 1 |
| r8-retreat-01.json | retreat | True | 1 | 12.278 | 1 |
| r8-inspection-01.json | empty_inspection | True | 1 | 8.794 | 0 |
| r8-inspection-02.json | empty_inspection | True | 1 | 13.793 | 0 |
| r8-green-pregrasp-01.json | green_pregrasp | False | 0 | 0.715 | 0 |
| r8-green-pregrasp-02.json | green_pregrasp | True | 1 | 5.710 | 0 |
| r8-support-refresh-01.json | refresh_stack_support | True | 0 | 0.967 | 0 |
| r8-green-approach-01.json | approach | False | 0 | 0.576 | 0 |
| r8-green-approach-02.json | approach | True | 4 | 12.473 | 0 |
| r8-green-close-01.json | close_grasp | True | 3 | 13.027 | 0 |
| r8-green-trial-01.json | trial_lift | True | 1 | 3.206 | 0 |
| r8-green-raise-01.json | transport | True | 1 | 5.275 | 0 |
| r8-green-transfer-01.json | transport | True | 1 | 6.675 | 0 |
| r8-green-raise-02.json | transport | True | 1 | 3.602 | 0 |
| r8-green-transfer-02.json | transport | True | 1 | 5.675 | 0 |
| r8-green-transfer-03.json | transport | True | 1 | 6.431 | 0 |
| r8-stack-lower-01.json | green_stack_lower | True | 1 | 11.979 | 1 |
| r8-stack-release-01.json | green_stack_release | False | 0 | 7.517 | 1 |
