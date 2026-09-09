# CHEAP-MULTI — stopped at user request

Runtime source baseline: Zone A e601edb; Zone C 5076902 (Claude added
heavy CLI/installed-world lookup; cheap perception/controller unchanged).
No source changes or controller tuning made during these tests.

Artifacts: /home/saurabh/ardu_ws/eval_results/cheap_multi_QKvtug/.
Zone A reached the endpoint and landed/disarmed. Existing axis-aligned
footprint evaluator minimum margin is only 0.014 m near boxA1. Offline
SDF-yaw-corrected XY footprint audit gives -0.091 m; altitude ~3.37 m is
above the 3 m box, so this is not proof of physical contact. Do not present
this as robust clearance. Audit JSON, trajectory PNG and flight.mp4 saved.

Zone C stopped at user request after gap-selection deadlock near
(84.95,1.75). 373 control frames, 42.8 capture-time seconds, 229 avoid and
144 blind frames. Latest detection interval roughly [-0.978,0.514] plus
0.55 rad padding blocks all candidates inside ±0.9 rad. LK can remain
11/11 valid while controller emits blind/zero: this is not camera failure.
No endpoint completion. result.json records user_stop, landed/disarmed,
cleanup errors empty. Raw/annotated videos and frame logs preserved.

User explicitly requested all processes stopped. Host verification found
no simulator, controller, MAVROS, bridge, MAVProxy, live viewer or ffplay
processes; UDP14550/14551/9002/9003 and TCP5760 free. Edit lock released.
No new offline test suite run because no runtime code changed. Further
analysis/implementation paused; next work is separate obstacle tracking,
persistent gap selection and yaw-aware evaluator geometry. Peer review pending.
