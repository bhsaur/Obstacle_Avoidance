# Handoff: FLIGHT-001 takeover (Claude)

Recovered the stale FLIGHT-001 edit lock from the codex session after the
user confirmed it stopped and instructed Claude to take over (backup +
note in .collaboration/backups/claude-recovery-20260906/).

## Diagnosis of the "crash" the user hit running autonomous_demo
1. Gazebo GUI (use_gui:=true) crashed on Qt: it inherited cv2's
   QT_QPA_PLATFORM_PLUGIN_PATH (opencv sets it on import) and tried to
   load cv2's incompatible xcb plugin -> gz died -> SITL never got a JSON
   connection ("No JSON sensor message ... Waiting for connection"). This
   is why my earlier live demos worked (sim launched in a separate,
   cv2-free process). Workaround used: run Gazebo headless; the demo's own
   cv2 live window still shows. PROPER FIX (not yet applied): sanitize the
   env passed to the gz launch subprocess (drop QT_QPA_PLATFORM_PLUGIN_PATH)
   so --gazebo-gui works.
2. Arm rejected result=4 ("Arm: Accels inconsistent" + "GPS still
   configuring"). autonomous_demo gave up after 3 attempts / ~22s, before
   SITL settled (GPS finished ~21s). FIX APPLIED: widened to 6 attempts /
   20s (~100s budget) in autonomous_demo.py's takeoff loop.

## Result (takeover_demo_4, zone_A, spawn_y=-3.8, headless gz)
Armed on attempt 3, flew AUTONOMOUS AVOID. Notable: the drone MOVED
LATERALLY y=-3.8 -> +6.7 while x went 10 -> 24.6 (~10m lateral), i.e.
Codex's body->world ENU velocity-frame fix genuinely makes turns
redirect the actual path (contrast Steps CO/CR where lateral displacement
was ~3cm under the frame bug). collided=false, ended via user_stop
(window closed), landed/disarmed cleanly, no residue.

## Open items
- Apply the --gazebo-gui Qt-env fix so the 3D world view works.
- Frame fix warrants revisiting Steps CO/CR collision-rate conclusions.
- Tests: 39 pass (test_sector_controller/orchestrator/score_normalization).
