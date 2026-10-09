# References for the FW 26.1.5 review (fetched 2026-09-28)

Supporting evidence for `../../FW26_CHANGES_2026_09_28.md`. Level A = REV official spec or REV code. B = REV docs or REV staff. C = community. "Sim" marks REV simulation code: REV says it was "translated directly from the Spark firmware" (SparkSim.java:190), but it has open bugs (issue #29), so firmware behaviour still needs a bench check.

## Versions checked for anything newer (2026-09-28)
- **REV-Specs** (github.com/REVrobotics/REV-Specs): the newest commit is still `1e90305632317f97e922257ad4b22d7a324f3afd` (2026-01-02, "Add SPARK frames 2.1.0"). `can-frames/` holds only `spark-frames-2.0.0-dev.11`, `spark-frames-2.1.0` and `servo_hub-frames-2.0.0-dev.11`. **Nothing newer than 2.1.0**, so the pinned copy in `../rev_2026_spark_frames_2.1.0.json` is still current.
- **SPARK MAX firmware**: `sm-26.1.5` (2026-03-12) is the newest `sm-*` release. **REVLib 2026**: `2026.0.5` (2026-03-12) is the newest 2026 release (REVLib-2026.json vendordep also lists 2026.0.5). There are newer 2027 alphas, up to `2027.0.0-alpha-7` (2026-09-11), which target 2027 firmware and Systemcore, not FW 26.x.
- **REV Hardware Client 1**: `rhc-1.7.7` (2026-09-12), whose only change is an EOL banner. RHC2 is at 1.4.2 per its docs changelog.

## Files
| File | Source (pinned) | Level |
|---|---|---|
| rev_release_notes_sm26_revlib2026_rhc_2026_09_28.md | GitHub Releases API, REVrobotics/REV-Software-Binaries: every `sm-26*` (incl. prereleases), `revlib-2026*`, `revlib-2027*`, `rhc-1.7.6/7` body, verbatim | A |
| rev_revlib_java_2026.0.5_SparkSim.java, …_MovingAverageFilterSim.java, …_SignalsConfig.java, …_MAXMotionConfig.java | https://maven.revrobotics.com/com/revrobotics/frc/REVLib-java/2026.0.5/REVLib-java-2026.0.5-sources.jar (sha256 1a02e179129e9750cf95bf6795c566c2ca705dbebea5c5824a7425f512673ae3) | A (sim) |
| rev_revlib_driver_2026.0.5_sim_disassembly.txt | objdump/gdb of `libREVLibDriver.so` from https://maven.revrobotics.com/com/revrobotics/frc/REVLib-driver/2026.0.5/REVLib-driver-2026.0.5-linuxx86-64debug.zip (sha256 7c99be219973065455287036f03988aac97097a040a8d99cbf8f4b2c16497643; ELF BuildID d24291261c494cc191e04c8c143ab0869cf41116; not stripped, with DWARF). Functions: `c_SIM_Spark_CalculateFeedforward` (sim/CANSpark.cpp:1232-1273), `c_SIM_Spark_CalculatePID` (:1294-1341), `c_SIM_Spark_GetSimPIDOutput` (CANSparkDriver.cpp:2276-2311), `c_SIM_Spark_SimulateMaxMotionVelocityControl` (sim/CANSpark.cpp:1500-1555), `FrameDaemon::Main` (SparkFrameManager.cpp:68-147) | A (sim / host code) |
| rev_revlib_driver_2026.0.5_param_defaults_gdb.txt | The same library's `s_Spark_ParameterTable` (227 entries: id, type, default, description), dumped at runtime with gdb (library loaded with RevLibBackendDriver-2026.0.5-linuxx86-64.zip, sha256 3112d6f26580aa5ed6040b3735b62634d151fffe0ce2e6075d1fc2d52da9bc2a) | A |
| rev_docs_*.md | https://docs.revrobotics.com/<path>.md (the official Markdown export; the path is in each file's first line) | B |
| github_rsb_issue{4,12,16,25,29}.md | https://github.com/REVrobotics/REV-Software-Binaries/issues/N plus comments (API); `jfabellera` = REV member | B/C |

## Decoded sim equations (from rev_revlib_driver_2026.0.5_sim_disassembly.txt)
```
// CalculateFeedforward(constants{kS,kV,kA,kG,kCos,kCosRatio}, state{position,velocity,acceleration}, signals{s,v,a,g,cos}, Vbus)
sgain  = copysignf(kS, state.velocity)          // :1233  (+0.0 → +kS)
vgain  = state.velocity * kV                     // :1234
again  = state.acceleration * kA                 // :1235
ggain  = kG; cosgain = kCos [* cos(position*kCosRatio*2π) if cos enabled]   // :1236-1239
ff_V   = s*sgain + v*vgain + a*again + g*ggain + cos*cosgain                 // :1242-1244
ref    = (param74 VoltageCompMode != 0) ? param75 CompensatedNominalVoltage : Vbus  // :1255-1267 (initial 12.0)
return ref == 0 ? 0 : ff_V / ref                 // :1270-1272  → duty

// GetSimPIDOutput, velocity mode (control type 1): state = {0, setpoint, 0}; signals = {s=1,v=1,a=0,g=0,cos=0}
// position mode (3): state = {setpoint, setpoint - pv, 0}; signals = {s=1,v=0,a=0,g=1,cos=1}
// MAXMotion velocity: signals = {s=1,v=1,a=1,g=0,cos=0}, using the profile's velocity/acceleration

// CalculatePID(slot): ids p=13+8*slot, i=+1, d=+2, izone=+4, dfilter=+5, min=+6, max=+7; allowedErr = 97+4*slot
if (|error| <= allowedErr) return clamp(ff, min, max)          // PID tolerance: FF only
iAccum = (izone==0 || |error| <= izone) ? iAccum + error*i*dt : 0
out = iAccum + p*error + d*(error-prev)/dt + ff; return clamp(out, min, max)

// SparkSim.iterate (Java :329-352): applied += arbFF/Vbus (units bit 0 = volts) or arbFF (duty);
// then, if param74 == 2: applied *= param75 / Vbus; then clamp to [-1, 1]
```

## Added 2026-09-28 for `../../KS_AT_ZERO_2026_09_28.md`
| File | Source (pinned) | sha256 | Level |
|---|---|---|---|
| rev_revlib_javadoc_2026_FeedForwardConfig.txt | https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/feedforwardconfig (HTML → text) | e683e40b315a7b66f1125fe68d9c8a196c99579b4870b07ff7d43cd2d60f30a8 | A |
| wpilib_v2026.2.1_SimpleMotorFeedforward.java | https://raw.githubusercontent.com/wpilibsuite/allwpilib/v2026.2.1/wpimath/src/main/java/edu/wpi/first/math/controller/SimpleMotorFeedforward.java (identical at v2026.1.1) | 353d60800efb233788fc103bb6d1ff2669df8180459a0eec0054733a27ea17fc | A (WPILib) |
| ctre_phoenix6_StaticFeedforwardSignValue.txt | https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/signals/StaticFeedforwardSignValue.html (HTML → text) | 4e03dff7df81678ce3bc8b65c4a628392da0839ced08c0d1a4fbb421c028f91a | A (CTRE) |
Re-fetched `rev_docs_revlib_spark_closed-loop_feed-forward-control.md`, `…closed-loop-control-getting-started.md`, `…velocity-control-mode.md` on 2026-09-28: feed-forward page content identical to the saved copy.
