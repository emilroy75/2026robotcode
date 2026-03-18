# 2026 Robot Development Log
**Team 250 — Emil Roy**

---

## 1. Shooter Subsystem Architecture
- **Structure:** AdvantageKit Tri-layer (Subsystem, IO Interface, Spark/Sim implementations).
- **Simulation:** Uses `DCMotorSim` with `createDCMotorSystem` to track both position and velocity.

## 2. REVLib 2026 Migration
- **Imports:** Moved `PersistMode` and `ResetMode` to `com.revrobotics`.
- **Method:** Using `sparkMax.configure()` with `kResetSafe` and `kPersist` flags.

## 3. Team-Side Logic (HubPose)
- **Initialization:** Handled in `Robot.java` inside the `Robot()` constructor (or `autonomousInit()` for safety).
- **Storage:** `public static Pose2d HubPose` in `Robot.java` serves as the "Global Target" for all subsystems.
- **Logic:** `if/else` checks for `Alliance.Red` vs `Alliance.Blue` to lock in field coordinates once.

## 4. Advanced Trajectory Math
- **Method:** `ShooterMath.java` uses a physics-based projectile motion formula rather than a simple linear line.
- **Workflow:**
  1. `getDistanceToSpeaker(drive.getPose())`
  2. `calculateTrajectoryVelocity(distance)` -> Returns meters per second.
  3. `velocityToRPM(velocity)` -> Returns final motor RPM.
  4. `calculateEnergyBoost(velocity)` -> Adds RPM to compensate for ball impact.
- **Benefits:** Accounts for gravity, hub height, and fixed launch angle.

## 5. Control System
- **Driver (0):** Swerve and gyro controls.
- **Operator (1):** Shooter automation (Right Trigger for auto-shoot, Left Trigger for manual feed).
- **Command Factory:** Implemented `ShooterCommands.java` to centralize all shooter logic (Auto-Aim + Spin-Up + Feed) without separate files.

## 6. Fixes & Cleanup
- **Typo Resolution:** Fixed `Commannds` -> `Commands` in `ShooterCommands.java`.
- **Import Resolution:** Added `edu.wpi.first.wpilibj2.command.Commands` to resolve command factory errors.
