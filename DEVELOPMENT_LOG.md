# 2026 Robot Development Log
**Team 250 — Emil Roy**

This log tracks the architectural decisions, bug fixes, and subsystem implementations for the 2026 robot.

---

## 1. Shooter Subsystem Architecture
We implemented a **tri-layered AdvantageKit architecture** for the shooter to support real hardware, physics simulation, and log replay.

- **`Shooter.java`**: The high-level subsystem. Handles logic like `atTargetRPM()`.
- **`ShooterIO.java`**: The interface defining inputs (velocity, current, etc.) and outputs (`setShootSpeed`, `setFeedSpeed`).
- **`ShooterIOSpark.java`**: Real-world implementation using **Spark MAX** (REVLib 2026).
- **`ShooterIOSim.java`**: Physics simulation using **WPILib `DCMotorSim`**.

### Key Decision: `ShooterConstants.java`
We centralized all shooter hardware parameters (CAN IDs, Gear Ratios, Current Limits) into one file to simplify tuning and hardware swaps.

---

## 2. REVLib 2026 Migration & Fixes
During implementation, we resolved several breaking changes in the **REVLib 2026** API:

- **Package Change**: `PersistMode` and `ResetMode` were moved from `com.revrobotics.spark` to the base **`com.revrobotics`** package.
- **Enum Renaming**: Shortened names are now required:
  - `kResetSafeParameters` → **`kResetSafe`**
  - `kPersistParameters` → **`kPersist`**
- **Configuration System**: Switched from imperative `restoreFactoryDefaults()` to the new **`sparkMax.configure()`** method.

---

## 3. Control System & Wiring
We updated `RobotContainer.java` to support a **two-controller setup**:

- **Driver (Controller 0)**: Handles field-relative swerve drive and gyro resets.
- **Operator (Controller 1)**: Dedicated to the shooter mechanism.
  - **Right Trigger**: Spin up shooter and autofeed (via `ShootCommand`).
  - **Left Trigger**: Manual feed control.

### Logic Fix: `Command.schedule()`
Resolved a deprecation warning by moving away from manual `.schedule()` calls inside button bindings, favoring direct command composition or `Commands.defer()`.

---

## 4. Shooter Math & Trajectory
- **`ShooterMath.java`**: Implemented a linear regression model (`y = mx + b`) to calculate required RPM based on distance.
- **`ShootCommand.java`**: Created an automated sequence that monitors shooter RPM and only triggers the feeder once the wheels have reached the target speed.

---

## 5. Simulation Physics
- **MOI (Moment of Inertia)**: Added placeholder MOI values to the simulation to ensure the shooter has realistic "spin-up" and "recovery" times in VS Code Sim.
- **Motor Model**: Updated simulation to use **NEO v1.1** brushless motor constants.
