# Team 250 — 2026 Ultimate Technical Manual
**The Complete API Encyclopedia for Robot Hardware, Software, and Physics**

---

## 1. WPILib Core: Command-Based Framework

### **SubsystemBase**
The fundamental unit of robot hardware.
*   **`periodic()`**: Runs once every 20ms. Used for sensor updates and telemetry.
*   **`setDefaultCommand(Command command)`**: Sets the command that runs when no other command is using the subsystem.
*   **`getCurrentCommand()`**: Returns the command currently controlling this subsystem.

### **Command Lifecycle**
*   **`initialize()`**: Called once when the command is scheduled. Reset sensors/timers here.
*   **`execute()`**: Called every 20ms. The main logic loop.
*   **`isFinished()`**: Logic to determine when the task is complete.
*   **`end(boolean interrupted)`**: Cleanup code. `interrupted` is true if the button was released early or another command took over.
*   **`addRequirements(Subsystem... subsystems)`**: Crucial for safety. Prevents two commands from moving the same motor simultaneously.

### **Composition API (Commands Class)**
*   **`Commands.run(Runnable action, Subsystem... reqs)`**: Continuous loop.
*   **`Commands.runOnce(Runnable action, Subsystem... reqs)`**: Single execution.
*   **`Commands.sequence(Command... cmds)`**: Runs commands one after another.
*   **`Commands.parallel(Command... cmds)`**: Runs all commands at the same time.
*   **`Commands.waitUntil(BooleanSupplier condition)`**: Pauses a sequence until a sensor triggers.

---

## 2. WPILib Geometry & Kinematics

### **Coordinate System**
*   **Field Origin (0,0)**: The right-side corner of the Blue Alliance wall.
*   **X-Axis**: Length of the field (Forward for Blue).
*   **Y-Axis**: Width of the field (Left for Blue).
*   **Rotation**: CCW (Counter-Clockwise) is positive.

### **Geometry Classes**
*   **`Pose2d(Translation2d, Rotation2d)`**: The robot's state.
    *   `getDistance(Pose2d other)`: Returns straight-line distance in meters.
    *   `relativeTo(Pose2d other)`: Calculates your position from the target's perspective.
*   **`Rotation2d`**: Handles angle wrapping (e.g., 370° automatically becomes 10°).
    *   `fromDegrees(double)` / `fromRadians(double)`.
    *   `getDegrees()` / `getRadians()`.
*   **`Transform2d`**: The "difference" between two poses.

### **Swerve Kinematics**
*   **`ChassisSpeeds`**:
    *   `vxMetersPerSecond`: Forward speed.
    *   `vyMetersPerSecond`: Strafe speed.
    *   `omegaRadiansPerSecond`: Rotation speed.
    *   `fromFieldRelativeSpeeds(...)`: Converts driver inputs to robot-relative movements.
*   **`SwerveModuleState`**: Represents one module's target speed and angle.
*   **`SwerveModulePosition`**: Represents one module's actual distance and angle (used for Odometry).

---

## 3. REV Robotics (REVLib 2026 API)

### **Motor Controllers**
*   **`SparkMax`**: Standard for NEO/NEO 550.
*   **`SparkFlex`**: Standard for NEO Vortex.

### **SparkMaxConfig / SparkFlexConfig**
The 2026 "Rulebook" structure:
*   **`idleMode(IdleMode mode)`**:
    *   `kBrake`: Shorts the motor leads to resist movement.
    *   `kCoast`: Allows the motor to spin freely.
*   **`smartCurrentLimit(int limit)`**: Current limit in Amps.
    *   Shooter: 40A | Drive: 40A | Intake: 30A.
*   **`voltageCompensation(double volts)`**: Standardizes motor power regardless of battery level (usually set to 12.0V).
*   **`inverted(boolean)`**: Flips positive/negative direction.

### **Closed-Loop Control**
*   **`SparkClosedLoopController`**: The hardware-level PID processor.
    *   `setSetpoint(double value, ControlType type)`:
        *   `kVelocity`: Used for shooters/rollers (RPM).
        *   `kPosition`: Used for pivots/arms (Rotations).
*   **`closedLoop.p(val)` / `i(val)` / `d(val)`**: Feedback gains.
*   **`closedLoop.feedForward.kV(val)`**: The voltage needed to maintain a constant speed.

---

## 4. Control Theory & Filtering

### **PIDController**
`Output = P*error + I*sum(error) + D*change(error)`
*   **`setTolerance(double)`**: Defines when the robot is "close enough" to the target.
*   **`enableContinuousInput(min, max)`**: Used for rotation so the robot knows that -179° is close to 179°.

### **SlewRateLimiter**
*   Used to prevent "tipping" or wheel slip by limiting how fast the driver can accelerate.
*   `calculate(double input)`: Returns the smoothed value.

---

## 5. AdvantageKit (Deterministic Logging)

### **Tri-Layer Philosophy**
1.  **Subsystem**: High-level logic (e.g., `setShootVelocity`).
2.  **IO Interface**: Defines the `Inputs` class.
3.  **IO Implementation**: Handles the hardware (`IOSpark`) or virtual physics (`IOSim`).

### **Logger API**
*   **`Logger.processInputs(String path, Inputs inputs)`**: Synchronizes data between the RIO and the Log.
*   **`Logger.recordOutput(String key, T value)`**:
    *   Log poses: `Logger.recordOutput("Drive/Pose", pose)`.
    *   Log arrays: `Logger.recordOutput("Drive/ModuleStates", states)`.
*   **`AutoLog`**: Automatically generates logging boilerplate for any class it is placed on.

---

## 6. Shooter Physics & Math

### **`ShooterMath` Methods**
*   **`getDistanceToHub(Pose2d pose)`**: 
    *   `hypot(Robot.HubPose.X - pose.X, Robot.HubPose.Y - pose.Y)`.
*   **`calculateTrajectoryVelocity(double d)`**:
    *   `v = sqrt( (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - h)) )`.
*   **`calculateEnergyBoost(double velocity)`**:
    *   Calculates the energy needed to accelerate the ball mass ($m$) using the wheels' inertia ($I$).
    *   `delta_omega = sqrt( (m * v^2) / (I * efficiency) )`.
*   **`velocityToRPM(double v)`**:
    *   `RPM = (v / wheelRadius) * (60 / 2pi) * beltRatio`.

---

## 7. Subsystem Telemetry Map

### **Drive Subsystem**
*   `drivePositionRad`: Accumulated distance of the wheel.
*   `driveVelocityRadPerSec`: Instantaneous speed.
*   `turnPosition`: Current module angle (Rotation2d).

### **Shooter Subsystem**
*   `shootVelocityRPM`: Real-time speed of the launch wheels.
*   `atTargetRPM(target)`: Checks if `abs(actual - target) < 100`.

### **Intake Subsystem**
*   `pivotPositionRad`: Current arm angle.
*   `rollerVelocityRPM`: Speed of the pickup wheels.

---

## 8. Vision Requirements

### **Custom NetworkTables (NT4) API**
The Raspberry Pi must publish to these keys:
1.  **`/CustomVision/targetPose`**: [X, Y, Z, Roll, Pitch, Yaw].
2.  **`/CustomVision/latency`**: Processing time in ms.
3.  **`/CustomVision/targetId`**: The ID of the AprilTag seen.

**Important:** The Pi MUST convert inches to **Meters** before sending to avoid RIO overhead.

---

## 9. Spark Swerve Infrastructure (Template)

*   **`SparkOdometryThread`**: Polling sensors at **100Hz** to minimize pose lag.
*   **`SparkUtil.tryUntilOk()`**: Safe configuration delivery.
*   **`SparkUtil.ifOk()`**: Signal validation to prevent data corruption.

---

## 10. Standards & Units
*   **Distance**: Meters ($m$).
*   **Angle**: Radians ($rad$).
*   **Time**: Seconds ($s$).
*   **Mass**: Kilograms ($kg$).
*   **Naming**: `lowerCamelCase` for variables/methods.
