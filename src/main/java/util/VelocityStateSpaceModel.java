package util;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.rivierarobotics.lib.MathUtil;

/**
 * State-Space Control System using the FRC Characterization method - kS kV and kA
 */
public class VelocityStateSpaceModel {
    private final LinearSystemLoop<N1, N1, N1> linearSystemLoop;
    private final SystemIdentification systemIdentification;
    private double targetVelocity;
    private final double loopTime;

    /**
     * State-Space Position Control System using the FRC Characterization method - kV and kA
     *
     * @param systemIdentification   kS (static friction) kV (volts/units/s) kA (volts/units/s*s)
     * @param velocityAccuracy       how accurate we think our velocity model is (higher is more aggressive)
     * @param velocityErrorTolerance how tolerable we are to velocity error (lower is more aggressive)
     * @param voltageControlEffort   best left at 12. decrease this to penalize control effort (using more voltage)
     * @param maxVoltage             the maximum voltage to apply to the motor (use this to limit speed)
     */
    public VelocityStateSpaceModel(SystemIdentification systemIdentification,
                                   double velocityAccuracy, double encoderAccuracy,
                                   double velocityErrorTolerance, double voltageControlEffort,
                                   double maxVoltage) {
        this(systemIdentification, velocityAccuracy, encoderAccuracy,
                velocityErrorTolerance, voltageControlEffort, maxVoltage, 0.02);
    }

    /**
     * State-Space Position Control System using the FRC Characterization method - kV and kA
     *
     * @param systemIdentification   kS (static friction) kV (volts/units/s) kA (volts/units/s*s)
     * @param velocityAccuracy       how accurate we think our velocity model is (higher is more aggressive)
     * @param velocityErrorTolerance how tolerable we are to velocity error (lower is more aggressive)
     * @param voltageControlEffort   best left at 12. decrease this to penalize control effort (using more voltage)
     * @param maxVoltage             the maximum voltage to apply to the motor (use this to limit speed)
     * @param loopTime               if you want to run the system faster than the default loop time of 20ms, use this
     */
    public VelocityStateSpaceModel(SystemIdentification systemIdentification,
                                   double velocityAccuracy, double encoderAccuracy,
                                   double velocityErrorTolerance, double voltageControlEffort,
                                   double maxVoltage, double loopTime) {
        this.loopTime = loopTime;
        this.systemIdentification = systemIdentification;

        LinearSystem<N1, N1, N1> linearPositionSystem =
                LinearSystemId.identifyVelocitySystem(systemIdentification.kV, systemIdentification.kA);

        KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                Nat.N1(), Nat.N1(),
                linearPositionSystem,
                VecBuilder.fill(velocityAccuracy),
                VecBuilder.fill(encoderAccuracy),
                loopTime
        );

        LinearQuadraticRegulator<N1, N1, N1> controller
                = new LinearQuadraticRegulator<>(linearPositionSystem,
                VecBuilder.fill(velocityErrorTolerance),
                VecBuilder.fill(voltageControlEffort),
                loopTime
        );

        this.linearSystemLoop = new LinearSystemLoop<>(
                linearPositionSystem,
                controller,
                observer,
                maxVoltage,
                loopTime
        );
    }

    public void setVelocity(double unitsPerS) {
        linearSystemLoop.setNextR(unitsPerS);
        targetVelocity = unitsPerS;
    }

    public boolean isWithinTolerance(double unitsPerS, double tolerance) {
        return MathUtil.isWithinTolerance(unitsPerS, targetVelocity, tolerance);
    }

    public double getAppliedVoltage(double unitsPerS) {
        linearSystemLoop.correct(VecBuilder.fill(unitsPerS));
        linearSystemLoop.predict(loopTime);
        return linearSystemLoop.getU(0) + systemIdentification.kS;
    }
}
