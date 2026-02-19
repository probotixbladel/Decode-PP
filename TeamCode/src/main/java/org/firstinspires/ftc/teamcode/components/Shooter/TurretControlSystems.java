package org.firstinspires.ftc.teamcode.components.Shooter;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.common.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.common.Constants.GeneralConstants;

import java.util.function.DoubleSupplier;

@Configurable
public class TurretControlSystems extends PositionVelocitySystem {
    private final DoubleSupplier motorPositionSupplier;
    private final DoubleSupplier motorVelocitySupplier;
    
    private double lastPOS_KAL_Q, lastPOS_KAL_R, lastVEL_KAL_Q, lastVEL_KAL_R;
    private double lastFF_KS, lastFF_KV, lastFF_KA;
    private double lastPOS_PID_KP, lastPOS_PID_KI, lastPOS_PID_KD;
    private double lastVEL_PID_KP, lastVEL_PID_KI, lastVEL_PID_KD;

    public TurretControlSystems(DoubleSupplier motorPosition, DoubleSupplier motorVelocity) {
        super(
                new KalmanEstimator(
                        motorPosition,
                        TurretConstants.POS_KAL_Q,
                        TurretConstants.POS_KAL_R,
                        TurretConstants.POS_KAL_N
                ),
                new KalmanEstimator(
                        motorVelocity,
                        TurretConstants.VEL_KAL_Q,
                        TurretConstants.VEL_KAL_R,
                        TurretConstants.VEL_KAL_N
                ),
                new BasicFeedforward(new FeedforwardCoefficients(
                        TurretConstants.FF_KS,
                        TurretConstants.FF_KV,
                        TurretConstants.FF_KA
                )),
                new BasicPID(new PIDCoefficients(
                        TurretConstants.POS_PID_KP,
                        TurretConstants.POS_PID_KI,
                        TurretConstants.POS_PID_KD
                )),
                new BasicPID(new PIDCoefficients(
                        TurretConstants.VEL_PID_KP,
                        TurretConstants.VEL_PID_KI,
                        TurretConstants.VEL_PID_KD
                ))
        );

        this.motorPositionSupplier = motorPosition;
        this.motorVelocitySupplier = motorVelocity;
        
        lastPOS_KAL_Q = TurretConstants.POS_KAL_Q;
        lastPOS_KAL_R = TurretConstants.POS_KAL_R;
        lastVEL_KAL_Q = TurretConstants.VEL_KAL_Q;
        lastVEL_KAL_R = TurretConstants.VEL_KAL_R;
        lastFF_KS = TurretConstants.FF_KS;
        lastFF_KV = TurretConstants.FF_KV;
        lastFF_KA = TurretConstants.FF_KA;
        lastPOS_PID_KP = TurretConstants.POS_PID_KP;
        lastPOS_PID_KI = TurretConstants.POS_PID_KI;
        lastPOS_PID_KD = TurretConstants.POS_PID_KD;
        lastVEL_PID_KP = TurretConstants.VEL_PID_KP;
        lastVEL_PID_KI = TurretConstants.VEL_PID_KI;
        lastVEL_PID_KD = TurretConstants.VEL_PID_KD;
    }

    public void checkAndUpdate() {
        if (!GeneralConstants.DEBUG) return;
        
        if (TurretConstants.POS_KAL_Q != lastPOS_KAL_Q ||
            TurretConstants.POS_KAL_R != lastPOS_KAL_R ||
            TurretConstants.VEL_KAL_Q != lastVEL_KAL_Q ||
            TurretConstants.VEL_KAL_R != lastVEL_KAL_R ||
            TurretConstants.FF_KS != lastFF_KS ||
            TurretConstants.FF_KV != lastFF_KV ||
            TurretConstants.FF_KA != lastFF_KA ||
            TurretConstants.POS_PID_KP != lastPOS_PID_KP ||
            TurretConstants.POS_PID_KI != lastPOS_PID_KI ||
            TurretConstants.POS_PID_KD != lastPOS_PID_KD ||
            TurretConstants.VEL_PID_KP != lastVEL_PID_KP ||
            TurretConstants.VEL_PID_KI != lastVEL_PID_KI ||
            TurretConstants.VEL_PID_KD != lastVEL_PID_KD) {
            reinitialize();
            
            lastPOS_KAL_Q = TurretConstants.POS_KAL_Q;
            lastPOS_KAL_R = TurretConstants.POS_KAL_R;
            lastVEL_KAL_Q = TurretConstants.VEL_KAL_Q;
            lastVEL_KAL_R = TurretConstants.VEL_KAL_R;
            lastFF_KS = TurretConstants.FF_KS;
            lastFF_KV = TurretConstants.FF_KV;
            lastFF_KA = TurretConstants.FF_KA;
            lastPOS_PID_KP = TurretConstants.POS_PID_KP;
            lastPOS_PID_KI = TurretConstants.POS_PID_KI;
            lastPOS_PID_KD = TurretConstants.POS_PID_KD;
            lastVEL_PID_KP = TurretConstants.VEL_PID_KP;
            lastVEL_PID_KI = TurretConstants.VEL_PID_KI;
            lastVEL_PID_KD = TurretConstants.VEL_PID_KD;
        }
    }

    public void reinitialize() {
        this.positionEstimator = new KalmanEstimator(
                motorPositionSupplier,
                TurretConstants.POS_KAL_Q,
                TurretConstants.POS_KAL_R,
                TurretConstants.POS_KAL_N
        );
        this.velocityEstimator = new KalmanEstimator(
                motorVelocitySupplier,
                TurretConstants.VEL_KAL_Q,
                TurretConstants.VEL_KAL_R,
                TurretConstants.VEL_KAL_N
        );
        this.feedforward = new BasicFeedforward(new FeedforwardCoefficients(
                TurretConstants.FF_KS,
                TurretConstants.FF_KV,
                TurretConstants.FF_KA
        ));
        this.positionFeedback = new BasicPID(new PIDCoefficients(
                TurretConstants.POS_PID_KP,
                TurretConstants.POS_PID_KI,
                TurretConstants.POS_PID_KD
        ));
        this.velocityFeedback = new BasicPID(new PIDCoefficients(
                TurretConstants.VEL_PID_KP,
                TurretConstants.VEL_PID_KI,
                TurretConstants.VEL_PID_KD
        ));
    }
}
