package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.AbstractMechanism;
import frc.Mechanisms.CatzMathUtils;

@SuppressWarnings("unused")
public class CatzIntakeWrist extends AbstractMechanism
{
    private static final int PIVOT_THREAD_PERIOD_MS = 100;

    private final int     PIVOT_MOTOR_CAN_ID         = 0;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;
    
    public enum WristPosID
    {
        //dummy angle values
        STW(0.0, "stow"),
        POS1(1.0, "dummy position 1"),
        POS2(2.0, "dummy position 2"),
        POS3(3.0, "dummy position 3"),
        NON(0.0, "null");
        
        public final double angle;
        public final String name;
        private WristPosID(double angle, String name)
        {
            this.angle = angle;
            this.name = name;
        }
    }

    private final double DEGREES_TO_ENC = 69.69; //dummy value

    private final double MAX_POWER = 0.8;
    private final double MIN_POWER = 0.05;
    private final double DECEL_DIST_DEGREE = 10.0;
    private final double POWER_GAIN_PER_DEGREE = MAX_POWER/DECEL_DIST_DEGREE;

    private final double DEADBAND_RADIUS_DEGREE = 1;
    private final double DEADBAND_RADIUS_ENC  = DEADBAND_RADIUS_DEGREE * DEGREES_TO_ENC;

    private final double PIVOT_MOTOR_MANUAL_EXT_POWER = 0.5;

    public volatile WristPosID targetPos = WristPosID.STW;
    public WristPosID currentPos = WristPosID.NON;

    private double distanceRemainingEnc;
    private double currentPositionEnc;
    private double targetPower;
    
    private WPI_TalonFX pivotMotor;
    private SupplyCurrentLimitConfiguration pivotMotorCurrentLimit;

    public CatzIntakeWrist()
    {
        super(PIVOT_THREAD_PERIOD_MS);

        pivotMotorCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        pivotMotor = new WPI_TalonFX(PIVOT_MOTOR_CAN_ID);
        pivotMotor.configFactoryDefault();
        pivotMotor.configSupplyCurrentLimit(pivotMotorCurrentLimit);

        pivotMotor.setSelectedSensorPosition(0.0);
        pivotMotor.setNeutralMode(NeutralMode.Brake);

        start();
    }

    @Override
    public void update()
    {
        currentPositionEnc = pivotMotor.getSelectedSensorPosition();
        distanceRemainingEnc = targetPos.angle * DEGREES_TO_ENC - currentPositionEnc;

        if(Math.abs(distanceRemainingEnc) <= DEADBAND_RADIUS_ENC)
        {
            currentPos = targetPos;
            distanceRemainingEnc = 0.0;
        }
        else
        {
            currentPos = WristPosID.NON;
        }

        targetPower = CatzMathUtils.clamp(distanceRemainingEnc * POWER_GAIN_PER_DEGREE, MIN_POWER, MAX_POWER);
        pivotMotor.set(ControlMode.PercentOutput, targetPower);
    }

    public void setPos(WristPosID pos)
    {
        targetPos = pos;
    }

    public void pivotMotorManual(double direction)
    {
        pivotMotor.set(ControlMode.PercentOutput, PIVOT_MOTOR_MANUAL_EXT_POWER * Math.signum(direction));
    }

    @Override
    public void smartDashboard()
    {
        SmartDashboard.putString("Intake Wrist Current Position", currentPos.name);
    }
    
    @Override
    public void smartDashboard_DEBUG()
    {
        SmartDashboard.putString("Intake Wrist Target Position", targetPos.name);
        SmartDashboard.putNumber("Intake Wrist Encoder", currentPositionEnc);
        SmartDashboard.putNumber("Target Power", targetPower);
    }
}