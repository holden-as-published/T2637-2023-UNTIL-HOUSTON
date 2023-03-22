
package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class CatzArm extends AbstractMechanism
{
    private static final int ARM_THREAD_PERIOD_MS = 100;

    private final int     ARM_MOTOR_CAN_ID         = 0;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;
    
    public enum PosID
    {
        STW(0), // stow
        LOW(1), // low
        MCN(2), // mid cone
        MCB(3), // mid cube
        TCN(4), // top cone
        TCB(5), // top cube
        NON(6); // none
        
        public final int id;
        private PosID(int id){
            this.id = id;
        }
    }

    //dummy values
    public final double[] POS_INCH_LIST = 
    {
        0.0, //stow
        1.0, //low
        2.0, //mid cone
        3.0, //mid cube
        4.0, //top cone
        5.0, //top cube
    };

    private final double INCH_TO_ENC = 69.69; //dummy value

    private final double MAX_POWER = 0.8;
    private final double MIN_POWER = 0.05;
    private final double DECEL_DIST_INCH = 10.0;
    private final double POWER_GAIN_PER_INCH = MAX_POWER/DECEL_DIST_INCH;

    private final double DEADBAND_RADIUS_INCH = 1;
    private final double DEADBAND_RADIUS_ENC  = DEADBAND_RADIUS_INCH * INCH_TO_ENC;

    private final double ARM_MOTOR_MANUAL_EXT_POWER = 0.5;

    public volatile PosID targetPos = PosID.STW;
    private PosID currentPos = PosID.NON;

    private double distanceRemainingEnc;
    private double currentPositionEnc;
    private double targetPower;
    
    private WPI_TalonFX armMotor;
    private SupplyCurrentLimitConfiguration armMotorCurrentLimit;

    public CatzArm()
    {
        super(ARM_THREAD_PERIOD_MS);

        armMotorCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        armMotor = new WPI_TalonFX(ARM_MOTOR_CAN_ID);
        armMotor.configFactoryDefault();
        armMotor.configSupplyCurrentLimit(armMotorCurrentLimit);

        armMotor.setSelectedSensorPosition(0.0);
        armMotor.setNeutralMode(NeutralMode.Brake);

        start();
    }

    @Override
    public void update()
    {
        currentPositionEnc = armMotor.getSelectedSensorPosition();
        distanceRemainingEnc = POS_INCH_LIST[targetPos.id] * INCH_TO_ENC - currentPositionEnc;

        if(Math.abs(distanceRemainingEnc) <= DEADBAND_RADIUS_ENC)
        {
            currentPos = targetPos;
            distanceRemainingEnc = 0.0;
        }
        else
        {
            currentPos = PosID.NON;
        }

        targetPower = CatzMathUtils.clamp(distanceRemainingEnc * POWER_GAIN_PER_INCH, MIN_POWER, MAX_POWER);
        armMotor.set(ControlMode.PercentOutput, targetPower);
    }

    public void setPos(PosID pos)
    {
        targetPos = pos;
    }

    public void armMotorManual(double direction)
    {
        armMotor.set(ControlMode.PercentOutput, ARM_MOTOR_MANUAL_EXT_POWER * Math.signum(direction));
    }


    public void smartDashboard()
    {
        SmartDashboard.putNumber("Arm Encoder", currentPositionEnc);
        SmartDashboard.putNumber("Arm Target Position", targetPos.id);
    }
    

    public void smartDashboard_DEBUG()
    {
        SmartDashboard.putNumber("Arm Current Position", currentPos.id);
    }
}
