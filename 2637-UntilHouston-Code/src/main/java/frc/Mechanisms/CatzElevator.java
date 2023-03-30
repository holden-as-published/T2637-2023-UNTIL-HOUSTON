
package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.*;

@SuppressWarnings("unused")
public class CatzElevator extends AbstractMechanism
{
    public enum PosID
    {
        TOP(4, "Top"),
        MID(3, "Mid"),
        LOW(2, "Low"),
        STOW(1, "Stow"),
        PICKUP(4, "Pickup"),
        NULL(999, "none");
        
        private final double inch;
        private final String name;

        private PosID(double inch, String name)
        {
            this.inch = inch;
            this.name = name;
        }
    }
        
    private static final int ELEVATOR_THREAD_PERIOD_MS = 100;

    private final int MID_LIMIT_SWITCH_CHANNEL = 1;

    private final int     ELEVATOR_MOTOR_CAN_ID         = 0;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final double ENC_COUNTS_TO_INCH = 1/69; //temp
    private final double INCH_TO_ENC_COUNTS  = 69/1;

    private final double DECEL_DIST_INCH = 10.0;

    private final double MAX_POWER = 0.8;
    private final double MIN_POWER = 0.05;

    private final double DEADBAND_RADIUS_INCH = 1;
    private final double DEADBAND_RADIUS_ENC  = DEADBAND_RADIUS_INCH * INCH_TO_ENC_COUNTS;

    private final double POWER_GAIN_PER_INCH = MAX_POWER/DECEL_DIST_INCH;

    private final double ELEVATOR_MOTOR_MANUAL_EXT_POWER = 0.5;

    private volatile PosID targetPos = PosID.LOW;
    private PosID currentPos = PosID.NULL;//to be used in smart dashboard

    private DigitalInput midLimitSwitch;

    private double distanceRemainingEnc;
    private double currentPositionEnc;
    private double targetPower;
    
    private WPI_TalonFX elevatorMotor;
    private SupplyCurrentLimitConfiguration elevatorMotorCurrentLimit;

    private final double P_VALUE = 0.1;
    private final double I_VALUE = 0.0;
    private final double D_VALUE = 0.0;

    private final double MAX_ACCEL = 99999.99;
    private final double MAX_SPEED = 99999.99;

    private boolean manualMode = false;

    public CatzElevator()
    {
        super(ELEVATOR_THREAD_PERIOD_MS);

        midLimitSwitch = new DigitalInput(MID_LIMIT_SWITCH_CHANNEL);

        elevatorMotorCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        elevatorMotor = new WPI_TalonFX(ELEVATOR_MOTOR_CAN_ID);
        elevatorMotor.configFactoryDefault();

        elevatorMotor.configSupplyCurrentLimit(elevatorMotorCurrentLimit);
        elevatorMotor.setSelectedSensorPosition(0.0);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);

        elevatorMotor.config_kP(0, P_VALUE);
        elevatorMotor.config_kP(0, I_VALUE);
        elevatorMotor.config_kP(0, D_VALUE);

        elevatorMotor.configMotionAcceleration(9999.0); //value determined using WAG method
        elevatorMotor.configMotionCruiseVelocity(9999.0);
        elevatorMotor.configMotionSCurveStrength(2); 


        

            start(); 

    }

    @Override
    public void update()
    {
        if (!manualMode && targetPos!=PosID.NULL) //this should prevent a null target position from being commanded, but may prevent command interruption 
        {
            elevatorMotor.set(ControlMode.MotionMagic, (targetPos.inch*INCH_TO_ENC_COUNTS));
        }

    }


    public void setElevatorTargetPose(PosID pos)
    {
        targetPos = pos;
    }

    public void elevatorMotorManual(double direction)
    {
        if (Math.abs(direction)>0.1 || manualMode == true)
        {
            manualMode = true;
            elevatorMotor.set(ControlMode.PercentOutput, ELEVATOR_MOTOR_MANUAL_EXT_POWER * Math.signum(direction));

            if (Math.abs(direction)<0.1)
            {
                manualMode = false;
            }
        }
    }

    @Override
    public void smartDashboard()
    {
        SmartDashboard.putString("Elevator Current Pos", currentPos.name);
    }

    @Override
    public void smartDashboard_DEBUG()
    {
        SmartDashboard.putNumber("Elevator Target Power", targetPower);
        SmartDashboard.putString("Elevator Target Pos", targetPos.name);
        SmartDashboard.putNumber("Elevator Distance Remaining", distanceRemainingEnc * ENC_COUNTS_TO_INCH);
    }
}


    
