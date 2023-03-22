package frc.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.AbstractMechanism;
import frc.Mechanisms.CatzIntakeElbow.ElbowPosID;
import frc.Mechanisms.CatzIntakeWrist.WristPosID;

public class CatzIntake extends AbstractMechanism 
{
    public enum IntakePosID
    {
        //dummy pivot ids
        STW(ElbowPosID.STW, WristPosID.STW, "stow"),
        POS1(ElbowPosID.POS1, WristPosID.POS1, "dummy position 1"),
        POS2(ElbowPosID.POS2, WristPosID.POS2, "dummy position 2"),
        POS3(ElbowPosID.POS3, WristPosID.POS3, "dummy position 3"),
        NON(ElbowPosID.NON, WristPosID.NON, "null");
        
        public final ElbowPosID elbowPosID;
        public final WristPosID wristPosID;
        public final String name;
        private IntakePosID(ElbowPosID elbowPosID, WristPosID wristPosID, String name)
        {
            this.elbowPosID = elbowPosID;
            this.wristPosID = wristPosID;
            this.name = name;
        }
    }

    public static final int INTAKE_THREAD_PERIOD_MS = 100;

    public volatile IntakePosID targetPos = IntakePosID.STW;
    public IntakePosID currentPos = IntakePosID.NON;

    public CatzIntakeElbow elbow;
    public CatzIntakeWrist wrist;

    public CatzIntake()
    {
        super(INTAKE_THREAD_PERIOD_MS);

        elbow = new CatzIntakeElbow();
        wrist = new CatzIntakeWrist();

        start();
    }

    @Override
    public void update()
    {
        if(elbow.currentPos != ElbowPosID.NON && wrist.currentPos != WristPosID.NON){
            currentPos = targetPos;
        }
        else
        {
            currentPos = IntakePosID.NON;
        }
    }

    @Override
    public void smartDashboard()
    {
        SmartDashboard.putString("Intake Current Position", currentPos.name);
    }
    
    @Override
    public void smartDashboard_DEBUG()
    {
        SmartDashboard.putString("Intake Target Position", targetPos.name);
    }
}