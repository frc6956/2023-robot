package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rotation;

public class RotateArmLow extends RotateArm{
    Rotation rotation;
    public RotateArmLow(Rotation rotation){
        //Use addRequirements() here to declare subsystem dependencies
        //Subject to change
        super(rotation, 80);
        addRequirements(rotation);

        this.rotation=rotation;
    }

    //Called when the command isinitially scheduled
    @Override
    public void initialize(){}

    @Override
    public void execute(){
        super.execute();
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}