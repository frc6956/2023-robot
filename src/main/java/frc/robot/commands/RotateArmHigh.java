package frc.robot.commands;

import frc.robot.subsystems.Rotation;

//import javax.swing.text.StyledEditorKit.BoldAction;

//import frc.robot.subsystems.Extension;
//import edu.wpi.first.wpilibj2.command.CommandBase;


public class RotateArmHigh extends RotateArm{
    public RotateArmHigh(Rotation rotation){
        //Use addRequirements() here to declare subsystem dependencies
        //Subject to change
        super(rotation, 0);
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
