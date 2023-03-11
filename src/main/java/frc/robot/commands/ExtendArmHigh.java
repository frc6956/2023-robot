package frc.robot.commands;

import frc.robot.subsystems.Extension;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.Rotation;


public class ExtendArmHigh extends ExtendArm{
    private Extension extension;
    //creates a new ExtendArmHigh
    public ExtendArmHigh(final Extension extension){
        super(extension, 149);

        addRequirements(extension);
        this.extension=extension;

        //use addRequirements() here to declare susbsystem dependencies
    }

    //called when the command is intially scheduled.
    @Override
    public void initialize(){}

    //called everytime the scheduler runs while the command is scheduled
    @Override
    public void execute(){
        super.execute();
    }

    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted){}

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        return false;
    }
}
