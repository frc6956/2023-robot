package frc.robot.commands;

import frc.robot.subsystems.Extension;
//import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendArmLow extends ExtendArm{
    private Extension extension;

    public ExtendArmLow(final Extension extension){
        super(extension, 100);
        addRequirements(extension);
        this.extension=extension;
        //Use addRequirements() here to declare subsystem dependencies
    }

    //Called when the command is initially scheduled
    @Override
    public void initialize() {}
    
    //called every time the scheduler runs while the command is scheduled
    @Override
    public void execute(){
        super.execute();
    }

    //Called oncce the command ends or is interrupred
    @Override
    public void end(boolean interrupted){}

    //Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return false;
    }
}