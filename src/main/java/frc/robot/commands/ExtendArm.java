package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;


public class ExtendArm extends CommandBase{
    Extension extension;
    double position;

    public ExtendArm(final Extension extension, final double newPosition){
        position=newPosition;
        this.extension=extension;

    }



    @Override
    public void initialize(){}

    //Xalled evvery time the scheduler tuns while th ecommand is scheduled
    @Override 
    public void execute(){
        if (extension.getExtensionAveragePosition()>position+1){
            extension.extendArm(0.3);
        }
        else if (extension.getExtensionAveragePosition()<position-1){
            extension.extendArm(-0.3);
        }
        else{
            extension.extendArm(0);
        }
    }

    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted){
        extension.extendArm(0);
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        /*if(position>0){
            if (extension.getExtensionAveragePosition()>=position){
                return true;
            }
            else{
                return false;
            }

        }
        else{
            if (extension.getExtensionAveragePosition()<=position){
                return true;
            }
            else{
                return false;
            }
            
        }*/
        //changed to possibly be better. TBD
        if((position>0)&&(extension.getExtensionAveragePosition()>=position)){
            return true;
        }
        else if ((position<=0)&&(extension.getExtensionAveragePosition()<=position)){
            return true;
        }
        else{
            return false;
        }
    }

}