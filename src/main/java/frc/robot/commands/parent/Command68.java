package frc.robot.commands.parent;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Command68 extends CommandBase{

    public int actionId = -1;

    public boolean isFinished = false;

    // Don't set an action id to -1 becacuse that would be bad.
    public Command68(int actionId){
        this.actionId = actionId;
    }

    public Command68(){

    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
    
}
