package frc.team670.robot.commands.vision;

import java.util.Map;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;

public class AutoAlignSelector extends SelectCommand {
    private enum SELECT {
        LEFT,
        CLOSEST,
        RIGHT
    }

    public AutoAlignSelector(Map<Object, Command> commands, Supplier<Object> selector) {
        
        super(commands, selector);
        //TODO Auto-generated constructor stub
    }
    
}
