package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDH extends SubsystemBase {
    private PowerDistribution power = new PowerDistribution(0, ModuleType.kRev);
    
    public PDH() {

    }
}
