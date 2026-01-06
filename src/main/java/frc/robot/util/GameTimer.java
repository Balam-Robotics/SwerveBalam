package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

public class GameTimer implements Sendable {

    public GameTimer() {
    }

    @Override  
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("GameTimer");
        builder.addDoubleProperty("Match Time", Timer::getMatchTime, null);
    }
}
