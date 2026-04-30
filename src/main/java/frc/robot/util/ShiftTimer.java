package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class ShiftTimer {
    
public Timer timer = new Timer();

public ShiftTimer() {}

public int minutes(){
    return (int) Math.floor(timer.getMatchTime());
}

public int seconds(){
    return (int) Math.round(100 * (timer.getMatchTime() - minutes()));
}

public String time(){
    return Integer.toString(minutes()) + " : " + Integer.toString(seconds());
}

}
