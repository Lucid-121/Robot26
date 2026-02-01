package Team4450.Robot26.utility;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ConsoleEveryX {
    private int x;
    private int targetX;
    private boolean enabled;
    private String id;

    public ConsoleEveryX(String id, int x) {
        this.x = 0;
        this.targetX = x;
        this.enabled = true;
        this.id = id;
        SmartDashboard.putBoolean(id, enabled);
    }

    public void update(String text) {
        this.enabled = SmartDashboard.getBoolean(this.id, this.enabled);
        if (this.enabled) {
            this.x++;
            if (this.x == this.targetX) {
                Util.consoleLog(text);
                this.x = 0;
            }
        } else {
            return;
        }
    }

    public void enable() {
        this.enabled = true;
    }

    public void disable() {
        this.enabled = false;
    }

    public void toggle() {
        this.enabled = !this.enabled;
    }
}
