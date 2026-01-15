package Team4450.Robot26.utility;

import Team4450.Lib.Util;

public class ConsoleEveryX {
    public int x = 0;
    public int targetX;

    public ConsoleEveryX(int x) {
        this.x = 0;
        this.targetX = x;
    }

    public void update(String text) {
        this.x++;
        if (this.x == this.targetX) {
            Util.consoleLog(text);
            this.x = 0;
        }

    }
}
