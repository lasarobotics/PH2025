package frc.robot;
import edu.wpi.first.wpilibj.PWM;

public class Led {
    public static record Hardware (
        PWM pwm
    ) {}

    private static Led s_Led;
    private static PWM m_pwm;


    private Led(Hardware ledHardware) {
        m_pwm = ledHardware.pwm;
    }

    public static Led getInstance(Hardware ledHardware) {
        if(s_Led == null){
            s_Led = new Led(ledHardware);
            return s_Led;
        } else {
            return null;
        }

    }

    public static Hardware initializHardware() {
        Hardware ledHardware = new Hardware(
            new PWM(Constants.LedHardware.PWM_PORT)
        );
        return ledHardware;
    }

    public void setViolet() {
        m_pwm.setSpeed(0.91);
    }

    public void setRed() {
        m_pwm.setSpeed(0.61);
    }

    public void setBlue() {
        m_pwm.setSpeed(0.87);
    }

    public void setWhite() {
        m_pwm.setSpeed(0.93);
    }

    public void setGreen() {
        m_pwm.setSpeed(0.77 );
    }

    public void setRainbow() {
        m_pwm.setSpeed(-0.99);
    }

    public void setYellow() {
        m_pwm.setSpeed(0.69);
    }
}
