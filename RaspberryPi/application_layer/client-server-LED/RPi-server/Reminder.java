
//------------------------------------------------------------
//SCU's Internet of Things Research Lab (SIOTLAB)
//Santa Clara University (SCU)
//Santa Clara, California
//------------------------------------------------------------


import java.util.Timer;
import java.util.TimerTask;
import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
/**
 * Simple demo that uses java.util.Timer to schedule a task 
 * to execute once 5 seconds have passed.
 */

public class Reminder extends TimerTask {
    
    GpioPinDigitalOutput pin;

    Reminder (GpioPinDigitalOutput pin) {
        this.pin = pin;
    }
    
    public void run() {
        pin.toggle();
    }

}