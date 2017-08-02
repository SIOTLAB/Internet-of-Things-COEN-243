
//------------------------------------------------------------
//SCU's Internet of Things Research Lab (SIOTLAB)
//Santa Clara University (SCU)
//Santa Clara, California
//------------------------------------------------------------


import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
import java.io.*;
import java.net.*;
import java.util.Timer;
//import java.util.TimerTask;

public class Server {

    public static void main(String args[]) {

        // create gpio controller
        final GpioController gpio = GpioFactory.getInstance();

        // provision gpio pin #01 as an output pin and turn on
        final GpioPinDigitalOutput pin = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_01, "MyLED", PinState.HIGH);

        Timer timer = new Timer();
        Reminder reminderTask = new Reminder(pin);

        try {

            ServerSocket ss = new ServerSocket(6999);
            System.out.println("Server is lisenting ... ");
            Socket skt = ss.accept();

            OutputStream os = skt.getOutputStream();
            ObjectOutputStream oos = new ObjectOutputStream(os);
            InputStream is = skt.getInputStream();
            ObjectInputStream ois = new ObjectInputStream(is);

            BufferedReader br = new BufferedReader(new InputStreamReader(System.in));

            int rcv_msg;
            String send_msg;
            timer.schedule(reminderTask, 0, 1000);

            while (true) {
                /* Recieve a message from client */
                if ((rcv_msg = (int) ois.readObject()) != 0) {
                    System.out.println("New interval : " + rcv_msg);
                }

                reminderTask.cancel();
                //reminderTask = new Reminder(pin);
                timer.schedule(reminderTask, 0, (long) rcv_msg);

                oos.writeObject("Interval changed!");
            }
        } catch (Exception e) {
            System.out.println(e);
        }
    }
}
