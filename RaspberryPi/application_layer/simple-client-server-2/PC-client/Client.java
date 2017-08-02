
//------------------------------------------------------------
//SCU's Internet of Things Research Lab (SIOTLAB)
//Santa Clara University (SCU)
//Santa Clara, California
//------------------------------------------------------------


import java.io.*;
import java.net.*;

public class Client {
    public static void main(String args[]) {
        try {
            // create a new socket for communicating with the server
            Socket skt = new Socket("192.168.1.138", 6999);
            System.out.println("Connected to server");
            InputStream is = skt.getInputStream();
            ObjectInputStream ois = new ObjectInputStream(is);
            String msg = (String)ois.readObject(); // recieve a message from server
            System.out.println("Recieved message from Server : " + msg);
         /* close open streams and sockets */
            ois.close();
            is.close();
            skt.close();
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}

