
//------------------------------------------------------------
//SCU's Internet of Things Research Lab (SIOTLAB)
//Santa Clara University (SCU)
//Santa Clara, California
//------------------------------------------------------------


import java.io.*;
import java.net.*;

public class Server {
   public static void main(String args[]) {
      try {
         // Create a new server socket that listens at port 7999
         ServerSocket ss = new ServerSocket(6999);
         System.out.println("Server is lisenting ... ");
         Socket skt = ss.accept();//accept a connection from client on port 7999
         OutputStream os = skt.getOutputStream();
         ObjectOutputStream oos = new ObjectOutputStream(os);
         String msg = "Sensor data returned!";
         oos.writeObject(msg); // send a message to client
         System.out.println("Sent a message to client");
         System.out.println("Server is exiting ... ");
         /* close open streams and sockets */
         oos.close();
         os.close();
         skt.close();
         ss.close();
      } catch(Exception e) {
         System.out.println(e);
      }
   }
}
