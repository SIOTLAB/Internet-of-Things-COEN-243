
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
         // Create a new server socket that listens at port 6999
         ServerSocket ss = new ServerSocket(6999);
         System.out.println("Server is lisenting ... ");
         Socket skt = ss.accept();//wait for connection from client on port 6999
         InputStream is = skt.getInputStream();
         ObjectInputStream ois = new ObjectInputStream(is);
         String msg = (String)ois.readObject(); // recieve message from client
         System.out.println("Server recieved message : " + msg);
         System.out.println("Server is exiting ... ");
         /* close open streams and socket */
         ois.close();
         is.close();
         skt.close();
         ss.close();
      } catch(Exception e) {
         System.out.println(e);
      }
   }
}


