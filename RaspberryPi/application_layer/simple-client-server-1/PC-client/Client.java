
//------------------------------------------------------------
//SCU's Internet of Things Research Lab (SIOTLAB)
//Santa Clara University (SCU)
//Santa Clara, California
//------------------------------------------------------------


import java.io.*; import java.net.*; import java.io.BufferedReader;

public class Client2Pi {
    public static void main(String args[]) {
        try {
            // create a new socket for communicating with the server
            Socket skt = new Socket("192.168.1.138", 6999);
            System.out.println("Connected to server");

            BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
            String accStr;
            System.out.println("Enter text: ");
            String msg = br.readLine();

            OutputStream os = skt.getOutputStream();
            ObjectOutputStream oos = new ObjectOutputStream(os);
            oos.writeObject(msg); // send a message to server
            System.out.println("Sent a message to server");
         /* close open streams and sockets */
            oos.close();
            os.close();
            skt.close();
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}


