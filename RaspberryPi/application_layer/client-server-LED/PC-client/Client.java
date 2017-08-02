
//------------------------------------------------------------
//SCU's Internet of Things Research Lab (SIOTLAB)
//Santa Clara University (SCU)
//Santa Clara, California
//------------------------------------------------------------



import java.io.*; import java.net.*;

public class Client {

    public static void main(String args[]) {
        try {
            Socket skt = new Socket("192.168.1.138", 6999);
            System.out.println("Connected to server");

            InputStream is = skt.getInputStream();
            ObjectInputStream ois = new ObjectInputStream(is);
            OutputStream os = skt.getOutputStream();
            ObjectOutputStream oos = new ObjectOutputStream(os);

            BufferedReader br=new BufferedReader(new InputStreamReader(System.in));

            String rcv_msg; int send_msg;
            while(true) {
            /* Send a message to Server */
                System.out.print("To Server : ");
                send_msg = Integer.parseInt(br.readLine());
                oos.writeObject(send_msg);

            /* Recieve a message from Server */
                System.out.println("Waiting for server to respond ... ");
                if((rcv_msg = (String)ois.readObject()) != null) {
                    System.out.println("\nFrom Server : " + rcv_msg);
                }
            }
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}

