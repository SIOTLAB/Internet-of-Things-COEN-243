//------------------------------------------------------------
//SCU's Internet of Things Research Lab (SIOTLAB)
//Santa Clara University (SCU)
//Santa Clara, California
//Behnam Dezfouli, PhD
//------------------------------------------------------------


package applayerprotocol;

// echo server
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.math.BigInteger;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.LinkedList;

class DeviceInfo { 

    long macAddress;
    int[] registers;
}

class DeviceDB {//Data base to hold device information

    LinkedList<DeviceInfo> devices;

    public DeviceDB() {
        devices = new LinkedList<>();
    }

    public int findDevice(long macAddress) {
        for (int index = 0; index < devices.size(); ++index) {
            if (devices.get(index).macAddress == macAddress) {
                return index;
            }
        }        
        return -1;
    }

    public int addDevice(long macAddress) {
        DeviceInfo newDevice = new DeviceInfo();
        newDevice.macAddress = macAddress;
        newDevice.registers = new int[256];
        devices.addLast(newDevice);
        return devices.size() - 1;
    }

    public void setReg(int index, int regAdd, int regVal) {
        devices.get(index).registers[regAdd] = regVal;
    }

    public int getReg(int index, int regAdd) {
        return devices.get(index).registers[regAdd];
    }
}

public class Server {

    public static int transID = 0;
    
    public static void main(String args[]) {

        ServerSocket ss2 = null;
        DeviceDB db = new DeviceDB();

        try {
            ss2 = new ServerSocket(6999); // can also use static final PORT_NUM , when defined
        } catch (IOException e) {
            System.out.println("Server error");
            java.lang.System.exit(1);
        }

        System.out.println("Server started...");

        while (true) {
            try {
                Socket s = null;
                s = ss2.accept();
                ServerThread st = new ServerThread(s, db);
                st.start();

            } catch (Exception e) {
                System.out.println("Connection Error");
            }
        }

    }

}

class ServerThread extends Thread {

    String line = null;
    BufferedReader is = null;
    PrintWriter os = null;
    Socket s = null;
    DeviceDB db;

    public ServerThread(Socket s, DeviceDB db) {
        this.s = s;
        this.db = db;
    }

    public Boolean ValidateWriteCmd(String[] command, PrintWriter os) {
        if (command[0].length() != 1 || command[1].length() != 4
                || command[2].length() != 12 || command[3].length() != 2 || 
                command[4].length() != 4) {
            System.out.println("Invalid write command!");            
            SendFailResponse(os);
            return false;
        }
        return true;
    }
    
    public Boolean ValidateReadCmd(String[] command, PrintWriter os) {
        if (command[0].length() != 1 || command[1].length() != 4
                || command[2].length() != 12 || command[3].length() != 2) {
            System.out.println("Invalid read command!");            
            SendFailResponse(os);
            return false;
        }
        return true;
    }    

    //Failed to parse the command
    public void SendFailResponse(PrintWriter os) {
        char[] response = {'I', '\n'};
        os.println(response);
        os.flush();
    }

    //Send a confirm after a successful write
    public void SendResponse(String retChar, String parts1, String parts2, 
        String parts3, String parts4, PrintWriter os) {
        String sentResponse = retChar + "-" + parts1 + "-"  + parts2 + "-" + parts3 + "-" + parts4;
        os.println(sentResponse);
        os.flush();
        System.out.println("Sent response: " + sentResponse);        
    }
    
    @Override
    public void run() {
        Server.transID++;
        System.out.println("----Transaction: " + Server.transID + "----");            
        
        try {
            is = new BufferedReader(new InputStreamReader(s.getInputStream()));
            os = new PrintWriter(s.getOutputStream());

        } catch (IOException e) {
            System.out.println("IO error in server thread");
        }

        try {
            line = is.readLine();
            System.out.println("Received: " + line);

            String parts[] = line.split("-");

            if ( (parts[0].equals("W") && ValidateWriteCmd(parts, os)) || (parts[0].equals("R") && ValidateReadCmd(parts, os))) {
               
                long mac = new BigInteger(parts[2], 16).longValue();
                //System.out.println("MAC: " + mac);

                int regAdd = Integer.valueOf(parts[3], 16);
                //System.out.println("Reg Address: " + regAdd);
                if (regAdd < 0 || regAdd > 255) {
                    SendFailResponse(os);
                }

                if (parts[0].equals("W")) {
                    int regVal = Integer.valueOf(parts[4], 16);
                    //System.out.println("Reg Value: " + regVal);
                    
                    System.out.println("Database size is: " + db.devices.size());

                    int index = db.findDevice(mac);
                    if (index != -1) {
                        System.out.println("Device found at index: "+ index);
                        db.setReg(index, regAdd, regVal);
                    } else {
                        System.out.println("New device added!");                        
                        int indexNew = db.addDevice(mac);
                        db.setReg(indexNew, regAdd, regVal);
                    }
                    
                    SendResponse("A", parts[1], parts[2], parts[3], parts[4], os);
                    
                }
                if (parts[0].equals("R")) {
                    int index = db.findDevice(mac);
                    if (index != -1) {
                        System.out.println("Device found at index: "+ index);
                        //String regVal = Integer.toHexString(db.getReg(index, regAdd));
                        String regVal = String.format("%04X", db.getReg(index, regAdd));
                        SendResponse("A", parts[1], parts[2], parts[3], regVal, os);                    
                    } else {
                        SendFailResponse(os);
                    }                    
                }
            } else {
                SendFailResponse(os);
            }

        } catch (IOException e) {
            line = this.getName(); //reused String line for getting thread name
            System.out.println("IO Error/ Client " + line + " terminated abruptly");
        } finally {
            try {
                if (is != null) is.close(); 
                if (os != null) os.close();
                if (s != null) s.close();
                System.out.println("Connection Closed.");                  
            } catch (IOException ie) {
                System.out.println("Socket Close Error");
            }
        }//end finally
    }
}
