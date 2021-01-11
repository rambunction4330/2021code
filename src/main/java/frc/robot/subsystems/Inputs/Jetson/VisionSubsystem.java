/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Inputs.Jetson;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.LineNumberReader;
import java.io.OutputStream;
import java.io.StringReader;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.HashMap;
import java.util.Map;

public class VisionSubsystem extends SubsystemBase {
    public static final int CONNECTION_TIMEOUT_SEC = 10;


    private String host;
    private int port;

    private Socket socket;
    private InputStream is;
    private OutputStream os;
    public boolean active = false;
    private static final byte[] GET_DATA_COMMAND = "DATA\n".getBytes();
    private static final byte[] STOP_COMMAND = "STOP\n".getBytes();
    private Map<String, Double> visionData;
    private double loadingBayBearing = Double.MAX_VALUE;
    private double loadingBayYAngle = Double.MAX_VALUE;
    private double loadingBayDistance = Double.MAX_VALUE;

    private double powerPortBearing = Double.MAX_VALUE;
    private double powerPortYAngle = Double.MAX_VALUE;
    private double powerPortDistance = Double.MAX_VALUE;


    /**
     * Use this constructor to change the default values of hostname and constructor
     * @param host
     * @param port
     */
    public VisionSubsystem(String host, int port) {
        this.host = host;
        this.port = port;
    }

    public synchronized void startUp() {

        System.out.println("Opening connection to Jetson on " + host + ":" + port +
                " with " + CONNECTION_TIMEOUT_SEC + " second timeout");

        socket = new Socket();

        // we don't want to block the calling thread for long time in case the
        // connection doesn't work immediately, so try to connect to the vision board
        // on another thread
        Thread connectThread = new Thread() {

            @Override
            public void run() {
                try {
                    // connect is a blocking call which will throw a SocketException if the socket
                    // is asked to close while blocked, so this thread will die
                    socket.connect(new InetSocketAddress(host,port), CONNECTION_TIMEOUT_SEC * 1000);
                    is = socket.getInputStream();
                    os = socket.getOutputStream();
                    active = true;
                    os.write(GET_DATA_COMMAND);
                    os.flush();

                    System.out.println("Successfully connected to Jetson");
                } catch ( Exception e ) {
                    System.err.println("Error connecting to Jetson on startup. " + e.getMessage());
                }
            }

        };

        connectThread.setDaemon(true);
        connectThread.start();
    }

    public synchronized void shutDown() {
        System.out.println("Disconnecting from Jetson.");
        active = false;

        if ( os != null ) {
            try {
                os.write(STOP_COMMAND);
                os.flush();
                os.close();
            } catch(Exception e) {
                //				e.printStackTrace();
            } finally {
                os = null;
            }
        }
        if ( is != null ) {
            try {
                is.close();
            }catch (Exception e) {

            }finally {
                is = null;
            }
        }
        if ( socket != null ) {
            try {
                socket.close();
            } catch (Exception e) {

            } finally {
                socket = null;
            }
        }
        System.out.println("Successfully disconnected from Jetson.");
    }

    public synchronized Map<String, Double> retrieveData() {
//        System.out.println("Does anyone hear me?");
        if ( !active ) {
            // the client may still be trying to connect
//            System.out.println("Uh oh, it's not active!!");
            return new HashMap<String,Double>();
        }

//        System.out.println("It's alive!!");

        try {
            return getMessages(os, is);
        } catch ( Exception e ) {
            e.printStackTrace();
            System.err.println("Error getting messages from the Jetson. " + e.getMessage());
            return new HashMap<String, Double>();
        }
    }

    static protected Map<String, Double> getMessages(OutputStream os, InputStream is) throws IOException {
        if (os == null || is == null) {
//            System.out.println("Oh no, it's a null!!");
            return new HashMap<String, Double>();
        }

        // send a request to server for data
        os.write("DATA\n".getBytes());
        os.flush();

        // read the response from the server
        ByteArrayOutputStream binaryData = new ByteArrayOutputStream();
        int lastChar = Integer.MIN_VALUE;
        int endLine = '\n';
        while (true) {
            int thisChar = is.read();
            if (thisChar == -1 || (thisChar == endLine && (lastChar == endLine || lastChar == Integer.MIN_VALUE) )) {
                break;
            }
            binaryData.write(thisChar);
            lastChar = thisChar;
//            System.out.println(binaryData);
        }


        if ( binaryData.size() == 0 ) {
//			System.out.println("Vision has no data.");
            return new HashMap<String,Double>();
        }

        // get the data as a string
        String data = binaryData.toString();
        Map<String, Double> map = new HashMap<String, Double>();

        // parse the data into map
        LineNumberReader reader = new LineNumberReader(new StringReader(data));
        String line = null;
        while ((line = reader.readLine()) != null) {
//			System.out.println("Vision has read line " + line);
            int index = line.indexOf("=");
            if (index == -1) {
                continue;
            }
            String key = line.substring(0, index);
            Double value = Double.parseDouble(line.substring(index + 1));
            map.put(key, value);
        }

        return map;
    }

    public Double getLoadingBayBearing() {
        return loadingBayBearing;
    }

    public Double getLoadingYAngle() {
        return loadingBayYAngle;
    }

    public Double getLoadingBayDistanc() {
        return loadingBayDistance;
    }

    public Double getPowerPortBearing() {
        return powerPortBearing;
    }

    public Double getPowerYAngle() {
        return powerPortYAngle;
    }

    public Double getPowerDistance() {
        return powerPortDistance;
    }

    public  Map<String, Double> getData() {
        return visionData;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (active) {
            Map<String, Double> data = retrieveData();
            try {
                if (data.get("pprb") != null) {
                    powerPortBearing = data.get("pprb");
                    powerPortYAngle = data.get("ppya");
                    powerPortDistance = data.get("ppd");
                } else {
                    powerPortBearing = Double.MIN_VALUE;
                    powerPortYAngle = Double.MIN_VALUE;
                    powerPortDistance = Double.MIN_VALUE;
                }
            } catch (Exception e) {
                powerPortBearing = Double.MIN_VALUE;
                powerPortYAngle = Double.MIN_VALUE;
                powerPortDistance = Double.MIN_VALUE;
            }
            try {
                if (data.get("lbrb") != null) {
                    loadingBayBearing = data.get("lbrb");
                    loadingBayYAngle = data.get("lbya");
                    loadingBayDistance = data.get("lbd");
                } else {
                    loadingBayBearing = Double.MIN_VALUE;
                    loadingBayYAngle = Double.MIN_VALUE;
                    loadingBayDistance = Double.MIN_VALUE;
                }
            } catch (Exception e) {
                loadingBayBearing = Double.MIN_VALUE;
                loadingBayYAngle = Double.MIN_VALUE;
                loadingBayDistance = Double.MIN_VALUE;
            }
        }
    }
}