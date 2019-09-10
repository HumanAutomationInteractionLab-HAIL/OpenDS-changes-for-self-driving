package edu.wichita.haillab;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;

import eu.opends.car.Car;
import eu.opends.drivingTask.settings.SettingsLoader;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.main.SimulationDefaults;
import eu.opends.main.Simulator;

public class UDPBroadcaster extends Thread{
	private Simulator sim;

	private boolean stoprequested = false;
	private boolean errorOccurred = false;

	private DatagramSocket socket;
	
	/**
	 * @param sim
	 * 			The simulator
	 */
	public UDPBroadcaster(Simulator sim)
    {
		super();		
		this.sim = sim;
    }
	
    
	/**
	 * Listens for incoming CAN instructions (as XML), such as gas, brake, steering angle, 
	 * reset and change view, which will be forwarded to the XML-parser
	 */
	@Override
	public void run() 
	{
		System.err.println("UDP broadcaster started");

		InetAddress host = null;		
		SettingsLoader settingsLoader = Simulator.getDrivingTask().getSettingsLoader();
		
		int port = settingsLoader.getSetting(Setting.UDPInterface_port, SimulationDefaults.UDPInterface_port);
		int broadcastsPerSecond = settingsLoader.getSetting(Setting.UDPInterface_updateRate, SimulationDefaults.UDPInterface_updateRate);
		
		try {
			String hostname = settingsLoader.getSetting(Setting.UDPInterface_host, SimulationDefaults.UDPInterface_host);
			host = InetAddress.getByName(hostname);
			socket = new DatagramSocket();
			socket.setSoTimeout(10);
			
			System.out.println("UDP broadcasting to " + host + ":" + port);
		} catch (Exception e) {
			e.printStackTrace();
			System.err.println("UDP connection failed at " + host + ":" + port);
			errorOccurred = true;
		}
		
		
		while(!stoprequested && !errorOccurred)
		{
			try {
			    ByteBuffer buffer = ByteBuffer.allocate(1 + Long.BYTES);
	 		    buffer.put((byte) 0x01);
			    buffer.putLong(System.currentTimeMillis());
			    
			    byte[] data = buffer.array();
			    
			    DatagramPacket timestampPacket = new DatagramPacket(data, data.length, host, port);
			    socket.send(timestampPacket);
			 	
			 	Thread.sleep(1000 / broadcastsPerSecond);
			} catch (InterruptedException exc) {
				// NOP
			} catch (Exception e) {
				System.err.println("UDP error: " + e.toString());
				errorOccurred = true;
			}
		}
		
		// close TCP connection to CAN-Interface if connected at all
		try {
			if (socket != null)
			{
				try {Thread.sleep(100);} 
				catch (InterruptedException e){}

				socket.close();
				
				System.out.println("Connection to UDP-Interface closed");
			}
		} catch (Exception ex) {
			System.err.println("Could not close connection to UDP-Interface");
		}
	}
	
	/**
	 * Requests the connection to close after the current loop
	 */
	public synchronized void requestStop() 
	{
		stoprequested = true;
	}
	
   
}
