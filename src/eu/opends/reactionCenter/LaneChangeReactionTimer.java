/*
*  This file is part of OpenDS (Open Source Driving Simulator).
*  Copyright (C) 2016 Rafael Math
*
*  OpenDS is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  OpenDS is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with OpenDS. If not, see <http://www.gnu.org/licenses/>.
*/

package eu.opends.reactionCenter;

import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.*; 

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;

import eu.opends.audio.AudioCenter;
import eu.opends.car.Car;
import eu.opends.car.SteeringCar;
import eu.opends.environment.LaneLimit;
import eu.opends.jasperReport.ReactionLogger;
import eu.opends.main.Simulator;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.Util;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficObject;

/**
 * 
 * @author Rafael Math
 */
public class LaneChangeReactionTimer extends ReactionTimer
{
	// These are relevant numbers for measuring when the driver is safely out of the way
	private float halfCarWidth = 0.75f;
	private float halfConeWidth = 0.1f;
	
	private long timer;
	private boolean timerSet = false;
	private String startLane;
	private String targetLane; 
	private float minSteeringAngle;
	private float steeringAngle;
	private long startTime;
	private float taskCompletionTime;
	private Vector3f startPosition;
	private float taskCompletionDistance;
	private boolean allowBrake;
	private float holdLaneFor;
	private String failSound;
	private String successSound;
	private Vector x_position;
	private Vector times; 
	private String leadVehicle;
	private String leadObstacle;
	private Vector TTC;
	private float avgTTC;
	private float minTTC;
	private int noCollisions;
	
	private boolean soundTimerIsActive = false;
	
	
	public LaneChangeReactionTimer(Simulator sim, ReactionLogger reactionLogger, long experimentStartTime, 
			String timerID, int index)
	{
		super(sim, reactionLogger, experimentStartTime, timerID, index);
	}

	
	public void setup(String newReactionGroupID, String startLane, String targetLane, 
			float minSteeringAngle, float taskCompletionTime, float taskCompletionDistance, 
			boolean allowBrake, float holdLaneFor, String failSound, String successSound, String leadVehicle, String leadObstacle, String newComment)
	{
		// check pre-condition
		if(startLane.equals(getCurrentLane()))
		{
			super.setup(newReactionGroupID, newComment);
			
			this.startLane = startLane;
			this.targetLane = targetLane;
			this.minSteeringAngle = minSteeringAngle;
			this.startTime = System.currentTimeMillis();
			this.taskCompletionTime = taskCompletionTime;
			this.startPosition = sim.getCar().getPosition();
			this.taskCompletionDistance = taskCompletionDistance;
			this.allowBrake = allowBrake;
			this.holdLaneFor = holdLaneFor;
			this.failSound = failSound;
			this.successSound = successSound;
			this.steeringAngle = 0;
			this.leadVehicle = leadVehicle;
			this.leadObstacle = leadObstacle;
			this.x_position = new Vector();
			this.times = new Vector();
			this.TTC= new Vector();
			this.avgTTC = 0; 
			this.minTTC = 0;
			this.noCollisions = 0;
			
			if(targetLane.equals("1") || targetLane.equals("3"))
				trialLogger.setTask(2);
			else if(targetLane.equals("0") || targetLane.equals("4"))
				trialLogger.setTask(3);
			
			timerIsActive = true;
			soundTimerIsActive = false;
			
			x_position.add(sim.getCar().getPosition().getX());
			times.add(startTime);
		}
		else
		{
			System.err.println("Not in start lane " + startLane + "! Currently: " + getCurrentLane());
			
			// play sound when time/distance has been passed
			this.startTime = System.currentTimeMillis();
			this.taskCompletionTime = taskCompletionTime;
			this.startPosition = sim.getCar().getPosition();
			this.taskCompletionDistance = taskCompletionDistance;
			this.failSound = failSound;
			
			soundTimerIsActive = true;
		}
	}


	public void reportMissedReaction()
	{
		super.reportMissedReaction();
		//System.err.println("MISSED");

		// play fail sound
		AudioCenter.playSound(failSound);
	}
	

	public void update()
	{		
		//super.update();
		
		if(timerIsActive)
		{
			boolean startTurning = false;
			long currentTime = System.currentTimeMillis();
			//System.out.println(currentTime);
			
			float currentSteeringAngle = FastMath.abs(sim.getCar().getSteeringWheelState());
			steeringAngle = Math.max(steeringAngle, currentSteeringAngle);
			//System.err.println("steering angle: " + steeringAngle);
			
			// This will be relevant later for measuring lateral acceleration
			x_position.add(sim.getCar().getPosition().getX());
			//System.out.println((float)sim.getCar().getPosition().getX());
			times.add(currentTime);
			
			// Continuously measure TTC until they safely get out of the way
			if(leadVehicle.equals(""))
				TTC.add(getTTC(getObstacleLoc(leadObstacle)));
			else
				TTC.add(getTTC(getVehicleLoc(leadVehicle)));
			
			if(isBraking())
				trialLogger.setAdditional_reaction(1);
			
			if(currentSteeringAngle >= 0.004444f)
			{
				if(startTurning= false)
				{
					x_position.clear();
					times.clear();
				}
				startTurning=true;
				trialLogger.setLaneChangeRT_2angle((int)(currentTime - startTime));
			}
				
			if(currentSteeringAngle >= 0.006666f)
				trialLogger.setLaneChangeRT_3angle((int)(currentTime - startTime));
			
			if(enteringTargetLane())
			{
				trialLogger.setLaneChangeRT_enterLane((int)(currentTime - startTime));
			}
			
			if(timeExceeded() || distanceExceeded())
			{	
				trialLogger.setStartTime(startTime);
				trialLogger.setlatAcl(getLatAccel(x_position,times));
				float sumTTC=(float) TTC.get(0);
				float minTTC=(float) TTC.get(0);
				
				// Go through the list of TTC values collected and find the smallest ones
				for (int a=1;a<=TTC.size()-1;a++)
				{
					sumTTC=sumTTC+(float)TTC.get(a);
					
					if (minTTC>(float)TTC.get(a))
					{
						minTTC=(float)TTC.get(a);
					}
					
				}				
				
				// This is to put the TTC values into the drivingTaskLog file
				avgTTC=sumTTC/TTC.size();
				trialLogger.setAvgTTC(avgTTC);
				trialLogger.setMinTTC(minTTC);
				
				reportMissedReaction();
			}
			else if(isBrakingWithoutPermission())
			{
				reportFailureReaction();
			}
			//else if(targetLane.equals(getCurrentLane()))
			else if(leadVehicle.equals(""))
			{
				// If they're out of danger of colliding with the lead obstacle, then stop recording the values
				// This is specific for the Construction Zone Event
				if(outOfDanger(halfConeWidth))
				{
//					PanelCenter.getMessageBox().addMessage("signal", 3);
					//System.out.println((float)sim.getCar().getPosition().getX());
					if(!timerSet)
					{
						timer = System.currentTimeMillis();
						timerSet = true;
					}
					
					//System.err.println("-----------------------hold lane: " + (currentTime-timer));
					if((currentTime-timer >= holdLaneFor) && (steeringAngle >= minSteeringAngle))
					{
						reportCorrectReaction();
					}
				}
			}
			else if(leadObstacle.equals(""))
			{
				// This is specific for the Broken Vehicle Event
				if(outOfDanger(halfCarWidth))
				{
//					PanelCenter.getMessageBox().addMessage("signal", 3);
					if(!timerSet)
					{
						timer = System.currentTimeMillis();
						timerSet = true;
					}
					
					//System.err.println("-----------------------hold lane: " + (currentTime-timer));
					if((currentTime-timer >= holdLaneFor) && (steeringAngle >= minSteeringAngle))
					{
						reportCorrectReaction();
					}
				}
			}
			else
			{
				timerSet = false;
			}
			
			long relativeStartTime = startTime - experimentStartTime;
			
			long holdLaneOffset = 0l;
			if(timerSet)
				holdLaneOffset = currentTime-timer;
			long reactionTime = currentTime - startTime - holdLaneOffset;
			
			if(correctReactionReported)
			{
				trialLogger.setStartTime(startTime);
				trialLogger.setlatAcl(getLatAccel(x_position,times));
				//System.err.println("CORRECT");
				float sumTTC=(float) TTC.get(0);
				float minTTC=(float) TTC.get(0);
				
				for (int a=1;a<=TTC.size()-1;a++)
				{
					sumTTC=sumTTC+(float)TTC.get(a);
					
					if (minTTC>(float)TTC.get(a))
					{
						minTTC=(float)TTC.get(a);
					}
					
				}
				
				//trialLogger.setlatAcl(getLatAccel(x_position,times));
				
				avgTTC=sumTTC/TTC.size();
				trialLogger.setAvgTTC(avgTTC);
				trialLogger.setMinTTC(minTTC);
				
				trialLogger.setLaneChangeRT_success((int)reactionTime);
				
				reactionLogger.add(reactionGroupID, 1, reactionTime, startTime, relativeStartTime, comment);
				
				reactionTimer = null;
				comment = "";
				
				trialLogger.setReaction(1);
				trialLogger.writeLog();
				
				// play success sound
				AudioCenter.playSound(successSound);
				
				timerIsActive = false;
			}
			else if(failureReactionReported)
			{
				trialLogger.setStartTime(startTime);
				//System.err.println("FAILED");
				
				float sumTTC=(float) TTC.get(0);
				float minTTC=(float) TTC.get(0);
				
				for (int a=1;a<=TTC.size()-1;a++)
				{
					sumTTC=sumTTC+(float)TTC.get(a);
					
					if (minTTC>(float)TTC.get(a))
					{
						minTTC=(float)TTC.get(a);
					}
					
				}
				
				avgTTC=sumTTC/TTC.size();
				trialLogger.setAvgTTC(avgTTC);
				trialLogger.setMinTTC(minTTC);
				
				reactionLogger.add(reactionGroupID, -1, reactionTime, startTime, relativeStartTime, comment);

				reactionTimer = null;
				comment = "";
				
				trialLogger.setReaction(0);
				trialLogger.writeLog();
				
				// play fail sound
				AudioCenter.playSound(failSound);
				
				timerIsActive = false;
			}
		}
		
		if(soundTimerIsActive)
		{
			if(timeExceeded() || distanceExceeded())
			{
				AudioCenter.playSound(failSound);
				soundTimerIsActive = false;
			}
		}
	}

	
	private boolean isBrakingWithoutPermission()
	{
		if(allowBrake)
			return false;
		else
			return isBraking();
	}
	
	
	private boolean isBraking() 
	{
		return (sim.getCar().getBrakePedalIntensity() > 0);
	}
	
	
	private boolean distanceExceeded()
	{
		// if task completion distance is 0 or less --> no distance limit
		if(taskCompletionDistance<=0)
			return false;
		else
		{
			Vector3f currentPosition = sim.getCar().getPosition();
			//System.err.println("Distance: " + currentPosition.distance(startPosition));
			return (currentPosition.distance(startPosition) > taskCompletionDistance);
		}
	}


	private boolean timeExceeded()
	{
		// if task completion time is 0 or less --> no time limit
		if(taskCompletionTime<=0)
			return false;
		else
		{
			long currentTime = System.currentTimeMillis();
			//System.err.println("Time: " + (currentTime-startTime));
			return (currentTime-startTime > taskCompletionTime);
		}
	}


	private String getCurrentLane()
	{
		float currentX = sim.getCar().getPosition().getX();
				
		Map<String, LaneLimit> laneList = Simulator.getDrivingTask().getScenarioLoader().getLaneList();
		Iterator<Entry<String, LaneLimit>> it = laneList.entrySet().iterator();
	    while(it.hasNext()) 
	    {
	        Entry<String, LaneLimit> pairs = (Entry<String, LaneLimit>)it.next();
	        String laneID = pairs.getKey();
	        LaneLimit laneLimit = pairs.getValue();
	     
	        float xMinReduced = laneLimit.getXMin() + halfCarWidth;
			float xMaxReduced = laneLimit.getXMax() - halfCarWidth;
			
	        if(xMinReduced <= currentX && currentX <= xMaxReduced)
	        	return laneID;
	    }
	    
		return null;
	}
	
	// Custom function made to verify that the driver wouldn't collide with the obstacle anymore if they drove straight
	private boolean outOfDanger(float halfObstacleWidth)
	{
		float currentX = sim.getCar().getPosition().getX();
		Map<String, LaneLimit> laneList = Simulator.getDrivingTask().getScenarioLoader().getLaneList();
		LaneLimit target = laneList.get(startLane);
		float obstacle_xpos = ((target.getXMax()-target.getXMin())/2)+target.getXMin();
		
		float xMinExtended = obstacle_xpos + halfCarWidth + halfObstacleWidth;
		float xMaxExtended = obstacle_xpos - halfCarWidth - halfObstacleWidth;
		
		/*System.out.println(currentX);
		System.out.println(xMinExtended);
		System.out.println(xMaxExtended);
		System.out.println("-");*/
		
		return (xMinExtended < currentX || currentX < xMaxExtended);
	}
	
	private boolean enteringTargetLane() 
	{
		float currentX = sim.getCar().getPosition().getX();
		
		Map<String, LaneLimit> laneList = Simulator.getDrivingTask().getScenarioLoader().getLaneList();
		LaneLimit target = laneList.get(targetLane);
		
		float xMinExtended = target.getXMin() - halfCarWidth;
		float xMaxExtended = target.getXMax() + halfCarWidth;
		
		return (xMinExtended <= currentX && currentX <= xMaxExtended);
	}

	private Vector3f getVehicleLoc(String obstacleName)
	{
		//System.out.println(obstacleName+ "hi");
		for(TrafficObject trafficObject : PhysicalTraffic.getTrafficObjectList())
		{
			if(trafficObject.getName().equals(obstacleName))
			{
				//System.out.println("Found");
				//System.out.println(trafficObject.getPosition());
				return trafficObject.getPosition();
			}
		}
		
/*		Spatial object = Util.findNode(sim.getRootNode(), obstacleName);
		System.out.println(object.getLocalTranslation());
		return object.getLocalTranslation();*/
		return null;
	}
	
	private Vector3f getObstacleLoc(String obstacleName)
	{
		//System.out.println(obstacleName+ "hi");
		/*for(TrafficObject trafficObject : PhysicalTraffic.getTrafficObjectList())
		{
			if(trafficObject.getName().equals(obstacleName))
			{
				System.out.println("Found");
				System.out.println(trafficObject.getPosition());
				return trafficObject.getPosition();
			}
		}*/
		
		Spatial object = Util.findNode(sim.getRootNode(), obstacleName);
		//System.out.println(object.getLocalTranslation());
		return object.getLocalTranslation();
//		return null;
	}
	
	// Custom function to measure TTC
	private Float getTTC(Vector3f obstaclePos)
	{
		//float distanceToObstacle = obstaclePos.distance(sim.getCar().getPosition());
		
		//float lateralDistance = sim.getCar().getLateralDistance(obstaclePos);
		float forwardDistance = (sim.getCar().getForwardDistance(obstaclePos))/1000;
		
		float TTC = forwardDistance/((sim.getCar().getCurrentSpeedKmh())/3600);
		
		return TTC;
	}
	
	// Custom function to measure lateral acceleration (but the way it formulates it is actually incorrect)
	private float getLatAccel(Vector x_pos, Vector times)
	{
		// Create empty vectors to store values
		Vector lat_veloc = new Vector();
		float sum_lat = 0;
		Vector lat_accel = new Vector();
		float sum_accel = 0;
		lat_veloc.add(sum_lat);
		
		//for(int i=0;i<=x_pos.size()-8;i+=6)
		for(int i=0;i<x_pos.size()-1;i+=1)
		{
			// Calculate how much distance and time has passed between each value
			// We will deduce velocity for every 0.1 seconds
			float dist_traveled = Math.abs((float)x_pos.get(i+1)-(float)x_pos.get(i));
			long time_elapsed = (long)times.get(i+1)-(long)times.get(i);
			float time_elapsed2 = ((float)time_elapsed)/1000;
			//System.out.println(time_elapsed2);
			
			/*System.out.println(dist_traveled);
			System.out.println(time_elapsed);*/
			
			sum_lat = sum_lat + (dist_traveled/time_elapsed2);
			//System.out.println(dist_traveled/time_elapsed2);
			
			// Store lateral velocities in this vector
			lat_veloc.add(dist_traveled/time_elapsed2);
		}
		
		//System.out.println("hi");
		int k = 1;
		
		for(int j=1;j<=lat_veloc.size()-2;j++)
		{
			//System.out.println(sum_accel);
			//sum_accel = sum_accel + (float)lat_veloc.get(j) - (float)lat_veloc.get(j-1);
			long time_elapsed = (long)times.get(k+1)-(long)times.get(k);
			float time_elapsed2 = ((float)time_elapsed)/1000;
			//System.out.println(time_elapsed2);
			
			lat_accel.add(((float)lat_veloc.get(j) - (float)lat_veloc.get(j-1))/time_elapsed2);
			//System.out.println(lat_accel.get(j-1));
			k+=1;
		}
		
		// Find out at what point the lateral acceleration became the highest
		float maxLatAccel=(float) lat_accel.get(0);
		
		for (int a=1;a<=lat_accel.size()-1;a++)
		{
			
			/*if (maxLatAccel>(float)lat_accel.get(a))
			{
				maxLatAccel=(float)TTC.get(a);
			}*/
			if (maxLatAccel<(float)lat_accel.get(a))
			{
				maxLatAccel=(float)lat_accel.get(a);
			}
			
		}
		return maxLatAccel;
	}
}
