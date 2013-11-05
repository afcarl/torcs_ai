/***************************************************************************
 
    file                 : MyDriver.cpp
    copyright            : (C) 2013 Daniel Jallov
 
 ***************************************************************************/

#include "MyDriver.h"


/* Gear Changing Constants*/
const int MyDriver::gearUp[6]=
    {
        5000,6000,6000,6500,7000,0
    };
const int MyDriver::gearDown[6]=
    {
        0,2500,3000,3000,3500,3500
    };

/* Stuck constants*/
const int MyDriver::stuckTime = 25;
const float MyDriver::stuckAngle = .523598775; //PI/6

/* Accel and Brake Constants*/
const float MyDriver::maxSpeedDist=70;
const float MyDriver::maxSpeed=150;
const float MyDriver::sin5 = 0.08716;
const float MyDriver::cos5 = 0.99619;

/* Steering constants*/
const float MyDriver::steerLock=0.366519 ;
const float MyDriver::steerSensitivityOffset=80.0;
const float MyDriver::wheelSensitivityCoeff=1;

/* ABS Filter Constants */
const float MyDriver::wheelRadius[4]={0.3306,0.3306,0.3276,0.3276};
const float MyDriver::absSlip=2.0;
const float MyDriver::absRange=3.0;
const float MyDriver::absMinSpeed=3.0;

/* Clutch constants */
const float MyDriver::clutchMax=0.5;
const float MyDriver::clutchDelta=0.05;
const float MyDriver::clutchRange=0.82;
const float MyDriver::clutchDeltaTime=0.02;
const float MyDriver::clutchDeltaRaced=10;
const float MyDriver::clutchDec=0.01;
const float MyDriver::clutchMaxModifier=1.3;
const float MyDriver::clutchMaxTime=1.5;


int
MyDriver::getGear(CarState &cs)
{

    int gear = cs.getGear();
    int rpm  = cs.getRpm();

    // if gear is 0 (N) or -1 (R) just return 1 
    if (gear<1)
        return 1;
    // check if the RPM value of car is greater than the one suggested 
    // to shift up the gear from the current one     
    if (gear <6 && rpm >= gearUp[gear-1])
        return gear + 1;
    else
    	// check if the RPM value of car is lower than the one suggested 
    	// to shift down the gear from the current one
        if (gear > 1 && rpm <= gearDown[gear-1])
            return gear - 1;
        else // otherwhise keep current gear
            return gear;
}

float
MyDriver::getSteer(CarState &cs)
{	
	float rightSensor=cs.getTrack(10);
    // reading of sensor parallel to car axis
    float centerSensor=cs.getTrack(9);
    // reading of sensor at -5 degree w.r.t. car axis
    float leftSensor =cs.getTrack(8);
	// Check if straight line or curve
	bool straightLine = (centerSensor > rightSensor && centerSensor > leftSensor) || (leftSensor > 190 && rightSensor > 190);
	if (straightLine) {
		// No need to turn on straight line
	//	return 0;
	}

	if (rightSensor < centerSensor && centerSensor < leftSensor) {
		// Left turn
		
		
	}
	return cs.getAngle() - cs.getTrackPos();
	/*float diff = leftSensor - rightSensor;
	cout << "Turn::Diff: " << diff << "\r\n";
	return 1 / diff;*/
}

float MyDriver::getBrake(CarState &cs) {
	// reading of sensor at +5 degree w.r.t. car axis
        float rightSensor=cs.getTrack(10);
        // reading of sensor parallel to car axis
        float centerSensor=cs.getTrack(9);
        // reading of sensor at -5 degree w.r.t. car axis
        float leftSensor =cs.getTrack(8);

		// Check if straight line or curve
		bool straightLine = (centerSensor > rightSensor && centerSensor > leftSensor) || (leftSensor > 190 && rightSensor > 190);
		if (straightLine) {
			// No need to brake on straight line
			return 0;
		}

		if (rightSensor < centerSensor && centerSensor < leftSensor || leftSensor < centerSensor < rightSensor) {
			// Approaching a turn - figure out how close to the turn we are and how fast we are driving
			// centerSensor is the distance to the curve ahead
			// Brake amount depends on how sharp the curve is (that we are headed towards)
			// Lower difference between right and left sensors means sharper turn
			float diff = std::abs(rightSensor - leftSensor);

			// Three factors apply 
			// 1. Difference between left and right - small difference means sharp turn => more brake
			// 2. Distance to edge of track - closer distance => more brake
			// 3. Speed of car - if going slow, no need to brake. The faster the speed => more brake

			float speed = cs.getSpeedX() / 3.6; // Get the speed in m/s
			if (speed < 10) {
				return 0;
			}
			float time = centerSensor / speed; // Seconds until we hit the edge of the track

			float temp = 1 / time * 1 / diff;
		//	cout << "Temp: " << temp << "\r\n";
			float s = sigmoid(temp);
		//	cout << "Sigmoid: " << s << "\r\n";
			return temp;
		}
}

float
MyDriver::getAccel(CarState &cs)
{
    // checks if car is out of track
    if (cs.getTrackPos() < 1 && cs.getTrackPos() > -1)
    {
        // reading of sensor at +5 degree w.r.t. car axis
        float rxSensor=cs.getTrack(10);
        // reading of sensor parallel to car axis
        float cSensor=cs.getTrack(9);
        // reading of sensor at -5 degree w.r.t. car axis
        float sxSensor=cs.getTrack(8);

        float targetSpeed;

        // track is straight and enough far from a turn so goes to max speed
        if (cSensor>maxSpeedDist || (cSensor>=rxSensor && cSensor >= sxSensor))
            targetSpeed = maxSpeed;
        else
        {
            // approaching a turn on right
            if(rxSensor>sxSensor)
            {
                // computing approximately the "angle" of turn
                float h = cSensor*sin5;
                float b = rxSensor - cSensor*cos5;
                float sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed*(cSensor*sinAngle/maxSpeedDist);
            }
            // approaching a turn on left
            else
            {
                // computing approximately the "angle" of turn
                float h = cSensor*sin5;
                float b = sxSensor - cSensor*cos5;
                float sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed*(cSensor*sinAngle/maxSpeedDist);
            }

        }

        // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
        return 2/(1+exp(cs.getSpeedX() - targetSpeed)) - 1;
    }
    else
        return 0.3; // when out of track returns a moderate acceleration command

}

CarControl
MyDriver::wDrive(CarState cs)
{
	// check if car is currently stuck
	if ( fabs(cs.getAngle()) > stuckAngle )
    {
		// update stuck counter
        stuck++;
    }
    else
    {
    	// if not stuck reset stuck counter
        stuck = 0;
    }

	// after car is stuck for a while apply recovering policy
    if (stuck > stuckTime)
    {
    	/* set gear and sterring command assuming car is 
    	 * pointing in a direction out of track */
    	
    	// to bring car parallel to track axis
        float steer = - cs.getAngle() / steerLock; 
        int gear=-1; // gear R
        
        // if car is pointing in the correct direction revert gear and steer  
        if (cs.getAngle()*cs.getTrackPos()>0)
        {
            gear = 1;
            steer = -steer;
        }

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc (1.0,0.0,gear,steer,clutch);
        return cc;
    }

	int gear = getGear(cs);
	float steer = getSteer(cs);
	float accel = getAcceleration(cs);
	float brake = getBrake(cs);
	accel = 1 - 2 * brake;
        // build a CarControl variable and return it
	CarControl cc(accel,brake,gear,steer,0); // float accel, float brake, int gear, float steer, float clutch, int focus=0 (implicit)
    return cc;
}

float MyDriver::sigmoid(float x)
{
     float exp_value;
     float return_value;

     /*** Exponential calculation ***/
     exp_value = exp((double) -x);

     /*** Final sigmoid value ***/
     return_value = 1 / (1 + 20 * exp_value);

     return return_value;
}

float MyDriver::getAcceleration(CarState &cs)
{
	float accel = 1;
	float frontSensor = cs.getTrack(9);
	float leftSensor = cs.getTrack(8);
	float rightSensor = cs.getTrack(10);
	if (leftSensor != rightSensor) {
	//	accel = frontSensor / 200;
	}
	return accel;
}

float
MyDriver::filterABS(CarState &cs,float brake)
{
	// convert speed to m/s
	float speed = cs.getSpeedX() / 3.6;
	// when spedd lower than min speed for abs do nothing
    if (speed < absMinSpeed)
        return brake;
    
    // compute the speed of wheels in m/s
    float slip = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        slip += cs.getWheelSpinVel(i) * wheelRadius[i];
    }
    // slip is the difference between actual speed of car and average speed of wheels
    slip = speed - slip/4.0f;
    // when slip too high applu ABS
    if (slip > absSlip)
    {
        brake = brake - (slip - absSlip)/absRange;
    }
    
    // check brake is not negative, otherwise set it to zero
    if (brake<0)
    	return 0;
    else
    	return brake;
}

void
MyDriver::onShutdown()
{
    cout << "Bye bye!" << endl;
}

void
MyDriver::onRestart()
{
    cout << "Restarting the race!" << endl;
}

void
MyDriver::clutching(CarState &cs, float &clutch)
{
  double maxClutch = clutchMax;

  // Check if the current situation is the race start
  if (cs.getCurLapTime()<clutchDeltaTime  && stage==RACE && cs.getDistRaced()<clutchDeltaRaced)
    clutch = maxClutch;

  // Adjust the current value of the clutch
  if(clutch > 0)
  {
    double delta = clutchDelta;
    if (cs.getGear() < 2)
	{
      // Apply a stronger clutch output when the gear is one and the race is just started
	  delta /= 2;
      maxClutch *= clutchMaxModifier;
      if (cs.getCurLapTime() < clutchMaxTime)
        clutch = maxClutch;
	}

    // check clutch is not bigger than maximum values
	clutch = min(maxClutch,double(clutch));

	// if clutch is not at max value decrease it quite quickly
	if (clutch!=maxClutch)
	{
	  clutch -= delta;
	  clutch = max(0.0,double(clutch));
	}
	// if clutch is at max value decrease it very slowly
	else
		clutch -= clutchDec;
  }
}

void
MyDriver::init(float *angles)
{

	// set angles as {-90,-75,-60,-45,-30,20,15,10,5,0,5,10,15,20,30,45,60,75,90}

	for (int i=0; i<5; i++)
	{
		angles[i]=-90+i*15;
		angles[18-i]=90-i*15;
	}

	for (int i=5; i<9; i++)
	{
			angles[i]=-20+(i-5)*5;
			angles[18-i]=20-(i-5)*5;
	}
	angles[9]=0;
}
