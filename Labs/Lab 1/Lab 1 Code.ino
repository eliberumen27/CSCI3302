#include <Sparki.h>

#define Print(x) Serial.println(x)

// Set up some global variables with default values to be replaced during operation
int current_state = 0;
const int threshold = 400; // IR reading threshold to detect whether thereds a black line under the sensor
int cm_distance = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

void setup() {
  // the task is done in setup. Once done, it's done, unless you reupload the code

  // Turn on the red LED
  sparki.RGB(RGB_RED); 
  // Center the ultrasonic sensor
  sparki.servo(SERVO_CENTER);
  // Give the motor time to turn
  delay(5000);
  Print("opening the grippers");
  sparki.gripperOpen();
  // Open the gripper
  delay(3000);
  // Give the motor time to open the griper
  sparki.gripperStop();
  // 5 seconds should be long enough, more of a preventative measure here

  // Change LED to green so we know the robot's setup is done!
  sparki.RGB(RGB_GREEN);

  // rotate the robot until you see the object
  rotate();
  // travel to the object, and grip it
  travelToObj();
  // go along the line until the start is encountered
  lineAlgorithm();
}

void readSensors() {
  // returns centimeter distance
  cm_distance = sparki.ping();
  // 900 is close to white, 200 is close to black
  line_left =  sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void lineAlgorithm() {
  bool hitTrack = false;
  while(1) {
    // measure the left IR sensor
    int lineLeft   = sparki.lineLeft();
    // measure the center IR sensor
    int lineCenter = sparki.lineCenter();
    // measure the right IR sensor
    int lineRight  = sparki.lineRight();

    if ( lineLeft < threshold ) // if line is below left line sensor
    {
      sparki.moveLeft(); // turn left
    }

    if ( lineRight < threshold ) // if line is below right line sensor
    {
      sparki.moveRight(); // turn right
    }

    // if readings are all high, we're in empty space, not on the track. Move forward.
    if ((lineCenter > threshold) && (lineLeft > threshold) && (lineRight > threshold) )
    {
      sparki.moveForward(); // move forward
      if (hitTrack) {
        hitTrack = false;
      }
    }

    // if the sparki is oriented straight on the line, keep moving forward
    if ((lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
    {
      sparki.moveForward(); // move forward
      hitTrack = true;
    }

    // if else if below to take care of separate cases

		// if the robot is not on the track yet, but sees black on all sides, turn 97 degrees
    if ( (lineCenter < threshold) && (lineLeft < threshold) && (lineRight < threshold) && (!hitTrack) )
    {
      sparki.moveForward();
      sparki.moveForward();
      delay(1000);
      sparki.moveLeft(97); // move forward
      hitTrack = true;
      delay(2000);
    }
  	// else if it's on the track, and all black is seen, then we stop here
    else if ( (lineCenter < threshold) && (lineLeft < threshold) && (lineRight < threshold) && (hitTrack) )
    {
      sparki.moveStop(); // move forward
      delay(2000);
      sparki.beep();
      sparki.gripperOpen(); // Open the gripper
      delay(3000); // Give the motor time to open the griper
      sparki.gripperStop();
      break;
    }

  }

}

// travels to object, and grips it
void travelToObj() {
  // keep moving forward until at around 8cm
  while(sparki.ping() > 3.5 || sparki.ping() == -1) {
    Print("Trying to travel to object");
    sparki.moveForward();
  }

  sparki.moveStop();
  sparki.gripperClose();
  delay(5000); // Give the motor time to open the griper
  sparki.gripperStop();
  Print("Stopping");
  sparki.moveStop();
  // execute the turn 180 degrees method
  sparki.moveRight(180);
}

void rotate() {
  // sparki rotates
  Print("In rotate");
  // loop will stop
  while(sparki.ping() > 30 || sparki.ping() == -1) { // while ultrasonic sensor does not detect object, rotate
    Print("Trying to rotate");
    sparki.moveRight(5);
    delay(50);
    // loop will stop if the ping is less and NOT -1
  }
  sparki.moveRight(12.5);
  delay(2000);

}

void loop() {
	/*
	 we made the choice to not put our main process code in the loop.
   The reason being that the code was more sequential to do, and not
   like a PID loop. To us, it made sense to take each step, and not have a loop at 10hz
   with a different state each time in the loop, because we want to do one step fully, then change states to the next step, and keep doing things
   until that step is done. It works.
	 */
}
