#include <Arduino.h>

#include <iostream>

//* rbc lib
#include "RBControl.hpp"
//* local lib for kaktusatko
#include "robot.hpp"

// Dear maintainer:
// When I wrote this code, only I and God
// knew what it was.
// Now, only God knows!
// So if you are done trying to 'optimize'
// this routine (and failed),
// please increment the following counter
// as a warning
// to the next guy:
// total_hours_wasted_here = 48

using namespace rb;





void setupServo() {
	rb::Manager& man = rb::Manager::get();
	man.initSmartServoBus(1, GPIO_NUM_17);
}

const int trigPin1 = 16;
const int echoPin1 = 14;
const int trigPin2 = 27;
const int echoPin2 = 4;
void setupSensors(Robot& rob) {
	pinMode(trigPin1, OUTPUT);
	pinMode(echoPin1, INPUT);
	pinMode(trigPin2, OUTPUT);
	pinMode(echoPin2, INPUT);

	rob.color.init(18, 19, 5);
}


void goUSFront(Robot& rob, int dis, int speed){
	while(rob.us.getDistance(trigPin2, echoPin2) > dis){
		rob.setSpeeds(speed, -speed);
		std::cout << rob.us.getDistance(trigPin2, echoPin2) << std::endl;
		delay(20);
	}
	while(rob.us.getDistance(trigPin2, echoPin2) < dis){
		rob.setSpeeds(-speed, speed);
		std::cout << rob.us.getDistance(trigPin2, echoPin2) << std::endl;
		delay(20);
	}
	rob.setSpeeds(0);
}


void eskoTam(Robot& rob){
	rob.go2(400, 50);
	rob.turn(180, 50, 280);
	rob.turn(-180, 50, 280);
	rob.go2(500, 50);
}


void setup() {
	Robot robot(172, 34);
	// pin setup
	pinMode(35, INPUT);
	setupServo();
	setupSensors(robot);

	//^ wait for interaction
	std::cout << "start" << std::endl;
	//while(digitalRead(35) == 1) {
		//delay(100);
	//}


	//^way to cubes
	//eskoTam(robot);


	//robot.turn(360, 30);



	int r, g, b = 0;
	while(true){
		robot.color.getRGB(r,g,b);
		std::cout << "color:\t" << r << " " << g << " " << b << "\t\t";
		std::cout << "dalka:\t" << robot.us.getDistance(trigPin1, echoPin1) << "\t" << robot.us.getDistance(trigPin2, echoPin2);
		std::cout << std::endl;
		delay(100);
	}
	
}

void loop() {}