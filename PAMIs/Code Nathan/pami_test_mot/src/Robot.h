#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <LinkedList.h>
#include "Wheel.h"
#include "Sensor.h"
#include "Odometry.h"
#include "Trajectory.h"
#include "MotionController.h"
#include "RobotState.h"
#include "UltrasonicSensor.h"
#include "MotorUP.h"

class Robot {
private:
    LinkedList<Component*> components;
    Odometry odometry;
    Trajectory trajectory;
    MotionController motionController;
    RobotState state;
    MotorUP* motorUP;
    double obstacleThreshold = 20.0; // Stop if closer than 20 cm
    unsigned long taskStartTime = 0; // For non-blocking task execution

public:
    void setState(RobotState newState) {
        state = newState;
        if (newState == RobotState::DANCING) {
            taskStartTime = millis(); // Initialiser le temps de début pour DANCING
        }
    }

    RobotState getState() const {
        return state;
    }
public:
    Robot(double wheel_radius, double wheel_base)
        : odometry(wheel_radius, wheel_base), motionController(1.0, 1.5), state(RobotState::IDLE) {}

    void addComponent(const char* name, Component* component) {
        components.add(component);
    }

    Component* getComponent(String name) {
        for (int i = 0; i < components.size(); i++) {
            if (components.get(i)->getName() == name) {
                return components.get(i);
            }
        }
        return nullptr;
    }

    void updateAll() {
        for (int i = 0; i < components.size(); i++) {
            components.get(i)->update();
        }

        Wheel* wheelL = static_cast<Wheel*>(getComponent("wheelL"));
        Wheel* wheelR = static_cast<Wheel*>(getComponent("wheelR"));
        UltrasonicSensor* sensor = static_cast<UltrasonicSensor*>(getComponent("ultrasonic"));
        MotorUP* motorUP = static_cast<MotorUP*>(getComponent("motorUP"));


        if (!wheelL || !wheelR || !sensor || !motorUP) return;

        odometry.update(wheelL->getEncoderValue(), wheelR->getEncoderValue());
        double distance = sensor->getDistance();

        switch (state) {
            case RobotState::IDLE:

                if (!trajectory.isFinished()) {
                    state = RobotState::NAVIGATING;
                } 
                break;

            case RobotState::NAVIGATING:
                /*if (distance < obstacleThreshold) {
                    Serial.println("Obstacle detected! Avoiding...");
                    state = RobotState::AVOIDING;
                    break;
                }*/
				
						
                if (!trajectory.isFinished()) {
                    Waypoint target = trajectory.getNextWaypoint();
                    double leftSpeed, rightSpeed;

                    motionController.computeSpeed(
                        odometry.getX(), odometry.getY(), odometry.getTheta(),
                        target, leftSpeed, rightSpeed);
					
                            // Debug: Print current position and target
                    Serial.print("Current position: (");
                    Serial.print(odometry.getX());
                    Serial.print(", ");
                    Serial.print(odometry.getY());
                    Serial.println(")");
                    
                    Serial.print("Target: (");
                    Serial.print(target.x);
                    Serial.print(", ");
                    Serial.print(target.y);
                    Serial.println(")");
                    
                    // Debug: Print speeds
                    Serial.print("Left Speed: ");
                    Serial.print(leftSpeed);
                    Serial.print(" | Right Speed: ");
                    Serial.println(rightSpeed);

                    const int MAX_SPEED = 255;
                    leftSpeed = map(constrain(abs(leftSpeed), 0, 1), 0, 1, 0, MAX_SPEED);
                    rightSpeed = map(constrain(abs(rightSpeed), 0, 1), 0, 1, 0, MAX_SPEED);
           

					// Met à jour les moteurs avec les vitesses calculées
					wheelL->setSpeed(leftSpeed);   
					wheelR->setSpeed(rightSpeed);  
					
					wheelL->setDirection(true, leftSpeed >= 0);
                    wheelR->setDirection(true, rightSpeed >= 0);
				
					wheelL->update();
					wheelR->update();	

					double distanceToTarget = sqrt(pow(target.x - odometry.getX(), 2) +
                                                   pow(target.y - odometry.getY(), 2));

                    /*
					if (sensor) {
						sensor->update();  
						double distance = sensor->getDistance();
                        /*
						Serial.print("Test Capteur Ultrasonique - Distance : ");
						Serial.print(distance);
						Serial.println(" cm");
                        
					}*/
					
					if (distanceToTarget < 0.05) {
                        Serial.println("Waypoint reached!");
                        trajectory.advanceWaypoint();
                        if (trajectory.isFinished()) {
                            Serial.println("Navigation completed!");
                            state = RobotState::STOPPED;
                        }
                    }
                } else {
                    state = RobotState::STOPPED;
                }
                break;

            case RobotState::AVOIDING:
                /*Serial.println("Avoiding obstacle...");
                wheelL->setSpeed(-100);
                wheelR->setSpeed(100);
                delay(500); // Rotate for 0.5 sec
                wheelL->setSpeed(100);
                wheelR->setSpeed(100);
                delay(300); // Move forward slightly

                if (sensor->getDistance() > obstacleThreshold) {
                    Serial.println("Path cleared. Resuming...");
                    state = RobotState::NAVIGATING;
                }
                break;*/
				
				state = RobotState::NAVIGATING;
                break;

            case RobotState::EXECUTING_TASK:
                if (millis() - taskStartTime >= 1000) { // Wait 1 sec non-blocking
                    Serial.println("Task completed. Moving to next waypoint...");
                    state = RobotState::NAVIGATING;
                }
                break;

            case RobotState::STOPPED:
                Serial.println("Mission completed!");
                wheelL->update();
                wheelR->update();
                break;

            case RobotState::DANCING:
                wheelL->setSpeed(0);
                wheelR->setSpeed(0);
                if (millis() - taskStartTime >= 3000) { // Wait 3 sec
                    Serial.println("Dancing!");
                    motorUP->enable(); 
                    motorUP->update();
                    state = RobotState::STOPPED;
                }
                
                break;
            
        }
    }

    void addWaypoint(double x, double y, double theta) {
        trajectory.addWaypoint(x, y, theta);
        if (state == RobotState::IDLE) {
            state = RobotState::NAVIGATING;           
        }
    }

    void stop() {
        state = RobotState::STOPPED;
    }
};

#endif // ROBOT_H
