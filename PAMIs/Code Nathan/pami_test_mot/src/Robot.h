#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <LinkedList.h>
#include "Wheel.h"
//#include "Sensor.h"
#include "Odometry.h"
#include "Trajectory.h"
#include "MotionController.h"
#include "RobotState.h"
//#include "UltrasonicSensor.h"

class Robot {
private:
    LinkedList<Component*> components;
    Odometry odometry;
    Trajectory trajectory;
    MotionController motionController;
    RobotState state;
    double obstacleThreshold = 20.0; // Stop if closer than 20 cm
    unsigned long taskStartTime = 0; // For non-blocking task execution

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
        //UltrasonicSensor* sensor = static_cast<UltrasonicSensor*>(getComponent("ultrasonic"));


        //if (!wheelL || !wheelR || !sensor) return;
		if (!wheelL || !wheelR) return;

        odometry.update(wheelL->getEncoderValue(), wheelR->getEncoderValue());
        //double distance = sensor->getDistance();

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
					
					// int pwmLeft = constrain(int(fabs(leftSpeed) * 255.0 / 1.0), 0, 255);
					// int pwmRight = constrain(int(fabs(rightSpeed) * 255.0 / 1.0), 0, 255);
					
					bool dirL = leftSpeed >= 0;
					bool dirR = rightSpeed >= 0;
					
					// Met à jour les moteurs avec les vitesses calculées
					wheelL->setSpeed(leftSpeed);   
					wheelR->setSpeed(rightSpeed);  
					
					wheelL->setDirection(true, dirL);
					wheelR->setDirection(true, dirR);
					
					// Déplace les moteurs
					wheelL->update();
					wheelR->update();	
					
					// Debug pour vérifier les vitesses
					Serial.print("Left Speed: ");
					Serial.print(leftSpeed);
					Serial.print(" | Right Speed: ");
					Serial.println(rightSpeed);
					
					double distanceToTarget = sqrt(pow(target.x - odometry.getX(), 2) +
                                                   pow(target.y - odometry.getY(), 2));

                    if (distanceToTarget < 0.05) {
                        trajectory.advanceWaypoint();
                        taskStartTime = millis(); // Start task timer
                        state = RobotState::STOPPED;
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
				
				Serial.println("Skipping obstacle (no sensor available). Resuming navigation.");
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
