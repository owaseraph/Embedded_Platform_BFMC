/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 */

#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include <utils/task.hpp>
#include <drivers/speedingmotor.hpp>
#include <drivers/steeringmotor.hpp>
#include <mbed.h>

namespace brain
{
    /**
     * @brief CAutonomous Class
     * Implements a simple state machine for a fixed maneuver: Forward -> Stop -> Turn -> Reverse -> Exit.
     */
    class CAutonomous : public utils::CTask
    {
    public:
        /**
         * @brief Class constructor
         * * @param f_period          Task period in milliseconds
         * @param f_speedMotor      Reference to speeding motor driver
         * @param f_steerMotor      Reference to steering motor driver
         * @param f_serialPort      Reference to serial object for debug messages
         */
        CAutonomous(
            uint32_t f_period,
            drivers::CSpeedingMotor& f_speedMotor,
            drivers::CSteeringMotor& f_steerMotor,
            UnbufferedSerial& f_serialPort
        );

        /** @brief Class destructor */
        virtual ~CAutonomous();

        /** * @brief Serial callback to trigger the maneuver
         * Command: #auto:1;; 
         */
        void serialCallbackAutonomousCommand(char const * message, char * response);

    private:
        /** @brief */
        virtual void _run();

        drivers::CSpeedingMotor& m_speedMotor;
        drivers::CSteeringMotor& m_steerMotor;
        UnbufferedSerial& m_serialPort;

        uint8_t m_state;
        uint32_t m_ticksRun;
        uint32_t m_period;     

        enum State {
            IDLE = 0,
            PHASE_1_FWD,
            PHASE_2_STOP,
            PHASE_3_LEFT,
            PHASE_4_REV,
            PHASE_5_EXIT
        };
    };
}; // namespace brain

#endif // AUTONOMOUS_HPP