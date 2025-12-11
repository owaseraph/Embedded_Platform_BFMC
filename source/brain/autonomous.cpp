/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 */

#include "brain/autonomous.hpp"

namespace brain
{
    /**
     * @brief CAutonomous Class constructor
     */
    CAutonomous::CAutonomous(
            uint32_t f_period,
            drivers::CSpeedingMotor& f_speedMotor,
            drivers::CSteeringMotor& f_steerMotor,
            UnbufferedSerial& f_serialPort
        ) 
        : utils::CTask(std::chrono::milliseconds(f_period))
        , m_speedMotor(f_speedMotor)
        , m_steerMotor(f_steerMotor)
        , m_serialPort(f_serialPort)
        , m_state(IDLE)
        , m_ticksRun(0)
        , m_period(f_period)
    {
        // Safety: Ensure motor is stopped on boot
        m_speedMotor.setSpeed(0);
    }

    /** @brief Class destructor */
    CAutonomous::~CAutonomous()
    {
    }

    /**
     * @brief Run method contains the main logic.
     * It executes the maneuver phases based on time (m_ticksRun).
     */
    void CAutonomous::_run()
    {
        if (m_state == IDLE) return;

        m_ticksRun += m_period;

        char buffer[64]; //debug messages

        switch(m_state)
        {
            case PHASE_1_FWD:
                m_speedMotor.setSpeed(150);
                m_steerMotor.setAngle(0);

                //send debug aswell as start motion
                if (m_ticksRun == m_period) {
                    int len = sprintf(buffer, "@debug:Phase 1 - Forward;;\r\n");
                    m_serialPort.write(buffer, len);
                }

                if (m_ticksRun >= 2000) {
                    m_state = PHASE_2_STOP;
                    m_ticksRun = 0; //reset timer for next phase
                }
                break;

            // PHASE 2: Stop for 0.5 Seconds
            case PHASE_2_STOP:
                m_speedMotor.setSpeed(0);

                if (m_ticksRun == m_period) {
                    int len = sprintf(buffer, "@debug:Phase 2 - Stopping;;\r\n");
                    m_serialPort.write(buffer, len);
                }

                if (m_ticksRun >= 500) {
                    m_state = PHASE_3_LEFT;
                    m_ticksRun = 0;
                }
                break;

            // PHASE 3: Turn Left + Forward for 1.5 Seconds
            case PHASE_3_LEFT:
                m_steerMotor.setAngle(-230);
                m_speedMotor.setSpeed(150);

                if (m_ticksRun == m_period) {
                    int len = sprintf(buffer, "@debug:Phase 3 - Turn Left;;\r\n");
                    m_serialPort.write(buffer, len);
                }

                if (m_ticksRun >= 1500) {
                    m_state = PHASE_4_REV;
                    m_ticksRun = 0;
                }
                break;

            // PHASE 4: Turn Right + Reverse for 1.5 Seconds
            case PHASE_4_REV:
                m_steerMotor.setAngle(230); 
                m_speedMotor.setSpeed(-150); 

                if (m_ticksRun == m_period) {
                    int len = sprintf(buffer, "@debug:Phase 4 - Reverse;;\r\n");
                    m_serialPort.write(buffer, len);
                }

                if (m_ticksRun >= 1500) {
                    m_state = PHASE_5_EXIT;
                    m_ticksRun = 0;
                }
                break;

            // PHASE 5: Drive Away for 1 Second
            case PHASE_5_EXIT:
                m_steerMotor.setAngle(0);
                m_speedMotor.setSpeed(150);

                if (m_ticksRun == m_period) {
                    int len = sprintf(buffer, "@debug:Phase 5 - Exit;;\r\n");
                    m_serialPort.write(buffer, len);
                }

                if (m_ticksRun >= 1000) {
                    // CRITICAL SAFETY STOP
                    m_speedMotor.setSpeed(0); 
                    
                    int len = sprintf(buffer, "@debug:Maneuver Complete;;\r\n");
                    m_serialPort.write(buffer, len);

                    m_state = IDLE;
                    m_ticksRun = 0;
                }
                break;
            
            default:
                m_state = IDLE;
                break;
        }
    }

    /**
     * @brief Serial callback method
     * Resets the state machine to start the maneuver.
     */
    void CAutonomous::serialCallbackAutonomousCommand(char const * message, char * response)
    {
        // Simple command to start: #auto:1;;
        
        m_state = PHASE_1_FWD;
        m_ticksRun = 0;
        
        sprintf(response, "Maneuver Started");
    }

}; // namespace brain