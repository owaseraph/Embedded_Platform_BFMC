#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "utils/task.hpp"
#include "drivers/speedingmotor.hpp"
#include "drivers/steeringmotor.hpp"

namespace brain
{
   /**
    * @brief Class autonomous
    *
    */
    class CAutonomous: public utils::CTask
    {
        public:
            /* Constructor */
            CAutonomous(
                uint32_t f_period,
                drivers::CSpeedingMotor& f_speedMotor,
                drivers::CSteeringMotor& f_steerMotor
            );
            virtual ~CAutonomous();
            //start moving
            //Command format: #autonomous:1;;
            void serialCallbackAutonomousCommand(char const * message, char * response);

        private:
            virtual void _run(); //loop function
            drivers::CSpeedingMotor& m_speedMotor;
            drivers::CSteeringMotor& m_steerMotor;

            enum State{
                IDLE, //do nothing
                PHASE_1_FWD, //go forward
                PHASE_2_STOP, //stop
                PHASE_3_LEFT, //turn left + fwd
                PHASE_4_REV, //turn right + rev
                PHASE_5_EXIT //drive away
            };
            State m_state;
            int m_counter; //time variable



    }; // class CAutonomous
}; // namespace brain

#endif // AUTONOMOUS_HPP
