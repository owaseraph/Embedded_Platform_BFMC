#include "brain/autonomous.hpp"

// TODO: Add your code here
namespace brain
{
   /**
    * @brief Class constructor autonomous
    *
    */
    CAutonomous::CAutonomous(uint32_t f_period,
                drivers::CSpeedingMotor& f_speedMotor,
                drivers::CSteeringMotor& f_steerMotor): 
                utils::CTask(std::chrono::milliseconds(f_period)),
                m_speedMotor(f_speedMotor),
                m_steerMotor(f_steerMotor),
                m_state(IDLE),
                m_counter(0){}

    /** @brief  CAutonomous class destructor
     */
    CAutonomous::~CAutonomous()
    {
    };

    void CAutonomous::serialCallbackAutonomousCommand(char const * message, char * response)
    {
        m_state=PHASE_1_FWD;
        m_counter=0;
        sprintf(response, "3 Move Turn Started!");
    }

    void CAutonomous::_run(){
        if(m_state==IDLE) return; //do nothing if not started
        
        m_counter++;//increase timer every time this runs

        switch(m_state){
            case IDLE: 
                break;
            case PHASE_1_FWD:
                m_speedMotor.setSpeed(150);
                m_steerMotor.setAngle(0);
                if(m_counter>20){
                    m_state=PHASE_2_STOP;
                    m_counter=0;
                }
                break;
            

            case PHASE_2_STOP:
                m_speedMotor.setSpeed(0);
                if(m_counter>5){
                    m_state=PHASE_3_LEFT;
                    m_counter=0;
                }
                break;

            case PHASE_3_LEFT:
                m_steerMotor.setAngle(-230);
                m_speedMotor.setSpeed(150);
                if(m_counter>15){
                    m_state = PHASE_4_REV;
                    m_counter = 0;
                }
                break;

            case PHASE_4_REV:
                m_steerMotor.setAngle(230);
                m_speedMotor.setSpeed(-150);
                if(m_counter>15){
                    m_state=PHASE_5_EXIT;
                    m_counter=0;
                }
                break;

            case PHASE_5_EXIT:
                m_steerMotor.setAngle(0);
                m_speedMotor.setSpeed(150);
                if(m_counter>10){
                    m_speedMotor.setSpeed(0);
                    m_state=IDLE;
                    m_counter=0;
                }
                break;
        }


    }

}; // namespace brain