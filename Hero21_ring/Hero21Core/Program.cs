//#define DEBUG

using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;


using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Motion;
using CTRE.Phoenix.MotorControl;


namespace Hero21Core
{
    public class Program
    {

        public static void Main()
        {
            // DO ALL THE INITIALIZATION STUFF HERE!
            SerialCom.InitializeSerialCom();
            RoboticArm.InitializeRoboticArmSys();

            
            
            // For ring buffer implementation
            byte[] scratch = new byte[1];

            while (true)
            {
                Watchdog.Feed();
                SerialCom.IncreaseNoMsgCounter();
                // GET THE SENSOR DATA AND TRANSMIT IT TO THE HIGH LEVEL CONTROLLER HERE!


                if (SerialCom._uart.CanWrite)
                {
                    SerialCom.SendFeedbackMsg();   
                }

                // GET SERIAL DATA AND PROCESS IT TO SET COMMANDS FOR MOTOR DRIVERS HERE!
                if (SerialCom._uart.BytesToRead > 0)
                {
                    int readCnt = SerialCom._uart.Read(SerialCom._rx, 0, SerialCom.CalcRemainingCap());
      
                    for (int i = 0; i < readCnt; ++i)
                    {

                        SerialCom.ReadCommand(SerialCom._rx[i]);                   
                    }
                       
                }

                if (SerialCom.CheckNoMsgCnt() == false || SerialCom.CheckSerialErrCnt() == false)
                {
                    // EMERGENCY STOP CONDITION
                    RoboticArm.StopArmActuators();
                    DebugClass.LogSpecCases(DebugClass.specCases.emergencyStop);
                    Debug.Print("EMERGENCY STOP");
                }
                



                if (SerialCom._txCnt > 0)
                { 
                    scratch[0] = SerialCom.PopByte();
                }
                

            }
        }
    }
}