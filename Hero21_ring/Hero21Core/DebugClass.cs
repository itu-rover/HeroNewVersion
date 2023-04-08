
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
    class DebugClass
    {

        private static string finalMsg;
        private static string motorInfo;
        private static string encoderInfo;

        public enum SysDebugModes
        {
            position,
            voltage
        }

        public enum specCases
        {
            mainLoop,
            joystickControl,
            emergencyStop,
            resetSensors
        }

        public static void LogCustomMsg(string msg)
        {
            msg = PadLeft(msg,50,'_');
            CheckAndSendMsg(msg);
            Watchdog.Feed();
        }
        public static void LogSpecCases(specCases mode)
        {

            switch (mode)
            {
                case specCases.mainLoop:
                    finalMsg = "Main loop is running";
                    break;
                case specCases.joystickControl:
                    finalMsg = "Joystick control mode is executing";
                    break;
                case specCases.emergencyStop:
                    finalMsg = "EMERGENCY STOP";
                    break;
                case specCases.resetSensors:
                    finalMsg = "Reset sensors";
                    break;
                default:
                    finalMsg = "";
                    break;
            }

            finalMsg = PadLeft(finalMsg, 50, '_');
            CheckAndSendMsg(finalMsg);

        }


        public static void LogMainLoop()
        {
            finalMsg = "Main loop is running";
            finalMsg = PadLeft(finalMsg, 50, '_');
            if (SerialCom._uartDebug.CanWrite)
                SerialCom.WriteDebug(finalMsg);
            Watchdog.Feed();
        }

        // mode = 0: position, mode = 1 voltage  ref.
        public static void LogSysCommands(SysDebugModes mode, int motorNum, int[] commandArray)
        {
            switch (mode)
            {
                case SysDebugModes.position:
                    finalMsg = "Positon commands: ";
                    break;
                case SysDebugModes.voltage:
                    finalMsg = "Voltage commands: ";
                    break;
                default:
                    finalMsg = "";
                    break;
            }


            for (int i = 0; i < motorNum; i++)
            {
                motorInfo = commandArray[i].ToString();
                finalMsg += motorInfo;
                finalMsg += " ";
            }

            finalMsg = PadLeft(finalMsg, 50, '_');

            CheckAndSendMsg(finalMsg);
        }




        private static string PadLeft(string strToBePadded, int length, char paddingElement)
        {
            while (strToBePadded.Length < length)
            {
                strToBePadded = strToBePadded + paddingElement;
            }
            return strToBePadded;
        }

        private static void CheckAndSendMsg(string msg)
        {
            if (SerialCom._uartDebug.CanWrite)
                SerialCom.WriteDebug(msg);
            Watchdog.Feed();
        }

    }
}
