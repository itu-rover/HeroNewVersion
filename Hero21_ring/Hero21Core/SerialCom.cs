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
    class SerialCom
    {
        public static System.IO.Ports.SerialPort _uart = new System.IO.Ports.SerialPort(CTRE.HERO.IO.Port4.UART, 115200);
        public static System.IO.Ports.SerialPort _uartDebug = new System.IO.Ports.SerialPort(CTRE.HERO.IO.Port1.UART, 115200);

        public static byte[] _rx = new byte[1024];
        static byte[] _tx = new byte[1024];
        public static int _txIn = 0;
        public static int _txOut = 0;
        public static int _txCnt = 0;
        public static int readCnt;

        private static int receiveCounter = 0;
        private static int resetCounter = 0;
        public static bool didReset = false;
        private static bool receiveFlag = false;
        private static int[] incomingData = new int[50];    // 30 is arbitrarily given (24 + 1)
        public static bool assignCommands = false;
        private static int checkCurr = 0;
        private static int checkPrev = 1;

        private static int serialErrCounter = 0;
        private static int serialErrCounterTreshold = 5;
        private static int noNewMsgCounter = 0;             // it increases when there is no new msg, reset if new msg is available
        private static int noNewMsgCounterTresh = 10;
        //TODO: If no new messages are coming, stop the motors

        private static int armMsgLen = 24;                  // string length of arm msgs
        private static int strPieceLen = 4;                 // string length of each motor info
        private static int commCheckByteIdx = armMsgLen;    // steering will be added

        public static int[] armCommandsArray = new int[RoboticArm.armMotorNum + 1];
        //public static int[] steeringCommandsArray = new int[SteeringSys.armMotorNum];

        private static string armFeedbackMsg = "";
        private static string steeringFeedbackMsg = "";
        private static string finalFeedbackMsg = "";


        private static int resetCntTresh = 5;
        private static int positionCmdCntTresh = 25;
        private static int voltageCmdCntTresh = 8;

        private static int startChar = 83;      // S
        private static int finishChar = 70;     // F
        private static int resetChar = 82;      // R
        private static int absoluteEncChar = 65; // A
        private static int toggleChar;
        private static int homeChar;

        /*
         * This function initializes the serial communication -> it opens the serial port
         */
        public static void InitializeSerialCom()
        {
            _uart.Open();
            _uartDebug.Open();
            Watchdog.Feed();
        }

        /*
         * This function converts string argument to bytes and then transmits it via uart
         */
        private static void Write(string msg)
        {
            byte[] msg_byte = String2Byte(msg);
            Watchdog.Feed();
            _uart.Write(msg_byte, 0, msg.Length);
        }

        public static void WriteDebug(string msg)
        {
            byte[] msg_byte = String2Byte(msg);
            Watchdog.Feed();
            _uartDebug.Write(msg_byte, 0, msg.Length);
        }

        /*
         * This function converts string into bytes
         */
        private static byte[] String2Byte(String msg) // return string type to byte type for better processing
        {
            byte[] retval = new byte[msg.Length];
            for (int i = 0; i < msg.Length; ++i)
                retval[i] = (byte)msg[i];
            Watchdog.Feed();
            return retval;
        }



        /*
         * This function gets systems feedback msgs and sends it via uart
         */
        public static void SendFeedbackMsg()
        {
            armFeedbackMsg = RoboticArm.GetArmFeedback();
            //TODO: Get steering system Feedback string
            string isPidRunStr = (RoboticArm.isPIDRunning() ? "Y" : "N");
            finalFeedbackMsg = "A" + armFeedbackMsg + isPidRunStr + "B";
            Debug.Print(finalFeedbackMsg);
            Write(finalFeedbackMsg);

        }

        public static void SendErrMsg()
        {
            string err = "EMERGENCY STOP !!!!!!!!!";
            Write(err);
        }

        // Rest are the methods used for cicular buffer
        public static int CalcRemainingCap()
        {
            /* first calc the remaining capacity in the ring buffer */
            int rem = _tx.Length - _txCnt;
            /* cap the return to the maximum capacity of the rx array */
            if (rem > _rx.Length)
                rem = _rx.Length;
            return rem;
        }

        /** @param received byte to push into ring buffer */
        public static void PushByte(byte datum)
        {
            _tx[_txIn] = datum;
            if (++_txIn >= _tx.Length)
                _txIn = 0;
            ++_txCnt;
        }

        /** 
         * Pop the oldest byte out of the ring buffer.
         * Caller must ensure there is at least one byte to pop out by checking _txCnt.
         * @return the oldest byte in buffer.
         */
        public static byte PopByte()
        {
            byte retval = _tx[_txOut];
            if (++_txOut >= _tx.Length)
                _txOut = 0;
            --_txCnt;
            return retval;
        }


        public unsafe static void ReadCommand(byte incomingByte)
        {
            /*
             * RRRRR: Reset arm sensors
             * TTTTT: Toggle PID controller
             * S   _ _ _ _    _ _ _ _    _ _ _ _    _ _ _ _    _ _ _ _    _ _ _ _    _   F: Position command
             * S _ _ _ _ _ _ F: Voltage command
             */
            int incASCII = (int)incomingByte;
            int* incASCIIPtr = &incASCII;

            if (CheckMsgResetChar(incASCIIPtr) && receiveFlag == false)    // Capture 'R' character
            {
                resetCounter = 0;
                receiveFlag = true; 
            }

            if (CheckMsgStartChar(incASCIIPtr))    // Capture 'S' character
            {
                resetCounter = 0;
                receiveFlag = true;
                receiveCounter = 0;
            }
            else if (CheckGetCmdsCondition(incASCIIPtr))      // If not 'S','F' and 'R'
            {
                incomingData[receiveCounter] = (incASCII) - 48;                    // ASCII to integer conversion
                receiveCounter++;
            }
            else if  (CheckFinishCondition(incASCIIPtr, finishChar))
            {

                CheckMsgContinuity(receiveCounter - 1);

                if (receiveCounter == positionCmdCntTresh && CheckSerialErrCnt() == true)
                    RoboticArm.ExecuteArmPositionCommands();
                else if (receiveCounter == voltageCmdCntTresh && CheckSerialErrCnt() == true)
                    RoboticArm.ExecuteArmVoltageCommands();

                receiveCounter = 0;
                noNewMsgCounter = 0;                   // Reset since new msg is available
                Debug.Print("No new msg counter reset!");
                receiveFlag = false;
                
            }

            else if (CheckFinishCondition(incASCIIPtr, absoluteEncChar))
            {
                //HandleEncoderCmdException(receiveCounter - 1);

                AssignArmPositionCmds();
                RoboticArm.ResetArmSensors(armCommandsArray);
                ResetSerialReadFlags();
            }

            else if (receiveFlag == true && CheckMsgResetChar(incASCIIPtr))
            {
                resetCounter++;
                if (resetCounter == resetCntTresh)
                {
                    RoboticArm.ResetArmSensors(RoboticArm.armHomePositions);
                    Debug.Print("RESET ARM SENSORS");
                    resetCounter = 0;
                    didReset = true;
                }
            }
        }

        private unsafe static bool CheckMsgStartChar(int* incomingASCII)
        {
            if (*incomingASCII == startChar)
                return true;
            else
                return false;
        }

        private unsafe static bool CheckMsgResetChar(int* incomingASCII)
        {
            if (*incomingASCII == resetChar)
                return true;
            else
                return false;
        }

        private unsafe static bool CheckGetCmdsCondition(int* incomingASCII)
        {
            if (receiveFlag == false)
                return false;
            else if (*incomingASCII == finishChar)
                return false;
            else if (*incomingASCII == resetChar)
                return false;
            else
                return true;
        }

        private unsafe static bool CheckFinishCondition(int* incomingASCII, int finishTrigChar)
        {
            if (receiveFlag == true && *incomingASCII == finishTrigChar)
                return true;
            else
                return false;

        }

        private static void ResetSerialReadFlags()
        {
            receiveCounter = 0;
            noNewMsgCounter = 0;
            receiveFlag = false;
        }


        public static void AssignArmVoltageCmds() {
            for (int i = 0; i < RoboticArm.armMotorNum + 1; i++)
            {
                armCommandsArray[i] = incomingData[i];
                // Debug.Print(incomingData[i].ToString());
            }
        }
        
        public static void AssignArmPositionCmds()
        {
 
            for (int i = 0, j = 0; i < armMsgLen; i += strPieceLen, j++)
            {
                armCommandsArray[j] = (incomingData[i] * 1000) + (incomingData[i + 1] * 100 + incomingData[i + 2] * 10 + incomingData[i + 3]);
                
            }
            DebugClass.LogCustomMsg("Arm position commands are set");
            
        }



        /*
         * This function is used to check if the incoming serial message is continuously updating
         * Last byte of the serial message changes with each message. If that is the case for receiver, a continuous stream is happening
         * If any match is detected, the commCheck changed to be 'false' to inform the program that high level controller is always sending the same message which is an error
         * Also it increments or resets the error counter
         */
        public static void CheckMsgContinuity(int byteToBeChecked)
        {
            checkCurr = incomingData[byteToBeChecked];

            serialErrCounter = (checkCurr == checkPrev) ? serialErrCounter += 1 : serialErrCounter = 0;

            DebugClass.LogCustomMsg("Serial error counter: " + serialErrCounter.ToString());

            checkPrev = checkCurr;
        }

        /*
         * This function checks if 'serialErrCounter' is below or above 'serialErrCountertreshold'
         * Serial counter is incremented by one if any discontinuity is detected in the serial message.
         * It is being set to zero if the incoming message is continuously updating
         * 'serialErrCounterTreshold' is the variable that determines how many errors are expandable
         */
        public static bool CheckSerialErrCnt()
        {
            if (serialErrCounter < serialErrCounterTreshold)
            {
                DebugClass.LogCustomMsg("No problem rn");
                return true;
            }
            else
            {
                DebugClass.LogCustomMsg("Serial error counter exceeded the treshold");
                return false;
            }
        }

        /*
         * This function increases noNewMsgCounter by 1 if no new message has arrived yet. This value resets
         * in the ReadCommand() method
         */
        public static void IncreaseNoMsgCounter()
        {
            noNewMsgCounter = noNewMsgCounter + 1;
            Debug.Print("NO NEW MSG CNT HAS INCREASED");
        }

        /*
         * This function checks if 'noNewMsgCounter' is below or above 'serialErrCountertreshold'
         * TODO: CheckNoMsgCnt and CheckSerialErrCnt might be merged
         */
        public static bool CheckNoMsgCnt()
        {
            if (noNewMsgCounter < noNewMsgCounterTresh)
                return true;
            else
                return false;
        }

    }
}

