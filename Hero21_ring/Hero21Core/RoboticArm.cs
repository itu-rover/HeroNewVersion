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
    class RoboticArm
    {
        // Robotic arm motor drivers
        public static TalonSRX armAxis1 = new TalonSRX(1);
        public static TalonSRX armAxis2 = new TalonSRX(2);
        public static TalonSRX armAxis3 = new TalonSRX(3);
        public static TalonSRX armAxis4 = new TalonSRX(4);
        public static TalonSRX armAxis5 = new TalonSRX(5);
        public static TalonSRX armAxis6 = new TalonSRX(6);
        public static TalonSRX armGripper = new TalonSRX(7);

        public const int timeOutMs = 30;

        public static int armMotorNum = 6; // Including gripper

        public static int armMode = 0;
        // 0 -> position command, 1 -> velocity command ...


        public static int[] armHomePositions = {2048, 1024, 0, 81920, 20480, 81920 };
        public static int[] armPositionCommands = new int[armMotorNum];
        public static double[] armEffortCommands = new double[armMotorNum+1];
        private static int[] prevEncoderData = { 2048, 1024, 0, 81920, 20480, 81920 };


        /*
         * This function factory defaults all talons to prevent unexpected behaviour. It is used to initialize the robotic arm talons
         */
        public static void SetFactoryDefault()
        { 
            armAxis1.ConfigFactoryDefault();
            armAxis2.ConfigFactoryDefault();
            armAxis3.ConfigFactoryDefault();           
            armAxis4.ConfigFactoryDefault();         
            armAxis5.ConfigFactoryDefault();
            armAxis6.ConfigFactoryDefault();
            armGripper.ConfigFactoryDefault();
            /*
           armGripper.ConfigFactoryDefault();
          */
            Watchdog.Feed();
        }

        /*
         * This function configures the CTRE_MagEncoders that is used on the robotic arm. It is used to initialize the robotic arm talons and sensors.
         */
        public static void ConfigureEncoders()
        {
            armAxis1.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
            armAxis2.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
            armAxis3.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);

            armAxis4.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);        
            armAxis5.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);            
            armAxis6.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
            /*
           armGripper.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
           */
            Watchdog.Feed();
        }

        /*
         * This function sets specific parameters to specific talons if necessary. It is used to initialize the robotic arm system
         */
        public static void SetAxisSpecificParams()
        {
            // AXIS 1
            armAxis1.Config_kP(0, 24f, timeOutMs);
            armAxis1.ConfigMotionAcceleration(20, timeOutMs);
            armAxis1.ConfigMotionCruiseVelocity(10, timeOutMs);
            armAxis1.ConfigMotionSCurveStrength(1, timeOutMs);

            // AXIS 2
            armAxis2.Config_kP(0, 8f, timeOutMs);
            armAxis2.ConfigMotionAcceleration(18, timeOutMs);
            armAxis2.ConfigMotionCruiseVelocity(9, timeOutMs);
            armAxis2.ConfigMotionSCurveStrength(1, timeOutMs);

            // AXIS 3
            armAxis3.Config_kP(0, 5f, timeOutMs);
            armAxis3.ConfigClosedLoopPeakOutput(0, 0.5f, timeOutMs);
            armAxis3.ConfigMotionAcceleration(250, timeOutMs);
            armAxis3.ConfigMotionCruiseVelocity(350, timeOutMs);
            armAxis3.ConfigMotionSCurveStrength(1, timeOutMs);

            // AXIS 4
            armAxis4.Config_kP(0, 48f, timeOutMs);
            armAxis4.ConfigMotionAcceleration(600, timeOutMs);
            armAxis4.ConfigMotionCruiseVelocity(350, timeOutMs);
            armAxis4.ConfigMotionSCurveStrength(1, timeOutMs);

            // AXIS 5
            armAxis5.Config_kP(0, 40f, timeOutMs);
            armAxis5.ConfigMotionAcceleration(600, timeOutMs);
            armAxis5.ConfigMotionCruiseVelocity(350, timeOutMs);
            armAxis5.ConfigMotionSCurveStrength(1, timeOutMs);

            // AXIS 6
            armAxis6.Config_kP(0, 40f, timeOutMs);
            armAxis6.ConfigMotionAcceleration(600, timeOutMs);
            armAxis6.ConfigMotionCruiseVelocity(350, timeOutMs);
            armAxis6.ConfigMotionSCurveStrength(1, timeOutMs);
            Watchdog.Feed();


            // AXIS 6
            armGripper.Config_kP(0, 12f, timeOutMs);
            armGripper.ConfigMotionAcceleration(50, timeOutMs);
            armGripper.ConfigMotionCruiseVelocity(5, timeOutMs);
            armGripper.ConfigMotionSCurveStrength(1, timeOutMs);
            Watchdog.Feed();

            //armGripper.ConfigVoltageCompSaturation(12, timeOutMs);

        }

        /*
         * This function sets the sensor phases. It is used to initialize the robotic arm talons and sensors.
         */
        public static void SetEncoderPhases()
        {
            armAxis1.SetSensorPhase(false);            
            armAxis2.SetSensorPhase(false);            
            armAxis3.SetSensorPhase(false);           
            armAxis4.SetSensorPhase(true);            
            armAxis5.SetSensorPhase(true);
            armAxis6.SetSensorPhase(true);
            //armGripper.SetSensorPhase(true);            
            Watchdog.Feed();
            armAxis1.SetInverted(false);
            armAxis2.SetInverted(false);
            armAxis3.SetInverted(false);
            armAxis4.SetInverted(true);
            armAxis5.SetInverted(true);
            armAxis6.SetInverted(true);
            Watchdog.Feed();
        }

        /*
         * This function selects profile slots of the Talon SRX motor drivers
         */
        public static void SelectTalonsProfileSlots()
        {
            armAxis1.SelectProfileSlot(0, 0);            
            armAxis2.SelectProfileSlot(0, 0);            
            armAxis3.SelectProfileSlot(0, 0);          
            armAxis4.SelectProfileSlot(0, 0);            
            armAxis5.SelectProfileSlot(0, 0);            
            armAxis6.SelectProfileSlot(0, 0);
            armGripper.SelectProfileSlot(0, 0);           
            Watchdog.Feed();
        }

        /*
         * This function resets the position sensors data. It can be used to initialize the sensors and 
         * it can also be used to configure the robotic arm physically
         */
        public static void ResetArmSensors(int[] resetCommands)
        {
            armAxis1.SetSelectedSensorPosition(resetCommands[0]);
            armAxis2.SetSelectedSensorPosition(resetCommands[1]);           
            armAxis3.SetSelectedSensorPosition(resetCommands[2]);            
            armAxis4.SetSelectedSensorPosition(resetCommands[3]);            
            armAxis5.SetSelectedSensorPosition(resetCommands[4]);
            armAxis6.SetSelectedSensorPosition(resetCommands[5]);

            DebugClass.LogCustomMsg("Reset arm sensors");

            Watchdog.Feed();
        }

        /*
         * This function calls all the necessary methods respectively to initialize the robotic arm system.
         * It it enough to call this method in the main program
         */
        public static void InitializeRoboticArmSys()
        {
            SetFactoryDefault();
            ConfigureEncoders();
            SetAxisSpecificParams();
            SetEncoderPhases();
            SelectTalonsProfileSlots();
            //ResetArmSensors();
        }

        /*
         * This function sets position commands to the motor drivers
         * Motor drivers have their own feedback control mechanism to perform necessary action.
         * Position commands are passed as encoder ticks and reduction rate is considered.
         */
        public static void SetPositionCommand()
        {
            DebugClass.LogSysCommands(DebugClass.SysDebugModes.position, armMotorNum, armPositionCommands);
            double maxFeedForward = -0.6;

            // This is the angle of 2nd and 3rd joint combined. 0 radians corresponds to where the arm is perpendicular to the ground. 
            //double currentAngle = ((double)prevEncoderData[1] / 2048.0 + (double)prevEncoderData[1] / 2048.0 - 1) * System.Math.PI;

            armAxis1.Set(ControlMode.MotionMagic, Utils.Clamp(armPositionCommands[0], 0, 4096));
            armAxis2.Set(ControlMode.MotionMagic, Utils.Clamp(armPositionCommands[1], 224, 1024 + 600));
            armAxis3.Set(ControlMode.MotionMagic, Utils.Clamp((int)Utils.Map((double)armPositionCommands[2], 0, 9999, 0, 49152), 1000, 49152));
            //armAxis3.Set(ControlMode.MotionMagic, Utils.Clamp(armPositionCommands[2], 1000, 49152),
            //                    DemandType.ArbitraryFeedForward, maxFeedForward * System.Math.Cos(currentAngle));
            armAxis4.Set(ControlMode.MotionMagic, ((int)(((double)armPositionCommands[3] / 9999) * 81920 * 2)));
            armAxis5.Set(ControlMode.MotionMagic, ((int)(((double)armPositionCommands[4] / 9999) * 20480 * 2)));
            armAxis6.Set(ControlMode.MotionMagic, ((int)(((double)armPositionCommands[5] / 9999) * 81920 * 2)));
            armGripper.Set(ControlMode.PercentOutput, 0.0);
            Watchdog.Feed();


            Debug.Print("----------- executing POSITION commands");
            //armGripper.Set(ControlMode.Position, (int)(armPositionCommands[6] / armMappingCoefs[6]));
        }

        public static void SetEffortCommand()
        {
            DebugClass.LogSysCommands(DebugClass.SysDebugModes.voltage,armMotorNum+1,SerialCom.armCommandsArray);

            armAxis1.Set(ControlMode.PercentOutput, (double) armEffortCommands[0]);
            armAxis2.Set(ControlMode.PercentOutput, (double) armEffortCommands[1]);
            armAxis3.Set(ControlMode.PercentOutput, (double) armEffortCommands[2]);
            armAxis4.Set(ControlMode.PercentOutput, (double) armEffortCommands[3]);
            armAxis5.Set(ControlMode.PercentOutput, (double) armEffortCommands[4]);
            armAxis6.Set(ControlMode.PercentOutput, (double) armEffortCommands[5]);
            armGripper.Set(ControlMode.PercentOutput, (double) armEffortCommands[6] / 2);

            Debug.Print("----------- executing VOLTAGE commands" + armEffortCommands[0].ToString() + armEffortCommands[1].ToString() + armEffortCommands[2].ToString() + armEffortCommands[3].ToString() + armEffortCommands[4].ToString() + armEffortCommands[5].ToString() + armEffortCommands[6].ToString());

        }

        public static void ExecuteArmPositionCommands()
        {
            string[] testDebug = new string[RoboticArm.armMotorNum];

            //SerialCom.CheckMsgContinuity();

            SerialCom.AssignArmPositionCmds();
            UpdatePositionCommands(SerialCom.armCommandsArray);
            SetPositionCommand();
            SerialCom.assignCommands = false;


            testDebug[0] = SerialCom.armCommandsArray[0].ToString();
            testDebug[1] = SerialCom.armCommandsArray[1].ToString();
            testDebug[2] = SerialCom.armCommandsArray[2].ToString();
            testDebug[3] = SerialCom.armCommandsArray[3].ToString();
            testDebug[4] = SerialCom.armCommandsArray[4].ToString();
            testDebug[5] = SerialCom.armCommandsArray[5].ToString();

            Debug.Print("Commands: " + testDebug[0] + testDebug[1] + testDebug[2] + testDebug[3] + testDebug[4] + testDebug[5]);

        }

        public static void ExecuteArmVoltageCommands()
        {
            SerialCom.AssignArmVoltageCmds();
            UpdateVoltageCommands(SerialCom.armCommandsArray);
            SetEffortCommand();

        }

        /*
         * This function stops all the actuators by sending 0 voltage commands. 
         * This function should be called when a termination of the movement is desired
         */
        public static void StopArmActuators()
        {
            armAxis1.Set(ControlMode.PercentOutput, 0);
            armAxis2.Set(ControlMode.PercentOutput, 0);
            armAxis3.Set(ControlMode.PercentOutput, 0);
            armAxis4.Set(ControlMode.PercentOutput, 0);
            armAxis5.Set(ControlMode.PercentOutput, 0);
            armAxis6.Set(ControlMode.PercentOutput, 0);
            armGripper.Set(ControlMode.PercentOutput, 0);

            Watchdog.Feed();
        }

        public static string GetArmFeedback()
        {
            string armFeedback;
            int[] encoderData = new int[6];
            string[] encoderStr = new string[6];

            encoderData[0] = armAxis1.GetSelectedSensorPosition();
            encoderStr[0] = Utils.Clamp(encoderData[0], 0, 4096).ToString("D4");
            Watchdog.Feed();

            encoderData[1] = armAxis2.GetSelectedSensorPosition();
            encoderStr[1] = Utils.Clamp(encoderData[1], 224, 1024 + 600).ToString("D4");
            Watchdog.Feed();

            encoderData[2] = armAxis3.GetSelectedSensorPosition();
            encoderStr[2] = ((int)Utils.Map(Utils.Clamp(encoderData[2], 0, 49152), 0, 49152, 0, 9999)).ToString("D4");
            Watchdog.Feed();

            encoderData[3] = armAxis4.GetSelectedSensorPosition();
            encoderStr[3] = ((int) Utils.Map(Utils.Clamp(encoderData[3], 0, 81920 * 2), 0, 81920 * 2, 0, 9999)).ToString("D4");
            Watchdog.Feed();

            encoderData[4] = armAxis5.GetSelectedSensorPosition();
            encoderStr[4] = ((int) Utils.Map(Utils.Clamp(encoderData[4], 0, 20480 * 2), 0, 20480 * 2, 0, 9999)).ToString("D4");
            Watchdog.Feed();

            encoderData[5] = armAxis6.GetSelectedSensorPosition();
            encoderStr[5] = ((int) Utils.Map(Utils.Clamp(encoderData[5], 0, 81920 * 2), 0, 81920 * 2, 0, 9999)).ToString("D4");
            Watchdog.Feed();


            armFeedback = encoderStr[0] + encoderStr[1] + encoderStr[2] + encoderStr[3] + encoderStr[4] + encoderStr[5];
            //Debug.Print("Encoder feedback: ");
            //Debug.Print(armFeedback);
            if (HandleTalonKillException(encoderData))
            {
                StopArmActuators();
                ResetArmSensors(prevEncoderData);
            }
            else
                AssignPrevEncoderData(encoderData);

            return armFeedback;
        }

        /*
         * This function updates current position commmands with the new ones (passed as an argument)
         */
        public static void UpdatePositionCommands(int[] newCommands)
        {
            for (int i = 0; i < armMotorNum; i++)
            {
                armPositionCommands[i] = newCommands[i];
            }
        }

        public static void UpdateVoltageCommands(int[] newCommands)
        {
            for (int i = 0; i < armMotorNum + 1; i++)
            {
                armEffortCommands[i] = (((double) newCommands[i]) - 5.0)/ 5.0;
            }

            Debug.Print("Voltage: "+armEffortCommands[0]+ " "+armEffortCommands[1]+ armEffortCommands[2] + " " + armEffortCommands[3] + " " + armEffortCommands[4] + " " + armEffortCommands[5] + " " + armEffortCommands[6]);
        }


        /*
         * This function returns back the robotic arm to its home position
         */
        public static void GoHome()
        {
            armAxis1.Set(ControlMode.Position, armHomePositions[0]);
            armAxis2.Set(ControlMode.Position, armHomePositions[1]);
            armAxis3.Set(ControlMode.Position, armHomePositions[2]);
            armAxis4.Set(ControlMode.Position, armHomePositions[3]);
            armAxis5.Set(ControlMode.Position, armHomePositions[4]);
            armAxis6.Set(ControlMode.Position, armHomePositions[5]);
        }

        private static int Absol(int data)
        {
            return (data < 0 ? data * (-1) : data);
        }


        public static bool isPIDRunning()
        {
            if (Absol(armAxis1.GetClosedLoopError(0)) > 10)
                return true;
            else if (Absol(armAxis2.GetClosedLoopError(0)) > 15)
                return true;
            else if (Absol(armAxis3.GetClosedLoopError(0)) > 25)
                return true;
            else if (Absol(armAxis4.GetClosedLoopError(0)) > 60)
                return true;
            else if (Absol(armAxis5.GetClosedLoopError(0)) > 60)
                return true;
            else if (Absol(armAxis6.GetClosedLoopError(0)) > 6*10)
                return true;
            else
                return false;

        }

        private static void AssignPrevEncoderData(int[] encoderData)
        {
            for (int i = 0; i < armMotorNum; i++)
            {
                prevEncoderData[i] = encoderData[i];
            }
        }

        /*
         * Returns true if any corrupted data (0000 in our case) from encoder is detected 
         */
        private static bool HandleTalonKillException(int[] encoderData)
        {
            for (int i = 0; i < armMotorNum; i++)
            {
                if (encoderData[i] == 0 && i != 2)
                    return true;
            }
            return false;
        }

    }
}
