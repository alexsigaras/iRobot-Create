// (c) Copyright Alexandros Sigaras.
// alex@sigaras.com
// https://github.com/alexsigaras/iRobot-Create/
//

using System;
using System.IO.Ports;
using System.Threading;

namespace iRobotCreate.Library
{
    public class iRobot
    {
        private Sensors iRobotSensors = new Sensors();
        private SerialPort serialPort1;

        private int irobot_velocity = 100;
        private Timer Clock;    //control update rates since sensor pool to transforming data
        private byte[] sensor_list = new byte[] { 7, 27 };
        private bool dataReturnMethod = true;   //true-stream; false-query
        private int delay = 250;

        /*
         * sensor opcodes availables:
            7;     //Bumps and Wheel drops
            25;    //battery charge
            26;    //battery capacity
            21;    //charging state
            27;    //wall signal
            */

        public iRobot()
        { }

        ~iRobot()
        {
            //close and dispose serial port object
            Disconnect();
        }

        public bool Connect(ref SerialPort aux)
        {

            serialPort1 = aux;
            try
            {
                if (dataReturnMethod)//use as a stream
                    PoolSensors();
                else
                    PausePoolSensors();

                serialPort1.DataReceived += new SerialDataReceivedEventHandler(DataReceived);

                // Create the timer callback delegate.
                TimerCallback cb = new TimerCallback(Timer_Tick);
                // Create the timer. It is autostart, so creating the timer will start it.
                Clock = new System.Threading.Timer(cb, null, 1, delay);

                return serialPort1.IsOpen;
            }
            catch
            {
                return false;
            }
        }

        // 
        public bool Connect(string com_port)
        {
            try
            {
                this.serialPort1 = new SerialPort();
                this.serialPort1.PortName = com_port;
                this.serialPort1.BaudRate = 57600;
                this.serialPort1.DataBits = 8;
                this.serialPort1.DtrEnable = false;
                this.serialPort1.StopBits = StopBits.One;
                this.serialPort1.Handshake = Handshake.None;
                this.serialPort1.Parity = Parity.None;
                this.serialPort1.RtsEnable = false;
                this.serialPort1.Close();
                this.serialPort1.Open();

                //------------------------------------
                if (dataReturnMethod)//use as a stream
                    PoolSensors();
                else
                    PausePoolSensors();

                //---create a event handler for serial data recieved from irobot
                serialPort1.DataReceived += new SerialDataReceivedEventHandler(DataReceived);

                //---create a thread that acts like a timer, to control the data flow from irobot sensors
                TimerCallback cb = new TimerCallback(Timer_Tick);       // Create the timer callback delegate.
                Clock = new System.Threading.Timer(cb, null, 1, delay); // Create the timer. It is autostart, so creating the timer will start it.

                return serialPort1.IsOpen;
            }
            catch { return false; }
        }

        public bool Disconnect()
        {
            try
            {
                if (serialPort1.IsOpen)
                {
                    Clock.Dispose();
                    //BeginSensorPool(false); //stop any sensor pool
                    serialPort1.Close();
                    return true;
                }
            }
            catch
            {
                return false;
            }

            return false;
        }

        public bool IsOpen()
        {
            try
            {
                return serialPort1.IsOpen;
            }
            catch { return false; }
        }

        public void setDataReturnMethod(bool status)
        {
            dataReturnMethod = status;
        }

        public void BeginSensorPool(bool flag)
        {
            if (flag)
            {
                PoolSensors();
            }
            else
            {
                PausePoolSensors();
            }
        }

        //
        private void Timer_Tick(object sender)
        {
            if (!dataReturnMethod)
                PoolSensors();
            else
                for (int i = 0; i < sensor_list.Length; i++)
                    Bumper(sensor_list[i], GetSensorData(sensor_list[i]));
        }

        public void DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int bytes = serialPort1.BytesToRead;
            //create a byte array to hold the awaiting data
            byte[] comBuffer = new byte[bytes];
            //read the data and store it
            serialPort1.Read(comBuffer, 0, bytes);

            if (dataReturnMethod)
                iRobotSensors.setData(comBuffer);   //for stream
            else
            {
                iRobotSensors.setDataQuery(comBuffer, sensor_list);    //for query

                for (int i = 0; i < sensor_list.Length; i++)
                    Bumper(sensor_list[i], GetSensorData(sensor_list[i]));
            }
        }

        #region Kinect power

        public void serialCommand(byte[] aux)
        {
            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        public void KinectPowerOn()
        {
            byte[] aux = new byte[6];

            aux[0] = 128;
            aux[1] = 132;
            aux[2] = 147;
            aux[3] = 7;
            aux[4] = 138;
            aux[5] = 7;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        public void KinectPowerOff()
        {
            byte[] aux = new byte[6];

            aux[0] = 128;
            aux[1] = 132;
            aux[2] = 147;
            aux[3] = 0;
            aux[4] = 138;
            aux[5] = 0;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        public void powerLedGreen()
        {
            byte[] aux = new byte[6];

            aux[0] = 128;
            aux[1] = 132;
            aux[2] = 139;
            aux[3] = 2;
            aux[4] = 0;
            aux[5] = 255;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);

        }

        public void powerLedOff()
        {
            byte[] aux = new byte[6];

            aux[0] = 128;
            aux[1] = 132;
            aux[2] = 139;
            aux[3] = 2;
            aux[4] = 0;
            aux[5] = 0;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);

        }

        public void powerLedRed()
        {
            byte[] aux = new byte[6];

            aux[0] = 128;
            aux[1] = 132;
            aux[2] = 139;
            aux[3] = 2;
            aux[4] = 255;
            aux[5] = 255;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        public void powerLedGreenAdvance()
        {
            byte[] aux = new byte[6];

            aux[0] = 128;
            aux[1] = 132;
            aux[2] = 139;
            aux[3] = 8;
            aux[4] = 0;
            aux[5] = 255;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);

        }

        public void powerLedAdvanceOff()
        {
            byte[] aux = new byte[6];

            aux[0] = 128;
            aux[1] = 132;
            aux[2] = 139;
            aux[3] = 8;
            aux[4] = 0;
            aux[5] = 0;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);

        }

        public void powerLedRedAdvance()
        {
            byte[] aux = new byte[6];

            aux[0] = 128;
            aux[1] = 132;
            aux[2] = 139;
            aux[3] = 8;
            aux[4] = 255;
            aux[5] = 255;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }






        #endregion Kinect power

        //just pooling bump+wheels and battery+charging - upgrade to a clever version sooner
        private void PoolSensors()
        {
            byte[] aux = new byte[9];

            aux[0] = 128;
            aux[1] = 131;
            if (dataReturnMethod)
                aux[2] = 148;   //148-stream; 149-query
            else
                aux[2] = 149;

            aux[3] = (byte)sensor_list.Length;

            //list sensor list
            for (int i = 0; i < sensor_list.Length; i++)
                aux[4 + i] = sensor_list[i];

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }
        private void PausePoolSensors()
        {
            byte[] aux = new byte[4];

            aux[0] = 128;
            aux[1] = 131;
            aux[2] = 150;
            aux[3] = 0;
            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        //-----movements---------
        public void move(int speed, int angle)
        {
            if (speed == 0)
                speed = irobot_velocity;

            byte[] aux = new byte[6];

            aux[0] = 131;  //Controll
            aux[1] = 137;  //Drive

            byte speedHi = (byte)(((Velocity)speed).ToInt >> 8);
            byte speedLo = (byte)(((Velocity)speed).ToInt & 255);

            aux[2] = speedHi;  //velocity, a positive number = foreward
            aux[3] = speedLo;  //

            byte byAngleHi = (byte)(((Radius)angle).ToInt >> 8);
            byte byAngleLo = (byte)(((Radius)angle).ToInt & 255);

            aux[4] = byAngleHi;   //0
            aux[5] = byAngleLo;    //90

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        public void moveRight(int speed)
        {
            if (speed == 0)
                speed = irobot_velocity;

            byte[] aux = new byte[6];

            aux[0] = 131;  //Controll
            aux[1] = 137;  //Drive

            byte speedHi = (byte)(speed >> 8);
            byte speedLo = (byte)(speed & 255);

            aux[2] = speedHi;  //velocity, a positive number = foreward
            aux[3] = speedLo;  //
            aux[4] = 255;  //radius, 2000 mm, 
            aux[5] = 255;  //

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        public void moveLeft(int speed)
        {
            if (speed == 0)
                speed = irobot_velocity;

            byte[] aux = new byte[7];

            aux[0] = 128;
            aux[1] = 131;  //Controll
            aux[2] = 137;  //Drive

            byte speedHi = (byte)(speed >> 8);
            byte speedLo = (byte)(speed & 255);

            aux[3] = speedHi;  //velocity, a positive number = foreward
            aux[4] = speedLo;  //
            aux[5] = 0;    //radius, 2000 mm,
            aux[6] = 0;  //

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        public void moveStop()
        {
            byte[] aux = new byte[6];

            aux[0] = 131;
            aux[1] = 137;
            aux[2] = 0;
            aux[3] = 0;
            aux[4] = 0;
            aux[5] = 0;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        //hack - use time to relate to degrees to a certain velocity...need improvements!!;)
        public void moveRotate(double angle)
        {
            int time_delay = 2000;  //number of ms to turn 90 degrees
            int aux = (int)Math.Ceiling(Math.Abs(angle)) * time_delay / 90;

            if (Math.Sign(angle) > 0)
                moveRight(0);
            else
            {
                moveLeft(0);
            }
            System.Threading.Thread.Sleep(aux);
            moveStop();
        }

        public void moveRotate2(double angle)
        {
            if (Math.Sign(angle) > 0)
                moveRight(0);
            else
            {
                moveLeft(0);
            }
            WaitAngle((Radius)(-angle));
            moveStop();
        }

        public void WaitAngle(Radius angle)
        {
            byte[] aux = new byte[4];

            aux[0] = 131;
            aux[1] = 157;

            int num = angle.ToInt;
            byte byAngleHi = (byte)(num >> 8);
            byte byAngleLo = (byte)(num & 255);

            aux[2] = byAngleHi;   //0
            aux[3] = byAngleLo;    //90

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        private void WaitDistance(int distance) //must be in mm
        {
            byte[] aux = new byte[4];

            aux[0] = 131;
            aux[1] = 156;

            byte bySpeedHi = (byte)(distance >> 8);
            byte bySpeedLo = (byte)(distance & 255);

            aux[2] = bySpeedHi;   //1
            aux[3] = bySpeedLo;    //144

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }
        private void waitEvent(int event_number)
        {
            byte[] aux = new byte[3];

            aux[0] = 131;
            aux[1] = 158;
            aux[2] = (byte)event_number;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        //----built-in programs--
        private void progStart()
        {
            byte[] aux = new byte[1];

            aux[0] = 128;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }

        public void progSpot()
        {
            byte[] aux = new byte[3];

            aux[0] = 128;
            aux[1] = 131;
            aux[2] = 134;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }
        public void progClean()
        {
            byte[] aux = new byte[2];

            progStart();

            aux[0] = 131;
            aux[1] = 135;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }
        //usefull for auto-docking
        public void progDock()
        {
            byte[] aux = new byte[2];

            progStart();

            aux[0] = 131;
            aux[1] = 143;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }
        public void progHome()
        {
            byte[] aux = new byte[2];

            progStart();

            aux[0] = 131;
            aux[1] = 136;

            if (serialPort1.IsOpen)
                serialPort1.Write(aux, 0, aux.Length);
        }
        //-----------------------

        public int GetSensorData(int sensorCode)
        {
            return iRobotSensors.getData(sensorCode);
        }

        //automatic behaviours
        private void Bumper(int sensor, int value)
        {
            try
            {
                switch (sensor)
                {
                    case (int)Sensors.Sensor.BumpsWheelDrop:
                        switch (value)
                        {
                            case 3: //front bump
                                //Clock.Change(-1, delay);
                                move(-100, 0);
                                WaitDistance(-20);
                                moveLeft(0);
                                WaitAngle(40);
                                moveStop();
                                //Clock.Change(0, delay);
                                break;
                            case 1: //right bump
                                //Clock.Change(-1, delay);
                                move(-100, 0);
                                WaitDistance(-50);
                                moveLeft(0);
                                WaitAngle(20);
                                moveStop();
                                //Clock.Change(0, delay);
                                break;
                            case 2: //left bump
                                //Clock.Change(-1, delay);
                                move(-100, 0);
                                WaitDistance(-50);
                                moveRight(0);
                                WaitAngle(-20);
                                moveStop();
                                //Clock.Change(0, delay);
                                break;
                            default:
                                break;
                        }
                        break;
                    case (int)Sensors.Sensor.WallSignal:
                        if (value > 70)
                        {
                            //Clock.Change(-1, delay);
                            moveLeft(100);
                            waitEvent(247);
                            moveStop();
                            //Clock.Change(0, delay);
                        }
                        break;
                    default:
                        break;
                }
            }
            catch { return; }
        }
    }
}
