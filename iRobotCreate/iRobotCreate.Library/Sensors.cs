using System;
using System.Collections.Generic;

namespace iRobotCreate.Library
{
    class Sensors
    {
        //define sensor packages
        public enum Sensor : int
        {
            BumpsWheelDrop = 7,
            BatteryTemperature = 24,
            BatteryCharge = 25,
            BatteryCapacity = 26,
            ChargingState = 21,
            WallSignal = 27,
            Angle = 20
        };

        Dictionary<int, int> sensorPool = new Dictionary<int, int>();
        int query_counter = 0;

        public Sensors()
        { }

        public Sensors(byte[] comByte)
        {
            setData(comByte);
        }
        ~Sensors() { sensorPool.Clear(); }

        public void setData(byte[] comByte)
        {
            try
            {
                if (comByte.Length == 0 || comByte[0] != 19)
                    return;

                //create a new StringBuilder object - best to manage data
                List<byte> builder = new List<byte>(comByte);

                //just pass the packets values
                if (comByte.Length > 1)
                    if (comByte.Length - 3 == builder[1])
                        ProcessData(builder.GetRange(2, builder[1]));
            }
            catch { return; }
        }

        //if is better to query rather to stream
        public void setDataQuery(byte[] comByte, byte[] sensor_list)
        {
            try
            {
                if (comByte.Length == 0)
                    return;

                //create a new StringBuilder object - best to manage data
                List<byte> builder = new List<byte>(comByte);

                //just pass the packets values
                if (comByte.Length > 1)
                {
                    query_counter = 0;
                    ProcessDataQuery(builder, sensor_list);
                }
            }
            catch { return; }
        }

        //separate by message Packet - just for query
        private void ProcessDataQuery(List<byte> aux, byte[] sensor_list)
        {
            try
            {
                //while exist packets to process...
                while (aux.Count > 0)
                {
                    if (sensor_list.Length < query_counter)
                        return;

                    switch ((int)sensor_list[query_counter])
                    {
                        case (int)Sensor.BumpsWheelDrop:
                            Set_BumpWheeldrop(aux.GetRange(0, 1));
                            aux.RemoveRange(0, 1);  //just remove current packet
                            break;
                        case (int)Sensor.BatteryCharge:
                            Set_BatteryCharge(aux.GetRange(0, 2));
                            aux.RemoveRange(0, 2);  //just remove current packet
                            break;
                        case (int)Sensor.BatteryCapacity:
                            Set_BatteryCapacity(aux.GetRange(0, 2));
                            aux.RemoveRange(0, 2);  //just remove current packet
                            break;
                        case (int)Sensor.ChargingState:
                            Set_ChargingState(aux.GetRange(0, 1));
                            aux.RemoveRange(0, 1);  //just remove current packet
                            break;
                        case (int)Sensor.WallSignal:
                            Set_WallSignal(aux.GetRange(0, 2));
                            aux.RemoveRange(0, 2);  //just remove current packet
                            break;
                        case (int)Sensor.Angle:
                            Set_Angle(aux.GetRange(0, 2));
                            aux.RemoveRange(0, 2);  //just remove current packet
                            break;
                        default:
                            return;
                    }
                    query_counter++;
                }
            }
            catch { return; }
        }

        //separate by message Packet
        private void ProcessData(List<byte> aux)
        {
            try
            {
                //while exist packets to process...
                while (aux.Count > 0)
                    switch ((int)aux[0])
                    {
                        case (int)Sensor.BumpsWheelDrop:
                            Set_BumpWheeldrop(aux.GetRange(1, 1));
                            aux.RemoveRange(0, 2);  //just remove current packet
                            break;
                        case (int)Sensor.BatteryCharge:
                            Set_BatteryCharge(aux.GetRange(1, 2));
                            aux.RemoveRange(0, 3);  //just remove current packet
                            break;
                        case (int)Sensor.BatteryCapacity:
                            Set_BatteryCapacity(aux.GetRange(1, 2));
                            aux.RemoveRange(0, 3);  //just remove current packet
                            break;
                        case (int)Sensor.ChargingState:
                            Set_ChargingState(aux.GetRange(1, 1));
                            aux.RemoveRange(0, 2);  //just remove current packet
                            break;
                        case (int)Sensor.WallSignal:
                            Set_WallSignal(aux.GetRange(1, 2));
                            aux.RemoveRange(0, 3);  //just remove current packet
                            break;
                        case (int)Sensor.Angle:
                            Set_Angle(aux.GetRange(1, 2));
                            aux.RemoveRange(0, 3);  //just remove current packet
                            break;
                        default:
                            return;
                    }
            }
            catch { return; }
        }

        //------fill data for each packet type---------------
        private void Set_BumpWheeldrop(List<byte> aux)
        {
            sensorPool[(int)Sensor.BumpsWheelDrop] = (int)ConvertData(aux, 31);
        }
        private void Set_BatteryCharge(List<byte> aux)
        {
            sensorPool[(int)Sensor.BatteryCharge] = (int)ConvertData(aux, 65535);
        }
        private void Set_BatteryCapacity(List<byte> aux)
        {
            sensorPool[(int)Sensor.BatteryCapacity] = (int)ConvertData(aux, 65535);
        }
        private void Set_ChargingState(List<byte> aux)
        {
            sensorPool[(int)Sensor.ChargingState] = (int)ConvertData(aux, 5);
        }
        private void Set_WallSignal(List<byte> aux)
        {
            sensorPool[(int)Sensor.WallSignal] = (int)ConvertData(aux, 4095);
        }
        private void Set_Angle(List<byte> aux)
        {
            sensorPool[(int)Sensor.Angle] = (int)ConvertData(aux, 32767);
        }
        //---------------------------------------------------

        private int ConvertData(List<byte> aux, int max_range)
        {
            int rawValue = 0;

            try
            {
                if (aux.Count < 1)
                    return -1;

                //We have to look at Max & Signed to know what type to cast this to.
                switch (max_range)
                {
                    case 255:
                    case 65535:
                        //rawValue = Convert.ToUInt16(currentSensor.Value);
                        rawValue = (UInt16)((aux[0] << 8) | aux[1]);
                        break;
                    case 4095:
                        rawValue = (aux[0] * 256) + aux[1];
                        break;
                    case 1:
                    case 3:
                    case 5:
                    case 31:
                    case 43:
                        rawValue = aux[0]; //No conversion needed here.
                        break;
                    case 500:
                    case 32767:
                        //Angle & Distance, Req Velocity..
                        rawValue = (short)((aux[0] << 8) | aux[1]);
                        break;
                    case 127:
                        //rawValue = BitConverter.ToInt16(dataBytes, 1);
                        break;
                    case 1023:
                        break;
                }
                return rawValue;
            }
            catch { return -1; }
        }

        //return data
        public int getData(int aux)
        {
            try
            {
                if (sensorPool.Count > 0 || sensorPool.ContainsKey(aux))
                {
                    int x = sensorPool[aux];
                    sensorPool[aux] = -1;
                    return x;
                }
                else
                    return -1;
            }
            catch { return -1; }
        }
    }
}
