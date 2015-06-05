using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Timers;
using uPLibrary.Networking.M2Mqtt;

namespace spiked3
{
    public class RpLidarSerial
    {
        readonly string[] HealtStatusStrings = { "Good", "Poor", "Critical", "Unknown" };

        SerialPort Lidar;
        bool lidarTimedOut;
        byte[] nodeBuf = new byte[5];
        int recvPos;

        ScanPoint[] ScanData = new ScanPoint[360];         
        
        bool StartOfNewScan = true;        

        public RpLidarSerial()
        {            
        }

        public void Dispose()
        {
            if (Lidar != null)
                Lidar.Close();
        }

        public bool Open(string comPort)
        {
            Console.WriteLine("RpLidarDriver::Open","2");
            Lidar = new SerialPort(comPort, 115200, Parity.None, 8, StopBits.One);
            try
            {
                Lidar.Open();
            }
            catch (Exception ex)
            {
                throw ex;   // bubble up
            }

            // retry until valid device info
            int tries = 0;
            while (++tries < 5)
            {
                LidarDevInfoResponse di;
                if (GetDeviceInfo(out di))
                {
                    if (di.Model == 0 && di.hardware == 0)
                    {
                        //Console.WriteLine($"Lidar Model({di.Model}, {di.hardware}), Firmware({di.FirmwareMajor}, {di.FirmwareMinor})");
                        return true;
                    }
                }
                else
                {
                    Console.WriteLine("Unable to get device info from RP LIDAR, device reset", "warn");
                    Reset();
                }
            }
            Console.WriteLine("Open Lidar failed 5 (re)tries", "error");
            return false;
        }

        public void LidarFlush()
        {
            while (Lidar.BytesToRead > 0)
                Lidar.ReadByte();
        }

        void LidarRequest(LidarCommand cmd, byte[] payload = null)
        {
            byte chksum = 0x00;
            List<byte> buf = new List<byte>();
            buf.Add(0xA5); //start
            buf.Add((byte)cmd);
            if (payload != null && payload.Length > 0)
            {
                buf.Add((byte)payload.Length);
                buf.AddRange(payload);
                buf.ForEach(x => chksum ^= x);
                buf.Add(chksum);
            }
            byte[] b = buf.ToArray<byte>();
            Lidar.Write(b, 0, b.Length);
        }

        public bool Start()
        {
            if (Lidar != null && Lidar.IsOpen)
            {
                byte[] r;
                LidarScanResponse sr;
                LidarFlush();
                LidarRequest(LidarCommand.Scan);
                if (GetLidarResponseWTimeout(out r, 7, 500))
                {
                    sr = r.ByteArrayToStructure<LidarScanResponse>(0);
                    Lidar.DataReceived += LidarScanDataReceived; // we expect responses until we tell it to stop
                }
            }
            return (Lidar != null && Lidar.IsOpen);
        }

        void LidarScanDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            //System.Diagnostics.Console.WriteLine(string.Format("LidarScanDataReceived bytes to read  {0}", Lidar.BytesToRead));
            while (Lidar.IsOpen && Lidar.BytesToRead > 0)
            {
                byte currentByte = (byte)Lidar.ReadByte();

                switch (recvPos)
                {
                    case 0:
                        int tmp = currentByte & 0x03;
                        if (tmp != 0x01 && tmp != 0x02)
                            continue;
                        break;
                    case 1:
                        if ((currentByte & 0x01) != 0x01)
                        {
                            recvPos = 0;
                            continue;
                        }
                        break;
                }

                nodeBuf[recvPos++] = currentByte;

                if (recvPos == 5)
                {
                    if (StartOfNewScan)
                    {
                        Array.Clear(ScanData, 0, 360);
                        StartOfNewScan = false;
                    }
                    // converted values
                    LidarScanData node = nodeBuf.ByteArrayToStructure<LidarScanData>(0);
                    float distance = node.Distance / 4f;

                    // I couldn't tell much difference rounded v not
                    //int angle = (int)((node.Angle >> 1) / 64.0);   
                    int angle = (int)Math.Round(((node.Angle >> 1) / 64.0), MidpointRounding.AwayFromZero);

                    int quality = node.Quality >> 2;
                    bool startBit = (node.Quality & 0x01) == 0x01;

                    //System.Diagnostics.Console.WriteLine(string.Format("s({0},{1}) c({2}) q({3}) a({4}) d({5})", s, s1, c, q, a, d));
                    if (distance > 0 && angle < 360)
                    {
                        // ++++
                        ScanData[angle] = new ScanPoint { Angle = angle, Quality = quality, Distance = distance };
                    }

                    recvPos = 0;
                    if (startBit)
                    {
                        StartOfNewScan = true;
                        NewScanSet();       // fire event
                    }
                }
            }
        }

        public bool GetHealth(out LidarHealthResponse hr)
        {
            hr = new LidarHealthResponse();
            if (Lidar != null && Lidar.IsOpen)
            {
                LidarFlush();
                LidarRequest(LidarCommand.GetHealth);
                byte[] r;
                if (GetLidarResponseWTimeout(out r, 7 + 3, 500))
                {
                    hr = r.ByteArrayToStructure<LidarHealthResponse>(0);
                    Debug.Assert(hr.Status <= 3);
                    Console.WriteLine(string.Format("Lidar Health {0}", HealtStatusStrings[hr.Status]),"2");
                    return true;
                }
            }
            return false;
        }

        public bool GetDeviceInfo(out LidarDevInfoResponse di)
        {
            di = new LidarDevInfoResponse();
            if (Lidar != null && Lidar.IsOpen)
            {
                di = new LidarDevInfoResponse();
                LidarFlush();
                LidarRequest(LidarCommand.GetInfo);
                byte[] r;
                if (GetLidarResponseWTimeout(out r, 7 + 20, 500))
                {
                    di = r.ByteArrayToStructure<LidarDevInfoResponse>(0);
                    //Console.WriteLine($"Model({di.Model}) Firmware({di.FirmwareMajor},{di.FirmwareMinor}) Hardware({di.hardware}) serial({BitConverter.ToString(di.SerialNum)})");                        
                    return true;
                }
            }
            return false;
        }

        public void Reset()
        {
            if (Lidar != null && Lidar.IsOpen)
            {
                Lidar.DataReceived -= LidarScanDataReceived;
                LidarRequest(LidarCommand.Reset);
                Lidar.Close();
                Lidar.Dispose();
                System.Threading.Thread.Sleep(500);
            }
        }

        public void Stop()
        {
            if (Lidar != null && Lidar.IsOpen)
            {
                LidarRequest(LidarCommand.Stop);
                System.Threading.Thread.Sleep(100);
                LidarFlush();
                Lidar.Close();
                Lidar.Dispose();
            }
        }

        bool GetLidarResponseWTimeout(out byte[] outBuf, int expectedLength, int timeout)
        {
            int idx = 0;
            outBuf = new byte[expectedLength];
            using (Timer timeoutTimer = new Timer(timeout))
            {
                timeoutTimer.Elapsed += LidarResponseTimeoutElapsed;
                lidarTimedOut = false;
                timeoutTimer.Start();
                while (!lidarTimedOut)
                {
                    if (Lidar.BytesToRead > 0)
                    {
                        outBuf[idx++] = (byte)Lidar.ReadByte();
                        if (idx >= expectedLength)
                            return true; // timer should be auto disposed??
                    }
                    System.Threading.Thread.Sleep(2);
                }
                return false;
            }
        }

        void LidarResponseTimeoutElapsed(object sender, ElapsedEventArgs e)
        {
            ((Timer)sender).Stop();
            lidarTimedOut = true;
        }

        int activityIdx = 0;
        char[] activityChars = { '\\', '|', '/', '-'};
        void NewScanSet()
        {
            Console.Write("\r" + activityChars[++activityIdx%activityChars.Length]);
            ConsoleApplication1.Program.C.Publish("RpLidar", ToByteArray<ScanPoint>(ScanData));
        }

        // +++ may consider faster unsafe method if needed
        static byte[] ToByteArray<T>(T[] source) where T : struct
        {
            GCHandle handle = GCHandle.Alloc(source, GCHandleType.Pinned);
            try
            {
                IntPtr pointer = handle.AddrOfPinnedObject();
                byte[] destination = new byte[source.Length * Marshal.SizeOf(typeof(T))];
                Marshal.Copy(pointer, destination, 0, destination.Length);
                return destination;
            }
            finally
            {
                if (handle.IsAllocated)
                    handle.Free();
            }
        }

        static T[] FromByteArray<T>(byte[] source) where T : struct
        {
            T[] destination = new T[source.Length / Marshal.SizeOf(typeof(T))];
            GCHandle handle = GCHandle.Alloc(destination, GCHandleType.Pinned);
            try
            {
                IntPtr pointer = handle.AddrOfPinnedObject();
                Marshal.Copy(source, 0, pointer, source.Length);
                return destination;
            }
            finally
            {
                if (handle.IsAllocated)
                    handle.Free();
            }
        }

    }

    public enum LidarCommand : byte
    {
        Stop = 0x25,
        Reset = 0x40,
        Scan = 0x20,
        ForceScan = 0x21,
        GetInfo = 0x50,
        GetHealth = 0x52
    };

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct LidarScanData
    {
        public byte Quality;
        public UInt16 Angle;
        public UInt16 Distance;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct LidarScanResponse
    {
        public byte Header1;
        public byte Header2;
        public Int32 Size;
        public byte DataType;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct LidarDevInfoResponse
    {
        public byte Header1;
        public byte Header2;
        public Int32 Size; // first 30 bits, last 2 bits are send mode
        public byte DataType;
        public byte Model; // first 6 bits, last 2 bits are scan indicators
        public byte FirmwareMinor;
        public byte FirmwareMajor;
        public byte hardware;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public byte[] SerialNum;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct LidarHealthResponse
    {
        public byte Header1;
        public byte Header2;
        public Int32 Size; // first 30 bits, last 2 bits are send mode
        public byte DataType;
        public byte Status;
        public Int16 ErrorCode;
    }

    public static class Extensions
    {
        public static T ByteArrayToStructure<T>(this byte[] bytes, int offset) where T : struct
        {
            GCHandle h = GCHandle.Alloc(bytes, GCHandleType.Pinned);
            T s = (T)Marshal.PtrToStructure(h.AddrOfPinnedObject() + offset, typeof(T));
            h.Free();
            return s;
        }
    }
    
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ScanPoint
    {
        public float Angle;
        public float Distance;
        public int Quality;
    }

}
