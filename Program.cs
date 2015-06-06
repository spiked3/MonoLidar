using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using uPLibrary.Networking.M2Mqtt;
using spiked3;

namespace ConsoleApplication1
{
    class Program
    {
        public static MqttClient C;
        static void Main(string[] args)
        {
            Console.WriteLine("spiked3.com RpLidar to MQTT Bridge");

            C = new MqttClient("127.0.0.1");
            C.Connect("LBrdg");
            Console.WriteLine("MQTT Connected");

            var l = new RpLidarSerial();
            //l.Open("/dev/ttyUSB0");
            if (l.Open(args[0]))
            {
                Console.WriteLine("Lidar Opened, starting .....");
                Console.WriteLine("Ctl-C to quit");
                l.Start();
            }

            //C.Disconnect();
        }
    }
}
