using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GyroLogging
{
    class SerialPortProtocolParser
    {
        #region 属性
        private List<byte> byteArray = new List<byte>();

        public List<byte> ByteArray
        {
            get { return byteArray; }
        }
        //private byte[] ch2;

        private byte header1;

        public byte Header1
        {
            get { return header1; }
        }
        private byte header2;

        public byte Header2
        {
            get { return header2; }
        }

        private byte length;

        public byte Length
        {
            get { return length; }
        }

        private byte command;

        public byte Command
        {
            get { return command; }
        }

        private byte[] content = new byte[0];

        public byte[] Content
        {
            get { return content; }
        }

        private byte checksum;

        public byte Checksum
        {
            get { return checksum; }
        }
 #endregion

        public SerialPortProtocolParser(byte header1, byte header2)
        {
            this.header1 = header1;
            this.header2 = header2;
        }

        public SerialPortProtocolParser(char header1, char header2)
        {
            this.header1 = (byte)header1;
            this.header2 = (byte)header2;
        }

        public byte[] GenerateFrame(byte command, byte[] content)
        {
            byte[] frame = new byte[3 + 1 + content.Length + 1];
            frame[0] = header1;
            frame[1] = header2;
            frame[2] = (byte)(frame.Length - 3);
            frame[3] = command;
            content.CopyTo(frame, 4);

            byte sum = command;
            for (int i = 0; i < content.Length; i++)
            {
                sum += content[i];
            }
            frame[frame.Length - 1] = sum;
            return frame;
        }

        public byte[] GenerateFrame(byte command)
        {
            byte[] frame = new byte[3 + 1 + 1];
            frame[0] = header1;
            frame[1] = header2;
            frame[2] = (byte)(frame.Length - 3);
            frame[3] = command;

            byte sum = command;
            frame[frame.Length - 1] = sum;
            return frame;
        }

        public byte[] GenerateFrame(byte[] content)
        {
            byte[] frame = new byte[3 + content.Length + 1];
            frame[0] = header1;
            frame[1] = header2;
            frame[2] = (byte)(frame.Length - 3);
            content.CopyTo(frame, 3);
            byte sum = 0x00;
            for (int i = 0; i < content.Length; i++)
            {
                sum += content[i];
            }
            frame[frame.Length - 1] = sum;
            return frame;
        }

        private bool CheckSum()
        {
            byte sum = command;
            for (int i = 0; i < content.Length; i++ )
            {
                sum += content[i];
            }
            return (sum == checksum) ? true : false;
        }

        public bool isFrameExisted()
        {
            int n = byteArray.Count;
            if (n < 2)
            {
                return false;
            } 
            else
            {
                // 遍历所有的数据寻找头
                for (int i = 0; i < n - 2; i++ )
                {
                    if (byteArray[i] == header1 && byteArray[i + 1] == header2)
                    {
                        length = byteArray[i + 2];
                        n -= (i + 3);
                        if (n >= length)
                        {
                            command = byteArray[i + 3];
                            if (length >= 1)
                            {
                                content = new byte[length - 2];
                                byteArray.CopyTo(i + 4, content, 0, length - 2);
                                checksum = byteArray[i + 2 + length];
                                byteArray.RemoveRange(0, i + 3 + length);
                                return CheckSum();
                            }
                            else
                            {
                                byteArray.RemoveRange(0, i + 2);
                                return false;
                            }
                        }
                        else
                        {
                            byteArray.RemoveRange(0, i);
                            return false;
                        }
                    }
                }
                byteArray.RemoveRange(0, n - 2);
                return false;
            }
        }

        public void Reset()
        {
            byteArray.Clear();
            content = new byte[0];
            length = 0;
            command = 0;
            checksum = 0;
        }

        public void ClearBuffer()
        {
            byteArray.Clear();
        }

        public void Append(byte b)
        {
            byteArray.Add(b);
        }

        public void Append(byte[] array)
        {
            foreach (byte b in array)
            {
                byteArray.Add(b);
            }
        }

        public void Append(List<byte> array)
        {
            byteArray.AddRange(array);
        }
    }
}
