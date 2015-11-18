using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;

namespace ControlPannel
{
    public class RobotStatus : DependencyObject
    {
        
        const short TF1_Zero = 0;
        const short TF2_Zero = 0;
        const short TF3_Zero = 0;
        const short TF4_Zero = 0;
        const short TF5_Zero = 0;
        const short TF6_Zero = 0;
        const short TF7_Zero = 0;
        const short TF8_Zero = 0;
        const double TensionFactor = 0.0003052;
        
        static readonly DependencyProperty TF1Property = DependencyProperty.Register("TF1", typeof(short), typeof(RobotStatus));
        static readonly DependencyProperty TF2Property = DependencyProperty.Register("TF2", typeof(short), typeof(RobotStatus));
        static readonly DependencyProperty TF3Property = DependencyProperty.Register("TF3", typeof(short), typeof(RobotStatus));
        static readonly DependencyProperty TF4Property = DependencyProperty.Register("TF4", typeof(short), typeof(RobotStatus));
        static readonly DependencyProperty TF5Property = DependencyProperty.Register("TF5", typeof(short), typeof(RobotStatus));
        static readonly DependencyProperty TF6Property = DependencyProperty.Register("TF6", typeof(short), typeof(RobotStatus));
        static readonly DependencyProperty TF7Property = DependencyProperty.Register("TF7", typeof(short), typeof(RobotStatus));
        static readonly DependencyProperty TF8Property = DependencyProperty.Register("TF8", typeof(short), typeof(RobotStatus));

        
        //力传感器AD值，setvalue的时候会把标定值也计算好
        public short TF1 { get { return (Int16)GetValue(TF1Property); } set { SetValue(TF1Property, value); SetValue(TF1CalProperty, (value - TF1_Zero) * TensionFactor); } }
        public short TF2 { get { return (Int16)GetValue(TF2Property); } set { SetValue(TF2Property, value); SetValue(TF2CalProperty, (value - TF2_Zero) * TensionFactor); } }
        public short TF3 { get { return (Int16)GetValue(TF3Property); } set { SetValue(TF3Property, value); SetValue(TF3CalProperty, (value - TF3_Zero) * TensionFactor); } }
        public short TF4 { get { return (Int16)GetValue(TF4Property); } set { SetValue(TF4Property, value); SetValue(TF4CalProperty, (value - TF4_Zero) * TensionFactor); } }
        public short TF5 { get { return (Int16)GetValue(TF5Property); } set { SetValue(TF5Property, value); SetValue(TF5CalProperty, (value - TF5_Zero) * TensionFactor); } }
        public short TF6 { get { return (Int16)GetValue(TF6Property); } set { SetValue(TF6Property, value); SetValue(TF6CalProperty, (value - TF6_Zero) * TensionFactor); } }
        public short TF7 { get { return (Int16)GetValue(TF7Property); } set { SetValue(TF7Property, value); SetValue(TF7CalProperty, (value - TF7_Zero) * TensionFactor); } }
        public short TF8 { get { return (Int16)GetValue(TF8Property); } set { SetValue(TF8Property, value); SetValue(TF8CalProperty, (value - TF8_Zero) * TensionFactor); } }

        static readonly DependencyProperty TF1CalProperty = DependencyProperty.Register("TF1Cal", typeof(double), typeof(RobotStatus));
        static readonly DependencyProperty TF2CalProperty = DependencyProperty.Register("TF2Cal", typeof(double), typeof(RobotStatus));
        static readonly DependencyProperty TF3CalProperty = DependencyProperty.Register("TF3Cal", typeof(double), typeof(RobotStatus));
        static readonly DependencyProperty TF4CalProperty = DependencyProperty.Register("TF4Cal", typeof(double), typeof(RobotStatus));
        static readonly DependencyProperty TF5CalProperty = DependencyProperty.Register("TF5Cal", typeof(double), typeof(RobotStatus));
        static readonly DependencyProperty TF6CalProperty = DependencyProperty.Register("TF6Cal", typeof(double), typeof(RobotStatus));
        static readonly DependencyProperty TF7CalProperty = DependencyProperty.Register("TF7Cal", typeof(double), typeof(RobotStatus));
        static readonly DependencyProperty TF8CalProperty = DependencyProperty.Register("TF8Cal", typeof(double), typeof(RobotStatus));

        //力传感器标定值
        public double TF1Cal { get { return (Int16)GetValue(TF1CalProperty); } }
        public double TF2Cal { get { return (Int16)GetValue(TF2CalProperty); } }
        public double TF3Cal { get { return (Int16)GetValue(TF3CalProperty); } }
        public double TF4Cal { get { return (Int16)GetValue(TF4CalProperty); } }
        public double TF5Cal { get { return (Int16)GetValue(TF5CalProperty); } }
        public double TF6Cal { get { return (Int16)GetValue(TF6CalProperty); } }
        public double TF7Cal { get { return (Int16)GetValue(TF7CalProperty); } }
        public double TF8Cal { get { return (Int16)GetValue(TF8CalProperty); } }



        static readonly DependencyProperty EulerX1Property = DependencyProperty.Register("EulerX1", typeof(Single), typeof(RobotStatus));
        static readonly DependencyProperty EulerY1Property = DependencyProperty.Register("EulerY1", typeof(Single), typeof(RobotStatus));
        static readonly DependencyProperty EulerZ1Property = DependencyProperty.Register("EulerZ1", typeof(Single), typeof(RobotStatus));
        static readonly DependencyProperty EulerX2Property = DependencyProperty.Register("EulerX2", typeof(Single), typeof(RobotStatus));
        static readonly DependencyProperty EulerY2Property = DependencyProperty.Register("EulerY2", typeof(Single), typeof(RobotStatus));
        static readonly DependencyProperty EulerZ2Property = DependencyProperty.Register("EulerZ2", typeof(Single), typeof(RobotStatus));
        static readonly DependencyProperty EulerX3Property = DependencyProperty.Register("EulerX3", typeof(Single), typeof(RobotStatus));
        static readonly DependencyProperty EulerY3Property = DependencyProperty.Register("EulerY3", typeof(Single), typeof(RobotStatus));
        static readonly DependencyProperty EulerZ3Property = DependencyProperty.Register("EulerZ3", typeof(Single), typeof(RobotStatus));
        
        public Single EulerX1{get { return (Single)GetValue(EulerX1Property); }set { SetValue(EulerX1Property, value); }}
        public Single EulerY1{get { return (Single)GetValue(EulerY1Property); }set { SetValue(EulerY1Property, value); }}
        public Single EulerZ1{get { return (Single)GetValue(EulerZ1Property); }set { SetValue(EulerZ1Property, value); }}
        public Single EulerX2{get { return (Single)GetValue(EulerX2Property); }set { SetValue(EulerX2Property, value); }}
        public Single EulerY2{get { return (Single)GetValue(EulerY2Property); }set { SetValue(EulerY2Property, value); }}
        public Single EulerZ2{get { return (Single)GetValue(EulerZ2Property); }set { SetValue(EulerZ2Property, value); }}
        public Single EulerX3{get { return (Single)GetValue(EulerX3Property); }set { SetValue(EulerX3Property, value); }}
        public Single EulerY3{get { return (Single)GetValue(EulerY3Property); }set { SetValue(EulerY3Property, value); }}
        public Single EulerZ3{get { return (Single)GetValue(EulerZ3Property); }set { SetValue(EulerZ3Property, value); }}





        public bool ReadJointAngle(byte[] array)
        {
            if (array.Length != 52)//modified,如果数据长度不等于52就不读了
            {
                return false;
            }
            else
            {
                try
                {
                    TF1 = BitConverter.ToInt16(array, 0);//下位机传过来的是16位AD值，转换成Int16
                    TF2 = BitConverter.ToInt16(array, 2);
                    TF3 = BitConverter.ToInt16(array, 4);
                    TF4 = BitConverter.ToInt16(array, 6);
                    TF5 = BitConverter.ToInt16(array, 8);
                    TF6 = BitConverter.ToInt16(array, 10);
                    TF7 = BitConverter.ToInt16(array, 12);
                    TF8 = BitConverter.ToInt16(array, 14);
                    EulerX1 = BitConverter.ToSingle(array, 16);//下位机传过来的是32位IMU浮点数，转换成Single
                    EulerY1 = BitConverter.ToSingle(array, 20);
                    EulerZ1 = BitConverter.ToSingle(array, 24); 
                    EulerX2 = BitConverter.ToSingle(array, 28);
                    EulerY2 = BitConverter.ToSingle(array, 32);
                    EulerZ2 = BitConverter.ToSingle(array, 36);
                    EulerX3 = BitConverter.ToSingle(array, 40);
                    EulerY3 = BitConverter.ToSingle(array, 44);
                    EulerZ3 = BitConverter.ToSingle(array, 48);

                    return true;
                }
                catch (Exception)
                {
                    return false;
                }
            }
        }
    }
}
