// 绳索收紧为负，释放为正

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.IO.Ports;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;
using Microsoft.Research.DynamicDataDisplay;
using Microsoft.Research.DynamicDataDisplay.DataSources;
using System.Threading;

namespace ControlPannel
{
	/// <summary>
	/// MainWindow.xaml 的交互逻辑
	/// </summary>
    /// 

    public partial class MainWindow : Window, INotifyPropertyChanged    
	{
        #region 私有字段

        private SerialPort sp = new SerialPort();
        private GyroLogging.SerialPortProtocolParser sp3 = new GyroLogging.SerialPortProtocolParser(0xFF, 0xFA);
        private string powerOnText = "打开串口";
        private string statusBarText = "准备";
        private DispatcherTimer timer = new DispatcherTimer();

        private ObservableDataSource<Point> jointAngle = new ObservableDataSource<Point>();
        private ObservableDataSource<Point> EulerX1 = new ObservableDataSource<Point>();//draw curve
        private ObservableDataSource<Point> EulerY1 = new ObservableDataSource<Point>();
        private ObservableDataSource<Point> EulerZ1 = new ObservableDataSource<Point>();
        private ObservableDataSource<Point> EulerX2 = new ObservableDataSource<Point>();//draw curve
        private ObservableDataSource<Point> EulerY2 = new ObservableDataSource<Point>();
        private ObservableDataSource<Point> EulerZ2 = new ObservableDataSource<Point>();
        private ObservableDataSource<Point> EulerX3 = new ObservableDataSource<Point>();//draw curve
        private ObservableDataSource<Point> EulerY3 = new ObservableDataSource<Point>();
        private ObservableDataSource<Point> EulerZ3 = new ObservableDataSource<Point>();

        private LineGraph graphEulerX1 = new LineGraph();
        private LineGraph graphEulerY1 = new LineGraph();
        private LineGraph graphEulerZ1 = new LineGraph();
        private LineGraph graphEulerX2 = new LineGraph();
        private LineGraph graphEulerY2 = new LineGraph();
        private LineGraph graphEulerZ2 = new LineGraph();
        private LineGraph graphEulerX3 = new LineGraph();
        private LineGraph graphEulerY3 = new LineGraph();
        private LineGraph graphEulerZ3 = new LineGraph();
        

        private double time = 0.0;
        private RobotStatus robotStatus;
        #endregion

        #region 依赖属性        
        #endregion

        public string PowerOnText
        {
            get { return powerOnText; }
            set { powerOnText = value; NotifyPropertyChanged("PowerOnText"); }
        }

        public string StatusBarText
        {
            get { return statusBarText; }
            set { statusBarText = value; NotifyPropertyChanged("StatusBarText"); }
        }
		
		public MainWindow()
		{
			this.InitializeComponent();
		}
        
        // 用户按下电机控制按钮,发送速度指令
        private void StackPanel_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
		{
			// 在此处添加事件处理程序实现
            Button btn = e.Source as Button;
            if (btn == null)
            {
                return;
            }
            List<UInt16> address = new List<UInt16>();
            List<UInt16> value = new List<UInt16>();
            byte[] xmtData;
            switch (btn.Name)
            {
                #region 绳索收紧
                case "btnPullS1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_1));
                    value.Add((UInt16)(-sdrS1.Value));
                    break;
                case "btnPullS2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_2));
                    value.Add((UInt16)(-sdrS2.Value));
                    break;
                case "btnPullS3":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_3));
                    value.Add((UInt16)(-sdrS3.Value));
                    break;
                case "btnPullS4":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_4));
                    value.Add((UInt16)(-sdrS4.Value));
                    break;
                case "btnPullW1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_7));
                    value.Add((UInt16)(-sdrW1.Value));
                    break;
                case "btnPullW2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_8));
                    value.Add((UInt16)(-sdrW2.Value));
                    break;
                case "btnPullW3":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_9));
                    value.Add((UInt16)(-sdrW3.Value));
                    break;
                case "btnPullW4":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_A));
                    value.Add((UInt16)(-sdrW4.Value));
                    break;
                case "btnPullE1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_5));
                    value.Add((UInt16)(-sdrE1.Value));
                    break;
                case "btnPullE2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_6));
                    value.Add((UInt16)(-sdrE2.Value));
                    break;
                #endregion

                #region 绳索释放
                case "btnReleaseS1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_1));
                    value.Add((UInt16)(sdrS1.Value));
                    break;
                case "btnReleaseS2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_2));
                    value.Add((UInt16)(sdrS2.Value));
                    break;
                case "btnReleaseS3":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_3));
                    value.Add((UInt16)(sdrS3.Value));
                    break;
                case "btnReleaseS4":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_4));
                    value.Add((UInt16)(sdrS4.Value));
                    break;
                case "btnReleaseW1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_7));
                    value.Add((UInt16)(sdrW1.Value));
                    break;
                case "btnReleaseW2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_8));
                    value.Add((UInt16)(sdrW2.Value));
                    break;
                case "btnReleaseW3":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_9));
                    value.Add((UInt16)(sdrW3.Value));
                    break;
                case "btnReleaseW4":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_A));
                    value.Add((UInt16)(sdrW4.Value));
                    break;
                case "btnReleaseE1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_5));
                    value.Add((UInt16)(sdrE1.Value));
                    break;
                case "btnReleaseE2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_6));
                    value.Add((UInt16)(sdrE2.Value));
                    break;
                #endregion
                
                default:
                    e.Handled = true;
                    return;
            }

            xmtData = sp3.GenerateFrame(GenerateAddressValuePair(address, value));
            if (sp.IsOpen)
            {
                sp.Write(xmtData, 0, xmtData.Length);
            }

            Run r = new Run(BitConverter.ToString(xmtData));
            paraConsole.Inlines.Add(r);
            paraConsole.Inlines.Add(new LineBreak());
            //滚动到当前光标处
            rtbConsole.ScrollToEnd();

            e.Handled = true;
		}

        // 用户释放电机控制按钮,发送停止指令
        private void StackPanel_PreviewMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            // 在此处添加事件处理程序实现
            Button btn = e.Source as Button;
            if (btn == null)
            {
                return;
            }
            List<UInt16> address = new List<UInt16>();
            List<UInt16> value = new List<UInt16>();
            byte[] xmtData;
            switch (btn.Name)
            {
                #region 绳索收紧
                case "btnPullS1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_1));
                    break;

                case "btnPullS2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_2));
                    break;

                case "btnPullS3":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_3));
                    break;

                case "btnPullS4":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_4));
                    break;

                case "btnPullW1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_7));
                    break;

                case "btnPullW2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_8));
                    break;

                case "btnPullW3":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_9));
                    break;

                case "btnPullW4":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_A));
                    break;

                case "btnPullE1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_5));
                    break;

                case "btnPullE2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_6));
                    break;
                #endregion

                #region 绳索释放
                case "btnReleaseS1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_1));
                    break;

                case "btnReleaseS2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_2));
                    break;

                case "btnReleaseS3":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_3));
                    break;

                case "btnReleaseS4":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_4));
                    break;

                case "btnReleaseW1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_7));
                    break;

                case "btnReleaseW2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_8));
                    break;

                case "btnReleaseW3":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_9));
                    break;

                case "btnReleaseW4":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_A));
                    break;

                case "btnReleaseE1":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_5));
                    break;

                case "btnReleaseE2":
                    address.Add(Convert.ToUInt16(DSP_ADDRESS.CMD_MOTOR_VELOCITY_6));
                    break;
                #endregion

                default:
                    e.Handled = true;
                    return;
            }
            value.Add(0);
            xmtData = sp3.GenerateFrame(GenerateAddressValuePair(address, value));
            if (sp.IsOpen)
            {
                sp.Write(xmtData, 0, xmtData.Length);
            }

            Run r = new Run(BitConverter.ToString(xmtData));
            paraConsole.Inlines.Add(r);
            paraConsole.Inlines.Add(new LineBreak());
            //滚动到当前光标处
            rtbConsole.ScrollToEnd();

            e.Handled = true;
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // 设置串口参数
            sp.PortName = "COM9";
            sp.BaudRate = 115200;
            sp.Parity = Parity.None;
            sp.DataBits = 8;
            sp.StopBits = StopBits.One;
            sp.ReceivedBytesThreshold = 1;
            sp.DataReceived += new SerialDataReceivedEventHandler(sp_DataReceived);
             
            btnPowerOn.DataContext = this;
            tbStatusBar.DataContext = this;

            //my code for plotting
            graphEulerX1 = plotterIMU1.AddLineGraph(EulerX1, Colors.Green, 2, "X");//add new curve
            graphEulerY1 = plotterIMU1.AddLineGraph(EulerY1, Colors.Red, 2, "Y");
            graphEulerZ1 = plotterIMU1.AddLineGraph(EulerZ1, Colors.Blue, 2, "Z");
            graphEulerX2 = plotterIMU2.AddLineGraph(EulerX2, Colors.Green, 2, "X");//add new curve
            graphEulerY2 = plotterIMU2.AddLineGraph(EulerY2, Colors.Red, 2, "Y");
            graphEulerZ2 = plotterIMU2.AddLineGraph(EulerZ2, Colors.Blue, 2, "Z");
            graphEulerX3 = plotterIMU3.AddLineGraph(EulerX3, Colors.Green, 2, "X");//add new curve
            graphEulerY3 = plotterIMU3.AddLineGraph(EulerY3, Colors.Red, 2, "Y");
            graphEulerZ3 = plotterIMU3.AddLineGraph(EulerZ3, Colors.Blue, 2, "Z");

            timer.Interval = TimeSpan.FromSeconds(1);
            timer.Tick += new EventHandler(timer_Tick);
            timer.IsEnabled = true;

            plotterIMU1.Viewport.FitToView();
            plotterIMU2.Viewport.FitToView();
            plotterIMU3.Viewport.FitToView();

            

            //senior's code for plotting
            /*
            timer.Interval = TimeSpan.FromSeconds(0.1);
            timer.Tick += new EventHandler(timer_Tick);

            plotterMotorVelocity.AddLineGraph(velocity, Colors.Green, 2, "Velocity");
            */
            robotStatus = FindResource("RobotStatus") as RobotStatus;
        }

        void timer_Tick(object sender, EventArgs e)
        {
            //my code from internet
            double x = time;

            double y = robotStatus.EulerX1/3.14*180;
            Point point = new Point(x, y);
            EulerX1.AppendAsync(base.Dispatcher, point);
            y = robotStatus.EulerY1 / 3.14 * 180;
            point = new Point(x, y);
            EulerY1.AppendAsync(base.Dispatcher, point);
            y = robotStatus.EulerZ1 / 3.14 * 180;
            point = new Point(x, y);
            EulerZ1.AppendAsync(base.Dispatcher, point);

            y = robotStatus.EulerX2 / 3.14 * 180;
            point = new Point(x, y);
            EulerX2.AppendAsync(base.Dispatcher, point);
            y = robotStatus.EulerY2 / 3.14 * 180;
            point = new Point(x, y);
            EulerY2.AppendAsync(base.Dispatcher, point);
            y = robotStatus.EulerZ2 / 3.14 * 180;
            point = new Point(x, y);
            EulerZ2.AppendAsync(base.Dispatcher, point);

            y = robotStatus.EulerX3 / 3.14 * 180;
            point = new Point(x, y);
            EulerX3.AppendAsync(base.Dispatcher, point);
            y = robotStatus.EulerY3 / 3.14 * 180;
            point = new Point(x, y);
            EulerY3.AppendAsync(base.Dispatcher, point);
            y = robotStatus.EulerZ3 / 3.14 * 180;
            point = new Point(x, y);
            EulerZ3.AppendAsync(base.Dispatcher, point);
            //cpuUsageText.Text = String.Format("{0:0}%", y);
            time+=1;
            
            //senior's code and commented
            /*
            time += 0.1;
            Point p = new Point(time, );
            tension.AppendAsync(base.Dispatcher, p);
            */
            
        }
        delegate void UpdateRobotStatusDelegate(byte[] data);
        private void UpdateRobotStatus(byte[] data)
        {
            robotStatus.ReadJointAngle(data);
        }
        void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            UpdateFrameCountReceived ufcr = delegate(int count)
            {
                tbFrameCount.Text = count.ToString();
            };
            int n = sp.BytesToRead;
            if (n > 0)
            {
                byte[] array = new byte[n];
                sp.Read(array, 0, n);
                sp3.Append(array);
                while (sp3.isFrameExisted())
                {
                    if (sp3.Length == 53)   // modified
                    {
                        
                        byte[] data = new byte[sp3.Length - 1];
                        data[0] = sp3.Command;
                        sp3.Content.CopyTo(data, 1);
                        UpdateRobotStatusDelegate ursd = new UpdateRobotStatusDelegate(UpdateRobotStatus);
                        Dispatcher.Invoke(ursd, data);
                    }
                    else if(sp3.Length == 6)
                    {
                        byte status = sp3.Command;
                        if (status == 1)
                        {
                            isTransferDone = true;
                        }
                        else
                        {
                            isTransferDone = false;
                        }
                        frameRcvCount = BitConverter.ToInt16(sp3.Content, 0);
                        frameMotionCount = BitConverter.ToInt16(sp3.Content, 2);
                        frameRcvCountList.Add(frameRcvCount);
                        frameMotionCountList.Add(frameMotionCount);
                        bufferSize = frameRcvCount - frameMotionCount;
                        Dispatcher.BeginInvoke(ufcr, bufferSize);
                    }
                }
            }
        }
        List<int> frameRcvCountList = new List<int>();
        List<int> frameMotionCountList = new List<int>();
        volatile bool isTransferDone = false;

        private byte[] GenerateAddressValuePair(List<UInt16> address, List<UInt16> value)
        {
            byte[] array = new byte[(address.Count + value.Count) * 2];
            byte[] add_byte;
            byte[] value_byte;

            if (address.Count != value.Count)
            {
                Console.WriteLine("地址和值的数量不匹配！");
                return null;
            }
            for (int i = 0; i < address.Count; i++)
            {
                add_byte = BitConverter.GetBytes(address[i]);
                Array.Reverse(add_byte);
                value_byte = BitConverter.GetBytes(value[i]);
                Array.Reverse(value_byte);
                add_byte.CopyTo(array, 4 * i);
                value_byte.CopyTo(array, 4 * i + 2);
            }
            return array;
        }

        private void btnPowerOn_Click(object sender, RoutedEventArgs e)
        {
            if (!btnPowerOn.IsChecked.HasValue)
            {
                MessageBox.Show("上电按钮失败!");
            }
            if ((bool)btnPowerOn.IsChecked)
            {
                try
                {
                    if (!sp.IsOpen)
                        sp.Open();
                    sp.DiscardInBuffer();
                    sp.DiscardOutBuffer();
                    //timer.Start();
                    PowerOnText = "关闭串口";
                }
                catch (System.Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                    btnPowerOn.IsChecked = false;
                }
            }
            else
            {
                try
                {
                    sp.DiscardInBuffer();
                    sp.DiscardOutBuffer();
                    sp.Close();
                    //timer.Stop();
                    time = 0.0;
                    PowerOnText = "打开串口";
                }
                catch (System.Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                }
            }
        }

        private void btnSingleJointZero_Click(object sender, RoutedEventArgs e)
        {
            sdrSZ.Value = 0;
            sdrSY.Value = 0;
            sdrSX.Value = 0;
            sdrE.Value = 0;
            sdrWZ.Value = 0;
            sdrWY.Value = 0;
            sdrWX.Value = 0;
        }

        private void btnSingleJointAction_Click(object sender, RoutedEventArgs e)
        {
            byte[] array;

            List<UInt16> address = new List<UInt16>();
            List<UInt16> value = new List<UInt16>();

            address.Add((UInt16)DSP_ADDRESS.CMD_JOINT_SZ);
            address.Add((UInt16)DSP_ADDRESS.CMD_JOINT_SY);
            address.Add((UInt16)DSP_ADDRESS.CMD_JOINT_SX);
            address.Add((UInt16)DSP_ADDRESS.CMD_JOINT_EL);
            address.Add((UInt16)DSP_ADDRESS.CMD_JOINT_WZ);
            address.Add((UInt16)DSP_ADDRESS.CMD_JOINT_WY);
            address.Add((UInt16)DSP_ADDRESS.CMD_JOINT_WX);

            value.Add((UInt16)(sdrSZ.Value * 10));
            value.Add((UInt16)(sdrSY.Value * 10));
            value.Add((UInt16)(sdrSX.Value * 10));
            value.Add((UInt16)(sdrE.Value * 10));
            value.Add((UInt16)(sdrWZ.Value * 10));
            value.Add((UInt16)(sdrWY.Value * 10));
            value.Add((UInt16)(sdrWX.Value * 10));

            array = sp3.GenerateFrame(GenerateAddressValuePair(address, value));

            if (sp.IsOpen)
            {
                sp.Write(array, 0, array.Length);
            }

            AddArray2RichTextBox(array);
            StatusBarText = "发送关节角命令";
        }

        private void AddArray2RichTextBox(byte[] array)
        {
            Run r = new Run(BitConverter.ToString(array));
            paraConsole.Inlines.Add(r);
            paraConsole.Inlines.Add(new LineBreak());
            //滚动到当前光标处
            rtbConsole.ScrollToEnd();
        }

        private void ComboBox_Loaded(object sender, RoutedEventArgs e)
        {
            List<CONTROL_MODE> controlModeList = new List<CONTROL_MODE>();
            controlModeList.Add(CONTROL_MODE.JOINT_CONTROL);
            controlModeList.Add(CONTROL_MODE.MOTOR_CONTROL);
            controlModeList.Add(CONTROL_MODE.TRAJ_TRANSFER);
            controlModeList.Add(CONTROL_MODE.LINE_MOVEMENT);

            ComboBox box = sender as ComboBox;
            box.ItemsSource = controlModeList;
        }

        private void btnModeConfirm_Click(object sender, RoutedEventArgs e)
        {
            if (cbMode.SelectedItem != null)
            {
                CONTROL_MODE cm = (CONTROL_MODE)cbMode.SelectedItem;
                List<ushort> address = new List<ushort>();
                List<ushort> value = new List<ushort>();
                address.Add(0x300);
                value.Add((ushort)cm);
                byte[] xmtData = sp3.GenerateFrame(GenerateAddressValuePair(address, value));
                if (sp.IsOpen)
                {
                    sp.Write(xmtData, 0, xmtData.Length);
                }
                Run r = new Run(BitConverter.ToString(xmtData));
                paraConsole.Inlines.Add(r);
                paraConsole.Inlines.Add(new LineBreak());
                //滚动到当前光标处
                rtbConsole.ScrollToEnd();
            }
        }

        private void btnResetConfirm_Click(object sender, RoutedEventArgs e)
        {
           
            List<ushort> address = new List<ushort>();
            List<ushort> value = new List<ushort>();
            address.Add(0x300);
            value.Add(0x05);//IMU_reset
            byte[] xmtData = sp3.GenerateFrame(GenerateAddressValuePair(address, value));
            if (sp.IsOpen)
            {
                sp.Write(xmtData, 0, xmtData.Length);
            }
            Run r = new Run(BitConverter.ToString(xmtData));
            paraConsole.Inlines.Add(r);
            paraConsole.Inlines.Add(new LineBreak());
            //滚动到当前光标处
            rtbConsole.ScrollToEnd();

            //clear the screen
            plotterIMU1.Children.Remove(graphEulerX1);
            plotterIMU1.Children.Remove(graphEulerY1);
            plotterIMU1.Children.Remove(graphEulerZ1);
            plotterIMU2.Children.Remove(graphEulerX2);
            plotterIMU2.Children.Remove(graphEulerY2);
            plotterIMU2.Children.Remove(graphEulerZ2);
            plotterIMU3.Children.Remove(graphEulerX3);
            plotterIMU3.Children.Remove(graphEulerY3);
            plotterIMU3.Children.Remove(graphEulerZ3);
            //timer.IsEnabled = true;
            EulerX1 = new ObservableDataSource<Point>();//draw curve
            EulerY1 = new ObservableDataSource<Point>();
            EulerZ1 = new ObservableDataSource<Point>();
            EulerX2 = new ObservableDataSource<Point>();//draw curve
            EulerY2 = new ObservableDataSource<Point>();
            EulerZ2 = new ObservableDataSource<Point>();
            EulerX3 = new ObservableDataSource<Point>();//draw curve
            EulerY3 = new ObservableDataSource<Point>();
            EulerZ3 = new ObservableDataSource<Point>();
            time = 0;
            graphEulerX1 = plotterIMU1.AddLineGraph(EulerX1, Colors.Green, 2, "X");//add new curve
            graphEulerY1 = plotterIMU1.AddLineGraph(EulerY1, Colors.Red, 2, "Y");
            graphEulerZ1 = plotterIMU1.AddLineGraph(EulerZ1, Colors.Blue, 2, "Z");
            graphEulerX2 = plotterIMU2.AddLineGraph(EulerX2, Colors.Green, 2, "X");//add new curve
            graphEulerY2 = plotterIMU2.AddLineGraph(EulerY2, Colors.Red, 2, "Y");
            graphEulerZ2 = plotterIMU2.AddLineGraph(EulerZ2, Colors.Blue, 2, "Z");
            graphEulerX3 = plotterIMU3.AddLineGraph(EulerX3, Colors.Green, 2, "X");//add new curve
            graphEulerY3 = plotterIMU3.AddLineGraph(EulerY3, Colors.Red, 2, "Y");
            graphEulerZ3 = plotterIMU3.AddLineGraph(EulerZ3, Colors.Blue, 2, "Z");
           
        }

        List<double> q1 = new List<double>();
        List<double> q2 = new List<double>();
        List<double> q3 = new List<double>();
        List<double> q4 = new List<double>();
        List<double> q5 = new List<double>();
        List<double> q6 = new List<double>();
        List<double> q7 = new List<double>();
        private void btnLoadFile_Click(object sender, RoutedEventArgs e)
        {
            string strPath = Environment.CurrentDirectory;
            Microsoft.Win32.OpenFileDialog dialogOpenFile = new Microsoft.Win32.OpenFileDialog();
            dialogOpenFile.DefaultExt = "txt";          //默认扩展名
            dialogOpenFile.AddExtension = true;         //是否自动添加扩展名
            dialogOpenFile.Filter = "文本文件(.txt)|*.txt";
            dialogOpenFile.CheckFileExists = true;
            dialogOpenFile.CheckPathExists = true;
            dialogOpenFile.FileName = "JointAngle";          //默认文件名
            dialogOpenFile.CheckPathExists = true;      //提示输入的文件名无效
            dialogOpenFile.Title = "打开关节角文件";

            bool? b = dialogOpenFile.ShowDialog();
            if (b == true)
            {
                tbFileName.Text = dialogOpenFile.FileName;
                q1.Clear(); q2.Clear(); q3.Clear(); q4.Clear(); q5.Clear(); q6.Clear(); q7.Clear();
                try
                {
                    using (StreamReader sr = new StreamReader(tbFileName.Text, System.Text.Encoding.Default))
                    {
                        while (!sr.EndOfStream)
                        {
                            string line = sr.ReadLine();
                            string[] values = line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                            if (values.Length != 7) break;
                            q1.Add(Convert.ToDouble(values[0]));
                            q2.Add(Convert.ToDouble(values[1]));
                            q3.Add(Convert.ToDouble(values[2]));
                            q4.Add(Convert.ToDouble(values[3]));
                            q5.Add(Convert.ToDouble(values[4]));
                            q6.Add(Convert.ToDouble(values[5]));
                            q7.Add(Convert.ToDouble(values[6]));
                        }
                    }
                }
                catch (System.Exception)
                {
                	
                }
            }
        }

        volatile int bufferSize = 0;
        volatile int frameRcvCount = 0;
        volatile int frameMotionCount = 0;

        delegate void ToggleBtnStartTransferStatus(bool status);
        delegate void UpdateFrameCountReceived(int allCount);
        void TransferTraj(object state)
        {
            ToggleBtnStartTransferStatus del = delegate(bool status)
            {
                if (status)
                {
                    btnStartTransfer.IsEnabled = true;
                    btnStartTransfer.Content = "开始运动";
                }
                else
                {
                    btnStartTransfer.IsEnabled = false;
                    btnStartTransfer.Content = "正在运动..";
                }
            };

            frameMotionCountList.Clear();
            frameRcvCountList.Clear();
            int n = q1.Count;
            if (n <= 0)
            {
                MessageBox.Show("数据不存在！");
                return;
            }
            List<UInt16> address = new List<UInt16>() { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            List<UInt16> values = new List<UInt16>() { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            this.Dispatcher.Invoke(del, false);
            for (int i = 0; i < n; i++)
            {
                byte[] xmtData;
                //if (bufferSize > 50)    // 只动不存
                //{
                //    while (bufferSize > 50)
                //    {
                //        address[7] = Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_CONTROL);
                //        values[7] = (UInt16)(0xAAA0);
                //        xmtData = sp3.GenerateFrame(GenerateAddressValuePair(address, values));
                //        if (sp.IsOpen)
                //        {
                //            sp.Write(xmtData, 0, xmtData.Length);
                //        }
                //        else
                //        {
                //            MessageBox.Show("请连接设备！");
                //            return;
                //        }
                //        Thread.Sleep(5);
                //    }
                //}
                address[0] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_SZ));
                address[1] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_SY));
                address[2] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_SX));
                address[3] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_EL));
                address[4] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_WZ));
                address[5] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_WY));
                address[6] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_WX));
                values[0] = ((UInt16)(q1[i] / Math.PI * 180 * 100));
                values[1] = ((UInt16)(q2[i] / Math.PI * 180 * 100));
                values[2] = ((UInt16)(q3[i] / Math.PI * 180 * 100));
                values[3] = ((UInt16)(q4[i] / Math.PI * 180 * 100));
                values[4] = ((UInt16)(q5[i] / Math.PI * 180 * 100));
                values[5] = ((UInt16)(q6[i] / Math.PI * 180 * 100));
                values[6] = ((UInt16)(q7[i] / Math.PI * 180 * 100));
                if (i < 50) // 只存不动
                {
                    address[7] = Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_CONTROL);
                    values[7] = (UInt16)(0xAA0A);
                }
                else  // 变动变存
                {
                    address[7] = Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_CONTROL);
                    values[7] = (UInt16)(0xAAAA);
                }
                address[8] = Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_INDEX);
                values[8] = (UInt16)(i + 1);
                xmtData = sp3.GenerateFrame(GenerateAddressValuePair(address, values));
                while (isTransferDone == false)
                {
                    if (sp.IsOpen)
                    {
                        sp.Write(xmtData, 0, xmtData.Length);
                    }
                    else
                    {
                        MessageBox.Show("请连接设备！");
                        return;
                    }
                }
                isTransferDone = false;

            }

//            while (bufferSize > 0)  // 只动不存
            {
                address[7] = Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_CONTROL);
                values[7] = (UInt16)(0xAAA0);
                byte[] xmtData = sp3.GenerateFrame(GenerateAddressValuePair(address, values));
                if (sp.IsOpen)
                {
                    sp.Write(xmtData, 0, xmtData.Length);
                }
                else
                {
                    MessageBox.Show("请连接设备！");
                    return;
                }
            }
            while (bufferSize > 0) ;

            this.Dispatcher.Invoke(del, true);

            using (StreamWriter sw = new StreamWriter("abc.txt", false))
            {
                for (int i = 0; i < frameMotionCountList.Count; i++)
                {
                    sw.Write(frameRcvCountList[i]);
                    sw.Write(',');
                    sw.Write(frameMotionCountList[i]);
                    sw.Write(',');
                    sw.WriteLine(frameRcvCountList[i] - frameMotionCountList[i]);
                }
            }

            MessageBox.Show("轨迹运动结束！");
        }
        
        private void btnStartTransfer_Click(object sender, RoutedEventArgs e)
        {
            ThreadPool.QueueUserWorkItem(new WaitCallback(TransferTraj), false);
        }

        // 运动到起点
        private void btnStartMovement_Click(object sender, RoutedEventArgs e)
        {
            List<UInt16> address = new List<UInt16>() { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            List<UInt16> values = new List<UInt16>() { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            try
            {
                address[0] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_SZ));
                address[1] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_SY));
                address[2] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_SX));
                address[3] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_EL));
                address[4] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_WZ));
                address[5] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_WY));
                address[6] = (Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_WX));
                values[0] = ((UInt16)(q1[0] / Math.PI * 180 * 100));
                values[1] = ((UInt16)(q2[0] / Math.PI * 180 * 100));
                values[2] = ((UInt16)(q3[0] / Math.PI * 180 * 100));
                values[3] = ((UInt16)(q4[0] / Math.PI * 180 * 100));
                values[4] = ((UInt16)(q5[0] / Math.PI * 180 * 100));
                values[5] = ((UInt16)(q6[0] / Math.PI * 180 * 100));
                values[6] = ((UInt16)(q7[0] / Math.PI * 180 * 100));
            }
            catch (System.Exception ex)
            {
                return;
            }

            address[7] = Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_CONTROL);
            values[7] = (UInt16)(0xAA0A);
            address[8] = Convert.ToUInt16(DSP_ADDRESS.CMD_TRAJ_INDEX);
            values[8] = (UInt16)(0);
            byte[] xmtData = sp3.GenerateFrame(GenerateAddressValuePair(address, values));
            while (isTransferDone == false)
            {
                if (sp.IsOpen)
                {
                    sp.Write(xmtData, 0, xmtData.Length);
                }
                else
                {
                    MessageBox.Show("请连接设备！");
                    return;
                }
            }
        }

        private void btnLoadStartPoint_Click(object sender, RoutedEventArgs e)
        {
            sdrSZ.Value = q1[0] / Math.PI * 180;
            sdrSY.Value = q2[0] / Math.PI * 180;
            sdrSX.Value = q3[0] / Math.PI * 180;
            sdrE.Value =  q4[0] / Math.PI * 180;
            sdrWZ.Value = q5[0] / Math.PI * 180;
            sdrWY.Value = q6[0] / Math.PI * 180;
            sdrWX.Value = q7[0] / Math.PI * 180;
        }

        #region INotifyPropertyChanged Members

        public event PropertyChangedEventHandler PropertyChanged;

        #endregion

        #region Private Helpers

        private void NotifyPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }

        #endregion
        
        
        private void btnPullW1_Click(object sender, RoutedEventArgs e)
        {

        }

        private void btnPullE2_Click(object sender, RoutedEventArgs e)
        {

        }
        

        private void btnMechPara_Click(object sender, RoutedEventArgs e)
        {
            byte[] array;

            List<UInt16> address = new List<UInt16>();
            List<UInt16> value = new List<UInt16>();

            
            address.Add((UInt16)DSP_ADDRESS.MECH_l1);
            address.Add((UInt16)DSP_ADDRESS.MECH_l2);
            address.Add((UInt16)DSP_ADDRESS.MECH_h1);
            address.Add((UInt16)DSP_ADDRESS.MECH_h2);
            address.Add((UInt16)DSP_ADDRESS.MECH_A1);
            address.Add((UInt16)DSP_ADDRESS.MECH_A2);
            address.Add((UInt16)DSP_ADDRESS.MECH_A3);
            address.Add((UInt16)DSP_ADDRESS.MECH_A4);
            address.Add((UInt16)DSP_ADDRESS.MECH_B1);
            address.Add((UInt16)DSP_ADDRESS.MECH_B2);
            address.Add((UInt16)DSP_ADDRESS.MECH_B3);
            address.Add((UInt16)DSP_ADDRESS.MECH_B4);
            address.Add((UInt16)DSP_ADDRESS.MECH_B5);
            address.Add((UInt16)DSP_ADDRESS.MECH_B6);
            address.Add((UInt16)DSP_ADDRESS.MECH_C1);
            address.Add((UInt16)DSP_ADDRESS.MECH_C2);
            

            value.Add((UInt16)(l1.Value * 10));
            value.Add((UInt16)(l2.Value * 10));
            value.Add((UInt16)(h1.Value * 10));
            value.Add((UInt16)(h2.Value * 10));
            value.Add((UInt16)(A1.Value * 10));
            value.Add((UInt16)(A2.Value * 10));
            value.Add((UInt16)(A3.Value * 10));
            value.Add((UInt16)(A4.Value * 10));
            value.Add((UInt16)(B1.Value * 10));
            value.Add((UInt16)(B2.Value * 10));
            value.Add((UInt16)(B3.Value * 10));
            value.Add((UInt16)(B4.Value * 10));
            value.Add((UInt16)(B5.Value * 10));
            value.Add((UInt16)(B6.Value * 10));
            value.Add((UInt16)(C1.Value * 10));
            value.Add((UInt16)(C2.Value * 10));
            



            array = sp3.GenerateFrame(GenerateAddressValuePair(address, value));

            if (sp.IsOpen)
            {
                sp.Write(array, 0, array.Length);
            }

            AddArray2RichTextBox(array);
            StatusBarText = "发送机械参数";
            
        }





        

    }
}