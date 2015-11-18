using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ControlPannel
{
    public enum DSP_ADDRESS
    {
        // 电机速度地址 0x00-0x09(RW)
        MOTOR_VELOCITY_1 = 0x0000,
        MOTOR_VELOCITY_2 = 0x0001,
        MOTOR_VELOCITY_3 = 0x0002,
        MOTOR_VELOCITY_4 = 0x0003,
        MOTOR_VELOCITY_5 = 0x0004,
        MOTOR_VELOCITY_6 = 0x0005,
        MOTOR_VELOCITY_7 = 0x0006,
        MOTOR_VELOCITY_8 = 0x0007,
        MOTOR_VELOCITY_9 = 0x0008,
        MOTOR_VELOCITY_A = 0x0009,

        // 电机位置地址 0x10-0x23(RO)
        MOTOR_POSITION_1H = 0x0010,
        MOTOR_POSITION_1L = 0x0011,
        MOTOR_POSITION_2H = 0x0012,
        MOTOR_POSITION_2L = 0x0013,
        MOTOR_POSITION_3H = 0x0014,
        MOTOR_POSITION_3L = 0x0015,
        MOTOR_POSITION_4H = 0x0016,
        MOTOR_POSITION_4L = 0x0017,
        MOTOR_POSITION_5H = 0x0018,
        MOTOR_POSITION_5L = 0x0019,
        MOTOR_POSITION_6H = 0x001A,
        MOTOR_POSITION_6L = 0x001B,
        MOTOR_POSITION_7H = 0x001C,
        MOTOR_POSITION_7L = 0x001D,
        MOTOR_POSITION_8H = 0x001E,
        MOTOR_POSITION_8L = 0x001F,
        MOTOR_POSITION_9H = 0x0020,
        MOTOR_POSITION_9L = 0x0021,
        MOTOR_POSITION_AH = 0x0022,
        MOTOR_POSITION_AL = 0x0023,

        // 绳索实际张力 0x30-0x37(RO)
        CABLE_TENSION_1 = 0x0030,
        CABLE_TENSION_2 = 0x0031,
        CABLE_TENSION_3 = 0x0032,
        CABLE_TENSION_4 = 0x0033,
        CABLE_TENSION_5 = 0x0034,
        CABLE_TENSION_6 = 0x0035,
        CABLE_TENSION_7 = 0x0036,
        CABLE_TENSION_8 = 0x0037,

        // 绳索张力指令 0x38-0x3F(WR)
        CMD_CABLE_TENSION_1 = 0x0038,
        CMD_CABLE_TENSION_2 = 0x0039,
        CMD_CABLE_TENSION_3 = 0x003A,
        CMD_CABLE_TENSION_4 = 0x003B,
        CMD_CABLE_TENSION_5 = 0x003C,
        CMD_CABLE_TENSION_6 = 0x003D,
        CMD_CABLE_TENSION_7 = 0x003E,
        CMD_CABLE_TENSION_8 = 0x003F,

        // 关节角指令 0x40-0x46(WR)
        CMD_JOINT_SZ = 0x0040,
        CMD_JOINT_SY = 0x0041,
        CMD_JOINT_SX = 0x0042,
        CMD_JOINT_EL = 0x0043,
        CMD_JOINT_WZ = 0x0044,
        CMD_JOINT_WY = 0x0045,
        CMD_JOINT_WX = 0x0046,

        // 电机速度指令
        CMD_MOTOR_VELOCITY_1 = 0x0050,
        CMD_MOTOR_VELOCITY_2 = 0x0051,
        CMD_MOTOR_VELOCITY_3 = 0x0052,
        CMD_MOTOR_VELOCITY_4 = 0x0053,
        CMD_MOTOR_VELOCITY_5 = 0x0054,
        CMD_MOTOR_VELOCITY_6 = 0x0055,
        CMD_MOTOR_VELOCITY_7 = 0x0056,
        CMD_MOTOR_VELOCITY_8 = 0x0057,
        CMD_MOTOR_VELOCITY_9 = 0x0058,
        CMD_MOTOR_VELOCITY_A = 0x0059,

        //机械结构参数
        MECH_l1 = 0x0060,
        MECH_l2 = 0x0061,
        MECH_h1 = 0x0062,
        MECH_h2 = 0x0063,
        MECH_A1 = 0x0064,
        MECH_A2 = 0x0065,
        MECH_A3 = 0x0066,
        MECH_A4 = 0x0067,
        MECH_B1 = 0x0068,
        MECH_B2 = 0x0069,
        MECH_B3 = 0x006A,
        MECH_B4 = 0x006B,
        MECH_B5 = 0x006C,
        MECH_B6 = 0x006D,
        MECH_C1 = 0x006E,
        MECH_C2 = 0x006F,


        // 轨迹所需关节角寄存器
        CMD_TRAJ_CONTROL    = 0x0100,
        CMD_TRAJ_INDEX      = 0x0101,
        CMD_TRAJ_SZ         = 0x0102,
        CMD_TRAJ_SY         = 0x0103,
        CMD_TRAJ_SX         = 0x0104,
        CMD_TRAJ_EL         = 0x0105,
        CMD_TRAJ_WZ         = 0x0106,
        CMD_TRAJ_WY         = 0x0107,
        CMD_TRAJ_WX         = 0x0108,

        // 机器人控制模式指令
        CMD_CONTROL_MODE = 0x0300,  // 0x01：表示电机速度控制，0x02：表示关节角度控制，0x03：表示传输轨迹，0x04：表示直线运动
    }

    public enum CONTROL_MODE
    {
        MOTOR_CONTROL = 0x01,
        JOINT_CONTROL = 0x02,
        TRAJ_TRANSFER = 0x03,
        LINE_MOVEMENT = 0x04
    }

    class DspMemoryModel
    {
        UInt16[] memory = new UInt16[1024];


        // 读取单个数据
        UInt16 LoadData(UInt16 address)
        {
            if (address < 0 || address > 1024)
            {
                Console.WriteLine("读取失败，地址超出范围");
                return 0;
            }
            return memory[address];
        }

        // 单个地址写入
        void SaveData(UInt16 address, UInt16 value)
        {
            if (address < 0 || address > 1024)
            {
                Console.WriteLine("写入失败，地址超出范围");
                return;
            }
            memory[address] = value;
        }

        // 连续地址写入
        void SaveData(UInt16 address, UInt16[] value)
        {
            for (int i = 0; i < value.Length; i++ )
            {
                SaveData(address, value[i]);
                address++;
            }
        }

        // 不连续地址写入
        void SaveData(UInt16[] address, UInt16[] value)
        {
            if (address.Length != value.Length)
            {
                Console.WriteLine("写入失败，数据地址数量不匹配");
                return;
            }
            int n = address.Length;
            for (int i = 0; i < n; i++ )
            {
                SaveData(address[i], value[i]);
            }
        }
    }
}
