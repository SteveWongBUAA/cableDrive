using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ControlPannel
{
    class Motor
    {
        private double _velocity;

        public double Velocity
        {
            get { return _velocity; }
            set { _velocity = value; }
        }

        private double _position;

        public double Position
        {
            get { return _position; }
            set { _position = value; }
        }
    }
}
