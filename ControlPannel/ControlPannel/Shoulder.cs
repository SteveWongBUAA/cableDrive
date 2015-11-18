using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.ComponentModel;

namespace ControlPannel
{
    class Shoulder: INotifyPropertyChanged
    {
        private double _az;

        public double Az
        {
            get { return _az; }
            set { _az = value; NotifyPropertyChanged("Az"); }
        }

        private double _ay;

        public double Ay
        {
            get { return _ay; }
            set { _ay = value; NotifyPropertyChanged("Ay"); }
        }

        private double _ax;

        public double Ax
        {
            get { return _ax; }
            set { _ax = value; NotifyPropertyChanged("Ax"); }
        }

        private Motor _m1;

        public Motor M1
        {
            get { return _m1; }
            set { _m1 = value; NotifyPropertyChanged("M1"); }
        }

        private Motor _m2;

        public Motor M2
        {
            get { return _m2; }
            set { _m2 = value; NotifyPropertyChanged("M2"); }
        }

        private Motor _m3;

        public Motor M3
        {
            get { return _m3; }
            set { _m3 = value; NotifyPropertyChanged("M3"); }
        }

        private Motor _m4;

        public Motor M4
        {
            get { return _m4; }
            set { _m4 = value; NotifyPropertyChanged("M4"); }
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

    }
}
