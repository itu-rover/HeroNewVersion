using Microsoft.SPOT;
using System;

namespace Hero21Core
{
    class Utils
    {
        public static int Clamp(int value, int lower, int upper)
        {
            if (value > upper)
                return upper;
            else if (value < lower)
                return lower;
            else
                return value;
        }

        public static double Map(double value, double current_from, double current_to, double target_from, double target_to)
        {
            return (int)(target_from + (value - current_from) / (current_to - current_from) * (target_to - target_from));
        }
    }
}
