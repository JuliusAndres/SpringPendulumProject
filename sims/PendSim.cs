//============================================================================
// PendSim.cs : Defines derived class for simulating a simple pendulum.
//============================================================================
using System;

public class PendSim : Simulator
{
    private double L_0;      // Natural original length
    private double L;      // Spring Extended Length
    private double k;     // Spring Constant
    private double m;     // ball mass

    public PendSim() : base(6)
    {
        // replacing 2 with 6 for base
        L_0 = 0.9f;
        k = 90;
        m = 1.4;
        g = 9.81;

        //x[0] = 1.0;   // default pendulum angle
        //x[1] = 0.0;   // default rotation rate

        x[0] = -0.7;    // X position
        x[1] = 0.0;     // X velocity
        x[2] = -0.2;    // Y position
        x[3] = 0.0;     // Y velocity
        x[4] = 0.5;     // Z position
        x[5] = 0.9;     // Z velocity

        SetRHSFunc(RHSFuncPendulum);
    }

    //----------------------------------------------------
    // RHSFuncPendulum
    //----------------------------------------------------
    private void RHSFuncPendulum(double[] xx,
        double t, double[] ff)
    {
        //ff[0] = xx[1];
        //ff[1] = -g/L_0 * Math.Sin(xx[0]);

        // Physics equations for spring pendulum go here
        L = Math.Sqrt((xx[0]*xx[0]) + (xx[2]*xx[2]) + (xx[4]*xx[4]));
        ff[0] = xx[1];
        ff[1] = (-k * (L - L_0) * xx[0]) / (m * L);
        ff[2] = xx[3];
        ff[3] = ((-k * (L - L_0) * xx[2]) / (m * L)) - g;
        ff[4] = xx[5];
        ff[5] = (-k * (L - L_0) * xx[4]) / (m * L);
    }

    //--------------------------------------------------------------------
    // Getters
    //--------------------------------------------------------------------
    // public double Angle
    // {
    //     get{
    //         return(x[0]);
    //     }

    //     set{
    //         x[0] = value;
    //     }
    // }
    public double X
    {
        get{
            return(x[0]);
        }
        set{
            x[0] = value;
        }
    }
    public double XDot
    {
        get{
            return(x[1]);
        }
        set{
            x[1] = value;
        }
    }
    public double Y
    {
        get{
            return(x[2]);
        }
        set{
            x[2] = value;
        }
    }
    public double YDot
    {
        get{
            return(x[3]);
        }
        set{
            x[3] = value;
        }
    }
    public double Z
    {
        get{
            return(x[4]);
        }
        set{
            x[4] = value;
        }
    }
    public double ZDot
    {
        get{
            return(x[5]);
        }
        set{
            x[5] = value;
        }
    }
    public double KineticE
    {
        get{
            // KE = 1/2 * m * (xdot^2 + ydot^2 + zdot^2)
            return(0.5 * m * (x[1]*x[1] + x[3]*x[3] + x[5]*x[5]));
        }
    }
    public double PotentialE
    {
        get{
            // PE = m*g*y + 1/2 * k * (L-L0)^2
            L = Math.Sqrt( (x[0]*x[0]) + (x[2]*x[2]) + (x[4]*x[4]) );
            return( (m*g*x[2]) + (0.5*k*(L-L_0)*(L-L_0)) );
        }
    }

}