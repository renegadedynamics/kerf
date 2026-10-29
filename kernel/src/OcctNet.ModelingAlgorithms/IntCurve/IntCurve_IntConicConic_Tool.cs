// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic_Tool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;

namespace Occt;

/// <summary>
/// The free functions and helper classes of OCCT's private header IntCurve_IntConicConic_Tool.hxx, used by
/// <see cref="IntCurve_IntConicConic"/>.
/// </summary>
/// <remarks>
/// OCCT's global classes Interval and PeriodicInterval are nested structs here (they are copied by value in C++),
/// imported with <c>using static</c>. Their mutators stay public within the internal types.
/// </remarks>
internal static class IntCurve_IntConicConic_Tool
{
    internal static readonly double PIpPI = M_PI + M_PI;

    private const double TOLERANCE_ANGULAIRE = 0.00000001;

    //======================================================================
    //===         R e s s o u r c e s        G e n e r a l e s           ===
    //======================================================================

    /// <summary>
    /// The transitions at an intersection point of a line or circle with another conic. OCCT normalises
    /// <paramref name="Tan1"/> in place in the tangent case, hence <c>ref</c>.
    /// </summary>
    internal static void Determine_Transition_LC(IntRes2d_Position Pos1,
                                                 ref gp_Vec2d Tan1,
                                                 in gp_Vec2d Norm1,
                                                 ref IntRes2d_Transition T1,
                                                 IntRes2d_Position Pos2,
                                                 ref gp_Vec2d Tan2,
                                                 in gp_Vec2d Norm2,
                                                 ref IntRes2d_Transition T2,
                                                 double Tol)
    {
        double sgn = Tan1.Crossed(Tan2);
        double norm = Tan1.Magnitude() * Tan2.Magnitude();

        if (Math.Abs(sgn) <= TOLERANCE_ANGULAIRE * norm)
        { // Transition TOUCH #########
            bool opos = (Tan1.Dot(Tan2)) < 0;

            gp_Vec2d Norm = new();
            //  Modified by Sergey KHROMOV - Thu Nov  2 17:57:15 2000 Begin
            Tan1.Normalize();
            //  Modified by Sergey KHROMOV - Thu Nov  2 17:57:16 2000 End
            Norm.SetCoord(-Tan1.Y(), Tan1.X());

            double Val1 = Norm.Dot(Norm1);
            double Val2 = Norm.Dot(Norm2);

            if (Math.Abs(Val1 - Val2) <= gp.Resolution())
            {
                T1.SetValue(true, Pos1, IntRes2d_Unknown, opos);
                T2.SetValue(true, Pos2, IntRes2d_Unknown, opos);
            }
            else if (Val2 > Val1)
            {
                T2.SetValue(true, Pos2, IntRes2d_Inside, opos);
                if (opos)
                {
                    T1.SetValue(true, Pos1, IntRes2d_Inside, opos);
                }
                else
                {
                    T1.SetValue(true, Pos1, IntRes2d_Outside, opos);
                }
            }
            else
            { // Val1 > Val2
                T2.SetValue(true, Pos2, IntRes2d_Outside, opos);
                if (opos)
                {
                    T1.SetValue(true, Pos1, IntRes2d_Outside, opos);
                }
                else
                {
                    T1.SetValue(true, Pos1, IntRes2d_Inside, opos);
                }
            }
        }
        else if (sgn < 0)
        {
            T1.SetValue(false, Pos1, IntRes2d_In);
            T2.SetValue(false, Pos2, IntRes2d_Out);
        }
        else
        {
            T1.SetValue(false, Pos1, IntRes2d_Out);
            T2.SetValue(false, Pos2, IntRes2d_In);
        }
    }

    //----------------------------------------------------------------------
    internal static double NormalizeOnCircleDomain(double _Param, in IntRes2d_Domain TheDomain)
    {
        double Param = _Param;
        while (Param < TheDomain.FirstParameter())
        {
            Param += PIpPI;
        }
        while (Param > TheDomain.LastParameter())
        {
            Param -= PIpPI;
        }
        return (Param);
    }

    //=====================================================================
    //====   C l a s s e     I n t e r v a l      !! Specifique !! ========
    //=====================================================================
    /// <summary>An interval of a line parameter, possibly unbounded on either side.</summary>
    /// <remarks>OCCT's default constructor gives a null interval; <c>default(Interval)</c> doesn't: use <c>new()</c>.</remarks>
    internal struct Interval
    {
        public double Binf;
        public double Bsup;
        public bool HasFirstBound;
        public bool HasLastBound;
        public bool IsNull;

        public Interval()
        {
            Binf = 0.0;
            Bsup = 0.0;
            HasFirstBound = false;
            HasLastBound = false;
            IsNull = true;
        }

        public Interval(double a, double b)
        {
            HasFirstBound = HasLastBound = true;
            if (a < b)
            {
                Binf = a;
                Bsup = b;
            }
            else
            {
                Binf = b;
                Bsup = a;
            }
            IsNull = false;
        }

        public Interval(in IntRes2d_Domain Domain)
        {
            Binf = 0.0;
            Bsup = 0.0;
            IsNull = false;
            if (Domain.HasFirstPoint())
            {
                HasFirstBound = true;
                Binf = Domain.FirstParameter() - Domain.FirstTolerance();
            }
            else
            {
                HasFirstBound = false;
            }
            if (Domain.HasLastPoint())
            {
                HasLastBound = true;
                Bsup = Domain.LastParameter() + Domain.LastTolerance();
            }
            else
            {
                HasLastBound = false;
            }
        }

        public Interval(double a, bool hf, double b, bool hl)
        {
            Binf = a;
            Bsup = b;
            IsNull = false;
            HasFirstBound = hf;
            HasLastBound = hl;
        }

        public readonly double Length() => ((IsNull) ? -1.0 : Math.Abs(Bsup - Binf));

        public readonly Interval IntersectionWithBounded(in Interval Inter)
        {
            if (IsNull || Inter.IsNull)
            {
                Interval PourSGI = new();
                return (PourSGI);
            }
            if (!(HasFirstBound || HasLastBound))
            {
                return (new Interval(Inter.Binf, Inter.Bsup));
            }
            double a, b;
            if (HasFirstBound)
            {
                if (Inter.Bsup < Binf)
                {
                    Interval PourSGI = new();
                    return (PourSGI);
                }
                a = (Inter.Binf < Binf) ? Binf : Inter.Binf;
            }
            else
            {
                a = Inter.Binf;
            }

            if (HasLastBound)
            {
                if (Inter.Binf > Bsup)
                {
                    Interval PourSGI = new();
                    return (PourSGI);
                }
                b = (Inter.Bsup > Bsup) ? Bsup : Inter.Bsup;
            }
            else
            {
                b = Inter.Bsup;
            }
            return (new Interval(a, b));
        }
    }

    //======================================================================
    //==  C L A S S E    P E R I O D I C    I N T E R V A L  (Specifique)
    //======================================================================
    /// <summary>An interval of an angle, normalised to start in [0, 2 PI].</summary>
    /// <remarks>OCCT's default constructor gives a null interval; <c>default(PeriodicInterval)</c> doesn't: use <c>new()</c>.</remarks>
    internal struct PeriodicInterval
    {
        public double Binf;
        public double Bsup;
        public bool isnull;

        public void SetNull()
        {
            isnull = true;
        }

        public readonly bool IsNull() => (isnull);

        public void Complement()
        {
            if (!isnull)
            {
                double t = Binf;
                Binf = Bsup;
                Bsup = t + PIpPI;
                if (Binf > PIpPI)
                {
                    Binf -= PIpPI;
                    Bsup -= PIpPI;
                }
            }
        }

        public readonly double Length() => ((isnull) ? -100.0 : Math.Abs(Bsup - Binf));

        public PeriodicInterval(in IntRes2d_Domain Domain)
        {
            isnull = false;
            if (Domain.HasFirstPoint())
            {
                Binf = Domain.FirstParameter();
            }
            else
            {
                Binf = -1.0;
            }
            if (Domain.HasLastPoint())
            {
                Bsup = Domain.LastParameter();
            }
            else
            {
                Bsup = 20.0;
            }
        }

        public PeriodicInterval()
        {
            isnull = true;
            Binf = Bsup = 0.0;
        }

        public PeriodicInterval(double a, double b)
        {
            isnull = false;
            Binf = a;
            Bsup = b;
            if ((b - a) < PIpPI)
            {
                this.Normalize();
            }
        }

        public void SetValues(double a, double b)
        {
            isnull = false;
            Binf = a;
            Bsup = b;
            if ((b - a) < PIpPI)
            {
                this.Normalize();
            }
        }

        public void Normalize()
        {
            if (!isnull)
            {
                while (Binf > PIpPI)
                {
                    Binf -= PIpPI;
                }
                while (Binf < 0.0)
                {
                    Binf += PIpPI;
                }
                while (Bsup < Binf)
                {
                    Bsup += PIpPI;
                }
                while (Bsup >= (Binf + PIpPI))
                {
                    Bsup -= PIpPI;
                }
            }
        }

        /// <summary>
        /// The first intersection with <paramref name="PInter"/>, which OCCT shifts by periods in place (hence
        /// <c>ref</c>: callers see the shifted interval).
        /// </summary>
        public readonly PeriodicInterval FirstIntersection(ref PeriodicInterval PInter)
        {
            double a, b;
            if (PInter.isnull || isnull)
            {
                PeriodicInterval PourSGI = new();
                return (PourSGI);
            }
            else
            {
                if (Length() >= PIpPI)
                {
                    return (new PeriodicInterval(PInter.Binf, PInter.Bsup));
                }
                if (PInter.Length() >= PIpPI)
                {
                    return (new PeriodicInterval(Binf, Bsup));
                }
                if (PInter.Bsup <= Binf)
                {
                    while (PInter.Binf <= Binf && PInter.Bsup <= Binf)
                    {
                        PInter.Binf += PIpPI;
                        PInter.Bsup += PIpPI;
                    }
                }
                if (PInter.Binf >= Bsup)
                {
                    while (PInter.Binf >= Bsup && PInter.Bsup >= Bsup)
                    {
                        PInter.Binf -= PIpPI;
                        PInter.Bsup -= PIpPI;
                    }
                }
                if ((PInter.Bsup < Binf) || (PInter.Binf > Bsup))
                {
                    PeriodicInterval PourSGI = new();
                    return (PourSGI);
                }
            }

            a = (PInter.Binf > Binf) ? PInter.Binf : Binf;
            b = (PInter.Bsup < Bsup) ? PInter.Bsup : Bsup;

            return (new PeriodicInterval(a, b));
        }

        /// <summary>The second intersection with <paramref name="PInter"/> (shifted by a period).</summary>
        public readonly PeriodicInterval SecondIntersection(ref PeriodicInterval PInter)
        {
            double a, b;

            if (PInter.isnull || isnull || this.Length() >= PIpPI || PInter.Length() >= PIpPI)
            {
                PeriodicInterval PourSGI = new();
                return (PourSGI);
            }

            double PInter_inf = PInter.Binf + PIpPI;
            double PInter_sup = PInter.Bsup + PIpPI;
            if (PInter_inf > Bsup)
            {
                PInter_inf = PInter.Binf - PIpPI;
                PInter_sup = PInter.Bsup - PIpPI;
            }
            if ((PInter_sup < Binf) || (PInter_inf > Bsup))
            {
                PeriodicInterval PourSGI = new();
                return (PourSGI);
            }
            else
            {
                a = (PInter_inf > Binf) ? PInter_inf : Binf;
                b = (PInter_sup < Bsup) ? PInter_sup : Bsup;
            }
            return (new PeriodicInterval(a, b));
        }
    }
}
