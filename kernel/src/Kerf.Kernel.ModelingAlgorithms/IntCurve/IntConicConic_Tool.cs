// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic_Tool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The free functions and helper classes of OCCT's private header IntCurve_IntConicConic_Tool.hxx, used by
/// <see cref="IntConicConic"/>.
/// </summary>
/// <remarks>
/// OCCT's global classes Interval and PeriodicInterval are nested structs here (they are copied by value in C++),
/// imported with <c>using static</c>. Their mutators stay public within the internal types.
/// </remarks>
internal static class IntConicConic_Tool
{
    internal static readonly double PIpPI = M_PI + M_PI;

    private const double TOLERANCE_ANGULAIRE = 0.00000001;

    //======================================================================
    //===         R e s s o u r c e s        G e n e r a l e s           ===
    //======================================================================

    /// <summary>
    /// The transitions at an intersection point of a line or circle with another conic. OCCT normalises
    /// <paramref name="tan1"/> in place in the tangent case, hence <c>ref</c>.
    /// </summary>
    internal static void Determine_Transition_LC(Position2D pos1,
                                                 ref Vector2D tan1,
                                                 in Vector2D norm1,
                                                 ref Transition t1,
                                                 Position2D pos2,
                                                 ref Vector2D tan2,
                                                 in Vector2D norm2,
                                                 ref Transition t2,
                                                 double tol)
    {
        double sgn = tan1.Crossed(tan2);
        double norm = tan1.Magnitude * tan2.Magnitude;

        if (Math.Abs(sgn) <= TOLERANCE_ANGULAIRE * norm)
        { // Transition TOUCH #########
            bool opos = (tan1.Dot(tan2)) < 0;

            Vector2D Norm = new();
            //  Modified by Sergey KHROMOV - Thu Nov  2 17:57:15 2000 Begin
            tan1 = tan1.Normalized();
            //  Modified by Sergey KHROMOV - Thu Nov  2 17:57:16 2000 End
            Norm = Norm.WithCoord(-tan1.Y, tan1.X);

            double val1 = Norm.Dot(norm1);
            double val2 = Norm.Dot(norm2);

            if (Math.Abs(val1 - val2) <= Constants.Resolution())
            {
                t1.SetValue(true, pos1, Situation.Unknown, opos);
                t2.SetValue(true, pos2, Situation.Unknown, opos);
            }
            else if (val2 > val1)
            {
                t2.SetValue(true, pos2, Situation.Inside, opos);
                if (opos)
                {
                    t1.SetValue(true, pos1, Situation.Inside, opos);
                }
                else
                {
                    t1.SetValue(true, pos1, Situation.Outside, opos);
                }
            }
            else
            { // Val1 > Val2
                t2.SetValue(true, pos2, Situation.Outside, opos);
                if (opos)
                {
                    t1.SetValue(true, pos1, Situation.Outside, opos);
                }
                else
                {
                    t1.SetValue(true, pos1, Situation.Inside, opos);
                }
            }
        }
        else if (sgn < 0)
        {
            t1.SetValue(false, pos1, TypeTrans.In);
            t2.SetValue(false, pos2, TypeTrans.Out);
        }
        else
        {
            t1.SetValue(false, pos1, TypeTrans.Out);
            t2.SetValue(false, pos2, TypeTrans.In);
        }
    }

    //----------------------------------------------------------------------
    internal static double NormalizeOnCircleDomain(double _Param, in TwoDDomain theDomain)
    {
        double param = _Param;
        for (int aStep = 0; param < theDomain.FirstParameter(); StepLimit.Next(ref aStep))
        {
            param += PIpPI;
        }
        for (int aStep = 0; param > theDomain.LastParameter(); StepLimit.Next(ref aStep))
        {
            param -= PIpPI;
        }
        return (param);
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

        public Interval(in TwoDDomain domain)
        {
            Binf = 0.0;
            Bsup = 0.0;
            IsNull = false;
            if (domain.HasFirstPoint)
            {
                HasFirstBound = true;
                Binf = domain.FirstParameter() - domain.FirstTolerance();
            }
            else
            {
                HasFirstBound = false;
            }
            if (domain.HasLastPoint)
            {
                HasLastBound = true;
                Bsup = domain.LastParameter() + domain.LastTolerance();
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

        public readonly Interval IntersectionWithBounded(in Interval inter)
        {
            if (IsNull || inter.IsNull)
            {
                Interval pourSGI = new();
                return (pourSGI);
            }
            if (!(HasFirstBound || HasLastBound))
            {
                return (new Interval(inter.Binf, inter.Bsup));
            }
            double a, b;
            if (HasFirstBound)
            {
                if (inter.Bsup < Binf)
                {
                    Interval pourSGI = new();
                    return (pourSGI);
                }
                a = (inter.Binf < Binf) ? Binf : inter.Binf;
            }
            else
            {
                a = inter.Binf;
            }

            if (HasLastBound)
            {
                if (inter.Binf > Bsup)
                {
                    Interval pourSGI = new();
                    return (pourSGI);
                }
                b = (inter.Bsup > Bsup) ? Bsup : inter.Bsup;
            }
            else
            {
                b = inter.Bsup;
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
        public bool Isnull;

        public void SetNull()
        {
            Isnull = true;
        }

        public readonly bool IsNull => (Isnull);

        public void Complement()
        {
            if (!Isnull)
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

        public readonly double Length() => ((Isnull) ? -100.0 : Math.Abs(Bsup - Binf));

        public PeriodicInterval(in TwoDDomain domain)
        {
            Isnull = false;
            if (domain.HasFirstPoint)
            {
                Binf = domain.FirstParameter();
            }
            else
            {
                Binf = -1.0;
            }
            if (domain.HasLastPoint)
            {
                Bsup = domain.LastParameter();
            }
            else
            {
                Bsup = 20.0;
            }
        }

        public PeriodicInterval()
        {
            Isnull = true;
            Binf = Bsup = 0.0;
        }

        public PeriodicInterval(double a, double b)
        {
            Isnull = false;
            Binf = a;
            Bsup = b;
            if ((b - a) < PIpPI)
            {
                this.Normalize();
            }
        }

        public void SetValues(double a, double b)
        {
            Isnull = false;
            Binf = a;
            Bsup = b;
            if ((b - a) < PIpPI)
            {
                this.Normalize();
            }
        }

        public void Normalize()
        {
            if (!Isnull)
            {
                for (int aStep = 0; Binf > PIpPI; StepLimit.Next(ref aStep))
                {
                    Binf -= PIpPI;
                }
                for (int aStep = 0; Binf < 0.0; StepLimit.Next(ref aStep))
                {
                    Binf += PIpPI;
                }
                for (int aStep = 0; Bsup < Binf; StepLimit.Next(ref aStep))
                {
                    Bsup += PIpPI;
                }
                for (int aStep = 0; Bsup >= (Binf + PIpPI); StepLimit.Next(ref aStep))
                {
                    Bsup -= PIpPI;
                }
            }
        }

        /// <summary>
        /// The first intersection with <paramref name="pInter"/>, which OCCT shifts by periods in place (hence
        /// <c>ref</c>: callers see the shifted interval).
        /// </summary>
        public readonly PeriodicInterval FirstIntersection(ref PeriodicInterval pInter)
        {
            double a, b;
            if (pInter.Isnull || Isnull)
            {
                PeriodicInterval pourSGI = new();
                return (pourSGI);
            }
            else
            {
                if (Length() >= PIpPI)
                {
                    return (new PeriodicInterval(pInter.Binf, pInter.Bsup));
                }
                if (pInter.Length() >= PIpPI)
                {
                    return (new PeriodicInterval(Binf, Bsup));
                }
                if (pInter.Bsup <= Binf)
                {
                    for (int aStep = 0; pInter.Binf <= Binf && pInter.Bsup <= Binf; StepLimit.Next(ref aStep))
                    {
                        pInter.Binf += PIpPI;
                        pInter.Bsup += PIpPI;
                    }
                }
                if (pInter.Binf >= Bsup)
                {
                    for (int aStep = 0; pInter.Binf >= Bsup && pInter.Bsup >= Bsup; StepLimit.Next(ref aStep))
                    {
                        pInter.Binf -= PIpPI;
                        pInter.Bsup -= PIpPI;
                    }
                }
                if ((pInter.Bsup < Binf) || (pInter.Binf > Bsup))
                {
                    PeriodicInterval pourSGI = new();
                    return (pourSGI);
                }
            }

            a = (pInter.Binf > Binf) ? pInter.Binf : Binf;
            b = (pInter.Bsup < Bsup) ? pInter.Bsup : Bsup;

            return (new PeriodicInterval(a, b));
        }

        /// <summary>The second intersection with <paramref name="pInter"/> (shifted by a period).</summary>
        public readonly PeriodicInterval SecondIntersection(ref PeriodicInterval pInter)
        {
            double a, b;

            if (pInter.Isnull || Isnull || this.Length() >= PIpPI || pInter.Length() >= PIpPI)
            {
                PeriodicInterval pourSGI = new();
                return (pourSGI);
            }

            double PInter_inf = pInter.Binf + PIpPI;
            double PInter_sup = pInter.Bsup + PIpPI;
            if (PInter_inf > Bsup)
            {
                PInter_inf = pInter.Binf - PIpPI;
                PInter_sup = pInter.Bsup - PIpPI;
            }
            if ((PInter_sup < Binf) || (PInter_inf > Bsup))
            {
                PeriodicInterval pourSGI = new();
                return (pourSGI);
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
