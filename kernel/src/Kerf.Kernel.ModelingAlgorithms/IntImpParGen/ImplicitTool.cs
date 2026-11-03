// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen_Tool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// The free functions of IntImpParGen_Tool.hxx (used by IntCurve_IntConicConic): older variants of the
/// <see cref="IntImpParGen"/> statics. Ported code imports them with <c>using static Occt.IntImpParGen_Tool;</c>.
/// </summary>
internal static class ImplicitTool
{
    private const double TOLERANCE_ANGULAIRE = 0.00000001;

    /// <summary>
    /// On a closed domain, <paramref name="param"/> moved up by periods until it is at least the first parameter,
    /// then down by one period if it is above the last; otherwise <paramref name="param"/> unchanged.
    /// </summary>
    internal static double NormalizeOnDomain(double param, in TwoDDomain theDomain)
    {
        double modParam = param;
        if (theDomain.IsClosed)
        {
            double periode, t;
            theDomain.EquivalentParameters(out t, out periode);
            periode -= t;
            if (theDomain.HasFirstPoint)
            {
                while (modParam < theDomain.FirstParameter())
                {
                    modParam += periode;
                }
            }

            if (theDomain.HasLastPoint)
            {
                if (modParam > theDomain.LastParameter())
                {
                    modParam -= periode;
                }
            }
        }

        return (modParam);
    }

    /// <summary>
    /// As <see cref="IntImpParGen.DeterminePosition"/>: the position (Head, Middle or End) on the domain of the point <paramref name="pnt1"/> of parameter
    /// <paramref name="param1"/>: Head or End when it is within the bound's tolerance (the closer bound in
    /// parameter when it is within both).
    /// </summary>
    internal static void Determine_Position(out Position2D pos1, in TwoDDomain theDomain, in Point2D pnt1,
                                         double param1)
    {
        pos1 = Position2D.Middle;

        if (theDomain.HasFirstPoint)
        {
            if (pnt1.Distance(theDomain.FirstPoint()) <= theDomain.FirstTolerance())
            {
                pos1 = Position2D.Head;
            }
        }

        if (theDomain.HasLastPoint)
        {
            if (pnt1.Distance(theDomain.LastPoint()) <= theDomain.LastTolerance())
            {
                if (pos1 == Position2D.Head)
                {
                    if (Math.Abs(param1 - theDomain.LastParameter())
                        < Math.Abs(param1 - theDomain.FirstParameter()))
                    {
                        pos1 = Position2D.End;
                    }
                }
                else
                {
                    pos1 = Position2D.End;
                }
            }
        }
    }

    /// <summary>
    /// <see cref="IntImpParGen.DetermineTransition(Position2D, ref Vector2D, in Vector2D, ref Transition, Position2D, ref Vector2D, in Vector2D, ref Transition, double)"/>
    /// with gp::Resolution() on the magnitudes instead of 1e-12 on their squares, and without setting the
    /// positions first.
    /// </summary>
    internal static void Determine_Transition(Position2D pos1,
                                           ref Vector2D tan1,
                                           in Vector2D norm1,
                                           ref Transition t1,
                                           Position2D pos2,
                                           ref Vector2D tan2,
                                           in Vector2D norm2,
                                           ref Transition t2,
                                           double tol)
    {
        _ = tol;
        bool courbure1 = true;
        bool courbure2 = true;
        bool decide = true;

        if (tan1.Magnitude() <= Constants.Resolution())
        {
            tan1 = norm1;
            courbure1 = false;
            if (tan1.Magnitude() <= Constants.Resolution())
            { // transition undecided
                decide = false;
            }
        }

        if (tan2.Magnitude() <= Constants.Resolution())
        {
            tan2 = norm2;
            courbure2 = false;
            if (tan2.Magnitude() <= Constants.Resolution())
            { // transition undecided
                decide = false;
            }
        }

        if (!decide)
        {
            t1.SetValue(pos1);
            t2.SetValue(pos2);
        }
        else
        {
            double sgn = tan1.Crossed(tan2);
            double norm = tan1.Magnitude() * tan2.Magnitude();

            if (Math.Abs(sgn) <= TOLERANCE_ANGULAIRE * norm)
            { // Transition TOUCH #########
                bool opos = (tan1.Dot(tan2)) < 0;
                if (!(courbure1 || courbure2))
                {
                    t1.SetValue(true, pos1, Situation.Unknown, opos);
                    t2.SetValue(true, pos2, Situation.Unknown, opos);
                }
                else
                {
                    Vector2D Norm = new();
                    Norm = Norm.WithCoord(-tan1.Y(), tan1.X());

                    double val1, val2;
                    if (!courbure1)
                    {
                        val1 = 0.0;
                    }
                    else
                    {
                        val1 = Norm.Dot(norm1);
                    }

                    if (!courbure2)
                    {
                        val2 = 0.0;
                    }
                    else
                    {
                        val2 = Norm.Dot(norm2);
                    }

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
            }
            else if (sgn < 0)
            {
                t1.SetValue(false, pos1, TypeTrans.In);
                t2.SetValue(false, pos2, TypeTrans.Out);
            }
            else
            { // sgn>0
                t1.SetValue(false, pos1, TypeTrans.Out);
                t2.SetValue(false, pos2, TypeTrans.In);
            }
        }
    }
}
