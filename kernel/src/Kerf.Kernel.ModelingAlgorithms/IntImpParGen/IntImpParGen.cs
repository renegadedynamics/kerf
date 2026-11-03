// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// Tools of the generic algorithm intersecting implicit curves with bounded parametric curves
/// (<see cref="Intersector{ImpTool, ParCurve, ParTool, ProjectOnPCurveTool, MyImpParTool}"/>), also used
/// by the package IntCurve.
/// </summary>
public static class IntImpParGen
{
    private const double TOLERANCE_ANGULAIRE = 0.00000001;
    private const double DERIVEE_PREMIERE_NULLE = 0.000000000001;

    /// <summary>
    /// The transitions at an intersection point, from the tangents and normals of both curves. A tangent of
    /// (nearly) zero length is replaced by the normal, which <paramref name="tan1"/> and <paramref name="tan2"/>
    /// then return. The tolerance is not used.
    /// </summary>
    public static void DetermineTransition(Position2D pos1,
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

        t1.SetPosition(pos1);
        t2.SetPosition(pos2);

        if (tan1.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
        {
            tan1 = norm1;
            courbure1 = false;
            if (tan1.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
            { // transition undecided
                decide = false;
            }
        }

        if (tan2.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
        {
            tan2 = norm2;
            courbure2 = false;
            if (tan2.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
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

                    if (Math.Abs(val1 - val2) <= TOLERANCE_ANGULAIRE)
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

    /// <summary>
    /// The transitions at an intersection point from the tangents only. Returns false, setting only the positions,
    /// when a tangent is (nearly) zero or the curves are tangent. The tolerance is not used.
    /// </summary>
    public static bool DetermineTransition(Position2D pos1,
                                           ref Vector2D tan1,
                                           ref Transition t1,
                                           Position2D pos2,
                                           ref Vector2D tan2,
                                           ref Transition t2,
                                           double tol)
    {
        _ = tol;
        t1.SetPosition(pos1);
        t2.SetPosition(pos2);

        double tan1Magnitude = tan1.Magnitude();
        if (tan1Magnitude <= DERIVEE_PREMIERE_NULLE)
        {
            return (false);
        }

        double tan2Magnitude = tan2.Magnitude();
        if (tan2Magnitude <= DERIVEE_PREMIERE_NULLE)
        {
            return (false);
        }

        double sgn = tan1.Crossed(tan2);
        double norm = tan1Magnitude * tan2Magnitude;

        if (Math.Abs(sgn) <= TOLERANCE_ANGULAIRE * norm)
        { // Transition TOUCH #########
            return (false);
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

        return (true);
    }

    /// <summary>
    /// The position (Head, Middle or End) on the domain of the point <paramref name="pnt1"/> of parameter
    /// <paramref name="param1"/>: Head or End when it is within the bound's tolerance (the closer bound in
    /// parameter when it is within both).
    /// </summary>
    public static void DeterminePosition(out Position2D pos1, in TwoDDomain theDomain, in Point2D pnt1,
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
    /// On a closed domain, <paramref name="param"/> shifted by periods towards the domain; otherwise
    /// <paramref name="param"/> unchanged.
    /// </summary>
    public static double NormalizeOnDomain(double param, in TwoDDomain theDomain)
    {
        double modParam = param;
        if (theDomain.IsClosed)
        {
            double periode, t;
            theDomain.EquivalentParameters(out t, out periode);
            periode -= t;
            while (modParam < theDomain.FirstParameter() && modParam + periode < theDomain.LastParameter())
            {
                modParam += periode;
            }

            while (modParam > theDomain.LastParameter() && modParam - periode > theDomain.FirstParameter())
            {
                modParam -= periode;
            }
        }

        return (modParam);
    }
}
