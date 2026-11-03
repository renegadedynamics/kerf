// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen_Tool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;

namespace Occt;

/// <summary>
/// The free functions of IntImpParGen_Tool.hxx (used by IntCurve_IntConicConic): older variants of the
/// <see cref="IntImpParGen"/> statics. Ported code imports them with <c>using static Occt.IntImpParGen_Tool;</c>.
/// </summary>
internal static class IntImpParGen_Tool
{
    private const double TOLERANCE_ANGULAIRE = 0.00000001;

    /// <summary>
    /// On a closed domain, <paramref name="Param"/> moved up by periods until it is at least the first parameter,
    /// then down by one period if it is above the last; otherwise <paramref name="Param"/> unchanged.
    /// </summary>
    internal static double NormalizeOnDomain(double Param, in IntRes2d_Domain TheDomain)
    {
        double modParam = Param;
        if (TheDomain.IsClosed())
        {
            double Periode, t;
            TheDomain.EquivalentParameters(out t, out Periode);
            Periode -= t;
            if (TheDomain.HasFirstPoint())
            {
                while (modParam < TheDomain.FirstParameter())
                {
                    modParam += Periode;
                }
            }

            if (TheDomain.HasLastPoint())
            {
                if (modParam > TheDomain.LastParameter())
                {
                    modParam -= Periode;
                }
            }
        }

        return (modParam);
    }

    /// <summary>
    /// As <see cref="IntImpParGen.DeterminePosition"/>: the position (Head, Middle or End) on the domain of the point <paramref name="Pnt1"/> of parameter
    /// <paramref name="Param1"/>: Head or End when it is within the bound's tolerance (the closer bound in
    /// parameter when it is within both).
    /// </summary>
    internal static void Determine_Position(out IntRes2d_Position Pos1, in IntRes2d_Domain TheDomain, in Point2D Pnt1,
                                         double Param1)
    {
        Pos1 = IntRes2d_Middle;

        if (TheDomain.HasFirstPoint())
        {
            if (Pnt1.Distance(TheDomain.FirstPoint()) <= TheDomain.FirstTolerance())
            {
                Pos1 = IntRes2d_Head;
            }
        }

        if (TheDomain.HasLastPoint())
        {
            if (Pnt1.Distance(TheDomain.LastPoint()) <= TheDomain.LastTolerance())
            {
                if (Pos1 == IntRes2d_Head)
                {
                    if (Math.Abs(Param1 - TheDomain.LastParameter())
                        < Math.Abs(Param1 - TheDomain.FirstParameter()))
                    {
                        Pos1 = IntRes2d_End;
                    }
                }
                else
                {
                    Pos1 = IntRes2d_End;
                }
            }
        }
    }

    /// <summary>
    /// <see cref="IntImpParGen.DetermineTransition(IntRes2d_Position, ref Vector2D, in Vector2D, ref IntRes2d_Transition, IntRes2d_Position, ref Vector2D, in Vector2D, ref IntRes2d_Transition, double)"/>
    /// with gp::Resolution() on the magnitudes instead of 1e-12 on their squares, and without setting the
    /// positions first.
    /// </summary>
    internal static void Determine_Transition(IntRes2d_Position Pos1,
                                           ref Vector2D Tan1,
                                           in Vector2D Norm1,
                                           ref IntRes2d_Transition T1,
                                           IntRes2d_Position Pos2,
                                           ref Vector2D Tan2,
                                           in Vector2D Norm2,
                                           ref IntRes2d_Transition T2,
                                           double Tol)
    {
        _ = Tol;
        bool courbure1 = true;
        bool courbure2 = true;
        bool decide = true;

        if (Tan1.Magnitude() <= Constants.Resolution())
        {
            Tan1 = Norm1;
            courbure1 = false;
            if (Tan1.Magnitude() <= Constants.Resolution())
            { // transition undecided
                decide = false;
            }
        }

        if (Tan2.Magnitude() <= Constants.Resolution())
        {
            Tan2 = Norm2;
            courbure2 = false;
            if (Tan2.Magnitude() <= Constants.Resolution())
            { // transition undecided
                decide = false;
            }
        }

        if (!decide)
        {
            T1.SetValue(Pos1);
            T2.SetValue(Pos2);
        }
        else
        {
            double sgn = Tan1.Crossed(Tan2);
            double norm = Tan1.Magnitude() * Tan2.Magnitude();

            if (Math.Abs(sgn) <= TOLERANCE_ANGULAIRE * norm)
            { // Transition TOUCH #########
                bool opos = (Tan1.Dot(Tan2)) < 0;
                if (!(courbure1 || courbure2))
                {
                    T1.SetValue(true, Pos1, IntRes2d_Unknown, opos);
                    T2.SetValue(true, Pos2, IntRes2d_Unknown, opos);
                }
                else
                {
                    Vector2D Norm = new();
                    Norm.SetCoord(-Tan1.Y(), Tan1.X());

                    double Val1, Val2;
                    if (!courbure1)
                    {
                        Val1 = 0.0;
                    }
                    else
                    {
                        Val1 = Norm.Dot(Norm1);
                    }

                    if (!courbure2)
                    {
                        Val2 = 0.0;
                    }
                    else
                    {
                        Val2 = Norm.Dot(Norm2);
                    }

                    if (Math.Abs(Val1 - Val2) <= Constants.Resolution())
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
            }
            else if (sgn < 0)
            {
                T1.SetValue(false, Pos1, IntRes2d_In);
                T2.SetValue(false, Pos2, IntRes2d_Out);
            }
            else
            { // sgn>0
                T1.SetValue(false, Pos1, IntRes2d_Out);
                T2.SetValue(false, Pos2, IntRes2d_In);
            }
        }
    }
}
