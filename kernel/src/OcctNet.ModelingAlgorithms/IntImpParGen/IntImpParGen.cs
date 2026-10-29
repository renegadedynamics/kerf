// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;

namespace Occt;

/// <summary>
/// Tools of the generic algorithm intersecting implicit curves with bounded parametric curves
/// (<see cref="IntImpParGen_Intersector{ImpTool, ParCurve, ParTool, ProjectOnPCurveTool, MyImpParTool}"/>), also used
/// by the package IntCurve.
/// </summary>
public static class IntImpParGen
{
    private const double TOLERANCE_ANGULAIRE = 0.00000001;
    private const double DERIVEE_PREMIERE_NULLE = 0.000000000001;

    /// <summary>
    /// The transitions at an intersection point, from the tangents and normals of both curves. A tangent of
    /// (nearly) zero length is replaced by the normal, which <paramref name="Tan1"/> and <paramref name="Tan2"/>
    /// then return. The tolerance is not used.
    /// </summary>
    public static void DetermineTransition(IntRes2d_Position Pos1,
                                           ref gp_Vec2d Tan1,
                                           in gp_Vec2d Norm1,
                                           ref IntRes2d_Transition T1,
                                           IntRes2d_Position Pos2,
                                           ref gp_Vec2d Tan2,
                                           in gp_Vec2d Norm2,
                                           ref IntRes2d_Transition T2,
                                           double Tol)
    {
        _ = Tol;
        bool courbure1 = true;
        bool courbure2 = true;
        bool decide = true;

        T1.SetPosition(Pos1);
        T2.SetPosition(Pos2);

        if (Tan1.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
        {
            Tan1 = Norm1;
            courbure1 = false;
            if (Tan1.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
            { // transition undecided
                decide = false;
            }
        }

        if (Tan2.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
        {
            Tan2 = Norm2;
            courbure2 = false;
            if (Tan2.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
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
                    gp_Vec2d Norm = new();
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

                    if (Math.Abs(Val1 - Val2) <= TOLERANCE_ANGULAIRE)
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

    /// <summary>
    /// The transitions at an intersection point from the tangents only. Returns false, setting only the positions,
    /// when a tangent is (nearly) zero or the curves are tangent. The tolerance is not used.
    /// </summary>
    public static bool DetermineTransition(IntRes2d_Position Pos1,
                                           ref gp_Vec2d Tan1,
                                           ref IntRes2d_Transition T1,
                                           IntRes2d_Position Pos2,
                                           ref gp_Vec2d Tan2,
                                           ref IntRes2d_Transition T2,
                                           double Tol)
    {
        _ = Tol;
        T1.SetPosition(Pos1);
        T2.SetPosition(Pos2);

        double Tan1Magnitude = Tan1.Magnitude();
        if (Tan1Magnitude <= DERIVEE_PREMIERE_NULLE)
        {
            return (false);
        }

        double Tan2Magnitude = Tan2.Magnitude();
        if (Tan2Magnitude <= DERIVEE_PREMIERE_NULLE)
        {
            return (false);
        }

        double sgn = Tan1.Crossed(Tan2);
        double norm = Tan1Magnitude * Tan2Magnitude;

        if (Math.Abs(sgn) <= TOLERANCE_ANGULAIRE * norm)
        { // Transition TOUCH #########
            return (false);
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

        return (true);
    }

    /// <summary>
    /// The position (Head, Middle or End) on the domain of the point <paramref name="Pnt1"/> of parameter
    /// <paramref name="Param1"/>: Head or End when it is within the bound's tolerance (the closer bound in
    /// parameter when it is within both).
    /// </summary>
    public static void DeterminePosition(out IntRes2d_Position Pos1, in IntRes2d_Domain TheDomain, in gp_Pnt2d Pnt1,
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
    /// On a closed domain, <paramref name="Param"/> shifted by periods towards the domain; otherwise
    /// <paramref name="Param"/> unchanged.
    /// </summary>
    public static double NormalizeOnDomain(double Param, in IntRes2d_Domain TheDomain)
    {
        double modParam = Param;
        if (TheDomain.IsClosed())
        {
            double Periode, t;
            TheDomain.EquivalentParameters(out t, out Periode);
            Periode -= t;
            while (modParam < TheDomain.FirstParameter() && modParam + Periode < TheDomain.LastParameter())
            {
                modParam += Periode;
            }

            while (modParam > TheDomain.LastParameter() && modParam - Periode > TheDomain.FirstParameter())
            {
                modParam -= Periode;
            }
        }

        return (modParam);
    }
}
