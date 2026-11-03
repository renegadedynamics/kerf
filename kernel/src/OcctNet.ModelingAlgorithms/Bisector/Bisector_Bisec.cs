// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Bisec.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Bisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The bisecting line of two elements (curves or points), trimmed by a point P and contained in the sector defined
/// by two vectors V1, V2 and a sense: if Sense is positive the bisector is in the sector of -V1 and -V2 in the
/// indirect sense, else in the direct sense. Lines and circles give an analytic bisector (Bisector_BisecAna), other
/// curves Bisector_BisecPC or Bisector_BisecCC; Value() is the bisector trimmed at its start and end.
/// </summary>
/// <remarks>
/// Tolerance defines degenerate bisectors: a hyperbola with a radius, a parabola with a focal length or an ellipse
/// with a minor radius smaller than Tolerance is replaced by a line or half-line on one of its axes.
/// A working object of the medial axis (MAT2d_Tool2d keeps and re-trims its curve in place, as OCCT does): Value()
/// and ChangeValue() return the trimmed curve itself, not frozen; its mutators are internal.
/// </remarks>
public sealed class Bisector_Bisec
{
    private TrimmedCurve2D thebisector = null!;

    /// <summary>An empty bisector.</summary>
    public Bisector_Bisec()
    {
    }

    //===========================================================================
    //    calculate the bissectrice between two curves coming from a point.
    //
    //   afirstcurve   : \ curves between which the
    //   asecondcurve  : / bissectrice is calculated.
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to determine the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold starting from which the bisectrices are degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting line of the curves afirstcurve and asecondcurve from apoint. oncurve is true if apoint
    /// is common to the two curves.
    /// </summary>
    public void Perform(Curve2D afirstcurve, Curve2D asecondcurve, in Point2D apoint,
                        in Vector2D afirstvector, in Vector2D asecondvector, double adirection,
                        JoinKind ajointype, double tolerance, bool oncurve = true)
    {
        Type Type1 = afirstcurve.GetType();
        Type Type2 = asecondcurve.GetType();
        Bisector_Curve Bis;
        double UFirst, ULast;

        if (Type1 == typeof(TrimmedCurve2D))
        {
            Type1 = ((TrimmedCurve2D)afirstcurve).ChangeBasisCurve().GetType();
        }
        if (Type2 == typeof(TrimmedCurve2D))
        {
            Type2 = ((TrimmedCurve2D)asecondcurve).ChangeBasisCurve().GetType();
        }

        Curve2D afirstcurve1 = afirstcurve;
        Curve2D asecondcurve1 = asecondcurve;

        if (Type1 == typeof(BSplineCurve2D))
        {
            BSplineCurve2D aBS;
            if (afirstcurve.GetType() == typeof(TrimmedCurve2D))
            {
                aBS = (BSplineCurve2D)((TrimmedCurve2D)afirstcurve).ChangeBasisCurve();
            }
            else
            {
                aBS = (BSplineCurve2D)afirstcurve;
            }
            if (aBS.Degree() == 1 && aBS.PoleCount == 2)
            {
                if (aBS.Pole(1).Distance(aBS.Pole(2)) < 1.0e-4)
                {
                    afirstcurve1 = new MakeSegment2d(aBS.Pole(1), aBS.Pole(2)).Value();
                    Type1 = typeof(LineCurve2D);
                }
            }
        }

        if (Type2 == typeof(BSplineCurve2D))
        {
            BSplineCurve2D aBS;
            if (asecondcurve.GetType() == typeof(TrimmedCurve2D))
            {
                aBS = (BSplineCurve2D)((TrimmedCurve2D)asecondcurve).ChangeBasisCurve();
            }
            else
            {
                aBS = (BSplineCurve2D)asecondcurve;
            }
            if (aBS.Degree() == 1 && aBS.PoleCount == 2)
            {
                if (aBS.Pole(1).Distance(aBS.Pole(2)) < 1.0e-4)
                {
                    asecondcurve1 = new MakeSegment2d(aBS.Pole(1), aBS.Pole(2)).Value();
                    Type2 = typeof(LineCurve2D);
                }
            }
        }

        if ((Type1 == typeof(CircleCurve2D) || Type1 == typeof(LineCurve2D))
            && (Type2 == typeof(CircleCurve2D) || Type2 == typeof(LineCurve2D)))
        {
            //------------------------------------------------------------------
            // Analytic Bissectrice.
            //------------------------------------------------------------------
            Bisector_BisecAna BisAna = new();
            BisAna.Perform(afirstcurve1, asecondcurve1, apoint, afirstvector, asecondvector, adirection, ajointype,
                           tolerance, oncurve);
            UFirst = BisAna.ParameterOfStartPoint();
            ULast = BisAna.ParameterOfEndPoint();
            Bis = BisAna;
        }
        else
        {
            bool IsLine = false;

            if (oncurve)
            {
                Direction2D Fd = new(afirstvector);
                Direction2D Sd = new(asecondvector);
                // if (Fd.Dot(Sd) < Precision::Angular() - 1.) {
                // if (Fd.Dot(Sd) < 10*Precision::Angular() - 1.) //patch
                if (Fd.Dot(Sd) < Math.Sqrt(2.0 * global::Kerf.Engine.Foundation.Precision.Angular()) - 1.0)
                {
                    IsLine = true;
                }
            }
            if (IsLine)
            {
                //------------------------------------------------------------------
                // Half-Staight.
                //------------------------------------------------------------------
                Direction2D N = new(-adirection * afirstvector.Y(), adirection * afirstvector.X());
                LineCurve2D L = new(apoint, N);
                TrimmedCurve2D BisL = new(L, 0, global::Kerf.Engine.Foundation.Precision.Infinite());
                Bisector_BisecAna BisAna = new();
                BisAna.Init(BisL);
                UFirst = BisAna.ParameterOfStartPoint();
                ULast = BisAna.ParameterOfEndPoint();
                Bis = BisAna;
            }
            else
            {
                //-------------------------------------------------------------------
                // Bissectrice algo
                //-------------------------------------------------------------------
                Bisector_BisecCC BisCC = new();
                BisCC.Perform(asecondcurve1, afirstcurve1, adirection, adirection, apoint);

                if (BisCC.IsEmpty())
                {
                    // bissectrice is empty. a point is projected at the end of the guide curve.
                    // Construction of a false bissectrice.
                    //  modified by NIZHNY-EAP Mon Feb 21 12:00:13 2000 ___BEGIN___
                    Point2D aP1 = afirstcurve1.Value(afirstcurve1.LastParameter());
                    Point2D aP2 = asecondcurve1.Value(asecondcurve1.FirstParameter());
                    Point2D aPm = new(.5 * (aP1.XY() + aP2.XY()));
                    double Nx, Ny;
                    if (aPm.Distance(apoint) > 10.0 * global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        Nx = apoint.X() - aPm.X();
                        Ny = apoint.Y() - aPm.Y();
                        if (adirection < 0)
                        {
                            Nx = -Nx;
                            Ny = -Ny;
                        }
                    }
                    else
                    {
                        Direction2D dir1 = new(afirstvector), dir2 = new(asecondvector);
                        Nx = -dir1.X() - dir2.X();
                        Ny = -dir1.Y() - dir2.Y();
                        if (Math.Abs(Nx) <= Constants.Resolution() && Math.Abs(Ny) <= Constants.Resolution())
                        {
                            Nx = -afirstvector.Y();
                            Ny = afirstvector.X();
                        }
                    }
                    Direction2D N = new(adirection * Nx, adirection * Ny);
                    //  modified by NIZHNY-EAP Mon Feb 21 12:00:19 2000 ___END___

                    LineCurve2D L = new(apoint, N);
                    TrimmedCurve2D BisL = new(L, 0, global::Kerf.Engine.Foundation.Precision.Infinite());
                    Bisector_BisecAna BisAna = new();
                    BisAna.Init(BisL);
                    UFirst = BisAna.ParameterOfStartPoint();
                    ULast = BisAna.ParameterOfEndPoint();
                    Bis = BisAna;
                }
                else
                {
                    UFirst = BisCC.FirstParameter();
                    ULast = BisCC.LastParameter();
                    Bis = BisCC;
                    ReplaceByLineIfIsToSmall(ref Bis, ref UFirst, ref ULast);
                }
            }
        }
        UFirst = Max(UFirst, Bis.FirstParameter());
        ULast = Min(ULast, Bis.LastParameter());
        thebisector = new TrimmedCurve2D(Bis, UFirst, ULast);
    }

    //===========================================================================
    //  calculate the bissectrice between a curve and a point starting in a point.
    //
    //   afirstcurve   : \ curve and point the bissectrice between which is calculated.
    //   asecondpoint  : /
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to find the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold starting from which the bisectrices are degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting line of the curve afirstcurve and the point asecondpoint from apoint. oncurve is true if
    /// apoint is the point asecondpoint.
    /// </summary>
    public void Perform(Curve2D afirstcurve, Point2 asecondpoint, in Point2D apoint,
                        in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance,
                        bool oncurve = true)
    {
        Bisector_Curve Bis;
        Type Type1 = afirstcurve.GetType();
        double UFirst, ULast;

        if (Type1 == typeof(TrimmedCurve2D))
        {
            Type1 = ((TrimmedCurve2D)afirstcurve).ChangeBasisCurve().GetType();
        }

        if (Type1 == typeof(CircleCurve2D) || Type1 == typeof(LineCurve2D))
        {
            //------------------------------------------------------------------
            // Analytic Bissectrice.
            //------------------------------------------------------------------
            Bisector_BisecAna BisAna = new();
            BisAna.Perform(afirstcurve, asecondpoint, apoint, afirstvector, asecondvector, adirection, tolerance,
                           oncurve);
            UFirst = BisAna.ParameterOfStartPoint();
            ULast = BisAna.ParameterOfEndPoint();
            Bis = BisAna;
        }
        else
        {
            bool IsLine = false;
            double RC = global::Kerf.Engine.Foundation.Precision.Infinite();

            if (oncurve)
            {
                if (Bisector.IsConvex(afirstcurve, adirection) || IsMaxRC(afirstcurve, afirstcurve.LastParameter(), ref RC))
                {
                    IsLine = true;
                }
            }
            if (IsLine)
            {
                //------------------------------------------------------------------
                // Half-Right.
                //------------------------------------------------------------------
                Direction2D N = new(-adirection * afirstvector.Y(), adirection * afirstvector.X());
                LineCurve2D L = new(apoint, N);
                TrimmedCurve2D BisL = new(L, 0, RC);
                Bisector_BisecAna BisAna = new();
                BisAna.Init(BisL);
                UFirst = BisAna.ParameterOfStartPoint();
                ULast = BisAna.ParameterOfEndPoint();
                Bis = BisAna;
            }
            else
            {
                //-------------------------------------------------------------------
                // Bissectrice algo
                //-------------------------------------------------------------------
                Bisector_BisecPC BisPC = new();
                Curve2D afirstcurvereverse = afirstcurve.Reversed();

                BisPC.Perform(afirstcurvereverse, asecondpoint.Pnt2d(), -adirection);
                //  Modified by Sergey KHROMOV - Thu Feb 21 16:49:54 2002 Begin
                if (BisPC.IsEmpty())
                {
                    Direction2D dir1 = new(afirstvector), dir2 = new(asecondvector);
                    double Nx = -dir1.X() - dir2.X(), Ny = -dir1.Y() - dir2.Y();
                    if (Math.Abs(Nx) <= Constants.Resolution() && Math.Abs(Ny) <= Constants.Resolution())
                    {
                        Nx = -afirstvector.Y();
                        Ny = afirstvector.X();
                    }
                    Direction2D N = new(adirection * Nx, adirection * Ny);
                    LineCurve2D L = new(apoint, N);
                    TrimmedCurve2D BisL = new(L, 0, RC);
                    Bisector_BisecAna BisAna = new();
                    BisAna.Init(BisL);
                    UFirst = BisAna.ParameterOfStartPoint();
                    ULast = BisAna.ParameterOfEndPoint();
                    Bis = BisAna;
                }
                else
                {
                    //  Modified by Sergey KHROMOV - Wed Mar  6 17:01:08 2002 End
                    UFirst = BisPC.Parameter(apoint);
                    ULast = BisPC.LastParameter();
                    if (UFirst >= ULast)
                    {
                        // Extrapolate by line
                        Vector2D V = new(BisPC.Value(BisPC.FirstParameter()), BisPC.Value(ULast));
                        Direction2D N = new(V);
                        LineCurve2D L = new(apoint, N);
                        TrimmedCurve2D BisL = new(L, 0, RC);
                        Bisector_BisecAna BisAna = new();
                        BisAna.Init(BisL);
                        UFirst = BisAna.ParameterOfStartPoint();
                        ULast = BisAna.ParameterOfEndPoint();
                        Bis = BisAna;
                    }
                    else
                    {
                        Bis = BisPC;
                    }
                }
            }
        }
        if (UFirst < Bis.FirstParameter())
        {
            UFirst = Bis.FirstParameter();
        }
        if (ULast > Bis.LastParameter())
        {
            ULast = Bis.LastParameter();
        }
        thebisector = new TrimmedCurve2D(Bis, UFirst, ULast);
    }

    //===========================================================================
    //   calculate the bissectrice between a curve and a point starting in a point.
    //
    //   afirstpoint   : \ curve and point the bissectrice between which is calculated.
    //   asecondcurve  : /
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to find the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold starting from which the bisectrices are degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting line of the point afirstpoint and the curve asecondcurve from apoint. oncurve is true
    /// if apoint is the point afirstpoint.
    /// </summary>
    public void Perform(Point2 afirstpoint, Curve2D asecondcurve, in Point2D apoint,
                        in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance,
                        bool oncurve = true)
    {
        Bisector_Curve Bis;
        Type Type1 = asecondcurve.GetType();
        double UFirst, ULast;

        if (Type1 == typeof(TrimmedCurve2D))
        {
            Type1 = ((TrimmedCurve2D)asecondcurve).ChangeBasisCurve().GetType();
        }

        if (Type1 == typeof(CircleCurve2D) || Type1 == typeof(LineCurve2D))
        {
            //------------------------------------------------------------------
            // Analytic Bissectrice.
            //------------------------------------------------------------------
            Bisector_BisecAna BisAna = new();
            BisAna.Perform(afirstpoint, asecondcurve, apoint, afirstvector, asecondvector, adirection, tolerance,
                           oncurve);
            UFirst = BisAna.ParameterOfStartPoint();
            ULast = BisAna.ParameterOfEndPoint();
            Bis = BisAna;
        }
        else
        {
            bool IsLine = false;
            double RC = global::Kerf.Engine.Foundation.Precision.Infinite();

            if (oncurve)
            {
                if (Bisector.IsConvex(asecondcurve, adirection)
                    || IsMaxRC(asecondcurve, asecondcurve.FirstParameter(), ref RC))
                {
                    IsLine = true;
                }
            }
            if (IsLine)
            {
                //------------------------------------------------------------------
                // Half-Staight.
                //------------------------------------------------------------------
                Direction2D N = new(-adirection * afirstvector.Y(), adirection * afirstvector.X());
                LineCurve2D L = new(apoint, N);
                TrimmedCurve2D BisL = new(L, 0, RC);
                Bisector_BisecAna BisAna = new();
                BisAna.Init(BisL);
                UFirst = BisAna.ParameterOfStartPoint();
                ULast = BisAna.ParameterOfEndPoint();
                Bis = BisAna;
            }
            else
            {
                //-------------------------------------------------------------------
                // Bissectrice algo
                //-------------------------------------------------------------------
                Bisector_BisecPC BisPC = new();
                BisPC.Perform(asecondcurve, afirstpoint.Pnt2d(), adirection);
                //  Modified by Sergey KHROMOV - Thu Feb 21 16:49:54 2002 Begin
                if (BisPC.IsEmpty())
                {
                    Direction2D dir1 = new(afirstvector), dir2 = new(asecondvector);
                    double Nx = -dir1.X() - dir2.X(), Ny = -dir1.Y() - dir2.Y();
                    if (Math.Abs(Nx) <= Constants.Resolution() && Math.Abs(Ny) <= Constants.Resolution())
                    {
                        Nx = -afirstvector.Y();
                        Ny = afirstvector.X();
                    }
                    Direction2D N = new(adirection * Nx, adirection * Ny);
                    LineCurve2D L = new(apoint, N);
                    TrimmedCurve2D BisL = new(L, 0, RC);
                    Bisector_BisecAna BisAna = new();
                    BisAna.Init(BisL);
                    UFirst = BisAna.ParameterOfStartPoint();
                    ULast = BisAna.ParameterOfEndPoint();
                    Bis = BisAna;
                }
                else
                {
                    //  Modified by Sergey KHROMOV - Thu Feb 21 16:49:58 2002 End
                    UFirst = BisPC.Parameter(apoint);
                    ULast = BisPC.LastParameter();
                    if (UFirst >= ULast)
                    {
                        // Extrapolate by line
                        Vector2D V = new(BisPC.Value(BisPC.FirstParameter()), BisPC.Value(ULast));
                        Direction2D N = new(V);
                        LineCurve2D L = new(apoint, N);
                        TrimmedCurve2D BisL = new(L, 0, RC);
                        Bisector_BisecAna BisAna = new();
                        BisAna.Init(BisL);
                        UFirst = BisAna.ParameterOfStartPoint();
                        ULast = BisAna.ParameterOfEndPoint();
                        Bis = BisAna;
                    }
                    else
                    {
                        Bis = BisPC;
                    }
                }
            }
        }

        UFirst = Max(UFirst, Bis.FirstParameter());
        ULast = Min(ULast, Bis.LastParameter());
        thebisector = new TrimmedCurve2D(Bis, UFirst, ULast);
    }

    //===========================================================================
    //        calculate the bissectrice between two points starting in a point.
    //
    //   afirstpoint   : \ curves the bissectrice between which should be
    //   asecondpoint  : / calculated.
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to find the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //===========================================================================
    /// <summary>Computes the bisecting line of the points afirstpoint and asecondpoint from apoint.</summary>
    /// <exception cref="GccAna_NoSolution">The points coincide.</exception>
    public void Perform(Point2 afirstpoint, Point2 asecondpoint, in Point2D apoint,
                        in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance = 0.0,
                        bool oncurve = true)
    {
        Bisector_BisecAna Bis = new();
        Bis.Perform(afirstpoint, asecondpoint, apoint, afirstvector, asecondvector, adirection, tolerance, oncurve);
        thebisector = new TrimmedCurve2D(Bis, Bis.ParameterOfStartPoint(), Bis.ParameterOfEndPoint());
    }

    /// <summary>The bisector: a Geom2d_TrimmedCurve whose basis is a Bisector_Curve.</summary>
    public TrimmedCurve2D Value() => thebisector;

    /// <summary>The bisector, for ported code that re-trims it in place (MAT2d_Tool2d).</summary>
    public TrimmedCurve2D ChangeValue() => thebisector;

    //=============================================================================
    //           replaced by a half-straight.
    //=============================================================================
    private static void ReplaceByLineIfIsToSmall(ref Bisector_Curve Bis, ref double UFirst, ref double ULast)
    {
        if (Math.Abs(ULast - UFirst) > 2.0 * global::Kerf.Engine.Foundation.Precision.PConfusion() * 10.0)
        {
            return; // patch
        }

        Point2D PF = Bis.Value(UFirst);
        Point2D PL = Bis.Value(ULast);

        if (PF.Distance(PL) > global::Kerf.Engine.Foundation.Precision.Confusion() * 10.0)
        {
            return;
        }

        Vector2D T1 = Bis.DN(UFirst, 1);

        LineCurve2D L = new(PF, new Direction2D(T1));
        TrimmedCurve2D BisL = new(L, 0, global::Kerf.Engine.Foundation.Precision.Infinite());
        Bisector_BisecAna BisAna = new();
        BisAna.Init(BisL);
        UFirst = BisAna.ParameterOfStartPoint();
        ULast = BisAna.ParameterOfEndPoint();
        Bis = BisAna;
    }

    private static bool IsMaxRC(Curve2D C, double U, ref double R)
    {
        double KF, KL;
        double US = C.FirstParameter();
        double UL = C.LastParameter();
        double Norm2;

        C.D2(US, out Point2D _, out Vector2D D1, out Vector2D D2);
        Norm2 = D1.SquareMagnitude();
        if (Norm2 < Constants.Resolution())
        {
            KF = 0.0;
        }
        else
        {
            KF = Math.Abs(D1 ^ D2) / (Norm2 * Math.Sqrt(Norm2));
        }

        C.D2(UL, out Point2D _, out D1, out D2);
        Norm2 = D1.SquareMagnitude();
        if (Norm2 < Constants.Resolution())
        {
            KL = 0.0;
        }
        else
        {
            KL = Math.Abs(D1 ^ D2) / (Norm2 * Math.Sqrt(Norm2));
        }

        bool IsMax = false;
        if (U == UL)
        {
            if (KL < KF)
            {
                if (KL == 0.0)
                {
                    R = global::Kerf.Engine.Foundation.Precision.Infinite();
                }
                else
                {
                    R = 1 / KL;
                }
                IsMax = true;
            }
        }
        else
        {
            if (KF < KL)
            {
                if (KF == 0.0)
                {
                    R = global::Kerf.Engine.Foundation.Precision.Infinite();
                }
                else
                {
                    R = 1 / KF;
                }
                IsMax = true;
            }
        }
        return IsMax;
    }
}
