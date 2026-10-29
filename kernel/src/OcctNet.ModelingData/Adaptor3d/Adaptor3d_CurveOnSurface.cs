// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_CurveOnSurface.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_CurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// A curve defined by a 2D curve in the parametric space of a surface: the 3D curve
/// <c>S(C(t))</c>. Lines and circles on planes and the isoparametric lines of the elementary
/// surfaces are recognised as lines and circles.
/// </summary>
/// <remarks>
/// An adaptor is a working object (decision 101): <see cref="Load(Adaptor3d_Surface)"/> and
/// <see cref="Load(Adaptor2d_Curve2d)"/> are public, and the intervals are cached, so evaluating one adaptor from
/// several threads at once is not safe. The adaptor keeps references to the adaptors it is given, as OCCT does.
/// </remarks>
public class Adaptor3d_CurveOnSurface : Adaptor3d_Curve
{
    private Adaptor3d_Surface? mySurface;
    private Adaptor2d_Curve2d? myCurve;
    private GeomAbs_CurveType myType;
    private gp_Circ myCirc = new gp_Circ();
    private gp_Lin myLin = new gp_Lin();
    private Adaptor3d_Surface? myFirstSurf;
    private Adaptor3d_Surface? myLastSurf;
    private NCollection_HSequence<double>? myIntervals;
    private GeomAbs_Shape myIntCont;

    private static gp_Pnt to3d(in gp_Pln Pl, in gp_Pnt2d P)
    {
        return ElSLib.Value(P.X(), P.Y(), Pl);
    }

    private static gp_Vec to3d(in gp_Pln Pl, in gp_Vec2d V)
    {
        gp_Vec Vx = new gp_Vec(Pl.XAxis().Direction());
        gp_Vec Vy = new gp_Vec(Pl.YAxis().Direction());
        Vx.Multiply(V.X());
        Vy.Multiply(V.Y());
        Vx.Add(Vy);
        return Vx;
    }

    private static gp_Ax2 to3d(in gp_Pln Pl, in gp_Ax22d A)
    {
        gp_Pnt P = to3d(Pl, A.Location());
        gp_Vec VX = to3d(Pl, new gp_Vec2d(A.XAxis().Direction()));
        gp_Vec VY = to3d(Pl, new gp_Vec2d(A.YAxis().Direction()));
        return new gp_Ax2(P, new gp_Dir(VX.Crossed(VY)), new gp_Dir(VX));
    }

    private static gp_Circ to3d(in gp_Pln Pl, in gp_Circ2d C)
    {
        return new gp_Circ(to3d(Pl, C.Axis()), C.Radius());
    }

    private static gp_Elips to3d(in gp_Pln Pl, in gp_Elips2d E)
    {
        return new gp_Elips(to3d(Pl, E.Axis()), E.MajorRadius(), E.MinorRadius());
    }

    private static gp_Hypr to3d(in gp_Pln Pl, in gp_Hypr2d H)
    {
        return new gp_Hypr(to3d(Pl, H.Axis()), H.MajorRadius(), H.MinorRadius());
    }

    private static gp_Parab to3d(in gp_Pln Pl, in gp_Parab2d P)
    {
        return new gp_Parab(to3d(Pl, P.Axis()), P.Focal());
    }

    private static gp_Vec SetLinearForm(in gp_Vec2d DW,
                                        in gp_Vec2d D2W,
                                        in gp_Vec2d D3W,
                                        in gp_Vec D1U,
                                        in gp_Vec D1V,
                                        in gp_Vec D2U,
                                        in gp_Vec D2V,
                                        in gp_Vec D2UV,
                                        in gp_Vec D3U,
                                        in gp_Vec D3V,
                                        in gp_Vec D3UUV,
                                        in gp_Vec D3UVV)
    {
        // OCCT: this is not the third derivative of S(u(t), v(t)). D1U is weighted by DW.X() where the chain rule
        // has D3W.X() (first line), and D3UVV ends up with 5 u' v'^2 instead of 3 u' v'^2 (the second V32 line
        // repeats it). Ported as is; Adaptor3d_CurveOnSurfaceTests.EvalD3_OcctFormula_DiffersFromDerivative.
        gp_Vec V31 = new gp_Vec(), V32 = new gp_Vec(), V33 = new gp_Vec(), V34 = new gp_Vec(), V3 = new gp_Vec();

        V31.SetLinearForm(DW.X(), D1U, D2W.X() * DW.X(), D2U, D2W.X() * DW.Y(), D2UV);
        V31.SetLinearForm(D3W.Y(), D1V, D2W.Y() * DW.X(), D2UV, D2W.Y() * DW.Y(), D2V, V31);

        V32.SetLinearForm(DW.X() * DW.X() * DW.Y(), D3UUV, DW.X() * DW.Y() * DW.Y(), D3UVV);
        V32.SetLinearForm(D2W.X() * DW.Y() + DW.X() * D2W.Y(), D2UV, DW.X() * DW.Y() * DW.Y(), D3UVV, V32);
        V33.SetLinearForm(2 * D2W.X() * DW.X(), D2U, DW.X() * DW.X() * DW.X(), D3U, DW.X() * DW.X() * DW.Y(), D3UUV);

        V34.SetLinearForm(2 * D2W.Y() * DW.Y(), D2V, DW.Y() * DW.Y() * DW.X(), D3UVV, DW.Y() * DW.Y() * DW.Y(), D3V);
        V3.SetLinearForm(1, V31, 2, V32, 1, V33, V34);
        return V3;
    }

    private static void CompareBounds(ref gp_Pnt2d P1, ref gp_Pnt2d P2) // SVV
    {
        double Lx = P1.X(), Ly = P1.Y();
        double Rx = P2.X(), Ry = P2.Y();

        if (Lx > Rx)
        {
            P1.SetX(Rx);
            P2.SetX(Lx);
        }
        if (Ly > Ry)
        {
            P1.SetY(Ry);
            P2.SetY(Ly);
        }
    }

    private static void Hunt(NCollection_Array1<double> Arr, double Coord, ref int Iloc)
    {
        // Warning: Hunt is used to find number of knot which equals coordinate component,
        //        when coordinate component definitely equals a knot only.
        double Tol = Precision.PConfusion() / 10;
        int i = 1;
        while ((i <= Arr.Upper()) && (Math.Abs(Coord - Arr[i]) > Tol))
        {
            i++;
        }

        // OCCT: when Coord is no knot, i is Upper() + 1 here and Arr(i) reads past the array (release build; the
        // port throws IndexOutOfRangeException instead). The callers only pass knots.
        if (Math.Abs(Coord - Arr[i]) < Tol)
        {
            Iloc = i;
        }
        else if (Math.Abs(Coord - Arr[i]) > Tol)
        {
            throw new Standard_NotImplemented("Adaptor3d_CurveOnSurface:Hunt");
        }
    }

    private static void ReverseParam(double In1, double In2, out double Out1, out double Out2)
    {
        if (In1 > In2)
        {
            Out1 = In2;
            Out2 = In1;
        }
        else
        {
            Out1 = In1;
            Out2 = In2;
        }
    }

    private static void ReverseParam(int In1, int In2, out int Out1, out int Out2)
    {
        if (In1 > In2)
        {
            Out1 = In2;
            Out2 = In1;
        }
        else
        {
            Out1 = In1;
            Out2 = In2;
        }
    }

    private static void FindBounds(NCollection_Array1<double> Arr,
                                   double Coord,
                                   double Der,
                                   ref int Bound1,
                                   ref int Bound2,
                                   out bool DerNull)
    {
        int N = 0;
        double Tol = Precision.PConfusion() / 10;
        Hunt(Arr, Coord, ref N);
        DerNull = false;

        if (N == Bound1)
        {
            if (Math.Abs(Der) > Tol)
            {
                DerNull = false;
            }
            if (Math.Abs(Der) <= Tol)
            {
                DerNull = true;
            }
            Bound1 = N;
            Bound2 = N + 1;
            return;
        }
        if (N == Bound2)
        {
            if (Math.Abs(Der) > Tol)
            {
                DerNull = false;
            }
            if (Math.Abs(Der) <= Tol)
            {
                DerNull = true;
            }
            Bound1 = N - 1;
            Bound2 = N;
            return;
        }
        if ((N != Bound1) && (N != Bound2))
        {
            if (Math.Abs(Der) > Tol)
            {
                if (Der > 0)
                {
                    Bound1 = N;
                    Bound2 = N + 1;
                }
                else
                {
                    Bound1 = N - 1;
                    Bound2 = N;
                }
                DerNull = false;
            }
            if (Math.Abs(Der) <= Tol)
            {
                DerNull = true;
                Bound1 = N - 1;
                Bound2 = N + 1;
            }
        }
    }

    private static void Locate1Coord(int Index,
                                     in gp_Pnt2d UV,
                                     in gp_Vec2d DUV,
                                     Geom_BSplineCurve BSplC,
                                     ref gp_Pnt2d LeftBot,
                                     ref gp_Pnt2d RightTop)
    {
        double Comp1 = 0, DComp1 = 0, cur, f = 0.0, l = 0.0;
        double Tol = Precision.PConfusion() / 10;
        int i = 1, Bnd1, Bnd2;
        bool DIsNull = false;
        NCollection_Array1<double> Arr = BSplC.InternalKnots();

        if (Index == 1)
        {
            Comp1 = UV.X();
            DComp1 = DUV.X();
        }
        if (Index == 2)
        {
            Comp1 = UV.Y();
            DComp1 = DUV.Y();
        }

        int Lo = BSplC.FirstUKnotIndex(), Up = BSplC.LastUKnotIndex();

        i = Lo;
        while ((Math.Abs(BSplC.Knot(i) - Comp1) > Tol) && (i != Up))
        {
            i++;
        }
        cur = BSplC.Knot(i);

        if (Math.Abs(Comp1 - cur) <= Tol)
        {
            Bnd1 = Lo;
            Bnd2 = Up;
            FindBounds(Arr, cur, DComp1, ref Bnd1, ref Bnd2, out DIsNull);
            ReverseParam(Bnd1, Bnd2, out Bnd1, out Bnd2);

            if (!DIsNull)
            {
                if (Index == 1)
                {
                    LeftBot.SetX(BSplC.Knot(Bnd1));
                    RightTop.SetX(BSplC.Knot(Bnd2));
                }
                else if (Index == 2)
                {
                    LeftBot.SetY(BSplC.Knot(Bnd1));
                    RightTop.SetY(BSplC.Knot(Bnd2));
                }
            }
            else if (DIsNull)
            {
                if (Math.Abs(Comp1 - (f = BSplC.Knot(Lo))) <= Tol)
                {
                    if (Index == 1)
                    {
                        LeftBot.SetX(BSplC.Knot(Lo));
                        RightTop.SetX(BSplC.Knot(Lo + 1));
                    }
                    else if (Index == 2)
                    {
                        LeftBot.SetY(BSplC.Knot(Lo));
                        RightTop.SetY(BSplC.Knot(Lo + 1));
                    }
                }
                else if (Math.Abs(Comp1 - (l = BSplC.Knot(Up))) <= Tol)
                {
                    if (Index == 1)
                    {
                        LeftBot.SetX(BSplC.Knot(Up - 1));
                        RightTop.SetX(BSplC.Knot(Up));
                    }
                    else if (Index == 2)
                    {
                        LeftBot.SetY(BSplC.Knot(Up - 1));
                        RightTop.SetY(BSplC.Knot(Up));
                    }
                }
                else if (Index == 1)
                {
                    LeftBot.SetX(BSplC.Knot(Bnd1));
                    RightTop.SetX(BSplC.Knot(Bnd2));
                }
                else if (Index == 2)
                {
                    LeftBot.SetY(BSplC.Knot(Bnd1));
                    RightTop.SetY(BSplC.Knot(Bnd2));
                }
            }
        }
        else //*********if Coord != Knot
        {
            i = Lo;
            while (i < Up)
            {
                // skl 28.03.2002 for OCC233
                f = BSplC.Knot(i);
                l = BSplC.Knot(i + 1);
                if (f < Comp1 && l > Comp1)
                {
                    break;
                }
                i++;
            }
            ReverseParam(f, l, out f, out l);

            if (i != Up)
            {
                if (Math.Abs(DComp1) < Tol)
                {
                    if (Index == 1)
                    {
                        LeftBot.SetX(f);
                        RightTop.SetX(l);
                    }
                    else if (Index == 2)
                    {
                        LeftBot.SetY(f);
                        RightTop.SetY(l);
                    }
                }
                else if (Math.Abs(DComp1) > Tol)
                {
                    if (Index == 1)
                    {
                        if (DComp1 > 0)
                        {
                            LeftBot.SetX(Comp1);
                            RightTop.SetX(l);
                        }
                        else if (DComp1 < 0)
                        {
                            LeftBot.SetX(f);
                            RightTop.SetX(Comp1);
                        }
                    }
                    else if (Index == 2)
                    {
                        if (DComp1 > 0)
                        {
                            LeftBot.SetY(Comp1);
                            RightTop.SetY(l);
                        }
                        else if (DComp1 < 0)
                        {
                            LeftBot.SetY(f);
                            RightTop.SetY(Comp1);
                        }
                    }
                }
            }
            else if (i == Up)
            {
                if (Index == 1)
                {
                    LeftBot.SetX(Comp1);
                    RightTop.SetX(BSplC.Knot(i));
                }
                else if (Index == 2)
                {
                    LeftBot.SetY(Comp1);
                    RightTop.SetY(BSplC.Knot(i));
                }
            }
        }
    }

    private static void Locate1Coord(int Index,
                                     in gp_Pnt2d UV,
                                     in gp_Vec2d DUV,
                                     Geom_BSplineSurface BSplS,
                                     out bool DIsNull,
                                     ref gp_Pnt2d LeftBot,
                                     ref gp_Pnt2d RightTop)
    {
        double Comp1 = 0, DComp1 = 0;
        double Tol = Precision.PConfusion() / 10;
        int i = 1, Up = 0, Up1, Up2, Down = 0, Down1, Down2;
        double cur = 0.0;

        DIsNull = false;

        Up1 = BSplS.LastUKnotIndex();
        Down1 = BSplS.FirstUKnotIndex();
        Up2 = BSplS.LastVKnotIndex();
        Down2 = BSplS.FirstVKnotIndex();

        if (Index == 1)
        {
            i = Down1;
            Comp1 = UV.X();
            DComp1 = DUV.X();
            Up = Up1;
            Down = Down1;
            while ((Math.Abs(BSplS.UKnot(i) - Comp1) > Tol) && (i != Up1))
            {
                i++;
            }
            cur = BSplS.UKnot(i);
        }
        else if (Index == 2)
        {
            i = Down2;
            Comp1 = UV.Y();
            DComp1 = DUV.Y();
            Up = Up2;
            Down = Down2;
            while ((Math.Abs(BSplS.VKnot(i) - Comp1) > Tol) && (i != Up2))
            {
                i++;
            }
            cur = BSplS.VKnot(i);
        }

        if (Math.Abs(Comp1 - cur) <= Tol)
        {
            int Bnd1 = Down, Bnd2 = Up;
            if (Index == 1)
            {
                NCollection_Array1<double> Arr1 = BSplS.InternalUKnots();
                FindBounds(Arr1, cur, DUV.X(), ref Bnd1, ref Bnd2, out DIsNull);
            }
            else if (Index == 2)
            {
                NCollection_Array1<double> Arr2 = BSplS.InternalVKnots();
                FindBounds(Arr2, cur, DUV.Y(), ref Bnd1, ref Bnd2, out DIsNull);
            }

            ReverseParam(Bnd1, Bnd2, out Bnd1, out Bnd2);

            if (!DIsNull)
            {
                if (Index == 1)
                {
                    LeftBot.SetX(BSplS.UKnot(Bnd1));
                    RightTop.SetX(BSplS.UKnot(Bnd2));
                }
                else if (Index == 2)
                {
                    LeftBot.SetY(BSplS.VKnot(Bnd1));
                    RightTop.SetY(BSplS.VKnot(Bnd2));
                }
            }
        }
        else //*********if Coord != Knot
        {
            if ((Index == 1) && (Comp1 < BSplS.UKnot(Down)))
            {
                LeftBot.SetX(BSplS.UKnot(Down));
                RightTop.SetX(BSplS.UKnot(Down + 1));
                return;
            }
            else if ((Index == 2) && (Comp1 < BSplS.VKnot(Down)))
            {
                LeftBot.SetY(BSplS.VKnot(Down));
                RightTop.SetY(BSplS.VKnot(Down + 1));
                return;
            }
            else if ((Index == 1) && (Comp1 > BSplS.UKnot(Up)))
            {
                RightTop.SetX(BSplS.UKnot(Up - 1));
                LeftBot.SetX(BSplS.UKnot(Up));
                return;
            }
            else if ((Index == 2) && (Comp1 > BSplS.VKnot(Up)))
            {
                RightTop.SetY(BSplS.VKnot(Up - 1));
                LeftBot.SetY(BSplS.VKnot(Up));
                return;
            }
            else
            {
                double f = 0.0, l = 1.0;
                if (Index == 1)
                {
                    f = BSplS.UKnot(Down);
                    l = BSplS.UKnot(Up);
                }
                else if (Index == 2)
                {
                    f = BSplS.VKnot(Down);
                    l = BSplS.VKnot(Up);
                }
                i = Down;
                if ((!(Comp1 < f)) && (!(Comp1 > l)))
                {
                    if (Index == 1)
                    {
                        while ((((f = BSplS.UKnot(i)) >= Comp1) || ((l = BSplS.UKnot(i + 1)) <= Comp1)) && (i < Up))
                        {
                            i++;
                        }
                    }
                    else if (Index == 2)
                    {
                        while ((((f = BSplS.VKnot(i)) >= Comp1) || ((l = BSplS.VKnot(i + 1)) <= Comp1)) && (i < Up))
                        {
                            i++;
                        }
                    }
                }
                else
                {
                    ReverseParam(f, l, out f, out l);
                }

                if (i != Up)
                {
                    if (Math.Abs(DComp1) > Tol)
                    {
                        if (Index == 1)
                        {
                            if (DComp1 > 0)
                            {
                                LeftBot.SetX(Comp1);
                                RightTop.SetX(l);
                            }
                            else if (DComp1 < 0)
                            {
                                LeftBot.SetX(f);
                                RightTop.SetX(Comp1);
                            }
                        }
                        else if (Index == 2)
                        {
                            if (DComp1 > 0)
                            {
                                LeftBot.SetY(Comp1);
                                RightTop.SetY(l);
                            }
                            else if (DComp1 < 0)
                            {
                                LeftBot.SetY(f);
                                RightTop.SetY(Comp1);
                            }
                        }
                    }
                    else
                    {
                        if (Math.Abs(DComp1) < Tol)
                        {
                            if (Index == 1)
                            {
                                LeftBot.SetX(f);
                                RightTop.SetX(l);
                            }
                            else if (Index == 2)
                            {
                                LeftBot.SetY(f);
                                RightTop.SetY(l);
                            }
                        }
                    }
                }
                else if (i == Up)
                {
                    if (Index == 1)
                    {
                        LeftBot.SetX(Comp1);
                        RightTop.SetX(BSplS.UKnot(i));
                    }
                    else if (Index == 2)
                    {
                        LeftBot.SetY(Comp1);
                        RightTop.SetY(BSplS.VKnot(i));
                    }
                }
            }
        }
    }

    private static void Locate2Coord(int Index,
                                     in gp_Pnt2d UV,
                                     in gp_Vec2d DUV,
                                     double I1,
                                     double I2,
                                     ref gp_Pnt2d LeftBot,
                                     ref gp_Pnt2d RightTop)
    {
        double Tol = Precision.PConfusion() / 10;
        double Comp1 = 0, DComp1 = 0;
        if (Index == 1)
        {
            Comp1 = UV.X();
            DComp1 = DUV.X();
        }
        else if (Index == 2)
        {
            Comp1 = UV.Y();
            DComp1 = DUV.Y();
        }

        if ((Comp1 != I1) && (Comp1 != I2))
        {
            if (Math.Abs(DComp1) > Tol)
            {
                if (DComp1 < 0)
                {
                    if (Index == 1)
                    {
                        LeftBot.SetX(I1);
                        RightTop.SetX(Comp1);
                    }
                    if (Index == 2)
                    {
                        LeftBot.SetY(I1);
                        RightTop.SetY(Comp1);
                    }
                }
                else if (DComp1 > 0)
                {
                    if (Index == 1)
                    {
                        LeftBot.SetX(Comp1);
                        RightTop.SetX(I2);
                    }
                    if (Index == 2)
                    {
                        LeftBot.SetY(Comp1);
                        RightTop.SetY(I2);
                    }
                }
                else
                {
                    if (Index == 1)
                    {
                        LeftBot.SetX(I1);
                        RightTop.SetX(I2);
                    }
                    if (Index == 2)
                    {
                        LeftBot.SetY(I1);
                        RightTop.SetY(I2);
                    }
                }
            }
            else if (Math.Abs(DComp1) <= Tol)
            {
                if (Index == 1)
                {
                    LeftBot.SetX(I1);
                    RightTop.SetX(I2);
                }
                if (Index == 2)
                {
                    LeftBot.SetY(I1);
                    RightTop.SetY(I2);
                }
            }
        }
        else if (Math.Abs(Comp1 - I1) < Tol)
        {
            if (Index == 1)
            {
                LeftBot.SetX(I1);
                RightTop.SetX(I2);
            }
            if (Index == 2)
            {
                LeftBot.SetY(I1);
                RightTop.SetY(I2);
            }
        }
        else if (Math.Abs(Comp1 - I2) < Tol)
        {
            if (Index == 1)
            {
                LeftBot.SetX(I1);
                RightTop.SetX(I2);
            }
            if (Index == 2)
            {
                LeftBot.SetY(I1);
                RightTop.SetY(I2);
            }
        }
    }

    private static void Locate2Coord(int Index,
                                     in gp_Pnt2d UV,
                                     in gp_Vec2d DUV,
                                     Geom_BSplineSurface BSplS,
                                     NCollection_Array1<double> Arr,
                                     ref gp_Pnt2d LeftBot,
                                     ref gp_Pnt2d RightTop)
    {
        double Comp = 0, DComp = 0, Tmp1 = 0.0, Tmp2 = 0.0;
        double Tol = Precision.PConfusion() / 10;
        int N = 0, NUp = 0, NLo = 0;
        if (Index == 1)
        {
            Comp = UV.X();
            DComp = DUV.Y();
            NUp = BSplS.LastUKnotIndex();
            NLo = BSplS.FirstUKnotIndex();
        }
        if (Index == 2)
        {
            Comp = UV.Y();
            DComp = DUV.X();
            NUp = BSplS.LastVKnotIndex();
            NLo = BSplS.FirstVKnotIndex();
        }

        if ((DComp > 0) && (Math.Abs(DComp) > Tol))
        {
            Hunt(Arr, Comp, ref N);
            if (N >= NUp)
            {
                // limit case: Hunt() caught upper knot. Take the last span.
                N = NUp - 1;
            }
            if (Index == 1)
            {
                Tmp1 = BSplS.UKnot(N);
                Tmp2 = BSplS.UKnot(N + 1);
            }
            else if (Index == 2)
            {
                Tmp1 = BSplS.VKnot(N);
                Tmp2 = BSplS.VKnot(N + 1);
            }

            ReverseParam(Tmp1, Tmp2, out Tmp1, out Tmp2);

            if (Index == 1)
            {
                LeftBot.SetX(Tmp1);
                RightTop.SetX(Tmp2);
            }
            else if (Index == 2)
            {
                LeftBot.SetY(Tmp1);
                RightTop.SetY(Tmp2);
            }
        }
        else if ((DComp < 0) && (Math.Abs(DComp) > Tol))
        {
            Hunt(Arr, Comp, ref N);
            if (N <= NLo)
            {
                // limit case: Hunt() caught lower knot. Take the first span.
                N = NLo + 1;
            }
            if (Index == 1)
            {
                Tmp1 = BSplS.UKnot(N - 1);
                Tmp2 = BSplS.UKnot(N);
            }
            else if (Index == 2)
            {
                Tmp1 = BSplS.VKnot(N - 1);
                Tmp2 = BSplS.VKnot(N);
            }

            ReverseParam(Tmp1, Tmp2, out Tmp1, out Tmp2);

            if (Index == 1)
            {
                LeftBot.SetX(Tmp1);
                RightTop.SetX(Tmp2);
            }
            else if (Index == 2)
            {
                LeftBot.SetY(Tmp1);
                RightTop.SetY(Tmp2);
            }
        }
    }

    /// <summary>An empty adaptor; load a surface and a curve before use.</summary>
    public Adaptor3d_CurveOnSurface()
    {
        myType = GeomAbs_OtherCurve;
        myIntCont = GeomAbs_CN;
    }

    /// <summary>An adaptor on the surface S; load a curve before use.</summary>
    public Adaptor3d_CurveOnSurface(Adaptor3d_Surface S)
    {
        myType = GeomAbs_OtherCurve;
        myIntCont = GeomAbs_CN;
        Load(S);
    }

    /// <summary>The curve C on the surface S.</summary>
    public Adaptor3d_CurveOnSurface(Adaptor2d_Curve2d C, Adaptor3d_Surface S)
    {
        myType = GeomAbs_OtherCurve;
        myIntCont = GeomAbs_CN;
        Load(S);
        Load(C);
    }

    /// <summary>A copy with shallow copies of the curve and surface adaptors; the interval cache is shared.</summary>
    public override Adaptor3d_Curve ShallowCopy()
    {
        Adaptor3d_CurveOnSurface aCopy = new Adaptor3d_CurveOnSurface();

        if (mySurface is not null)
        {
            aCopy.mySurface = mySurface.ShallowCopy();
        }
        if (myCurve is not null)
        {
            aCopy.myCurve = myCurve.ShallowCopy();
        }
        aCopy.myType = myType;
        aCopy.myCirc = myCirc;
        aCopy.myLin = myLin;
        if (myFirstSurf is not null)
        {
            aCopy.myFirstSurf = myFirstSurf.ShallowCopy();
        }
        if (myLastSurf is not null)
        {
            aCopy.myLastSurf = myLastSurf.ShallowCopy();
        }
        // OCCT shares the handle; NbIntervals replaces the sequence rather than changing it, so sharing is safe.
        aCopy.myIntervals = myIntervals;
        aCopy.myIntCont = myIntCont;

        return aCopy;
    }

    /// <summary>Changes the surface.</summary>
    public void Load(Adaptor3d_Surface S)
    {
        mySurface = S;
        if (myCurve is not null)
        {
            EvalKPart();
        }
    }

    /// <summary>Changes the 2D curve.</summary>
    public void Load(Adaptor2d_Curve2d C)
    {
        myCurve = C;
        if (mySurface is null)
        {
            return;
        }

        EvalKPart();

        GeomAbs_SurfaceType SType = mySurface.GetType();
        if (SType == GeomAbs_OffsetSurface)
        {
            SType = mySurface.BasisSurface().GetType();
        }
        if (SType == GeomAbs_BSplineSurface || SType == GeomAbs_SurfaceOfExtrusion
            || SType == GeomAbs_SurfaceOfRevolution)
        {
            EvalFirstLastSurf();
        }
    }

    /// <summary>Loads both the curve and the surface.</summary>
    public void Load(Adaptor2d_Curve2d C, Adaptor3d_Surface S)
    {
        Load(C);
        Load(S);
    }

    /// <summary>The 2D curve (null if none is loaded).</summary>
    public Adaptor2d_Curve2d? GetCurve() => myCurve;

    /// <summary>The surface (null if none is loaded).</summary>
    public Adaptor3d_Surface? GetSurface() => mySurface;

    /// <summary>
    /// A reference to the stored 2D curve, as OCCT's <c>ChangeCurve()</c>. Assigning through it does not
    /// recompute the special cases; use <see cref="Load(Adaptor2d_Curve2d)"/> for that.
    /// </summary>
    public ref Adaptor2d_Curve2d? ChangeCurve() => ref myCurve;

    /// <summary>A reference to the stored surface, as OCCT's <c>ChangeSurface()</c> (see <see cref="ChangeCurve"/>).</summary>
    public ref Adaptor3d_Surface? ChangeSurface() => ref mySurface;

    public override double FirstParameter() => myCurve!.FirstParameter();

    public override double LastParameter() => myCurve!.LastParameter();

    public override GeomAbs_Shape Continuity()
    {
        GeomAbs_Shape ContC = myCurve!.Continuity();
        GeomAbs_Shape ContSu = mySurface!.UContinuity();
        if (ContSu < ContC)
        {
            ContC = ContSu;
        }
        GeomAbs_Shape ContSv = mySurface.VContinuity();
        if (ContSv < ContC)
        {
            ContC = ContSv;
        }

        return ContC;
    }

    // Auxiliary: adds roots of equation to sorted sequence of parameters
    // along curve, keeping it sorted and avoiding repetitions (within tolerance Tol)
    private static void AddIntervals(NCollection_HSequence<double> theParameters, math_FunctionRoots theRoots, double theTol)
    {
        if (!theRoots.IsDone() || theRoots.IsAllNull())
        {
            return;
        }

        int nsol = theRoots.NbSolutions();
        for (int i = 1; i <= nsol; i++)
        {
            double param = theRoots.Value(i);
            if (param - theParameters.Value(1) < theTol)
            { // skip param if equal to or less than theParameters(1)
                continue;
            }
            for (int j = 2; j <= theParameters.Length(); ++j)
            {
                double aDelta = theParameters.Value(j) - param;
                if (aDelta > theTol)
                {
                    theParameters.InsertBefore(j, param);
                    break;
                }
                else if (aDelta >= -theTol)
                { // param == theParameters(j) within Tol
                    break;
                }
            }
        }
    }

    /// <summary>
    /// The number of intervals of continuity S: the curve's, split where it crosses the surface's U and V
    /// interval bounds. The result is cached for the last S.
    /// </summary>
    public override int NbIntervals(GeomAbs_Shape S)
    {
        if (S == myIntCont && myIntervals is not null)
        {
            return myIntervals.Length() - 1;
        }

        int nu, nv, nc;
        nu = mySurface!.NbUIntervals(S);
        nv = mySurface.NbVIntervals(S);
        nc = myCurve!.NbIntervals(S);

        // Allocate the memory for arrays TabU, TabV, TabC only once using the buffer TabBuf.
        NCollection_Array1<double> TabBuf = new(1, nu + nv + nc + 3);
        NCollection_Array1<double> TabU = new(TabBuf, 1, 1, nu + 1);
        NCollection_Array1<double> TabV = new(TabBuf, nu + 2, 1, nv + 1);
        NCollection_Array1<double> TabC = new(TabBuf, nu + nv + 3, 1, nc + 1);

        int NbSample = 20;
        double U, V, Tdeb, Tfin;
        Tdeb = myCurve.FirstParameter();
        Tfin = myCurve.LastParameter();

        myCurve.Intervals(TabC, S);

        double Tol = Precision.PConfusion() / 10;

        // sorted sequence of parameters defining continuity intervals;
        // started with own intervals of curve and completed by
        // additional points coming from surface discontinuities
        NCollection_HSequence<double> aIntervals = new NCollection_HSequence<double>();
        for (int i = 1; i <= nc + 1; i++)
        {
            aIntervals.Append(TabC[i]);
        }

        if (nu > 1)
        {
            mySurface.UIntervals(TabU, S);
            for (int iu = 2; iu <= nu; iu++)
            {
                U = TabU.Value(iu);
                Adaptor3d_InterFunc Func = new Adaptor3d_InterFunc(myCurve, U, 1);
                math_FunctionRoots Resol = new math_FunctionRoots(Func, Tdeb, Tfin, NbSample, Tol, Tol, Tol, 0.0);
                AddIntervals(aIntervals, Resol, Tol);
            }
        }
        if (nv > 1)
        {
            mySurface.VIntervals(TabV, S);
            for (int iv = 2; iv <= nv; iv++)
            {
                V = TabV.Value(iv);
                Adaptor3d_InterFunc Func = new Adaptor3d_InterFunc(myCurve, V, 2);
                math_FunctionRoots Resol = new math_FunctionRoots(Func, Tdeb, Tfin, NbSample, Tol, Tol, Tol, 0.0);
                AddIntervals(aIntervals, Resol, Tol);
            }
        }

        // for case intervals==1 and first point == last point SequenceOfReal
        // contains only one value, therefore it is necessary to add second
        // value into aIntervals which will be equal first value.
        if (aIntervals.Length() == 1)
        {
            aIntervals.Append(aIntervals.Value(1));
        }

        myIntervals = aIntervals;
        myIntCont = S;
        return myIntervals.Length() - 1;
    }

    /// <summary>Stores in T the bounds of the intervals of continuity S; T must have NbIntervals(S) + 1 values.</summary>
    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        NbIntervals(S);
        if (T.Length() != myIntervals!.Length())
        {
            throw new Standard_ProgramError("Error: Wrong size of array buffer in call to Adaptor3d_CurveOnSurface::Intervals");
        }
        for (int i = 1; i <= myIntervals.Length(); i++)
        {
            // OCCT writes T(i), assuming the array starts at 1.
            T[i] = myIntervals.Value(i);
        }
    }

    /// <summary>A new adaptor on the same surface, of the curve trimmed to [First, Last].</summary>
    public override Adaptor3d_Curve Trim(double First, double Last, double Tol)
    {
        Adaptor3d_CurveOnSurface HCS = new Adaptor3d_CurveOnSurface();
        HCS.Load(mySurface!);
        HCS.Load(myCurve!.Trim(First, Last, Tol));
        return HCS;
    }

    public override bool IsClosed() => myCurve!.IsClosed();

    public override bool IsPeriodic()
    {
        if (myType == GeomAbs_Circle || myType == GeomAbs_Ellipse)
        {
            return true;
        }

        return myCurve!.IsPeriodic();
    }

    public override double Period()
    {
        if (myType == GeomAbs_Circle || myType == GeomAbs_Ellipse)
        {
            return (2.0 * M_PI);
        }

        return myCurve!.Period();
    }

    /// <summary>The point of parameter theU.</summary>
    public sealed override gp_Pnt EvalD0(double theU)
    {
        gp_Pnt P;
        gp_Pnt2d Puv;

        if (myType == GeomAbs_Line)
        {
            P = ElCLib.Value(theU, myLin);
        }
        else if (myType == GeomAbs_Circle)
        {
            P = ElCLib.Value(theU, myCirc);
        }
        else
        {
            myCurve!.D0(theU, out Puv);
            mySurface!.D0(Puv.X(), Puv.Y(), out P);
        }

        return P;
    }

    /// <summary>The point and first derivative at theU.</summary>
    public sealed override Geom_Curve.ResD1 EvalD1(double theU)
    {
        gp_Pnt2d Puv;
        gp_Vec2d Duv;
        gp_Vec D1U, D1V;

        Geom_Curve.ResD1 aRes = new Geom_Curve.ResD1();

        double FP = myCurve!.FirstParameter();
        double LP = myCurve.LastParameter();

        double Tol = Precision.PConfusion() / 10;
        if ((Math.Abs(theU - FP) < Tol) && (myFirstSurf is not null))
        {
            myCurve.D1(theU, out Puv, out Duv);
            myFirstSurf.D1(Puv.X(), Puv.Y(), out aRes.Point, out D1U, out D1V);
            aRes.D1.SetLinearForm(Duv.X(), D1U, Duv.Y(), D1V);
        }
        else if ((Math.Abs(theU - LP) < Tol) && (myLastSurf is not null))
        {
            myCurve.D1(theU, out Puv, out Duv);
            myLastSurf.D1(Puv.X(), Puv.Y(), out aRes.Point, out D1U, out D1V);
            aRes.D1.SetLinearForm(Duv.X(), D1U, Duv.Y(), D1V);
        }
        else if (myType == GeomAbs_Line)
        {
            ElCLib.D1(theU, myLin, out aRes.Point, out aRes.D1);
        }
        else if (myType == GeomAbs_Circle)
        {
            ElCLib.D1(theU, myCirc, out aRes.Point, out aRes.D1);
        }
        else
        {
            myCurve.D1(theU, out Puv, out Duv);
            mySurface!.D1(Puv.X(), Puv.Y(), out aRes.Point, out D1U, out D1V);
            aRes.D1.SetLinearForm(Duv.X(), D1U, Duv.Y(), D1V);
        }

        return aRes;
    }

    /// <summary>The point and first two derivatives at theU.</summary>
    public sealed override Geom_Curve.ResD2 EvalD2(double theU)
    {
        gp_Pnt2d UV;
        gp_Vec2d DW, D2W;
        gp_Vec D1U, D1V, D2U, D2V, D2UV;

        Geom_Curve.ResD2 aRes = new Geom_Curve.ResD2();

        double FP = myCurve!.FirstParameter();
        double LP = myCurve.LastParameter();

        double Tol = Precision.PConfusion() / 10;
        if ((Math.Abs(theU - FP) < Tol) && (myFirstSurf is not null))
        {
            myCurve.D2(theU, out UV, out DW, out D2W);
            myFirstSurf.D2(UV.X(), UV.Y(), out aRes.Point, out D1U, out D1V, out D2U, out D2V, out D2UV);

            aRes.D1.SetLinearForm(DW.X(), D1U, DW.Y(), D1V);
            aRes.D2.SetLinearForm(D2W.X(), D1U, D2W.Y(), D1V, 2.0 * DW.X() * DW.Y(), D2UV);
            aRes.D2.SetLinearForm(DW.X() * DW.X(), D2U, DW.Y() * DW.Y(), D2V, aRes.D2);
        }
        else if ((Math.Abs(theU - LP) < Tol) && (myLastSurf is not null))
        {
            myCurve.D2(theU, out UV, out DW, out D2W);
            myLastSurf.D2(UV.X(), UV.Y(), out aRes.Point, out D1U, out D1V, out D2U, out D2V, out D2UV);

            aRes.D1.SetLinearForm(DW.X(), D1U, DW.Y(), D1V);
            aRes.D2.SetLinearForm(D2W.X(), D1U, D2W.Y(), D1V, 2.0 * DW.X() * DW.Y(), D2UV);
            aRes.D2.SetLinearForm(DW.X() * DW.X(), D2U, DW.Y() * DW.Y(), D2V, aRes.D2);
        }
        else if (myType == GeomAbs_Line)
        {
            ElCLib.D1(theU, myLin, out aRes.Point, out aRes.D1);
            aRes.D2.SetCoord(0.0, 0.0, 0.0);
        }
        else if (myType == GeomAbs_Circle)
        {
            ElCLib.D2(theU, myCirc, out aRes.Point, out aRes.D1, out aRes.D2);
        }
        else
        {
            myCurve.D2(theU, out UV, out DW, out D2W);
            mySurface!.D2(UV.X(), UV.Y(), out aRes.Point, out D1U, out D1V, out D2U, out D2V, out D2UV);

            aRes.D1.SetLinearForm(DW.X(), D1U, DW.Y(), D1V);
            aRes.D2.SetLinearForm(D2W.X(), D1U, D2W.Y(), D1V, 2.0 * DW.X() * DW.Y(), D2UV);
            aRes.D2.SetLinearForm(DW.X() * DW.X(), D2U, DW.Y() * DW.Y(), D2V, aRes.D2);
        }

        return aRes;
    }

    /// <summary>The point and first three derivatives at theU.</summary>
    public sealed override Geom_Curve.ResD3 EvalD3(double theU)
    {
        double Tol = Precision.PConfusion() / 10;
        gp_Pnt2d UV;
        gp_Vec2d DW, D2W, D3W;
        gp_Vec D1U, D1V, D2U, D2V, D2UV, D3U, D3V, D3UUV, D3UVV;

        Geom_Curve.ResD3 aRes = new Geom_Curve.ResD3();

        double FP = myCurve!.FirstParameter();
        double LP = myCurve.LastParameter();

        if ((Math.Abs(theU - FP) < Tol) && (myFirstSurf is not null))
        {
            myCurve.D3(theU, out UV, out DW, out D2W, out D3W);
            myFirstSurf.D3(UV.X(), UV.Y(), out aRes.Point, out D1U, out D1V, out D2U, out D2V, out D2UV, out D3U, out D3V,
                           out D3UUV, out D3UVV);
            aRes.D1.SetLinearForm(DW.X(), D1U, DW.Y(), D1V);
            aRes.D2.SetLinearForm(D2W.X(), D1U, D2W.Y(), D1V, 2.0 * DW.X() * DW.Y(), D2UV);
            aRes.D2.SetLinearForm(DW.X() * DW.X(), D2U, DW.Y() * DW.Y(), D2V, aRes.D2);
            aRes.D3 = SetLinearForm(DW, D2W, D3W, D1U, D1V, D2U, D2V, D2UV, D3U, D3V, D3UUV, D3UVV);
        }
        else if ((Math.Abs(theU - LP) < Tol) && (myLastSurf is not null))
        {
            myCurve.D3(theU, out UV, out DW, out D2W, out D3W);
            myLastSurf.D3(UV.X(), UV.Y(), out aRes.Point, out D1U, out D1V, out D2U, out D2V, out D2UV, out D3U, out D3V,
                          out D3UUV, out D3UVV);
            aRes.D1.SetLinearForm(DW.X(), D1U, DW.Y(), D1V);
            aRes.D2.SetLinearForm(D2W.X(), D1U, D2W.Y(), D1V, 2.0 * DW.X() * DW.Y(), D2UV);
            aRes.D2.SetLinearForm(DW.X() * DW.X(), D2U, DW.Y() * DW.Y(), D2V, aRes.D2);
            aRes.D3 = SetLinearForm(DW, D2W, D3W, D1U, D1V, D2U, D2V, D2UV, D3U, D3V, D3UUV, D3UVV);
        }
        else if (myType == GeomAbs_Line)
        {
            ElCLib.D1(theU, myLin, out aRes.Point, out aRes.D1);
            aRes.D2.SetCoord(0.0, 0.0, 0.0);
            aRes.D3.SetCoord(0.0, 0.0, 0.0);
        }
        else if (myType == GeomAbs_Circle)
        {
            ElCLib.D3(theU, myCirc, out aRes.Point, out aRes.D1, out aRes.D2, out aRes.D3);
        }
        else
        {
            myCurve.D3(theU, out UV, out DW, out D2W, out D3W);
            mySurface!.D3(UV.X(), UV.Y(), out aRes.Point, out D1U, out D1V, out D2U, out D2V, out D2UV, out D3U, out D3V,
                          out D3UUV, out D3UVV);
            aRes.D1.SetLinearForm(DW.X(), D1U, DW.Y(), D1V);
            aRes.D2.SetLinearForm(D2W.X(), D1U, D2W.Y(), D1V, 2.0 * DW.X() * DW.Y(), D2UV);
            aRes.D2.SetLinearForm(DW.X() * DW.X(), D2U, DW.Y() * DW.Y(), D2V, aRes.D2);
            aRes.D3 = SetLinearForm(DW, D2W, D3W, D1U, D1V, D2U, D2V, D2UV, D3U, D3V, D3UUV, D3UVV);
        }

        return aRes;
    }

    /// <summary>The derivative of order theN (1 to 3) at theU.</summary>
    /// <exception cref="Standard_NotImplemented">theN is above 3.</exception>
    public sealed override gp_Vec EvalDN(double theU, int theN)
    {
        switch (theN)
        {
            case 1:
                return EvalD1(theU).D1;
            case 2:
                return EvalD2(theU).D2;
            case 3:
                return EvalD3(theU).D3;
            default:
                throw new Standard_NotImplemented("Adaptor3d_CurveOnSurface:EvalDN");
        }
    }

    public override double Resolution(double R3d)
    {
        double ru, rv;
        ru = mySurface!.UResolution(R3d);
        rv = mySurface.VResolution(R3d);
        return myCurve!.Resolution(Min(ru, rv));
    }

    public override GeomAbs_CurveType GetType() => myType;

    public override gp_Lin Line()
    {
        Standard_NoSuchObject_Raise_if(myType != GeomAbs_Line, "Adaptor3d_CurveOnSurface::Line(): curve is not a line");
        return myLin;
    }

    public override gp_Circ Circle()
    {
        // OCCT's message says Line() (cosmetic).
        Standard_NoSuchObject_Raise_if(myType != GeomAbs_Circle, "Adaptor3d_CurveOnSurface::Line(): curve is not a circle");
        return myCirc;
    }

    /// <summary>The ellipse, for an ellipse on a plane.</summary>
    public override gp_Elips Ellipse() => to3d(mySurface!.Plane(), myCurve!.Ellipse());

    /// <summary>The hyperbola, for a hyperbola on a plane.</summary>
    public override gp_Hypr Hyperbola() => to3d(mySurface!.Plane(), myCurve!.Hyperbola());

    /// <summary>The parabola, for a parabola on a plane.</summary>
    public override gp_Parab Parabola() => to3d(mySurface!.Plane(), myCurve!.Parabola());

    /// <summary>The degree of the 2D curve (not multiplied by the surface's).</summary>
    public override int Degree()
    {
        // on a parametric surface should multiply
        // return TheCurve2dTool::Degree(myCurve);

        return myCurve!.Degree();
    }

    public override bool IsRational()
    {
        return (myCurve!.IsRational() || mySurface!.IsURational() || mySurface.IsVRational());
    }

    public override int NbPoles()
    {
        // on a parametric surface should multiply
        return myCurve!.NbPoles();
    }

    public override int NbKnots()
    {
        if (mySurface!.GetType() == GeomAbs_Plane)
        {
            return myCurve!.NbKnots();
        }
        else
        {
            throw new Standard_NoSuchObject();
        }
    }

    /// <summary>The 3D Bezier curve of a 2D Bezier curve on a plane (a new curve).</summary>
    public override Geom_BezierCurve Bezier()
    {
        Standard_NoSuchObject_Raise_if(mySurface!.GetType() != GeomAbs_Plane, "Adaptor3d_CurveOnSurface : Bezier");

        Geom2d_BezierCurve Bez2d = myCurve!.Bezier();
        int NbPoles = Bez2d.NbPoles();

        gp_Pln Plane = mySurface.Plane();

        NCollection_Array1<gp_Pnt> Poles = new(1, NbPoles);
        for (int i = 1; i <= NbPoles; i++)
        {
            Poles[i] = to3d(Plane, Bez2d.Pole(i));
        }
        Geom_BezierCurve Bez;

        if (Bez2d.IsRational())
        {
            NCollection_Array1<double> Weights = Bez2d.InternalWeightsArray();
            Bez = new Geom_BezierCurve(Poles, Weights);
        }
        else
        {
            Bez = new Geom_BezierCurve(Poles);
        }
        return Bez;
    }

    /// <summary>The 3D B-spline curve of a 2D B-spline curve on a plane (a new curve).</summary>
    public override Geom_BSplineCurve BSpline()
    {
        Standard_NoSuchObject_Raise_if(mySurface!.GetType() != GeomAbs_Plane, "Adaptor3d_CurveOnSurface : BSpline");

        Geom2d_BSplineCurve Bsp2d = myCurve!.BSpline();
        int NbPoles = Bsp2d.NbPoles();

        gp_Pln Plane = mySurface.Plane();

        NCollection_Array1<gp_Pnt> Poles = new(1, NbPoles);
        for (int i = 1; i <= NbPoles; i++)
        {
            Poles[i] = to3d(Plane, Bsp2d.Pole(i));
        }

        NCollection_Array1<double> Knots = Bsp2d.InternalKnots();
        NCollection_Array1<int> Mults = Bsp2d.InternalMultiplicities();
        Geom_BSplineCurve Bsp;

        if (Bsp2d.IsRational())
        {
            NCollection_Array1<double> Weights = Bsp2d.InternalWeightsArray();
            Bsp = new Geom_BSplineCurve(Poles, Weights, Knots, Mults, Bsp2d.Degree(), Bsp2d.IsPeriodic());
        }
        else
        {
            Bsp = new Geom_BSplineCurve(Poles, Knots, Mults, Bsp2d.Degree(), Bsp2d.IsPeriodic());
        }
        return Bsp;
    }

    // Recognises lines and circles: curves of a plane, and the isoparametric lines of the elementary surfaces
    private void EvalKPart()
    {
        myType = GeomAbs_OtherCurve;

        GeomAbs_SurfaceType STy = mySurface!.GetType();
        GeomAbs_CurveType CTy = myCurve!.GetType();
        if (STy == GeomAbs_Plane)
        {
            myType = CTy;
            if (myType == GeomAbs_Circle)
            {
                myCirc = to3d(mySurface.Plane(), myCurve.Circle());
            }
            else if (myType == GeomAbs_Line)
            {
                gp_Pnt P;
                gp_Vec V = new gp_Vec();
                gp_Pnt2d Puv;
                gp_Vec2d Duv;
                myCurve.D1(0.0, out Puv, out Duv);
                gp_Vec D1U, D1V;
                mySurface.D1(Puv.X(), Puv.Y(), out P, out D1U, out D1V);
                V.SetLinearForm(Duv.X(), D1U, Duv.Y(), D1V);
                myLin = new gp_Lin(P, new gp_Dir(V));
            }
        }
        else
        {
            if (CTy == GeomAbs_Line)
            {
                gp_Dir2d D = myCurve.Line().Direction();
                if (D.IsParallel(gp.DX2d(), Precision.Angular()))
                { // Iso V.
                    if (STy == GeomAbs_Sphere)
                    {
                        gp_Pnt2d P = myCurve.Line().Location();
                        if (Math.Abs(Math.Abs(P.Y()) - M_PI / 2.0) >= Precision.PConfusion())
                        {
                            myType = GeomAbs_Circle;
                            gp_Sphere Sph = mySurface.Sphere();
                            gp_Ax3 Axis = Sph.Position();
                            myCirc = ElSLib.SphereVIso(Axis, Sph.Radius(), P.Y());
                            gp_Dir DRev = Axis.XDirection().Crossed(Axis.YDirection());
                            gp_Ax1 AxeRev = new gp_Ax1(Axis.Location(), DRev);
                            myCirc.Rotate(AxeRev, P.X());
                            if (D.IsOpposite(gp.DX2d(), Precision.Angular()))
                            {
                                gp_Ax2 Ax = myCirc.Position();
                                Ax.SetDirection(Ax.Direction().Reversed());
                                myCirc.SetPosition(Ax);
                            }
                        }
                    }
                    else if (STy == GeomAbs_Cylinder)
                    {
                        myType = GeomAbs_Circle;
                        gp_Cylinder Cyl = mySurface.Cylinder();
                        gp_Pnt2d P = myCurve.Line().Location();
                        gp_Ax3 Axis = Cyl.Position();
                        myCirc = ElSLib.CylinderVIso(Axis, Cyl.Radius(), P.Y());
                        gp_Dir DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        gp_Ax1 AxeRev = new gp_Ax1(Axis.Location(), DRev);
                        myCirc.Rotate(AxeRev, P.X());
                        if (D.IsOpposite(gp.DX2d(), Precision.Angular()))
                        {
                            gp_Ax2 Ax = myCirc.Position();
                            Ax.SetDirection(Ax.Direction().Reversed());
                            myCirc.SetPosition(Ax);
                        }
                    }
                    else if (STy == GeomAbs_Cone)
                    {
                        myType = GeomAbs_Circle;
                        gp_Cone Cone = mySurface.Cone();
                        gp_Pnt2d P = myCurve.Line().Location();
                        gp_Ax3 Axis = Cone.Position();
                        myCirc = ElSLib.ConeVIso(Axis, Cone.RefRadius(), Cone.SemiAngle(), P.Y());
                        gp_Dir DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        gp_Ax1 AxeRev = new gp_Ax1(Axis.Location(), DRev);
                        myCirc.Rotate(AxeRev, P.X());
                        if (D.IsOpposite(gp.DX2d(), Precision.Angular()))
                        {
                            gp_Ax2 Ax = myCirc.Position();
                            Ax.SetDirection(Ax.Direction().Reversed());
                            myCirc.SetPosition(Ax);
                        }
                    }
                    else if (STy == GeomAbs_Torus)
                    {
                        myType = GeomAbs_Circle;
                        gp_Torus Tore = mySurface.Torus();
                        gp_Pnt2d P = myCurve.Line().Location();
                        gp_Ax3 Axis = Tore.Position();
                        myCirc = ElSLib.TorusVIso(Axis, Tore.MajorRadius(), Tore.MinorRadius(), P.Y());
                        gp_Dir DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        gp_Ax1 AxeRev = new gp_Ax1(Axis.Location(), DRev);
                        myCirc.Rotate(AxeRev, P.X());
                        if (D.IsOpposite(gp.DX2d(), Precision.Angular()))
                        {
                            gp_Ax2 Ax = myCirc.Position();
                            Ax.SetDirection(Ax.Direction().Reversed());
                            myCirc.SetPosition(Ax);
                        }
                    }
                }
                else if (D.IsParallel(gp.DY2d(), Precision.Angular()))
                { // Iso U.
                    if (STy == GeomAbs_Sphere)
                    {
                        myType = GeomAbs_Circle;
                        gp_Sphere Sph = mySurface.Sphere();
                        gp_Pnt2d P = myCurve.Line().Location();
                        gp_Ax3 Axis = Sph.Position();
                        // Compute the iso 0.
                        myCirc = ElSLib.SphereUIso(Axis, Sph.Radius(), 0.0);

                        // Same-parametrization (circle rotation - Y offset)
                        gp_Dir DRev = Axis.XDirection().Crossed(Axis.Direction());
                        gp_Ax1 AxeRev = new gp_Ax1(Axis.Location(), DRev);
                        myCirc.Rotate(AxeRev, P.Y());

                        // Transform to iso U ( = P.X())
                        DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        AxeRev = new gp_Ax1(Axis.Location(), DRev);
                        myCirc.Rotate(AxeRev, P.X());

                        if (D.IsOpposite(gp.DY2d(), Precision.Angular()))
                        {
                            gp_Ax2 Ax = myCirc.Position();
                            Ax.SetDirection(Ax.Direction().Reversed());
                            myCirc.SetPosition(Ax);
                        }
                    }
                    else if (STy == GeomAbs_Cylinder)
                    {
                        myType = GeomAbs_Line;
                        gp_Cylinder Cyl = mySurface.Cylinder();
                        gp_Pnt2d P = myCurve.Line().Location();
                        myLin = ElSLib.CylinderUIso(Cyl.Position(), Cyl.Radius(), P.X());
                        gp_Vec Tr = new gp_Vec(myLin.Direction());
                        Tr.Multiply(P.Y());
                        myLin.Translate(Tr);
                        if (D.IsOpposite(gp.DY2d(), Precision.Angular()))
                        {
                            myLin.Reverse();
                        }
                    }
                    else if (STy == GeomAbs_Cone)
                    {
                        myType = GeomAbs_Line;
                        gp_Cone Cone = mySurface.Cone();
                        gp_Pnt2d P = myCurve.Line().Location();
                        myLin = ElSLib.ConeUIso(Cone.Position(), Cone.RefRadius(), Cone.SemiAngle(), P.X());
                        gp_Vec Tr = new gp_Vec(myLin.Direction());
                        Tr.Multiply(P.Y());
                        myLin.Translate(Tr);
                        if (D.IsOpposite(gp.DY2d(), Precision.Angular()))
                        {
                            myLin.Reverse();
                        }
                    }
                    else if (STy == GeomAbs_Torus)
                    {
                        myType = GeomAbs_Circle;
                        gp_Torus Tore = mySurface.Torus();
                        gp_Pnt2d P = myCurve.Line().Location();
                        gp_Ax3 Axis = Tore.Position();
                        myCirc = ElSLib.TorusUIso(Axis, Tore.MajorRadius(), Tore.MinorRadius(), P.X());
                        myCirc.Rotate(myCirc.Axis(), P.Y());

                        if (D.IsOpposite(gp.DY2d(), Precision.Angular()))
                        {
                            gp_Ax2 Ax = myCirc.Position();
                            Ax.SetDirection(Ax.Direction().Reversed());
                            myCirc.SetPosition(Ax);
                        }
                    }
                }
            }
        }
    }

    // Finds the patches of the surface around the curve's ends, used to evaluate there on the right side of a knot
    private void EvalFirstLastSurf()
    {
        double FirstPar, LastPar;
        gp_Pnt2d UV, LeftBot = new gp_Pnt2d(), RightTop = new gp_Pnt2d();
        gp_Vec2d DUV;
        double Tol = Precision.PConfusion() / 10;
        bool Ok = true;

        FirstPar = myCurve!.FirstParameter();
        myCurve.D1(FirstPar, out UV, out DUV);

        if (DUV.Magnitude() <= Tol)
        {
            Ok = false;
        }

        if (Ok)
        {
            switch (mySurface!.GetType())
            {
                case GeomAbs_BSplineSurface:
                    LocatePart(UV, DUV, mySurface, ref LeftBot, ref RightTop);
                    break;
                case GeomAbs_SurfaceOfRevolution:
                case GeomAbs_SurfaceOfExtrusion:
                    Ok = LocatePart_RevExt(UV, DUV, mySurface, ref LeftBot, ref RightTop);
                    break;
                case GeomAbs_OffsetSurface:
                    Ok = LocatePart_Offset(UV, DUV, mySurface, ref LeftBot, ref RightTop);
                    break;
                default:
                    throw new Standard_NotImplemented("Adaptor3d_CurveOnSurface::EvalFirstLastSurf");
            }
        }

        if (Ok)
        {
            CompareBounds(ref LeftBot, ref RightTop); // SVV

            myFirstSurf = mySurface!.UTrim(LeftBot.X(), RightTop.X(), Tol);
            myFirstSurf = myFirstSurf.VTrim(LeftBot.Y(), RightTop.Y(), Tol);
        }
        else
        {
            myFirstSurf = mySurface;
        }

        LastPar = myCurve.LastParameter();
        Ok = true;
        myCurve.D1(LastPar, out UV, out DUV);
        DUV.Reverse(); // We want the other part

        if (DUV.Magnitude() <= Tol)
        {
            Ok = false;
        }

        if (Ok)
        {
            switch (mySurface!.GetType())
            {
                case GeomAbs_BSplineSurface:
                    LocatePart(UV, DUV, mySurface, ref LeftBot, ref RightTop);
                    break;
                case GeomAbs_SurfaceOfRevolution:
                case GeomAbs_SurfaceOfExtrusion:
                    Ok = LocatePart_RevExt(UV, DUV, mySurface, ref LeftBot, ref RightTop);
                    break;
                case GeomAbs_OffsetSurface:
                    Ok = LocatePart_Offset(UV, DUV, mySurface, ref LeftBot, ref RightTop);
                    break;
                default:
                    throw new Standard_NotImplemented("Adaptor3d_CurveOnSurface::EvalFirstLastSurf");
            }
        }

        if (Ok)
        {
            CompareBounds(ref LeftBot, ref RightTop); // SVV

            myLastSurf = mySurface!.UTrim(LeftBot.X(), RightTop.X(), Tol);
            myLastSurf = myLastSurf.VTrim(LeftBot.Y(), RightTop.Y(), Tol);
        }
        else
        {
            myLastSurf = mySurface;
        }
    }

    private bool LocatePart_RevExt(in gp_Pnt2d UV, in gp_Vec2d DUV, Adaptor3d_Surface S, ref gp_Pnt2d LeftBot,
                                   ref gp_Pnt2d RightTop)
    {
        Adaptor3d_Curve AHC = S.BasisCurve();

        if (AHC.GetType() == GeomAbs_BSplineCurve)
        {
            Geom_BSplineCurve BSplC;
            BSplC = AHC.BSpline();

            if ((S.GetType()) == GeomAbs_SurfaceOfExtrusion)
            {
                Locate1Coord(1, UV, DUV, BSplC, ref LeftBot, ref RightTop);
                Locate2Coord(2, UV, DUV, S.FirstVParameter(), S.LastVParameter(), ref LeftBot, ref RightTop);
            }
            else if ((S.GetType()) == GeomAbs_SurfaceOfRevolution)
            {
                Locate1Coord(2, UV, DUV, BSplC, ref LeftBot, ref RightTop);
                Locate2Coord(1, UV, DUV, S.FirstUParameter(), S.LastUParameter(), ref LeftBot, ref RightTop);
            }

            double u1, u2, v1, v2;
            ReverseParam(LeftBot.X(), RightTop.X(), out u1, out u2);
            LeftBot.SetX(u1);
            RightTop.SetX(u2);
            ReverseParam(LeftBot.Y(), RightTop.Y(), out v1, out v2);
            LeftBot.SetY(v1);
            RightTop.SetY(v2);
            return true;
        }
        return false;
    }

    private bool LocatePart_Offset(in gp_Pnt2d UV, in gp_Vec2d DUV, Adaptor3d_Surface S, ref gp_Pnt2d LeftBot,
                                   ref gp_Pnt2d RightTop)
    {
        bool Ok = true;
        Adaptor3d_Surface AHS;
        AHS = S.BasisSurface();
        GeomAbs_SurfaceType BasisSType = AHS.GetType();
        switch (BasisSType)
        {
            case GeomAbs_SurfaceOfRevolution:
            case GeomAbs_SurfaceOfExtrusion:
                Ok = LocatePart_RevExt(UV, DUV, AHS, ref LeftBot, ref RightTop);
                break;

            case GeomAbs_BSplineSurface:
                LocatePart(UV, DUV, AHS, ref LeftBot, ref RightTop);
                break;

            default:
                Ok = false;
                break;
        }
        return Ok;
    }

    private void LocatePart(in gp_Pnt2d UV, in gp_Vec2d DUV, Adaptor3d_Surface S, ref gp_Pnt2d LeftBot,
                            ref gp_Pnt2d RightTop)
    {
        Geom_BSplineSurface BSplS;
        BSplS = S.BSpline();
        bool DUIsNull, DVIsNull;

        Locate1Coord(1, UV, DUV, BSplS, out DUIsNull, ref LeftBot, ref RightTop);
        Locate1Coord(2, UV, DUV, BSplS, out DVIsNull, ref LeftBot, ref RightTop);

        if ((DUIsNull) && (!DVIsNull))
        {
            NCollection_Array1<double> ArrU = BSplS.InternalUKnots();
            Locate2Coord(1, UV, DUV, BSplS, ArrU, ref LeftBot, ref RightTop);
        }
        else if ((DVIsNull) && (!DUIsNull))
        {
            NCollection_Array1<double> ArrV = BSplS.InternalVKnots();
            Locate2Coord(2, UV, DUV, BSplS, ArrV, ref LeftBot, ref RightTop);
        }
    }
}
