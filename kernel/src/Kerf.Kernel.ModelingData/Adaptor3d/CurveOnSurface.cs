// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_CurveOnSurface.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_CurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// A curve defined by a 2D curve in the parametric space of a surface: the 3D curve
/// <c>S(C(t))</c>. Lines and circles on planes and the isoparametric lines of the elementary
/// surfaces are recognised as lines and circles.
/// </summary>
/// <remarks>
/// An adaptor is a working object (decision 101): <see cref="Load(SurfaceAdaptorBase)"/> and
/// <see cref="Load(Curve2DAdaptorBase)"/> are public, and the intervals are cached, so evaluating one adaptor from
/// several threads at once is not safe. The adaptor keeps references to the adaptors it is given, as OCCT does.
/// </remarks>
public class CurveOnSurface : CurveAdaptorBase
{
    private SurfaceAdaptorBase? _surface;
    private Curve2DAdaptorBase? _curve;
    private CurveKind _type;
    private Circle3D _circ = new Circle3D();
    private Line3D _lin = new Line3D();
    private SurfaceAdaptorBase? _firstSurf;
    private SurfaceAdaptorBase? _lastSurf;
    private NCollection_HSequence<double>? _intervals;
    private Continuity _intCont;

    private static Point3D To3d(in Plane3D pl, in Point2D p)
    {
        return global::Kerf.Engine.Geometry.ElSLib.Value(p.X(), p.Y(), pl);
    }

    private static Vector3D To3d(in Plane3D pl, in Vector2D v)
    {
        Vector3D vx = new Vector3D(pl.XAxis().Direction());
        Vector3D vy = new Vector3D(pl.YAxis().Direction());
        vx = vx.Multiplied(v.X());
        vy = vy.Multiplied(v.Y());
        vx = vx.Added(vy);
        return vx;
    }

    private static Frame3D To3d(in Plane3D pl, in Frame2D a)
    {
        Point3D p = To3d(pl, a.Location());
        Vector3D vx = To3d(pl, new Vector2D(a.XAxis().Direction()));
        Vector3D vy = To3d(pl, new Vector2D(a.YAxis().Direction()));
        return new Frame3D(p, new Direction3D(vx.Crossed(vy)), new Direction3D(vx));
    }

    private static Circle3D To3d(in Plane3D pl, in Circle2D c)
    {
        return new Circle3D(To3d(pl, c.Axis()), c.Radius());
    }

    private static Ellipse3D To3d(in Plane3D pl, in Ellipse2D e)
    {
        return new Ellipse3D(To3d(pl, e.Axis()), e.MajorRadius(), e.MinorRadius());
    }

    private static Hyperbola3D To3d(in Plane3D pl, in Hyperbola2D h)
    {
        return new Hyperbola3D(To3d(pl, h.Axis()), h.MajorRadius(), h.MinorRadius());
    }

    private static Parabola3D To3d(in Plane3D pl, in Parabola2D p)
    {
        return new Parabola3D(To3d(pl, p.Axis()), p.Focal());
    }

    private static Vector3D SetLinearForm(in Vector2D dw,
                                        in Vector2D d2W,
                                        in Vector2D d3W,
                                        in Vector3D d1U,
                                        in Vector3D d1V,
                                        in Vector3D d2U,
                                        in Vector3D d2V,
                                        in Vector3D d2UV,
                                        in Vector3D d3U,
                                        in Vector3D d3V,
                                        in Vector3D d3UUV,
                                        in Vector3D d3UVV)
    {
        // The third derivative of S(u(t), v(t)) by the chain rule (OCCT weighted D1U by u' for u''' and gave D3UVV
        // 5 u' v'^2 for 3 u' v'^2; Kerf decision 1488)
        Vector3D v31 = new Vector3D(), v32 = new Vector3D(), v33 = new Vector3D(), v34 = new Vector3D(), v3 = new Vector3D();

        v31 = Vector3D.LinearForm(d3W.X(), d1U, d2W.X() * dw.X(), d2U, d2W.X() * dw.Y(), d2UV);
        v31 = Vector3D.LinearForm(d3W.Y(), d1V, d2W.Y() * dw.X(), d2UV, d2W.Y() * dw.Y(), d2V, v31);

        v32 = Vector3D.LinearForm(dw.X() * dw.X() * dw.Y(), d3UUV, dw.X() * dw.Y() * dw.Y(), d3UVV);
        v32 = Vector3D.LinearForm(d2W.X() * dw.Y() + dw.X() * d2W.Y(), d2UV, v32);
        v33 = Vector3D.LinearForm(2 * d2W.X() * dw.X(), d2U, dw.X() * dw.X() * dw.X(), d3U, dw.X() * dw.X() * dw.Y(), d3UUV);

        v34 = Vector3D.LinearForm(2 * d2W.Y() * dw.Y(), d2V, dw.Y() * dw.Y() * dw.X(), d3UVV, dw.Y() * dw.Y() * dw.Y(), d3V);
        v3 = Vector3D.LinearForm(1, v31, 2, v32, 1, v33, v34);
        return v3;
    }

    private static void CompareBounds(ref Point2D p1, ref Point2D p2) // SVV
    {
        double lx = p1.X(), ly = p1.Y();
        double rx = p2.X(), ry = p2.Y();

        if (lx > rx)
        {
            p1 = p1.WithX(rx);
            p2 = p2.WithX(lx);
        }
        if (ly > ry)
        {
            p1 = p1.WithY(ry);
            p2 = p2.WithY(ly);
        }
    }

    private static void Hunt(NCollection_Array1<double> arr, double coord, ref int iloc)
    {
        // Warning: Hunt is used to find number of knot which equals coordinate component,
        //        when coordinate component definitely equals a knot only.
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        int i = 1;
        while ((i <= arr.Upper()) && (Math.Abs(coord - arr[i]) > tol))
        {
            i++;
        }

        // OCCT: when Coord is no knot, i is Upper() + 1 here and Arr(i) reads past the array (release build; the
        // port throws IndexOutOfRangeException instead). The callers only pass knots.
        if (Math.Abs(coord - arr[i]) < tol)
        {
            iloc = i;
        }
        else if (Math.Abs(coord - arr[i]) > tol)
        {
            throw new MethodNotImplementedException("Adaptor3d_CurveOnSurface:Hunt");
        }
    }

    private static void ReverseParam(double in1, double in2, out double out1, out double out2)
    {
        if (in1 > in2)
        {
            out1 = in2;
            out2 = in1;
        }
        else
        {
            out1 = in1;
            out2 = in2;
        }
    }

    private static void ReverseParam(int in1, int in2, out int out1, out int out2)
    {
        if (in1 > in2)
        {
            out1 = in2;
            out2 = in1;
        }
        else
        {
            out1 = in1;
            out2 = in2;
        }
    }

    private static void FindBounds(NCollection_Array1<double> arr,
                                   double coord,
                                   double der,
                                   ref int bound1,
                                   ref int bound2,
                                   out bool derNull)
    {
        int n = 0;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        Hunt(arr, coord, ref n);
        derNull = false;

        if (n == bound1)
        {
            if (Math.Abs(der) > tol)
            {
                derNull = false;
            }
            if (Math.Abs(der) <= tol)
            {
                derNull = true;
            }
            bound1 = n;
            bound2 = n + 1;
            return;
        }
        if (n == bound2)
        {
            if (Math.Abs(der) > tol)
            {
                derNull = false;
            }
            if (Math.Abs(der) <= tol)
            {
                derNull = true;
            }
            bound1 = n - 1;
            bound2 = n;
            return;
        }
        if ((n != bound1) && (n != bound2))
        {
            if (Math.Abs(der) > tol)
            {
                if (der > 0)
                {
                    bound1 = n;
                    bound2 = n + 1;
                }
                else
                {
                    bound1 = n - 1;
                    bound2 = n;
                }
                derNull = false;
            }
            if (Math.Abs(der) <= tol)
            {
                derNull = true;
                bound1 = n - 1;
                bound2 = n + 1;
            }
        }
    }

    private static void Locate1Coord(int index,
                                     in Point2D uv,
                                     in Vector2D duv,
                                     BSplineCurve3D bSplC,
                                     ref Point2D leftBot,
                                     ref Point2D rightTop)
    {
        double comp1 = 0, dComp1 = 0, cur, f = 0.0, l = 0.0;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        int i = 1, bnd1, bnd2;
        bool dIsNull = false;
        NCollection_Array1<double> arr = bSplC.InternalKnots();

        if (index == 1)
        {
            comp1 = uv.X();
            dComp1 = duv.X();
        }
        if (index == 2)
        {
            comp1 = uv.Y();
            dComp1 = duv.Y();
        }

        int lo = bSplC.FirstUKnotIndex(), up = bSplC.LastUKnotIndex();

        i = lo;
        while ((Math.Abs(bSplC.Knot(i) - comp1) > tol) && (i != up))
        {
            i++;
        }
        cur = bSplC.Knot(i);

        if (Math.Abs(comp1 - cur) <= tol)
        {
            bnd1 = lo;
            bnd2 = up;
            FindBounds(arr, cur, dComp1, ref bnd1, ref bnd2, out dIsNull);
            ReverseParam(bnd1, bnd2, out bnd1, out bnd2);

            if (!dIsNull)
            {
                if (index == 1)
                {
                    leftBot = leftBot.WithX(bSplC.Knot(bnd1));
                    rightTop = rightTop.WithX(bSplC.Knot(bnd2));
                }
                else if (index == 2)
                {
                    leftBot = leftBot.WithY(bSplC.Knot(bnd1));
                    rightTop = rightTop.WithY(bSplC.Knot(bnd2));
                }
            }
            else if (dIsNull)
            {
                if (Math.Abs(comp1 - (f = bSplC.Knot(lo))) <= tol)
                {
                    if (index == 1)
                    {
                        leftBot = leftBot.WithX(bSplC.Knot(lo));
                        rightTop = rightTop.WithX(bSplC.Knot(lo + 1));
                    }
                    else if (index == 2)
                    {
                        leftBot = leftBot.WithY(bSplC.Knot(lo));
                        rightTop = rightTop.WithY(bSplC.Knot(lo + 1));
                    }
                }
                else if (Math.Abs(comp1 - (l = bSplC.Knot(up))) <= tol)
                {
                    if (index == 1)
                    {
                        leftBot = leftBot.WithX(bSplC.Knot(up - 1));
                        rightTop = rightTop.WithX(bSplC.Knot(up));
                    }
                    else if (index == 2)
                    {
                        leftBot = leftBot.WithY(bSplC.Knot(up - 1));
                        rightTop = rightTop.WithY(bSplC.Knot(up));
                    }
                }
                else if (index == 1)
                {
                    leftBot = leftBot.WithX(bSplC.Knot(bnd1));
                    rightTop = rightTop.WithX(bSplC.Knot(bnd2));
                }
                else if (index == 2)
                {
                    leftBot = leftBot.WithY(bSplC.Knot(bnd1));
                    rightTop = rightTop.WithY(bSplC.Knot(bnd2));
                }
            }
        }
        else //*********if Coord != Knot
        {
            i = lo;
            while (i < up)
            {
                // skl 28.03.2002 for OCC233
                f = bSplC.Knot(i);
                l = bSplC.Knot(i + 1);
                if (f < comp1 && l > comp1)
                {
                    break;
                }
                i++;
            }
            ReverseParam(f, l, out f, out l);

            if (i != up)
            {
                if (Math.Abs(dComp1) < tol)
                {
                    if (index == 1)
                    {
                        leftBot = leftBot.WithX(f);
                        rightTop = rightTop.WithX(l);
                    }
                    else if (index == 2)
                    {
                        leftBot = leftBot.WithY(f);
                        rightTop = rightTop.WithY(l);
                    }
                }
                else if (Math.Abs(dComp1) > tol)
                {
                    if (index == 1)
                    {
                        if (dComp1 > 0)
                        {
                            leftBot = leftBot.WithX(comp1);
                            rightTop = rightTop.WithX(l);
                        }
                        else if (dComp1 < 0)
                        {
                            leftBot = leftBot.WithX(f);
                            rightTop = rightTop.WithX(comp1);
                        }
                    }
                    else if (index == 2)
                    {
                        if (dComp1 > 0)
                        {
                            leftBot = leftBot.WithY(comp1);
                            rightTop = rightTop.WithY(l);
                        }
                        else if (dComp1 < 0)
                        {
                            leftBot = leftBot.WithY(f);
                            rightTop = rightTop.WithY(comp1);
                        }
                    }
                }
            }
            else if (i == up)
            {
                if (index == 1)
                {
                    leftBot = leftBot.WithX(comp1);
                    rightTop = rightTop.WithX(bSplC.Knot(i));
                }
                else if (index == 2)
                {
                    leftBot = leftBot.WithY(comp1);
                    rightTop = rightTop.WithY(bSplC.Knot(i));
                }
            }
        }
    }

    private static void Locate1Coord(int index,
                                     in Point2D uv,
                                     in Vector2D duv,
                                     global::Kerf.Geometry.BSplineSurface bSplS,
                                     out bool dIsNull,
                                     ref Point2D leftBot,
                                     ref Point2D rightTop)
    {
        double comp1 = 0, dComp1 = 0;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        int i = 1, up = 0, up1, up2, down = 0, down1, down2;
        double cur = 0.0;

        dIsNull = false;

        up1 = bSplS.LastUKnotIndex();
        down1 = bSplS.FirstUKnotIndex();
        up2 = bSplS.LastVKnotIndex();
        down2 = bSplS.FirstVKnotIndex();

        if (index == 1)
        {
            i = down1;
            comp1 = uv.X();
            dComp1 = duv.X();
            up = up1;
            down = down1;
            while ((Math.Abs(bSplS.UKnot(i) - comp1) > tol) && (i != up1))
            {
                i++;
            }
            cur = bSplS.UKnot(i);
        }
        else if (index == 2)
        {
            i = down2;
            comp1 = uv.Y();
            dComp1 = duv.Y();
            up = up2;
            down = down2;
            while ((Math.Abs(bSplS.VKnot(i) - comp1) > tol) && (i != up2))
            {
                i++;
            }
            cur = bSplS.VKnot(i);
        }

        if (Math.Abs(comp1 - cur) <= tol)
        {
            int bnd1 = down, bnd2 = up;
            if (index == 1)
            {
                NCollection_Array1<double> arr1 = bSplS.InternalUKnots();
                FindBounds(arr1, cur, duv.X(), ref bnd1, ref bnd2, out dIsNull);
            }
            else if (index == 2)
            {
                NCollection_Array1<double> arr2 = bSplS.InternalVKnots();
                FindBounds(arr2, cur, duv.Y(), ref bnd1, ref bnd2, out dIsNull);
            }

            ReverseParam(bnd1, bnd2, out bnd1, out bnd2);

            if (!dIsNull)
            {
                if (index == 1)
                {
                    leftBot = leftBot.WithX(bSplS.UKnot(bnd1));
                    rightTop = rightTop.WithX(bSplS.UKnot(bnd2));
                }
                else if (index == 2)
                {
                    leftBot = leftBot.WithY(bSplS.VKnot(bnd1));
                    rightTop = rightTop.WithY(bSplS.VKnot(bnd2));
                }
            }
        }
        else //*********if Coord != Knot
        {
            if ((index == 1) && (comp1 < bSplS.UKnot(down)))
            {
                leftBot = leftBot.WithX(bSplS.UKnot(down));
                rightTop = rightTop.WithX(bSplS.UKnot(down + 1));
                return;
            }
            else if ((index == 2) && (comp1 < bSplS.VKnot(down)))
            {
                leftBot = leftBot.WithY(bSplS.VKnot(down));
                rightTop = rightTop.WithY(bSplS.VKnot(down + 1));
                return;
            }
            else if ((index == 1) && (comp1 > bSplS.UKnot(up)))
            {
                rightTop = rightTop.WithX(bSplS.UKnot(up - 1));
                leftBot = leftBot.WithX(bSplS.UKnot(up));
                return;
            }
            else if ((index == 2) && (comp1 > bSplS.VKnot(up)))
            {
                rightTop = rightTop.WithY(bSplS.VKnot(up - 1));
                leftBot = leftBot.WithY(bSplS.VKnot(up));
                return;
            }
            else
            {
                double f = 0.0, l = 1.0;
                if (index == 1)
                {
                    f = bSplS.UKnot(down);
                    l = bSplS.UKnot(up);
                }
                else if (index == 2)
                {
                    f = bSplS.VKnot(down);
                    l = bSplS.VKnot(up);
                }
                i = down;
                if ((!(comp1 < f)) && (!(comp1 > l)))
                {
                    if (index == 1)
                    {
                        while ((((f = bSplS.UKnot(i)) >= comp1) || ((l = bSplS.UKnot(i + 1)) <= comp1)) && (i < up))
                        {
                            i++;
                        }
                    }
                    else if (index == 2)
                    {
                        while ((((f = bSplS.VKnot(i)) >= comp1) || ((l = bSplS.VKnot(i + 1)) <= comp1)) && (i < up))
                        {
                            i++;
                        }
                    }
                }
                else
                {
                    ReverseParam(f, l, out f, out l);
                }

                if (i != up)
                {
                    if (Math.Abs(dComp1) > tol)
                    {
                        if (index == 1)
                        {
                            if (dComp1 > 0)
                            {
                                leftBot = leftBot.WithX(comp1);
                                rightTop = rightTop.WithX(l);
                            }
                            else if (dComp1 < 0)
                            {
                                leftBot = leftBot.WithX(f);
                                rightTop = rightTop.WithX(comp1);
                            }
                        }
                        else if (index == 2)
                        {
                            if (dComp1 > 0)
                            {
                                leftBot = leftBot.WithY(comp1);
                                rightTop = rightTop.WithY(l);
                            }
                            else if (dComp1 < 0)
                            {
                                leftBot = leftBot.WithY(f);
                                rightTop = rightTop.WithY(comp1);
                            }
                        }
                    }
                    else
                    {
                        if (Math.Abs(dComp1) < tol)
                        {
                            if (index == 1)
                            {
                                leftBot = leftBot.WithX(f);
                                rightTop = rightTop.WithX(l);
                            }
                            else if (index == 2)
                            {
                                leftBot = leftBot.WithY(f);
                                rightTop = rightTop.WithY(l);
                            }
                        }
                    }
                }
                else if (i == up)
                {
                    if (index == 1)
                    {
                        leftBot = leftBot.WithX(comp1);
                        rightTop = rightTop.WithX(bSplS.UKnot(i));
                    }
                    else if (index == 2)
                    {
                        leftBot = leftBot.WithY(comp1);
                        rightTop = rightTop.WithY(bSplS.VKnot(i));
                    }
                }
            }
        }
    }

    private static void Locate2Coord(int index,
                                     in Point2D uv,
                                     in Vector2D duv,
                                     double i1,
                                     double i2,
                                     ref Point2D leftBot,
                                     ref Point2D rightTop)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        double comp1 = 0, dComp1 = 0;
        if (index == 1)
        {
            comp1 = uv.X();
            dComp1 = duv.X();
        }
        else if (index == 2)
        {
            comp1 = uv.Y();
            dComp1 = duv.Y();
        }

        if ((comp1 != i1) && (comp1 != i2))
        {
            if (Math.Abs(dComp1) > tol)
            {
                if (dComp1 < 0)
                {
                    if (index == 1)
                    {
                        leftBot = leftBot.WithX(i1);
                        rightTop = rightTop.WithX(comp1);
                    }
                    if (index == 2)
                    {
                        leftBot = leftBot.WithY(i1);
                        rightTop = rightTop.WithY(comp1);
                    }
                }
                else if (dComp1 > 0)
                {
                    if (index == 1)
                    {
                        leftBot = leftBot.WithX(comp1);
                        rightTop = rightTop.WithX(i2);
                    }
                    if (index == 2)
                    {
                        leftBot = leftBot.WithY(comp1);
                        rightTop = rightTop.WithY(i2);
                    }
                }
                else
                {
                    if (index == 1)
                    {
                        leftBot = leftBot.WithX(i1);
                        rightTop = rightTop.WithX(i2);
                    }
                    if (index == 2)
                    {
                        leftBot = leftBot.WithY(i1);
                        rightTop = rightTop.WithY(i2);
                    }
                }
            }
            else if (Math.Abs(dComp1) <= tol)
            {
                if (index == 1)
                {
                    leftBot = leftBot.WithX(i1);
                    rightTop = rightTop.WithX(i2);
                }
                if (index == 2)
                {
                    leftBot = leftBot.WithY(i1);
                    rightTop = rightTop.WithY(i2);
                }
            }
        }
        else if (Math.Abs(comp1 - i1) < tol)
        {
            if (index == 1)
            {
                leftBot = leftBot.WithX(i1);
                rightTop = rightTop.WithX(i2);
            }
            if (index == 2)
            {
                leftBot = leftBot.WithY(i1);
                rightTop = rightTop.WithY(i2);
            }
        }
        else if (Math.Abs(comp1 - i2) < tol)
        {
            if (index == 1)
            {
                leftBot = leftBot.WithX(i1);
                rightTop = rightTop.WithX(i2);
            }
            if (index == 2)
            {
                leftBot = leftBot.WithY(i1);
                rightTop = rightTop.WithY(i2);
            }
        }
    }

    private static void Locate2Coord(int index,
                                     in Point2D uv,
                                     in Vector2D duv,
                                     global::Kerf.Geometry.BSplineSurface bSplS,
                                     NCollection_Array1<double> arr,
                                     ref Point2D leftBot,
                                     ref Point2D rightTop)
    {
        double comp = 0, dComp = 0, tmp1 = 0.0, tmp2 = 0.0;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        int n = 0, nUp = 0, nLo = 0;
        if (index == 1)
        {
            comp = uv.X();
            dComp = duv.Y();
            nUp = bSplS.LastUKnotIndex();
            nLo = bSplS.FirstUKnotIndex();
        }
        if (index == 2)
        {
            comp = uv.Y();
            dComp = duv.X();
            nUp = bSplS.LastVKnotIndex();
            nLo = bSplS.FirstVKnotIndex();
        }

        if ((dComp > 0) && (Math.Abs(dComp) > tol))
        {
            Hunt(arr, comp, ref n);
            if (n >= nUp)
            {
                // limit case: Hunt() caught upper knot. Take the last span.
                n = nUp - 1;
            }
            if (index == 1)
            {
                tmp1 = bSplS.UKnot(n);
                tmp2 = bSplS.UKnot(n + 1);
            }
            else if (index == 2)
            {
                tmp1 = bSplS.VKnot(n);
                tmp2 = bSplS.VKnot(n + 1);
            }

            ReverseParam(tmp1, tmp2, out tmp1, out tmp2);

            if (index == 1)
            {
                leftBot = leftBot.WithX(tmp1);
                rightTop = rightTop.WithX(tmp2);
            }
            else if (index == 2)
            {
                leftBot = leftBot.WithY(tmp1);
                rightTop = rightTop.WithY(tmp2);
            }
        }
        else if ((dComp < 0) && (Math.Abs(dComp) > tol))
        {
            Hunt(arr, comp, ref n);
            if (n <= nLo)
            {
                // limit case: Hunt() caught lower knot. Take the first span.
                n = nLo + 1;
            }
            if (index == 1)
            {
                tmp1 = bSplS.UKnot(n - 1);
                tmp2 = bSplS.UKnot(n);
            }
            else if (index == 2)
            {
                tmp1 = bSplS.VKnot(n - 1);
                tmp2 = bSplS.VKnot(n);
            }

            ReverseParam(tmp1, tmp2, out tmp1, out tmp2);

            if (index == 1)
            {
                leftBot = leftBot.WithX(tmp1);
                rightTop = rightTop.WithX(tmp2);
            }
            else if (index == 2)
            {
                leftBot = leftBot.WithY(tmp1);
                rightTop = rightTop.WithY(tmp2);
            }
        }
    }

    /// <summary>An empty adaptor; load a surface and a curve before use.</summary>
    public CurveOnSurface()
    {
        _type = OtherCurve;
        _intCont = CN;
    }

    /// <summary>An adaptor on the surface S; load a curve before use.</summary>
    public CurveOnSurface(SurfaceAdaptorBase s)
    {
        _type = OtherCurve;
        _intCont = CN;
        Load(s);
    }

    /// <summary>The curve C on the surface S.</summary>
    public CurveOnSurface(Curve2DAdaptorBase c, SurfaceAdaptorBase s)
    {
        _type = OtherCurve;
        _intCont = CN;
        Load(s);
        Load(c);
    }

    /// <summary>A copy with shallow copies of the curve and surface adaptors; the interval cache is shared.</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        CurveOnSurface copy = new CurveOnSurface();

        if (_surface is not null)
        {
            copy._surface = _surface.ShallowCopy();
        }
        if (_curve is not null)
        {
            copy._curve = _curve.ShallowCopy();
        }
        copy._type = _type;
        copy._circ = _circ;
        copy._lin = _lin;
        if (_firstSurf is not null)
        {
            copy._firstSurf = _firstSurf.ShallowCopy();
        }
        if (_lastSurf is not null)
        {
            copy._lastSurf = _lastSurf.ShallowCopy();
        }
        // OCCT shares the handle; NbIntervals replaces the sequence rather than changing it, so sharing is safe.
        copy._intervals = _intervals;
        copy._intCont = _intCont;

        return copy;
    }

    /// <summary>Changes the surface.</summary>
    public void Load(SurfaceAdaptorBase s)
    {
        _surface = s;
        if (_curve is not null)
        {
            EvalKPart();
        }
    }

    /// <summary>Changes the 2D curve.</summary>
    public void Load(Curve2DAdaptorBase c)
    {
        _curve = c;
        if (_surface is null)
        {
            return;
        }

        EvalKPart();

        SurfaceKind sType = _surface.GetType();
        if (sType == SurfaceKind.OffsetSurface)
        {
            sType = _surface.BasisSurface().GetType();
        }
        if (sType == SurfaceKind.BSplineSurface || sType == SurfaceOfExtrusion
            || sType == SurfaceOfRevolution)
        {
            EvalFirstLastSurf();
        }
    }

    /// <summary>Loads both the curve and the surface.</summary>
    public void Load(Curve2DAdaptorBase c, SurfaceAdaptorBase s)
    {
        Load(c);
        Load(s);
    }

    /// <summary>The 2D curve (null if none is loaded).</summary>
    public Curve2DAdaptorBase? GetCurve() => _curve;

    /// <summary>The surface (null if none is loaded).</summary>
    public SurfaceAdaptorBase? GetSurface() => _surface;

    /// <summary>
    /// A reference to the stored 2D curve, as OCCT's <c>ChangeCurve()</c>. Assigning through it does not
    /// recompute the special cases; use <see cref="Load(Curve2DAdaptorBase)"/> for that.
    /// </summary>
    public ref Curve2DAdaptorBase? ChangeCurve() => ref _curve;

    /// <summary>A reference to the stored surface, as OCCT's <c>ChangeSurface()</c> (see <see cref="ChangeCurve"/>).</summary>
    public ref SurfaceAdaptorBase? ChangeSurface() => ref _surface;

    public override double FirstParameter() => _curve!.FirstParameter();

    public override double LastParameter() => _curve!.LastParameter();

    public override Continuity Continuity()
    {
        Continuity contC = _curve!.Continuity();
        Continuity contSu = _surface!.UContinuity();
        if (contSu < contC)
        {
            contC = contSu;
        }
        Continuity contSv = _surface.VContinuity();
        if (contSv < contC)
        {
            contC = contSv;
        }

        return contC;
    }

    // Auxiliary: adds roots of equation to sorted sequence of parameters
    // along curve, keeping it sorted and avoiding repetitions (within tolerance Tol)
    private static void AddIntervals(NCollection_HSequence<double> parameters, FunctionRoots roots, double tol)
    {
        if (!roots.IsDone || roots.IsAllNull)
        {
            return;
        }

        int nsol = roots.SolutionCount;
        for (int i = 1; i <= nsol; i++)
        {
            double param = roots.Value(i);
            if (param - parameters.Value(1) < tol)
            { // skip param if equal to or less than theParameters(1)
                continue;
            }
            for (int j = 2; j <= parameters.Length(); ++j)
            {
                double delta = parameters.Value(j) - param;
                if (delta > tol)
                {
                    parameters.InsertBefore(j, param);
                    break;
                }
                else if (delta >= -tol)
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
    public override int NbIntervals(Continuity s)
    {
        if (s == _intCont && _intervals is not null)
        {
            return _intervals.Length() - 1;
        }

        int nu, nv, nc;
        nu = _surface!.NbUIntervals(s);
        nv = _surface.NbVIntervals(s);
        nc = _curve!.NbIntervals(s);

        // Allocate the memory for arrays TabU, TabV, TabC only once using the buffer TabBuf.
        NCollection_Array1<double> tabBuf = new(1, nu + nv + nc + 3);
        NCollection_Array1<double> tabU = new(tabBuf, 1, 1, nu + 1);
        NCollection_Array1<double> tabV = new(tabBuf, nu + 2, 1, nv + 1);
        NCollection_Array1<double> tabC = new(tabBuf, nu + nv + 3, 1, nc + 1);

        int nbSample = 20;
        double u, v, tdeb, tfin;
        tdeb = _curve.FirstParameter();
        tfin = _curve.LastParameter();

        _curve.Intervals(tabC, s);

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;

        // sorted sequence of parameters defining continuity intervals;
        // started with own intervals of curve and completed by
        // additional points coming from surface discontinuities
        NCollection_HSequence<double> intervals = new NCollection_HSequence<double>();
        for (int i = 1; i <= nc + 1; i++)
        {
            intervals.Append(tabC[i]);
        }

        if (nu > 1)
        {
            _surface.UIntervals(tabU, s);
            for (int iu = 2; iu <= nu; iu++)
            {
                u = tabU.Value(iu);
                InterFunc func = new InterFunc(_curve, u, 1);
                FunctionRoots resol = new FunctionRoots(func, tdeb, tfin, nbSample, tol, tol, tol, 0.0);
                AddIntervals(intervals, resol, tol);
            }
        }
        if (nv > 1)
        {
            _surface.VIntervals(tabV, s);
            for (int iv = 2; iv <= nv; iv++)
            {
                v = tabV.Value(iv);
                InterFunc func = new InterFunc(_curve, v, 2);
                FunctionRoots resol = new FunctionRoots(func, tdeb, tfin, nbSample, tol, tol, tol, 0.0);
                AddIntervals(intervals, resol, tol);
            }
        }

        // for case intervals==1 and first point == last point SequenceOfReal
        // contains only one value, therefore it is necessary to add second
        // value into aIntervals which will be equal first value.
        if (intervals.Length() == 1)
        {
            intervals.Append(intervals.Value(1));
        }

        _intervals = intervals;
        _intCont = s;
        return _intervals.Length() - 1;
    }

    /// <summary>Stores in T the bounds of the intervals of continuity S; T must have NbIntervals(S) + 1 values.</summary>
    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        NbIntervals(s);
        if (t.Length() != _intervals!.Length())
        {
            throw new ProgramErrorException("Error: Wrong size of array buffer in call to Adaptor3d_CurveOnSurface::Intervals");
        }
        for (int i = 1; i <= _intervals.Length(); i++)
        {
            // OCCT writes T(i), assuming the array starts at 1.
            t[i] = _intervals.Value(i);
        }
    }

    /// <summary>A new adaptor on the same surface, of the curve trimmed to [First, Last].</summary>
    public override CurveAdaptorBase Trim(double first, double last, double tol)
    {
        CurveOnSurface hcs = new CurveOnSurface();
        hcs.Load(_surface!);
        hcs.Load(_curve!.Trim(first, last, tol));
        return hcs;
    }

    public override bool IsClosed => _curve!.IsClosed;

    public override bool IsPeriodic
    {
        get
        {
            if (_type == global::Kerf.Geometry.CurveKind.Circle || _type == global::Kerf.Geometry.CurveKind.Ellipse)
            {
                return true;
            }

            return _curve!.IsPeriodic;
        }
    }

    public override double Period()
    {
        if (_type == global::Kerf.Geometry.CurveKind.Circle || _type == global::Kerf.Geometry.CurveKind.Ellipse)
        {
            return (2.0 * M_PI);
        }

        return _curve!.Period();
    }

    /// <summary>The point of parameter theU.</summary>
    public sealed override Point3D EvalD0(double u)
    {
        Point3D p;
        Point2D puv;

        if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            p = global::Kerf.Engine.Geometry.ElCLib.Value(u, _lin);
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Circle)
        {
            p = global::Kerf.Engine.Geometry.ElCLib.Value(u, _circ);
        }
        else
        {
            _curve!.D0(u, out puv);
            _surface!.D0(puv.X(), puv.Y(), out p);
        }

        return p;
    }

    /// <summary>The point and first derivative at theU.</summary>
    public sealed override Curve3D.ResD1 EvalD1(double u)
    {
        Point2D puv;
        Vector2D duv;
        Vector3D d1U, d1V;

        Curve3D.ResD1 res = new Curve3D.ResD1();

        double fp = _curve!.FirstParameter();
        double lp = _curve.LastParameter();

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        if ((Math.Abs(u - fp) < tol) && (_firstSurf is not null))
        {
            _curve.D1(u, out puv, out duv);
            _firstSurf.D1(puv.X(), puv.Y(), out res.Point, out d1U, out d1V);
            res.D1 = Vector3D.LinearForm(duv.X(), d1U, duv.Y(), d1V);
        }
        else if ((Math.Abs(u - lp) < tol) && (_lastSurf is not null))
        {
            _curve.D1(u, out puv, out duv);
            _lastSurf.D1(puv.X(), puv.Y(), out res.Point, out d1U, out d1V);
            res.D1 = Vector3D.LinearForm(duv.X(), d1U, duv.Y(), d1V);
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            global::Kerf.Engine.Geometry.ElCLib.D1(u, _lin, out res.Point, out res.D1);
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Circle)
        {
            global::Kerf.Engine.Geometry.ElCLib.D1(u, _circ, out res.Point, out res.D1);
        }
        else
        {
            _curve.D1(u, out puv, out duv);
            _surface!.D1(puv.X(), puv.Y(), out res.Point, out d1U, out d1V);
            res.D1 = Vector3D.LinearForm(duv.X(), d1U, duv.Y(), d1V);
        }

        return res;
    }

    /// <summary>The point and first two derivatives at theU.</summary>
    public sealed override Curve3D.ResD2 EvalD2(double u)
    {
        Point2D uv;
        Vector2D dw, d2W;
        Vector3D d1U, d1V, d2U, d2V, d2UV;

        Curve3D.ResD2 res = new Curve3D.ResD2();

        double fp = _curve!.FirstParameter();
        double lp = _curve.LastParameter();

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        if ((Math.Abs(u - fp) < tol) && (_firstSurf is not null))
        {
            _curve.D2(u, out uv, out dw, out d2W);
            _firstSurf.D2(uv.X(), uv.Y(), out res.Point, out d1U, out d1V, out d2U, out d2V, out d2UV);

            res.D1 = Vector3D.LinearForm(dw.X(), d1U, dw.Y(), d1V);
            res.D2 = Vector3D.LinearForm(d2W.X(), d1U, d2W.Y(), d1V, 2.0 * dw.X() * dw.Y(), d2UV);
            res.D2 = Vector3D.LinearForm(dw.X() * dw.X(), d2U, dw.Y() * dw.Y(), d2V, res.D2);
        }
        else if ((Math.Abs(u - lp) < tol) && (_lastSurf is not null))
        {
            _curve.D2(u, out uv, out dw, out d2W);
            _lastSurf.D2(uv.X(), uv.Y(), out res.Point, out d1U, out d1V, out d2U, out d2V, out d2UV);

            res.D1 = Vector3D.LinearForm(dw.X(), d1U, dw.Y(), d1V);
            res.D2 = Vector3D.LinearForm(d2W.X(), d1U, d2W.Y(), d1V, 2.0 * dw.X() * dw.Y(), d2UV);
            res.D2 = Vector3D.LinearForm(dw.X() * dw.X(), d2U, dw.Y() * dw.Y(), d2V, res.D2);
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            global::Kerf.Engine.Geometry.ElCLib.D1(u, _lin, out res.Point, out res.D1);
            res.D2 = res.D2.WithCoord(0.0, 0.0, 0.0);
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Circle)
        {
            global::Kerf.Engine.Geometry.ElCLib.D2(u, _circ, out res.Point, out res.D1, out res.D2);
        }
        else
        {
            _curve.D2(u, out uv, out dw, out d2W);
            _surface!.D2(uv.X(), uv.Y(), out res.Point, out d1U, out d1V, out d2U, out d2V, out d2UV);

            res.D1 = Vector3D.LinearForm(dw.X(), d1U, dw.Y(), d1V);
            res.D2 = Vector3D.LinearForm(d2W.X(), d1U, d2W.Y(), d1V, 2.0 * dw.X() * dw.Y(), d2UV);
            res.D2 = Vector3D.LinearForm(dw.X() * dw.X(), d2U, dw.Y() * dw.Y(), d2V, res.D2);
        }

        return res;
    }

    /// <summary>The point and first three derivatives at theU.</summary>
    public sealed override Curve3D.ResD3 EvalD3(double u)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        Point2D uv;
        Vector2D dw, d2W, d3W;
        Vector3D d1U, d1V, d2U, d2V, d2UV, d3U, d3V, d3UUV, d3UVV;

        Curve3D.ResD3 res = new Curve3D.ResD3();

        double fp = _curve!.FirstParameter();
        double lp = _curve.LastParameter();

        if ((Math.Abs(u - fp) < tol) && (_firstSurf is not null))
        {
            _curve.D3(u, out uv, out dw, out d2W, out d3W);
            _firstSurf.D3(uv.X(), uv.Y(), out res.Point, out d1U, out d1V, out d2U, out d2V, out d2UV, out d3U, out d3V,
                           out d3UUV, out d3UVV);
            res.D1 = Vector3D.LinearForm(dw.X(), d1U, dw.Y(), d1V);
            res.D2 = Vector3D.LinearForm(d2W.X(), d1U, d2W.Y(), d1V, 2.0 * dw.X() * dw.Y(), d2UV);
            res.D2 = Vector3D.LinearForm(dw.X() * dw.X(), d2U, dw.Y() * dw.Y(), d2V, res.D2);
            res.D3 = SetLinearForm(dw, d2W, d3W, d1U, d1V, d2U, d2V, d2UV, d3U, d3V, d3UUV, d3UVV);
        }
        else if ((Math.Abs(u - lp) < tol) && (_lastSurf is not null))
        {
            _curve.D3(u, out uv, out dw, out d2W, out d3W);
            _lastSurf.D3(uv.X(), uv.Y(), out res.Point, out d1U, out d1V, out d2U, out d2V, out d2UV, out d3U, out d3V,
                          out d3UUV, out d3UVV);
            res.D1 = Vector3D.LinearForm(dw.X(), d1U, dw.Y(), d1V);
            res.D2 = Vector3D.LinearForm(d2W.X(), d1U, d2W.Y(), d1V, 2.0 * dw.X() * dw.Y(), d2UV);
            res.D2 = Vector3D.LinearForm(dw.X() * dw.X(), d2U, dw.Y() * dw.Y(), d2V, res.D2);
            res.D3 = SetLinearForm(dw, d2W, d3W, d1U, d1V, d2U, d2V, d2UV, d3U, d3V, d3UUV, d3UVV);
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Line)
        {
            global::Kerf.Engine.Geometry.ElCLib.D1(u, _lin, out res.Point, out res.D1);
            res.D2 = res.D2.WithCoord(0.0, 0.0, 0.0);
            res.D3 = res.D3.WithCoord(0.0, 0.0, 0.0);
        }
        else if (_type == global::Kerf.Geometry.CurveKind.Circle)
        {
            global::Kerf.Engine.Geometry.ElCLib.D3(u, _circ, out res.Point, out res.D1, out res.D2, out res.D3);
        }
        else
        {
            _curve.D3(u, out uv, out dw, out d2W, out d3W);
            _surface!.D3(uv.X(), uv.Y(), out res.Point, out d1U, out d1V, out d2U, out d2V, out d2UV, out d3U, out d3V,
                          out d3UUV, out d3UVV);
            res.D1 = Vector3D.LinearForm(dw.X(), d1U, dw.Y(), d1V);
            res.D2 = Vector3D.LinearForm(d2W.X(), d1U, d2W.Y(), d1V, 2.0 * dw.X() * dw.Y(), d2UV);
            res.D2 = Vector3D.LinearForm(dw.X() * dw.X(), d2U, dw.Y() * dw.Y(), d2V, res.D2);
            res.D3 = SetLinearForm(dw, d2W, d3W, d1U, d1V, d2U, d2V, d2UV, d3U, d3V, d3UUV, d3UVV);
        }

        return res;
    }

    /// <summary>The derivative of order theN (1 to 3) at theU.</summary>
    /// <exception cref="MethodNotImplementedException">theN is above 3.</exception>
    public sealed override Vector3D EvalDN(double u, int n)
    {
        switch (n)
        {
            case 1:
                return EvalD1(u).D1;
            case 2:
                return EvalD2(u).D2;
            case 3:
                return EvalD3(u).D3;
            default:
                throw new MethodNotImplementedException("Adaptor3d_CurveOnSurface:EvalDN");
        }
    }

    public override double Resolution(double r3d)
    {
        double ru, rv;
        ru = _surface!.UResolution(r3d);
        rv = _surface.VResolution(r3d);
        return _curve!.Resolution(Min(ru, rv));
    }

    public override CurveKind GetType() => _type;

    public override Line3D Line()
    {
        Standard_NoSuchObject_Raise_if(_type != global::Kerf.Geometry.CurveKind.Line, "Adaptor3d_CurveOnSurface::Line(): curve is not a line");
        return _lin;
    }

    public override Circle3D Circle()
    {
        // OCCT's message says Line() (cosmetic).
        Standard_NoSuchObject_Raise_if(_type != global::Kerf.Geometry.CurveKind.Circle, "Adaptor3d_CurveOnSurface::Line(): curve is not a circle");
        return _circ;
    }

    /// <summary>The ellipse, for an ellipse on a plane.</summary>
    public override Ellipse3D Ellipse() => To3d(_surface!.Plane(), _curve!.Ellipse());

    /// <summary>The hyperbola, for a hyperbola on a plane.</summary>
    public override Hyperbola3D Hyperbola() => To3d(_surface!.Plane(), _curve!.Hyperbola());

    /// <summary>The parabola, for a parabola on a plane.</summary>
    public override Parabola3D Parabola() => To3d(_surface!.Plane(), _curve!.Parabola());

    /// <summary>The degree of the 2D curve (not multiplied by the surface's).</summary>
    public override int Degree()
    {
        // on a parametric surface should multiply
        // return TheCurve2dTool::Degree(myCurve);

        return _curve!.Degree();
    }

    public override bool IsRational
    {
        get
        {
            return (_curve!.IsRational || _surface!.IsURational || _surface.IsVRational);
        }
    }

    public override int PoleCount
    {
        get
        {
            // on a parametric surface should multiply
            return _curve!.PoleCount;
        }
    }

    public override int KnotCount
    {
        get
        {
            if (_surface!.GetType() == Plane)
            {
                return _curve!.KnotCount;
            }
            else
            {
                throw new NoSuchObjectException();
            }
        }
    }

    /// <summary>The 3D Bezier curve of a 2D Bezier curve on a plane (a new curve).</summary>
    public override BezierCurve3D Bezier()
    {
        Standard_NoSuchObject_Raise_if(_surface!.GetType() != global::Kerf.Geometry.SurfaceKind.Plane, "Adaptor3d_CurveOnSurface : Bezier");

        BezierCurve2D bez2d = _curve!.Bezier();
        int nbPoles = bez2d.PoleCount;

        Plane3D plane = _surface.Plane();

        NCollection_Array1<Point3D> poles = new(1, nbPoles);
        for (int i = 1; i <= nbPoles; i++)
        {
            poles[i] = To3d(plane, bez2d.Pole(i));
        }
        BezierCurve3D bez;

        if (bez2d.IsRational)
        {
            NCollection_Array1<double> weights = bez2d.InternalWeightsArray();
            bez = new BezierCurve3D(poles, weights);
        }
        else
        {
            bez = new BezierCurve3D(poles);
        }
        return bez;
    }

    /// <summary>The 3D B-spline curve of a 2D B-spline curve on a plane (a new curve).</summary>
    public override BSplineCurve3D BSpline()
    {
        Standard_NoSuchObject_Raise_if(_surface!.GetType() != global::Kerf.Geometry.SurfaceKind.Plane, "Adaptor3d_CurveOnSurface : BSpline");

        BSplineCurve2D bsp2d = _curve!.BSpline();
        int nbPoles = bsp2d.PoleCount;

        Plane3D plane = _surface.Plane();

        NCollection_Array1<Point3D> poles = new(1, nbPoles);
        for (int i = 1; i <= nbPoles; i++)
        {
            poles[i] = To3d(plane, bsp2d.Pole(i));
        }

        NCollection_Array1<double> knots = bsp2d.InternalKnots();
        NCollection_Array1<int> mults = bsp2d.InternalMultiplicities();
        BSplineCurve3D bsp;

        if (bsp2d.IsRational)
        {
            NCollection_Array1<double> weights = bsp2d.InternalWeightsArray();
            bsp = new BSplineCurve3D(poles, weights, knots, mults, bsp2d.Degree(), bsp2d.IsPeriodic);
        }
        else
        {
            bsp = new BSplineCurve3D(poles, knots, mults, bsp2d.Degree(), bsp2d.IsPeriodic);
        }
        return bsp;
    }

    // Recognises lines and circles: curves of a plane, and the isoparametric lines of the elementary surfaces
    private void EvalKPart()
    {
        _type = OtherCurve;

        SurfaceKind sTy = _surface!.GetType();
        CurveKind cTy = _curve!.GetType();
        if (sTy == Plane)
        {
            _type = cTy;
            if (_type == global::Kerf.Geometry.CurveKind.Circle)
            {
                _circ = To3d(_surface.Plane(), _curve.Circle());
            }
            else if (_type == global::Kerf.Geometry.CurveKind.Line)
            {
                Point3D p;
                Vector3D v = new Vector3D();
                Point2D puv;
                Vector2D duv;
                _curve.D1(0.0, out puv, out duv);
                Vector3D d1U, d1V;
                _surface.D1(puv.X(), puv.Y(), out p, out d1U, out d1V);
                v = Vector3D.LinearForm(duv.X(), d1U, duv.Y(), d1V);
                _lin = new Line3D(p, new Direction3D(v));
            }
        }
        else
        {
            if (cTy == global::Kerf.Geometry.CurveKind.Line)
            {
                Direction2D d = _curve.Line().Direction();
                if (d.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                { // Iso V.
                    if (sTy == SurfaceKind.Sphere)
                    {
                        Point2D p = _curve.Line().Location();
                        if (Math.Abs(Math.Abs(p.Y()) - M_PI / 2.0) >= global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            _type = global::Kerf.Geometry.CurveKind.Circle;
                            Sphere sph = _surface.Sphere();
                            CoordinateSystem axis = sph.Position();
                            _circ = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(axis, sph.Radius(), p.Y());
                            Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                            Axis3D axeRev = new Axis3D(axis.Location(), dRev);
                            _circ = _circ.Rotated(axeRev, p.X());
                            if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                Frame3D ax = _circ.Position();
                                ax = ax.WithDirection(ax.Direction().Reversed());
                                _circ = _circ.WithPosition(ax);
                            }
                        }
                    }
                    else if (sTy == SurfaceKind.Cylinder)
                    {
                        _type = global::Kerf.Geometry.CurveKind.Circle;
                        Cylinder cyl = _surface.Cylinder();
                        Point2D p = _curve.Line().Location();
                        CoordinateSystem axis = cyl.Position();
                        _circ = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(axis, cyl.Radius(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        Axis3D axeRev = new Axis3D(axis.Location(), dRev);
                        _circ = _circ.Rotated(axeRev, p.X());
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Frame3D ax = _circ.Position();
                            ax = ax.WithDirection(ax.Direction().Reversed());
                            _circ = _circ.WithPosition(ax);
                        }
                    }
                    else if (sTy == SurfaceKind.Cone)
                    {
                        _type = global::Kerf.Geometry.CurveKind.Circle;
                        Cone cone = _surface.Cone();
                        Point2D p = _curve.Line().Location();
                        CoordinateSystem axis = cone.Position();
                        _circ = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(axis, cone.RefRadius(), cone.SemiAngle(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        Axis3D axeRev = new Axis3D(axis.Location(), dRev);
                        _circ = _circ.Rotated(axeRev, p.X());
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Frame3D ax = _circ.Position();
                            ax = ax.WithDirection(ax.Direction().Reversed());
                            _circ = _circ.WithPosition(ax);
                        }
                    }
                    else if (sTy == SurfaceKind.Torus)
                    {
                        _type = global::Kerf.Geometry.CurveKind.Circle;
                        Torus tore = _surface.Torus();
                        Point2D p = _curve.Line().Location();
                        CoordinateSystem axis = tore.Position();
                        _circ = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(axis, tore.MajorRadius(), tore.MinorRadius(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        Axis3D axeRev = new Axis3D(axis.Location(), dRev);
                        _circ = _circ.Rotated(axeRev, p.X());
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Frame3D ax = _circ.Position();
                            ax = ax.WithDirection(ax.Direction().Reversed());
                            _circ = _circ.WithPosition(ax);
                        }
                    }
                }
                else if (d.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                { // Iso U.
                    if (sTy == SurfaceKind.Sphere)
                    {
                        _type = global::Kerf.Geometry.CurveKind.Circle;
                        Sphere sph = _surface.Sphere();
                        Point2D p = _curve.Line().Location();
                        CoordinateSystem axis = sph.Position();
                        // Compute the iso 0.
                        _circ = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(axis, sph.Radius(), 0.0);

                        // Same-parametrization (circle rotation - Y offset)
                        Direction3D dRev = axis.XDirection().Crossed(axis.Direction());
                        Axis3D axeRev = new Axis3D(axis.Location(), dRev);
                        _circ = _circ.Rotated(axeRev, p.Y());

                        // Transform to iso U ( = P.X())
                        dRev = axis.XDirection().Crossed(axis.YDirection());
                        axeRev = new Axis3D(axis.Location(), dRev);
                        _circ = _circ.Rotated(axeRev, p.X());

                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Frame3D ax = _circ.Position();
                            ax = ax.WithDirection(ax.Direction().Reversed());
                            _circ = _circ.WithPosition(ax);
                        }
                    }
                    else if (sTy == SurfaceKind.Cylinder)
                    {
                        _type = global::Kerf.Geometry.CurveKind.Line;
                        Cylinder cyl = _surface.Cylinder();
                        Point2D p = _curve.Line().Location();
                        _lin = global::Kerf.Engine.Geometry.ElSLib.CylinderUIso(cyl.Position(), cyl.Radius(), p.X());
                        Vector3D tr = new Vector3D(_lin.Direction());
                        tr = tr.Multiplied(p.Y());
                        _lin = _lin.Translated(tr);
                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            _lin = _lin.Reversed();
                        }
                    }
                    else if (sTy == SurfaceKind.Cone)
                    {
                        _type = global::Kerf.Geometry.CurveKind.Line;
                        Cone cone = _surface.Cone();
                        Point2D p = _curve.Line().Location();
                        _lin = global::Kerf.Engine.Geometry.ElSLib.ConeUIso(cone.Position(), cone.RefRadius(), cone.SemiAngle(), p.X());
                        Vector3D tr = new Vector3D(_lin.Direction());
                        tr = tr.Multiplied(p.Y());
                        _lin = _lin.Translated(tr);
                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            _lin = _lin.Reversed();
                        }
                    }
                    else if (sTy == SurfaceKind.Torus)
                    {
                        _type = global::Kerf.Geometry.CurveKind.Circle;
                        Torus tore = _surface.Torus();
                        Point2D p = _curve.Line().Location();
                        CoordinateSystem axis = tore.Position();
                        _circ = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(axis, tore.MajorRadius(), tore.MinorRadius(), p.X());
                        _circ = _circ.Rotated(_circ.Axis(), p.Y());

                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Frame3D ax = _circ.Position();
                            ax = ax.WithDirection(ax.Direction().Reversed());
                            _circ = _circ.WithPosition(ax);
                        }
                    }
                }
            }
        }
    }

    // Finds the patches of the surface around the curve's ends, used to evaluate there on the right side of a knot
    private void EvalFirstLastSurf()
    {
        double firstPar, lastPar;
        Point2D uv, leftBot = new Point2D(), rightTop = new Point2D();
        Vector2D duv;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 10;
        bool ok = true;

        firstPar = _curve!.FirstParameter();
        _curve.D1(firstPar, out uv, out duv);

        if (duv.Magnitude() <= tol)
        {
            ok = false;
        }

        if (ok)
        {
            switch (_surface!.GetType())
            {
                case SurfaceKind.BSplineSurface:
                    LocatePart(uv, duv, _surface, ref leftBot, ref rightTop);
                    break;
                case SurfaceOfRevolution:
                case SurfaceOfExtrusion:
                    ok = LocatePart_RevExt(uv, duv, _surface, ref leftBot, ref rightTop);
                    break;
                case SurfaceKind.OffsetSurface:
                    ok = LocatePart_Offset(uv, duv, _surface, ref leftBot, ref rightTop);
                    break;
                default:
                    throw new MethodNotImplementedException("Adaptor3d_CurveOnSurface::EvalFirstLastSurf");
            }
        }

        if (ok)
        {
            CompareBounds(ref leftBot, ref rightTop); // SVV

            _firstSurf = _surface!.UTrim(leftBot.X(), rightTop.X(), tol);
            _firstSurf = _firstSurf.VTrim(leftBot.Y(), rightTop.Y(), tol);
        }
        else
        {
            _firstSurf = _surface;
        }

        lastPar = _curve.LastParameter();
        ok = true;
        _curve.D1(lastPar, out uv, out duv);
        duv = duv.Reversed(); // We want the other part

        if (duv.Magnitude() <= tol)
        {
            ok = false;
        }

        if (ok)
        {
            switch (_surface!.GetType())
            {
                case SurfaceKind.BSplineSurface:
                    LocatePart(uv, duv, _surface, ref leftBot, ref rightTop);
                    break;
                case SurfaceOfRevolution:
                case SurfaceOfExtrusion:
                    ok = LocatePart_RevExt(uv, duv, _surface, ref leftBot, ref rightTop);
                    break;
                case SurfaceKind.OffsetSurface:
                    ok = LocatePart_Offset(uv, duv, _surface, ref leftBot, ref rightTop);
                    break;
                default:
                    throw new MethodNotImplementedException("Adaptor3d_CurveOnSurface::EvalFirstLastSurf");
            }
        }

        if (ok)
        {
            CompareBounds(ref leftBot, ref rightTop); // SVV

            _lastSurf = _surface!.UTrim(leftBot.X(), rightTop.X(), tol);
            _lastSurf = _lastSurf.VTrim(leftBot.Y(), rightTop.Y(), tol);
        }
        else
        {
            _lastSurf = _surface;
        }
    }

    private bool LocatePart_RevExt(in Point2D uv, in Vector2D duv, SurfaceAdaptorBase s, ref Point2D leftBot,
                                   ref Point2D rightTop)
    {
        CurveAdaptorBase ahc = s.BasisCurve();

        if (ahc.GetType() == BSplineCurve)
        {
            BSplineCurve3D bSplC;
            bSplC = ahc.BSpline();

            if ((s.GetType()) == SurfaceOfExtrusion)
            {
                Locate1Coord(1, uv, duv, bSplC, ref leftBot, ref rightTop);
                Locate2Coord(2, uv, duv, s.FirstVParameter(), s.LastVParameter(), ref leftBot, ref rightTop);
            }
            else if ((s.GetType()) == SurfaceOfRevolution)
            {
                Locate1Coord(2, uv, duv, bSplC, ref leftBot, ref rightTop);
                Locate2Coord(1, uv, duv, s.FirstUParameter(), s.LastUParameter(), ref leftBot, ref rightTop);
            }

            double u1, u2, v1, v2;
            ReverseParam(leftBot.X(), rightTop.X(), out u1, out u2);
            leftBot = leftBot.WithX(u1);
            rightTop = rightTop.WithX(u2);
            ReverseParam(leftBot.Y(), rightTop.Y(), out v1, out v2);
            leftBot = leftBot.WithY(v1);
            rightTop = rightTop.WithY(v2);
            return true;
        }
        return false;
    }

    private bool LocatePart_Offset(in Point2D uv, in Vector2D duv, SurfaceAdaptorBase s, ref Point2D leftBot,
                                   ref Point2D rightTop)
    {
        bool ok = true;
        SurfaceAdaptorBase ahs;
        ahs = s.BasisSurface();
        SurfaceKind basisSType = ahs.GetType();
        switch (basisSType)
        {
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
                ok = LocatePart_RevExt(uv, duv, ahs, ref leftBot, ref rightTop);
                break;

            case SurfaceKind.BSplineSurface:
                LocatePart(uv, duv, ahs, ref leftBot, ref rightTop);
                break;

            default:
                ok = false;
                break;
        }
        return ok;
    }

    private void LocatePart(in Point2D uv, in Vector2D duv, SurfaceAdaptorBase s, ref Point2D leftBot,
                            ref Point2D rightTop)
    {
        global::Kerf.Geometry.BSplineSurface bSplS;
        bSplS = s.BSpline();
        bool duIsNull, dvIsNull;

        Locate1Coord(1, uv, duv, bSplS, out duIsNull, ref leftBot, ref rightTop);
        Locate1Coord(2, uv, duv, bSplS, out dvIsNull, ref leftBot, ref rightTop);

        if ((duIsNull) && (!dvIsNull))
        {
            NCollection_Array1<double> arrU = bSplS.InternalUKnots();
            Locate2Coord(1, uv, duv, bSplS, arrU, ref leftBot, ref rightTop);
        }
        else if ((dvIsNull) && (!duIsNull))
        {
            NCollection_Array1<double> arrV = bSplS.InternalVKnots();
            Locate2Coord(2, uv, duv, bSplS, arrV, ref leftBot, ref rightTop);
        }
    }
}
