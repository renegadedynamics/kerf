// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The 2D polygon of a walking or restriction line in the parametric space of one of its surfaces, with its
/// deflection (estimated from parabolas through consecutive points). The end points of a walking line are pushed
/// out by 1e-7 of the end segments.
/// </summary>
/// <remarks>
/// OCCT's unused field <c>pnt</c> is left out. OCCT leaves the line type uninitialised until a line is set; the
/// port's is IntPatch_Lin (and the queries fail on the missing line, as OCCT's would).
/// </remarks>
internal class PolyLine : Polygo
{
    //-- lbr le 12 juin : Ajout des fleches sur les Lines
    //-- msv 13.03.2002 : compute deflection for WLine; Error() returns deflection
    private static double INITDEFLE => global::Kerf.Engine.Foundation.Precision.PConfusion() * 100.0;

    private PatchLineType _typ;
    private bool _onfirst;
    private WLine? _wpoly;
    private RLine? _rpoly;

    public PolyLine()
        : base(INITDEFLE)
    {
        _onfirst = false;
    }

    /// <summary>A polygon whose initial deflection is <paramref name="initDefle"/>.</summary>
    public PolyLine(double initDefle)
        : base(initDefle)
    {
        _onfirst = false;
    }

    /// <summary>Takes the points of a walking line on the first surface (<paramref name="onFirst"/>) or the second.</summary>
    public void SetWLine(bool onFirst, WLine line)
    {
        _typ = PatchLineType.Walking;
        _wpoly = line;
        _onfirst = onFirst;
        Prepare();
    }

    /// <summary>Takes the points of a restriction line on the first surface (<paramref name="onFirst"/>) or the second.</summary>
    public void SetRLine(bool onFirst, RLine line)
    {
        _typ = PatchLineType.Restriction;
        _rpoly = line;
        _onfirst = onFirst;
        Prepare();
    }

    private void Prepare()
    {
        int i;
        _box.SetVoid();
        int n = PointCount;
        double eps_2 = _error * _error;

        Point2D p1 = new(), p2 = new();
        if (n >= 3)
        {
            p1 = Point(1);
            p2 = Point(2);
        }
        for (i = 1; i <= n; i++)
        {
            Point2D p3 = Point(i);
            if (i >= 3)
            {
                Xy v13 = p3.XY() - p1.XY();
                Xy v12 = p2.XY() - p1.XY();
                double d13_2 = v13.SquareModulus(), d_2;
                if (d13_2 > eps_2)
                {
                    d_2 = v13.CrossSquareMagnitude(v12) / d13_2;
                }
                else
                {
                    d_2 = eps_2;
                }
                if (d_2 > _error * _error)
                {
                    // try to compute deflection more precisely using parabola interpolation
                    Xy v23 = p3.XY() - p2.XY();
                    double d12 = v12.Modulus(), d23 = v23.Modulus();
                    // compute parameter of P2 (assume parameters of P1,P3 are 0,1)
                    double tm = d12 / (d12 + d23);
                    if (tm > 0.1 && tm < 0.9)
                    {
                        tm -= (tm - 0.5) * 0.6;
                        double tm1mtm = tm * (1 - tm);
                        // coefficients of parabola
                        double ax = (tm * v13.X() - v12.X()) / tm1mtm;
                        double bx = (v12.X() - tm * tm * v13.X()) / tm1mtm;
                        double cx = p1.X;
                        double ay = (tm * v13.Y() - v12.Y()) / tm1mtm;
                        double By = (v12.Y() - tm * tm * v13.Y()) / tm1mtm;
                        double cy = p1.Y;
                        // equations of lines P1-P2 and P2-P3
                        double a1 = v12.Y() / d12;
                        double b1 = -v12.X() / d12;
                        double c1 = (p2.X * p1.Y - p1.X * p2.Y) / d12;
                        double a2 = v23.Y() / d23;
                        double b2 = -v23.X() / d23;
                        double c2 = (p3.X * p2.Y - p2.X * p3.Y) / d23;
                        // points on parabola with max deflection
                        double t1 = -0.5 * (a1 * bx + b1 * By) / (a1 * ax + b1 * ay);
                        double t2 = -0.5 * (a2 * bx + b2 * By) / (a2 * ax + b2 * ay);
                        double xt1 = ax * t1 * t1 + bx * t1 + cx;
                        double yt1 = ay * t1 * t1 + By * t1 + cy;
                        double xt2 = ax * t2 * t2 + bx * t2 + cx;
                        double yt2 = ay * t2 * t2 + By * t2 + cy;
                        // max deflection on segments P1-P2 and P2-P3
                        double d1 = Math.Abs(a1 * xt1 + b1 * yt1 + c1);
                        double d2 = Math.Abs(a2 * xt2 + b2 * yt2 + c2);
                        if (d2 > d1)
                        {
                            d1 = d2;
                        }
                        // select min deflection from linear and parabolic ones
                        if (d1 * d1 < d_2)
                        {
                            d_2 = d1 * d1;
                        }
                    }
                    if (d_2 > _error * _error)
                    {
                        _error = Math.Sqrt(d_2);
                    }
                }
                p1 = p2;
                p2 = p3;
            }
            _box.Add(p3);
        }
        _box.Enlarge(_error);
    }

    /// <summary>Resets the deflection to its initial value (100 times Precision.PConfusion()).</summary>
    public void ResetError() => _error = INITDEFLE;

    public override int PointCount => (_typ == PatchLineType.Walking ? _wpoly!.NbPnts() : _rpoly!.NbPnts());

    public override Point2D Point(int index)
    {
        double x, y, x1, y1, dx, dy;
        dx = dy = 0;
        if (_onfirst)
        {
            if (_typ == PatchLineType.Walking)
            {
                _wpoly!.Point(index).ParametersOnS1(out x, out y);
                if (index == 1)
                {
                    _wpoly.Point(2).ParametersOnS1(out x1, out y1);
                    dx = 0.0000001 * (x - x1);
                    dy = 0.0000001 * (y - y1);
                }
                else if (index == _wpoly.NbPnts())
                {
                    _wpoly.Point(index - 1).ParametersOnS1(out x1, out y1);
                    dx = 0.0000001 * (x - x1);
                    dy = 0.0000001 * (y - y1);
                }
            }
            else
            {
                _rpoly!.Point(index).ParametersOnS1(out x, out y);
            }
        }
        else
        {
            if (_typ == PatchLineType.Walking)
            {
                _wpoly!.Point(index).ParametersOnS2(out x, out y);
                if (index == 1)
                {
                    _wpoly.Point(2).ParametersOnS2(out x1, out y1);
                    dx = 0.0000001 * (x - x1);
                    dy = 0.0000001 * (y - y1);
                }
                else if (index == _wpoly.NbPnts())
                {
                    _wpoly.Point(index - 1).ParametersOnS2(out x1, out y1);
                    dx = 0.0000001 * (x - x1);
                    dy = 0.0000001 * (y - y1);
                }
            }
            else
            {
                _rpoly!.Point(index).ParametersOnS2(out x, out y);
            }
        }

        return (new Point2D(x + dx, y + dy));
    }
}
