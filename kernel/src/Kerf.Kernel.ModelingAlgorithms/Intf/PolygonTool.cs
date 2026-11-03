// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Tool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>
/// Provides services to create boxes for infinite curves (lines, hyperbolas, parabolas) in a given context: the
/// part of the curve inside a bounding domain, as a box and as parameter segments on the curve.
/// </summary>
/// <remarks>
/// A working object (decision 21): the box methods are public and fill the tool's segments, read back through
/// <see cref="SegmentCount"/>, <see cref="BeginParam"/> and <see cref="EndParam"/>. The output boxes are
/// <c>ref</c> parameters (OCCT's <c>Bnd_Box2d&amp;</c>/<c>Bnd_Box&amp;</c>); they are reset first.
/// </remarks>
public class PolygonTool
{
    private int _nbSeg;
    private readonly double[] _beginOnCurve = new double[6];
    private readonly double[] _endOnCurve = new double[6];
    private readonly int[] _bord = new int[12];
    private readonly double[] _xint = new double[12];
    private readonly double[] _yint = new double[12];
    private readonly double[] _zint = new double[12];
    private readonly double[] _parint = new double[12];

    public PolygonTool()
    {
        _nbSeg = 0;
    }

    public void Lin2dBox(in Line2D l2d, in Box2d domain, ref Box2d boxLin)
    {
        _nbSeg = 0;
        boxLin.SetVoid();
        if (domain.IsWhole)
        {
            boxLin.Set(l2d.Location, l2d.Direction);
            boxLin.Add(l2d.Direction.Reversed());
            _nbSeg = 1;
            _beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            _endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        double xmin, xmax, ymin, ymax;
        double Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0;
        double parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
        double parcur, par1, par2;
        bool xToSet, yToSet;

        domain.Get(out xmin, out ymin, out xmax, out ymax);

        if (l2d.Direction.XY().X() > 0.0)
        {
            if (domain.IsOpenXmin)
            {
                parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmin = (xmin - l2d.Location.XY().X()) / l2d.Direction.XY().X();
            }
            if (domain.IsOpenXmax)
            {
                parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmax = (xmax - l2d.Location.XY().X()) / l2d.Direction.XY().X();
            }
            xToSet = true;
        }
        else if (l2d.Direction.XY().X() < 0.0)
        {
            if (domain.IsOpenXmax)
            {
                parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmin = (xmax - l2d.Location.XY().X()) / l2d.Direction.XY().X();
            }
            if (domain.IsOpenXmin)
            {
                parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmax = (xmin - l2d.Location.XY().X()) / l2d.Direction.XY().X();
            }
            xToSet = true;
        }
        else
        { // Parallel to axis  X
            if (l2d.Location.XY().X() < xmin || xmax < l2d.Location.XY().X())
            {
                return;
            }
            Xmin = l2d.Location.XY().X();
            Xmax = l2d.Location.XY().X();
            xToSet = false;
        }

        if (l2d.Direction.XY().Y() > 0.0)
        {
            if (domain.IsOpenYmin)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymin - l2d.Location.XY().Y()) / l2d.Direction.XY().Y();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenYmax)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymax - l2d.Location.XY().Y()) / l2d.Direction.XY().Y();
            }
            parmax = Min(parmax, parcur);
            yToSet = true;
        }
        else if (l2d.Direction.XY().Y() < 0.0)
        {
            if (domain.IsOpenYmax)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymax - l2d.Location.XY().Y()) / l2d.Direction.XY().Y();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenYmin)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymin - l2d.Location.XY().Y()) / l2d.Direction.XY().Y();
            }
            parmax = Min(parmax, parcur);
            yToSet = true;
        }
        else
        { // Parallel to axis  Y
            if (l2d.Location.XY().Y() < ymin || ymax < l2d.Location.XY().Y())
            {
                return;
            }
            Ymin = l2d.Location.XY().Y();
            Ymax = l2d.Location.XY().Y();
            yToSet = false;
        }

        _nbSeg++;
        _beginOnCurve[0] = parmin;
        _endOnCurve[0] = parmax;

        if (xToSet)
        {
            par1 = l2d.Location.XY().X() + parmin * l2d.Direction.XY().X();
            par2 = l2d.Location.XY().X() + parmax * l2d.Direction.XY().X();
            Xmin = Min(par1, par2);
            Xmax = Max(par1, par2);
        }

        if (yToSet)
        {
            par1 = l2d.Location.XY().Y() + parmin * l2d.Direction.XY().Y();
            par2 = l2d.Location.XY().Y() + parmax * l2d.Direction.XY().Y();
            Ymin = Min(par1, par2);
            Ymax = Max(par1, par2);
        }

        boxLin.Update(Xmin, Ymin, Xmax, Ymax);
    }

    public void Hypr2dBox(in Hyperbola2D hypr2d, in Box2d domain, ref Box2d boxHypr2d)
    {
        _nbSeg = 0;
        boxHypr2d.SetVoid();
        if (domain.IsWhole)
        {
            boxHypr2d.SetWhole();
            _nbSeg = 1;
            _beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            _endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        int nbPi = Inters2d(hypr2d, domain);

        if (nbPi > 0)
        {
            double xmin, xmax, ymin, ymax;

            domain.Get(out xmin, out ymin, out xmax, out ymax);

            int npi;
            for (npi = 0; npi < nbPi; npi++)
            {
                xmin = Min(xmin, _xint[npi]);
                xmax = Max(xmax, _xint[npi]);
                ymin = Min(ymin, _yint[npi]);
                ymax = Max(ymax, _yint[npi]);
            }
            boxHypr2d.Update(xmin, ymin, xmax, ymax);

            int npj, npk;
            double parmin;
            for (npi = 0; npi < nbPi; npi++)
            {
                npk = npi;
                for (npj = npi + 1; npj < nbPi; npj++)
                {
                    if (_parint[npj] < _parint[npk])
                    {
                        npk = npj;
                    }
                }
                if (npk != npi)
                {
                    parmin = _parint[npk];
                    _parint[npk] = _parint[npi];
                    _parint[npi] = parmin;
                    npj = _bord[npk];
                    _bord[npk] = _bord[npi];
                    _bord[npi] = npj;
                }
            }

            Point2D pn;
            Vector2D tan;
            double sinan = 0;
            bool out_ = true;

            for (npi = 0; npi < nbPi; npi++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(_parint[npi], hypr2d, out pn, out tan);
                switch (_bord[npi])
                {
                    case 1:
                        sinan = new Xy(-1.0, 0.0) ^ tan.XY();
                        break;
                    case 2:
                        sinan = new Xy(0.0, -1.0) ^ tan.XY();
                        break;
                    case 3:
                        sinan = new Xy(1.0, 0.0) ^ tan.XY();
                        break;
                    case 4:
                        sinan = new Xy(0.0, 1.0) ^ tan.XY();
                        break;
                }
                if (Math.Abs(sinan) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    if (sinan > 0.0)
                    {
                        if (_nbSeg < 6)
                        {
                            out_ = false;
                            _beginOnCurve[_nbSeg] = _parint[npi];
                            _nbSeg++;
                        }
                    }
                    else
                    {
                        if (out_ && _nbSeg < 6)
                        {
                            _beginOnCurve[_nbSeg] = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            _nbSeg++;
                        }
                        if (_nbSeg > 0)
                        {
                            _endOnCurve[_nbSeg - 1] = _parint[npi];
                        }
                        out_ = true;

                        int ipmin;
                        if (_beginOnCurve[_nbSeg - 1] < -10.0)
                        {
                            ipmin = -10;
                        }
                        else
                        {
                            ipmin = (int)_beginOnCurve[_nbSeg - 1];
                        }
                        int ipmax;
                        if (_endOnCurve[_nbSeg - 1] > 10.0)
                        {
                            ipmax = 10;
                        }
                        else
                        {
                            ipmax = (int)_endOnCurve[_nbSeg - 1];
                        }

                        ipmin = ipmin * 10 + 1;
                        ipmax = ipmax * 10 - 1;
                        int ip, pas = 1;
                        for (ip = ipmin; ip <= ipmax; ip += pas)
                        {
                            boxHypr2d.Add(global::Kerf.Engine.Geometry.ElCLib.Value((double)ip / 10.0, hypr2d));
                            if (Math.Abs(ip) <= 10)
                            {
                                pas = 1;
                            }
                            else
                            {
                                pas = 10;
                            }
                        }
                    }
                }
            }
            if (!out_ && _nbSeg > 0)
            {
                _endOnCurve[_nbSeg - 1] = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }
        else if (!domain.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, hypr2d)))
        {
            boxHypr2d = domain;
            _beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            _endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            _nbSeg = 1;
        }
    }

    private int Inters2d(in Hyperbola2D curv, in Box2d domain)
    {
        int nbpi = 0;
        int npi;
        double xmin, xmax, ymin, ymax;

        domain.Get(out xmin, out ymin, out xmax, out ymax);

        if (!domain.IsOpenYmax)
        {
            Line2D l1 = new(new Point2D(0.0, ymax), new Direction2D(Direction2D.D.NX));
            AnalyticIntersection inters1 = new(curv, new Conic(l1));
            if (inters1.IsDone)
            {
                if (!inters1.IsEmpty)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).Value().X;
                        if (xmin < _xint[nbpi] && _xint[nbpi] <= xmax)
                        {
                            _yint[nbpi] = ymax;
                            _parint[nbpi] = inters1.Point(npi).ParamOnFirst();
                            _bord[nbpi] = 1;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenXmin)
        {
            Line2D l2 = new(new Point2D(xmin, 0.0), new Direction2D(Direction2D.D.NY));
            AnalyticIntersection inters2 = new(curv, new Conic(l2));
            if (inters2.IsDone)
            {
                if (!inters2.IsEmpty)
                {
                    for (npi = 1; npi <= inters2.PointCount; npi++)
                    {
                        _yint[nbpi] = inters2.Point(npi).Value().Y;
                        if (ymin < _yint[nbpi] && _yint[nbpi] <= ymax)
                        {
                            _xint[nbpi] = xmin;
                            _parint[nbpi] = inters2.Point(npi).ParamOnFirst();
                            _bord[nbpi] = 2;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenYmin)
        {
            Line2D l3 = new(new Point2D(0.0, ymin), new Direction2D(Direction2D.D.X));
            AnalyticIntersection inters3 = new(curv, new Conic(l3));
            if (inters3.IsDone)
            {
                if (!inters3.IsEmpty)
                {
                    for (npi = 1; npi <= inters3.PointCount; npi++)
                    {
                        _xint[nbpi] = inters3.Point(npi).Value().X;
                        if (xmin <= _xint[nbpi] && _xint[nbpi] < xmax)
                        {
                            _yint[nbpi] = ymin;
                            _parint[nbpi] = inters3.Point(npi).ParamOnFirst();
                            _bord[nbpi] = 3;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenXmax)
        {
            Line2D l4 = new(new Point2D(xmax, 0.0), new Direction2D(Direction2D.D.Y));
            AnalyticIntersection inters4 = new(curv, new Conic(l4));
            if (inters4.IsDone)
            {
                if (!inters4.IsEmpty)
                {
                    for (npi = 1; npi <= inters4.PointCount; npi++)
                    {
                        _yint[nbpi] = inters4.Point(npi).Value().Y;
                        if (ymin <= _yint[nbpi] && _yint[nbpi] < ymax)
                        {
                            _xint[nbpi] = xmax;
                            _parint[nbpi] = inters4.Point(npi).ParamOnFirst();
                            _bord[nbpi] = 4;
                            nbpi++;
                        }
                    }
                }
            }
        }
        return nbpi;
    }

    public void Parab2dBox(in Parabola2D parab2d, in Box2d domain, ref Box2d boxParab2d)
    {
        _nbSeg = 0;
        boxParab2d.SetVoid();
        if (domain.IsWhole)
        {
            boxParab2d.SetWhole();
            _nbSeg = 1;
            _beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            _endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        int nbPi = Inters2d(parab2d, domain);

        if (nbPi > 0)
        {
            double xmin, xmax, ymin, ymax;

            domain.Get(out xmin, out ymin, out xmax, out ymax);

            int npi;
            for (npi = 0; npi < nbPi; npi++)
            {
                xmin = Min(xmin, _xint[npi]);
                xmax = Max(xmax, _xint[npi]);
                ymin = Min(ymin, _yint[npi]);
                ymax = Max(ymax, _yint[npi]);
            }
            boxParab2d.Update(xmin, ymin, xmax, ymax);

            int npj, npk;
            double parmin;
            for (npi = 0; npi < nbPi; npi++)
            {
                npk = npi;
                for (npj = npi + 1; npj < nbPi; npj++)
                {
                    if (_parint[npj] < _parint[npk])
                    {
                        npk = npj;
                    }
                }
                if (npk != npi)
                {
                    parmin = _parint[npk];
                    _parint[npk] = _parint[npi];
                    _parint[npi] = parmin;
                    npj = _bord[npk];
                    _bord[npk] = _bord[npi];
                    _bord[npi] = npj;
                }
            }

            Point2D pn;
            Vector2D tan;
            double sinan = 0;
            bool out_ = true;

            for (npi = 0; npi < nbPi; npi++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(_parint[npi], parab2d, out pn, out tan);
                switch (_bord[npi])
                {
                    case 1:
                        sinan = new Xy(-1.0, 0.0) ^ tan.XY();
                        break;
                    case 2:
                        sinan = new Xy(0.0, -1.0) ^ tan.XY();
                        break;
                    case 3:
                        sinan = new Xy(1.0, 0.0) ^ tan.XY();
                        break;
                    case 4:
                        sinan = new Xy(0.0, 1.0) ^ tan.XY();
                        break;
                }
                if (Math.Abs(sinan) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    if (sinan > 0.0)
                    {
                        if (_nbSeg < 6)
                        {
                            out_ = false;
                            _beginOnCurve[_nbSeg] = _parint[npi];
                            _nbSeg++;
                        }
                    }
                    else
                    {
                        if (out_ && _nbSeg < 6)
                        {
                            _beginOnCurve[_nbSeg] = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            _nbSeg++;
                        }
                        if (_nbSeg > 0)
                        {
                            _endOnCurve[_nbSeg - 1] = _parint[npi];
                        }
                        out_ = true;

                        int ipmin;
                        if (_beginOnCurve[_nbSeg - 1] < -10.0)
                        {
                            ipmin = -10;
                        }
                        else
                        {
                            ipmin = (int)_beginOnCurve[_nbSeg - 1];
                        }
                        int ipmax;
                        if (_endOnCurve[_nbSeg - 1] > 10.0)
                        {
                            ipmax = 10;
                        }
                        else
                        {
                            ipmax = (int)_endOnCurve[_nbSeg - 1];
                        }

                        ipmin = ipmin * 10 + 1;
                        ipmax = ipmax * 10 - 1;
                        int ip, pas = 1;
                        for (ip = ipmin; ip <= ipmax; ip += pas)
                        {
                            boxParab2d.Add(global::Kerf.Engine.Geometry.ElCLib.Value((double)ip / 10.0, parab2d));
                            if (Math.Abs(ip) <= 10)
                            {
                                pas = 1;
                            }
                            else
                            {
                                pas = 10;
                            }
                        }
                    }
                }
            }
            if (!out_ && _nbSeg > 0)
            {
                _endOnCurve[_nbSeg - 1] = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }
        else if (!domain.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, parab2d)))
        {
            boxParab2d = domain;
            _beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            _endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            _nbSeg = 1;
        }
    }

    private int Inters2d(in Parabola2D curv, in Box2d domain)
    {
        int nbpi = 0;
        int npi;
        double xmin, xmax, ymin, ymax;

        domain.Get(out xmin, out ymin, out xmax, out ymax);

        if (!domain.IsOpenYmax)
        {
            Line2D l1 = new(new Point2D(0.0, ymax), new Direction2D(Direction2D.D.NX));
            AnalyticIntersection inters1 = new(curv, new Conic(l1));
            if (inters1.IsDone)
            {
                if (!inters1.IsEmpty)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).Value().X;
                        if (xmin < _xint[nbpi] && _xint[nbpi] <= xmax)
                        {
                            _yint[nbpi] = ymax;
                            _parint[nbpi] = inters1.Point(npi).ParamOnFirst();
                            _bord[nbpi] = 1;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenXmin)
        {
            Line2D l2 = new(new Point2D(xmin, 0.0), new Direction2D(Direction2D.D.NY));
            AnalyticIntersection inters2 = new(curv, new Conic(l2));
            if (inters2.IsDone)
            {
                if (!inters2.IsEmpty)
                {
                    for (npi = 1; npi <= inters2.PointCount; npi++)
                    {
                        _yint[nbpi] = inters2.Point(npi).Value().Y;
                        if (ymin < _yint[nbpi] && _yint[nbpi] <= ymax)
                        {
                            _xint[nbpi] = xmin;
                            _parint[nbpi] = inters2.Point(npi).ParamOnFirst();
                            _bord[nbpi] = 2;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenYmin)
        {
            Line2D l3 = new(new Point2D(0.0, ymin), new Direction2D(Direction2D.D.X));
            AnalyticIntersection inters3 = new(curv, new Conic(l3));
            if (inters3.IsDone)
            {
                if (!inters3.IsEmpty)
                {
                    for (npi = 1; npi <= inters3.PointCount; npi++)
                    {
                        _xint[nbpi] = inters3.Point(npi).Value().X;
                        if (xmin <= _xint[nbpi] && _xint[nbpi] < xmax)
                        {
                            _yint[nbpi] = ymin;
                            _parint[nbpi] = inters3.Point(npi).ParamOnFirst();
                            _bord[nbpi] = 3;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenXmax)
        {
            Line2D l4 = new(new Point2D(xmax, 0.0), new Direction2D(Direction2D.D.Y));
            AnalyticIntersection inters4 = new(curv, new Conic(l4));
            if (inters4.IsDone)
            {
                if (!inters4.IsEmpty)
                {
                    for (npi = 1; npi <= inters4.PointCount; npi++)
                    {
                        _yint[nbpi] = inters4.Point(npi).Value().Y;
                        if (ymin <= _yint[nbpi] && _yint[nbpi] < ymax)
                        {
                            _xint[nbpi] = xmax;
                            _parint[nbpi] = inters4.Point(npi).ParamOnFirst();
                            _bord[nbpi] = 4;
                            nbpi++;
                        }
                    }
                }
            }
        }
        return nbpi;
    }

    public void LinBox(in Line3D l, in Box domain, ref Box boxLin)
    {
        _nbSeg = 0;
        boxLin.SetVoid();
        if (domain.IsWhole)
        {
            boxLin.Set(l.Location, l.Direction);
            boxLin.Add(l.Direction.Reversed());
            _nbSeg = 1;
            _beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            _endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        double xmin, xmax, ymin, ymax, zmin, zmax;
        double Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0, Zmin = 0, Zmax = 0;
        double parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
        double parcur, par1, par2;
        bool xToSet, yToSet, zToSet;

        domain.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);

        if (l.Direction.XYZ().X() > 0.0)
        {
            if (domain.IsOpenXmin)
            {
                parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmin = (xmin - l.Location.XYZ().X()) / l.Direction.XYZ().X();
            }
            if (domain.IsOpenXmax)
            {
                parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmax = (xmax - l.Location.XYZ().X()) / l.Direction.XYZ().X();
            }
            xToSet = true;
        }
        else if (l.Direction.XYZ().X() < 0.0)
        {
            if (domain.IsOpenXmax)
            {
                parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmin = (xmax - l.Location.XYZ().X()) / l.Direction.XYZ().X();
            }
            if (domain.IsOpenXmin)
            {
                parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmax = (xmin - l.Location.XYZ().X()) / l.Direction.XYZ().X();
            }
            xToSet = true;
        }
        else
        { // Perpendiculaire a l axe  X
            if (l.Location.XYZ().X() < xmin || xmax < l.Location.XYZ().X())
            {
                return;
            }
            Xmin = l.Location.XYZ().X();
            Xmax = l.Location.XYZ().X();
            xToSet = false;
        }

        if (l.Direction.XYZ().Y() > 0.0)
        {
            if (domain.IsOpenYmin)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymin - l.Location.XYZ().Y()) / l.Direction.XYZ().Y();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenYmax)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymax - l.Location.XYZ().Y()) / l.Direction.XYZ().Y();
            }
            parmax = Min(parmax, parcur);
            yToSet = true;
        }
        else if (l.Direction.XYZ().Y() < 0.0)
        {
            if (domain.IsOpenYmax)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymax - l.Location.XYZ().Y()) / l.Direction.XYZ().Y();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenYmin)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymin - l.Location.XYZ().Y()) / l.Direction.XYZ().Y();
            }
            parmax = Min(parmax, parcur);
            yToSet = true;
        }
        else
        { // Perpendiculaire a l axe  Y
            if (l.Location.XYZ().Y() < ymin || ymax < l.Location.XYZ().Y())
            {
                return;
            }
            Ymin = l.Location.XYZ().Y();
            Ymax = l.Location.XYZ().Y();
            yToSet = false;
        }

        if (l.Direction.XYZ().Z() > 0.0)
        {
            if (domain.IsOpenZmin)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (zmin - l.Location.XYZ().Z()) / l.Direction.XYZ().Z();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenZmax)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (zmax - l.Location.XYZ().Z()) / l.Direction.XYZ().Z();
            }
            parmax = Min(parmax, parcur);
            zToSet = true;
        }
        else if (l.Direction.XYZ().Z() < 0.0)
        {
            if (domain.IsOpenZmax)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (zmax - l.Location.XYZ().Z()) / l.Direction.XYZ().Z();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenZmin)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (zmin - l.Location.XYZ().Z()) / l.Direction.XYZ().Z();
            }
            parmax = Min(parmax, parcur);
            zToSet = true;
        }
        else
        { // Perpendicular to axis Z
            if (l.Location.XYZ().Z() < zmin || zmax < l.Location.XYZ().Z())
            {
                return;
            }
            Zmin = l.Location.XYZ().Z();
            Zmax = l.Location.XYZ().Z();
            zToSet = false;
        }

        _nbSeg++;
        _beginOnCurve[0] = parmin;
        _endOnCurve[0] = parmax;

        if (xToSet)
        {
            par1 = l.Location.XYZ().X() + parmin * l.Direction.XYZ().X();
            par2 = l.Location.XYZ().X() + parmax * l.Direction.XYZ().X();
            Xmin = Min(par1, par2);
            Xmax = Max(par1, par2);
        }

        if (yToSet)
        {
            par1 = l.Location.XYZ().Y() + parmin * l.Direction.XYZ().Y();
            par2 = l.Location.XYZ().Y() + parmax * l.Direction.XYZ().Y();
            Ymin = Min(par1, par2);
            Ymax = Max(par1, par2);
        }

        if (zToSet)
        {
            par1 = l.Location.XYZ().Z() + parmin * l.Direction.XYZ().Z();
            par2 = l.Location.XYZ().Z() + parmax * l.Direction.XYZ().Z();
            Zmin = Min(par1, par2);
            Zmax = Max(par1, par2);
        }

        boxLin.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
    }

    public void HyprBox(in Hyperbola3D hypr, in Box domain, ref Box boxHypr)
    {
        _nbSeg = 0;
        boxHypr.SetVoid();

        if (domain.IsWhole)
        {
            boxHypr.SetWhole();
            _nbSeg = 1;
            // beginOnCurve[0]=-Precision::Infinite();
            // endOnCurve[0]=Precision::Infinite();
            _beginOnCurve[0] = -100.0;
            _endOnCurve[0] = 100.0;
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        int nbPi;

        nbPi = Inters3d(hypr, domain);
        if (nbPi > 0)
        {
            int npi;
            double xmin, ymin, zmin, xmax, ymax, zmax;

            domain.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);

            for (npi = 0; npi < nbPi; npi++)
            {
                xmin = Min(xmin, _xint[npi]);
                xmax = Max(xmax, _xint[npi]);
                ymin = Min(ymin, _yint[npi]);
                ymax = Max(ymax, _yint[npi]);
                zmin = Min(zmin, _zint[npi]);
                zmax = Max(zmax, _zint[npi]);
            }
            boxHypr.Update(xmin, ymin, zmin, xmax, ymax, zmax);

            Point3D pn;
            Vector3D tan;
            double sinan = 0;
            bool out_ = true;

            for (npi = 0; npi < nbPi; npi++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(_parint[npi], hypr, out pn, out tan);
                switch (_bord[npi])
                {
                    case 1:
                        sinan = new Xyz(1.0, 0.0, 0.0) * tan.XYZ();
                        break;
                    case 2:
                        sinan = new Xyz(0.0, 1.0, 0.0) * tan.XYZ();
                        break;
                    case 3:
                        sinan = new Xyz(0.0, 0.0, 1.0) * tan.XYZ();
                        break;
                    case 4:
                        sinan = new Xyz(-1.0, 0.0, 0.0) * tan.XYZ();
                        break;
                    case 5:
                        sinan = new Xyz(0.0, -1.0, 0.0) * tan.XYZ();
                        break;
                    case 6:
                        sinan = new Xyz(0.0, 0.0, -1.0) * tan.XYZ();
                        break;
                }
                if (Math.Abs(sinan) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    if (sinan > 0.0)
                    {
                        out_ = false;
                        // OCCT: unlike the other boxes (fixed for bugs #75/#76), HyprBox doesn't check nbSeg < 6
                        // before writing the 6-element segment arrays; more than six entries overflow them in C++
                        // and throw IndexOutOfRangeException here.
                        _beginOnCurve[_nbSeg] = _parint[npi];
                        //// modified by jgv, 10.11.2009 /////
                        _endOnCurve[_nbSeg] = 10.0;
                        //////////////////////////////////////
                        _nbSeg++;
                    }
                    else
                    {
                        if (out_)
                        {
                            _beginOnCurve[_nbSeg] = -10.0; // OCCT: unchecked index, as above
                            _nbSeg++;
                        }
                        _endOnCurve[_nbSeg - 1] = _parint[npi];
                        out_ = true;

                        double ipmin, ipmax, ip, pas;

                        ipmin = -10.0;
                        if (_beginOnCurve[_nbSeg - 1] > ipmin)
                        {
                            ipmin = _beginOnCurve[_nbSeg - 1];
                        }
                        ipmax = 10.0;
                        if (_endOnCurve[_nbSeg - 1] < ipmax)
                        {
                            ipmax = _endOnCurve[_nbSeg - 1];
                        }
                        ipmin = ipmin * 10.0 + 1.0;
                        ipmax = ipmax * 10.0 - 1.0;

                        pas = 1.0;
                        for (ip = ipmin; ip <= ipmax; ip += pas)
                        {
                            boxHypr.Add(global::Kerf.Engine.Geometry.ElCLib.Value(ip / 10.0, hypr));
                            pas = 10.0;
                            if (Math.Abs(ip) <= 10.0)
                            {
                                pas = 1.0;
                            }
                        }
                    }
                }
            }
        }
        else if (!domain.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, hypr)))
        {
            boxHypr = domain;
            // beginOnCurve[0]=-Precision::Infinite();
            // endOnCurve[0]=Precision::Infinite();
            _beginOnCurve[0] = -100.0;
            _endOnCurve[0] = 100.0;
            _nbSeg = 1;
        }
    }

    private int Inters3d(in Hyperbola3D curv, in Box domain)
    {
        int nbpi = 0;
        int npi;
        double xmin, ymin, zmin, xmax, ymax, zmax;

        domain.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);

        if (!domain.IsOpenXmin)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(1.0, 0.0, 0.0, -xmin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _yint[nbpi] = inters1.Point(npi).Y;
                        _zint[nbpi] = inters1.Point(npi).Z;
                        if (ymin <= _yint[nbpi] && _yint[nbpi] < ymax && zmin <= _zint[nbpi] && _zint[nbpi] < zmax)
                        {
                            _xint[nbpi] = xmin;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 1;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenYmin)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(0.0, 1.0, 0.0, -ymin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).X;
                        _zint[nbpi] = inters1.Point(npi).Z;
                        if (xmin < _xint[nbpi] && _xint[nbpi] <= xmax && zmin <= _zint[nbpi] && _zint[nbpi] < zmax)
                        {
                            _yint[nbpi] = ymin;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 2;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenZmin)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(0.0, 0.0, 1.0, -zmin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).X;
                        _yint[nbpi] = inters1.Point(npi).Y;
                        if (xmin < _xint[nbpi] && _xint[nbpi] <= xmax && ymin < _yint[nbpi] && _yint[nbpi] <= ymax)
                        {
                            _zint[nbpi] = zmin;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 3;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenXmax)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(-1.0, 0.0, 0.0, xmax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _yint[nbpi] = inters1.Point(npi).Y;
                        _zint[nbpi] = inters1.Point(npi).Z;
                        if (ymin < _yint[nbpi] && _yint[nbpi] <= ymax && zmin < _zint[nbpi] && _zint[nbpi] <= zmax)
                        {
                            _xint[nbpi] = xmax;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 4;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenYmax)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(0.0, -1.0, 0.0, ymax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).X;
                        _zint[nbpi] = inters1.Point(npi).Z;
                        if (xmin <= _xint[nbpi] && _xint[nbpi] < xmax && zmin < _zint[nbpi] && _zint[nbpi] <= zmax)
                        {
                            _yint[nbpi] = ymax;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 5;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenZmax)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(0.0, 0.0, -1.0, zmax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).X;
                        _yint[nbpi] = inters1.Point(npi).Y;
                        if (xmin <= _xint[nbpi] && _xint[nbpi] < xmax && ymin <= _yint[nbpi] && _yint[nbpi] < ymax)
                        {
                            _zint[nbpi] = zmax;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 6;
                            nbpi++;
                        }
                    }
                }
            }
        }

        int nbDiffPoints = nbpi;

        // Sort parint and check if parint contains several
        // matched values. If that is true they will be deleted.
        for (int i = nbpi - 1; i > 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (_parint[i] <= _parint[j])
                {
                    (_parint[i], _parint[j]) = (_parint[j], _parint[i]);
                    (_zint[i], _zint[j]) = (_zint[j], _zint[i]);
                    (_yint[i], _yint[j]) = (_yint[j], _yint[i]);
                    (_xint[i], _xint[j]) = (_xint[j], _xint[i]);
                    (_bord[i], _bord[j]) = (_bord[j], _bord[i]);
                }

                if ((i < nbpi - 1) && IsEqual(_parint[i], _parint[i + 1]))
                {
                    nbDiffPoints--;
                    for (int k = i; k < nbDiffPoints; k++)
                    {
                        _parint[k] = _parint[k + 1];
                        _zint[k] = _zint[k + 1];
                        _yint[k] = _yint[k + 1];
                        _xint[k] = _xint[k + 1];
                        _bord[k] = _bord[k + 1];
                    }
                }
            }
        }

        return nbDiffPoints;
    }

    private int Inters3d(in Parabola3D curv, in Box domain)
    {
        int nbpi = 0;
        int npi;
        double xmin, ymin, zmin, xmax, ymax, zmax;

        domain.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);

        if (!domain.IsOpenXmin)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(1.0, 0.0, 0.0, -xmin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _yint[nbpi] = inters1.Point(npi).Y;
                        _zint[nbpi] = inters1.Point(npi).Z;
                        if (ymin <= _yint[nbpi] && _yint[nbpi] < ymax && zmin <= _zint[nbpi] && _zint[nbpi] < zmax)
                        {
                            _xint[nbpi] = xmin;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 1;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenYmin)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(0.0, 1.0, 0.0, -ymin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).X;
                        _zint[nbpi] = inters1.Point(npi).Z;
                        if (xmin < _xint[nbpi] && _xint[nbpi] <= xmax && zmin <= _zint[nbpi] && _zint[nbpi] < zmax)
                        {
                            _yint[nbpi] = ymin;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 2;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenZmin)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(0.0, 0.0, 1.0, -zmin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).X;
                        _yint[nbpi] = inters1.Point(npi).Y;
                        if (xmin < _xint[nbpi] && _xint[nbpi] <= xmax && ymin < _yint[nbpi] && _yint[nbpi] <= ymax)
                        {
                            _zint[nbpi] = zmin;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 3;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenXmax)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(-1.0, 0.0, 0.0, xmax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _yint[nbpi] = inters1.Point(npi).Y;
                        _zint[nbpi] = inters1.Point(npi).Z;
                        if (ymin < _yint[nbpi] && _yint[nbpi] <= ymax && zmin < _zint[nbpi] && _zint[nbpi] <= zmax)
                        {
                            _xint[nbpi] = xmax;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 4;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenYmax)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(0.0, -1.0, 0.0, ymax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).X;
                        _zint[nbpi] = inters1.Point(npi).Z;
                        if (xmin <= _xint[nbpi] && _xint[nbpi] < xmax && zmin < _zint[nbpi] && _zint[nbpi] <= zmax)
                        {
                            _yint[nbpi] = ymax;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 5;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!domain.IsOpenZmax)
        {
            IntConicQuad inters1 = new(curv, new Plane3D(0.0, 0.0, -1.0, zmax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (inters1.IsDone)
            {
                if (!inters1.IsInQuadric)
                {
                    for (npi = 1; npi <= inters1.PointCount; npi++)
                    {
                        _xint[nbpi] = inters1.Point(npi).X;
                        _yint[nbpi] = inters1.Point(npi).Y;
                        if (xmin <= _xint[nbpi] && _xint[nbpi] < xmax && ymin <= _yint[nbpi] && _yint[nbpi] < ymax)
                        {
                            _zint[nbpi] = zmax;
                            _parint[nbpi] = inters1.ParamOnConic(npi);
                            _bord[nbpi] = 6;
                            nbpi++;
                        }
                    }
                }
            }
        }

        int nbDiffPoints = nbpi;

        // Sort parint and check if parint contains several
        // matched values. If that is true they will be deleted.
        for (int i = nbpi - 1; i > 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (_parint[i] <= _parint[j])
                {
                    (_parint[i], _parint[j]) = (_parint[j], _parint[i]);
                    (_zint[i], _zint[j]) = (_zint[j], _zint[i]);
                    (_yint[i], _yint[j]) = (_yint[j], _yint[i]);
                    (_xint[i], _xint[j]) = (_xint[j], _xint[i]);
                    (_bord[i], _bord[j]) = (_bord[j], _bord[i]);
                }

                if ((i < nbpi - 1) && IsEqual(_parint[i], _parint[i + 1]))
                {
                    nbDiffPoints--;
                    for (int k = i; k < nbDiffPoints; k++)
                    {
                        _parint[k] = _parint[k + 1];
                        _zint[k] = _zint[k + 1];
                        _yint[k] = _yint[k + 1];
                        _xint[k] = _xint[k + 1];
                        _bord[k] = _bord[k + 1];
                    }
                }
            }
        }

        return nbDiffPoints;
    }

    public void ParabBox(in Parabola3D parab, in Box domain, ref Box boxParab)
    {
        _nbSeg = 0;
        boxParab.SetVoid();

        if (domain.IsWhole)
        {
            boxParab.SetWhole();
            _nbSeg = 1;
            _beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            _endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        int nbPi = Inters3d(parab, domain);

        if (nbPi > 0)
        {
            int npi;
            double xmin, ymin, zmin, xmax, ymax, zmax;
            domain.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);
            for (npi = 0; npi < nbPi; npi++)
            {
                xmin = Min(xmin, _xint[npi]);
                xmax = Max(xmax, _xint[npi]);
                ymin = Min(ymin, _yint[npi]);
                ymax = Max(ymax, _yint[npi]);
                zmin = Min(zmin, _zint[npi]);
                zmax = Max(zmax, _zint[npi]);
            }
            boxParab.Update(xmin, ymin, zmin, xmax, ymax, zmax);

            Point3D pn;
            Vector3D tan;
            double sinan = 0;
            bool out_ = true;

            for (npi = 0; npi < nbPi; npi++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(_parint[npi], parab, out pn, out tan);
                switch (_bord[npi])
                {
                    case 1:
                        sinan = new Xyz(1.0, 0.0, 0.0) * tan.XYZ();
                        break;
                    case 2:
                        sinan = new Xyz(0.0, 1.0, 0.0) * tan.XYZ();
                        break;
                    case 3:
                        sinan = new Xyz(0.0, 0.0, 1.0) * tan.XYZ();
                        break;
                    case 4:
                        sinan = new Xyz(-1.0, 0.0, 0.0) * tan.XYZ();
                        break;
                    case 5:
                        sinan = new Xyz(0.0, -1.0, 0.0) * tan.XYZ();
                        break;
                    case 6:
                        sinan = new Xyz(0.0, 0.0, -1.0) * tan.XYZ();
                        break;
                }
                if (Math.Abs(sinan) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    if (sinan > 0.0)
                    {
                        if (_nbSeg < 6)
                        {
                            out_ = false;
                            _beginOnCurve[_nbSeg] = _parint[npi];
                            _nbSeg++;
                        }
                    }
                    else
                    {
                        if (out_ && _nbSeg < 6)
                        {
                            _beginOnCurve[_nbSeg] = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            _nbSeg++;
                        }
                        if (_nbSeg > 0)
                        {
                            _endOnCurve[_nbSeg - 1] = _parint[npi];
                        }
                        out_ = true;

                        int ipmin;
                        if (_beginOnCurve[_nbSeg - 1] < -10.0)
                        {
                            ipmin = -10;
                        }
                        else
                        {
                            ipmin = (int)_beginOnCurve[_nbSeg - 1];
                        }
                        int ipmax;
                        if (_endOnCurve[_nbSeg - 1] > 10.0)
                        {
                            ipmax = 10;
                        }
                        else
                        {
                            ipmax = (int)_endOnCurve[_nbSeg - 1];
                        }

                        ipmin = ipmin * 10 + 1;
                        ipmax = ipmax * 10 - 1;
                        int ip, pas = 1;
                        for (ip = ipmin; ip <= ipmax; ip += pas)
                        {
                            boxParab.Add(global::Kerf.Engine.Geometry.ElCLib.Value((double)ip / 10.0, parab));
                            if (Math.Abs(ip) <= 10)
                            {
                                pas = 1;
                            }
                            else
                            {
                                pas = 10;
                            }
                        }
                    }
                }
            }
            if (!out_ && _nbSeg > 0)
            {
                _endOnCurve[_nbSeg - 1] = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }
        else if (!domain.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, parab)))
        {
            boxParab = domain;
            _beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            _endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            _nbSeg = 1;
        }
    }

    public int SegmentCount => _nbSeg;

    public double BeginParam(int segmentNum)
    {
        Standard_OutOfRange_Raise_if(segmentNum < 1 || segmentNum > _nbSeg, "Intf_Tool::BeginParam");
        return _beginOnCurve[segmentNum - 1];
    }

    public double EndParam(int segmentNum)
    {
        Standard_OutOfRange_Raise_if(segmentNum < 1 || segmentNum > _nbSeg, "Intf_Tool::EndParam");
        return _endOnCurve[segmentNum - 1];
    }
}
