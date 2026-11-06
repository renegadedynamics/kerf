// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_FClass2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_FClass2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Topology.Adaptors;

/// <summary>
/// A fast classifier of 2D points in a face: each wire is sampled once into a polygon (CSLib_Class2d), and points
/// that the polygons can't decide go to <see cref="FaceClassifier"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21) built once for a face; its queries don't change it. OCCT forbids copies (Copy and
/// the assignment throw Standard_ConstructionError): the port has neither.
/// </remarks>
internal sealed class FClass2d
{
    private readonly NCollection_Sequence<Class2d> _tabClass = new();
    private readonly NCollection_Sequence<int> _tabOrien = new();
    private double _toluv;
    private Face _face;
    // The period of the surface, set in U1..U2, V1..V2 and never read (as decision 221's fields)
    private double _umin;
    private double _umax;
    private double _vmin;
    private double _vmax;

    // Increments theValue by theIncrement towards theDirection, ensuring that the result is
    // different from theValue. For large values of theValue with small theIncrement the result of
    // theValue + theIncrement can be equal to theValue due to the limited resolution of double
    // precision. This function guarantees to return the next representable value in the direction of
    // theDirection in such cases.
    private static double SafeIncrement(double value, double direction, double increment)
    {
        double nextValue = value + increment;
        return nextValue == value ? NextAfter(value, direction) : nextValue;
    }

    // std::nextafter
    private static double NextAfter(double theFrom, double to)
    {
        if (double.IsNaN(theFrom) || double.IsNaN(to))
        {
            return theFrom + to;
        }

        if (theFrom == to)
        {
            return to;
        }

        return to > theFrom ? Math.BitIncrement(theFrom) : Math.BitDecrement(theFrom);
    }

    // Checks whether the curve is degenerated between theStartParam and theEndParam.
    // The check is performed by sampling the curve at several points and measuring the distance to the
    // start point. If all sampled points are closer than Precision::Confusion() to the start point, the
    // curve is considered degenerated.
    private static bool IsDegenerated(EdgeCurveAdaptor curve, double startParam, double endParam)
    {
        double parametricStep = (endParam - startParam) * 0.1;
        Point3D startPoint = curve.Value(startParam);
        for (double currParam = startParam; currParam < endParam;
             currParam = SafeIncrement(currParam, endParam, parametricStep))
        {
            Point3D currentPoint = curve.Value(currParam);
            if (startPoint.SquareDistanceTo(currentPoint) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>A classifier of the face <paramref name="aFace"/> with the tolerance <paramref name="tolUV"/> in its parameter space.</summary>
    public FClass2d(in Face aFace, double tolUV)
    {
        _toluv = tolUV;
        _face = aFace;

        //-- dead end on surfaces defined on more than one period

        _face.Orient(Forward);
        var surf = new FaceSurfaceAdaptor();
        surf.Initialize(aFace, false);
        Edge edge;
        Orientation or;
        var wireExplorer = new WireExplorer();

        _umin = _vmin = 0.0; // RealLast();
        _umax = _vmax = -_umin;

        int nbE = 0;
        const double eps = 1.0e-10;
        bool isBadWire = false;
        for (var faceExplorer = new Explorer(_face, ShapeType.Wire); (faceExplorer.More() && !isBadWire);
             faceExplorer.Next())
        {
            int nbpnts = 0;
            var seqPnt2d = new NCollection_Sequence<Point2D>();
            int firstpoint = 1;
            double flecheU = 0.0;
            double flecheV = 0.0;
            bool wireIsNotEmpty = false;
            int nbEdges = 0;

            var explorer = new Explorer();
            for (explorer.Init(faceExplorer.Current(), ShapeType.Edge); explorer.More(); explorer.Next())
            {
                nbEdges++;
            }

            nbE = nbEdges;

            Point3D ancienpnt3d = new(0, 0, 0);
            bool ancienpnt3dinitialise = false;

            for (wireExplorer.Init(ShapeCasts.Wire(faceExplorer.Current()), _face); wireExplorer.More();
                 wireExplorer.Next())
            {
                nbEdges--;
                edge = wireExplorer.Current();
                or = edge.Orientation;
                if (or == Forward || or == Reversed)
                {
                    double pfbid, plbid;
                    if (Tool.CurveOnSurface(edge, _face, out pfbid, out plbid) is null)
                    {
                        // OCCT returns here, leaving the classifier half built: the wires explored so far, Umin..Vmax as far as
                        // they got and U1..V2 at 0 (occt-issues.md)
                        return;
                    }

                    if (Math.Abs(plbid - pfbid) < 1.0e-9)
                    {
                        continue;
                    }

                    bool degenerated = false;
                    if (Tool.Degenerated(edge))
                    {
                        degenerated = true;
                    }

                    if (Tool.IsClosed(edge, _face))
                    {
                        degenerated = true;
                    }

                    Exploration.Vertices(edge, out Vertex va, out Vertex vb);
                    if (va.IsNull || vb.IsNull)
                    {
                        degenerated = true;
                    }

                    var curveAdaptor2D = new EdgeCurve2DAdaptor(edge, _face);
                    var curveAdaptor3D = new EdgeCurveAdaptor(edge, _face);
                    //-- Check cases when it was forgotten to code degenerated :  PRO17410 (janv 99)
                    if (!degenerated)
                    {
                        degenerated = IsDegenerated(curveAdaptor3D, pfbid, plbid);
                    }

                    //-- ----------------------------------------
                    // int nbs = 1 + Geom2dInt_Geom2dCurveTool::NbSamples(C);
                    int nbs = Geom2dCurveTool.NbSamples(curveAdaptor2D);
                    //-- Attention to rational bsplines of degree 3. (ends of circles among others)
                    if (nbs > 2)
                    {
                        nbs *= 4;
                    }

                    double du = (plbid - pfbid) / (double)(nbs - 1);
                    double u = 0.0;
                    if (or == Forward)
                    {
                        u = pfbid;
                    }
                    else
                    {
                        u = plbid;
                        du = -du;
                    }

                    //-- ------------------------------------------------------------
                    //-- Check distance uv between the start point of the edge
                    //-- and the last point registered in SeqPnt2d
                    //-- Try to remote the first point of the current edge
                    //-- from the last saved point

                    if (firstpoint == 2)
                    {
                        u += du;
                    }

                    int avant = nbpnts;
                    for (int e = firstpoint; e <= nbs; e++)
                    {
                        Point2D P2d = curveAdaptor2D.Value(u);
                        if (P2d.X < _umin)
                        {
                            _umin = P2d.X;
                        }

                        if (P2d.X > _umax)
                        {
                            _umax = P2d.X;
                        }

                        if (P2d.Y < _vmin)
                        {
                            _vmin = P2d.Y;
                        }

                        if (P2d.Y > _vmax)
                        {
                            _vmax = P2d.Y;
                        }

                        double dist3dptcourant_ancienpnt = 1e+20; // RealLast();
                        Point3D p3d = new();
                        if (!degenerated)
                        {
                            p3d = curveAdaptor3D.Value(u);
                            if (nbpnts > 1 && ancienpnt3dinitialise)
                            {
                                dist3dptcourant_ancienpnt = p3d.DistanceTo(ancienpnt3d);
                            }
                        }

                        bool isRealCurve3d = true; // patch
                        if (dist3dptcourant_ancienpnt < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            Point3D midP3d = curveAdaptor3D.Value(u - du / 2.0);
                            if (p3d.DistanceTo(midP3d) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                isRealCurve3d = false;
                            }
                        }

                        if (isRealCurve3d)
                        {
                            if (!degenerated)
                            {
                                ancienpnt3d = p3d;
                                ancienpnt3dinitialise = true;
                            }

                            nbpnts++;
                            seqPnt2d.Append(P2d);
                        }

                        u += du;
                        int ii = nbpnts;
                        //  Modified by Sergey KHROMOV - Fri Apr 19 09:46:12 2002 Begin
                        if (ii > (avant + 4) && seqPnt2d[ii - 2].SquareDistanceTo(seqPnt2d[ii]) != 0.0)
                        //  Modified by Sergey KHROMOV - Fri Apr 19 09:46:13 2002 End
                        {
                            var lin = new Line2D(seqPnt2d[ii - 2], new Direction2D(new Vector2D(seqPnt2d[ii - 2], seqPnt2d[ii])));
                            double ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, seqPnt2d[ii - 1]);
                            Point2D pp = global::Kerf.Engine.Geometry.ElCLib.Value(ul, lin);
                            double dU = Math.Abs(pp.X - seqPnt2d[ii - 1].X);
                            double dV = Math.Abs(pp.Y - seqPnt2d[ii - 1].Y);
                            if (dU > flecheU)
                            {
                                flecheU = dU;
                            }

                            if (dV > flecheV)
                            {
                                flecheV = dV;
                            }
                        }
                    } // for(e=firstpoint

                    if (firstpoint == 1)
                    {
                        firstpoint = 2;
                    }

                    wireIsNotEmpty = true;
                } // if(Or==FORWARD,REVERSED
            } //-- Edges -> for(Ware.Explorer

            if (nbEdges != 0)
            {
                //-- on compte ++ with a normal explorer and with the Wire Explorer
                var pClass = new NCollection_Array1<Point2D>(1, 2);
                //// modified by jgv, 28.04.2009 ////
                pClass.Init(new Point2D(0.0, 0.0));
                /////////////////////////////////////
                _tabClass.Append(new Class2d(pClass, flecheU, flecheV, _umin, _vmin, _umax, _vmax));
                isBadWire = true;
                _tabOrien.Append(-1);
            }
            else if (wireIsNotEmpty)
            {
                var pClass = new NCollection_Array1<Point2D>(1, nbpnts);
                double square = 0.0;

                //-------------------------------------------------------------------
                //-- ** The mode of calculation was somewhat changed
                //-- Before Oct 31 97 , the total angle of
                //-- rotation of the wire was evaluated on all angles except for the last
                //-- ** Now, exactly the angle of rotation is evaluated
                //-- If a value remote from 2PI or -2PI is found, it means that there is
                //-- an uneven number of loops

                if (nbpnts > 3)
                {
                    int im1 = nbpnts - 1;
                    int im0 = 1;
                    pClass[im1] = seqPnt2d.Value(im1);
                    pClass[nbpnts] = seqPnt2d.Value(nbpnts);

                    double per = 0.0;
                    for (int ii = 1; ii < nbpnts; ii++, im0++, im1++)
                    {
                        if (im1 >= nbpnts)
                        {
                            im1 = 1;
                        }

                        pClass[ii] = seqPnt2d.Value(ii);
                        square += (pClass[im0].X - pClass[im1].X) * (pClass[im0].Y + pClass[im1].Y) * .5;
                        per += (pClass[im0].XY() - pClass[im1].XY()).Modulus();
                    }

                    double expThick = Max(2.0 * Math.Abs(square) / per, 1e-7);
                    double defl = Max(flecheU, flecheV);
                    double discrDefl = Min(defl * 0.1, expThick * 10.0);
                    while (defl > expThick && discrDefl > 1e-7)
                    {
                        // Deflection of the polygon is too much for this ratio of area and perimeter,
                        // and this might lead to self-intersections.
                        // Discretize the wire more tightly to eliminate the error.
                        firstpoint = 1;
                        seqPnt2d.Clear();
                        flecheU = 0.0;
                        flecheV = 0.0;
                        for (wireExplorer.Init(ShapeCasts.Wire(faceExplorer.Current()), _face); wireExplorer.More();
                             wireExplorer.Next())
                        {
                            edge = wireExplorer.Current();
                            or = edge.Orientation;
                            if (or == Forward || or == Reversed)
                            {
                                Tool.Range(edge, _face, out double pfbid, out double plbid);
                                if (Math.Abs(plbid - pfbid) < 1.0e-9)
                                {
                                    continue;
                                }

                                var c = new EdgeCurve2DAdaptor(edge, _face);
                                var discr = new QuasiUniformDeflection(c, discrDefl);
                                if (!discr.IsDone)
                                {
                                    break;
                                }

                                int nbp = discr.PointCount;
                                int iStep = 1, i = 1, iEnd = nbp + 1;
                                if (or == Reversed)
                                {
                                    iStep = -1;
                                    i = nbp;
                                    iEnd = 0;
                                }

                                if (firstpoint == 2)
                                {
                                    i += iStep;
                                }

                                for (; i != iEnd; i += iStep)
                                {
                                    Point2D p2d = c.Value(discr.Parameter(i));
                                    seqPnt2d.Append(p2d);
                                }

                                if (nbp > 2)
                                {
                                    int ii = seqPnt2d.Length();
                                    var lin = new Line2D(seqPnt2d[ii - 2], new Direction2D(new Vector2D(seqPnt2d[ii - 2], seqPnt2d[ii])));
                                    double ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, seqPnt2d[ii - 1]);
                                    Point2D pp = global::Kerf.Engine.Geometry.ElCLib.Value(ul, lin);
                                    double dU = Math.Abs(pp.X - seqPnt2d[ii - 1].X);
                                    double dV = Math.Abs(pp.Y - seqPnt2d[ii - 1].Y);
                                    if (dU > flecheU)
                                    {
                                        flecheU = dU;
                                    }

                                    if (dV > flecheV)
                                    {
                                        flecheV = dV;
                                    }
                                }

                                firstpoint = 2;
                            }
                        }

                        nbpnts = seqPnt2d.Length();
                        pClass.Resize(1, nbpnts, false);
                        im1 = nbpnts - 1;
                        im0 = 1;
                        pClass[im1] = seqPnt2d.Value(im1);
                        pClass[nbpnts] = seqPnt2d.Value(nbpnts);
                        square = 0.0;
                        per = 0.0;
                        for (int ii = 1; ii < nbpnts; ii++, im0++, im1++)
                        {
                            if (im1 >= nbpnts)
                            {
                                im1 = 1;
                            }

                            pClass[ii] = seqPnt2d.Value(ii);
                            square +=
                                (pClass[im0].X - pClass[im1].X) * (pClass[im0].Y + pClass[im1].Y) * .5;
                            per += (pClass[im0].XY() - pClass[im1].XY()).Modulus();
                        }

                        expThick = Max(2.0 * Math.Abs(square) / per, 1e-7);
                        defl = Max(flecheU, flecheV);
                        discrDefl = Min(discrDefl * 0.1, expThick * 10.0);
                    }

                    //-- FlecheU*=10.0;
                    //-- FlecheV*=10.0;
                    if (nbE == 1 && flecheU < eps && flecheV < eps && Math.Abs(square) < eps)
                    {
                        _tabOrien.Append(1);
                    }
                    else
                    {
                        _tabOrien.Append(((square < 0.0) ? 1 : 0));
                    }

                    if (flecheU < _toluv)
                    {
                        flecheU = _toluv;
                    }

                    if (flecheV < _toluv)
                    {
                        flecheV = _toluv;
                    }

                    _tabClass.Append(new Class2d(pClass, flecheU, flecheV, _umin, _vmin, _umax, _vmax));
                } // if(nbpoints>3
                else
                {
                    isBadWire = true;
                    _tabOrien.Append(-1);
                    var xPClass = new NCollection_Array1<Point2D>(1, 2);
                    // OCCT reads the first two points even when the wire gave fewer (out of range with
                    // No_Standard_OutOfRange); the port throws Standard_OutOfRange (occt-issues.md)
                    xPClass[1] = seqPnt2d[1];
                    xPClass[2] = seqPnt2d[2];
                    _tabClass.Append(new Class2d(xPClass, flecheU, flecheV, _umin, _vmin, _umax, _vmax));
                }
            } // else if(WareIsNotEmpty
        } // for(FaceExplorer

        int nbtabclass = _tabClass.Length();

        if (nbtabclass > 0)
        {
            //-- If an error was detected on a wire: set all TabOrien to -1
            if (isBadWire)
            {
                _tabOrien[1] = -1;
            }

            if (surf.GetType() == SurfaceKind.Cone || surf.GetType() == SurfaceKind.Cylinder
                || surf.GetType() == SurfaceKind.Torus || surf.GetType() == SurfaceKind.Sphere
                || surf.GetType() == SurfaceOfRevolution)
            {
            }
            else
            {
            }

            if (surf.GetType() == SurfaceKind.Torus)
            {
            }
            else
            {
            }
        }
    }

    /// <summary>The state of a point far outside the face's UV box (IN if the face has no box).</summary>
    public Containment PerformInfinitePoint()
    {
        if (_umax == -RealLast() || _vmax == -RealLast() || _umin == RealLast() || _vmin == RealLast())
        {
            return (In);
        }

        Point2D p = new(_umin - (_umax - _umin), _vmin - (_vmax - _vmin));
        return (Perform(p, false));
    }

    /// <summary>
    /// The state of the point <paramref name="_Puv"/> in the face. On a periodic surface the point is also tried at
    /// its periods within the face's UV box, if <paramref name="recadreOnPeriodic"/>.
    /// </summary>
    public Containment Perform(in Point2D _Puv, bool recadreOnPeriodic = true)
        => Classify(_Puv, _toluv, recadreOnPeriodic, false);

    /// <summary>
    /// The state of the point <paramref name="_Puv"/> tested with an offset of <paramref name="tol"/>: ON where the
    /// polygons can't decide (internal use).
    /// </summary>
    public Containment TestOnRestriction(in Point2D _Puv, double tol, bool recadreOnPeriodic = true)
        => Classify(_Puv, tol, recadreOnPeriodic, true);

    // Perform and TestOnRestriction, which OCCT writes out twice: they differ in the polygon test (SiDans or
    // SiDans_OnMode with Tol), in what an undecided polygon gives (the face classifier with Toluv capped at 4, or ON)
    // and in the face classifier's tolerance for a bad wire (Toluv or Tol).
    private Containment Classify(in Point2D _Puv, double tol, bool recadreOnPeriodic, bool onMode)
    {
        int dedans;
        int nbtabclass = _tabClass.Length();

        if (nbtabclass == 0)
        {
            return (In);
        }

        //-- U1 is the First Param and U2 in this case is U1+Period
        double u = _Puv.X;
        double v = _Puv.Y;
        double uu = u, vv = v;

        var surf = new FaceSurfaceAdaptor();
        surf.Initialize(_face, false);
        bool isUPer = surf.IsUPeriodic;
        bool isVPer = surf.IsVPeriodic;
        double uperiod = isUPer ? surf.UPeriod() : 0.0;
        double vperiod = isVPer ? surf.VPeriod() : 0.0;
        Containment status = Unknown;
        bool urecadre = false, vrecadre = false;

        if (recadreOnPeriodic)
        {
            if (isUPer)
            {
                if (uu < _umin)
                {
                    while (uu < _umin)
                    {
                        uu += uperiod;
                    }
                }
                else
                {
                    while (uu >= _umin)
                    {
                        uu -= uperiod;
                    }

                    uu += uperiod;
                }
            }

            if (isVPer)
            {
                if (vv < _vmin)
                {
                    while (vv < _vmin)
                    {
                        vv += vperiod;
                    }
                }
                else
                {
                    while (vv >= _vmin)
                    {
                        vv -= vperiod;
                    }

                    vv += vperiod;
                }
            }
        }

        for (;;)
        {
            dedans = 1;
            Point2D puv = new(u, v);

            if (_tabOrien[1] != -1)
            {
                for (int n = 1; n <= nbtabclass; n++)
                {
                    int cur = onMode ? (int)_tabClass[n].SiDans_OnMode(puv, tol) : (int)_tabClass[n].SiDans(puv);
                    if (cur == 1)
                    {
                        if (_tabOrien[n] == 0)
                        {
                            dedans = -1;
                            break;
                        }
                    }
                    else if (cur == -1)
                    {
                        if (_tabOrien[n] == 1)
                        {
                            dedans = -1;
                            break;
                        }
                    }
                    else
                    {
                        dedans = 0;
                        break;
                    }
                }

                if (dedans == 0)
                {
                    if (onMode)
                    {
                        status = On;
                    }
                    else
                    {
                        var classifier = new FaceClassifier();
                        double m_Toluv = (_toluv > 4.0) ? 4.0 : _toluv;
                        // aClassifier.Perform(Face,Puv,Toluv);
                        classifier.Perform(_face, puv, m_Toluv);
                        status = classifier.State();
                    }
                }

                if (dedans == 1)
                {
                    status = In;
                }

                if (dedans == -1)
                {
                    status = Out;
                }
            }
            else
            {
                //-- TabOrien(1)=-1    False Wire
                var classifier = new FaceClassifier();
                classifier.Perform(_face, puv, onMode ? tol : _toluv);
                status = classifier.State();
            }

            if (!recadreOnPeriodic || (!isUPer && !isVPer))
            {
                return status;
            }

            if (status == In || status == On)
            {
                return status;
            }

            if (!urecadre)
            {
                u = uu;
                urecadre = true;
            }
            else if (isUPer)
            {
                u += uperiod;
            }

            if (u > _umax || !isUPer)
            {
                if (!vrecadre)
                {
                    v = vv;
                    vrecadre = true;
                }
                else if (isVPer)
                {
                    v += vperiod;
                }

                u = uu;

                if (v > _vmax || !isVPer)
                {
                    return status;
                }
            }
        } // for (;;)
    }

    /// <summary>OCCT's Destroy: clears the polygons, after which every point is IN.</summary>
    internal void Destroy() => _tabClass.Clear();
}
