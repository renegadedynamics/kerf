// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Intersector.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Intersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.TwoD.Position2D;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The intersections of the hidden-line algorithm: of a projected edge with itself, of two projected edges (each
/// shortened at its ends as asked), a simulated single crossing, and of a line of sight with a face's surface.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. The edges' curves and the loaded surface are kept by reference, as OCCT keeps
/// pointers. The polyhedron of a non-quadric surface is built at the first line intersected and kept until the next
/// <see cref="Load"/>. OCCT's <c>PERF</c> counters (a compile-time option, off) are left out.
/// </remarks>
internal sealed class HiddenLineIntersector
{
    private IntersectionPoint2D _singlePoint;
    private int _typePerform;
    private readonly CInter _intersector = new();
    private readonly InterCSurf _csIntersector = new();
    private HlrSurface? _surface;
    private ThePolyhedronOfInterCSurf? _polyhedron;

    public HiddenLineIntersector()
    {
        _typePerform = 0;
        _surface = null;
        _polyhedron = null;
        // OCCT_DEBUG / PERF statistics left out.
        const int minNbHLRSamples = 4;
        _intersector.SetMinNbSamples(minNbHLRSamples);
    }

    /// <summary>The self-intersection of the edge's projection, its ends cut by the fractions theDa1 and theDb1.</summary>
    public void Perform(HiddenLineEdgeData edge1, double da1, double db1)
    {
        HiddenLineCurve myC1 = edge1.Curve();

        _typePerform = 1;

        double a, b, d, tol;
        float ta, tb;

        edge1.Status().Bounds(out a, out ta, out b, out tb);
        d = b - a;
        if (da1 != 0)
        {
            a = a + d * da1;
        }
        if (db1 != 0)
        {
            b = b - d * db1;
        }
        myC1.D0(a, out Point2D pa);
        myC1.D0(b, out Point2D pb);
        a = myC1.Parameter2d(a);
        b = myC1.Parameter2d(b);
        TwoDDomain d1 = new(pa, a, (double)ta, pb, b, (double)tb);

        // modified by jgv, 18.04.2016 for OCC27341
        // tol = (double)(((HLRBRep_EdgeData*) A1)->Tolerance());
        tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        _intersector.Perform(myC1, d1, tol, tol);
    }

    /// <summary>
    /// The intersection of the projections of two edges, each shortened at its ends as asked (theDa, theDb); with
    /// theNoBound (the edges share a vertex), the cut grows until the domains' ends are far enough apart.
    /// </summary>
    public void Perform(int na, HiddenLineEdgeData edge1, double da1, double db1, int nb,
                        HiddenLineEdgeData edge2, double da2, double db2, bool enBout)
    {
        HiddenLineCurve myC1 = edge1.Curve();
        HiddenLineCurve myC2 = edge2.Curve();

        _typePerform = 1;

        Point2D pa1, pb1, pa2, pb2;
        Vector2D va1, vb1, va2, vb2;
        double a1, b1, a2, b2, d, dd, tol, tol1, tol2;
        float ta, tb;

        // modified by jgv, 18.04.2016 for OCC27341
        // tol1 = theEdge1->Tolerance();
        // tol2 = theEdge2->Tolerance();
        tol1 = global::Kerf.Engine.Foundation.Precision.Confusion();
        tol2 = global::Kerf.Engine.Foundation.Precision.Confusion();

        if (tol1 > tol2)
        {
            tol = tol1;
        }
        else
        {
            tol = tol2;
        }

        bool pasBon;
        double decalagea1 = 100.0;
        double decalagea2 = 100.0;
        double decalageb1 = 100.0;
        double decalageb2 = 100.0;
        do
        {
            pasBon = false;
            edge1.Status().Bounds(out a1, out ta, out b1, out tb); //--   -> Parametres 3d
            double mtol = tol;
            if (mtol < ta)
            {
                mtol = ta;
            }
            if (mtol < tb)
            {
                mtol = tb;
            }
            d = b1 - a1;

            double pdist = tol;
            if (pdist < 0.0000001)
            {
                pdist = 0.0000001;
            }

            if (da1 != 0)
            {
                //-- a = a + d * da1;
                myC1.D1(a1, out pa1, out va1);
                double qwe = va1.Magnitude;
                if (qwe > 1e-12)
                {
                    dd = pdist * decalagea1 / qwe;
                    if (dd < d * 0.4)
                    {
                        a1 += dd;
                    }
                    else
                    {
                        a1 += d * da1;
                        decalagea1 = -1;
                    }
                }
                else
                {
                    a1 += d * da1;
                    decalagea1 = -1;
                }
            }

            if (db1 != 0)
            {
                //-- b = b - d * db1;
                myC1.D1(b1, out pb1, out vb1);
                double qwe = vb1.Magnitude;
                if (qwe > 1e-12)
                {
                    dd = pdist * decalageb1 / qwe;
                    if (dd < d * 0.4)
                    {
                        b1 -= dd;
                    }
                    else
                    {
                        b1 -= d * db1;
                        decalageb1 = -1;
                    }
                }
                else
                {
                    b1 -= d * db1;
                    decalageb1 = -1;
                }
            }

            myC1.D0(a1, out pa1);
            myC1.D0(b1, out pb1);

            a1 = myC1.Parameter2d(a1);
            b1 = myC1.Parameter2d(b1);

            if (enBout)
            {
                ta = tb = -1.0f;
            }

            if (ta > tol)
            {
                ta = (float)tol;
            }
            if (tb > tol)
            {
                tb = (float)tol;
            }

            TwoDDomain d1 = new(pa1, a1, (double)ta, pb1, b1, (double)tb);

            edge2.Status().Bounds(out a2, out ta, out b2, out tb);
            mtol = tol;
            if (mtol < ta)
            {
                mtol = ta;
            }
            if (mtol < tb)
            {
                mtol = tb;
            }

            d = b2 - a2;

            if (da2 != 0)
            {
                //-- a = a + d * da2;
                myC2.D1(a2, out pa2, out va2);
                double qwe = va2.Magnitude;
                if (qwe > 1e-12)
                {
                    dd = pdist * decalagea2 / qwe;
                    if (dd < d * 0.4)
                    {
                        a2 += dd;
                    }
                    else
                    {
                        a2 += d * da2;
                        decalagea2 = -1;
                    }
                }
                else
                {
                    a2 += d * da2;
                    decalagea2 = -1;
                }
            }

            if (db2 != 0)
            {
                //-- b = b - d * db2;
                myC2.D1(b2, out pb2, out vb2);
                double qwe = vb2.Magnitude;
                if (qwe > 1e-12)
                {
                    dd = pdist * decalageb2 / qwe;
                    if (dd < d * 0.4)
                    {
                        b2 -= dd;
                    }
                    else
                    {
                        b2 -= d * db2;
                        decalageb2 = -1;
                    }
                }
                else
                {
                    b2 -= d * db2;
                    decalageb2 = -1;
                }
            }

            myC2.D0(a2, out pa2);
            myC2.D0(b2, out pb2);

            a2 = myC2.Parameter2d(a2);
            b2 = myC2.Parameter2d(b2);

            if (enBout)
            {
                ta = tb = -1.0f;
            }

            if (ta > tol)
            {
                ta = (float)tol;
            }
            if (tb > tol)
            {
                tb = (float)tol;
            }

            TwoDDomain d2 = new(pa2, a2, (double)ta, pb2, b2, (double)tb);

            if (enBout)
            {
                double a1a2 = (da1 != 0 || da2 != 0) ? pa1.DistanceTo(pa2) : RealLast();
                double a1b2 = (da1 != 0 || db2 != 0) ? pa1.DistanceTo(pb2) : RealLast();
                double b1a2 = (db1 != 0 || da2 != 0) ? pb1.DistanceTo(pa2) : RealLast();
                double b1b2 = (db1 != 0 || db2 != 0) ? pb1.DistanceTo(pb2) : RealLast();

                int cote = 1;
                double mindist = a1a2; //-- cas 1
                if (mindist > a1b2)
                {
                    mindist = a1b2;
                    cote = 2;
                }
                if (mindist > b1a2)
                {
                    mindist = b1a2;
                    cote = 3;
                }
                if (mindist > b1b2)
                {
                    mindist = b1b2;
                    cote = 4;
                }


                if (mindist < tol * 1000)
                {
                    pasBon = true;
                    switch (cote)
                    {
                        case 1:
                        {
                            decalagea1 *= 2;
                            decalagea2 *= 2;
                            break;
                        }
                        case 2:
                        {
                            decalagea1 *= 2;
                            decalageb2 *= 2;
                            break;
                        }
                        case 3:
                        {
                            decalageb1 *= 2;
                            decalagea2 *= 2;
                            break;
                        }
                        default:
                        {
                            decalageb1 *= 2;
                            decalageb2 *= 2;
                            break;
                        }
                    }
                    if (decalagea1 < 0.0 || decalagea2 < 0.0 || decalageb1 < 0.0 || decalageb2 <= 0.0)
                    {
                        pasBon = false;
                    }
                }
            }
            if (!pasBon)
            {
                _intersector.Perform(myC1, d1, myC2, d2, tol, tol);
            }
        } while (pasBon);

        // PERF statistics left out.
    }

    /// <summary>Records a single crossing of the two edges at the parameters theU and theV (2D) without computing it.</summary>
    public void SimulateOnePoint(HiddenLineEdgeData edge1, double u, HiddenLineEdgeData edge2, double v)
    {
        HiddenLineCurve myC1 = edge1.Curve();
        HiddenLineCurve myC2 = edge2.Curve();

        double u3 = myC1.Parameter3d(u);
        double v3 = myC2.Parameter3d(v);
        myC1.D1(u3, out Point2D p13, out Vector2D t13);
        myC2.D1(v3, out Point2D p23, out Vector2D t23);

        Transition tr1 = new(), tr2 = new();
        Position2D pos1, pos2;
        pos1 = pos2 = Position2D.Middle;

        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref t13, ref tr1, pos2, ref t23, ref tr2, 0.0);
        _typePerform = 0;
        _singlePoint.SetValues(p13, u, v, tr1, tr2, false);
    }

    /// <summary>Loads the surface the next lines are intersected with; drops the polyhedron of the last one.</summary>
    public void Load(HlrSurface surface)
    {
        _surface = surface;
        _polyhedron?.Destroy();
        _polyhedron = null;
    }

    /// <summary>The intersection of the line L with the loaded surface, up to the parameter P on the line.</summary>
    public void Perform(in Line3D l, double P)
    {
        _typePerform = 2;
        SurfaceKind typ = HiddenLineSurfaceTool.GetType(_surface!);
        switch (typ)
        {
            case Plane:
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                _csIntersector.Perform(l, _surface!);
                break;
            default:
            {
                if (_polyhedron is null)
                {
                    int nbsu, nbsv;
                    double u1, v1, u2, v2;
                    u1 = HiddenLineSurfaceTool.FirstUParameter(_surface!);
                    v1 = HiddenLineSurfaceTool.FirstVParameter(_surface!);
                    u2 = HiddenLineSurfaceTool.LastUParameter(_surface!);
                    v2 = HiddenLineSurfaceTool.LastVParameter(_surface!);
                    nbsu = HiddenLineSurfaceTool.NbSamplesU(_surface!, u1, u2);
                    nbsv = HiddenLineSurfaceTool.NbSamplesV(_surface!, v1, v2);
                    _polyhedron = new ThePolyhedronOfInterCSurf(_surface!, nbsu, nbsv, u1, v1, u2, v2);
                }
                double x0, y0, z0, x1, y1, z1, pmin, pmax; //,pp;
                _polyhedron.Bounding().Get(out x0, out y0, out z0, out x1, out y1, out z1);
                double p;
                p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, new Point3D(x0, y0, z0));
                pmin = pmax = p;
                p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, new Point3D(x0, y0, z1));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, new Point3D(x1, y0, z0));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, new Point3D(x1, y0, z1));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, new Point3D(x0, y1, z0));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, new Point3D(x0, y1, z1));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, new Point3D(x1, y1, z0));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, new Point3D(x1, y1, z1));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }
                pmin -= 0.000001;
                pmax += 0.000001;

                if (pmin > P)
                {
                    pmin = pmax + 1;
                    pmax = pmax + 2;
                } //-- on va rejeter avec les boites
                else
                {
                    if (pmax > P)
                    {
                        pmax = P + 0.0000001;
                    }
                }
                ThePolygonOfInterCSurf polygon = new(l, pmin, pmax, 3);
                _csIntersector.Perform(l, polygon, _surface!, _polyhedron);

                break;
            }
        }
        // PERF statistics left out.
    }

    public bool IsDone()
    {
        if (_typePerform == 1)
        {
            return _intersector.IsDone;
        }
        else if (_typePerform == 2)
        {
            return _csIntersector.IsDone;
        }
        else
        {
            return (true);
        }
    }

    public int NbPoints()
    {
        if (_typePerform == 43)
        {
            return (0);
        }
        if (_typePerform == 1)
        {
            return _intersector.PointCount;
        }
        else if (_typePerform == 2)
        {
            return _csIntersector.PointCount;
        }
        else
        {
            return (1);
        }
    }

    /// <summary>The point N of the 2D intersection (the simulated point after <see cref="SimulateOnePoint"/>).</summary>
    public IntersectionPoint2D Point(int n)
    {
        if (_typePerform == 0)
        {
            return (_singlePoint);
        }
        else
        {
            return _intersector.Point(n);
        }
    }

    /// <summary>The point N of the line/surface intersection.</summary>
    public CurveSurfaceIntersectionPoint CSPoint(int n) => _csIntersector.Point(n);

    public int NbSegments()
    {
        if (_typePerform == 1)
        {
            return _intersector.SegmentCount;
        }
        else if (_typePerform == 2)
        {
            return _csIntersector.SegmentCount;
        }
        else
        {
            return (0);
        }
    }

    public IntersectionSegment2D Segment(int n) => _intersector.Segment(n);

    public IntersectionSegment CSSegment(int n) => _csIntersector.Segment(n);

    /// <summary>Drops the polyhedron.</summary>
    public void Destroy()
    {
        _polyhedron?.Destroy();
        _polyhedron = null;
    }
}
