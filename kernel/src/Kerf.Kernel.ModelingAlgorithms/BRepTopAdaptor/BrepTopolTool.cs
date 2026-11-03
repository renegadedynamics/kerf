// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_TopolTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_TopolTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Topology.Adaptors;

/// <summary>
/// The topological tool of a face for the intersection algorithms: its restrictions are the pcurves of its edges, its
/// vertices the edges' vertices, and points are classified with a <see cref="FClass2d"/>.
/// </summary>
/// <remarks>
/// A working object (decision 101). The surface given to <see cref="Initialize(SurfaceAdaptorBase)"/> must be a
/// <see cref="FaceSurfaceAdaptor"/>, the curves <see cref="EdgeCurve2DAdaptor"/>s. The 2D classifier is built at
/// the first classification, with that call's tolerance, and kept (OCCT's <c>void*</c>).
/// </remarks>
public class BrepTopolTool : TopolTool
{
    private readonly Explorer _vIterator = new();
    private Face _face = new();
    private FClass2d? _fClass2d;
    private EdgeCurve2DAdaptor? _curve;
    private readonly NCollection_List<EdgeCurve2DAdaptor> _curves = new();
    private NCollection_List<EdgeCurve2DAdaptor>.Iterator _cIterator = new();
    private double _u0;
    private double _v0;
    private double _du;
    private double _dv;

    /// <summary>An empty tool.</summary>
    public BrepTopolTool()
    {
        _fClass2d = null;
        _u0 = 0.0;
        _v0 = 0.0;
        _du = 0.0;
        _dv = 0.0;
        _nbSamplesU = -1;
    }

    /// <summary>The tool of the face of <paramref name="s"/> (a BRepAdaptor_Surface).</summary>
    public BrepTopolTool(SurfaceAdaptorBase s)
    {
        _fClass2d = null;
        Initialize(s);
        // myS = S;
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as in OCCT.</summary>
    public override void Initialize()
    {
        throw new MethodNotImplementedException("BRepTopAdaptor_TopolTool::Initialize()");
    }

    /// <summary>Loads the face of <paramref name="s"/> (a BRepAdaptor_Surface) and the pcurves of its edges.</summary>
    /// <exception cref="ConstructionErrorException">The surface is not a BRepAdaptor_Surface.</exception>
    public override void Initialize(SurfaceAdaptorBase s)
    {
        if (s is not FaceSurfaceAdaptor brhs)
        {
            throw new ConstructionErrorException();
        }

        Shape s_wnt = brhs.Face();
        s_wnt.Orient(Forward);
        _face = ShapeCasts.Face(s_wnt);
        _fClass2d = null;
        _nbSamplesU = -1;
        _s = s;
        _curves.Clear();
        var ex = new Explorer(_face, global::Kerf.Topology.ShapeType.Edge);
        for (; ex.More(); ex.Next())
        {
            var curve = new EdgeCurve2DAdaptor(ShapeCasts.Edge(ex.Current()), _face);
            _curves.Append(curve);
        }

        _cIterator = new NCollection_List<EdgeCurve2DAdaptor>.Iterator();
    }

    /// <summary>Loads the restriction <paramref name="c"/> (a BRepAdaptor_Curve2d) for the vertex iteration.</summary>
    /// <exception cref="ConstructionErrorException">The curve is not a BRepAdaptor_Curve2d.</exception>
    public override void Initialize(Curve2DAdaptorBase c)
    {
        _curve = c as EdgeCurve2DAdaptor;
        if (_curve is null)
        {
            throw new ConstructionErrorException();
        }
    }

    /// <summary>Starts the iteration over the restrictions.</summary>
    public override void Init() => _cIterator.Initialize(_curves);

    public override bool More() => _cIterator.More();

    public override void Next() => _cIterator.Next();

    /// <summary>The current restriction (a BRepAdaptor_Curve2d).</summary>
    public override Curve2DAdaptorBase Value() => _cIterator.Value();

    /// <summary>The edge of the current restriction (a boxed TopoDS_Edge: OCCT's <c>void*</c> to it).</summary>
    // modified by NIZNHY-PKV Tue Mar 27 14:23:40 2001 f
    public override object? Edge()
    {
        EdgeCurve2DAdaptor hCurve = _cIterator.Value();
        return hCurve.Edge();
    }

    /// <summary>Starts the iteration over the vertices of the restriction loaded by <see cref="Initialize(Curve2DAdaptorBase)"/>.</summary>
    public override void InitVertexIterator() => _vIterator.Init(_curve!.Edge(), global::Kerf.Topology.ShapeType.Vertex);

    public override bool MoreVertex() => _vIterator.More();

    public override void NextVertex() => _vIterator.Next();

    /// <summary>The current vertex (a new BRepTopAdaptor_HVertex).</summary>
    public override HVertex Vertex() => new TopologyHVertex(ShapeCasts.Vertex(_vIterator.Current()), _curve!);

    /// <summary>
    /// The state of the point <paramref name="p"/> in the face (UNKNOWN if no face is loaded). The 2D classifier is
    /// built at the first call, with its tolerance <paramref name="tol"/>.
    /// </summary>
    public override Containment Classify(in Point2D p, double tol, bool recadreOnPeriodic = true)
    {
        if (_face.IsNull)
        {
            return Unknown;
        }

        _fClass2d ??= new FClass2d(_face, tol);

        return (_fClass2d.Perform(p, recadreOnPeriodic));
    }

    /// <summary>True if the point <paramref name="p"/> is on the face's boundary within <paramref name="tol"/>.</summary>
    public override bool IsThePointOn(in Point2D p, double tol, bool recadreOnPeriodic = true)
    {
        _fClass2d ??= new FClass2d(_face, tol);

        return (On == _fClass2d.TestOnRestriction(p, tol, recadreOnPeriodic));
    }

    /// <summary>Drops the 2D classifier (OCCT's Destroy, also called by its destructor).</summary>
    public void Destroy() => _fClass2d = null;

    /// <summary>The orientation of the edge of the arc <paramref name="c"/> (a BRepAdaptor_Curve2d).</summary>
    public override Orientation Orientation(Curve2DAdaptorBase c)
    {
        var brhc = (EdgeCurve2DAdaptor)c;
        return brhc.Edge().Orientation;
    }

    /// <summary>The orientation of the vertex.</summary>
    public override Orientation Orientation(HVertex c) => base.Orientation(c);

    //-- ============================================================
    //-- methods  used for samples
    //-- ============================================================

    private static void Analyse(NCollection_Array2<Point3D> array2,
                                int nbup,
                                int nbvp,
                                ref int myNbSamplesU,
                                ref int myNbSamplesV)
    {
        Vector3D vi = new(), vip1 = new();
        int sh, nbch, i, j;

        sh = 1;
        nbch = 0;
        if (nbvp > 2)
        {
            for (i = 2; i < nbup; i++)
            {
                Point3D a = array2.Value(i, 1);
                Point3D b = array2.Value(i, 2);
                Point3D c = array2.Value(i, 3);
                vi = vi.WithCoord(c.X - b.X - b.X + a.X,
                            c.Y - b.Y - b.Y + a.Y,
                            c.Z - b.Z - b.Z + a.Z);

                int locnbch = 0;

                for (j = 3; j < nbvp; j++)
                {
                    //-- test
                    Point3D ax = array2.Value(i, j - 1);
                    Point3D bx = array2.Value(i, j);
                    Point3D cx = array2.Value(i, j + 1);
                    vip1 = vip1.WithCoord(cx.X - bx.X - bx.X + ax.X,
                                  cx.Y - bx.Y - bx.Y + ax.Y,
                                  cx.Z - bx.Z - bx.Z + ax.Z);
                    double pd = vi.Dot(vip1);
                    vi = vip1;
                    if (pd > 1.0e-7 || pd < -1.0e-7)
                    {
                        if (pd > 0)
                        {
                            if (sh == -1)
                            {
                                sh = 1;
                                locnbch++;
                            }
                        }
                        else
                        {
                            if (sh == 1)
                            {
                                sh = -1;
                                locnbch++;
                            }
                        }
                    }
                }

                if (locnbch > nbch)
                {
                    nbch = locnbch;
                }
            }
        }

        myNbSamplesV = nbch + 5;

        nbch = 0;
        if (nbup > 2)
        {
            for (j = 2; j < nbvp; j++)
            {
                Point3D a = array2.Value(1, j);
                Point3D b = array2.Value(2, j);
                Point3D c = array2.Value(3, j);
                vi = vi.WithCoord(c.X - b.X - b.X + a.X,
                            c.Y - b.Y - b.Y + a.Y,
                            c.Z - b.Z - b.Z + a.Z);

                int locnbch = 0;

                for (i = 3; i < nbup; i++)
                {
                    //-- test
                    Point3D ax = array2.Value(i - 1, j);
                    Point3D bx = array2.Value(i, j);
                    Point3D cx = array2.Value(i + 1, j);
                    vip1 = vip1.WithCoord(cx.X - bx.X - bx.X + ax.X,
                                  cx.Y - bx.Y - bx.Y + ax.Y,
                                  cx.Z - bx.Z - bx.Z + ax.Z);
                    double pd = vi.Dot(vip1);
                    vi = vip1;
                    if (pd > 1.0e-7 || pd < -1.0e-7)
                    {
                        if (pd > 0)
                        {
                            if (sh == -1)
                            {
                                sh = 1;
                                locnbch++;
                            }
                        }
                        else
                        {
                            if (sh == 1)
                            {
                                sh = -1;
                                locnbch++;
                            }
                        }
                    }
                }

                if (locnbch > nbch)
                {
                    nbch = locnbch;
                }
            }
        }

        myNbSamplesU = nbch + 5;
    }

    /// <summary>Computes the numbers of sample points in U and V and their spacing.</summary>
    public override void ComputeSamplePoints()
    {
        double uinf, usup, vinf, vsup;
        uinf = _s!.FirstUParameter();
        usup = _s.LastUParameter();
        vinf = _s.FirstVParameter();
        vsup = _s.LastVParameter();
        if (usup < uinf)
        {
            double temp = uinf;
            uinf = usup;
            usup = temp;
        }

        if (vsup < vinf)
        {
            double temp = vinf;
            vinf = vsup;
            vsup = temp;
        }

        if (uinf == RealFirst() && usup == RealLast())
        {
            uinf = -1.0e5;
            usup = 1.0e5;
        }
        else if (uinf == RealFirst())
        {
            uinf = usup - 2.0e5;
        }
        else if (usup == RealLast())
        {
            usup = uinf + 2.0e5;
        }

        if (vinf == RealFirst() && vsup == RealLast())
        {
            vinf = -1.0e5;
            vsup = 1.0e5;
        }
        else if (vinf == RealFirst())
        {
            vinf = vsup - 2.0e5;
        }
        else if (vsup == RealLast())
        {
            vsup = vinf + 2.0e5;
        }

        int nbsu, nbsv;
        SurfaceKind typS = _s.GetType();
        switch (typS)
        {
            case Plane:
            {
                nbsv = 2;
                nbsu = 2;
            }
                break;
            case SurfaceKind.BezierSurface:
            {
                nbsv = 3 + _s.VPoleCount;
                nbsu = 3 + _s.UPoleCount;
            }
                break;
            case SurfaceKind.BSplineSurface:
            {
                nbsv = _s.VKnotCount;
                nbsv *= _s.VDegree();
                if (nbsv < 4)
                {
                    nbsv = 4;
                }

                nbsu = _s.UKnotCount;
                nbsu *= _s.UDegree();
                if (nbsu < 4)
                {
                    nbsu = 4;
                }
            }
                break;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            {
                //-- Set 15 for 2pi
                //-- Not enough ->25 for 2pi
                nbsu = (int)(8 * (usup - uinf));
                nbsv = (int)(7 * (vsup - vinf));
                if (nbsu < 5)
                {
                    nbsu = 5;
                }

                if (nbsv < 5)
                {
                    nbsv = 5;
                }

                if (nbsu > 30)
                {
                    nbsu = 30; // modif HRT buc60462
                }

                if (nbsv > 15)
                {
                    nbsv = 15;
                }
                //-- printf("\n nbsu=%d nbsv=%d\n",nbsu,nbsv);
            }
                break;
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
            {
                nbsv = 15;
                nbsu = 25;
            }
                break;
            default:
            {
                nbsu = 10;
                nbsv = 10;
            }
                break;
        }

        //-- If the number of points is too great, analyze
        //--
        //--

        if (nbsu < 10)
        {
            nbsu = 10;
        }

        if (nbsv < 10)
        {
            nbsv = 10;
        }

        _nbSamplesU = nbsu;
        _nbSamplesV = nbsv;

        //-- printf("\n BRepTopAdaptor_TopolTool NbSu=%d NbSv=%d ",nbsu,nbsv);
        if (nbsu > 10 || nbsv > 10)
        {
            if (typS == SurfaceKind.BSplineSurface)
            {
                global::Kerf.Geometry.BSplineSurface bspl = _s.BSpline();
                int nbup = bspl.UPoleCount;
                int nbvp = bspl.VPoleCount;
                NCollection_Array2<Point3D> array2 = bspl.InternalPoles();
                Analyse(array2, nbup, nbvp, ref _nbSamplesU, ref _nbSamplesV);
                nbsu = _nbSamplesU;
                nbsv = _nbSamplesV;
                //-- printf("\n Apres analyse BSPline  NbSu=%d NbSv=%d ",myNbSamplesU,myNbSamplesV);
            }
            else if (typS == SurfaceKind.BezierSurface)
            {
                global::Kerf.Geometry.BezierSurface bez = _s.Bezier();
                int nbup = bez.UPoleCount;
                int nbvp = bez.VPoleCount;
                NCollection_Array2<Point3D> array2 = bez.InternalPoles();
                Analyse(array2, nbup, nbvp, ref _nbSamplesU, ref _nbSamplesV);
                nbsu = _nbSamplesU;
                nbsv = _nbSamplesV;
                //-- printf("\n Apres analyse Bezier  NbSu=%d NbSv=%d ",myNbSamplesU,myNbSamplesV);
            }
        }

        if (nbsu < 10)
        {
            nbsu = 10;
        }

        if (nbsv < 10)
        {
            nbsv = 10;
        }

        _nbSamplesU = nbsu;
        _nbSamplesV = nbsv;

        _u0 = uinf;
        _v0 = vinf;

        _du = (usup - uinf) / (_nbSamplesU + 1);
        _dv = (vsup - vinf) / (_nbSamplesV + 1);
    }

    /// <summary>The number of sample points in U (computed at the first call).</summary>
    public override int NbSamplesU()
    {
        if (_nbSamplesU < 0)
        {
            ComputeSamplePoints();
        }

        return (_nbSamplesU);
    }

    /// <summary>The number of sample points in V (computed at the first call).</summary>
    public override int NbSamplesV()
    {
        if (_nbSamplesU < 0)
        {
            ComputeSamplePoints();
        }

        return (_nbSamplesV);
    }

    /// <summary>The number of sample points (computed at the first call).</summary>
    public override int NbSamples()
    {
        if (_nbSamplesU < 0)
        {
            ComputeSamplePoints();
        }

        return (_nbSamplesU * _nbSamplesV);
    }

    /// <summary>The sample point of index <paramref name="i"/>, in the parametric domain and in 3D.</summary>
    public override void SamplePoint(int i, out Point2D p2d, out Point3D p3d)
    {
        int iv = 1 + i / _nbSamplesU;
        int iu = 1 + i - (iv - 1) * _nbSamplesU;
        double u = _u0 + iu * _du;
        double v = _v0 + iv * _dv;
        p2d = new Point2D(u, v);
        p3d = _s!.Value(u, v);
    }

    /// <summary>True if one of the bounds of the surface is infinite.</summary>
    public override bool DomainIsInfinite()
    {
        double uinf, usup, vinf, vsup;
        uinf = _s!.FirstUParameter();
        usup = _s.LastUParameter();
        vinf = _s.FirstVParameter();
        vsup = _s.LastVParameter();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uinf))
        {
            return (true);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(usup))
        {
            return (true);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vinf))
        {
            return (true);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vsup))
        {
            return (true);
        }

        return (false);
    }

    /// <summary>True: arcs and vertices have 3D representations.</summary>
    public override bool Has3d() => true;

    /// <summary>The 3D tolerance of the edge of the arc <paramref name="c"/>.</summary>
    /// <exception cref="DomainErrorException">The arc is not a BRepAdaptor_Curve2d with an edge.</exception>
    public override double Tol3d(Curve2DAdaptorBase c)
    {
        if (c is not EdgeCurve2DAdaptor brhc)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: arc has no 3d representation");
        }

        Edge edge = brhc.Edge();
        if (edge.IsNull)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: arc has no 3d representation");
        }

        return Tool.Tolerance(edge);
    }

    /// <summary>The 3D tolerance of the vertex <paramref name="v"/>.</summary>
    /// <exception cref="DomainErrorException">The vertex is not a BRepTopAdaptor_HVertex with a vertex.</exception>
    public override double Tol3d(HVertex v)
    {
        if (v is not TopologyHVertex brhv)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: vertex has no 3d representation");
        }

        Vertex ver = brhv.Vertex();
        if (ver.IsNull)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: vertex has no 3d representation");
        }

        return Tool.Tolerance(ver);
    }

    /// <summary>The 3D point of the vertex <paramref name="v"/>.</summary>
    /// <exception cref="DomainErrorException">The vertex is not a BRepTopAdaptor_HVertex with a vertex.</exception>
    public override Point3D Pnt(HVertex v)
    {
        if (v is not TopologyHVertex brhv)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: vertex has no 3d representation");
        }

        Vertex ver = brhv.Vertex();
        if (ver.IsNull)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: vertex has no 3d representation");
        }

        return Tool.Pnt(ver);
    }
}
