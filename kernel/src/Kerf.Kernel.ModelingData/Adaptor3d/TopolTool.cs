// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_TopolTool.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_TopolTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// The default topological tool of the intersection and outline algorithms, based on the Umin, Vmin, Umax, Vmax
/// of a surface adaptor: the restrictions of the parametric domain are 2D lines, their ends are the vertices.
/// All methods may be redefined by a derived class (BRepTopAdaptor_TopolTool).
/// </summary>
/// <remarks>A working object (decision 101): <c>Initialize</c> and the iterators are public, as in OCCT.</remarks>
public class TopolTool : Transient
{
    private static double myInfinite => global::Kerf.Engine.Foundation.Precision.Infinite();

    // Absolute possible minimum of sample points (restriction of IntPolyh).
    private const int MinPnts = 4;

    protected SurfaceAdaptorBase? _s;
    protected int _nbSamplesU;
    protected int _nbSamplesV;
    protected NCollection_HArray1<double>? _uPars;
    protected NCollection_HArray1<double>? _vPars;

    private int _nbRestr;
    private int _idRestr;
    private double _uinf;
    private double _usup;
    private double _vinf;
    private double _vsup;
    private readonly Line2d?[] _restr = new Line2d?[4];
    private int _nbVtx;
    private int _idVtx;
    private readonly HVertex?[] _vtx = new HVertex?[2];

    public TopolTool()
    {
        _nbSamplesU = -1;
        _nbSamplesV = -1;
        _nbRestr = 0;
        _idRestr = 0;
        _uinf = 0.0;
        _usup = 0.0;
        _vinf = 0.0;
        _vsup = 0.0;
        _nbVtx = 0;
        _idVtx = 0;
    }

    public TopolTool(SurfaceAdaptorBase surface)
    {
        // OCCT: myNbSamplesV, nbVtx and idVtx are left uninitialised here (C# zeroes them).
        Initialize(surface);
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as in OCCT.</summary>
    public virtual void Initialize()
    {
        throw new MethodNotImplementedException("Adaptor3d_TopolTool::Initialize ()");
    }

    /// <summary>Takes the parametric domain of the surface and builds its restrictions.</summary>
    public virtual void Initialize(SurfaceAdaptorBase s)
    {
        double pinf, psup, deltap;

        _nbSamplesU = -1;
        _uinf = s.FirstUParameter(); // where UIntervalFirst ??
        _vinf = s.FirstVParameter();
        _usup = s.LastUParameter();
        _vsup = s.LastVParameter();
        _nbRestr = 0;
        _idRestr = 0;

        bool uinfinfinite = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_uinf);
        bool usupinfinite = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_usup);
        bool vinfinfinite = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_vinf);
        bool vsupinfinite = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_vsup);

        if (!vinfinfinite)
        {
            deltap = Min(_usup - _uinf, 2.0 * myInfinite);
            if (_uinf >= -myInfinite)
            {
                pinf = _uinf;
                psup = pinf + deltap;
            }
            else if (_usup <= myInfinite)
            {
                psup = _usup;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            _restr[_nbRestr] = new Line2d(new Point2D(0.0, _vinf), new Direction2D(Direction2D.D.X), pinf, psup);
            _nbRestr++;
        }

        if (!usupinfinite)
        {
            deltap = Min(_vsup - _vinf, 2.0 * myInfinite);
            if (_vinf >= -myInfinite)
            {
                pinf = _vinf;
                psup = pinf + deltap;
            }
            else if (_vsup <= myInfinite)
            {
                psup = _vsup;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            _restr[_nbRestr] = new Line2d(new Point2D(_usup, 0.0), new Direction2D(Direction2D.D.Y), pinf, psup);
            _nbRestr++;
        }

        if (!vsupinfinite)
        {
            deltap = Min(_usup - _uinf, 2.0 * myInfinite);
            if (-_usup >= -myInfinite)
            {
                pinf = -_usup;
                psup = pinf + deltap;
            }
            else if (-_uinf <= myInfinite)
            {
                psup = -_uinf;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            _restr[_nbRestr] = new Line2d(new Point2D(0.0, _vsup), new Direction2D(Direction2D.D.NX), pinf, psup);
            _nbRestr++;
        }

        if (!uinfinfinite)
        {
            deltap = Min(_vsup - _vinf, 2.0 * myInfinite);
            if (-_vsup >= -myInfinite)
            {
                pinf = -_vsup;
                psup = pinf + deltap;
            }
            else if (-_vinf <= myInfinite)
            {
                psup = -_vinf;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            _restr[_nbRestr] = new Line2d(new Point2D(_uinf, 0.0), new Direction2D(Direction2D.D.NY), pinf, psup);
            _nbRestr++;
        }

        _s = s;

        if (_nbRestr == 2 && s.GetType() == SurfaceKind.Cone)
        {
            double u = 0.0, v = 0.0;
            GetConeApexParam(s.Cone(), out u, out v);

            deltap = Min(_usup - _uinf, 2.0 * myInfinite);
            if (_uinf >= -myInfinite)
            {
                pinf = _uinf;
                psup = pinf + deltap;
            }
            else if (_usup <= myInfinite)
            {
                psup = _usup;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            _restr[_nbRestr] = new Line2d(new Point2D(u, v), new Direction2D(Direction2D.D.X), pinf, psup);
            _nbRestr++;
        }
    }

    /// <summary>Takes a restriction and builds its vertices (its finite ends).</summary>
    public virtual void Initialize(Curve2DAdaptorBase curve)
    {
        _nbVtx = 0;
        _idVtx = 0;
        double theUinf, theUsup;
        theUinf = curve.FirstParameter();
        theUsup = curve.LastParameter();
        //  if (!Precision::IsNegativeInfinite(theUinf)) {
        if (theUinf > -myInfinite)
        {
            _vtx[_nbVtx] = new HVertex(curve.Value(theUinf), Forward, 1.0e-8);
            _nbVtx++;
        }
        //  if (!Precision::IsPositiveInfinite(theUsup)) {
        if (theUsup < myInfinite)
        {
            _vtx[_nbVtx] = new HVertex(curve.Value(theUsup), Reversed, 1.0e-8);
            _nbVtx++;
        }
    }

    /// <summary>Starts the iteration over the restrictions.</summary>
    public virtual void Init()
    {
        _idRestr = 0;
    }

    public virtual bool More() => (_idRestr < _nbRestr);

    /// <summary>The current restriction.</summary>
    /// <exception cref="DomainErrorException">The iteration is over.</exception>
    public virtual Curve2DAdaptorBase Value()
    {
        if (_idRestr >= _nbRestr)
        {
            throw new DomainErrorException();
        }
        return _restr[_idRestr]!;
    }

    public virtual void Next()
    {
        _idRestr++;
    }

    /// <summary>Starts the iteration over the vertices of the restriction given to <c>Initialize(Adaptor2d_Curve2d)</c>.</summary>
    public virtual void InitVertexIterator()
    {
        _idVtx = 0;
    }

    public virtual bool MoreVertex() => (_idVtx < _nbVtx);

    /// <summary>The current vertex.</summary>
    /// <exception cref="DomainErrorException">The iteration is over.</exception>
    public virtual HVertex Vertex()
    {
        if (_idVtx >= _nbVtx)
        {
            throw new DomainErrorException();
        }
        return _vtx[_idVtx]!;
    }

    public virtual void NextVertex()
    {
        _idVtx++;
    }

    /// <summary>Classifies a point of the parametric domain: IN, ON (within Tol of a finite bound) or OUT.</summary>
    /// <remarks>OCCT ignores <paramref name="reacdreOnPeriodic"/>.</remarks>
    public virtual Containment Classify(in Point2D p, double tol, bool reacdreOnPeriodic = true)
    {
        double u = p.X();
        double v = p.Y();

        if (_nbRestr == 4)
        {
            if ((u < _uinf - tol) || (u > _usup + tol) || (v < _vinf - tol) || (v > _vsup + tol))
            {
                return Out;
            }
            if ((Math.Abs(u - _uinf) <= tol) || (Math.Abs(u - _usup) <= tol) || (Math.Abs(v - _vinf) <= tol)
                || (Math.Abs(v - _vsup) <= tol))
            {
                return On;
            }
            return In;
        }
        else if (_nbRestr == 0)
        {
            return In;
        }
        else
        {
            bool dansu, dansv, surumin, surumax, survmin, survmax;
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_uinf) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_usup))
            {
                dansu = true;
                surumin = surumax = false;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_uinf))
            {
                surumin = false;
                // OCCT: `>=` here where the V branch below has `>` (docs/occt-issues.md).
                if (u >= _usup + tol)
                {
                    dansu = false;
                    surumax = false;
                }
                else
                {
                    dansu = true;
                    surumax = false;
                    if (Math.Abs(u - _usup) <= tol)
                    {
                        surumax = true;
                    }
                }
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_usup))
            {
                surumax = false;
                if (u < _uinf - tol)
                {
                    dansu = false;
                    surumin = false;
                }
                else
                {
                    dansu = true;
                    surumin = false;
                    if (Math.Abs(u - _uinf) <= tol)
                    {
                        surumin = true;
                    }
                }
            }
            else
            {
                if ((u < _uinf - tol) || (u > _usup + tol))
                {
                    surumin = surumax = dansu = false;
                }
                else
                {
                    dansu = true;
                    surumin = surumax = false;
                    if (Math.Abs(u - _uinf) <= tol)
                    {
                        surumin = true;
                    }
                    else if (Math.Abs(u - _usup) <= tol)
                    {
                        surumax = true;
                    }
                }
            }

            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_vinf) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_vsup))
            {
                dansv = true;
                survmin = survmax = false;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_vinf))
            {
                survmin = false;
                if (v > _vsup + tol)
                {
                    dansv = false;
                    survmax = false;
                }
                else
                {
                    dansv = true;
                    survmax = false;
                    if (Math.Abs(v - _vsup) <= tol)
                    {
                        survmax = true;
                    }
                }
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_vsup))
            {
                survmax = false;
                if (v < _vinf - tol)
                {
                    dansv = false;
                    survmin = false;
                }
                else
                {
                    dansv = true;
                    survmin = false;
                    if (Math.Abs(v - _vinf) <= tol)
                    {
                        survmin = true;
                    }
                }
            }
            else
            {
                if ((v < _vinf - tol) || (v > _vsup + tol))
                {
                    survmin = survmax = dansv = false;
                }
                else
                {
                    dansv = true;
                    survmin = survmax = false;
                    if (Math.Abs(v - _vinf) <= tol)
                    {
                        survmin = true;
                    }
                    else if (Math.Abs(v - _vsup) <= tol)
                    {
                        survmax = true;
                    }
                }
            }

            if (!dansu || !dansv)
            {
                return Out;
            }
            if (surumin || survmin || surumax || survmax)
            {
                return On;
            }
            return In;
        }
    }

    /// <summary>True if the point is on a finite bound of the domain (within Tol).</summary>
    /// <remarks>OCCT ignores <paramref name="reacdreOnPeriodic"/>.</remarks>
    public virtual bool IsThePointOn(in Point2D p, double tol, bool reacdreOnPeriodic = true)
    {
        double u = p.X();
        double v = p.Y();

        if (_nbRestr == 4)
        {
            if ((u < _uinf - tol) || (u > _usup + tol) || (v < _vinf - tol) || (v > _vsup + tol))
            {
                return (false);
            }
            if ((Math.Abs(u - _uinf) <= tol) || (Math.Abs(u - _usup) <= tol) || (Math.Abs(v - _vinf) <= tol)
                || (Math.Abs(v - _vsup) <= tol))
            {
                return (true);
            }
            return (false);
        }
        else if (_nbRestr == 0)
        {
            return (false);
        }
        else
        {
            bool dansu, dansv, surumin, surumax, survmin, survmax;
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_uinf) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_usup))
            {
                dansu = true;
                surumin = surumax = false;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_uinf))
            {
                surumin = false;
                // OCCT: `>=` here where the V branch below has `>` (docs/occt-issues.md).
                if (u >= _usup + tol)
                {
                    dansu = false;
                    surumax = false;
                }
                else
                {
                    dansu = true;
                    surumax = false;
                    if (Math.Abs(u - _usup) <= tol)
                    {
                        surumax = true;
                    }
                }
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_usup))
            {
                surumax = false;
                if (u < _uinf - tol)
                {
                    dansu = false;
                    surumin = false;
                }
                else
                {
                    dansu = true;
                    surumin = false;
                    if (Math.Abs(u - _uinf) <= tol)
                    {
                        surumin = true;
                    }
                }
            }
            else
            {
                if ((u < _uinf - tol) || (u > _usup + tol))
                {
                    surumin = surumax = dansu = false;
                }
                else
                {
                    dansu = true;
                    surumin = surumax = false;
                    if (Math.Abs(u - _uinf) <= tol)
                    {
                        surumin = true;
                    }
                    else if (Math.Abs(u - _usup) <= tol)
                    {
                        surumax = true;
                    }
                }
            }

            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_vinf) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_vsup))
            {
                dansv = true;
                survmin = survmax = false;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_vinf))
            {
                survmin = false;
                if (v > _vsup + tol)
                {
                    dansv = false;
                    survmax = false;
                }
                else
                {
                    dansv = true;
                    survmax = false;
                    if (Math.Abs(v - _vsup) <= tol)
                    {
                        survmax = true;
                    }
                }
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_vsup))
            {
                survmax = false;
                if (v < _vinf - tol)
                {
                    dansv = false;
                    survmin = false;
                }
                else
                {
                    dansv = true;
                    survmin = false;
                    if (Math.Abs(v - _vinf) <= tol)
                    {
                        survmin = true;
                    }
                }
            }
            else
            {
                if ((v < _vinf - tol) || (v > _vsup + tol))
                {
                    survmin = survmax = dansv = false;
                }
                else
                {
                    dansv = true;
                    survmin = survmax = false;
                    if (Math.Abs(v - _vinf) <= tol)
                    {
                        survmin = true;
                    }
                    else if (Math.Abs(v - _vsup) <= tol)
                    {
                        survmax = true;
                    }
                }
            }

            if (!dansu || !dansv)
            {
                return (false);
            }
            if (surumin || survmin || surumax || survmax)
            {
                return (true);
            }
            return false;
        }
    }

    /// <summary>
    /// The orientation of an arc: FORWARD or REVERSED for a "real" limit of the surface, INTERNAL or EXTERNAL for
    /// an arc on the surface. Always FORWARD here.
    /// </summary>
    public virtual Orientation Orientation(Curve2DAdaptorBase c) => Forward;

    /// <summary>The orientation of the vertex on the arc where it was found.</summary>
    public virtual Orientation Orientation(HVertex v) => v.Orientation();

    /// <summary>True if the vertices are the same, whatever their orientations.</summary>
    public virtual bool Identical(HVertex v1, HVertex v2) => v1.IsSame(v2);

    /// <summary>True if arcs and vertices have 3D representations (Tol3d and Pnt can be used). False here.</summary>
    public virtual bool Has3d() => false;

    /// <exception cref="DomainErrorException">Always: no 3D representation.</exception>
    public virtual double Tol3d(Curve2DAdaptorBase c) =>
        throw new DomainErrorException("Adaptor3d_TopolTool: has no 3d representation");

    /// <exception cref="DomainErrorException">Always: no 3D representation.</exception>
    public virtual double Tol3d(HVertex v) =>
        throw new DomainErrorException("Adaptor3d_TopolTool: has no 3d representation");

    /// <exception cref="DomainErrorException">Always: no 3D representation.</exception>
    public virtual Point3D Pnt(HVertex v) =>
        throw new DomainErrorException("Adaptor3d_TopolTool: has no 3d representation");

    //-- ============================================================
    //-- methods used for the samples
    //-- ============================================================

    private static void Analyse(NCollection_Array2<Point3D> array2, int nbup, int nbvp, ref int myNbSamplesU,
                                ref int myNbSamplesV)
    {
        Vector3D vi = new Vector3D(), vip1 = new Vector3D();
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
                vi.SetCoord(c.X() - b.X() - b.X() + a.X(),
                            c.Y() - b.Y() - b.Y() + a.Y(),
                            c.Z() - b.Z() - b.Z() + a.Z());
                int locnbch = 0;
                for (j = 3; j < nbvp; j++)
                { //-- try
                    Point3D a1 = array2.Value(i, j - 1);
                    Point3D b1 = array2.Value(i, j);
                    Point3D c1 = array2.Value(i, j + 1);
                    vip1.SetCoord(c1.X() - b1.X() - b1.X() + a1.X(),
                                  c1.Y() - b1.Y() - b1.Y() + a1.Y(),
                                  c1.Z() - b1.Z() - b1.Z() + a1.Z());
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
                vi.SetCoord(c.X() - b.X() - b.X() + a.X(),
                            c.Y() - b.Y() - b.Y() + a.Y(),
                            c.Z() - b.Z() - b.Z() + a.Z());
                int locnbch = 0;
                for (i = 3; i < nbup; i++)
                { //-- try
                    Point3D a1 = array2.Value(i - 1, j);
                    Point3D b1 = array2.Value(i, j);
                    Point3D c1 = array2.Value(i + 1, j);
                    vip1.SetCoord(c1.X() - b1.X() - b1.X() + a1.X(),
                                  c1.Y() - b1.Y() - b1.Y() + a1.Y(),
                                  c1.Z() - b1.Z() - b1.Z() + a1.Z());
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

    /// <summary>Computes the numbers of sample points of the intersection algorithms (NbSamplesU and NbSamplesV).</summary>
    public virtual void ComputeSamplePoints()
    {
        const int maxNbSample = 50;

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
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
            {
                nbsv = 15;
                nbsu = 15;
            }
                break;
            default:
            {
                nbsu = 10;
                nbsv = 10;
            }
                break;
        }

        //-- If the number of points is too great... analyze

        if (nbsu < 6)
        {
            nbsu = 6;
        }
        if (nbsv < 6)
        {
            nbsv = 6;
        }

        if (typS == SurfaceKind.BSplineSurface)
        {
            if (nbsu > 8 || nbsv > 8)
            {
                global::Kerf.Geometry.BSplineSurface bspl = _s.BSpline();
                NCollection_Array2<Point3D> array2 = bspl.InternalPoles();
                int nbup = array2.NbRows();
                int nbvp = array2.NbColumns();
                Analyse(array2, nbup, nbvp, ref nbsu, ref nbsv);
            }
            // Check anisotropy
            double uLen = (usup - uinf) / _s.UResolution(1.0);
            double vLen = (vsup - vinf) / _s.VResolution(1.0);
            double ratio = uLen / vLen;
            if (ratio >= 10.0)
            {
                nbsu *= 2;
                nbsu = Min(nbsu, maxNbSample);
            }
            else if (ratio <= 0.1)
            {
                nbsv *= 2;
                nbsv = Min(nbsv, maxNbSample);
            }
        }
        else if (typS == SurfaceKind.BezierSurface)
        {
            if (nbsu > 8 || nbsv > 8)
            {
                global::Kerf.Geometry.BezierSurface bez = _s.Bezier();
                NCollection_Array2<Point3D> array2 = bez.InternalPoles();
                int nbup = array2.NbRows();
                int nbvp = array2.NbColumns();
                Analyse(array2, nbup, nbvp, ref nbsu, ref nbsv);
            }
        }

        _nbSamplesU = nbsu;
        _nbSamplesV = nbsv;
    }

    public virtual int NbSamplesU()
    {
        if (_nbSamplesU < 0)
        {
            ComputeSamplePoints();
        }
        return (_nbSamplesU);
    }

    public virtual int NbSamplesV()
    {
        if (_nbSamplesU < 0)
        {
            ComputeSamplePoints();
        }
        return (_nbSamplesV);
    }

    public virtual int NbSamples()
    {
        if (_nbSamplesU < 0)
        {
            ComputeSamplePoints();
        }
        return (_nbSamplesU * _nbSamplesV);
    }

    /// <summary>Copies the U parameters computed by <see cref="SamplePnts"/> into <paramref name="array"/> (OCCT's array assignment).</summary>
    public void UParameters(NCollection_Array1<double> array)
    {
        array.Assign(_uPars!.Array1());
    }

    /// <summary>Copies the V parameters computed by <see cref="SamplePnts"/> into <paramref name="array"/> (OCCT's array assignment).</summary>
    public void VParameters(NCollection_Array1<double> array)
    {
        array.Assign(_vPars!.Array1());
    }

    /// <summary>The sample point of index <paramref name="index"/>, in the parametric domain and in 3D.</summary>
    public virtual void SamplePoint(int index, out Point2D p2d, out Point3D p3d)
    {
        int i = index;
        int iu, iv;
        double u, v;
        if (_uPars is null)
        {
            double myDU = (_usup - _uinf) / (_nbSamplesU + 1);
            double myDV = (_vsup - _vinf) / (_nbSamplesV + 1);
            iv = 1 + i / _nbSamplesU;
            iu = 1 + i - (iv - 1) * _nbSamplesU;
            u = _uinf + iu * myDU;
            v = _vinf + iv * myDV;
        }
        else
        {
            iv = (i - 1) / _nbSamplesU + 1;
            iu = (i - 1) % _nbSamplesU + 1;
            u = _uPars.Value(iu);
            v = _vPars!.Value(iv);
        }

        p2d = new Point2D(u, v);
        p3d = _s!.Value(u, v);
    }

    /// <summary>True if one of the bounds of the domain is infinite.</summary>
    public virtual bool DomainIsInfinite()
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_uinf))
        {
            return (true);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_usup))
        {
            return (true);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_vinf))
        {
            return (true);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_vsup))
        {
            return (true);
        }
        return (false);
    }

    /// <summary>The topological edge of a derived tool (OCCT's <c>void*</c>); null here.</summary>
    public virtual object? Edge() => null;

    /// <summary>
    /// Computes the sample parameters of the intersection algorithms: adaptively (<see cref="BSplSamplePnts"/>)
    /// for a B-spline surface; otherwise the numbers of <see cref="ComputeSamplePoints"/>, uniformly spaced.
    /// </summary>
    public virtual void SamplePnts(double defl, int nUmin, int nVmin)
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

        SurfaceKind typS = _s.GetType();
        if (typS == SurfaceKind.BSplineSurface)
        {
            // Processing BSpline surface
            BSplSamplePnts(defl, nUmin, nVmin);
            return;
        }
        else
        {
            ComputeSamplePoints();
        }

        _uPars = new NCollection_HArray1<double>(1, _nbSamplesU);
        _vPars = new NCollection_HArray1<double>(1, _nbSamplesV);
        int i;
        double t, dt = (usup - uinf) / (_nbSamplesU - 1);
        _uPars.SetValue(1, uinf);
        _uPars.SetValue(_nbSamplesU, usup);
        for (i = 2, t = uinf + dt; i < _nbSamplesU; ++i, t += dt)
        {
            _uPars.SetValue(i, t);
        }

        dt = (vsup - vinf) / (_nbSamplesV - 1);
        _vPars.SetValue(1, vinf);
        _vPars.SetValue(_nbSamplesV, vsup);
        for (i = 2, t = vinf + dt; i < _nbSamplesV; ++i, t += dt)
        {
            _vPars.SetValue(i, t);
        }
    }

    /// <summary>
    /// Computes the sample parameters of a B-spline surface adaptively: the knots and degree subdivisions, thinned
    /// where the iso-curves stay within <paramref name="defl"/> of a chord.
    /// </summary>
    public virtual void BSplSamplePnts(double defl, int nUmin, int nVmin)
    {
        const int maxPnts = 1001;
        global::Kerf.Geometry.BSplineSurface bs = _s!.BSpline();
        double uinf, usup, vinf, vsup;
        uinf = _s.FirstUParameter();
        usup = _s.LastUParameter();
        vinf = _s.FirstVParameter();
        vsup = _s.LastVParameter();

        int i, k, j = 1;
        double t1, t2, dt;
        int ui1 = bs.FirstUKnotIndex();
        int ui2 = bs.LastUKnotIndex();
        int vi1 = bs.FirstVKnotIndex();
        int vi2 = bs.LastVKnotIndex();

        for (i = ui1; i < ui2; ++i)
        {
            if (uinf >= bs.UKnot(i) && uinf < bs.UKnot(i + 1))
            {
                ui1 = i;
                break;
            }
        }

        for (i = ui2; i > ui1; --i)
        {
            if (usup <= bs.UKnot(i) && usup > bs.UKnot(i - 1))
            {
                ui2 = i;
                break;
            }
        }

        for (i = vi1; i < vi2; ++i)
        {
            if (vinf >= bs.VKnot(i) && vinf < bs.VKnot(i + 1))
            {
                vi1 = i;
                break;
            }
        }

        for (i = vi2; i > vi1; --i)
        {
            if (vsup <= bs.VKnot(i) && vsup > bs.VKnot(i - 1))
            {
                vi2 = i;
                break;
            }
        }

        int nbsu = ui2 - ui1 + 1;
        nbsu += (nbsu - 1) * (bs.UDegree() - 1);
        int nbsv = vi2 - vi1 + 1;
        nbsv += (nbsv - 1) * (bs.VDegree() - 1);
        bool bUuniform = false;
        bool bVuniform = false;

        // modified by NIZHNY-EMV Mon Jun 10 14:19:04 2013
        if (nbsu < nUmin || nbsv < nVmin)
        {
            int nb;
            if (nbsu < nbsv)
            {
                nb = (int)(nbsv * ((double)nUmin) / ((double)nbsu));
                nb = Min(nb, 30);
                bVuniform = (nb > nbsv) ? true : bVuniform;
                nbsv = bVuniform ? nb : nbsv;
            }
            else
            {
                nb = (int)(nbsu * ((double)nVmin) / ((double)nbsv));
                nb = Min(nb, 30);
                bUuniform = (nb > nbsu) ? true : bUuniform;
                nbsu = bUuniform ? nb : nbsu;
            }
        }
        // modified by NIZHNY-EMV Mon Jun 10 14:19:05 2013

        if (nbsu < nUmin)
        {
            nbsu = nUmin;
            bUuniform = true;
        }
        else if (nbsu > maxPnts)
        {
            nbsu = maxPnts;
            bUuniform = true;
        }
        if (nbsv < nVmin)
        {
            nbsv = nVmin;
            bVuniform = true;
        }
        else if (nbsv > maxPnts)
        {
            nbsv = maxPnts;
            bVuniform = true;
        }

        NCollection_Array1<double> uPars = new(1, nbsu);
        NCollection_Array1<bool> uFlg = new(1, nbsu);
        NCollection_Array1<double> vPars = new(1, nbsv);
        NCollection_Array1<bool> vFlg = new(1, nbsv);

        // Filling of sample parameters
        if (bUuniform)
        {
            t1 = uinf;
            t2 = usup;
            dt = (t2 - t1) / (nbsu - 1);
            uPars[1] = t1;
            uFlg[1] = false;
            uPars[nbsu] = t2;
            uFlg[nbsu] = false;
            for (i = 2, t1 += dt; i < nbsu; ++i, t1 += dt)
            {
                uPars[i] = t1;
                uFlg[i] = false;
            }
        }
        else
        {
            int nbi = bs.UDegree();
            k = 0;
            t1 = uinf;
            for (i = ui1 + 1; i <= ui2; ++i)
            {
                if (i == ui2)
                {
                    t2 = usup;
                }
                else
                {
                    t2 = bs.UKnot(i);
                }
                dt = (t2 - t1) / nbi;
                j = 1;
                do
                {
                    ++k;
                    uPars[k] = t1;
                    uFlg[k] = false;
                    t1 += dt;
                } while (++j <= nbi);
                t1 = t2;
            }
            ++k;
            uPars[k] = t1;
        }

        if (bVuniform)
        {
            t1 = vinf;
            t2 = vsup;
            dt = (t2 - t1) / (nbsv - 1);
            vPars[1] = t1;
            vFlg[1] = false;
            vPars[nbsv] = t2;
            vFlg[nbsv] = false;
            for (i = 2, t1 += dt; i < nbsv; ++i, t1 += dt)
            {
                vPars[i] = t1;
                vFlg[i] = false;
            }
        }
        else
        {
            int nbi = bs.VDegree();
            k = 0;
            t1 = vinf;
            for (i = vi1 + 1; i <= vi2; ++i)
            {
                if (i == vi2)
                {
                    t2 = vsup;
                }
                else
                {
                    t2 = bs.VKnot(i);
                }
                dt = (t2 - t1) / nbi;
                j = 1;
                do
                {
                    ++k;
                    vPars[k] = t1;
                    vFlg[k] = false;
                    t1 += dt;
                } while (++j <= nbi);
                t1 = t2;
            }
            ++k;
            vPars[k] = t1;
        }

        // Analysis of deflection

        double defl2 = Max(defl * defl, 1.0e-9);
        double tol = Max(0.01 * defl2, 1.0e-9);
        int l;

        uFlg[1] = true;
        uFlg[nbsu] = true;
        for (i = 1; i <= nbsv; ++i)
        {
            t1 = vPars[i];
            j = 1;
            bool bCont = true;
            while (j < nbsu - 1 && bCont)
            {
                if (uFlg[j + 1])
                {
                    ++j;
                    continue;
                }

                t2 = uPars[j];
                Point3D p1 = _s.Value(t2, t1);
                for (k = j + 2; k <= nbsu; ++k)
                {
                    t2 = uPars[k];
                    Point3D p2 = _s.Value(t2, t1);

                    if (p1.SquareDistance(p2) <= tol)
                    {
                        continue;
                    }

                    Line3D lin = new(p1, new Direction3D(new Vector3D(p1, p2)));
                    bool ok = true;
                    for (l = j + 1; l < k; ++l)
                    {
                        if (uFlg[l])
                        {
                            ok = false;
                            break;
                        }

                        Point3D pp = _s.Value(uPars[l], t1);
                        double d = lin.SquareDistance(pp);

                        if (d <= defl2)
                        {
                            continue;
                        }

                        ok = false;
                        break;
                    }

                    if (!ok)
                    {
                        j = k - 1;
                        uFlg[j] = true;
                        break;
                    }

                    if (uFlg[k])
                    {
                        j = k;
                        break;
                    }
                }

                if (k >= nbsu)
                {
                    bCont = false;
                }
            }
        }

        _nbSamplesU = 0;
        for (i = 1; i <= nbsu; i++)
        {
            if (uFlg[i])
            {
                _nbSamplesU++;
            }
        }

        if (_nbSamplesU < MinPnts)
        {
            if (_nbSamplesU == 2)
            {
                //"uniform" distribution;
                int nn = nbsu / MinPnts;
                uFlg[1 + nn] = true;
                uFlg[nbsu - nn] = true;
                // OCCT: with nbsu < 4, nn is 0 and no flag is added, but the count below is still set to 4, so the
                // last entries of myUPars stay 0 (docs/occt-issues.md). j is the deflection loop's (1 or nbsu here).
            }
            else
            { // myNbSamplesU == 3
                // insert in bigger segment
                i = 2;
                while (!uFlg[i++])
                {
                    ;
                }
                if (i < nbsu / 2)
                {
                    j = Min(i + (nbsu - i) / 2, nbsu - 1);
                }
                else
                {
                    j = Max(i / 2, 2);
                }
            }
            uFlg[j] = true;
            _nbSamplesU = MinPnts;
        }

        vFlg[1] = true;
        vFlg[nbsv] = true;
        for (i = 1; i <= nbsu; ++i)
        {
            t1 = uPars[i];
            j = 1;
            bool bCont = true;
            while (j < nbsv - 1 && bCont)
            {
                if (vFlg[j + 1])
                {
                    ++j;
                    continue;
                }

                t2 = vPars[j];
                Point3D p1 = _s.Value(t1, t2);
                for (k = j + 2; k <= nbsv; ++k)
                {
                    t2 = vPars[k];
                    Point3D p2 = _s.Value(t1, t2);

                    if (p1.SquareDistance(p2) <= tol)
                    {
                        continue;
                    }
                    Line3D lin = new(p1, new Direction3D(new Vector3D(p1, p2)));
                    bool ok = true;
                    for (l = j + 1; l < k; ++l)
                    {
                        if (vFlg[l])
                        {
                            ok = false;
                            break;
                        }

                        Point3D pp = _s.Value(t1, vPars[l]);
                        double d = lin.SquareDistance(pp);

                        if (d <= defl2)
                        {
                            continue;
                        }

                        ok = false;
                        break;
                    }

                    if (!ok)
                    {
                        j = k - 1;
                        vFlg[j] = true;
                        break;
                    }

                    if (vFlg[k])
                    {
                        j = k;
                        break;
                    }
                }

                if (k >= nbsv)
                {
                    bCont = false;
                }
            }
        }

        _nbSamplesV = 0;
        for (i = 1; i <= nbsv; i++)
        {
            if (vFlg[i])
            {
                _nbSamplesV++;
            }
        }

        if (_nbSamplesV < MinPnts)
        {
            if (_nbSamplesV == 2)
            {
                //"uniform" distribution;
                int nn = nbsv / MinPnts;
                vFlg[1 + nn] = true;
                vFlg[nbsv - nn] = true;
                _nbSamplesV = MinPnts;
                // OCCT: as for U, with nbsv < 4 the count becomes 4 without new flags (docs/occt-issues.md).
            }
            else
            { // myNbSamplesU == 3
                // insert in bigger segment
                i = 2;
                while (!vFlg[i++])
                {
                    ;
                }
                if (i < nbsv / 2)
                {
                    j = Min(i + (nbsv - i) / 2, nbsv - 1);
                }
                else
                {
                    j = Max(i / 2, 2);
                }
            }
            _nbSamplesV = MinPnts;
            vFlg[j] = true;
        }

        // modified by NIZNHY-PKV Fri Dec 16 10:05:01 2011f
        bool bFlag;

        // U
        bFlag = (_nbSamplesU < nUmin);
        if (bFlag)
        {
            _nbSamplesU = nbsu;
        }

        _uPars = new NCollection_HArray1<double>(1, _nbSamplesU);

        for (j = 0, i = 1; i <= nbsu; ++i)
        {
            if (bFlag)
            {
                _uPars.SetValue(i, uPars[i]);
            }
            else
            {
                if (uFlg[i])
                {
                    ++j;
                    _uPars.SetValue(j, uPars[i]);
                }
            }
        }

        // V
        bFlag = (_nbSamplesV < nVmin);
        if (bFlag)
        {
            _nbSamplesV = nbsv;
        }

        _vPars = new NCollection_HArray1<double>(1, _nbSamplesV);

        for (j = 0, i = 1; i <= nbsv; ++i)
        {
            if (bFlag)
            {
                _vPars.SetValue(i, vPars[i]);
            }
            else
            {
                if (vFlg[i])
                {
                    ++j;
                    _vPars.SetValue(j, vPars[i]);
                }
            }
        }
        // modified by NIZNHY-PKV Mon Dec 26 12:25:35 2011t
    }

    /// <summary>True if the sampling is uniform (any surface but a B-spline).</summary>
    public virtual bool IsUniformSampling()
    {
        SurfaceKind typS = _s!.GetType();

        return typS != SurfaceKind.BSplineSurface;
    }

    /// <summary>Computes the parameters of the apex of a cone.</summary>
    public static void GetConeApexParam(in Cone c, out double u, out double v)
    {
        CoordinateSystem pos = c.Position();
        double radius = c.RefRadius();
        double sAngle = c.SemiAngle();
        Point3D p = c.Apex();

        Transform3D t = new();
        t.SetTransformation(pos);
        Point3D ploc = p.Transformed(t);

        if (ploc.X() == 0.0 && ploc.Y() == 0.0)
        {
            u = 0.0;
        }
        else if (-radius > ploc.Z() * Math.Tan(sAngle))
        {
            // the point is at the `wrong` side of the apex
            u = Math.Atan2(-ploc.Y(), -ploc.X());
        }
        else
        {
            u = Math.Atan2(ploc.Y(), ploc.X());
        }

        if (u < -1.0e-16)
        {
            u += (M_PI + M_PI);
        }
        else if (u < 0)
        {
            u = 0;
        }

        v =
            Math.Sin(sAngle) * (ploc.X() * Math.Cos(u) + ploc.Y() * Math.Sin(u) - radius) + Math.Cos(sAngle) * ploc.Z();
    }
}
