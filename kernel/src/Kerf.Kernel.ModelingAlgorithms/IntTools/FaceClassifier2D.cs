// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FClass2d.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FClass2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification;

/// <summary>
/// The 2D point classifier of a face used by the Boolean component: each wire is sampled once into a polygon
/// (<see cref="Class2d"/>), and points the polygons can't decide go to <see cref="FClassifier"/> over a
/// face explorer made on the first such point.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21), cached per face by <see cref="IntersectionContext.FClass2d"/>. Its queries keep no
/// result-changing state: the only thing they change is the lazily made face explorer, whose probing restarts at
/// every classification (decision 723).
/// </para>
/// <para>
/// OCCT's DEBUG_PCLASS_POLYGON block (DrawTrSurf) is left out.
/// </para>
/// </remarks>
internal sealed class FaceClassifier2D
{
    private readonly NCollection_Sequence<Class2d> _tabClass = new();
    private readonly NCollection_Sequence<int> _tabOrien = new();
    private double _toluv;
    private Face _face;
    // The period of the surface, set in U1..U2, V1..V2 and never read (as decision 221's fields)
#pragma warning disable CS0414
#pragma warning restore CS0414
    private double _umin;
    private double _umax;
    private double _vmin;
    private double _vmax;
    private bool _isHole;

    // OCCT's mutable std::unique_ptr<BRepClass_FaceExplorer>, made by the first query the polygons can't decide.
    private FaceExplorer? _fExplorer;

    /// <summary>An empty classifier (every point is IN). OCCT leaves its numbers uninitialised; the port's are 0.</summary>
    public FaceClassifier2D()
    {
        _face = new Face();
    }

    /// <summary>The classifier of the face <paramref name="aFace"/> with the tolerance <paramref name="tolUV"/> in its parameter space.</summary>
    public FaceClassifier2D(in Face aFace, double tolUV)
    {
        _toluv = tolUV;
        _face = aFace;
        Init(_face, _toluv);
    }

    /// <summary>True if the last wire sampled with a non-zero area is clockwise (a hole), or no wire was.</summary>
    public bool IsHole => _isHole;

    /// <summary>Initializes the classifier with the face and the tolerance in its parameter space.</summary>
    /// <remarks>
    /// As in OCCT, a second <c>Init</c> appends the new face's polygons to the old ones and keeps the face explorer made
    /// for the old face (occt-issues.md); IntTools_Context never re-initializes a classifier.
    /// </remarks>
    public void Init(in Face aFace, double tolUV)
    {
        bool wireIsNotEmpty, ancienpnt3dinitialise, degenerated;
        int firstpoint, nbEdges;
        int iX, nbs1, nbs, avant, badWire;
        double u, du, tole, tol, pfbid, plbid;
        double flecheU, flecheV, tolVertex1, tolVertex;
        double uFirst, uLast;
        double prCf, prCf2;
        //
        Edge edge;
        Orientation or;
        WireExplorer wExp = new();
        Explorer expF = new(), exp = new();
        Curve2D? c2D;
        Point3D ancienpnt3d = new();
        NCollection_Sequence<Point2D> seqPnt2d = Class2dPoints.Rent();
        Dictionary<int, int> indexMap = new();
        NCollection_Sequence<Vector2D> d1Prev = new();
        NCollection_Sequence<Vector2D> d1Next = new();
        //
        prCf = global::Kerf.Engine.Foundation.Precision.Confusion();
        prCf2 = prCf * prCf;
        _isHole = true;
        //
        _toluv = tolUV;
        _face = aFace;
        _face.Orient(Forward);
        FaceSurfaceAdaptor surf = new();
        surf.Initialize(aFace, false);
        //
        tole = 0.0;
        tol = 0.0;
        _umin = _vmin = RealLast();
        _umax = _vmax = -_umin;
        badWire = 0;
        //
        // if face has several wires and one of them is bad,
        // it is necessary to process all of them for correct
        // calculation of Umin, Umax, Vmin, Vmax - ifv, 23.08.06
        //
        expF.Init(_face, ShapeType.Wire);
        for (; expF.More(); expF.Next())
        {
            Wire w = ShapeCasts.Wire(expF.Current());
            //
            firstpoint = 1;
            flecheU = 0.0;
            flecheV = 0.0;
            tolVertex1 = 0.0;
            tolVertex = 0.0;
            wireIsNotEmpty = false;
            ancienpnt3dinitialise = false;
            ancienpnt3d = ancienpnt3d.WithCoord(0.0, 0.0, 0.0);
            //
            seqPnt2d.Clear();
            indexMap.Clear();
            d1Prev.Clear();
            d1Next.Clear();
            //
            // NbEdges
            nbEdges = 0;
            exp.Init(w, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                nbEdges++;
            }
            //
            wExp.Init(w, _face);
            for (; wExp.More(); wExp.Next())
            {
                nbEdges--;
                edge = wExp.Current();
                or = edge.Orientation;
                if (or != Forward && or != Reversed)
                {
                    continue;
                }
                //
                c2D = Tool.CurveOnSurface(edge, _face, out pfbid, out plbid);
                if (c2D is null)
                {
                    // OCCT returns here, leaving the classifier half built: the wires sampled so far, Umin..Vmax as far
                    // as they got, U1..V2 unset (occt-issues.md)
                    return;
                }
                //
                EdgeCurve2DAdaptor c = new(edge, _face);
                EdgeCurveAdaptor c3d = new();
                //------------------------------------------
                degenerated = false;
                if (Tool.Degenerated(edge) || Tool.IsClosed(edge, _face))
                {
                    degenerated = true;
                }
                //
                Exploration.Vertices(edge, out Vertex va, out Vertex vb);
                //
                tolVertex1 = 0.0;
                tolVertex = 0.0;
                if (va.IsNull)
                {
                    degenerated = true;
                }
                else
                {
                    tolVertex1 = Tool.Tolerance(va);
                }
                if (vb.IsNull)
                {
                    degenerated = true;
                }
                else
                {
                    tolVertex = Tool.Tolerance(vb);
                }
                //
                if (tolVertex < tolVertex1)
                {
                    tolVertex = tolVertex1;
                }
                //
                //-- Verification of cases when forgotten to code degenereted
                if (!degenerated)
                {
                    // check that whole curve is located in vicinity of its middle point
                    // (within sphere of Precision::Confusion() diameter)
                    c3d.Initialize(edge, _face);
                    Point3D p3da = c3d.Value(0.5 * (pfbid + plbid));
                    du = plbid - pfbid;
                    const int nbsteps = 10;
                    double prec2 = 0.25 * global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion();
                    degenerated = true;
                    for (int i = 0; i <= nbsteps; i++)
                    {
                        double U = pfbid + i * du / nbsteps;
                        Point3D p3db = c3d.Value(U);
                        double r2 = p3da.SquareDistanceTo(p3db);
                        if (r2 > prec2)
                        {
                            degenerated = false;
                            break;
                        }
                    }
                } // if(!degenerated)
                //-- ----------------------------------------
                tole = Tool.Tolerance(edge);
                if (tole > tol)
                {
                    tol = tole;
                }
                //
                // NbSamples +> nbs
                nbs = Geom2dCurveTool.NbSamples(c);
                if (nbs > 2)
                {
                    nbs *= 4;
                }
                du = (plbid - pfbid) / (double)(nbs - 1);
                //
                if (or == Forward)
                {
                    u = pfbid;
                    uFirst = pfbid;
                    uLast = plbid;
                }
                else
                {
                    u = plbid;
                    uFirst = plbid;
                    uLast = pfbid;
                    du = -du;
                }
                //
                // aPrms
                nbs1 = nbs + 1;
                NCollection_Array1<double> prms = new(1, nbs1);
                //
                if (nbs == 2)
                {
                    double coef = 0.0025;
                    prms[1] = uFirst;
                    prms[2] = uFirst + coef * (uLast - uFirst);
                    prms[3] = uLast;
                }
                else if (nbs > 2)
                {
                    nbs1 = nbs;
                    prms[1] = uFirst;
                    for (iX = 2; iX < nbs1; ++iX)
                    {
                        prms[iX] = u + (iX - 1) * du;
                    }
                    prms[nbs1] = uLast;
                }
                //
                //-- ------------------------------------------------------------
                //-- Check distance uv between the start point of the edge
                //-- and the last point saved in SeqPnt2d
                //-- To to set the first point of the current
                //-- afar from the last saved point
                avant = seqPnt2d.Length();
                for (iX = firstpoint; iX <= nbs1; iX++)
                {
                    bool isRealCurve3d;
                    int ii;
                    double dstX;
                    Point2D P2d;
                    Point3D p3d = new();
                    //
                    u = prms[iX];
                    P2d = c.Value(u);
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
                    //
                    dstX = RealLast();
                    if (!degenerated)
                    {
                        p3d = c3d.Value(u);
                        if (!seqPnt2d.IsEmpty())
                        {
                            if (ancienpnt3dinitialise)
                            {
                                dstX = p3d.SquareDistanceTo(ancienpnt3d);
                            }
                        }
                    }
                    //
                    isRealCurve3d = true;
                    if (dstX < prCf2)
                    {
                        if (iX > 1)
                        {
                            double dstX1;
                            Point3D midP3d;
                            //
                            midP3d = c3d.Value(0.5 * (u + prms[iX - 1]));
                            dstX1 = p3d.SquareDistanceTo(midP3d);
                            if (dstX1 < prCf2)
                            {
                                isRealCurve3d = false;
                            }
                        }
                    }
                    //
                    if (isRealCurve3d)
                    {
                        if (!degenerated)
                        {
                            ancienpnt3d = p3d;
                            ancienpnt3dinitialise = true;
                        }
                        seqPnt2d.Append(P2d);
                    }
                    //
                    ii = seqPnt2d.Length();
                    if (ii > (avant + 4))
                    {
                        double ul, dU, dV;
                        Point2D pp;
                        //
                        Line2D lin = new(seqPnt2d[ii - 2], new Direction2D(new Vector2D(seqPnt2d[ii - 2], seqPnt2d[ii])));
                        ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, seqPnt2d[ii - 1]);
                        pp = global::Kerf.Engine.Geometry.ElCLib.Value(ul, lin);
                        dU = Math.Abs(pp.X - seqPnt2d[ii - 1].X);
                        dV = Math.Abs(pp.Y - seqPnt2d[ii - 1].Y);
                        if (dU > flecheU)
                        {
                            flecheU = dU;
                        }
                        if (dV > flecheV)
                        {
                            flecheV = dV;
                        }
                    }
                } // for(iX=firstpoint; iX<=aNbs1; iX++) {
                //
                if (badWire != 0)
                {
                    continue; // if face has several wires and one of them is bad,
                              // it is necessary to process all of them for correct
                              // calculation of Umin, Umax, Vmin, Vmax - ifv, 23.08.06
                }
                //
                if (firstpoint == 1)
                {
                    firstpoint = 2;
                }
                wireIsNotEmpty = true;
                // Append the derivative of the first parameter.
                double aU = prms[1];
                c.D1(aU, out _, out Vector2D v);

                if (or == Reversed)
                {
                    v = v.Reversed();
                }

                d1Next.Append(v);

                // Append the derivative of the last parameter.
                aU = prms[nbs1];
                c.D1(aU, out _, out v);

                if (or == Reversed)
                {
                    v = v.Reversed();
                }

                if (nbEdges > 0)
                {
                    d1Prev.Append(v);
                }
                else
                {
                    d1Prev.Prepend(v);
                }

                // Fill the map anIndexMap.
                if (avant > 0)
                {
                    indexMap[avant] = d1Next.Length();
                }
                else
                {
                    indexMap[1] = d1Next.Length();
                }
            } // for(;aWExp.More(); aWExp.Next()) {
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            //
            if (nbEdges != 0)
            {
                //-- count ++ with normal explorer and -- with Wire Explorer
                NCollection_Array1<Point2D> pClass = new(1, 2);
                Point2D initPnt = new(0.0, 0.0);
                //
                pClass.Init(initPnt);
                _tabClass.Append(new Class2d(pClass, flecheU, flecheV, _umin, _vmin, _umax, _vmax));
                badWire = 1;
                _tabOrien.Append(-1);
            }
            //
            else if (wireIsNotEmpty)
            {
                if (seqPnt2d.Length() > 3)
                {
                    global::Kerf.Engine.Meshing.Poly.PolygonProperties(seqPnt2d, out double s, out double per);

                    double expThick = Max(2.0 * Math.Abs(s) / per, 1e-7);
                    double defl = Max(flecheU, flecheV);
                    double discrDefl = Min(defl * 0.1, expThick * 10.0);
                    bool isChanged = false;
                    while (defl > expThick && discrDefl > 1e-7)
                    {
                        // Deflection of the polygon is too much for this ratio of area and perimeter,
                        // and this might lead to self-intersections.
                        // Discretize the wire more tightly to eliminate the error.
                        firstpoint = 1;
                        isChanged = true;
                        seqPnt2d.Clear();
                        flecheU = 0.0;
                        flecheV = 0.0;
                        for (wExp.Init(ShapeCasts.Wire(expF.Current()), _face); wExp.More(); wExp.Next())
                        {
                            edge = wExp.Current();
                            or = edge.Orientation;
                            if (or == Forward || or == Reversed)
                            {
                                Tool.Range(edge, _face, out pfbid, out plbid);
                                if (Math.Abs(plbid - pfbid) < 1.0e-9)
                                {
                                    continue;
                                }
                                EdgeCurve2DAdaptor c = new(edge, _face);
                                QuasiUniformDeflection discr = new(c, discrDefl);
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
                                    Line2D lin = new(seqPnt2d[ii - 2], new Direction2D(new Vector2D(seqPnt2d[ii - 2], seqPnt2d[ii])));
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
                        // OCCT doesn't recompute the polygon's area and perimeter inside the loop, so anExpThick
                        // stays the first polygon's
                        expThick = Max(2.0 * Math.Abs(s) / per, 1e-7);
                        defl = Max(flecheU, flecheV);
                        discrDefl = Min(discrDefl * 0.1, expThick * 10.0);
                    }

                    if (isChanged)
                    {
                        global::Kerf.Engine.Meshing.Poly.PolygonProperties(seqPnt2d, out s, out per);
                    }
                    //
                    if (flecheU < _toluv)
                    {
                        flecheU = _toluv;
                    }

                    if (flecheV < _toluv)
                    {
                        flecheV = _toluv;
                    }

                    _tabClass.Append(new Class2d(seqPnt2d, flecheU, flecheV, _umin, _vmin, _umax, _vmax));
                    //
                    if (Math.Abs(s) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        badWire = 1;
                        _tabOrien.Append(-1);
                    }
                    else
                    {
                        if (s > 0.0)
                        {
                            _isHole = false;
                            _tabOrien.Append(1);
                        }
                        else
                        {
                            _isHole = true;
                            _tabOrien.Append(0);
                        }
                    }
                }
                else
                {
                    badWire = 1;
                    _tabOrien.Append(-1);
                    seqPnt2d.Clear();
                    _tabClass.Append(new Class2d(seqPnt2d, flecheU, flecheV, _umin, _vmin, _umax, _vmax));
                }
            } // else if(WireIsNotEmpty)
        } // for(; aExpF.More();  aExpF.Next()) {
        //
        int nbtabclass = _tabClass.Length();
        //
        if (nbtabclass > 0)
        {
            //-- if an error on a wire was detected : all TabOrien set to -1
            if (badWire != 0)
            {
                _tabOrien[1] = -1;
            }
        }
        Class2dPoints.Return(seqPnt2d);
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
    {
        int nbtabclass = _tabClass.Length();
        if (nbtabclass == 0)
        {
            return In;
        }

        //-- U1 is the First Param and U2 is in this case U1+Period
        double u = _Puv.X;
        double v = _Puv.Y;
        double uu = u;
        double vv = v;
        Containment status = Unknown;

        FaceSurfaceAdaptor surf = new();
        surf.Initialize(_face, false);

        bool isUPer = surf.IsUPeriodic;
        bool isVPer = surf.IsVPeriodic;
        double uperiod = isUPer ? surf.UPeriod() : 0.0;
        double vperiod = isVPer ? surf.VPeriod() : 0.0;

        bool urecadre, vrecadre, bUseClassifier;
        int dedans;
        //
        urecadre = false;
        vrecadre = false;
        //
        if (recadreOnPeriodic)
        {
            if (isUPer)
            {
                global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(uu, _umin, _umax, uperiod, out uu, out _);
            } // if (IsUPer) {
            //
            if (isVPer)
            {
                global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(vv, _vmin, _vmax, vperiod, out vv, out _);
            } // if (IsVPer) {
        }
        //
        for (;;)
        {
            dedans = 1;
            Point2D puv = new(u, v);
            bUseClassifier = (_tabOrien[1] == -1);
            if (!bUseClassifier)
            {
                int n, cur, TabOrien_n;
                for (n = 1; n <= nbtabclass; n++)
                {
                    cur = (int)_tabClass[n].SiDans(puv);
                    TabOrien_n = _tabOrien[n];

                    if (cur == 1)
                    {
                        if (TabOrien_n == 0)
                        {
                            dedans = -1;
                            break;
                        }
                    }
                    else if (cur == -1)
                    {
                        if (TabOrien_n == 1)
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
                } // for(n=1; n<=nbtabclass; n++)

                if (dedans == 0)
                {
                    bUseClassifier = true;
                }
                else
                {
                    status = (dedans == 1) ? In : Out;
                }
            } // if(TabOrien(1)!=-1) {
            // compute state of the point using face classifier
            if (bUseClassifier)
            {
                // compute tolerance to use in face classifier
                double uRes, vRes, fcTol;
                bool bUIn, bVIn;
                //
                uRes = surf.UResolution(_toluv);
                vRes = surf.VResolution(_toluv);
                //
                bUIn = (u >= _umin) && (u <= _umax);
                bVIn = (v >= _vmin) && (v <= _vmax);
                //
                if (bUIn == bVIn)
                {
                    fcTol = Min(uRes, vRes);
                }
                else
                {
                    fcTol = (!bUIn) ? uRes : vRes;
                }
                //

                _fExplorer ??= new FaceExplorer(_face);

                FClassifier classifier = new();
                classifier.Perform(_fExplorer, puv, fcTol);
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
            else
            {
                if (isUPer)
                {
                    u += uperiod;
                }
            }

            if (u > _umax || !isUPer)
            {
                if (!vrecadre)
                {
                    v = vv;
                    vrecadre = true;
                }
                else
                {
                    if (isVPer)
                    {
                        v += vperiod;
                    }
                }

                u = uu;

                if (v > _vmax || !isVPer)
                {
                    return status;
                }
            }
        } // while (1)
    }

    /// <summary>
    /// The state of the point <paramref name="_Puv"/> tested with an offset of <paramref name="tol"/>: ON where some
    /// offset points are IN and some OUT (internal use).
    /// </summary>
    public Containment TestOnRestriction(in Point2D _Puv, double tol, bool recadreOnPeriodic = true)
    {
        int nbtabclass = _tabClass.Length();
        if (nbtabclass == 0)
        {
            return In;
        }

        //-- U1 is the First Param and U2 in this case is U1+Period
        double u = _Puv.X;
        double v = _Puv.Y;
        double uu = u, vv = v;

        FaceSurfaceAdaptor surf = new();
        surf.Initialize(_face, false);
        bool isUPer = surf.IsUPeriodic;
        bool isVPer = surf.IsVPeriodic;
        double uperiod = isUPer ? surf.UPeriod() : 0.0;
        double vperiod = isVPer ? surf.VPeriod() : 0.0;
        Containment status = Unknown;
        bool urecadre = false, vrecadre = false;
        int dedans;

        if (recadreOnPeriodic)
        {
            if (isUPer)
            {
                global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(uu, _umin, _umax, uperiod, out uu, out _);
            } // if (IsUPer) {
            //
            if (isVPer)
            {
                global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(vv, _vmin, _vmax, vperiod, out vv, out _);
            } // if (IsVPer) {
        }
        //
        for (;;)
        {
            dedans = 1;
            Point2D puv = new(u, v);

            if (_tabOrien[1] != -1)
            {
                for (int n = 1; n <= nbtabclass; n++)
                {
                    int cur = (int)_tabClass[n].SiDans_OnMode(puv, tol);
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
                    status = On;
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
            { //-- TabOrien(1)=-1  Wrong  Wire

                _fExplorer ??= new FaceExplorer(_face);

                FClassifier classifier = new();
                classifier.Perform(_fExplorer, puv, tol);
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
}
