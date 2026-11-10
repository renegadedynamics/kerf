// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_Intersector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_Intersector.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_Intersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The intersection of a line or a curve with a face: the points of the curve on the face's surface that are IN or
/// ON the face, sorted along the curve.
/// </summary>
/// <remarks>
/// A working object (decision 21): load a face once, then intersect any number of curves with
/// <see cref="Perform(in Line3D, double, double)"/>. On surfaces other than the elementary ones it keeps a polyhedron
/// of the surface, and the sorted boxes of its triangles, for all the curves. It only reads the face.
/// </remarks>
internal class CurveSurfaceIntersector : Transient
{
    private readonly BrepTopolTool _topolTool;
    private readonly FaceSurfaceAdaptor _hsurface;
    private readonly double _tol;
    private readonly NCollection_Sequence<CurveSurfaceIntersectionPoint> _seqPnt = new();
    private readonly NCollection_Sequence<int> _seqState = new();
    private bool _done;
    private readonly bool _ready;
    private int _nbpnt;
    private readonly Face _face;
    private readonly ThePolyhedronOfHInter? _polyhedron;
    private BoundSortBox? _bndBounding;
    private bool _useBoundTol;
    private bool _isParallel; // Curve is "parallel" face surface
                               // This case is recognized only for some pairs
                               // of analytical curves and surfaces (plane - line, ...)
    private Box? _faceBox; // Kerf: the face's box, widened (MissesFace)
    // Kerf: one intersection of a curve and the surface, which each Perform resets, not one made for each: a solid's
    // classifier intersects its line with each face it nears, for every point it classifies (unit 13R)
    private HInter? _hics;
    // Kerf: the lines this thread has intersected with a face, for the tests that hold a classification's cost down
    [ThreadStatic]
    internal static int s_linesIntersected;

    private static void ComputeSamplePars(SurfaceAdaptorBase hsurface,
                                          int nbsu,
                                          int nbsv,
                                          out NCollection_HArray1<double> uPars,
                                          out NCollection_HArray1<double> vPars)
    {
        int nbUInts = hsurface.NbUIntervals(C2);
        int nbVInts = hsurface.NbVIntervals(C2);
        var uInts = new NCollection_Array1<double>(1, nbUInts + 1);
        var vInts = new NCollection_Array1<double>(1, nbVInts + 1);
        hsurface.UIntervals(uInts, C2);
        hsurface.VIntervals(vInts, C2);
        //
        var nbUSubInts = new NCollection_Array1<int>(1, nbUInts);
        var nbVSubInts = new NCollection_Array1<int>(1, nbVInts);
        //
        int i, j, ind, nbU, nbV;
        double t, dt;
        t = uInts[nbUInts + 1] - uInts[1];
        t = 1.0 / t;
        nbU = 0;
        for (i = 1; i <= nbUInts; ++i)
        {
            dt = (uInts[i + 1] - uInts[i]);
            nbUSubInts[i] = RealToInt(nbsu * dt * t) + 1;
            nbU += nbUSubInts[i];
        }

        t = vInts[nbVInts + 1] - vInts[1];
        t = 1.0 / t;
        nbV = 0;
        for (i = 1; i <= nbVInts; ++i)
        {
            dt = (vInts[i + 1] - vInts[i]);
            nbVSubInts[i] = RealToInt(nbsv * dt * t) + 1;
            nbV += nbVSubInts[i];
        }

        uPars = new NCollection_HArray1<double>(1, nbU + 1);
        vPars = new NCollection_HArray1<double>(1, nbV + 1);
        //
        ind = 1;
        for (i = 1; i <= nbUInts; ++i)
        {
            uPars.SetValue(ind++, uInts[i]);
            dt = (uInts[i + 1] - uInts[i]) / nbUSubInts[i];
            t = uInts[i];
            for (j = 1; j < nbUSubInts[i]; ++j)
            {
                t += dt;
                uPars.SetValue(ind++, t);
            }
        }

        uPars.SetValue(ind, uInts[nbUInts + 1]);
        //
        ind = 1;
        for (i = 1; i <= nbVInts; ++i)
        {
            vPars.SetValue(ind++, vInts[i]);
            dt = (vInts[i + 1] - vInts[i]) / nbVSubInts[i];
            t = vInts[i];
            for (j = 1; j < nbVSubInts[i]; ++j)
            {
                t += dt;
                vPars.SetValue(ind++, t);
            }
        }

        vPars.SetValue(ind, vInts[nbVInts + 1]);
    }

    /// <summary>The type of the face's surface.</summary>
    public SurfaceKind SurfaceType() => (HSurfaceTool.GetType(_hsurface));

    /// <summary>
    /// Loads the face <paramref name="face"/>. <paramref name="aTol"/> decides whether the first point of a segment
    /// is near the face (the parameter of an intersection may then be negative, above -aTol). With
    /// <paramref name="aRestr"/> the surface is restricted to the face's UV box. With <paramref name="useBToler"/> the
    /// intersection points are classified with the larger of aTol and the tolerances of the face's edges, otherwise
    /// with a null tolerance.
    /// </summary>
    public CurveSurfaceIntersector(in Face face, double aTol, bool aRestr = true, bool useBToler = true)
    {
        _tol = aTol;
        _done = false;
        _ready = false;
        _nbpnt = 0;
        _useBoundTol = useBToler;
        _isParallel = false;

        // C++ initialises a local adaptor and keeps a copy of it; the port keeps the adaptor itself
        var surface = new FaceSurfaceAdaptor();
        _face = face;
        surface.Initialize(face, aRestr);
        _hsurface = surface;
        _topolTool = new BrepTopolTool(_hsurface);

        SurfaceKind surfaceType = HSurfaceTool.GetType(_hsurface);
        if ((surfaceType != Plane) && (surfaceType != SurfaceKind.Cylinder)
            && (surfaceType != SurfaceKind.Cone) && (surfaceType != SurfaceKind.Sphere)
            && (surfaceType != SurfaceKind.Torus))
        {
            int nbsu, nbsv;
            double u0, v0, u1, v1;
            u0 = _hsurface.FirstUParameter();
            u1 = _hsurface.LastUParameter();
            v0 = _hsurface.FirstVParameter();
            v1 = _hsurface.LastVParameter();
            //
            nbsu = _topolTool.NbSamplesU();
            nbsv = _topolTool.NbSamplesV();
            //
            double uRes = _hsurface.UResolution(1.0);
            double vRes = _hsurface.VResolution(1.0);

            // Checking correlation between number of samples and length of the face along each axis
            const double tresh = 100.0;
            int minSamples = 20;
            const int maxSamples = 40;
            const int maxSamples2 = maxSamples * maxSamples;
            double dU = (u1 - u0) / uRes;
            double dV = (v1 - v0) / vRes;
            if (nbsu < minSamples)
            {
                nbsu = minSamples;
            }

            if (nbsv < minSamples)
            {
                nbsv = minSamples;
            }

            if (nbsu > maxSamples)
            {
                nbsu = maxSamples;
            }

            if (nbsv > maxSamples)
            {
                nbsv = maxSamples;
            }

            if (dU > global::Kerf.Engine.Foundation.Precision.Confusion() && dV > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (Max(dU, dV) > Min(dU, dV) * tresh)
                {
                    minSamples = 10;
                    nbsu = (int)(Math.Sqrt(dU / dV) * maxSamples);
                    if (nbsu < minSamples)
                    {
                        nbsu = minSamples;
                    }

                    nbsv = maxSamples2 / nbsu;
                    if (nbsv < minSamples)
                    {
                        nbsv = minSamples;
                        nbsu = maxSamples2 / minSamples;
                    }
                }
            }
            else
            {
                return; // surface has no extension along one of directions
            }

            int nbUOnS = _hsurface.NbUIntervals(C2);
            int nbVOnS = _hsurface.NbVIntervals(C2);

            if (nbUOnS > 1 || nbVOnS > 1)
            {
                ComputeSamplePars(_hsurface, nbsu, nbsv, out NCollection_HArray1<double> uPars, out NCollection_HArray1<double> vPars);
                _polyhedron = new ThePolyhedronOfHInter(_hsurface,
                                                                         uPars.ChangeArray1(),
                                                                         vPars.ChangeArray1());
            }
            else
            {
                _polyhedron = new ThePolyhedronOfHInter(_hsurface, nbsu, nbsv, u0, v0, u1, v1);
            }
        }

        _ready = true;
    }

    private void InternalCall(HInter hics, double parinf, double parsup)
    {
        if (hics.IsDone && hics.PointCount > 0)
        {
            // Calculate tolerance for 2d classifier
            double mintol3d = Tool.Tolerance(_face);
            double maxtol3d = mintol3d;
            double mintol2d = _tol, maxtol2d = _tol;
            var exp = new Explorer(_face, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                double curtol = Tool.Tolerance(ShapeCasts.Edge(exp.Current()));
                mintol3d = Min(mintol3d, curtol);
                maxtol3d = Max(maxtol3d, curtol);
            }

            double minres = Max(_hsurface.UResolution(mintol3d), _hsurface.VResolution(mintol3d));
            double maxres = Max(_hsurface.UResolution(maxtol3d), _hsurface.VResolution(maxtol3d));
            mintol2d = Max(minres, _tol);
            maxtol2d = Max(maxres, _tol);
            //
            BrepTopolTool? additionalTool = null;
            for (int index = hics.PointCount; index >= 1; index--)
            {
                CurveSurfaceIntersectionPoint hicsPointindex = hics.Point(index);
                Point2D puv = new(hicsPointindex.U(), hicsPointindex.V());

                // TopAbs_State currentstate = myTopolTool->Classify(Puv,Tol);
                Containment currentstate = _topolTool.Classify(puv, !_useBoundTol ? 0 : mintol2d);
                if (_useBoundTol && currentstate == global::Kerf.Topology.Containment.Out && maxtol2d > mintol2d)
                {
                    additionalTool ??= new BrepTopolTool(_hsurface);

                    currentstate = additionalTool.Classify(puv, maxtol2d);
                    if (currentstate == On)
                    {
                        currentstate = global::Kerf.Topology.Containment.Out;
                        // Find out nearest edge and it's tolerance
                        exp.Init(_face, ShapeType.Edge);
                        for (; exp.More(); exp.Next())
                        {
                            Edge e = ShapeCasts.Edge(exp.Current());
                            // OCCT: an edge without a 3D curve (a degenerated edge) gives a null curve to the projection, which
                            // throws (occt-issues.md)
                            Curve3D pc = Tool.Curve(e, out double f, out double l)!;
                            var proj = new ProjectPointOnCurve(hicsPointindex.Pnt(), pc, f, l);
                            if (proj.NbPoints() > 0)
                            {
                                if (proj.LowerDistance() <= maxtol3d)
                                {
                                    // Nearest edge is found, state is really ON
                                    currentstate = On;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (currentstate == global::Kerf.Topology.Containment.In || currentstate == On)
                {
                    double hicsw = hicsPointindex.W();
                    if (hicsw >= parinf && hicsw <= parsup)
                    {
                        double u = hicsPointindex.U();
                        double v = hicsPointindex.V();
                        double w = hicsw;
                        TransitionOnCurve transition = hicsPointindex.Transition();
                        Point3D pnt = hicsPointindex.Pnt();
                        //  Modified by skv - Wed Sep  3 16:14:10 2003 OCC578 Begin
                        int intState = (currentstate == global::Kerf.Topology.Containment.In) ? 0 : 1;
                        //  Modified by skv - Wed Sep  3 16:14:11 2003 OCC578 End

                        if (transition != TransitionOnCurve.Tangent && _face.Orientation == Reversed)
                        {
                            if (transition == TransitionOnCurve.In)
                            {
                                transition = TransitionOnCurve.Out;
                            }
                            else
                            {
                                transition = TransitionOnCurve.In;
                            }
                        }

                        //----- Insertion du point
                        if (_nbpnt == 0)
                        {
                            var ppp = new CurveSurfaceIntersectionPoint(pnt, u, v, w, transition);
                            _seqPnt.Append(ppp);
                            //  Modified by skv - Wed Sep  3 16:14:10 2003 OCC578 Begin
                            _seqState.Append(intState);
                            //  Modified by skv - Wed Sep  3 16:14:11 2003 OCC578 End
                        }
                        else
                        {
                            int i = 1;
                            int b = _nbpnt + 1;
                            while (i <= _nbpnt)
                            {
                                CurveSurfaceIntersectionPoint pnti = _seqPnt.Value(i);
                                double wi = pnti.W();
                                if (wi >= w)
                                {
                                    b = i;
                                    i = _nbpnt;
                                }

                                i++;
                            }

                            var ppp = new CurveSurfaceIntersectionPoint(pnt, u, v, w, transition);
                            //  Modified by skv - Wed Sep  3 16:14:10 2003 OCC578 Begin
                            if (b > _nbpnt)
                            {
                                _seqPnt.Append(ppp);
                                _seqState.Append(intState);
                            }
                            else if (b > 0)
                            {
                                _seqPnt.InsertBefore(b, ppp);
                                _seqState.InsertBefore(b, intState);
                            }
                            //  Modified by skv - Wed Sep  3 16:14:11 2003 OCC578 End
                        }

                        _nbpnt++;
                    }
                } //-- classifier state is IN or ON
            } //-- Loop on Intersection points.
        } //-- HICS.IsDone()
        else if (hics.IsDone)
        {
            _isParallel = hics.IsParallel;
        }
    }

    /// <summary>
    /// Intersects the segment [<paramref name="parMin"/>, <paramref name="parMax"/>] of the line
    /// <paramref name="l"/> (they may be ∓RealLast) with the face.
    /// </summary>
    public void Perform(in Line3D l, double parMin, double parMax)
    {
        _done = false;
        if (!_ready)
        {
            return;
        }

        _done = true;
        _seqPnt.Clear();
        _seqState.Clear();
        _nbpnt = 0;

        // Kerf: a segment of the line that passes well clear of the face can meet it nowhere (unit 13A)
        if (_polyhedron is null && MissesFace(l, parMin, parMax))
        {
            _isParallel = false;
            return;
        }
        s_linesIntersected++;

        HInter hics = _hics ??= new HInter();
        var geomline = new LineCurve3D(l);
        // C++ builds an adaptor and copies it into a handle: one adaptor here
        var hll = new CurveAdaptor(geomline);
        double parinf = parMin;
        double parsup = parMax;
        //
        if (_polyhedron is null)
        {
            hics.Perform(hll, _hsurface);
        }
        else
        {
            var bndTool = new PolygonTool();
            Box boxLine = new();
            bndTool.LinBox(l, _polyhedron.Bounding(), ref boxLine);
            if (bndTool.SegmentCount == 0)
            {
                return;
            }

            for (int nbseg = 1; nbseg <= bndTool.SegmentCount; nbseg++)
            {
                double pinf = bndTool.BeginParam(nbseg);
                double psup = bndTool.EndParam(nbseg);
                double pppp = 0.05 * (psup - pinf);
                pinf -= pppp;
                psup += pppp;
                if ((psup - pinf) < 1e-10)
                {
                    pinf -= 1e-10;
                    psup += 1e-10;
                }

                if (nbseg == 1)
                {
                    parinf = pinf;
                    parsup = psup;
                }
                else
                {
                    if (parinf > pinf)
                    {
                        parinf = pinf;
                    }

                    if (parsup < psup)
                    {
                        parsup = psup;
                    }
                }
            }

            if (parinf > parMax)
            {
                return;
            }

            if (parsup < parMin)
            {
                return;
            }

            if (parinf < parMin)
            {
                parinf = parMin;
            }

            if (parsup > parMax)
            {
                parsup = parMax;
            }

            if (parinf > (parsup - 1e-9))
            {
                return;
            }

            var polygon = new ThePolygonOfHInter(hll, parinf, parsup, 2);
            // OPTIMISATION is 1 in OCCT: the boxes of the triangles are sorted once for all the curves
            if (_bndBounding is null)
            {
                _bndBounding = new BoundSortBox();
                _bndBounding.Initialize(ThePolyhedronToolOfHInter.Bounding(_polyhedron),
                                         ThePolyhedronToolOfHInter.ComponentsBounding(_polyhedron));
            }

            hics.Perform(hll, polygon, _hsurface, _polyhedron, _bndBounding);
        }

        InternalCall(hics, parinf, parsup);
    }

    /// <summary>
    /// Intersects the part [<paramref name="parMin"/>, <paramref name="parMax"/>] of the curve
    /// <paramref name="hCu"/> (they may be ∓infinite) with the face.
    /// </summary>
    public void Perform(CurveAdaptorBase hCu, double parMin, double parMax)
    {
        _done = false;
        if (!_ready)
        {
            return;
        }

        _done = true;
        _seqPnt.Clear();
        //  Modified by skv - Wed Sep  3 16:14:10 2003 OCC578 Begin
        _seqState.Clear();
        //  Modified by skv - Wed Sep  3 16:14:11 2003 OCC578 End
        _nbpnt = 0;
        HInter hics = _hics ??= new HInter();

        //--
        double parinf = parMin;
        double parsup = parMax;

        if (_polyhedron is null)
        {
            hics.Perform(hCu, _hsurface);
        }
        else
        {
            parinf = TheHCurveTool.FirstParameter(hCu);
            parsup = TheHCurveTool.LastParameter(hCu);
            if (parinf < parMin)
            {
                parinf = parMin;
            }

            if (parsup > parMax)
            {
                parsup = parMax;
            }

            if (parinf > (parsup - 1e-9))
            {
                return;
            }

            int nbs;
            nbs = TheHCurveTool.NbSamples(hCu, parinf, parsup);

            var polygon = new ThePolygonOfHInter(hCu, parinf, parsup, nbs);
            // OPTIMISATION is 1 in OCCT: the boxes of the triangles are sorted once for all the curves
            if (_bndBounding is null)
            {
                _bndBounding = new BoundSortBox();
                _bndBounding.Initialize(ThePolyhedronToolOfHInter.Bounding(_polyhedron),
                                         ThePolyhedronToolOfHInter.ComponentsBounding(_polyhedron));
            }

            hics.Perform(hCu, polygon, _hsurface, _polyhedron, _bndBounding);
        }

        InternalCall(hics, parinf, parsup);
    }

    /// <summary>True when the intersection has been computed.</summary>
    public bool IsDone => (_done);

    /// <summary>The number of intersection points.</summary>
    public int PntCount => (_nbpnt);

    /// <summary>The U parameter of the <paramref name="i"/>-th point on the surface.</summary>
    public double UParameter(int i) => (_seqPnt.Value(i).U());

    /// <summary>The V parameter of the <paramref name="i"/>-th point on the surface.</summary>
    public double VParameter(int i) => (_seqPnt.Value(i).V());

    /// <summary>The parameter of the <paramref name="i"/>-th point on the line or curve.</summary>
    public double WParameter(int i) => (_seqPnt.Value(i).W());

    /// <summary>The <paramref name="i"/>-th point.</summary>
    public Point3D Pnt(int i) => (_seqPnt.Value(i).Pnt());

    /// <summary>The transition of the line or curve on the surface at the <paramref name="i"/>-th point (taking the face's orientation into account).</summary>
    public TransitionOnCurve Transition(int i) => (_seqPnt.Value(i).Transition());

    /// <summary>The state of the <paramref name="i"/>-th point in the face: IN, or ON its boundary.</summary>
    //  Modified by skv - Wed Sep  3 15:34:18 2003 OCC578 Begin
    public Containment State(int i) => (_seqState.Value(i) == 0) ? global::Kerf.Topology.Containment.In : On;

    /// <summary>
    /// True if the curve is parallel to the surface or lies on it (only recognised for some analytic pairs, a line and
    /// a plane for one).
    /// </summary>
    public bool IsParallel => _isParallel;

    /// <summary>
    /// Kerf (unit 15C): the flag of a line found parallel set back to a new intersector's. A <c>Perform</c> sets it only when
    /// it intersects and finds no point, so it carries over the others, as in OCCT; a kept solid classifier sets it back
    /// between points (<see cref="Classification.SolidClassifier.ForANewPoint"/>), so each point sees what a new one would.
    /// </summary>
    internal void ForgetParallel() => _isParallel = false;

    /// <summary>The face.</summary>
    public Face Face() => (_face);

    /// <summary>The state of the point <paramref name="puv"/> of the face's parameter space (tolerance 1e-7).</summary>
    public Containment ClassifyUVPoint(in Point2D puv)
    {
        Containment state = _topolTool.Classify(puv, 1e-7);
        return state;
    }

    // Kerf (unit 13A): whether the part [parMin, parMax] of the line passes clear of the face's box, widened past anything the
    // intersection could keep. On an elementary surface the line meets the whole surface, and InternalCall keeps a point only
    // if it's in the face or on its boundary, within the edges' tolerances in 3D or the 2D tolerance in the parameters, and its
    // parameter is in [parMin, parMax]; such a point is in the face's box (its edges or surface, each widened by its tolerance)
    // widened by those tolerances. So the line meets no such point, and what Perform gives is the same, nothing. A solid's
    // classifier intersects its ray with every face, and on a part most are far from the ray: a point classified in a door of
    // 52 faces was about 60 KB of curves, adaptors and lists, most of them for faces the ray never came near.
    // IsParallel is left false: a parallel line found nothing either, and the classifier asks it only to see whether its
    // point is within the tolerance of the surface inside the face, which a point outside the widened box isn't
    private bool MissesFace(in Line3D l, double parMin, double parMax)
    {
        if (_faceBox is not { } box)
        {
            box = new Box();
            // Not a torus: its box can be short of the face where the tube is wider than the ring (OCC620a.brep's)
            if (HSurfaceTool.GetType(_hsurface) is Plane or SurfaceKind.Cylinder or SurfaceKind.Cone or SurfaceKind.Sphere)
            {
                Measurement.Bounds.Add(_face, ref box, false);
            }
            if (!box.IsVoid && !box.IsOpen)
            {
                double tol3d = Tool.Tolerance(_face);
                for (var exp = new Explorer(_face, ShapeType.Edge); exp.More(); exp.Next())
                {
                    tol3d = Max(tol3d, Tool.Tolerance(ShapeCasts.Edge(exp.Current())));
                }
                // The 2D tolerance InternalCall may use, in 3D: a parameter's length on the surface is at most 1 / resolution
                double tol2d = Max(Max(_hsurface.UResolution(tol3d), _hsurface.VResolution(tol3d)), _tol);
                double perParameter = Max(1 / _hsurface.UResolution(1.0), 1 / _hsurface.VResolution(1.0));
                box.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                double size = Math.Sqrt(((xmax - xmin) * (xmax - xmin)) + ((ymax - ymin) * (ymax - ymin)) + ((zmax - zmin) * (zmax - zmin)));
                double widen = (10 * tol3d) + (10 * tol2d * perParameter) + (1e-3 * size) + global::Kerf.Engine.Foundation.Precision.Confusion();
                if (double.IsFinite(widen))
                {
                    box.Enlarge(widen);
                }
                else
                {
                    box = new Box();
                }
            }
            _faceBox = box;
        }

        if (box.IsVoid || box.IsOpen)
        {
            return false;
        }

        box.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        Point3D o = l.Location;
        Direction3D d = l.Direction;
        double from = parMin, to = parMax;
        return !Slab(o.X, d.X, x0, x1, ref from, ref to) || !Slab(o.Y, d.Y, y0, y1, ref from, ref to) || !Slab(o.Z, d.Z, z0, z1, ref from, ref to);
    }

    // Narrows [from, to] to the parameters where the line is between low and high on one axis; false if none are
    private static bool Slab(double origin, double direction, double low, double high, ref double from, ref double to)
    {
        if (Math.Abs(direction) < 1e-300)
        {
            return origin >= low && origin <= high;
        }
        double t0 = (low - origin) / direction, t1 = (high - origin) / direction;
        if (t0 > t1)
        {
            (t0, t1) = (t1, t0);
        }
        from = Max(from, t0);
        to = Min(to, t1);
        return from <= to;
    }

    /// <summary>The box of the polyhedron of the surface; a void box for the elementary surfaces.</summary>
    public Box Bounding()
    {
        if (_polyhedron is not null)
        {
            return _polyhedron.Bounding();
        }
        else
        {
            Box b = new();
            return b;
        }
    }

    /// <summary>Sets whether the points are classified with the tolerances of the face's edges.</summary>
    public void SetUseBoundToler(bool useBToler) => _useBoundTol = useBToler;

    /// <summary>True if the points are classified with the tolerances of the face's edges.</summary>
    public bool GetUseBoundToler() => _useBoundTol;
}
