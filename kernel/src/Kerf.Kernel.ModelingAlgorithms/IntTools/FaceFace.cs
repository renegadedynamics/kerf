// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// Intersects the surfaces of two faces (the face/face intersection of the Boolean operations): the intersection
/// curves, each with its 2D curves on the faces and its tolerances, and the isolated intersection points. The
/// faces' tolerances (plus half the fuzzy value each) are the intersection criteria.
/// </summary>
/// <remarks>
/// A working object (decision 21), as GeomInt_IntSS: set the parameters, Perform, read the results. Plane/plane
/// pairs are solved analytically; other pairs go through IntPatch_Intersection on the faces' UV bounds (enlarged),
/// then each line is made a curve (trimmed analytic curves, approximated or polygonal B-splines, approximated
/// restriction lines). <see cref="Lines"/> hands out the curves frozen; the Boolean operations read
/// <see cref="InternalLines"/>.
/// </remarks>
internal partial class FaceFace
{
    protected bool _isDone;
    protected PatchIntersection _intersector;
    protected LineConstructor _lConstruct;
    protected SurfaceAdaptor _hs1;
    protected SurfaceAdaptor _hs2;
    protected int _nbrestr;
    protected bool _approx;
    protected bool _approx1;
    protected bool _approx2;
    protected double _tolApprox;
    protected double _tolF1;
    protected double _tolF2;
    protected double _tol;
    protected double _fuzzyValue;
    protected NCollection_Sequence<IntersectionCurve> _seqOfCurve;
    protected bool _tangentFaces;
    protected Face _face1;
    protected Face _face2;
    protected NCollection_Sequence<PntOn2Faces> _pnts;
    protected NCollection_List<PntOn2S> _listOfPnts;
    protected IntersectionContext? _context;

    /// <summary>Empty constructor: approximation of the 3D and both 2D curves at 1e-7.</summary>
    public FaceFace()
    {
        _intersector = new PatchIntersection();
        _lConstruct = new LineConstructor();
        _seqOfCurve = new NCollection_Sequence<IntersectionCurve>();
        _pnts = new NCollection_Sequence<PntOn2Faces>();
        _listOfPnts = new NCollection_List<PntOn2S>();
        _face1 = new Face();
        _face2 = new Face();
        _isDone = false;
        _tangentFaces = false;
        //
        _hs1 = new SurfaceAdaptor();
        _hs2 = new SurfaceAdaptor();
        _tolF1 = 0.0;
        _tolF2 = 0.0;
        _tol = 0.0;
        _fuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        SetParameters(true, true, true, 1.0e-07);
    }

    /// <summary>Sets the intersection context (one is made by Perform if none is set).</summary>
    public void SetContext(IntersectionContext? aContext) => _context = aContext;

    /// <summary>The intersection context.</summary>
    public IntersectionContext? Context() => _context;

    /// <summary>The first of the processed faces.</summary>
    public Face Face1() => _face1;

    /// <summary>The second of the processed faces.</summary>
    public Face Face2() => _face2;

    /// <summary>True if the faces are tangent (same domain).</summary>
    public bool TangentFaces() => _tangentFaces;

    /// <summary>
    /// The intersection points (a copy). OCCT never clears them: a second Perform on the same object appends to the
    /// first's (occt-issues.md).
    /// </summary>
    public NCollection_Sequence<PntOn2Faces> Points() => new(_pnts);

    /// <summary>True if the intersection was successful.</summary>
    public bool IsDone => _isDone;

    /// <summary>
    /// The intersection curves (a copy of the sequence), their 3D and 2D curves frozen (design 4.2). Throws
    /// StdFail_NotDone if not done.
    /// </summary>
    public NCollection_Sequence<IntersectionCurve> Lines()
    {
        StdFail_NotDone_Raise_if(!_isDone, "IntTools_FaceFace::Lines() => myIntersector NOT DONE");
        NCollection_Sequence<IntersectionCurve> lines = new(_seqOfCurve);
        for (int i = 1; i <= lines.Length(); i++)
        {
            IntersectionCurve ic = lines.Value(i);
            ic.Curve()?.Freeze();
            ic.FirstCurve2d()?.Freeze();
            ic.SecondCurve2d()?.Freeze();
        }
        return lines;
    }

    /// <summary>
    /// The intersection curves themselves, unfrozen (OCCT's reference): for the Boolean operations, which keep
    /// building on them. Throws StdFail_NotDone if not done.
    /// </summary>
    internal NCollection_Sequence<IntersectionCurve> InternalLines()
    {
        StdFail_NotDone_Raise_if(!_isDone, "IntTools_FaceFace::Lines() => myIntersector NOT DONE");
        return _seqOfCurve;
    }

    /// <summary>
    /// Sets whether the 3D curves (ToApproxC3d) and the 2D curves on each face are approximated, and the
    /// approximation's tolerance.
    /// </summary>
    public void SetParameters(bool toApproxC3d, bool toApproxC2dOnS1, bool toApproxC2dOnS2,
                              double approximationTolerance)
    {
        _approx = toApproxC3d;
        _approx1 = toApproxC2dOnS1;
        _approx2 = toApproxC2dOnS2;
        _tolApprox = approximationTolerance;
    }

    /// <summary>Sets the fuzzy value (at least Precision::Confusion).</summary>
    public void SetFuzzyValue(double fuzz) => _fuzzyValue = Max(fuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <summary>The fuzzy value.</summary>
    public double FuzzyValue() => _fuzzyValue;

    /// <summary>
    /// Sets the start points of the parametric intersection (a copy of the list; OCCT copies it too). The list is
    /// kept between Performs and may grow (decision 706), as OCCT's member.
    /// </summary>
    public void SetList(NCollection_List<PntOn2S> aListOfPnts) => _listOfPnts = new(aListOfPnts);

    private static bool IsTreatAnalityc(FaceSurfaceAdaptor bas1, FaceSurfaceAdaptor bas2, double tol)
    {
        const double tolang = 1.0e-8;
        double high = 0.0;

        SurfaceKind type1 = bas1.GetType();
        SurfaceKind type2 = bas2.GetType();

        Plane3D s1;
        Cylinder3D s2;
        if (type1 == Plane)
        {
            s1 = bas1.Plane();
        }
        else if (type2 == Plane)
        {
            s1 = bas2.Plane();
        }
        else
        {
            return true;
        }

        if (type1 == SurfaceKind.Cylinder)
        {
            s2 = bas1.Cylinder();
            double vMin = bas1.FirstVParameter();
            double vMax = bas1.LastVParameter();

            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                return true;
            }
            else
            {
                high = vMax - vMin;
            }
        }
        else if (type2 == SurfaceKind.Cylinder)
        {
            s2 = bas2.Cylinder();

            double vMin = bas2.FirstVParameter();
            double vMax = bas2.LastVParameter();

            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                return true;
            }
            else
            {
                high = vMax - vMin;
            }
        }
        else
        {
            return true;
        }

        QuadQuadGeo inter = new();
        inter.Perform(s1, s2, tolang, tol, high);
        if (inter.TypeInter() == ResultType.Ellipse)
        {
            Ellipse3D el = inter.Ellipse(1);
            double majorR = el.MajorRadius;
            double minorR = el.MinorRadius;

            return (majorR < 100000.0 * minorR);
        }
        else
        {
            return inter.IsDone;
        }
    }

    /// <summary>
    /// Intersects the surfaces of aF1 and aF2, with the sum of the faces' tolerances (and the fuzzy value) as the
    /// intersection criterion. theToRunParallel runs the tolerance computations in parallel.
    /// </summary>
    public void Perform(in Face aF1, in Face aF2, bool toRunParallel = false)
    {
        if (_context is null)
        {
            _context = new IntersectionContext();
        }

        _seqOfCurve.Clear(); // OCCT clears the curves but not myPnts: a second Perform appends to the first's points
        _isDone = false;
        _nbrestr = 0; //?

        _face1 = aF1;
        _face2 = aF2;

        FaceSurfaceAdaptor bas1 = _context.SurfaceAdaptor(_face1);
        FaceSurfaceAdaptor bas2 = _context.SurfaceAdaptor(_face2);
        SurfaceKind type1 = bas1.GetType();
        SurfaceKind type2 = bas2.GetType();

        bool bReverse = SortTypes(type1, type2);
        if (bReverse)
        {
            _face1 = aF2;
            _face2 = aF1;
            type1 = bas2.GetType();
            type2 = bas1.GetType();

            // OCCT swaps the kept start points in place: a second reversed Perform swaps them back
            if (_listOfPnts.Extent() != 0)
            {
                double aU1, aV1, aU2, aV2;
                NCollection_List<PntOn2S>.Iterator itP2S = new(_listOfPnts);
                //
                for (; itP2S.More(); itP2S.Next())
                {
                    ref PntOn2S p2S = ref itP2S.ChangeValue();
                    p2S.Parameters(out aU1, out aV1, out aU2, out aV2);
                    p2S.SetValue(aU2, aV2, aU1, aV1);
                }
            }
            //
            bool aproxTmp = _approx1;
            _approx1 = _approx2;
            _approx2 = aproxTmp;
        }

        Surface s1 = Tool.Surface(_face1)!;
        Surface s2 = Tool.Surface(_face2)!;

        double fuzz = _fuzzyValue / 2.0;
        _tolF1 = Tool.Tolerance(_face1) + fuzz;
        _tolF2 = Tool.Tolerance(_face2) + fuzz;
        _tol = _tolF1 + _tolF2;

        double tolArc = _tol;
        double tolTang = tolArc;

        bool isFace1Quad = (type1 == SurfaceKind.Cylinder || type1 == SurfaceKind.Cone || type1 == SurfaceKind.Torus);

        bool isFace2Quad = (type2 == SurfaceKind.Cylinder || type2 == SurfaceKind.Cone || type2 == SurfaceKind.Torus);

        if (type1 == Plane && type2 == Plane)
        {
            double umin, umax, vmin, vmax;
            //
            _context.UVBounds(_face1, out umin, out umax, out vmin, out vmax);
            _hs1.Load(s1, umin, umax, vmin, vmax);
            //
            _context.UVBounds(_face2, out umin, out umax, out vmin, out vmax);
            _hs2.Load(s2, umin, umax, vmin, vmax);
            //
            double tolAng = 1.0e-8;
            //
            PerformPlanes(_hs1, _hs2, _tolF1, _tolF2, tolAng, tolTang, _approx1, _approx2, _seqOfCurve,
                          out _tangentFaces);
            //
            _isDone = true;
            //
            if (!_tangentFaces)
            {
                int nbLinPP = _seqOfCurve.Length();
                if (nbLinPP != 0 && bReverse)
                {
                    Curve2D? c2D1, c2D2;
                    int nbLin = _seqOfCurve.Length();
                    for (int i = 1; i <= nbLin; ++i)
                    {
                        ref IntersectionCurve ic = ref _seqOfCurve[i];
                        c2D1 = ic.FirstCurve2d();
                        c2D2 = ic.SecondCurve2d();
                        ic.SetFirstCurve2d(c2D2);
                        ic.SetSecondCurve2d(c2D1);
                    }
                }
            }
            return;
        } // if(aType1==GeomAbs_Plane && aType2==GeomAbs_Plane){

        if ((type1 == Plane) && isFace2Quad)
        {
            double umin, umax, vmin, vmax;
            // F1
            _context.UVBounds(_face1, out umin, out umax, out vmin, out vmax);
            CorrectPlaneBoundaries(ref umin, ref umax, ref vmin, ref vmax);
            _hs1.Load(s1, umin, umax, vmin, vmax);
            // F2
            _context.UVBounds(_face2, out umin, out umax, out vmin, out vmax);
            CorrectSurfaceBoundaries(_face2, _tol * 2.0, ref umin, ref umax, ref vmin, ref vmax);
            _hs2.Load(s2, umin, umax, vmin, vmax);
        }
        else if ((type2 == Plane) && isFace1Quad)
        {
            double umin, umax, vmin, vmax;
            // F1
            _context.UVBounds(_face1, out umin, out umax, out vmin, out vmax);
            CorrectSurfaceBoundaries(_face1, _tol * 2.0, ref umin, ref umax, ref vmin, ref vmax);
            _hs1.Load(s1, umin, umax, vmin, vmax);
            // F2
            _context.UVBounds(_face2, out umin, out umax, out vmin, out vmax);
            CorrectPlaneBoundaries(ref umin, ref umax, ref vmin, ref vmax);
            _hs2.Load(s2, umin, umax, vmin, vmax);
        }
        else
        {
            double umin, umax, vmin, vmax;
            _context.UVBounds(_face1, out umin, out umax, out vmin, out vmax);
            CorrectSurfaceBoundaries(_face1, _tol * 2.0, ref umin, ref umax, ref vmin, ref vmax);
            _hs1.Load(s1, umin, umax, vmin, vmax);
            _context.UVBounds(_face2, out umin, out umax, out vmin, out vmax);
            CorrectSurfaceBoundaries(_face2, _tol * 2.0, ref umin, ref umax, ref vmin, ref vmax);
            _hs2.Load(s2, umin, umax, vmin, vmax);
        }

        IntersectionTopolTool dom1 = new(_hs1);
        IntersectionTopolTool dom2 = new(_hs2);

        _lConstruct.Load(dom1, dom2, _hs1, _hs2);

        Tolerances(_hs1, _hs2, ref tolTang);

        {
            double uvMaxStep = PatchIntersection.DefineUVMaxStep(_hs1, dom1, _hs2, dom2);
            double deflection = 0.1;
            if (type1 == SurfaceKind.BSplineSurface && type2 == SurfaceKind.BSplineSurface)
            {
                deflection /= 10.0;
            }
            _intersector.SetTolerances(tolArc, tolTang, uvMaxStep, deflection);
        }

        if ((type1 != SurfaceKind.BSplineSurface) && (type1 != SurfaceKind.BezierSurface)
            && (type1 != OtherSurface) && (type2 != SurfaceKind.BSplineSurface)
            && (type2 != SurfaceKind.BezierSurface) && (type2 != OtherSurface))
        {
            if ((type1 == SurfaceKind.Torus) || (type2 == SurfaceKind.Torus))
            {
                _listOfPnts.Clear();
            }
        }

        // INTTOOLS_FACEFACE_DEBUG's print of the start points goes (translation rules, section 6).

        bool isGeomInt = IsTreatAnalityc(bas1, bas2, _tol);
        if (aF1.IsSame(aF2))
        {
            _intersector.Perform(_hs1, dom1, tolArc, tolTang);
        }
        else
        {
            _intersector.Perform(_hs1, dom1, _hs2, dom2, tolArc, tolTang, _listOfPnts, isGeomInt);
        }

        _isDone = _intersector.IsDone;

        if (_isDone)
        {
            _tangentFaces = _intersector.TangentFaces();
            if (_tangentFaces)
            {
                return;
            }
            //
            int nbLinIntersector = _intersector.LineCount;
            for (int i = 1; i <= nbLinIntersector; ++i)
            {
                MakeCurve(i, dom1, dom2, tolArc);
            }
            //
            ComputeTolReached3d(toRunParallel);
            //
            if (bReverse)
            {
                Curve2D? c2D1, c2D2;
                //
                int nbLinSeqOfCurve = _seqOfCurve.Length();
                for (int i = 1; i <= nbLinSeqOfCurve; ++i)
                {
                    ref IntersectionCurve ic = ref _seqOfCurve[i];
                    c2D1 = ic.FirstCurve2d();
                    c2D2 = ic.SecondCurve2d();
                    ic.SetFirstCurve2d(c2D2);
                    ic.SetSecondCurve2d(c2D1);
                }
            }

            // Points
            bool bValid2D1, bValid2D2;
            double u1, v1, u2, v2;
            PntOnFace pntOnF1 = new(), pntOnF2 = new();
            PntOn2Faces pntOn2Faces = new();
            //
            int nbPnts = _intersector.PntCount;
            for (int i = 1; i <= nbPnts; ++i)
            {
                PntOn2S isPnt = _intersector.Point(i).PntOn2S();
                Point3D pnt = isPnt.Value();
                isPnt.Parameters(out u1, out v1, out u2, out v2);
                //
                // check the validity of the intersection point for the faces
                bValid2D1 = _context.IsPointInOnFace(_face1, new Point2D(u1, v1));
                if (!bValid2D1)
                {
                    continue;
                }
                //
                bValid2D2 = _context.IsPointInOnFace(_face2, new Point2D(u2, v2));
                if (!bValid2D2)
                {
                    continue;
                }
                //
                // add the intersection point
                pntOnF1.Init(_face1, pnt, u1, v1);
                pntOnF2.Init(_face2, pnt, u2, v2);
                //
                if (!bReverse)
                {
                    pntOn2Faces.SetP1(pntOnF1);
                    pntOn2Faces.SetP2(pntOnF2);
                }
                else
                {
                    pntOn2Faces.SetP2(pntOnF1);
                    pntOn2Faces.SetP1(pntOnF2);
                }

                _pnts.Append(pntOn2Faces);
            }
        }
    }

    /// <summary>
    /// Computes the valid tolerance of the intersection curves: the largest deviation between the 3D curve and its
    /// 2D curves on the faces, or, without a 2D curve, between the 3D curve and the face's surface.
    /// </summary>
    protected void ComputeTolReached3d(bool toRunParallel)
    {
        int i, j, nbLin = _seqOfCurve.Length();
        if (nbLin == 0)
        {
            return;
        }
        //
        // Minimal tangential tolerance for the curve
        double tolFMax = Max(_tolF1, _tolF2);
        //
        Surface s1 = _hs1.Surface()!;
        Surface s2 = _hs2.Surface()!;
        //
        for (i = 1; i <= nbLin; ++i)
        {
            ref IntersectionCurve ic = ref _seqOfCurve[i];
            Curve3D? c3D = ic.Curve();
            if (c3D is null)
            {
                continue;
            }
            //
            double tolC = ic.Tolerance();
            double first = c3D.FirstParameter;
            double last = c3D.LastParameter;
            //
            // Compute the tolerance for the curve
            Curve2D? c2D1 = ic.FirstCurve2d();
            Curve2D? c2D2 = ic.SecondCurve2d();
            //
            for (j = 0; j < 2; ++j)
            {
                Curve2D? c2D = j == 0 ? c2D1 : c2D2;
                if (c2D is not null)
                {
                    // Look for the maximal deviation between 3D and 2D curves
                    Surface s = j == 0 ? s1 : s2;
                    if (IntersectionTools.ComputeTolerance(c3D, c2D, s, first, last, out double d, out _,
                                                        global::Kerf.Engine.Foundation.Precision.PConfusion(), toRunParallel))
                    {
                        if (d > tolC)
                        {
                            tolC = d;
                        }
                    }
                }
                else
                {
                    // Look for the maximal deviation between 3D curve and surface
                    Face f = j == 0 ? _face1 : _face2;
                    double d = FindMaxDistance(c3D, first, last, f, _context!);
                    if (d > tolC)
                    {
                        tolC = d;
                    }
                }
            }
            // Set the valid tolerance for the curve
            ic.SetTolerance(tolC);
            //
            // Set the tangential tolerance for the curve.
            // Note, that, currently, computation of the tangential tolerance is
            // implemented for the Plane/Plane case only.
            // Thus, set the tangential tolerance equal to maximal tolerance of faces.
            if (ic.TangentialTolerance() < tolFMax)
            {
                ic.SetTangentialTolerance(tolFMax);
            }
        }
    }

    /// <summary>
    /// Post-processes the result lines: splits the closed 3D curves into parts (bToSplit), and rejects the extra
    /// lines of a plane/cone pair that gave four lines.
    /// </summary>
    public void PrepareLines3D(bool bToSplit = true)
    {
        int i, nbCurves;
        SurfaceKind type1, type2;
        NCollection_Sequence<IntersectionCurve> newCvs = new();
        //
        // 1. Treatment closed  curves
        nbCurves = _seqOfCurve.Length();
        for (i = 1; i <= nbCurves; ++i)
        {
            IntersectionCurve ic = _seqOfCurve.Value(i);
            //
            if (bToSplit)
            {
                int j, nbC;
                NCollection_Sequence<IntersectionCurve> seqCvs = new();
                //
                nbC = IntersectionTools.SplitCurve(ic, seqCvs);
                if (nbC != 0)
                {
                    for (j = 1; j <= nbC; ++j)
                    {
                        newCvs.Append(seqCvs.Value(j));
                    }
                }
                else
                {
                    newCvs.Append(ic);
                }
            }
            else
            {
                newCvs.Append(ic);
            }
        }
        //
        // 2. Plane\Cone intersection when we had 4 curves
        type1 = _hs1.GetType();
        type2 = _hs2.GetType();
        nbCurves = newCvs.Length();
        //
        if ((type1 == Plane && type2 == SurfaceKind.Cone)
            || (type2 == Plane && type1 == SurfaceKind.Cone))
        {
            if (nbCurves == 4)
            {
                CurveKind cType1;
                //
                cType1 = newCvs.Value(1).Type();
                if (cType1 == CurveKind.Line)
                {
                    NCollection_Sequence<IntersectionCurve> seqIn = new(), seqOut = new();
                    //
                    for (i = 1; i <= nbCurves; ++i)
                    {
                        seqIn.Append(newCvs.Value(i));
                    }
                    //
                    IntersectionTools.RejectLines(seqIn, seqOut);
                    //
                    newCvs.Clear();
                    nbCurves = seqOut.Length();
                    for (i = 1; i <= nbCurves; ++i)
                    {
                        newCvs.Append(seqOut.Value(i));
                    }
                }
            }
        } // if ((aType1==GeomAbs_Plane && aType2==GeomAbs_Cone)...
        //
        // 3. Fill  mySeqOfCurve
        _seqOfCurve.Clear();
        nbCurves = newCvs.Length();
        for (i = 1; i <= nbCurves; ++i)
        {
            _seqOfCurve.Append(newCvs.Value(i));
        }
    }
}
