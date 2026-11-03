// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.TopAbs_State;

namespace Occt;

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
public partial class IntTools_FaceFace
{
    protected bool myIsDone;
    protected IntPatch_Intersection myIntersector;
    protected GeomInt_LineConstructor myLConstruct;
    protected GeomAdaptor_Surface myHS1;
    protected GeomAdaptor_Surface myHS2;
    protected int myNbrestr;
    protected bool myApprox;
    protected bool myApprox1;
    protected bool myApprox2;
    protected double myTolApprox;
    protected double myTolF1;
    protected double myTolF2;
    protected double myTol;
    protected double myFuzzyValue;
    protected NCollection_Sequence<IntTools_Curve> mySeqOfCurve;
    protected bool myTangentFaces;
    protected TopoDS_Face myFace1;
    protected TopoDS_Face myFace2;
    protected NCollection_Sequence<IntTools_PntOn2Faces> myPnts;
    protected NCollection_List<IntSurf_PntOn2S> myListOfPnts;
    protected IntTools_Context? myContext;

    /// <summary>Empty constructor: approximation of the 3D and both 2D curves at 1e-7.</summary>
    public IntTools_FaceFace()
    {
        myIntersector = new IntPatch_Intersection();
        myLConstruct = new GeomInt_LineConstructor();
        mySeqOfCurve = new NCollection_Sequence<IntTools_Curve>();
        myPnts = new NCollection_Sequence<IntTools_PntOn2Faces>();
        myListOfPnts = new NCollection_List<IntSurf_PntOn2S>();
        myFace1 = new TopoDS_Face();
        myFace2 = new TopoDS_Face();
        myIsDone = false;
        myTangentFaces = false;
        //
        myHS1 = new GeomAdaptor_Surface();
        myHS2 = new GeomAdaptor_Surface();
        myTolF1 = 0.0;
        myTolF2 = 0.0;
        myTol = 0.0;
        myFuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        SetParameters(true, true, true, 1.0e-07);
    }

    /// <summary>Sets the intersection context (one is made by Perform if none is set).</summary>
    public void SetContext(IntTools_Context? aContext) => myContext = aContext;

    /// <summary>The intersection context.</summary>
    public IntTools_Context? Context() => myContext;

    /// <summary>The first of the processed faces.</summary>
    public TopoDS_Face Face1() => myFace1;

    /// <summary>The second of the processed faces.</summary>
    public TopoDS_Face Face2() => myFace2;

    /// <summary>True if the faces are tangent (same domain).</summary>
    public bool TangentFaces() => myTangentFaces;

    /// <summary>
    /// The intersection points (a copy). OCCT never clears them: a second Perform on the same object appends to the
    /// first's (occt-issues.md).
    /// </summary>
    public NCollection_Sequence<IntTools_PntOn2Faces> Points() => new(myPnts);

    /// <summary>True if the intersection was successful.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>
    /// The intersection curves (a copy of the sequence), their 3D and 2D curves frozen (design 4.2). Throws
    /// StdFail_NotDone if not done.
    /// </summary>
    public NCollection_Sequence<IntTools_Curve> Lines()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "IntTools_FaceFace::Lines() => myIntersector NOT DONE");
        NCollection_Sequence<IntTools_Curve> aLines = new(mySeqOfCurve);
        for (int i = 1; i <= aLines.Length(); i++)
        {
            IntTools_Curve aIC = aLines.Value(i);
            aIC.Curve()?.Freeze();
            aIC.FirstCurve2d()?.Freeze();
            aIC.SecondCurve2d()?.Freeze();
        }
        return aLines;
    }

    /// <summary>
    /// The intersection curves themselves, unfrozen (OCCT's reference): for the Boolean operations, which keep
    /// building on them. Throws StdFail_NotDone if not done.
    /// </summary>
    internal NCollection_Sequence<IntTools_Curve> InternalLines()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "IntTools_FaceFace::Lines() => myIntersector NOT DONE");
        return mySeqOfCurve;
    }

    /// <summary>
    /// Sets whether the 3D curves (ToApproxC3d) and the 2D curves on each face are approximated, and the
    /// approximation's tolerance.
    /// </summary>
    public void SetParameters(bool ToApproxC3d, bool ToApproxC2dOnS1, bool ToApproxC2dOnS2,
                              double ApproximationTolerance)
    {
        myApprox = ToApproxC3d;
        myApprox1 = ToApproxC2dOnS1;
        myApprox2 = ToApproxC2dOnS2;
        myTolApprox = ApproximationTolerance;
    }

    /// <summary>Sets the fuzzy value (at least Precision::Confusion).</summary>
    public void SetFuzzyValue(double theFuzz) => myFuzzyValue = Max(theFuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <summary>The fuzzy value.</summary>
    public double FuzzyValue() => myFuzzyValue;

    /// <summary>
    /// Sets the start points of the parametric intersection (a copy of the list; OCCT copies it too). The list is
    /// kept between Performs and may grow (decision 706), as OCCT's member.
    /// </summary>
    public void SetList(NCollection_List<IntSurf_PntOn2S> aListOfPnts) => myListOfPnts = new(aListOfPnts);

    private static bool isTreatAnalityc(BRepAdaptor_Surface theBAS1, BRepAdaptor_Surface theBAS2, double theTol)
    {
        const double Tolang = 1.0e-8;
        double aHigh = 0.0;

        SurfaceKind aType1 = theBAS1.GetType();
        SurfaceKind aType2 = theBAS2.GetType();

        Plane3D aS1;
        Cylinder aS2;
        if (aType1 == Plane)
        {
            aS1 = theBAS1.Plane();
        }
        else if (aType2 == Plane)
        {
            aS1 = theBAS2.Plane();
        }
        else
        {
            return true;
        }

        if (aType1 == SurfaceKind.Cylinder)
        {
            aS2 = theBAS1.Cylinder();
            double VMin = theBAS1.FirstVParameter();
            double VMax = theBAS1.LastVParameter();

            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
            {
                return true;
            }
            else
            {
                aHigh = VMax - VMin;
            }
        }
        else if (aType2 == SurfaceKind.Cylinder)
        {
            aS2 = theBAS2.Cylinder();

            double VMin = theBAS2.FirstVParameter();
            double VMax = theBAS2.LastVParameter();

            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
            {
                return true;
            }
            else
            {
                aHigh = VMax - VMin;
            }
        }
        else
        {
            return true;
        }

        IntAna_QuadQuadGeo inter = new();
        inter.Perform(aS1, aS2, Tolang, theTol, aHigh);
        if (inter.TypeInter() == IntAna_ResultType.IntAna_Ellipse)
        {
            Ellipse3D anEl = inter.Ellipse(1);
            double aMajorR = anEl.MajorRadius();
            double aMinorR = anEl.MinorRadius();

            return (aMajorR < 100000.0 * aMinorR);
        }
        else
        {
            return inter.IsDone();
        }
    }

    /// <summary>
    /// Intersects the surfaces of aF1 and aF2, with the sum of the faces' tolerances (and the fuzzy value) as the
    /// intersection criterion. theToRunParallel runs the tolerance computations in parallel.
    /// </summary>
    public void Perform(in TopoDS_Face aF1, in TopoDS_Face aF2, bool theToRunParallel = false)
    {
        if (myContext is null)
        {
            myContext = new IntTools_Context();
        }

        mySeqOfCurve.Clear(); // OCCT clears the curves but not myPnts: a second Perform appends to the first's points
        myIsDone = false;
        myNbrestr = 0; //?

        myFace1 = aF1;
        myFace2 = aF2;

        BRepAdaptor_Surface aBAS1 = myContext.SurfaceAdaptor(myFace1);
        BRepAdaptor_Surface aBAS2 = myContext.SurfaceAdaptor(myFace2);
        SurfaceKind aType1 = aBAS1.GetType();
        SurfaceKind aType2 = aBAS2.GetType();

        bool bReverse = SortTypes(aType1, aType2);
        if (bReverse)
        {
            myFace1 = aF2;
            myFace2 = aF1;
            aType1 = aBAS2.GetType();
            aType2 = aBAS1.GetType();

            // OCCT swaps the kept start points in place: a second reversed Perform swaps them back
            if (myListOfPnts.Extent() != 0)
            {
                double aU1, aV1, aU2, aV2;
                NCollection_List<IntSurf_PntOn2S>.Iterator aItP2S = new(myListOfPnts);
                //
                for (; aItP2S.More(); aItP2S.Next())
                {
                    ref IntSurf_PntOn2S aP2S = ref aItP2S.ChangeValue();
                    aP2S.Parameters(out aU1, out aV1, out aU2, out aV2);
                    aP2S.SetValue(aU2, aV2, aU1, aV1);
                }
            }
            //
            bool anAproxTmp = myApprox1;
            myApprox1 = myApprox2;
            myApprox2 = anAproxTmp;
        }

        Geom_Surface S1 = BRep_Tool.Surface(myFace1)!;
        Geom_Surface S2 = BRep_Tool.Surface(myFace2)!;

        double aFuzz = myFuzzyValue / 2.0;
        myTolF1 = BRep_Tool.Tolerance(myFace1) + aFuzz;
        myTolF2 = BRep_Tool.Tolerance(myFace2) + aFuzz;
        myTol = myTolF1 + myTolF2;

        double TolArc = myTol;
        double TolTang = TolArc;

        bool isFace1Quad = (aType1 == SurfaceKind.Cylinder || aType1 == SurfaceKind.Cone || aType1 == SurfaceKind.Torus);

        bool isFace2Quad = (aType2 == SurfaceKind.Cylinder || aType2 == SurfaceKind.Cone || aType2 == SurfaceKind.Torus);

        if (aType1 == Plane && aType2 == Plane)
        {
            double umin, umax, vmin, vmax;
            //
            myContext.UVBounds(myFace1, out umin, out umax, out vmin, out vmax);
            myHS1.Load(S1, umin, umax, vmin, vmax);
            //
            myContext.UVBounds(myFace2, out umin, out umax, out vmin, out vmax);
            myHS2.Load(S2, umin, umax, vmin, vmax);
            //
            double TolAng = 1.0e-8;
            //
            PerformPlanes(myHS1, myHS2, myTolF1, myTolF2, TolAng, TolTang, myApprox1, myApprox2, mySeqOfCurve,
                          out myTangentFaces);
            //
            myIsDone = true;
            //
            if (!myTangentFaces)
            {
                int NbLinPP = mySeqOfCurve.Length();
                if (NbLinPP != 0 && bReverse)
                {
                    Geom2d_Curve? aC2D1, aC2D2;
                    int aNbLin = mySeqOfCurve.Length();
                    for (int i = 1; i <= aNbLin; ++i)
                    {
                        ref IntTools_Curve aIC = ref mySeqOfCurve[i];
                        aC2D1 = aIC.FirstCurve2d();
                        aC2D2 = aIC.SecondCurve2d();
                        aIC.SetFirstCurve2d(aC2D2);
                        aIC.SetSecondCurve2d(aC2D1);
                    }
                }
            }
            return;
        } // if(aType1==GeomAbs_Plane && aType2==GeomAbs_Plane){

        if ((aType1 == Plane) && isFace2Quad)
        {
            double umin, umax, vmin, vmax;
            // F1
            myContext.UVBounds(myFace1, out umin, out umax, out vmin, out vmax);
            CorrectPlaneBoundaries(ref umin, ref umax, ref vmin, ref vmax);
            myHS1.Load(S1, umin, umax, vmin, vmax);
            // F2
            myContext.UVBounds(myFace2, out umin, out umax, out vmin, out vmax);
            CorrectSurfaceBoundaries(myFace2, myTol * 2.0, ref umin, ref umax, ref vmin, ref vmax);
            myHS2.Load(S2, umin, umax, vmin, vmax);
        }
        else if ((aType2 == Plane) && isFace1Quad)
        {
            double umin, umax, vmin, vmax;
            // F1
            myContext.UVBounds(myFace1, out umin, out umax, out vmin, out vmax);
            CorrectSurfaceBoundaries(myFace1, myTol * 2.0, ref umin, ref umax, ref vmin, ref vmax);
            myHS1.Load(S1, umin, umax, vmin, vmax);
            // F2
            myContext.UVBounds(myFace2, out umin, out umax, out vmin, out vmax);
            CorrectPlaneBoundaries(ref umin, ref umax, ref vmin, ref vmax);
            myHS2.Load(S2, umin, umax, vmin, vmax);
        }
        else
        {
            double umin, umax, vmin, vmax;
            myContext.UVBounds(myFace1, out umin, out umax, out vmin, out vmax);
            CorrectSurfaceBoundaries(myFace1, myTol * 2.0, ref umin, ref umax, ref vmin, ref vmax);
            myHS1.Load(S1, umin, umax, vmin, vmax);
            myContext.UVBounds(myFace2, out umin, out umax, out vmin, out vmax);
            CorrectSurfaceBoundaries(myFace2, myTol * 2.0, ref umin, ref umax, ref vmin, ref vmax);
            myHS2.Load(S2, umin, umax, vmin, vmax);
        }

        IntTools_TopolTool dom1 = new(myHS1);
        IntTools_TopolTool dom2 = new(myHS2);

        myLConstruct.Load(dom1, dom2, myHS1, myHS2);

        Tolerances(myHS1, myHS2, ref TolTang);

        {
            double UVMaxStep = IntPatch_Intersection.DefineUVMaxStep(myHS1, dom1, myHS2, dom2);
            double Deflection = 0.1;
            if (aType1 == BSplineSurface && aType2 == BSplineSurface)
            {
                Deflection /= 10.0;
            }
            myIntersector.SetTolerances(TolArc, TolTang, UVMaxStep, Deflection);
        }

        if ((aType1 != BSplineSurface) && (aType1 != BezierSurface)
            && (aType1 != OtherSurface) && (aType2 != BSplineSurface)
            && (aType2 != BezierSurface) && (aType2 != OtherSurface))
        {
            if ((aType1 == SurfaceKind.Torus) || (aType2 == SurfaceKind.Torus))
            {
                myListOfPnts.Clear();
            }
        }

        // INTTOOLS_FACEFACE_DEBUG's print of the start points goes (translation rules, section 6).

        bool isGeomInt = isTreatAnalityc(aBAS1, aBAS2, myTol);
        if (aF1.IsSame(aF2))
        {
            myIntersector.Perform(myHS1, dom1, TolArc, TolTang);
        }
        else
        {
            myIntersector.Perform(myHS1, dom1, myHS2, dom2, TolArc, TolTang, myListOfPnts, isGeomInt);
        }

        myIsDone = myIntersector.IsDone();

        if (myIsDone)
        {
            myTangentFaces = myIntersector.TangentFaces();
            if (myTangentFaces)
            {
                return;
            }
            //
            int aNbLinIntersector = myIntersector.NbLines();
            for (int i = 1; i <= aNbLinIntersector; ++i)
            {
                MakeCurve(i, dom1, dom2, TolArc);
            }
            //
            ComputeTolReached3d(theToRunParallel);
            //
            if (bReverse)
            {
                Geom2d_Curve? aC2D1, aC2D2;
                //
                int aNbLinSeqOfCurve = mySeqOfCurve.Length();
                for (int i = 1; i <= aNbLinSeqOfCurve; ++i)
                {
                    ref IntTools_Curve aIC = ref mySeqOfCurve[i];
                    aC2D1 = aIC.FirstCurve2d();
                    aC2D2 = aIC.SecondCurve2d();
                    aIC.SetFirstCurve2d(aC2D2);
                    aIC.SetSecondCurve2d(aC2D1);
                }
            }

            // Points
            bool bValid2D1, bValid2D2;
            double U1, V1, U2, V2;
            IntTools_PntOnFace aPntOnF1 = new(), aPntOnF2 = new();
            IntTools_PntOn2Faces aPntOn2Faces = new();
            //
            int aNbPnts = myIntersector.NbPnts();
            for (int i = 1; i <= aNbPnts; ++i)
            {
                IntSurf_PntOn2S aISPnt = myIntersector.Point(i).PntOn2S();
                Point3D aPnt = aISPnt.Value();
                aISPnt.Parameters(out U1, out V1, out U2, out V2);
                //
                // check the validity of the intersection point for the faces
                bValid2D1 = myContext.IsPointInOnFace(myFace1, new Point2D(U1, V1));
                if (!bValid2D1)
                {
                    continue;
                }
                //
                bValid2D2 = myContext.IsPointInOnFace(myFace2, new Point2D(U2, V2));
                if (!bValid2D2)
                {
                    continue;
                }
                //
                // add the intersection point
                aPntOnF1.Init(myFace1, aPnt, U1, V1);
                aPntOnF2.Init(myFace2, aPnt, U2, V2);
                //
                if (!bReverse)
                {
                    aPntOn2Faces.SetP1(aPntOnF1);
                    aPntOn2Faces.SetP2(aPntOnF2);
                }
                else
                {
                    aPntOn2Faces.SetP2(aPntOnF1);
                    aPntOn2Faces.SetP1(aPntOnF2);
                }

                myPnts.Append(aPntOn2Faces);
            }
        }
    }

    /// <summary>
    /// Computes the valid tolerance of the intersection curves: the largest deviation between the 3D curve and its
    /// 2D curves on the faces, or, without a 2D curve, between the 3D curve and the face's surface.
    /// </summary>
    protected void ComputeTolReached3d(bool theToRunParallel)
    {
        int i, j, aNbLin = mySeqOfCurve.Length();
        if (aNbLin == 0)
        {
            return;
        }
        //
        // Minimal tangential tolerance for the curve
        double aTolFMax = Max(myTolF1, myTolF2);
        //
        Geom_Surface aS1 = myHS1.Surface()!;
        Geom_Surface aS2 = myHS2.Surface()!;
        //
        for (i = 1; i <= aNbLin; ++i)
        {
            ref IntTools_Curve aIC = ref mySeqOfCurve[i];
            Geom_Curve? aC3D = aIC.Curve();
            if (aC3D is null)
            {
                continue;
            }
            //
            double aTolC = aIC.Tolerance();
            double aFirst = aC3D.FirstParameter();
            double aLast = aC3D.LastParameter();
            //
            // Compute the tolerance for the curve
            Geom2d_Curve? aC2D1 = aIC.FirstCurve2d();
            Geom2d_Curve? aC2D2 = aIC.SecondCurve2d();
            //
            for (j = 0; j < 2; ++j)
            {
                Geom2d_Curve? aC2D = j == 0 ? aC2D1 : aC2D2;
                if (aC2D is not null)
                {
                    // Look for the maximal deviation between 3D and 2D curves
                    Geom_Surface aS = j == 0 ? aS1 : aS2;
                    if (IntTools_Tools.ComputeTolerance(aC3D, aC2D, aS, aFirst, aLast, out double aD, out _,
                                                        global::Kerf.Engine.Foundation.Precision.PConfusion(), theToRunParallel))
                    {
                        if (aD > aTolC)
                        {
                            aTolC = aD;
                        }
                    }
                }
                else
                {
                    // Look for the maximal deviation between 3D curve and surface
                    TopoDS_Face aF = j == 0 ? myFace1 : myFace2;
                    double aD = FindMaxDistance(aC3D, aFirst, aLast, aF, myContext!);
                    if (aD > aTolC)
                    {
                        aTolC = aD;
                    }
                }
            }
            // Set the valid tolerance for the curve
            aIC.SetTolerance(aTolC);
            //
            // Set the tangential tolerance for the curve.
            // Note, that, currently, computation of the tangential tolerance is
            // implemented for the Plane/Plane case only.
            // Thus, set the tangential tolerance equal to maximal tolerance of faces.
            if (aIC.TangentialTolerance() < aTolFMax)
            {
                aIC.SetTangentialTolerance(aTolFMax);
            }
        }
    }

    /// <summary>
    /// Post-processes the result lines: splits the closed 3D curves into parts (bToSplit), and rejects the extra
    /// lines of a plane/cone pair that gave four lines.
    /// </summary>
    public void PrepareLines3D(bool bToSplit = true)
    {
        int i, aNbCurves;
        SurfaceKind aType1, aType2;
        NCollection_Sequence<IntTools_Curve> aNewCvs = new();
        //
        // 1. Treatment closed  curves
        aNbCurves = mySeqOfCurve.Length();
        for (i = 1; i <= aNbCurves; ++i)
        {
            IntTools_Curve aIC = mySeqOfCurve.Value(i);
            //
            if (bToSplit)
            {
                int j, aNbC;
                NCollection_Sequence<IntTools_Curve> aSeqCvs = new();
                //
                aNbC = IntTools_Tools.SplitCurve(aIC, aSeqCvs);
                if (aNbC != 0)
                {
                    for (j = 1; j <= aNbC; ++j)
                    {
                        aNewCvs.Append(aSeqCvs.Value(j));
                    }
                }
                else
                {
                    aNewCvs.Append(aIC);
                }
            }
            else
            {
                aNewCvs.Append(aIC);
            }
        }
        //
        // 2. Plane\Cone intersection when we had 4 curves
        aType1 = myHS1.GetType();
        aType2 = myHS2.GetType();
        aNbCurves = aNewCvs.Length();
        //
        if ((aType1 == Plane && aType2 == SurfaceKind.Cone)
            || (aType2 == Plane && aType1 == SurfaceKind.Cone))
        {
            if (aNbCurves == 4)
            {
                CurveKind aCType1;
                //
                aCType1 = aNewCvs.Value(1).Type();
                if (aCType1 == CurveKind.Line)
                {
                    NCollection_Sequence<IntTools_Curve> aSeqIn = new(), aSeqOut = new();
                    //
                    for (i = 1; i <= aNbCurves; ++i)
                    {
                        aSeqIn.Append(aNewCvs.Value(i));
                    }
                    //
                    IntTools_Tools.RejectLines(aSeqIn, aSeqOut);
                    //
                    aNewCvs.Clear();
                    aNbCurves = aSeqOut.Length();
                    for (i = 1; i <= aNbCurves; ++i)
                    {
                        aNewCvs.Append(aSeqOut.Value(i));
                    }
                }
            }
        } // if ((aType1==GeomAbs_Plane && aType2==GeomAbs_Cone)...
        //
        // 3. Fill  mySeqOfCurve
        mySeqOfCurve.Clear();
        aNbCurves = aNewCvs.Length();
        for (i = 1; i <= aNbCurves; ++i)
        {
            mySeqOfCurve.Append(aNewCvs.Value(i));
        }
    }
}
