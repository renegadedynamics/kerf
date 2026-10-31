// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALineToWLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALineToWLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.IntPatch_SpecPntType;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

/// <summary>
/// Converts an analytic intersection line (IntPatch_ALine) of two quadrics into walking lines: points sampled along
/// the line with a step adapted to its curvature, the line's vertices inserted, split at poles and apexes.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. Its quadrics are copies (structs) built in the constructor from the surfaces.
/// </remarks>
public class IntPatch_ALineToWLine
{
    private readonly Adaptor3d_Surface myS1;
    private readonly Adaptor3d_Surface myS2;
    private IntSurf_Quadric myQuad1 = new();
    private IntSurf_Quadric myQuad2 = new();

    private readonly int myNbPointsInWline;
    private double myTolOpenDomain;
    private double myTolTransition;
    private double myTol3D;

    //=================================================================================================

    private static void AddPointIntoLine(IntSurf_LineOn2S theLine, ReadOnlySpan<double> theArrPeriods,
                                         ref IntSurf_PntOn2S thePoint)
    {
        if (theLine.NbPoints() > 0)
        {
            if (thePoint.IsSame(theLine.Value(theLine.NbPoints()), Precision.Confusion()))
            {
                return;
            }

            IntPatch_SpecialPoints.AdjustPointAndVertex(theLine.Value(theLine.NbPoints()), theArrPeriods,
                                                        ref thePoint);
        }

        theLine.Add(thePoint);
    }

    // OCCT's AddPointIntoLine with a non-null vertex pointer.
    private static void AddPointIntoLine(IntSurf_LineOn2S theLine, ReadOnlySpan<double> theArrPeriods,
                                         ref IntSurf_PntOn2S thePoint, ref IntPatch_Point theVertex)
    {
        if (theLine.NbPoints() > 0)
        {
            if (thePoint.IsSame(theLine.Value(theLine.NbPoints()), Precision.Confusion()))
            {
                return;
            }

            IntPatch_SpecialPoints.AdjustPointAndVertex(theLine.Value(theLine.NbPoints()), theArrPeriods,
                                                        ref thePoint, ref theVertex);
        }

        theLine.Add(thePoint);
    }

    //=======================================================================
    // function : AddVertexPoint
    // purpose  : Extracts IntSurf_PntOn2S from theVertex and adds result in theLine.
    //=======================================================================
    private static void AddVertexPoint(IntSurf_LineOn2S theLine, ref IntPatch_Point theVertex,
                                       ReadOnlySpan<double> theArrPeriods)
    {
        IntSurf_PntOn2S anApexPoint = theVertex.PntOn2S();
        AddPointIntoLine(theLine, theArrPeriods, ref anApexPoint, ref theVertex);
    }

    //=======================================================================
    // function : IsPoleOrSeam
    // purpose  : Processes theVertex depending on its type
    //            (pole/apex/point on boundary etc.) and adds it in theLine.
    //           thePIsoRef is the reference point using in case when the
    //            value of correspond parameter cannot be precise.
    //           theSingularSurfaceID contains the ID of surface with
    //            special point (0 - none, 1 - theS1, 2 - theS2)
    //=======================================================================
    private static IntPatch_SpecPntType IsPoleOrSeam(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                                     in IntSurf_PntOn2S thePIsoRef, IntSurf_LineOn2S theLine,
                                                     ref IntPatch_Point theVertex, ReadOnlySpan<double> theArrPeriods,
                                                     double theTol3d, out int theSingularSurfaceID)
    {
        theSingularSurfaceID = 0;

        for (int i = 0; i < 2; i++)
        {
            bool isReversed = (i > 0);
            GeomAbs_SurfaceType aType = isReversed ? theS2.GetType() : theS1.GetType();

            IntPatch_SpecPntType anAddedPType = IntPatch_SPntNone;
            IntSurf_PntOn2S anApexPoint = new();

            switch (aType)
            {
                case GeomAbs_Sphere:
                case GeomAbs_Cone:
                {
                    if (IntPatch_SpecialPoints.AddSingularPole((isReversed ? theS2 : theS1),
                                                               (isReversed ? theS1 : theS2), thePIsoRef,
                                                               ref theVertex, ref anApexPoint, isReversed, true))
                    {
                        anAddedPType = IntPatch_SPntPole;
                        break;
                    }
                }
                    goto case GeomAbs_Torus; // [[fallthrough]]
                case GeomAbs_Torus:
                    if (aType == GeomAbs_Torus)
                    {
                        if (IntPatch_SpecialPoints.AddCrossUVIsoPoint((isReversed ? theS2 : theS1),
                                                                      (isReversed ? theS1 : theS2), thePIsoRef,
                                                                      theTol3d, ref anApexPoint, isReversed))
                        {
                            anAddedPType = IntPatch_SPntSeamUV;
                            break;
                        }
                    }
                    goto case GeomAbs_Cylinder; // [[fallthrough]]
                case GeomAbs_Cylinder:
                    theSingularSurfaceID = i + 1;
                    AddVertexPoint(theLine, ref theVertex, theArrPeriods);
                    return IntPatch_SPntSeamU;

                default:
                    break;
            }

            if (anAddedPType != IntPatch_SPntNone)
            {
                theSingularSurfaceID = i + 1;
                AddPointIntoLine(theLine, theArrPeriods, ref anApexPoint, ref theVertex);
                return anAddedPType;
            }
        }

        return IntPatch_SPntNone;
    }

    //=================================================================================================

    public IntPatch_ALineToWLine(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2, int theNbPoints = 200)
    {
        myS1 = theS1;
        myS2 = theS2;
        myNbPointsInWline = theNbPoints;
        myTolOpenDomain = 1.0e-9;
        myTolTransition = 1.0e-8;
        myTol3D = Precision.Confusion();

        GeomAbs_SurfaceType aTyps1 = theS1.GetType();
        GeomAbs_SurfaceType aTyps2 = theS2.GetType();

        switch (aTyps1)
        {
            case GeomAbs_Plane:
                myQuad1.SetValue(theS1.Plane());
                break;

            case GeomAbs_Cylinder:
                myQuad1.SetValue(theS1.Cylinder());
                break;

            case GeomAbs_Sphere:
                myQuad1.SetValue(theS1.Sphere());
                break;

            case GeomAbs_Cone:
                myQuad1.SetValue(theS1.Cone());
                break;

            case GeomAbs_Torus:
                myQuad1.SetValue(theS1.Torus());
                break;

            default:
                break;
        }

        switch (aTyps2)
        {
            case GeomAbs_Plane:
                myQuad2.SetValue(theS2.Plane());
                break;
            case GeomAbs_Cylinder:
                myQuad2.SetValue(theS2.Cylinder());
                break;

            case GeomAbs_Sphere:
                myQuad2.SetValue(theS2.Sphere());
                break;

            case GeomAbs_Cone:
                myQuad2.SetValue(theS2.Cone());
                break;

            case GeomAbs_Torus:
                myQuad2.SetValue(theS2.Torus());
                break;

            default:
                break;
        }
    }

    /// <summary>Sets the 3D tolerance (the deflection of the walking line from the analytic one).</summary>
    public void SetTol3D(double aT) => myTol3D = aT;

    /// <summary>The 3D tolerance.</summary>
    public double Tol3D() => myTol3D;

    /// <summary>Sets the tolerance of the transitions' test.</summary>
    public void SetTolTransition(double aT) => myTolTransition = aT;

    /// <summary>The tolerance of the transitions' test.</summary>
    public double TolTransition() => myTolTransition;

    /// <summary>Sets the shift of the ends of an open domain.</summary>
    public void SetTolOpenDomain(double aT) => myTolOpenDomain = aT;

    /// <summary>The shift of the ends of an open domain.</summary>
    public double TolOpenDomain() => myTolOpenDomain;

    //=================================================================================================

    /// <summary>Corrects the parameters of a point at a pole or an apex by linear extrapolation from the two
    /// previous points.</summary>
    protected internal void CorrectEndPoint(IntSurf_LineOn2S theLine, int theIndex)
    {
        const double aTol = 1.0e-5;
        const double aSqTol = 1.0e-10;

        // Perform linear extrapolation from two previous points
        int anIndFirst, anIndSecond;
        if (theIndex == 1)
        {
            anIndFirst = 3;
            anIndSecond = 2;
        }
        else
        {
            anIndFirst = theIndex - 2;
            anIndSecond = theIndex - 1;
        }
        IntSurf_PntOn2S aPntOn2S = theLine.Value(theIndex);

        for (int ii = 1; ii <= 2; ii++)
        {
            bool anIsOnFirst = (ii == 1);

            IntSurf_Quadric aQuad = (ii == 1) ? myQuad1 : myQuad2;
            if (aQuad.TypeQuadric() == GeomAbs_Cone)
            {
                gp_Cone aCone = aQuad.Cone();
                gp_Pnt anApex = aCone.Apex();
                if (anApex.SquareDistance(aPntOn2S.Value()) > aSqTol)
                {
                    continue;
                }
            }
            else if (aQuad.TypeQuadric() == GeomAbs_Sphere)
            {
                aPntOn2S.ParametersOnSurface(anIsOnFirst, out double aU, out double aV);
                if (Math.Abs(aV - M_PI / 2) > aTol && Math.Abs(aV + M_PI / 2) > aTol)
                {
                    continue;
                }
            }
            else
            {
                continue;
            }

            gp_Pnt2d PrevPrevP2d = theLine.Value(anIndFirst).ValueOnSurface(anIsOnFirst);
            gp_Pnt2d PrevP2d = theLine.Value(anIndSecond).ValueOnSurface(anIsOnFirst);
            gp_Dir2d aDir = new(new gp_Vec2d(PrevPrevP2d, PrevP2d));
            double aX0 = PrevPrevP2d.X(), aY0 = PrevPrevP2d.Y();
            aPntOn2S.ParametersOnSurface(anIsOnFirst, out double aXend, out double aYend);

            if (Math.Abs(aDir.Y()) < gp.Resolution())
            {
                continue;
            }

            double aNewXend = aDir.X() / aDir.Y() * (aYend - aY0) + aX0;

            theLine.SetUV(theIndex, anIsOnFirst, aNewXend, aYend);
        }
    }

    //=================================================================================================

    /// <summary>The radius of the smallest section of a cone or a sphere through the point (RealLast if neither
    /// surface is one).</summary>
    protected internal double GetSectionRadius(in gp_Pnt thePnt3d)
    {
        double aRetVal = RealLast();
        for (int i = 0; i < 2; i++)
        {
            IntSurf_Quadric aQuad = (i != 0) ? myQuad2 : myQuad1;
            if (aQuad.TypeQuadric() == GeomAbs_Cone)
            {
                gp_Cone aCone = aQuad.Cone();
                gp_XYZ aRVec = thePnt3d.XYZ() - aCone.Apex().XYZ();
                gp_XYZ aDir = aCone.Axis().Direction().XYZ();

                aRetVal = Min(aRetVal, Math.Abs(aRVec.Dot(aDir) * Math.Tan(aCone.SemiAngle())));
            }
            else if (aQuad.TypeQuadric() == GeomAbs_Sphere)
            {
                gp_Sphere aSphere = aQuad.Sphere();
                gp_XYZ aRVec = thePnt3d.XYZ() - aSphere.Location().XYZ();
                gp_XYZ aDir = aSphere.Position().Direction().XYZ();
                double aR = aSphere.Radius();
                double aD = aRVec.Dot(aDir);
                double aDelta = aR * aR - aD * aD;
                if (aDelta <= 0.0)
                {
                    aRetVal = 0.0;
                    break;
                }
                else
                {
                    aRetVal = Min(aRetVal, Math.Sqrt(aDelta));
                }
            }
        }

        return aRetVal;
    }

    //=================================================================================================

    /// <summary>Converts the analytic line on its whole domain (an open end shifted by TolOpenDomain) into walking
    /// lines appended to theLines.</summary>
    public void MakeWLine(IntPatch_ALine theAline, NCollection_Sequence<IntPatch_Line> theLines)
    {
        double f = theAline.FirstParameter(out bool included);
        if (!included)
        {
            f += myTolOpenDomain;
        }
        double l = theAline.LastParameter(out included);
        if (!included)
        {
            l -= myTolOpenDomain;
        }

        MakeWLine(theAline, f, l, theLines);
    }

    //=================================================================================================

    /// <summary>Converts the analytic line between theFPar and theLPar into walking lines appended to
    /// theLines. The line's vertices' tolerances are unified where they mark the same point.</summary>
    public void MakeWLine(IntPatch_ALine theALine, double theFPar, double theLPar,
                          NCollection_Sequence<IntPatch_Line> theLines)
    {
        int aNbVert = theALine.NbVertex();
        if (aNbVert == 0)
        {
            return;
        }

        // The same points can be marked by different vertices.
        // The code below unifies tolerances of all vertices
        // marking the same points.
        for (int i = 1; i < aNbVert; i++)
        {
            ref IntPatch_Point aCurVert = ref theALine.ChangeVertex(i);
            IntSurf_PntOn2S aCurrPt = aCurVert.PntOn2S();
            double aCurToler = aCurVert.Tolerance();
            for (int j = i + 1; j <= aNbVert; j++)
            {
                ref IntPatch_Point aVert = ref theALine.ChangeVertex(j);
                IntSurf_PntOn2S aNewPt = aVert.PntOn2S();
                double aToler = aVert.Tolerance();

                double aSumTol = aCurToler + aToler;
                if (aCurrPt.IsSame(aNewPt, aSumTol))
                {
                    aCurVert.SetTolerance(aSumTol);
                    aVert.SetTolerance(aSumTol);
                }
            }
        }

        double aTol = 2.0 * myTol3D + Precision.Confusion();
        double aPrmTol = Max(1.0e-4 * (theLPar - theFPar), Precision.PConfusion());

        IntPatch_SpecPntType aPrePointExist = IntPatch_SPntNone;

        NCollection_Array1<double> aVertexParams = new(1, aNbVert);
        NCollection_Array1<IntPatch_Point> aSeqVertex = new(1, aNbVert);

        // It is possible to have several vertices with equal parameters.
        NCollection_Array1<bool> hasVertexBeenChecked = new(1, aNbVert);

        IntSurf_LineOn2S aLinOn2S;
        double aParameter = theFPar;

        for (int i = aVertexParams.Lower(); i <= aVertexParams.Upper(); i++)
        {
            IntPatch_Point aVert = theALine.Vertex(i);
            double aPar = aVert.ParameterOnLine();
            aVertexParams[i] = aPar;
            hasVertexBeenChecked[i] = false;
        }

        int aSingularSurfaceID = 0;
        Span<double> anArrPeriods = [0.0,  // U1
                                     0.0,  // V1
                                     0.0,  // U2
                                     0.0]; // V2

        IntSurf.SetPeriod(myS1, myS2, anArrPeriods);

        IntSurf_PntOn2S aPrevLPoint = new();

        while (aParameter < theLPar)
        {
            double aStep = (theLPar - aParameter) / (double)(myNbPointsInWline - 1);
            if (aStep < Epsilon(theLPar))
            {
                break;
            }

            bool isStepReduced = false;
            double aLPar = theLPar;

            for (int i = aVertexParams.Lower(); i <= aVertexParams.Upper(); i++)
            {
                if (hasVertexBeenChecked[i])
                {
                    continue;
                }

                aLPar = aVertexParams[i];
                if (Math.Abs(aLPar - aParameter) < aPrmTol)
                {
                    continue;
                }

                break;
            }

            if ((aStep - (aLPar - aParameter) > aPrmTol) && (Math.Abs(aLPar - aParameter) > aPrmTol))
            {
                aStep = Max((aLPar - aParameter) / 5, 1.0e-5);
                isStepReduced = true;
            }

            int aNewVertID = 0;
            aLinOn2S = new IntSurf_LineOn2S();
            bool anIsFirstDegenerated = false, anIsLastDegenerated = false;

            double aStepMin = 0.1 * aStep, aStepMax = 10.0 * aStep;

            bool isLast = false;
            double aPrevParam = aParameter;
            for (; !isLast; aParameter += aStep)
            {
                IntSurf_PntOn2S aPOn2S = new();

                if (theLPar <= aParameter)
                {
                    isLast = true;
                    if (aPrePointExist != IntPatch_SPntNone)
                    {
                        break;
                    }
                    else
                    {
                        aParameter = theLPar;
                    }
                }

                bool isPointValid = false;
                double aTgMagn = 0.0;
                {
                    theALine.D1(aParameter, out gp_Pnt aPnt3d, out gp_Vec aTg);
                    if (GetSectionRadius(aPnt3d) < 5.0e-6)
                    {
                        // We cannot compute 2D-parameters of
                        // aPOn2S correctly.

                        if (anIsLastDegenerated)
                        { // the current last point is wrong
                            aLinOn2S.RemovePoint(aLinOn2S.NbPoints());
                        }

                        isPointValid = false;
                    }
                    else
                    {
                        isPointValid = true;
                    }

                    aTgMagn = aTg.Magnitude();
                    double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                    myQuad1.Parameters(aPnt3d, ref u1, ref v1);
                    myQuad2.Parameters(aPnt3d, ref u2, ref v2);
                    aPOn2S.SetValue(aPnt3d, u1, v1, u2, v2);
                }

                if (aPrePointExist != IntPatch_SPntNone)
                {
                    double aURes = Max(myS1.UResolution(myTol3D), myS2.UResolution(myTol3D)),
                           aVRes = Max(myS1.VResolution(myTol3D), myS2.VResolution(myTol3D));

                    double aTol2d = (aPrePointExist == IntPatch_SPntPole) ? -1.0
                                    : (aPrePointExist == IntPatch_SPntSeamV) ? aVRes
                                    : (aPrePointExist == IntPatch_SPntSeamUV) ? Max(aURes, aVRes)
                                                                              : aURes;

                    IntSurf_PntOn2S aRPT = aPOn2S;

                    if (aPrePointExist == IntPatch_SPntPole)
                    {
                        double aPrt = 0.5 * (aPrevParam + theLPar);
                        for (int i = aVertexParams.Lower(); i <= aVertexParams.Upper(); i++)
                        {
                            double aParam = aVertexParams[i];

                            if (aParam <= aPrevParam)
                            {
                                continue;
                            }

                            if ((aParam - aPrevParam) < aPrmTol)
                            {
                                gp_Pnt aPnt3dV = theALine.Value(aParam);
                                if (aPOn2S.Value().SquareDistance(aPnt3dV) < Precision.SquareConfusion())
                                {
                                    // i-th vertex is the same as a Pole/Apex.
                                    // So, it should be ignored.
                                    continue;
                                }
                            }

                            aPrt = 0.5 * (aParam + aPrevParam);
                            break;
                        }

                        gp_Pnt aPnt3d = theALine.Value(aPrt);
                        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                        myQuad1.Parameters(aPnt3d, ref u1, ref v1);
                        myQuad2.Parameters(aPnt3d, ref u2, ref v2);
                        aRPT.SetValue(aPnt3d, u1, v1, u2, v2);

                        if (aPOn2S.IsSame(aPrevLPoint, Max(Precision.Approximation(), aTol)))
                        {
                            // Set V-parameter as precise value found on the previous step.
                            if (aSingularSurfaceID == 1)
                            {
                                aPOn2S.ParametersOnS1(out u2, out v2);
                                aPOn2S.SetValue(true, u1, v2);
                            }
                            else // if (aSingularSurfaceID == 2)
                            {
                                aPOn2S.ParametersOnS2(out u1, out v1);
                                aPOn2S.SetValue(false, u2, v1);
                            }
                        }
                    }

                    if (IntPatch_SpecialPoints.ContinueAfterSpecialPoint(myS1, myS2, aRPT, aPrePointExist, aTol2d,
                                                                         ref aPrevLPoint, false))
                    {
                        AddPointIntoLine(aLinOn2S, anArrPeriods, ref aPrevLPoint);
                    }
                    else if (aParameter == theLPar)
                    { // Strictly equal!!!
                        break;
                    }
                    else if (aParameter + aStep < theLPar)
                    {
                        // Prediction of the next point
                        theALine.D1(aParameter + aStep, out gp_Pnt aPnt3dNext, out _);
                        double anU1 = 0.0, aV1 = 0.0, anU2 = 0.0, aV2 = 0.0;
                        myQuad1.Parameters(aPnt3dNext, ref anU1, ref aV1);
                        myQuad2.Parameters(aPnt3dNext, ref anU2, ref aV2);
                        IntSurf_PntOn2S aPOn2SNext = new();
                        aPOn2SNext.SetValue(aPnt3dNext, anU1, aV1, anU2, aV2);

                        if (aPOn2SNext.ValueOnSurface(false).SquareDistance(aRPT.ValueOnSurface(false)) > M_PI * M_PI
                            || aPOn2SNext.ValueOnSurface(true).SquareDistance(aRPT.ValueOnSurface(true)) > M_PI * M_PI)
                        {
                            aPrevLPoint = aRPT;
                            aPrevParam = aParameter;
                            continue;
                        }
                    }
                }

                aPrePointExist = IntPatch_SPntNone;

                int aVertexNumber = -1;
                for (int i = aVertexParams.Lower(); i <= aVertexParams.Upper(); i++)
                {
                    if (hasVertexBeenChecked[i])
                    {
                        continue;
                    }

                    IntPatch_Point aVP = theALine.Vertex(i);
                    double aParam = aVertexParams[i];
                    if (((aPrevParam < aParam) && (aParam <= aParameter))
                        || ((aPrevParam == aParameter) && (aParam == aParameter))
                        || (aPOn2S.IsSame(aVP.PntOn2S(), aVP.Tolerance())
                            && (Math.Abs(aVP.ParameterOnLine() - aParameter) < aPrmTol)))
                    {
                        // We have either jumped over the vertex or "fell" on the vertex.
                        // However, ALine can be self-interfered. Therefore, we need to check
                        // vertex parameter and 3D-distance together.

                        aVertexNumber = i;
                        break;
                    }
                }

                aPrevParam = aParameter;

                if (aVertexNumber < 0)
                {
                    if (isPointValid)
                    {
                        if (!isStepReduced)
                        {
                            StepComputing(theALine, aPOn2S, theLPar, aParameter, aTgMagn, aStepMin, aStepMax, myTol3D,
                                          ref aStep);
                        }

                        AddPointIntoLine(aLinOn2S, anArrPeriods, ref aPOn2S);
                        aPrevLPoint = aPOn2S;
                    }
                    else
                    {
                        // add point, set corresponding status: to be corrected later
                        bool ToAdd = false;
                        if (aLinOn2S.NbPoints() == 0)
                        {
                            anIsFirstDegenerated = true;
                            ToAdd = true;
                        }
                        else if (aLinOn2S.NbPoints() > 1)
                        {
                            anIsLastDegenerated = true;
                            ToAdd = true;
                        }

                        if (ToAdd)
                        {
                            AddPointIntoLine(aLinOn2S, anArrPeriods, ref aPOn2S);
                            aPrevLPoint = aPOn2S;
                        }
                    }

                    continue;
                }

                IntPatch_Point aVtx = theALine.Vertex(aVertexNumber);
                double aNewVertexParam = aLinOn2S.NbPoints() + 1;
                int aNbPointsPrev = aLinOn2S.NbPoints();

                // ATTENTION!!!
                //  IsPoleOrSeam inserts new point in aLinOn2S if aVtx respects
                // to some special point. Otherwise, aLinOn2S is not changed.

                // Find a point for reference parameter. It will be used
                // if real parameter value cannot be precise (see comment to
                // IsPoleOrSeam(...) function).
                IntSurf_PntOn2S aPrefIso = aVtx.PntOn2S();
                if (aLinOn2S.NbPoints() < 1)
                {
                    for (int i = aVertexNumber + 1; i <= aVertexParams.Upper(); i++)
                    {
                        double aParam = aVertexParams[i];
                        if ((aParam - aVertexParams[aVertexNumber]) > Precision.PConfusion())
                        {
                            double aPrm = 0.5 * (aParam + aVertexParams[aVertexNumber]);
                            gp_Pnt aPnt3d = theALine.Value(aPrm);
                            double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                            myQuad1.Parameters(aPnt3d, ref u1, ref v1);
                            myQuad2.Parameters(aPnt3d, ref u2, ref v2);
                            aPrefIso.SetValue(aPnt3d, u1, v1, u2, v2);
                            break;
                        }
                    }
                }
                else
                {
                    aPrefIso = aLinOn2S.Value(aLinOn2S.NbPoints());
                }

                aPrePointExist = IsPoleOrSeam(myS1, myS2, aPrefIso, aLinOn2S, ref aVtx, anArrPeriods, aTol,
                                              out aSingularSurfaceID);

                if (aPrePointExist == IntPatch_SPntPole || aPrePointExist == IntPatch_SPntPoleSeamU)
                {
                    // set corresponding status: to be corrected later
                    if (aLinOn2S.NbPoints() == 1)
                    {
                        anIsFirstDegenerated = true;
                    }
                    else
                    {
                        anIsLastDegenerated = true;
                    }
                }

                double aCurVertParam = aVtx.ParameterOnLine();
                if (aPrePointExist != IntPatch_SPntNone)
                {
                    if (aNbPointsPrev == aLinOn2S.NbPoints())
                    {
                        // Vertex coincides any point of line and was not added into line
                        aNewVertexParam = aNbPointsPrev;
                    }
                    aPrevParam = aParameter = aCurVertParam;
                }
                else
                {
                    if (!isPointValid)
                    {
                        // Take a farther point of ALine (with greater parameter)
                        continue;
                    }

                    if (aVtx.Tolerance() > aTol)
                    {
                        aVtx.SetValue(aPOn2S);
                        AddPointIntoLine(aLinOn2S, anArrPeriods, ref aPOn2S);
                    }
                    else
                    {
                        AddVertexPoint(aLinOn2S, ref aVtx, anArrPeriods);
                    }
                }

                aPrevLPoint = aPOn2S = aLinOn2S.Value(aLinOn2S.NbPoints());

                {
                    bool isFound = false;
                    double aSqTol = aTol * aTol;
                    gp_Pnt aP1 = theALine.Value(aCurVertParam);
                    IntSurf_PntOn2S aVertP2S = aVtx.PntOn2S();
                    double aVertToler = aVtx.Tolerance();

                    for (int i = aVertexParams.Lower(); i <= aVertexParams.Upper(); i++)
                    {
                        if (hasVertexBeenChecked[i])
                        {
                            continue;
                        }

                        gp_Pnt aP2 = theALine.Value(aVertexParams[i]);

                        if (aP1.SquareDistance(aP2) < aSqTol)
                        {
                            IntPatch_Point aLVtx = theALine.Vertex(i);
                            aLVtx.SetValue(aVertP2S);
                            aLVtx.SetTolerance(aVertToler);
                            double aParam = aLVtx.ParameterOnLine();
                            if (Math.Abs(aParam - theLPar) <= Precision.PConfusion())
                            {                           // in the case of closed curve,
                                aLVtx.SetParameter(-1); // we don't know yet the number of points in the curve
                            }
                            else
                            {
                                aLVtx.SetParameter(aNewVertexParam);
                            }
                            aSeqVertex[++aNewVertID] = aLVtx;
                            hasVertexBeenChecked[i] = true;
                            isFound = true;
                        }
                        else if (isFound)
                        {
                            break;
                        }
                    }
                }

                if ((aPrePointExist != IntPatch_SPntNone) && (aLinOn2S.NbPoints() > 1))
                {
                    break;
                }

                if (isStepReduced)
                {
                    isStepReduced = false;

                    aStep = (theLPar - aParameter) / (double)(myNbPointsInWline - 1);
                    if (aStep < Epsilon(theLPar))
                    {
                        break;
                    }

                    aLPar = aVertexParams[aNbVert];
                    for (int i = aVertexParams.Lower(); i <= aVertexParams.Upper(); i++)
                    {
                        if (hasVertexBeenChecked[i])
                        {
                            continue;
                        }

                        aLPar = aVertexParams[i];
                        if (Math.Abs(aLPar - aParameter) < aPrmTol)
                        {
                            continue;
                        }

                        break;
                    }

                    if ((aStep - (aLPar - aParameter) > aPrmTol) && (Math.Abs(aLPar - aParameter) > aPrmTol))
                    {
                        aStep = Max((aLPar - aParameter) / 5, 1.0e-5);
                        isStepReduced = true;
                    }

                    aStepMin = 0.1 * aStep;
                    aStepMax = 10.0 * aStep;
                }
            } // for(; !isLast; aParameter += aStep)

            if (aLinOn2S.NbPoints() < 2)
            {
                aParameter += aStep;
                continue;
            }

            // Correct first and last points if needed
            if (aLinOn2S.NbPoints() >= 3)
            {
                if (anIsFirstDegenerated)
                {
                    CorrectEndPoint(aLinOn2S, 1);
                }
                if (anIsLastDegenerated)
                {
                    CorrectEndPoint(aLinOn2S, aLinOn2S.NbPoints());
                }
            }

            //-----------------------------------------------------------------
            //--              W  L  i  n  e     c  r  e  a  t  i  o  n      ---
            //-----------------------------------------------------------------
            IntPatch_WLine aWLine;
            //
            if (theALine.TransitionOnS1() == IntSurf_Touch)
            {
                aWLine = new IntPatch_WLine(aLinOn2S, theALine.IsTangent(), theALine.SituationS1(),
                                            theALine.SituationS2());
                aWLine.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpImp);
            }
            else if (theALine.TransitionOnS1() == IntSurf_Undecided)
            {
                aWLine = new IntPatch_WLine(aLinOn2S, theALine.IsTangent());
                aWLine.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpImp);
            }
            else
            {
                // Computation of transitions of the line on two surfaces    ---
                int indice1 = Max(aLinOn2S.NbPoints() / 3, 2);
                gp_Pnt aPP0 = aLinOn2S.Value(indice1 - 1).Value(),
                       aPP1 = aLinOn2S.Value(indice1).Value();
                gp_Vec tgvalid = new(aPP0, aPP1);
                gp_Vec aNQ1 = myQuad1.Normale(aPP0), aNQ2 = myQuad2.Normale(aPP0);

                double dotcross = tgvalid.DotCross(aNQ2, aNQ1);

                IntSurf_TypeTrans trans1 = IntSurf_Undecided, trans2 = IntSurf_Undecided;

                if (dotcross > myTolTransition)
                {
                    trans1 = IntSurf_Out;
                    trans2 = IntSurf_In;
                }
                else if (dotcross < -myTolTransition)
                {
                    trans1 = IntSurf_In;
                    trans2 = IntSurf_Out;
                }

                aWLine = new IntPatch_WLine(aLinOn2S, theALine.IsTangent(), trans1, trans2);
                aWLine.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpImp);
            }

            for (int i = aSeqVertex.Lower(); i <= aNewVertID; i++)
            {
                IntPatch_Point aVtx = aSeqVertex[i];
                if (aVtx.ParameterOnLine() == -1)
                {                                       // in the case of closed curve,
                    aVtx.SetParameter(aWLine.NbPnts()); // we set the last parameter
                }
                aWLine.AddVertex(aVtx);
            }

            aWLine.SetPeriod(anArrPeriods[0], anArrPeriods[1], anArrPeriods[2], anArrPeriods[3]);

            // the method ComputeVertexParameters can reduce the number of points in <aWLine>
            aWLine.ComputeVertexParameters(myTol3D);

            if (aWLine.NbPnts() > 1)
            {
                aWLine.EnablePurging(false);
                theLines.Append(aWLine);
            }
        } // while(aParameter < theLPar)
    }

    //=======================================================================
    // function : CheckDeflection
    // purpose  : Returns:
    //            -1 - step is too small
    //            0  - step is normal
    //            +1 - step is too big
    //=======================================================================
    protected internal int CheckDeflection(in gp_XYZ theMidPt, double theMaxDeflection)
    {
        double aDist = Math.Abs(myQuad1.Distance(new gp_Pnt(theMidPt)));
        if (aDist > theMaxDeflection)
        {
            return 1;
        }

        aDist = Max(Math.Abs(myQuad2.Distance(new gp_Pnt(theMidPt))), aDist);

        if (aDist > theMaxDeflection)
        {
            return 1;
        }

        if ((aDist + aDist) < theMaxDeflection)
        {
            return -1;
        }

        return 0;
    }

    //=================================================================================================

    /// <summary>Computes the step along the analytic line from the osculating circle at the point, refined by
    /// bisection against the deflection; false if the step couldn't be computed (theStep unchanged then).</summary>
    protected internal bool StepComputing(IntPatch_ALine theALine, in IntSurf_PntOn2S thePOn2S, double theLastParOfAline,
                                          double theCurParam, double theTgMagnitude, double theStepMin,
                                          double theStepMax, double theMaxDeflection, ref double theStep)
    {
        if (theTgMagnitude < Precision.Confusion())
        {
            return false;
        }

        double anEps = myTol3D;

        // Indeed, 1.0e+15 < 2^50 < 1.0e+16. Therefore,
        // if we apply bisection method to the range with length
        // 1.0e+6 then we will be able to find solution with max error ~1.0e-9.
        const int aNbIterMax = 50;

        double aNotFilledRange = theLastParOfAline - theCurParam;
        double aMinStep = theStepMin, aMaxStep = Min(theStepMax, aNotFilledRange);

        if (aMinStep > aMaxStep)
        {
            theStep = aMaxStep;
            return true;
        }

        double aR = IntPatch_PointLine.CurvatureRadiusOfIntersLine(myS1, myS2, thePOn2S);

        if (aR < 0.0)
        {
            return false;
        }
        else
        {
            // The 3D-step is defined as length of the tangent to the osculating circle
            // by the condition that the distance from end point of the tangent to the
            // circle is no greater than anEps. theStep is the step in
            // parameter space of intersection curve (must be converted from 3D-step).

            theStep = Min(Math.Sqrt(anEps * (2.0 * aR + anEps)) / theTgMagnitude, aMaxStep);
            theStep = Max(theStep, aMinStep);
        }

        // The step value has been computed for osculating circle.
        // Now it should be checked for real intersection curve
        // and is made more precise in case of necessity.

        int aNbIter = 0;
        do
        {
            aNbIter++;

            gp_XYZ aP1 = thePOn2S.Value().XYZ();
            gp_XYZ aP2 = theALine.Value(theCurParam + theStep).XYZ();
            int aStatus = CheckDeflection(0.5 * (aP1 + aP2), theMaxDeflection);

            if (aStatus == 0)
            {
                break;
            }

            if (aStatus < 0)
            {
                aMinStep = theStep;
            }
            else // if(aStatus > 0)
            {
                aMaxStep = theStep;
            }

            theStep = 0.5 * (aMinStep + aMaxStep);
        } while (((aMaxStep - aMinStep) > Precision.PConfusion()) && (aNbIter <= aNbIterMax));

        return aNbIter <= aNbIterMax;
    }
}
