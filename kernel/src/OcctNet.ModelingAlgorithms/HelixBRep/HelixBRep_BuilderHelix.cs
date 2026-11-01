// Port of: src/ModelingAlgorithms/TKHelix/HelixBRep/HelixBRep_BuilderHelix.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixBRep/HelixBRep_BuilderHelix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Builds a helix wire: a composite helix of several parts, each with its own height, pitch (or number of turns)
/// and start and end diameters, about an axis.
/// </summary>
/// <remarks>
/// <para>Error status: 0 OK; 1 just initialized (no parameters set); 2 the approximation failed; 3 an edge could not
/// be made; 10 the start point is too close to the axis; 11 the pitch is below the tolerance; 12 the height is below
/// the tolerance; 13 the taper angle is above π/2 - 1e-7 (OCCT documents negative angles as errors too, but doesn't
/// check them: occt-issues.md, unit 7D). Warning status: 0 OK; 1 the tolerance reached by the
/// approximation is above the requested one.</para>
/// <para>A working object (decision 21). <see cref="Shape"/> is a publication point: it freezes the wire.</para>
/// </remarks>
public class HelixBRep_BuilderHelix
{
    protected gp_Ax3 myAxis3 = new();
    protected NCollection_HArray1<double>? myDiams;
    protected NCollection_HArray1<double>? myHeights;
    protected NCollection_HArray1<double>? myPitches;
    protected NCollection_HArray1<bool>? myIsPitches;
    protected double myTolerance;
    protected double myTolReached;
    protected GeomAbs_Shape myContinuity;
    protected int myMaxDegree;
    protected int myMaxSegments;
    protected int myErrorStatus;
    protected int myWarningStatus;
    protected TopoDS_Shape myShape = new();
    private int myNParts;

    /// <summary>A builder with no parameters (error status 1), about the global Z axis.</summary>
    public HelixBRep_BuilderHelix()
    {
        gp_Pnt aP0 = new(0.0, 0.0, 0);
        // Initialize coordinate system and data arrays
        myAxis3.SetDirection(gp.DZ());
        myAxis3.SetLocation(aP0);
        myDiams = null;
        myHeights = null;
        myPitches = null;
        myIsPitches = null;
        myNParts = 1;
        myShape.Nullify();

        // Initialize approximation parameters
        myTolerance = 0.0001;
        myContinuity = GeomAbs_Shape.GeomAbs_C1;
        myMaxDegree = 8;
        myMaxSegments = 1000;
        // Initialize status variables
        myTolReached = 99.0;
        // Set initial error state
        myErrorStatus = 1;
        myWarningStatus = 1;
    }

    /// <summary>
    /// Sets the parameters of a general composite helix: np + 1 diameters, and np heights, pitches and flags (true:
    /// the pitch is a pitch; false: it is a number of turns).
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The arrays' lengths don't match.</exception>
    public void SetParameters(in gp_Ax3 theAxis,
                              NCollection_Array1<double> theDiams,
                              NCollection_Array1<double> theHeights,
                              NCollection_Array1<double> thePitches,
                              NCollection_Array1<bool> bIsPitches)
    {
        myNParts = theDiams.Length() - 1;

        myAxis3 = theAxis;

        myDiams = null;
        myHeights = null;
        myPitches = null;
        myShape.Nullify();

        myErrorStatus = 1;
        myWarningStatus = 1;

        if (myNParts != theHeights.Length() || myNParts != thePitches.Length()
            || myNParts != bIsPitches.Length())
        {
            throw new Standard_ConstructionError(
                "HelixBRep_BuilderHelix::SetParameters: wrong array dimension");
        }

        myDiams = new NCollection_HArray1<double>(1, myNParts + 1);
        myHeights = new NCollection_HArray1<double>(1, myNParts);
        myPitches = new NCollection_HArray1<double>(1, myNParts);
        myIsPitches = new NCollection_HArray1<bool>(1, myNParts);

        myDiams.ChangeArray1().Assign(theDiams);
        myHeights.ChangeArray1().Assign(theHeights);
        myPitches.ChangeArray1().Assign(thePitches);
        myIsPitches.ChangeArray1().Assign(bIsPitches);

        myErrorStatus = 0;
        myWarningStatus = 0;
    }

    /// <summary>Sets the parameters of a pure (cylindrical) helix of diameter <paramref name="theDiam"/>.</summary>
    /// <exception cref="Standard_ConstructionError">The arrays' lengths don't match.</exception>
    public void SetParameters(in gp_Ax3 theAxis,
                              double theDiam,
                              NCollection_Array1<double> theHeights,
                              NCollection_Array1<double> thePitches,
                              NCollection_Array1<bool> bIsPitches)
    {
        int aNbParts = theHeights.Length();
        NCollection_Array1<double> aDiams = new(1, aNbParts + 1);
        aDiams.Init(theDiam);
        SetParameters(theAxis, aDiams, theHeights, thePitches, bIsPitches);
    }

    /// <summary>
    /// Sets the parameters of a pure spiral: the diameter changes linearly with the height from
    /// <paramref name="theDiam1"/> to <paramref name="theDiam2"/>.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The arrays' lengths don't match.</exception>
    public void SetParameters(in gp_Ax3 theAxis,
                              double theDiam1,
                              double theDiam2,
                              NCollection_Array1<double> theHeights,
                              NCollection_Array1<double> thePitches,
                              NCollection_Array1<bool> bIsPitches)
    {
        int aNbParts = theHeights.Length();
        NCollection_Array1<double> aDiams = new(1, aNbParts + 1);

        int i, j;
        double anH = 0.0;
        for (i = theHeights.Lower(); i <= theHeights.Upper(); ++i)
        {
            anH += theHeights[i];
        }
        double K = (theDiam2 - theDiam1) / anH;
        aDiams[1] = theDiam1;
        aDiams[aNbParts + 1] = theDiam2;

        // OCCT: theHeights(1), not theHeights(Lower()): an array not starting at 1 is read wrongly or throws.
        anH = theHeights[1];
        for (i = theHeights.Lower() + 1, j = 2; i <= theHeights.Upper(); ++i, ++j)
        {
            aDiams[j] = theDiam1 + K * anH;
            anH += theHeights[i];
        }

        SetParameters(theAxis, aDiams, theHeights, thePitches, bIsPitches);
    }

    /// <summary>
    /// Sets the parameters of a general composite helix by pitches and numbers of turns: np + 1 diameters, np pitches
    /// and np numbers of turns.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The arrays' lengths don't match.</exception>
    public void SetParameters(in gp_Ax3 theAxis,
                              NCollection_Array1<double> theDiams,
                              NCollection_Array1<double> thePitches,
                              NCollection_Array1<double> theNbTurns)
    {
        int aNbParts = theDiams.Length() - 1;

        if (aNbParts != thePitches.Length() || aNbParts != theNbTurns.Length())
        {
            throw new Standard_ConstructionError(
                "HelixBRep_BuilderHelix::SetParameters: wrong array dimension");
        }

        NCollection_Array1<double> aHeights = new(1, aNbParts);
        NCollection_Array1<bool> bIsPitches = new(1, aNbParts);
        bIsPitches.Init(true);
        int i, ip, @in;
        for (i = 1, ip = thePitches.Lower(), @in = theNbTurns.Lower(); i <= thePitches.Length();
             ++i, ip++, @in++)
        {
            aHeights[i] = thePitches[ip] * theNbTurns[@in];
        }

        SetParameters(theAxis, theDiams, aHeights, thePitches, bIsPitches);
    }

    /// <summary>Sets the parameters of a pure helix by pitches and numbers of turns.</summary>
    /// <exception cref="Standard_ConstructionError">The arrays' lengths don't match.</exception>
    public void SetParameters(in gp_Ax3 theAxis,
                              double theDiam,
                              NCollection_Array1<double> thePitches,
                              NCollection_Array1<double> theNbTurns)
    {
        int aNbParts = thePitches.Length();

        if (aNbParts != theNbTurns.Length())
        {
            throw new Standard_ConstructionError(
                "HelixBRep_BuilderHelix::SetParameters: wrong array dimension");
        }

        NCollection_Array1<double> aHeights = new(1, aNbParts);
        NCollection_Array1<bool> bIsPitches = new(1, aNbParts);
        bIsPitches.Init(true);
        int i, ip, @in;
        for (i = 1, ip = thePitches.Lower(), @in = theNbTurns.Lower(); i <= thePitches.Length();
             ++i, ip++, @in++)
        {
            aHeights[i] = thePitches[ip] * theNbTurns[@in];
        }

        SetParameters(theAxis, theDiam, aHeights, thePitches, bIsPitches);
    }

    /// <summary>Sets the parameters of a pure spiral by pitches and numbers of turns.</summary>
    /// <exception cref="Standard_ConstructionError">The arrays' lengths don't match.</exception>
    public void SetParameters(in gp_Ax3 theAxis,
                              double theDiam1,
                              double theDiam2,
                              NCollection_Array1<double> thePitches,
                              NCollection_Array1<double> theNbTurns)
    {
        int aNbParts = thePitches.Length();

        if (aNbParts != theNbTurns.Length())
        {
            throw new Standard_ConstructionError(
                "HelixBRep_BuilderHelix::SetParameters: wrong array dimension");
        }

        NCollection_Array1<double> aHeights = new(1, aNbParts);
        NCollection_Array1<bool> bIsPitches = new(1, aNbParts);
        bIsPitches.Init(true);
        int i, ip, @in;
        for (i = 1, ip = thePitches.Lower(), @in = theNbTurns.Lower(); i <= thePitches.Length();
             ++i, ip++, @in++)
        {
            aHeights[i] = thePitches[ip] * theNbTurns[@in];
        }

        SetParameters(theAxis, theDiam1, theDiam2, aHeights, thePitches, bIsPitches);
    }

    /// <summary>Sets the approximation's tolerance, maximum degree and continuity.</summary>
    public void SetApproxParameters(double aTolerance, int aMaxDegree, GeomAbs_Shape aCont)
    {
        myTolerance = aTolerance;
        myMaxDegree = aMaxDegree;
        myContinuity = aCont;
    }

    /// <summary>The tolerance reached by the approximation.</summary>
    public double ToleranceReached() => myTolReached;

    /// <summary>Builds the helix wire.</summary>
    public void Perform()
    {
        if (myErrorStatus != 0)
        {
            return;
        }

        int i;

        myTolReached = 0.0;
        myShape.Nullify();
        BRep_Builder aBB = new();
        // aBB.MakeCompound(TopoDS::Compound(myShape));

        NCollection_List<TopoDS_Shape> anLst = new();

        gp_Ax1 anAxis = myAxis3.Axis();
        gp_Pnt aPStart = myAxis3.Location();
        aPStart.Translate(.5 * myDiams!.Value(1) * new gp_Vec(myAxis3.XDirection()));
        bool bIsClockwise = myAxis3.Direct();

        double aHeight;
        double aPitch;
        double aTaperAngle;

        for (i = 1; i <= myNParts; ++i)
        {
            aHeight = myHeights!.Value(i);

            if (myIsPitches!.Value(i))
            {
                aPitch = myPitches!.Value(i);
            }
            else
            {
                aPitch = aHeight / myPitches!.Value(i);
            }

            aTaperAngle = Math.Atan(.5 * (myDiams.Value(i + 1) - myDiams.Value(i)) / aHeight);

            TopoDS_Wire aPart = new();
            aBB.MakeWire(ref aPart);

            BuildPart(anAxis, aPStart, aHeight, aPitch, aTaperAngle, bIsClockwise, ref aPart);
            if (myErrorStatus != 0)
            {
                return;
            }

            TopExp.Vertices(aPart, out TopoDS_Vertex V1, out TopoDS_Vertex V2);

            aPStart = BRep_Tool.Pnt(V2);

            anAxis.SetLocation(anAxis.Location().Translated(aHeight * new gp_Vec(anAxis.Direction())));

            anLst.Append(aPart);
            // aBB.Add(myShape, aPart);
        }

        Smoothing(anLst);

        NCollection_List<TopoDS_Shape>.Iterator anIt = new(anLst);
        BRepBuilderAPI_MakeWire aMkWire = new(TopoDS.Wire(anLst.First()));
        anIt.Next();
        for (; anIt.More(); anIt.Next())
        {
            aMkWire.Add(TopoDS.Wire(anIt.Value()));
        }

        myShape = aMkWire.InternalShape();
    }

    private void BuildPart(in gp_Ax1 theAxis,
                           in gp_Pnt thePStart,
                           double theHeight,
                           double thePitch,
                           double theTaperAngle,
                           bool bIsClockwise,
                           ref TopoDS_Wire thePart)
    {
        if (myErrorStatus != 0)
        {
            return;
        }

        myErrorStatus = 0;
        myWarningStatus = 0;

        // 1. check & prepare data
        double aTolPrec, aDist, aDM, aTwoPI, aC1, aT2, aT1, aT0;
        // Initialize tolerance and angular precision
        aTolPrec = myTolerance;
        double aTolAng = 1.0e-7;
        // Validate input parameters
        if (theTaperAngle > M_PI / 2.0 - aTolAng)
        {
            myErrorStatus = 13; // invalid TaperAngle value
            return;
        }
        if (theHeight < aTolPrec)
        {
            myErrorStatus = 12; // invalid Height value
            return;
        }
        if (thePitch < aTolPrec)
        {
            myErrorStatus = 11; // invalid Pitch value
            return;
        }
        //
        gp_Lin aLin = new(theAxis);
        aDist = aLin.Distance(thePStart);
        if (aDist < aTolPrec)
        {
            myErrorStatus = 10; // myPStart belongs to the myAxis
            return;
        }

        aTolAng = aTolPrec / aDist;
        double aAngleStart = 0.0;

        // const bool bIsOutWard = theTaperAngle > 0.0;
        gp_Dir aDir = theAxis.Direction();
        gp_Vec aVec1 = new(aDir);
        gp_Pnt aM0 = theAxis.Location();
        gp_Vec aVec = new(aM0, thePStart);
        aDM = aVec1.Dot(aVec);
        gp_Pnt aM1 = aM0.Translated(aDM * aVec1);
        gp_Vec aVecX = new(aM1, thePStart);
        gp_Dir aDirX = new(aVecX);
        gp_Ax2 aAx2 = new(aM1, aDir, aDirX);
        //
        aTwoPI = 2.0 * M_PI;
        aC1 = thePitch / aTwoPI;
        aT0 = 0.0;
        aT1 = aAngleStart;

        aT2 = theHeight / aC1;
        //
        // 2. compute
        bool bIsDone;
        int iErr, aNbC, i;
        HelixGeom_BuilderHelix aBH = new();
        gp_Pnt aP1, aP2;
        BRep_Builder aBB = new();
        BRepBuilderAPI_MakeEdge aBME = new();
        TopoDS_Vertex aV1 = new(), aV2 = new();
        // TopoDS_Wire aW;
        TopoDS_Edge aE;
        //
        aBH.SetPosition(aAx2);
        aBH.SetCurveParameters(aT0, aT2, thePitch, aDist, theTaperAngle, bIsClockwise);
        aBH.SetTolerance(myTolerance);
        aBH.SetApproxParameters(myContinuity, myMaxDegree, myMaxSegments);
        //
        aBH.Perform();
        iErr = aBH.ErrorStatus();
        if (iErr != 0)
        {
            myErrorStatus = 2;
            return;
        }
        //
        aBB.MakeWire(ref thePart);
        //
        myTolReached = Math.Max(myTolReached, aBH.ToleranceReached());
        NCollection_Sequence<Geom_Curve> aSC = new();
        aSC.Assign(aBH.InternalCurves());
        if (aT1 < 0.0)
        {
            HelixGeom_BuilderHelix aBH1 = new();
            aBH1.SetPosition(aAx2);
            aBH1.SetCurveParameters(aT1, aT0, thePitch, aDist, theTaperAngle, bIsClockwise);
            aBH1.SetTolerance(myTolerance);
            aBH1.SetApproxParameters(myContinuity, myMaxDegree, myMaxSegments);
            //
            aBH1.Perform();
            iErr = aBH1.ErrorStatus();
            if (iErr != 0)
            {
                myErrorStatus = 2;
                return;
            }

            myTolReached = Math.Max(myTolReached, aBH1.ToleranceReached());
            NCollection_Sequence<Geom_Curve> aSC1 = aBH1.InternalCurves();
            int nbc = aSC1.Length();
            for (i = nbc; i >= 1; i--)
            {
                aSC.Prepend(aSC1.Value(i));
            }
        }

        aNbC = aSC.Length();
        for (i = 1; i <= aNbC; ++i)
        {
            Geom_Curve aC = aSC[i];
            //
            if (i == 1)
            {
                if (aT1 > 0.0)
                {
                    aT2 = aC.LastParameter();
                    Geom_TrimmedCurve aCT = new(aC, aT1, aT2);
                    aC = aCT;
                }
                aT1 = aC.FirstParameter();
                aC.D0(aT1, out aP1);
                aBB.MakeVertex(ref aV1, aP1, myTolReached);
                aV1.Orientation(TopAbs_FORWARD);
            }
            //
            aT2 = aC.LastParameter();
            aC.D0(aT2, out aP2);
            aBB.MakeVertex(ref aV2, aP2, myTolReached);
            aV2.Orientation(TopAbs_REVERSED);
            //
            aBME.Init(aC, aV1, aV2);
            bIsDone = aBME.IsDone();
            if (!bIsDone)
            {
                myErrorStatus = 3;
                return;
            }
            aE = aBME.InternalEdge();
            aBB.UpdateEdge(aE, myTolReached);
            aBB.Add(thePart, aE);

            //
            aV1 = aV2;
            aV1.Orientation(TopAbs_FORWARD);
        }
        //
        if (myTolReached > myTolerance)
        {
            myWarningStatus = 1;
        }
    }

    private void Smoothing(NCollection_List<TopoDS_Shape> theParts)
    {
        if (theParts.Extent() == 1)
        {
            return;
        }

        BRepTools_WireExplorer anExpl = new();

        TopoDS_Wire aPrevWire = TopoDS.Wire(theParts.First());
        anExpl.Init(aPrevWire);
        TopoDS_Edge aPrevEdge = new();
        for (; anExpl.More(); anExpl.Next())
        {
            aPrevEdge = anExpl.Current();
        }

        NCollection_List<TopoDS_Shape>.Iterator anIter = new(theParts);
        anIter.Next();
        for (; anIter.More(); anIter.Next())
        {
            TopoDS_Wire aNextWire = TopoDS.Wire(anIter.Value());
            anExpl.Clear();
            anExpl.Init(aNextWire);
            TopoDS_Edge aNextEdge = anExpl.Current();

            // Smoothing curves
            SmoothingEdges(aPrevEdge, aNextEdge);

            for (; anExpl.More(); anExpl.Next())
            {
                aPrevEdge = anExpl.Current();
            }
        }
    }

    private void SmoothingEdges(in TopoDS_Edge thePrev, in TopoDS_Edge theNext)
    {
        const double EpsAng = 1.0e-7;
        BRep_Builder aBB = new();

        double f1, l1, f2, l2;
        Geom_BSplineCurve aCPrev = (BRep_Tool.Curve(thePrev, out f1, out l1) as Geom_BSplineCurve)!;

        Geom_BSplineCurve aCNext = (BRep_Tool.Curve(theNext, out f2, out l2) as Geom_BSplineCurve)!;

        gp_Pnt P1, P2;
        gp_Vec V1, V2;

        aCPrev.D1(l1, out P1, out V1);
        aCNext.D1(f2, out P2, out V2);

        if (V1.Angle(V2) < EpsAng)
        {
            return;
        }

        V1 = 0.5 * (V1 + V2);
        V2 = V1;

        // OCCT changes the edges' own curves in place, which BRep_Builder froze when it attached them: the port
        // changes copies and puts them on the edges (decision 994). OCCT's curve also keeps the degrees it was
        // raised to when no degree lets the tangent move; the copy is put on the edge in that case too.
        aCPrev = (Geom_BSplineCurve)aCPrev.Copy();
        aCNext = (Geom_BSplineCurve)aCNext.Copy();

        int anErrorStatus = 1;
        int aDegMax = Geom_BSplineCurve.MaxDegree();
        int aDeg = aCPrev.Degree();
        int i;
        bool bPrevOK = false;
        bool bNextOK = false;

        aCPrev.MovePointAndTangent(l1, P1, V1, myTolerance, 1, -1, out anErrorStatus);
        if (anErrorStatus != 0)
        {
            for (i = aDeg + 1; i <= aDegMax; ++i)
            {
                aCPrev.IncreaseDegree(i);
                aCPrev.MovePointAndTangent(l1, P1, V1, myTolerance, 1, -1, out anErrorStatus);
                if (anErrorStatus == 0)
                {
                    bPrevOK = true;
                    break;
                }
            }
        }
        else
        {
            bPrevOK = true;
        }

        if (bPrevOK || aCPrev.Degree() != aDeg)
        {
            aBB.UpdateEdge(thePrev, aCPrev, BRep_Tool.Tolerance(thePrev));
        }

        aDeg = aCNext.Degree();
        aCNext.MovePointAndTangent(f2, P2, V2, myTolerance, -1, 1, out anErrorStatus);
        if (anErrorStatus != 0)
        {
            for (i = aDeg + 1; i <= aDegMax; ++i)
            {
                aCNext.IncreaseDegree(i);
                aCNext.MovePointAndTangent(f2, P2, V2, myTolerance, -1, 1, out anErrorStatus);
                if (anErrorStatus == 0)
                {
                    bNextOK = true;
                    break;
                }
            }
        }
        else
        {
            bNextOK = true;
        }

        if (bNextOK || aCNext.Degree() != aDeg)
        {
            aBB.UpdateEdge(theNext, aCNext, BRep_Tool.Tolerance(theNext));
        }
    }

    /// <summary>The error status (see the class remarks).</summary>
    public int ErrorStatus() => myErrorStatus;

    /// <summary>The warning status (see the class remarks).</summary>
    public int WarningStatus() => myWarningStatus;

    /// <summary>The helix wire (frozen), or a null shape if it wasn't built.</summary>
    public TopoDS_Shape Shape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>OCCT's <c>Shape()</c>, without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalShape() => myShape;
}
