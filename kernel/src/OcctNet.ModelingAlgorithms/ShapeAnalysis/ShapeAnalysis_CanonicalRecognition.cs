// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_CanonicalRecognition.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_CanonicalRecognition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.GeomConvert_ConvType;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Finds whether the geometry of a shape (a face, a shell, an edge or a wire) can be represented within a tolerance
/// by a simple analytical entity (plane, cylinder, cone, sphere, line, circle, ellipse), which could replace complex
/// (for example B-spline) geometry. Only reads the shape.
/// </summary>
/// <remarks>
/// A working object (decision 21). The <c>Is*</c> methods take the sample entity by <c>ref</c>: they read its
/// position and parameters as a first guess (for edges and wires) and write the result only on success.
/// </remarks>
public sealed class ShapeAnalysis_CanonicalRecognition
{
    private TopoDS_Shape myShape;
    private TopAbs_ShapeEnum mySType;
    private double myGap;
    private int myStatus;

    /// <summary>Empty constructor (status -1: not initialised).</summary>
    public ShapeAnalysis_CanonicalRecognition()
    {
        myShape = new TopoDS_Shape();
        mySType = TopAbs_SHAPE;
        myGap = -1.0;
        myStatus = -1;
    }

    /// <summary>Constructor with the shape.</summary>
    public ShapeAnalysis_CanonicalRecognition(in TopoDS_Shape theShape)
    {
        myShape = new TopoDS_Shape();
        mySType = TopAbs_SHAPE;
        myGap = -1.0;
        myStatus = -1;
        Init(theShape);
    }

    /// <summary>Sets the shape: a shell, a face, a wire or an edge (otherwise the status is 1).</summary>
    public void SetShape(in TopoDS_Shape theShape)
    {
        Init(theShape);
    }

    /// <summary>The input shape.</summary>
    public TopoDS_Shape GetShape() => myShape;

    /// <summary>The deviation between the input geometry and the analytical entity.</summary>
    public double GetGap() => myGap;

    /// <summary>
    /// The status: -1 not initialised by a shape, 0 no errors, 1 an error during an operation (usually wrong input).
    /// Operations work when it is 0; after one fails, <see cref="ClearStatus"/> resets it.
    /// </summary>
    public int GetStatus() => myStatus;

    /// <summary>Resets the status to 0.</summary>
    public void ClearStatus()
    {
        myStatus = 0;
    }

    private void Init(in TopoDS_Shape theShape)
    {
        TopAbs_ShapeEnum aT = theShape.ShapeType();
        switch (aT)
        {
            case TopAbs_SHELL:
            case TopAbs_FACE:
            case TopAbs_WIRE:
            case TopAbs_EDGE:
                myShape = theShape;
                mySType = aT;
                myStatus = 0;
                break;
            default:
                myStatus = 1;
                break;
        }
    }

    private bool IsElementarySurf(SurfaceKind theTarget, double theTol, ref CoordinateSystem thePos,
                                  NCollection_Array1<double> theParams)
    {
        if (myStatus != 0)
        {
            return false;
        }
        //
        if (mySType == TopAbs_FACE)
        {
            Geom_Surface? anElemSurf = GetSurface(TopoDS.Face(myShape), theTol, GeomConvert_Target, theTarget, ref myGap, ref myStatus);
            if (anElemSurf is null)
            {
                return false;
            }
            //
            bool isOK = SetSurfParams(theTarget, anElemSurf, ref thePos, theParams);
            if (!isOK)
            {
                myStatus = 1;
                return false;
            }
            return true;
        }
        else if (mySType == TopAbs_SHELL)
        {
            Geom_Surface? anElemSurf = GetSurface(TopoDS.Shell(myShape), theTol, GeomConvert_Target, theTarget, ref myGap, ref myStatus);
            if (anElemSurf is null)
            {
                return false;
            }
            bool isOK = SetSurfParams(theTarget, anElemSurf, ref thePos, theParams);
            if (!isOK)
            {
                myStatus = 1;
                return false;
            }
            return true;
        }
        else if (mySType == TopAbs_EDGE)
        {
            Geom_Surface? anElemSurf = GetSurface(TopoDS.Edge(myShape), theTol, GeomConvert_Target, theTarget, ref thePos,
                                                  theParams, ref myGap, ref myStatus);

            bool isOK = SetSurfParams(theTarget, anElemSurf, ref thePos, theParams);
            if (!isOK)
            {
                myStatus = 1;
                return false;
            }
            return true;
        }
        else if (mySType == TopAbs_WIRE)
        {
            Geom_Surface? anElemSurf = GetSurface(TopoDS.Wire(myShape), theTol, GeomConvert_Target, theTarget, ref thePos,
                                                  theParams, ref myGap, ref myStatus);

            bool isOK = SetSurfParams(theTarget, anElemSurf, ref thePos, theParams);
            if (!isOK)
            {
                myStatus = 1;
                return false;
            }
            return true;
        }
        return false;
    }

    /// <summary>
    /// True if the underlying surface can be represented by a plane within <paramref name="theTol"/>; the plane is
    /// then set in <paramref name="thePln"/>. An edge or a wire is also tried with BRepLib_FindSurface.
    /// </summary>
    public bool IsPlane(double theTol, ref Plane3D thePln)
    {
        CoordinateSystem aPos = thePln.Position();
        NCollection_Array1<double> aParams = new(1, 1);
        //
        SurfaceKind aTarget = Plane;
        if (IsElementarySurf(aTarget, theTol, ref aPos, aParams))
        {
            thePln.SetPosition(aPos);
            return true;
        }
        // Try to build plane for wire or edge
        if (mySType == TopAbs_EDGE || mySType == TopAbs_WIRE)
        {
            BRepLib_FindSurface aFndSurf = new(myShape, theTol, true, false);
            if (aFndSurf.Found())
            {
                var aPlane = aFndSurf.Surface() as Geom_Plane;
                thePln = aPlane!.Pln();
                myGap = aFndSurf.ToleranceReached();
                myStatus = 0;
                return true;
            }
            else
            {
                myStatus = 1;
            }
        }
        return false;
    }

    /// <summary>
    /// True if the underlying surface can be represented by a cylinder within <paramref name="theTol"/>; then set in
    /// <paramref name="theCyl"/>. For an edge or a wire, the sample cylinder (if its radius is set) starts a
    /// least-squares fit.
    /// </summary>
    public bool IsCylinder(double theTol, ref Cylinder theCyl)
    {
        CoordinateSystem aPos = theCyl.Position();
        NCollection_Array1<double> aParams = new(1, 1);
        aParams[1] = theCyl.Radius();
        //
        SurfaceKind aTarget = SurfaceKind.Cylinder;
        if (IsElementarySurf(aTarget, theTol, ref aPos, aParams))
        {
            theCyl.SetPosition(aPos);
            theCyl.SetRadius(aParams[1]);
            return true;
        }

        if (aParams[1] > Kerf.Engine.Foundation.Precision.Infinite())
        {
            // Sample cylinder does not seem to be set, least square method is not applicable.
            return false;
        }
        if (myShape.ShapeType() == TopAbs_EDGE || myShape.ShapeType() == TopAbs_WIRE)
        {
            // Try to build surface by least square method;
            TopoDS_Wire aWire = MakeWireOf(myShape);
            bool isDone = GetSurfaceByLS(aWire, theTol, aTarget, ref aPos, aParams, ref myGap, ref myStatus);
            if (isDone)
            {
                theCyl.SetPosition(aPos);
                theCyl.SetRadius(aParams[1]);
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// True if the underlying surface can be represented by a cone within <paramref name="theTol"/>; then set in
    /// <paramref name="theCone"/>. For an edge or a wire, the sample cone (if its radius is set) starts a least-squares
    /// fit.
    /// </summary>
    public bool IsCone(double theTol, ref Cone theCone)
    {
        CoordinateSystem aPos = theCone.Position();
        NCollection_Array1<double> aParams = new(1, 2);
        aParams[1] = theCone.SemiAngle();
        aParams[2] = theCone.RefRadius();
        //
        SurfaceKind aTarget = SurfaceKind.Cone;
        if (IsElementarySurf(aTarget, theTol, ref aPos, aParams))
        {
            theCone.SetPosition(aPos);
            theCone.SetSemiAngle(aParams[1]);
            theCone.SetRadius(aParams[2]);
            return true;
        }

        if (aParams[2] > Kerf.Engine.Foundation.Precision.Infinite())
        {
            // Sample cone does not seem to be set, least square method is not applicable.
            return false;
        }
        if (myShape.ShapeType() == TopAbs_EDGE || myShape.ShapeType() == TopAbs_WIRE)
        {
            // Try to build surface by least square method;
            TopoDS_Wire aWire = MakeWireOf(myShape);
            bool isDone = GetSurfaceByLS(aWire, theTol, aTarget, ref aPos, aParams, ref myGap, ref myStatus);
            if (isDone)
            {
                theCone.SetPosition(aPos);
                theCone.SetSemiAngle(aParams[1]);
                theCone.SetRadius(aParams[2]);
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// True if the underlying surface can be represented by a sphere within <paramref name="theTol"/>; then set in
    /// <paramref name="theSphere"/>. For an edge or a wire, the sample sphere (if its radius is set) starts a
    /// least-squares fit.
    /// </summary>
    public bool IsSphere(double theTol, ref Sphere theSphere)
    {
        CoordinateSystem aPos = theSphere.Position();
        NCollection_Array1<double> aParams = new(1, 1);
        aParams[1] = theSphere.Radius();
        //
        SurfaceKind aTarget = SurfaceKind.Sphere;
        if (IsElementarySurf(aTarget, theTol, ref aPos, aParams))
        {
            theSphere.SetPosition(aPos);
            theSphere.SetRadius(aParams[1]);
            return true;
        }
        //
        if (aParams[1] > Kerf.Engine.Foundation.Precision.Infinite())
        {
            // Sample sphere does not seem to be set, least square method is not applicable.
            return false;
        }
        if (myShape.ShapeType() == TopAbs_EDGE || myShape.ShapeType() == TopAbs_WIRE)
        {
            // Try to build surface by least square method;
            TopoDS_Wire aWire = MakeWireOf(myShape);
            bool isDone = GetSurfaceByLS(aWire, theTol, aTarget, ref aPos, aParams, ref myGap, ref myStatus);
            if (isDone)
            {
                theSphere.SetPosition(aPos);
                theSphere.SetRadius(aParams[1]);
                return true;
            }
        }
        return false;
    }

    // OCCT.NET: the wire OCCT builds inline in IsCylinder, IsCone and IsSphere (a new wire holding the edge, or the wire
    // itself). Only that new wire is written.
    private static TopoDS_Wire MakeWireOf(in TopoDS_Shape theShape)
    {
        TopoDS_Wire aWire = new();
        if (theShape.ShapeType() == TopAbs_EDGE)
        {
            BRep_Builder aBB = new();
            aBB.MakeWire(ref aWire);
            aBB.Add(aWire, theShape);
        }
        else
        {
            aWire = TopoDS.Wire(theShape);
        }
        return aWire;
    }

    private bool IsConic(CurveKind theTarget, double theTol, ref Frame3D thePos, NCollection_Array1<double> theParams)
    {
        if (myStatus != 0)
        {
            return false;
        }

        if (mySType == TopAbs_EDGE)
        {
            Geom_Curve? aConic = GetCurve(TopoDS.Edge(myShape), theTol, GeomConvert_Target, theTarget, ref myGap, ref myStatus);

            if (aConic is null)
            {
                return false;
            }

            bool isOK = SetConicParameters(theTarget, aConic, ref thePos, theParams);

            if (!isOK)
            {
                myStatus = 1;
                return false;
            }
            return true;
        }
        else if (mySType == TopAbs_WIRE)
        {
            TopoDS_Iterator anIter = new(myShape);
            if (!anIter.More())
            {
                myStatus = 1;
                return false;
            }
            Frame3D aPos = new();
            NCollection_Array1<double> aParams = new(1, theParams.Length());
            TopoDS_Shape anEdge = anIter.Value();

            Geom_Curve? aConic = GetCurve(TopoDS.Edge(anEdge), theTol, GeomConvert_Target, theTarget, ref myGap, ref myStatus);
            if (aConic is null)
            {
                return false;
            }
            bool isOK = SetConicParameters(theTarget, aConic, ref thePos, theParams);
            if (!isOK)
            {
                myStatus = 1;
                return false;
            }
            if (!anIter.More())
            {
                return true;
            }
            else
            {
                anIter.Next();
            }
            for (; anIter.More(); anIter.Next())
            {
                aConic = GetCurve(TopoDS.Edge(anIter.Value()), theTol, GeomConvert_Target, theTarget, ref myGap, ref myStatus);
                if (aConic is null)
                {
                    return false;
                }
                isOK = SetConicParameters(theTarget, aConic, ref aPos, aParams);
                isOK = CompareConicParams(theTarget, theTol, thePos, theParams, aPos, aParams);

                if (!isOK)
                {
                    return false;
                }
            }
            return true;
        }
        myStatus = 1;
        return false;
    }

    /// <summary>
    /// True if the underlying curve (of an edge, or of every edge of a wire) can be represented by a line within
    /// <paramref name="theTol"/>; then set in <paramref name="theLin"/>.
    /// </summary>
    public bool IsLine(double theTol, ref Line3D theLin)
    {
        Frame3D aPos = new();
        NCollection_Array1<double> aParams = new(1, 1);

        CurveKind aTarget = Line;
        bool isOK = IsConic(aTarget, theTol, ref aPos, aParams);
        if (isOK)
        {
            theLin.SetPosition(aPos.Axis());
        }
        return isOK;
    }

    /// <summary>
    /// True if the underlying curve can be represented by a circle within <paramref name="theTol"/>; then set in
    /// <paramref name="theCirc"/>.
    /// </summary>
    public bool IsCircle(double theTol, ref Circle3D theCirc)
    {
        Frame3D aPos = new();
        NCollection_Array1<double> aParams = new(1, 1);

        CurveKind aTarget = Circle;
        bool isOK = IsConic(aTarget, theTol, ref aPos, aParams);
        if (isOK)
        {
            theCirc.SetPosition(aPos);
            theCirc.SetRadius(aParams[1]);
        }
        return isOK;
    }

    /// <summary>
    /// True if the underlying curve can be represented by an ellipse within <paramref name="theTol"/>; then set in
    /// <paramref name="theElips"/>.
    /// </summary>
    public bool IsEllipse(double theTol, ref Ellipse3D theElips)
    {
        Frame3D aPos = new();
        NCollection_Array1<double> aParams = new(1, 2);

        CurveKind aTarget = Ellipse;
        bool isOK = IsConic(aTarget, theTol, ref aPos, aParams);
        if (isOK)
        {
            theElips.SetPosition(aPos);
            theElips.SetMajorRadius(aParams[1]);
            theElips.SetMinorRadius(aParams[2]);
        }
        return isOK;
    }

    private static Geom_Surface? GetSurface(in TopoDS_Face theFace, double theTol, GeomConvert_ConvType theType,
                                            SurfaceKind theTarget, ref double theGap, ref int theStatus)
    {
        theStatus = 0;
        Geom_Surface? aSurf = BRep_Tool.Surface(theFace, out Placement aLoc);
        if (aSurf is null)
        {
            theStatus = 1;
            return aSurf;
        }
        GeomConvert_SurfToAnaSurf aConv = new(aSurf);
        aConv.SetConvType(theType);
        aConv.SetTarget(theTarget);
        Geom_Surface? anAnaSurf = aConv.ConvertToAnalytical(theTol);
        if (anAnaSurf is null)
        {
            return anAnaSurf;
        }
        //
        if (!aLoc.IsIdentity)
        {
            anAnaSurf.Transform(aLoc.Transformation()); // a new surface of the converter
        }
        //
        theGap = aConv.Gap();
        return anAnaSurf;
    }

    private static Geom_Surface? GetSurface(in TopoDS_Shell theShell, double theTol, GeomConvert_ConvType theType,
                                            SurfaceKind theTarget, ref double theGap, ref int theStatus)
    {
        Geom_Surface? anElemSurf = null;
        TopoDS_Iterator anIter = new(theShell);
        if (!anIter.More())
        {
            theStatus = 1;
            return anElemSurf;
        }
        CoordinateSystem aPos1 = new();
        NCollection_Array1<double> aParams1 = new(1, Max(1, GetNbPars(theTarget)));
        TopoDS_Shape aFace = anIter.Value();

        anElemSurf = GetSurface(TopoDS.Face(aFace), theTol, theType, theTarget, ref theGap, ref theStatus);
        if (anElemSurf is null)
        {
            return anElemSurf;
        }
        SetSurfParams(theTarget, anElemSurf, ref aPos1, aParams1);
        if (!anIter.More())
        {
            return anElemSurf;
        }
        else
        {
            anIter.Next();
        }
        CoordinateSystem aPos = new();
        NCollection_Array1<double> aParams = new(1, Max(1, GetNbPars(theTarget)));
        bool isOK;
        for (; anIter.More(); anIter.Next())
        {
            Geom_Surface? anElemSurf1 = GetSurface(TopoDS.Face(anIter.Value()), theTol, theType, theTarget, ref theGap, ref theStatus);
            if (anElemSurf1 is null)
            {
                return anElemSurf1;
            }
            // OCCT: the parameters are taken from the first face's surface again, not from anElemSurf1, so the faces
            // after the first always compare equal (occt-issues.md)
            SetSurfParams(theTarget, anElemSurf, ref aPos, aParams);
            isOK = CompareSurfParams(theTarget, theTol, aPos1, aParams1, aPos, aParams);

            if (!isOK)
            {
                anElemSurf = null;
                return anElemSurf;
            }
        }
        return anElemSurf;
    }

    private static Geom_Surface? GetSurface(in TopoDS_Edge theEdge, double theTol, GeomConvert_ConvType theType,
                                            SurfaceKind theTarget, ref CoordinateSystem thePos,
                                            NCollection_Array1<double> theParams, ref double theGap, ref int theStatus)
    {
        // Get surface list
        NCollection_DynamicArray<Geom_Surface> aSurfs = new();
        NCollection_DynamicArray<double> aGaps = new();
        int j = 0;
        for (;;)
        {
            j++;
            Geom_Surface? aSurf = null;
            Placement aLoc = new();
            Geom2d_Curve? aPCurve = null;
            double ff = 0.0, ll = 0.0;
            BRep_Tool.CurveOnSurface(theEdge, ref aPCurve, ref aSurf, ref aLoc, ref ff, ref ll, j);
            if (aSurf is null)
            {
                break;
            }
            GeomConvert_SurfToAnaSurf aConv = new(aSurf);
            aConv.SetConvType(theType);
            aConv.SetTarget(theTarget);
            Geom_Surface? anAnaSurf = aConv.ConvertToAnalytical(theTol);
            if (anAnaSurf is null)
            {
                continue;
            }
            //
            if (!aLoc.IsIdentity)
            {
                anAnaSurf.Transform(aLoc.Transformation()); // a new surface of the converter
            }
            //
            aGaps.Append(aConv.Gap());
            aSurfs.Append(anAnaSurf);
        }

        if (aSurfs.Size() == 0)
        {
            theStatus = 1;
            return null;
        }

        CoordinateSystem aPos = new();
        int aNbPars = Max(1, GetNbPars(theTarget));
        NCollection_Array1<double> aParams = new(1, aNbPars);

        int ifit = -1, i;
        double aMinDev = RealLast();
        if (aSurfs.Size() == 1)
        {
            ifit = 0;
        }
        else
        {
            for (i = 0; i < aSurfs.Length(); ++i)
            {
                SetSurfParams(theTarget, aSurfs[i], ref aPos, aParams);
                double aDev = DeviationSurfParams(theTarget, thePos, theParams, aPos, aParams);
                if (aDev < aMinDev)
                {
                    aMinDev = aDev;
                    ifit = i;
                }
            }
        }
        if (ifit >= 0)
        {
            SetSurfParams(theTarget, aSurfs[ifit], ref thePos, theParams);
            theGap = aGaps[ifit];
            return aSurfs[ifit];
        }
        else
        {
            theStatus = 1;
            return null;
        }
    }

    private static Geom_Surface? GetSurface(in TopoDS_Wire theWire, double theTol, GeomConvert_ConvType theType,
                                            SurfaceKind theTarget, ref CoordinateSystem thePos,
                                            NCollection_Array1<double> theParams, ref double theGap, ref int theStatus)
    {
        TopoDS_Iterator anIter = new(theWire);
        if (!anIter.More())
        {
            // Empty wire
            theStatus = 1;
            return null;
        }
        // First edge
        TopoDS_Edge anEdge1 = TopoDS.Edge(anIter.Value());
        CoordinateSystem aPos1 = thePos;
        int aNbPars = GetNbPars(theTarget);
        NCollection_Array1<double> aParams1 = new(1, Max(1, aNbPars));
        double aGap1 = 0.0; // OCCT.NET: uninitialised in OCCT; written whenever the result is used
        int i;
        for (i = 1; i <= aNbPars; ++i)
        {
            aParams1[i] = theParams[i];
        }
        Geom_Surface? anElemSurf1 = GetSurface(anEdge1, theTol, theType, theTarget, ref aPos1, aParams1, ref aGap1, ref theStatus);
        if (theStatus != 0 || anElemSurf1 is null)
        {
            return null;
        }
        anIter.Next();
        for (; anIter.More(); anIter.Next())
        {
            CoordinateSystem aPos = aPos1;
            aNbPars = GetNbPars(theTarget);
            NCollection_Array1<double> aParams = new(1, Max(1, aNbPars));
            for (i = 1; i <= aNbPars; ++i)
            {
                aParams[i] = aParams1[i];
            }
            double aGap = 0.0;
            TopoDS_Edge anEdge = TopoDS.Edge(anIter.Value());
            Geom_Surface? anElemSurf = GetSurface(anEdge, theTol, theType, theTarget, ref aPos, aParams, ref aGap, ref theStatus);
            if (theStatus != 0 || anElemSurf is null)
            {
                return null;
            }
            bool isOK = CompareSurfParams(theTarget, theTol, aPos1, aParams1, aPos, aParams);
            if (!isOK)
            {
                return null;
            }
        }

        SetSurfParams(theTarget, anElemSurf1, ref thePos, theParams);
        theGap = aGap1;
        return anElemSurf1;
    }

    private static bool GetSurfaceByLS(in TopoDS_Wire theWire, double theTol, SurfaceKind theTarget, ref CoordinateSystem thePos,
                                       NCollection_Array1<double> theParams, ref double theGap, ref int theStatus)
    {
        int aNbMaxInt = 100;
        if (!GetSamplePoints(theWire, theTol, aNbMaxInt, out NCollection_HArray1<Xyz>? aPoints))
        {
            return false;
        }

        theGap = GetLSGap(aPoints!, theTarget, thePos, theParams);
        if (theGap <= theTol)
        {
            theStatus = 0;
            return true;
        }

        int aNbVar = 0;
        if (theTarget == SurfaceKind.Sphere)
        {
            aNbVar = 4;
        }
        else if (theTarget == SurfaceKind.Cylinder)
        {
            aNbVar = 4;
        }
        else if (theTarget == SurfaceKind.Cone)
        {
            aNbVar = 5;
        }
        else
        {
            return false;
        }

        math_Vector aFBnd = new(1, aNbVar), aLBnd = new(1, aNbVar), aStartPoint = new(1, aNbVar);

        double aRelDev = 0.2; // Customer can set parameters of sample surface
                              //  with relative precision about aRelDev.
                              //  For example, if radius of sample surface is R,
                              //  it means, that "exact" value is in interav
                              //[R - aRelDev*R, R + aRelDev*R]. This interval is set
                              //  for R as boundary values for optimization algo.
        FillSolverData(theTarget, thePos, theParams, aStartPoint, aFBnd, aLBnd, aRelDev);

        //
        double aTol = Kerf.Engine.Foundation.Precision.Confusion();
        MultipleVarFunction aPFunc;
        GeomConvert_FuncSphereLSDist aFuncSph = new(aPoints!);
        GeomConvert_FuncCylinderLSDist aFuncCyl = new(aPoints!, thePos.Direction());
        GeomConvert_FuncConeLSDist aFuncCon = new(aPoints!, thePos.Direction());
        if (theTarget == SurfaceKind.Sphere)
        {
            aPFunc = aFuncSph;
        }
        else if (theTarget == SurfaceKind.Cylinder)
        {
            aPFunc = aFuncCyl;
        }
        else
        {
            aPFunc = aFuncCon;
        }
        //
        math_Vector aSteps = new(1, aNbVar);
        int aNbInt = 10;
        int i;
        for (i = 1; i <= aNbVar; ++i)
        {
            aSteps[i] = (aLBnd[i] - aFBnd[i]) / aNbInt;
        }
        Pso aGlobSolver = new(aPFunc, aFBnd, aLBnd, aSteps);
        aGlobSolver.Perform(aSteps, out double _, aStartPoint);
        SetCanonicParameters(theTarget, aStartPoint, ref thePos, theParams);

        theGap = GetLSGap(aPoints!, theTarget, thePos, theParams);
        if (theGap <= theTol)
        {
            theStatus = 0;
            return true;
        }
        //
        Matrix aDirMatrix = new(1, aNbVar, 1, aNbVar, 0.0);
        for (i = 1; i <= aNbVar; i++)
        {
            aDirMatrix[i, i] = 1.0;
        }

        if (theTarget == SurfaceKind.Cylinder || theTarget == SurfaceKind.Cone)
        {
            // Set search direction for location to be perpendicular to axis to avoid
            // searching along axis
            Direction3D aDir = thePos.Direction();
            Plane3D aPln = new(thePos.Location(), aDir);
            Direction3D aUDir = aPln.Position().XDirection();
            Direction3D aVDir = aPln.Position().YDirection();
            for (i = 1; i <= 3; ++i)
            {
                aDirMatrix[i, 1] = aUDir.Coord(i);
                aDirMatrix[i, 2] = aVDir.Coord(i);
                Direction3D aUVDir = new(aUDir.XYZ() + aVDir.XYZ());
                aDirMatrix[i, 3] = aUVDir.Coord(i);
            }
        }

        Powell aSolver = new(aPFunc, aTol);
        aSolver.Perform(aPFunc, aStartPoint, aDirMatrix);

        if (aSolver.IsDone)
        {
            aSolver.Location(aStartPoint);
            theStatus = 0;
            SetCanonicParameters(theTarget, aStartPoint, ref thePos, theParams);
            theGap = GetLSGap(aPoints!, theTarget, thePos, theParams);
            theStatus = 0;
            if (theGap <= theTol)
            {
                return true;
            }
        }
        else
        {
            theStatus = 1;
        }

        return false;
    }

    private static Geom_Curve? GetCurve(in TopoDS_Edge theEdge, double theTol, GeomConvert_ConvType theType,
                                        CurveKind theTarget, ref double theGap, ref int theStatus)
    {
        theStatus = 0;
        Geom_Curve? aCurv = BRep_Tool.Curve(theEdge, out Placement aLoc, out double f, out double l);
        if (aCurv is null)
        {
            theStatus = 1;
            return aCurv;
        }
        GeomConvert_CurveToAnaCurve aConv = new(aCurv);
        aConv.SetConvType(theType);
        aConv.SetTarget(theTarget);
        Geom_Curve? anAnaCurv = null;
        aConv.ConvertToAnalytical(theTol, ref anAnaCurv, f, l, out double _, out double _);
        if (anAnaCurv is null)
        {
            return anAnaCurv;
        }
        //
        if (!aLoc.IsIdentity)
        {
            // OCCT: when the edge's curve is already the target conic, the converter returns the edge's own curve,
            // and OCCT transforms that shared curve in place (occt-issues.md). OCCT.NET: frozen: it is frozen here, so
            // the port transforms a copy; the result is the same, and the edge is unchanged.
            if (anAnaCurv.IsFrozen)
            {
                anAnaCurv = (Geom_Curve)anAnaCurv.Copy();
            }
            anAnaCurv.Transform(aLoc.Transformation());
        }
        //
        theGap = aConv.Gap();
        return anAnaCurv;
    }

    // Static methods

    private static int GetNbPars(CurveKind theTarget)
    {
        int aNbPars = 0;
        switch (theTarget)
        {
            case Line:
                aNbPars = 0;
                break;
            case Circle:
                aNbPars = 1;
                break;
            case Ellipse:
                aNbPars = 2;
                break;
            default:
                aNbPars = 0;
                break;
        }

        return aNbPars;
    }

    private static int GetNbPars(SurfaceKind theTarget)
    {
        int aNbPars = 0;
        switch (theTarget)
        {
            case Plane:
                aNbPars = 0;
                break;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Sphere:
                aNbPars = 1;
                break;
            case SurfaceKind.Cone:
                aNbPars = 2;
                break;
            default:
                aNbPars = 0;
                break;
        }

        return aNbPars;
    }

    private static bool SetConicParameters(CurveKind theTarget, Geom_Curve? theConic, ref Frame3D thePos,
                                           NCollection_Array1<double> theParams)
    {
        if (theConic is null)
        {
            return false;
        }
        GeomAdaptor_Curve aGAC = new(theConic);
        if (aGAC.GetType() != theTarget)
        {
            return false;
        }

        if (theTarget == Line)
        {
            Line3D aLin = aGAC.Line();
            thePos.SetAxis(aLin.Position());
        }
        else if (theTarget == Circle)
        {
            Circle3D aCirc = aGAC.Circle();
            thePos = aCirc.Position();
            theParams[1] = aCirc.Radius();
        }
        else if (theTarget == Ellipse)
        {
            Ellipse3D anElips = aGAC.Ellipse();
            thePos = anElips.Position();
            theParams[1] = anElips.MajorRadius();
            theParams[2] = anElips.MinorRadius();
        }
        else
        {
            return false;
        }
        return true;
    }

    private static bool CompareConicParams(CurveKind theTarget, double theTol, in Frame3D theRefPos,
                                           NCollection_Array1<double> theRefParams, in Frame3D thePos,
                                           NCollection_Array1<double> theParams)
    {
        int i, aNbPars = GetNbPars(theTarget);

        for (i = 1; i <= aNbPars; ++i)
        {
            if (Math.Abs(theRefParams[i] - theParams[i]) > theTol)
            {
                return false;
            }
        }

        double anAngTol = theTol / (2.0 * M_PI);
        double aTol = theTol;
        if (theTarget == Line)
        {
            aTol = Kerf.Engine.Foundation.Precision.Infinite();
        }

        Axis3D aRef = theRefPos.Axis();
        Axis3D anAx1 = thePos.Axis();
        Axis3D anAx1Rev = anAx1.Reversed();

        return aRef.IsCoaxial(anAx1, anAngTol, aTol) || aRef.IsCoaxial(anAx1Rev, anAngTol, aTol);
    }

    private static bool SetSurfParams(SurfaceKind theTarget, Geom_Surface? theElemSurf, ref CoordinateSystem thePos,
                                      NCollection_Array1<double> theParams)
    {
        //
        if (theElemSurf is null)
        {
            return false;
        }
        GeomAdaptor_Surface aGAS = new(theElemSurf);
        if (aGAS.GetType() != theTarget)
        {
            return false;
        }

        double aNbPars = GetNbPars(theTarget);
        if (theParams.Length() < aNbPars)
        {
            return false;
        }

        if (theTarget == Plane)
        {
            Plane3D aPln = aGAS.Plane();
            thePos = aPln.Position();
        }
        else if (theTarget == SurfaceKind.Cylinder)
        {
            Cylinder aCyl = aGAS.Cylinder();
            thePos = aCyl.Position();
            theParams[1] = aCyl.Radius();
        }
        else if (theTarget == SurfaceKind.Cone)
        {
            Cone aCon = aGAS.Cone();
            thePos = aCon.Position();
            theParams[1] = aCon.SemiAngle();
            theParams[2] = aCon.RefRadius();
        }
        else if (theTarget == SurfaceKind.Sphere)
        {
            Sphere aSph = aGAS.Sphere();
            thePos = aSph.Position();
            theParams[1] = aSph.Radius();
        }
        else
        {
            return false;
        }
        return true;
    }

    private static bool CompareSurfParams(SurfaceKind theTarget, double theTol, in CoordinateSystem theRefPos,
                                          NCollection_Array1<double> theRefParams, in CoordinateSystem thePos,
                                          NCollection_Array1<double> theParams)
    {
        if (theTarget != Plane)
        {
            if (Math.Abs(theRefParams[1] - theParams[1]) > theTol)
            {
                return false;
            }
        }
        //
        if (theTarget == SurfaceKind.Sphere)
        {
            Point3D aRefLoc = theRefPos.Location(), aLoc = thePos.Location();
            return aRefLoc.SquareDistance(aLoc) <= theTol * theTol;
        }
        //
        double anAngTol = theTol / (2.0 * M_PI);
        double aTol = theTol;
        if (theTarget == SurfaceKind.Cylinder || theTarget == SurfaceKind.Cone)
        {
            aTol = Kerf.Engine.Foundation.Precision.Infinite();
        }

        Axis3D aRef = theRefPos.Axis();
        Axis3D anAx1 = thePos.Axis();
        Axis3D anAx1Rev = anAx1.Reversed();
        if (!(aRef.IsCoaxial(anAx1, anAngTol, aTol) || aRef.IsCoaxial(anAx1Rev, anAngTol, aTol)))
        {
            return false;
        }

        if (theTarget == SurfaceKind.Cone)
        {
            Cone aRefCone = new(theRefPos, theRefParams[1], theRefParams[2]);
            Cone aCone = new(thePos, theParams[1], theParams[2]);
            Point3D aRefApex = aRefCone.Apex();
            Point3D anApex = aCone.Apex();
            return aRefApex.SquareDistance(anApex) <= theTol * theTol;
        }

        return true;
    }

    private static double DeviationSurfParams(SurfaceKind theTarget, in CoordinateSystem theRefPos,
                                              NCollection_Array1<double> theRefParams, in CoordinateSystem thePos,
                                              NCollection_Array1<double> theParams)
    {
        double aDevPars = 0.0;
        if (theTarget != Plane)
        {
            aDevPars = Math.Abs(theRefParams[1] - theParams[1]);
        }
        //
        if (theTarget == SurfaceKind.Sphere)
        {
            Point3D aRefLoc = theRefPos.Location(), aLoc = thePos.Location();
            aDevPars += aRefLoc.Distance(aLoc);
        }
        else
        {
            Direction3D aRefDir = theRefPos.Direction();
            Direction3D aDir = thePos.Direction();
            double anAngDev = (1.0 - Math.Abs(aRefDir * aDir));
            aDevPars += anAngDev;
        }

        return aDevPars;
    }

    private static bool GetSamplePoints(in TopoDS_Wire theWire, double theTol, int theMaxNbInt,
                                        out NCollection_HArray1<Xyz>? thePoints)
    {
        thePoints = null; // OCCT.NET: OCCT leaves the caller's handle unchanged (null) when it returns false
        NCollection_DynamicArray<double> aLengths = new();
        NCollection_DynamicArray<BRepAdaptor_Curve> aCurves = new();
        NCollection_DynamicArray<Xyz> aPoints = new();
        double aTol = Max(1.0e-3, theTol / 10.0);
        double aTotalLength = 0.0;
        TopoDS_Iterator anEIter = new(theWire);
        for (; anEIter.More(); anEIter.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anEIter.Value());
            if (BRep_Tool.Degenerated(anE))
            {
                continue;
            }
            BRepAdaptor_Curve aBAC = new(anE);
            double aClength = GCPnts_AbscissaPoint.Length(aBAC, aTol);
            aTotalLength += aClength;
            aCurves.Append(aBAC);
            aLengths.Append(aClength);
        }

        if (aTotalLength < theTol)
        {
            return false;
        }

        int i, aNb = aLengths.Length();
        for (i = 0; i < aNb; ++i)
        {
            BRepAdaptor_Curve aC = aCurves[i];
            double aClength = GCPnts_AbscissaPoint.Length(aC, aTol);
            int aNbPoints = RealToInt(aClength / aTotalLength * theMaxNbInt + 1);
            aNbPoints = Max(2, aNbPoints);
            GCPnts_QuasiUniformAbscissa aPointGen = new(aC, aNbPoints);
            if (!aPointGen.IsDone())
            {
                continue;
            }
            aNbPoints = aPointGen.NbPoints();
            int j;
            for (j = 1; j <= aNbPoints; ++j)
            {
                double t = aPointGen.Parameter(j);
                Point3D aP = aC.Value(t);
                aPoints.Append(aP.XYZ());
            }
        }

        if (aPoints.Length() < 1)
        {
            return false;
        }

        thePoints = new NCollection_HArray1<Xyz>(1, aPoints.Length());
        for (i = 0; i < aPoints.Length(); ++i)
        {
            thePoints.SetValue(i + 1, aPoints[i]);
        }

        return true;
    }

    private static double GetLSGap(NCollection_HArray1<Xyz> thePoints, SurfaceKind theTarget, in CoordinateSystem thePos,
                                   NCollection_Array1<double> theParams)
    {
        double aGap = 0.0;
        Xyz aLoc = thePos.Location().XYZ();
        Vector3D aDir = new(thePos.Direction());

        int i;
        if (theTarget == SurfaceKind.Sphere)
        {
            double anR = theParams[1];
            for (i = thePoints.Lower(); i <= thePoints.Upper(); ++i)
            {
                Xyz aD = thePoints.Value(i) - aLoc;
                aGap = Max(aGap, Math.Abs((aD.Modulus() - anR)));
            }
        }
        else if (theTarget == SurfaceKind.Cylinder)
        {
            double anR = theParams[1];
            for (i = thePoints.Lower(); i <= thePoints.Upper(); ++i)
            {
                Vector3D aD = new(thePoints.Value(i) - aLoc);
                aD.Cross(aDir);
                aGap = Max(aGap, Math.Abs((aD.Magnitude() - anR)));
            }
        }
        else if (theTarget == SurfaceKind.Cone)
        {
            double anAng = theParams[1];
            double anR = theParams[2];
            for (i = thePoints.Lower(); i <= thePoints.Upper(); ++i)
            {
                Point3D aPi = new(thePoints.Value(i));
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(thePos, anR, anAng, aPi, out double u, out double v);
                global::Kerf.Engine.Geometry.ElSLib.ConeD0(u, v, thePos, anR, anAng, out Point3D aPp);
                aGap = Max(aGap, aPi.SquareDistance(aPp));
            }
            aGap = Math.Sqrt(aGap);
        }

        return aGap;
    }

    private static void FillSolverData(SurfaceKind theTarget, in CoordinateSystem thePos, NCollection_Array1<double> theParams,
                                       math_Vector theStartPoint, math_Vector theFBnd, math_Vector theLBnd,
                                       double theRelDev = 0.2)
    {
        if (theTarget == SurfaceKind.Sphere || theTarget == SurfaceKind.Cylinder)
        {
            theStartPoint[1] = thePos.Location().X();
            theStartPoint[2] = thePos.Location().Y();
            theStartPoint[3] = thePos.Location().Z();
            theStartPoint[4] = theParams[1];
            double aDR = theRelDev * theParams[1];
            double aDXYZ = aDR;
            int i;
            for (i = 1; i <= 3; ++i)
            {
                theFBnd[i] = theStartPoint[i] - aDXYZ;
                theLBnd[i] = theStartPoint[i] + aDXYZ;
            }
            theFBnd[4] = theStartPoint[4] - aDR;
            theLBnd[4] = theStartPoint[4] + aDR;
        }
        if (theTarget == SurfaceKind.Cone)
        {
            theStartPoint[1] = thePos.Location().X();
            theStartPoint[2] = thePos.Location().Y();
            theStartPoint[3] = thePos.Location().Z();
            theStartPoint[4] = theParams[1]; // SemiAngle
            theStartPoint[5] = theParams[2]; // Radius
            double aDR = theRelDev * theParams[2];
            if (aDR < Kerf.Engine.Foundation.Precision.Confusion())
            {
                aDR = 0.1;
            }
            double aDXYZ = aDR;
            double aDAng = theRelDev * Math.Abs(theParams[1]);
            int i;
            for (i = 1; i <= 3; ++i)
            {
                theFBnd[i] = theStartPoint[i] - aDXYZ;
                theLBnd[i] = theStartPoint[i] + aDXYZ;
            }
            if (theParams[1] >= 0.0)
            {
                theFBnd[4] = theStartPoint[4] - aDAng;
                theLBnd[4] = Min(M_PI_2, theStartPoint[4] + aDR); // OCCT: the angle's bound uses the radius step aDR (occt-issues.md)
            }
            else
            {
                theFBnd[4] = Max(-M_PI_2, theStartPoint[4] - aDAng);
                theLBnd[4] = theStartPoint[4] + aDAng;
            }
            theFBnd[5] = theStartPoint[5] - aDR;
            theLBnd[5] = theStartPoint[5] + aDR;
        }
    }

    private static void SetCanonicParameters(SurfaceKind theTarget, math_Vector theSol, ref CoordinateSystem thePos,
                                             NCollection_Array1<double> theParams)
    {
        Point3D aLoc = new(theSol[1], theSol[2], theSol[3]);
        thePos.SetLocation(aLoc);
        if (theTarget == SurfaceKind.Sphere || theTarget == SurfaceKind.Cylinder)
        {
            theParams[1] = theSol[4]; // radius
        }
        else if (theTarget == SurfaceKind.Cone)
        {
            theParams[1] = theSol[4]; // semiangle
            theParams[2] = theSol[5]; // radius
        }
    }
}
