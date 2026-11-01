// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Tue Mar 15 16:20:43 2005
// Add methods for supporting history.
//  Modified by skv - Mon Jan 12 11:50:02 2004 OCC4455

using Bento.Collections;
using static Occt.BRepOffset_Error;
using static Occt.BRepOffset_Mode;
using static Occt.BRepOffset_Status;
using static Occt.ChFiDS_TypeOfConcavity;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_JoinType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The 3D offset of a shape (a face, a shell, a solid or a compound of them): the offset shell or solid, and the thick
/// solid (<see cref="MakeThickSolid"/>), with the faces connected by arcs (pipes and spheres) or by intersection.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). OCCT's algorithm writes into the shape it works on: tolerances
/// (<c>UpdateTolerance</c> unlocks the shape with <c>Locked(false)</c>), pcurves on the free boundary edges
/// (<c>MakeMissingWalls</c>), <c>Free(true)</c> before re-adding sub-shapes. So <see cref="MakeOffsetShape"/> and
/// <see cref="MakeThickSolid"/> first copy the caller's shape when it holds frozen TShapes (decision 1005, with
/// <see cref="BRepTools_CopyAtEntry"/>), map the closing faces and the faces with their own offsets onto the copy, and
/// run OCCT's code unchanged on the copy. At the end the copied sub-shapes the algorithm left unchanged are the
/// caller's own again in the result (a thick solid holds the caller's faces), and the history
/// (<see cref="Generated"/>, <see cref="Modified"/>, <see cref="IsDeleted"/>) answers for the caller's sub-shapes.
/// </para>
/// <para>
/// <see cref="Shape"/>, <see cref="Generated"/>, <see cref="Modified"/>, <see cref="ClosingFaces"/>,
/// <see cref="GetBadShape"/> and the images freeze what they hand out (publication points); ported code reads the
/// <c>Internal</c> forms. <see cref="GetAnalyse"/> is internal (decision 1007): its tangent treatment holds the
/// offset's working shapes.
/// </para>
/// </remarks>
public sealed partial class BRepOffset_MakeOffset
{
    private double myOffset;
    private double myTol;
    private TopoDS_Shape myInitialShape = new();
    private TopoDS_Shape myShape = new();
    private TopoDS_Compound myFaceComp = new();
    private BRepOffset_Mode myMode;
    private bool myIsLinearizationAllowed;
    private bool myInter;
    private bool mySelfInter;
    private GeomAbs_JoinType myJoin;
    private bool myThickening;
    private bool myRemoveIntEdges;
    private readonly NCollection_DataMap<TopoDS_Shape, double> myFaceOffset = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myFaces = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myOriginalFaces = new(TopTools_ShapeMapHasher.Instance);
    private readonly BRepOffset_Analyse myAnalyse = new();
    private TopoDS_Shape myOffsetShape = new();
    private readonly BRepAlgo_Image myInitOffsetFace = new();
    private readonly BRepAlgo_Image myInitOffsetEdge = new();
    private readonly BRepAlgo_Image myImageOffset = new();
    private readonly BRepAlgo_Image myImageVV = new();
    private readonly NCollection_List<TopoDS_Shape> myWalls = new();
    private BRepAlgo_AsDes myAsDes;
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myEdgeIntEdges =
        new(TopTools_ShapeMapHasher.Instance);
    private bool myDone;
    private BRepOffset_Error myError;
    private readonly BRepOffset_MakeLoops myMakeLoops = new();
    private bool myIsPerformSewing; // Handle bad walls in thicksolid mode.
    private bool myIsPlanar;
    private TopoDS_Shape myBadShape = new();
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myFacePlanfaceMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_List<TopoDS_Shape> myGenerated = new();
    private readonly NCollection_Map<TopoDS_Shape> myResMap = new(TopTools_ShapeMapHasher.Instance);

    // OCCT.NET: the copy at the entry (decision 1005); null until a run starts, and after Initialize.
    private BRepTools_CopyAtEntry? myEntry;

    // OCCT_DEBUG: the globals AffichInt2d, AffichOffC, ChronBuild, NbAE, NbAF, NVP, NVM, NVN, the chronometer and
    // DEBVerticesControl are left out.

    //=======================================================================
    // function : BRepOffset_PIOperation
    // purpose  : List of operations to be supported by the Progress Indicator
    //=======================================================================
    private const int PIOperation_CheckInputData = 0;
    private const int PIOperation_Analyse = 1;
    private const int PIOperation_BuildOffsetBy = 2;
    private const int PIOperation_Intersection = 3;
    private const int PIOperation_MakeMissingWalls = 4;
    private const int PIOperation_MakeShells = 5;
    private const int PIOperation_MakeSolid = 6;
    private const int PIOperation_Sewing = 7;
    private const int PIOperation_Last = 8;

    //=======================================================================
    // function : normalizeSteps
    // purpose  : Normalization of progress steps
    //=======================================================================
    private static void normalizeSteps(double theWhole, NCollection_Array1<double> theSteps)
    {
        double aSum = 0.0;
        for (int i = theSteps.Lower(); i <= theSteps.Upper(); ++i)
        {
            aSum += theSteps[i];
        }

        // Normalize steps
        for (int i = theSteps.Lower(); i <= theSteps.Upper(); ++i)
        {
            theSteps[i] = theWhole * theSteps[i] / aSum;
        }
    }

    //=================================================================================================

    private static TopAbs_Orientation OrientationOfEdgeInFace(in TopoDS_Edge theEdge, in TopoDS_Face theFace)
    {
        TopAbs_Orientation anOr = TopAbs_EXTERNAL;

        TopExp_Explorer Explo = new(theFace, TopAbs_EDGE);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Shape anEdge = Explo.Current();
            if (anEdge.IsSame(theEdge))
            {
                anOr = anEdge.Orientation();
                break;
            }
        }

        return anOr;
    }

    //=================================================================================================

    private static bool FindParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double U)
    {
        // Search the vertex in the edge

        bool rev = false;
        TopoDS_Shape VF = new();
        TopAbs_Orientation orient = TopAbs_INTERNAL;

        TopoDS_Iterator itv = new(E.Oriented(TopAbs_FORWARD));

        // if the edge has no vertices
        // and is degenerated use the vertex orientation
        // RLE, june 94

        if (!itv.More() && BRep_Tool.Degenerated(E))
        {
            orient = V.Orientation();
        }

        while (itv.More())
        {
            TopoDS_Shape Vcur = itv.Value();
            if (V.IsSame(Vcur))
            {
                if (VF.IsNull())
                {
                    VF = Vcur;
                }
                else
                {
                    rev = E.Orientation() == TopAbs_REVERSED;
                    if (Vcur.Orientation() == V.Orientation())
                    {
                        VF = Vcur;
                    }
                }
            }
            itv.Next();
        }

        if (!VF.IsNull())
        {
            orient = VF.Orientation();
        }

        double f, l;

        if (orient == TopAbs_FORWARD)
        {
            BRep_Tool.Range(E, out f, out l);
            // return (rev) ? l : f;
            U = (rev) ? l : f;
            return true;
        }

        else if (orient == TopAbs_REVERSED)
        {
            BRep_Tool.Range(E, out f, out l);
            // return (rev) ? f : l;
            U = (rev) ? f : l;
            return true;
        }

        else
        {
            Geom_Curve? C = BRep_Tool.Curve(E, out TopLoc_Location L, out f, out l);
            L = L.Predivided(V.Location());
            if (C is not null || BRep_Tool.Degenerated(E))
            {
                NCollection_List<BRep_PointRepresentation>.Iterator itpr = new(((BRep_TVertex)V.TShape()!).Points());

                while (itpr.More())
                {
                    BRep_PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnCurve(C, L))
                    {
                        double p = pr.Parameter();
                        double res = p; // SVV 4 nov 99 - to avoid warnings on Linux
                        if (C is not null)
                        {
                            // Closed curves RLE 16 june 94
                            if (Precision.IsNegativeInfinite(f))
                            {
                                // return pr->Parameter();//p;
                                U = pr.Parameter();
                                return true;
                            }
                            if (Precision.IsPositiveInfinite(l))
                            {
                                // return pr->Parameter();//p;
                                U = pr.Parameter();
                                return true;
                            }
                            gp_Pnt Pf = C.Value(f).Transformed(L.Transformation());
                            gp_Pnt Pl = C.Value(l).Transformed(L.Transformation());
                            double tol = BRep_Tool.Tolerance(V);
                            if (Pf.Distance(Pl) < tol)
                            {
                                if (Pf.Distance(BRep_Tool.Pnt(V)) < tol)
                                {
                                    if (V.Orientation() == TopAbs_FORWARD)
                                    {
                                        res = f; // p = f;
                                    }
                                    else
                                    {
                                        res = l; // p = l;
                                    }
                                }
                            }
                        }
                        // return res;//p;
                        U = res;
                        return true;
                    }
                    itpr.Next();
                }
            }
            else
            {
                // no 3d curve !!
                // let us try with the first pcurve
                BRep_Tool.CurveOnSurface(E, out Geom2d_Curve? PC, out Geom_Surface? S, out L, out f, out l);
                L = L.Predivided(V.Location());
                NCollection_List<BRep_PointRepresentation>.Iterator itpr = new(((BRep_TVertex)V.TShape()!).Points());

                while (itpr.More())
                {
                    BRep_PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnCurveOnSurface(PC, S, L))
                    {
                        double p = pr.Parameter();
                        // Closed curves RLE 16 june 94
                        if (PC!.IsClosed())
                        {
                            if ((p == PC.FirstParameter()) || (p == PC.LastParameter()))
                            {
                                if (V.Orientation() == TopAbs_FORWARD)
                                {
                                    p = PC.FirstParameter();
                                }
                                else
                                {
                                    p = PC.LastParameter();
                                }
                            }
                        }
                        // return p;
                        U = p;
                        return true;
                    }
                    itpr.Next();
                }
            }
        }

        // throw Standard_NoSuchObject("BRep_Tool:: no parameter on edge");
        return false;
    }

    //=======================================================================
    // function : GetEdgePoints
    // purpose  : gets the first, last and middle points of the edge
    //=======================================================================
    private static void GetEdgePoints(in TopoDS_Edge anEdge, in TopoDS_Face aFace, out gp_Pnt fPnt, out gp_Pnt mPnt,
                                      out gp_Pnt lPnt)
    {
        Geom2d_Curve theCurve = BRep_Tool.CurveOnSurface(anEdge, aFace, out double f, out double l)!;
        gp_Pnt2d fPnt2d = theCurve.Value(f);
        gp_Pnt2d lPnt2d = theCurve.Value(l);
        gp_Pnt2d mPnt2d = theCurve.Value(0.5 * (f + l));
        Geom_Surface aSurf = BRep_Tool.Surface(aFace)!;
        fPnt = aSurf.Value(fPnt2d.X(), fPnt2d.Y());
        lPnt = aSurf.Value(lPnt2d.X(), lPnt2d.Y());
        mPnt = aSurf.Value(mPnt2d.X(), mPnt2d.Y());
    }

    //=======================================================================
    // function : FillContours
    // purpose  : fills free boundary contours and faces connected (MapEF)
    //=======================================================================
    private static void FillContours(in TopoDS_Shape aShape, BRepOffset_Analyse Analyser,
                                     NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Contours,
                                     NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapEF)
    {
        NCollection_List<TopoDS_Shape> Edges = new();

        TopExp_Explorer Explo = new(aShape, TopAbs_FACE);
        BRepTools_WireExplorer Wexp = new();

        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(Explo.Current());
            TopoDS_Iterator itf = new(aFace);
            for (; itf.More(); itf.Next())
            {
                TopoDS_Wire aWire = TopoDS.Wire(itf.Value());
                for (Wexp.Init(aWire, aFace); Wexp.More(); Wexp.Next())
                {
                    TopoDS_Edge anEdge = Wexp.Current();
                    if (BRep_Tool.Degenerated(anEdge))
                    {
                        continue;
                    }
                    NCollection_List<BRepOffset_Interval> Lint = Analyser.Type(anEdge);
                    if (!Lint.IsEmpty() && Lint.First().Type() == ChFiDS_FreeBound)
                    {
                        MapEF.Bind(anEdge, aFace);
                        Edges.Append(anEdge);
                    }
                }
            }
        }

        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        while (!Edges.IsEmpty())
        {
            TopoDS_Edge StartEdge = TopoDS.Edge(Edges.First());
            Edges.RemoveFirst();
            TopExp.Vertices(StartEdge, out TopoDS_Vertex StartVertex, out TopoDS_Vertex CurVertex, true);
            NCollection_List<TopoDS_Shape> aContour = new();
            aContour.Append(StartEdge);
            while (!CurVertex.IsSame(StartVertex))
            {
                for (itl.Initialize(Edges); itl.More(); itl.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(itl.Value());
                    TopExp.Vertices(anEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    if (V1.IsSame(CurVertex) || V2.IsSame(CurVertex))
                    {
                        aContour.Append(anEdge);
                        CurVertex = (V1.IsSame(CurVertex)) ? V2 : V1;
                        Edges.Remove(itl);
                        break;
                    }
                }
            }
            Contours.Add(StartVertex, aContour);
        }
    }

    //
    //-----------------------------------------------------------------------
    //
    //=================================================================================================

    /// <summary>An empty algorithm; call <see cref="Initialize"/>.</summary>
    public BRepOffset_MakeOffset()
    {
        myAsDes = new BRepAlgo_AsDes();
    }

    //=================================================================================================

    /// <summary>Initializes the algorithm and makes the offset shape (<see cref="MakeOffsetShape"/>).</summary>
    public BRepOffset_MakeOffset(in TopoDS_Shape S, double Offset, double Tol, BRepOffset_Mode Mode = BRepOffset_Skin,
                                 bool Inter = false, bool SelfInter = false, GeomAbs_JoinType Join = GeomAbs_Arc,
                                 bool Thickening = false, bool RemoveIntEdges = false,
                                 Message_ProgressRange theRange = default)
    {
        myOffset = Offset;
        myTol = Tol;
        myInitialShape = S;
        myShape = S;
        myMode = Mode;
        myInter = Inter;
        mySelfInter = SelfInter;
        myJoin = Join;
        myThickening = Thickening;
        myRemoveIntEdges = RemoveIntEdges;
        myDone = false;
        myAsDes = new BRepAlgo_AsDes();
        myIsLinearizationAllowed = true;

        MakeOffsetShape(theRange);
    }

    //=================================================================================================

    /// <summary>Sets the arguments of the offset and clears the results of an earlier run.</summary>
    public void Initialize(in TopoDS_Shape S, double Offset, double Tol, BRepOffset_Mode Mode = BRepOffset_Skin,
                           bool Inter = false, bool SelfInter = false, GeomAbs_JoinType Join = GeomAbs_Arc,
                           bool Thickening = false, bool RemoveIntEdges = false)
    {
        myOffset = Offset;
        myInitialShape = S;
        myShape = S;
        myTol = Tol;
        myMode = Mode;
        myInter = Inter;
        mySelfInter = SelfInter;
        myJoin = Join;
        myThickening = Thickening;
        myRemoveIntEdges = RemoveIntEdges;
        myIsLinearizationAllowed = true;
        myDone = false;
        myIsPerformSewing = false;
        myIsPlanar = false;
        myEntry = null;
        Clear();
    }

    //=================================================================================================

    /// <summary>Clears the results.</summary>
    public void Clear()
    {
        myOffsetShape.Nullify();
        myInitOffsetFace.Clear();
        myInitOffsetEdge.Clear();
        myImageOffset.Clear();
        myImageVV.Clear();
        myFaces.Clear();
        myOriginalFaces.Clear();
        myFaceOffset.Clear();
        myEdgeIntEdges.Clear();
        myAsDes.Clear();
        myDone = false;
        myGenerated.Clear();
        myResMap.Clear();
    }

    //=================================================================================================

    /// <summary>Changes the flag allowing the linearization.</summary>
    public void AllowLinearization(bool theIsAllowed)
    {
        myIsLinearizationAllowed = theIsAllowed;
    }

    //=================================================================================================

    /// <summary>Adds a closing face; <paramref name="F"/> has to be in the initial shape.</summary>
    public void AddFace(in TopoDS_Face F)
    {

        myOriginalFaces.Add(F);
    }

    //=================================================================================================

    /// <summary>Sets the offset <paramref name="Off"/> on the face <paramref name="F"/>.</summary>
    public void SetOffsetOnFace(in TopoDS_Face F, double Off)
    {
        myFaceOffset.Bind(F, Off);
    }

    //=================================================================================================

    private static void RemoveCorks(ref TopoDS_Shape S, NCollection_IndexedMap<TopoDS_Shape> Faces)
    {
        TopoDS_Compound SS = new();
        BRep_Builder B = new();
        B.MakeCompound(ref SS);
        //-----------------------------------------------------
        // Construction of a shape without caps.
        // and Orientation of caps as in shape S.
        //-----------------------------------------------------
        TopExp_Explorer exp = new(S, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            TopoDS_Shape Cork = exp.Current();
            if (!Faces.Contains(Cork))
            {
                B.Add(SS, Cork);
            }
            else
            {
                Faces.RemoveKey(Cork);
                Faces.Add(Cork); // to reset it with proper orientation.
            }
        }
        S = SS;
    }

    //=================================================================================================

    private static bool IsConnectedShell(in TopoDS_Shape S)
    {
        BRepTools_Quilt Glue = new();
        Glue.Add(S);

        TopoDS_Shape SS = Glue.InternalShells();
        TopExp_Explorer Explo = new(SS, TopAbs_SHELL);
        Explo.Next();
        return !Explo.More();
    }

    //=================================================================================================

    private static void MakeList(NCollection_List<TopoDS_Shape> OffsetFaces, BRepAlgo_Image myInitOffsetFace,
                                 NCollection_IndexedMap<TopoDS_Shape> myFaces)
    {
        NCollection_List<TopoDS_Shape>.Iterator itLOF = new(myInitOffsetFace.Roots());
        for (; itLOF.More(); itLOF.Next())
        {
            TopoDS_Shape Root = itLOF.Value();
            if (!myFaces.Contains(Root))
            {
                if (myInitOffsetFace.HasImage(Root))
                {
                    NCollection_List<TopoDS_Shape>.Iterator aItLS = new(myInitOffsetFace.Image(Root));
                    for (; aItLS.More(); aItLS.Next())
                    {
                        OffsetFaces.Append(aItLS.Value());
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static void EvalMax(in TopoDS_Shape S, ref double Tol)
    {
        TopExp_Explorer exp = new();
        for (exp.Init(S, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            TopoDS_Vertex V = TopoDS.Vertex(exp.Current());
            double TolV = BRep_Tool.Tolerance(V);
            if (TolV > Tol)
            {
                Tol = TolV;
            }
        }
    }

    //=================================================================================================

    private void SetFaces()
    {
        for (int ii = 1; ii <= myOriginalFaces.Extent(); ii++)
        {
            TopoDS_Face aFace = TopoDS.Face(myOriginalFaces[ii]);
            if (myFacePlanfaceMap.Find(aFace, out TopoDS_Shape aPlanface))
            {
                aFace = TopoDS.Face(aPlanface);
            }

            myFaces.Add(aFace);
            //-------------
            // MAJ SD.
            //-------------
            myInitOffsetFace.SetRoot(aFace);
            myInitOffsetFace.Bind(aFace, aFace);
            myImageOffset.SetRoot(aFace);
        }
    }

    //=================================================================================================

    private void SetFacesWithOffset()
    {
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anItmap in myFacePlanfaceMap)
        {
            TopoDS_Face aFace = TopoDS.Face(anItmap.Key);
            TopoDS_Face aPlanface = TopoDS.Face(anItmap.Value);
            if (myFaceOffset.IsBound(aFace))
            {
                double anOffset = myFaceOffset[aFace];
                myFaceOffset.UnBind(aFace);
                myFaceOffset.Bind(aPlanface, anOffset);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Copies the caller's shape at the entry of a run (decision 1005) when it holds frozen TShapes, and maps the closing
    /// faces and the faces with their own offsets onto the copy. Once per <see cref="Initialize"/>.
    /// </summary>
    private void EnterCopy()
    {
        if (myEntry is not null)
        {
            return;
        }
        myEntry = new BRepTools_CopyAtEntry(myShape);
        if (!myEntry.IsCopied())
        {
            return;
        }
        myShape = myEntry.Shape();
        NCollection_IndexedMap<TopoDS_Shape> anOriginalFaces = new(myOriginalFaces);
        myOriginalFaces.Clear();
        for (int i = 1; i <= anOriginalFaces.Extent(); ++i)
        {
            myOriginalFaces.Add(myEntry.Copied(anOriginalFaces[i]));
        }
        NCollection_DataMap<TopoDS_Shape, double> aFaceOffset = new(myFaceOffset);
        myFaceOffset.Clear();
        foreach (KeyValuePair<TopoDS_Shape, double> anIt in aFaceOffset)
        {
            myFaceOffset.Bind(myEntry.Copied(anIt.Key), anIt.Value);
        }
    }

    /// <summary>
    /// Ends a run: the copied sub-shapes left unchanged are the caller's own again in the result (decision 1005).
    /// </summary>
    private void FinishCopy()
    {
        if (myEntry is null || !myEntry.IsCopied() || myOffsetShape.IsNull())
        {
            return;
        }
        myOffsetShape = myEntry.Finish(myOffsetShape);
    }

    /// <summary>The working copy of a sub-shape of the caller's shape (itself when the shape wasn't copied).</summary>
    internal TopoDS_Shape InternalCopied(in TopoDS_Shape theS) => myEntry is null ? theS : myEntry.Copied(theS);

    /// <summary>What stands in the result for a shape of the run (see <see cref="BRepTools_CopyAtEntry.Final"/>).</summary>
    internal TopoDS_Shape InternalFinal(in TopoDS_Shape theS) =>
        myEntry is null || !myEntry.IsCopied() || theS.IsNull() ? theS : myEntry.Final(theS);

    //=================================================================================================

    /// <summary>Makes the offset shape (a shell, or a solid for a solid input).</summary>
    public void MakeOffsetShape(Message_ProgressRange theRange = default)
    {
        EnterCopy();
        InternalMakeOffsetShape(theRange);
        FinishCopy();
    }

    private void InternalMakeOffsetShape(Message_ProgressRange theRange)
    {
        myDone = false;
        //

        // check if shape consists of only planar faces
        myIsPlanar = IsPlanar();

        SetFaces();
        SetFacesWithOffset();

        BuildFaceComp();

        //------------------------------------------
        // Construction of myShape without caps.
        //------------------------------------------
        if (!myFaces.IsEmpty())
        {
            RemoveCorks(ref myShape, myOriginalFaces);
            TopoDS_Shape aFaceComp = myFaceComp;
            RemoveCorks(ref aFaceComp, myFaces);
            myFaceComp = TopoDS.Compound(aFaceComp);
        }

        using Message_ProgressScope aPS = new(theRange, "Making offset shape", 100);

        NCollection_Array1<double> aSteps = new(0, PIOperation_Last - 1);
        analyzeProgress(100.0, aSteps);

        if (!CheckInputData(aPS.Next(aSteps[PIOperation_CheckInputData])) || myError != BRepOffset_NoError)
        {
            // There is error in input data.
            // Check Error() method.
            return;
        }
        myError = BRepOffset_NoError;
        TopAbs_State Side = TopAbs_IN;
        if (myOffset < 0.0)
        {
            Side = TopAbs_OUT;
        }

        // ------------
        // Preanalyse.
        // ------------
        EvalMax(myShape, ref myTol);
        // There are possible second variant: analytical continuation of arcsin.
        double TolAngleCoeff = Math.Min(myTol / (Math.Abs(myOffset * 0.5) + Precision.Confusion()), 1.0);
        double TolAngle = 4 * Math.Asin(TolAngleCoeff);
        if ((myJoin == GeomAbs_Intersection) && myInter && myIsPlanar)
        {
            myAnalyse.SetOffsetValue(myOffset);
            myAnalyse.SetFaceOffsetMap(myFaceOffset);
        }
        myAnalyse.Perform(myFaceComp, TolAngle, aPS.Next(aSteps[PIOperation_Analyse]));
        TopExp_Explorer anEExp = new(myFaceComp, TopAbs_EDGE);
        for (; anEExp.More(); anEExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anEExp.Current());
            NCollection_List<BRepOffset_Interval> aLI = myAnalyse.Type(anE);
            if (aLI.IsEmpty())
            {
                continue;
            }
            if (aLI.Last().Type() == ChFiDS_Mixed)
            {
                myError = BRepOffset_MixedConnectivity;
                return;
            }
        }
        if (!aPS.More())
        {
            myError = BRepOffset_UserBreak;
            return;
        }
        //---------------------------------------------------
        // Construction of Offset from preanalysis.
        //---------------------------------------------------
        //----------------------------
        // MaJ of SD Face - Offset
        //----------------------------
        UpdateFaceOffset();

        if (myJoin == GeomAbs_Arc)
        {
            BuildOffsetByArc(aPS.Next(aSteps[PIOperation_BuildOffsetBy]));
        }
        else if (myJoin == GeomAbs_Intersection)
        {
            BuildOffsetByInter(aPS.Next(aSteps[PIOperation_BuildOffsetBy]));
        }
        if (myError != BRepOffset_NoError)
        {
            return;
        }
        //-----------------
        // Auto unwinding.
        //-----------------
        // if (mySelfInter)  SelfInter(Modif);
        //-----------------
        // Intersection 3d .
        //-----------------
        // OCCT's aPSInter.SetName(...) names the scope "Connect offset faces by arc" or "... by intersection".
        using Message_ProgressScope aPSInter = new(aPS.Next(aSteps[PIOperation_Intersection]),
                                                   (myJoin == GeomAbs_Arc) ? "Connect offset faces by arc"
                                                                           : "Connect offset faces by intersection",
                                                   100);

        BRepOffset_Inter3d Inter = new(myAsDes, Side, myTol);
        Intersection3D(Inter, aPSInter.Next(90));
        if (myError != BRepOffset_NoError)
        {
            return;
        }
        //-----------------
        // Intersection2D
        //-----------------
        NCollection_IndexedMap<TopoDS_Shape> Modif = Inter.TouchedFaces();
        NCollection_IndexedMap<TopoDS_Shape> NewEdges = Inter.NewEdges();

        if (!Modif.IsEmpty())
        {
            Intersection2D(Modif, NewEdges, aPSInter.Next(4));
            if (myError != BRepOffset_NoError)
            {
                return;
            }
        }

        //-------------------------------------------------------
        // Unwinding 2D and reconstruction of modified faces
        //----------------------------------------------------
        MakeLoops(Modif, aPSInter.Next(4));
        if (myError != BRepOffset_NoError)
        {
            return;
        }
        //-----------------------------------------------------
        // Reconstruction of non modified faces sharing
        // reconstructed edges
        //------------------------------------------------------
        if (!Modif.IsEmpty())
        {
            MakeFaces(Modif, aPSInter.Next(2));
            if (myError != BRepOffset_NoError)
            {
                return;
            }
        }

        aPSInter.Close();

        if (myThickening)
        {
            MakeMissingWalls(aPS.Next(aSteps[PIOperation_MakeMissingWalls]));
            if (myError != BRepOffset_NoError)
            {
                return;
            }
        }

        //-------------------------
        // Construction of shells.
        //-------------------------
        MakeShells(aPS.Next(aSteps[PIOperation_MakeShells]));
        if (myError != BRepOffset_NoError)
        {
            return;
        }
        if (myOffsetShape.IsNull())
        {
            // not done
            myDone = false;
            return;
        }
        //--------------
        // Unwinding 3D.
        //--------------
        SelectShells();
        //----------------------------------
        // Remove INTERNAL edges if necessary
        //----------------------------------
        if (myRemoveIntEdges)
        {
            RemoveInternalEdges();
        }
        //----------------------------------
        // Coding of regularities.
        //----------------------------------
        EncodeRegularity();
        //----------------------------------
        // Replace roots in history maps
        //----------------------------------
        ReplaceRoots();
        //----------------------
        // Creation of solids.
        //----------------------
        MakeSolid(aPS.Next(aSteps[PIOperation_MakeSolid]));
        if (myError != BRepOffset_NoError)
        {
            return;
        }
        //-----------------------------
        // MAJ Tolerance edge and Vertex
        // ----------------------------
        if (!myOffsetShape.IsNull())
        {
            if (myThickening)
            {
                UpdateTolerance(ref myOffsetShape, myFaces, myShape);
            }
            else
            {
                TopoDS_Shape aDummy = new();
                UpdateTolerance(ref myOffsetShape, myFaces, aDummy);
            }
            BRepLib.UpdateTolerances(myOffsetShape);
        }

        CorrectConicalFaces();

        // Result solid should be computed in MakeOffset scope.
        if (myThickening && myIsPerformSewing)
        {
            BRepBuilderAPI_Sewing aSew = new(myTol);
            aSew.Add(myOffsetShape);
            aSew.Perform(aPS.Next(aSteps[PIOperation_Sewing] / 2.0));
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            myOffsetShape = aSew.InternalSewedShape();

            // Rebuild solid.
            // Offset shape expected to be really closed after sewing.
            myOffsetShape.Closed(true);
            MakeSolid(aPS.Next(aSteps[PIOperation_Sewing] / 2.0));
            if (myError != BRepOffset_NoError)
            {
                return;
            }
        }

        myDone = true;
    }

    //=================================================================================================

    /// <summary>
    /// Makes the thick solid: the offset shell of the shape without its closing faces (<see cref="AddFace"/>), closed
    /// by the shape's faces.
    /// </summary>
    public void MakeThickSolid(Message_ProgressRange theRange = default)
    {
        EnterCopy();
        InternalMakeThickSolid(theRange);
        FinishCopy();
    }

    private void InternalMakeThickSolid(Message_ProgressRange theRange)
    {
        //--------------------------------------------------------------
        // Construction of shell parallel to shell (initial without cap).
        //--------------------------------------------------------------
        InternalMakeOffsetShape(theRange);

        if (!myDone)
        {
            // Save return code and myDone state.
            return;
        }

        //--------------------------------------------------------------------
        // Construction of a solid with the initial shell, parallel shell
        // limited by caps.
        //--------------------------------------------------------------------
        if (!myFaces.IsEmpty())
        {
            TopoDS_Solid Res = new();
            TopExp_Explorer exp = new();
            BRep_Builder B = new();
            int NbF = myFaces.Extent();

            B.MakeSolid(ref Res);

            BRepTools_Quilt Glue = new();
            for (exp.Init(myShape, TopAbs_FACE); exp.More(); exp.Next())
            {
                NbF++;
                Glue.Add(exp.Current());
            }
            bool YaResult = false;
            if (!myOffsetShape.IsNull())
            {
                for (exp.Init(myOffsetShape, TopAbs_FACE); exp.More(); exp.Next())
                {
                    YaResult = true;
                    Glue.Add(exp.Current().Reversed());
                }
                // OCCT_DEBUG: "OffsetShape does not contain a FACES." left out.
            }
            // OCCT_DEBUG: "OffsetShape is null!" left out.

            if (!YaResult)
            {
                myDone = false;
                myError = BRepOffset_UnknownError;
                return;
            }

            myOffsetShape = Glue.InternalShells();
            for (exp.Init(myOffsetShape, TopAbs_SHELL); exp.More(); exp.Next())
            {
                B.Add(Res, exp.Current());
            }
            Res.Closed(true);
            myOffsetShape = Res;

            // Test of Validity of the result of thick Solid
            // more face than the initial solid.
            int NbOF = 0;
            for (exp.Init(myOffsetShape, TopAbs_FACE); exp.More(); exp.Next())
            {
                NbOF++;
            }
            if (NbOF < NbF)
            {
                myDone = false;
                myError = BRepOffset_UnknownError;
                return;
            }
            if (NbOF == NbF)
            {
                myOffset = 0;
            }
        }

        if (myOffset > 0)
        {
            myOffsetShape.Reverse();
        }

        myDone = true;
    }

    //=================================================================================================

    /// <summary>True if the last run succeeded.</summary>
    public bool IsDone() => myDone;

    //=================================================================================================

    /// <summary>The error of the last run.</summary>
    public BRepOffset_Error Error() => myError;

    //=================================================================================================

    /// <summary>The result (frozen).</summary>
    public TopoDS_Shape Shape()
    {
        myOffsetShape.TShape()?.FreezeGraph();
        return myOffsetShape;
    }

    /// <summary>The result, unfrozen, for ported code.</summary>
    internal TopoDS_Shape InternalShape() => myOffsetShape;

    /// <summary>The initial shape, as given.</summary>
    public TopoDS_Shape InitShape() => myInitialShape;

    //=================================================================================================

    private void MakeOffsetFaces(NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> theMapSF,
                                 Message_ProgressRange theRange)
    {
        double aCurOffset;
        NCollection_List<TopoDS_Shape> aLF = new();
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> ShapeTgt = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new();
        //
        bool OffsetOutside = (myOffset > 0.0);
        //
        BRepLib.SortFaces(myFaceComp, aLF);
        //
        using Message_ProgressScope aPS = new(theRange, "Making offset faces", aLF.Extent());
        aItLF.Initialize(aLF);
        for (; aItLF.More(); aItLF.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Face aF = TopoDS.Face(aItLF.Value());
            aCurOffset = myFaceOffset.IsBound(aF) ? myFaceOffset[aF] : myOffset;
            BRepOffset_Offset OF = new(aF, aCurOffset, ShapeTgt, OffsetOutside, myJoin);
            NCollection_List<TopoDS_Shape> Let = new();
            myAnalyse.Edges(aF, ChFiDS_Tangential, Let);
            NCollection_List<TopoDS_Shape>.Iterator itl = new(Let);
            for (; itl.More(); itl.Next())
            {
                TopoDS_Edge Cur = TopoDS.Edge(itl.Value());
                if (!ShapeTgt.IsBound(Cur) && !myAnalyse.HasGenerated(Cur))
                {
                    TopoDS_Shape aLocalShape = OF.InternalGenerated(Cur);
                    TopoDS_Edge OTE = TopoDS.Edge(aLocalShape);
                    ShapeTgt.Bind(Cur, OF.InternalGenerated(Cur));
                    TopExp.Vertices(Cur, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    TopExp.Vertices(OTE, out TopoDS_Vertex OV1, out TopoDS_Vertex OV2);
                    NCollection_List<TopoDS_Shape> LE = new();
                    if (!ShapeTgt.IsBound(V1))
                    {
                        myAnalyse.Edges(V1, ChFiDS_Tangential, LE);
                        NCollection_List<TopoDS_Shape> LA = myAnalyse.Ancestors(V1);
                        if (LE.Extent() == LA.Extent())
                        {
                            ShapeTgt.Bind(V1, OV1);
                        }
                    }
                    if (!ShapeTgt.IsBound(V2))
                    {
                        LE.Clear();
                        myAnalyse.Edges(V2, ChFiDS_Tangential, LE);
                        NCollection_List<TopoDS_Shape> LA = myAnalyse.Ancestors(V2);
                        if (LE.Extent() == LA.Extent())
                        {
                            ShapeTgt.Bind(V2, OV2);
                        }
                    }
                }
            }
            theMapSF.Bind(aF, OF);
        }
        //
        NCollection_List<TopoDS_Shape> aNewFaces = myAnalyse.NewFaces();
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aNewFaces); it.More(); it.Next())
        {
            TopoDS_Face aF = TopoDS.Face(it.Value());
            BRepOffset_Offset OF = new(aF, 0.0, ShapeTgt, OffsetOutside, myJoin);
            theMapSF.Bind(aF, OF);
        }
    }

    //=================================================================================================

    private void BuildOffsetByInter(Message_ProgressRange theRange)
    {
        // OCCT_DEBUG: the chronometer is left out.

        using Message_ProgressScope aPSOuter = new(theRange, "Connect offset faces by intersection", 100);

        // just for better management and visualization of the progress steps
        // define a nested enum listing all the steps of the current method.
        const int BuildOffsetByInter_MakeOffsetFaces = 0;
        const int BuildOffsetByInter_ConnexIntByInt = 1;
        const int BuildOffsetByInter_ContextIntByInt = 2;
        const int BuildOffsetByInter_IntersectEdges = 3;
        const int BuildOffsetByInter_CompleteEdgesIntersection = 4;
        const int BuildOffsetByInter_BuildFaces = 5;
        const int BuildOffsetByInter_FillHistoryForOffsets = 6;
        const int BuildOffsetByInter_FillHistoryForDeepenings = 7;
        const int BuildOffsetByInter_Last = 8;

        double aNbFaces = myFaceComp.NbChildren() + myAnalyse.NewFaces().Extent() + myFaces.Extent();
        double anOffsetsPart = (myFaceComp.NbChildren() + myAnalyse.NewFaces().Extent()) / aNbFaces;
        double aDeepeningsPart = myFaces.Extent() / aNbFaces;

        NCollection_Array1<double> aSteps = new(0, BuildOffsetByInter_Last - 1);
        {
            aSteps.Init(0);

            bool isInter = myJoin == GeomAbs_Intersection;
            double aFaceInter = isInter ? 25.0 : 50.0;
            double aBuildFaces = isInter ? 50.0 : 25.0;
            aSteps[BuildOffsetByInter_MakeOffsetFaces] = 5.0;
            aSteps[BuildOffsetByInter_ConnexIntByInt] = aFaceInter * anOffsetsPart;
            aSteps[BuildOffsetByInter_ContextIntByInt] = aFaceInter * aDeepeningsPart;
            aSteps[BuildOffsetByInter_IntersectEdges] = 10.0;
            aSteps[BuildOffsetByInter_CompleteEdgesIntersection] = 5.0;
            aSteps[BuildOffsetByInter_BuildFaces] = aBuildFaces;
            aSteps[BuildOffsetByInter_FillHistoryForOffsets] = 5.0 * anOffsetsPart;
            aSteps[BuildOffsetByInter_FillHistoryForDeepenings] = 5.0 * aDeepeningsPart;
            normalizeSteps(100.0, aSteps);
        }

        TopExp_Explorer Exp = new(), Exp2 = new(), ExpC = new();
        NCollection_List<TopoDS_Shape>.Iterator itLF = new();

        //--------------------------------------------------------
        // Construction of faces parallel to initial faces
        //--------------------------------------------------------
        NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> MapSF = new(TopTools_ShapeMapHasher.Instance);
        MakeOffsetFaces(MapSF, aPSOuter.Next(aSteps[BuildOffsetByInter_MakeOffsetFaces]));
        if (!aPSOuter.More())
        {
            myError = BRepOffset_UserBreak;
            return;
        }
        //--------------------------------------------------------------------
        // MES   : Map of OffsetShape -> Extended Shapes.
        // Build : Map of Initial SS  -> OffsetShape build by Inter.
        //                               can be an edge or a compound of edges
        //---------------------------------------------------------------------
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MES = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Build = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> Failed = new();
        TopAbs_State Side = TopAbs_IN;
        BRepAlgo_AsDes AsDes = new();

        //-------------------------------------------------------------------
        // Extension of faces and calculation of new edges of intersection.
        //-------------------------------------------------------------------
        bool ExtentContext = false;
        if (myOffset > 0)
        {
            ExtentContext = true;
        }

        BRepOffset_Inter3d Inter3 = new(AsDes, Side, myTol);
        // Intersection between parallel faces
        Inter3.ConnexIntByInt(myFaceComp, MapSF, myAnalyse, MES, Build, Failed,
                              aPSOuter.Next(aSteps[BuildOffsetByInter_ConnexIntByInt]), myIsPlanar);
        if (!aPSOuter.More())
        {
            myError = BRepOffset_UserBreak;
            return;
        }
        // Intersection with caps.
        Inter3.ContextIntByInt(myFaces, ExtentContext, MapSF, myAnalyse, MES, Build, Failed,
                               aPSOuter.Next(aSteps[BuildOffsetByInter_ContextIntByInt]), myIsPlanar);
        if (!aPSOuter.More())
        {
            myError = BRepOffset_UserBreak;
            return;
        }

        NCollection_List<TopoDS_Shape> aLFaces = new();
        for (Exp.Init(myFaceComp, TopAbs_FACE); Exp.More(); Exp.Next())
        {
            aLFaces.Append(Exp.Current());
        }
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(myAnalyse.NewFaces()); it.More(); it.Next())
        {
            aLFaces.Append(it.Value());
        }
        //---------------------------------------------------------------------------------
        // Extension of neighbor edges of new edges and intersection between neighbors.
        //--------------------------------------------------------------------------------
        BRepAlgo_AsDes AsDes2d = new();
        IntersectEdges(aLFaces, MapSF, MES, Build, ref AsDes, ref AsDes2d,
                       aPSOuter.Next(aSteps[BuildOffsetByInter_IntersectEdges]));
        if (myError != BRepOffset_NoError)
        {
            return;
        }
        //-----------------------------------------------------------
        // Great restriction of new edges and update of AsDes.
        //------------------------------------------ ----------------
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEdgesOrigins =
            new(TopTools_ShapeMapHasher.Instance); // offset edge - initial edges
        NCollection_IndexedMap<TopoDS_Shape> NewEdges = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aETrimEInf =
            new(TopTools_ShapeMapHasher.Instance); // trimmed - not trimmed edges
        //
        // Map of edges obtained after FACE-FACE (offsetted) intersection.
        // Key1 is edge trimmed by intersection points with other edges;
        // Item is not-trimmed edge.
        if (!TrimEdges(myFaceComp, myOffset, myAnalyse, MapSF, MES, Build, ref AsDes, ref AsDes2d, NewEdges,
                       aETrimEInf, anEdgesOrigins))
        {
            myError = BRepOffset_CannotTrimEdges;
            return;
        }
        //
        //---------------------------------
        // Intersection 2D on //
        //---------------------------------
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVV =
            new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aFacesOrigins =
            new(TopTools_ShapeMapHasher.Instance); // offset face - initial face
        NCollection_List<TopoDS_Shape> LFE = new();
        BRepAlgo_Image IMOE = new();
        GetEnlargedFaces(aLFaces, MapSF, MES, aFacesOrigins, IMOE, LFE);
        //
        NCollection_List<TopoDS_Shape>.Iterator itLFE = new(LFE);
        using Message_ProgressScope aPS2d =
            new(aPSOuter.Next(aSteps[BuildOffsetByInter_CompleteEdgesIntersection]), null, 2);
        using Message_ProgressScope aPS2dOffsets = new(aPS2d.Next(2.0 * anOffsetsPart), null, LFE.Extent());
        for (; itLFE.More(); itLFE.Next())
        {
            if (!aPS2dOffsets.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Face NEF = TopoDS.Face(itLFE.Value());
            double aCurrFaceTol = BRep_Tool.Tolerance(NEF);
            BRepOffset_Inter2d.Compute(AsDes, NEF, NewEdges, aCurrFaceTol, myEdgeIntEdges, aDMVV,
                                       aPS2dOffsets.Next());
        }
        //----------------------------------------------
        // Intersections 2d on caps.
        //----------------------------------------------
        int i;
        using Message_ProgressScope aPS2dCaps = new(aPS2d.Next(2.0 * aDeepeningsPart), null, myFaces.Extent());
        for (i = 1; i <= myFaces.Extent(); i++)
        {
            if (!aPS2dCaps.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Face Cork = TopoDS.Face(myFaces[i]);
            double aCurrFaceTol = BRep_Tool.Tolerance(Cork);
            BRepOffset_Inter2d.Compute(AsDes, Cork, NewEdges, aCurrFaceTol, myEdgeIntEdges, aDMVV, aPS2dCaps.Next());
        }
        //
        BRepOffset_Inter2d.FuseVertices(aDMVV, AsDes, myImageVV);
        //-------------------------------
        // Unwinding of extended Faces.
        //-------------------------------
        //
        NCollection_Map<TopoDS_Shape> aMFDone = new(TopTools_ShapeMapHasher.Instance);
        //
        if ((myJoin == GeomAbs_Intersection) && myInter && myIsPlanar)
        {
            BuildSplitsOfExtendedFaces(LFE, myAnalyse, AsDes, anEdgesOrigins, aFacesOrigins, aETrimEInf, IMOE,
                                       aPSOuter.Next(aSteps[BuildOffsetByInter_BuildFaces]));
            if (myError != BRepOffset_NoError)
            {
                return;
            }
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLF = new(LFE);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aS = aItLF.Value();
                aMFDone.Add(aS);
            }
        }
        else
        {
            myMakeLoops.Build(LFE, AsDes, IMOE, myImageVV, aPSOuter.Next(aSteps[BuildOffsetByInter_BuildFaces]));
            if (!aPSOuter.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
        }
        //
        // OCCT_DEBUG: COES is left out.
        //---------------------------
        // MAJ SD. for faces //
        //---------------------------
        using Message_ProgressScope aPSHist = new(aPSOuter.Next(aSteps[BuildOffsetByInter_FillHistoryForOffsets]),
                                                  "Fill history for offset faces", aLFaces.Extent());
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLFaces); it.More(); it.Next(), aPSHist.Next())
        {
            if (!aPSHist.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Shape FI = it.Value();
            myInitOffsetFace.SetRoot(FI);
            TopoDS_Face OF = MapSF[FI].InternalFace();
            if (MES.IsBound(OF))
            {
                OF = TopoDS.Face(MES[OF]);
                if (IMOE.HasImage(OF))
                {
                    NCollection_List<TopoDS_Shape> LOFE = IMOE.Image(OF);
                    myInitOffsetFace.Bind(FI, LOFE);
                    for (itLF.Initialize(LOFE); itLF.More(); itLF.Next())
                    {
                        TopoDS_Shape OFE = itLF.Value();
                        myImageOffset.SetRoot(OFE);
                        NCollection_Map<TopoDS_Shape> View = new(TopTools_ShapeMapHasher.Instance);
                        for (Exp2.Init(OFE.Oriented(TopAbs_FORWARD), TopAbs_EDGE); Exp2.More(); Exp2.Next())
                        {
                            TopoDS_Edge COE = TopoDS.Edge(Exp2.Current());

                            myAsDes.Add(OFE, COE);
                            if (View.Add(COE))
                            {
                                if (!myAsDes.HasDescendant(COE))
                                {
                                    TopExp.Vertices(COE, out TopoDS_Vertex CV1, out TopoDS_Vertex CV2);
                                    if (!CV1.IsNull())
                                    {
                                        myAsDes.Add(COE, CV1.Oriented(TopAbs_FORWARD));
                                    }
                                    if (!CV2.IsNull())
                                    {
                                        myAsDes.Add(COE, CV2.Oriented(TopAbs_REVERSED));
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (aMFDone.Contains(OF))
                    {
                        continue;
                    }
                    //
                    myInitOffsetFace.Bind(FI, OF);
                    myImageOffset.SetRoot(OF);
                    NCollection_List<TopoDS_Shape> LE = AsDes.Descendant(OF);
                    for (itLF.Initialize(LE); itLF.More(); itLF.Next())
                    {
                        TopoDS_Edge OE = TopoDS.Edge(itLF.Value());
                        if (IMOE.HasImage(OE))
                        {
                            NCollection_List<TopoDS_Shape> LOE = IMOE.Image(OE);
                            NCollection_List<TopoDS_Shape>.Iterator itLOE = new(LOE);
                            for (; itLOE.More(); itLOE.Next())
                            {
                                TopoDS_Shape aLocalShape = itLOE.Value().Oriented(OE.Orientation());
                                TopoDS_Edge COE = TopoDS.Edge(aLocalShape);
                                //              const TopoDS_Edge& COE =
                                //              TopoDS::Edge(itLOE.Value().Oriented(OE.Orientation()));
                                myAsDes.Add(OF, COE);

                                if (!myAsDes.HasDescendant(COE))
                                {
                                    TopExp.Vertices(COE, out TopoDS_Vertex CV1, out TopoDS_Vertex CV2);
                                    if (!CV1.IsNull())
                                    {
                                        myAsDes.Add(COE, CV1.Oriented(TopAbs_FORWARD));
                                    }
                                    if (!CV2.IsNull())
                                    {
                                        myAsDes.Add(COE, CV2.Oriented(TopAbs_REVERSED));
                                    }
                                }
                            }
                        }
                        else
                        {
                            myAsDes.Add(OF, OE);

                            NCollection_List<TopoDS_Shape> LV = AsDes.Descendant(OE);
                            myAsDes.Add(OE, LV);
                        }
                    }
                }
            }
            else
            {
                myInitOffsetFace.Bind(FI, OF);
                myImageOffset.SetRoot(OF);
                NCollection_Map<TopoDS_Shape> View = new(TopTools_ShapeMapHasher.Instance);
                for (Exp2.Init(OF.Oriented(TopAbs_FORWARD), TopAbs_EDGE); Exp2.More(); Exp2.Next())
                {

                    TopoDS_Edge COE = TopoDS.Edge(Exp2.Current());
                    myAsDes.Add(OF, COE);

                    if (View.Add(Exp2.Current()))
                    {
                        if (!myAsDes.HasDescendant(COE))
                        {
                            TopExp.Vertices(COE, out TopoDS_Vertex CV1, out TopoDS_Vertex CV2);
                            if (!CV1.IsNull())
                            {
                                myAsDes.Add(COE, CV1.Oriented(TopAbs_FORWARD));
                            }
                            if (!CV2.IsNull())
                            {
                                myAsDes.Add(COE, CV2.Oriented(TopAbs_REVERSED));
                            }
                        }
                    }
                }
            }
        }
        //  Modified by skv - Tue Mar 15 16:20:43 2005
        // Add methods for supporting history.
        NCollection_Map<TopoDS_Shape> aMapEdges = new(TopTools_ShapeMapHasher.Instance);

        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLFaces); it.More(); it.Next())
        {
            TopoDS_Shape aFaceRef = it.Value();
            Exp2.Init(aFaceRef.Oriented(TopAbs_FORWARD), TopAbs_EDGE);

            for (; Exp2.More(); Exp2.Next())
            {
                TopoDS_Shape anEdgeRef = Exp2.Current();

                if (aMapEdges.Add(anEdgeRef))
                {
                    myInitOffsetEdge.SetRoot(anEdgeRef);
                    if (Build.IsBound(anEdgeRef))
                    {
                        TopoDS_Shape aNewShape = Build[anEdgeRef];

                        if (aNewShape.ShapeType() == TopAbs_EDGE)
                        {
                            if (IMOE.HasImage(aNewShape))
                            {
                                NCollection_List<TopoDS_Shape> aListNewE = IMOE.Image(aNewShape);

                                myInitOffsetEdge.Bind(anEdgeRef, aListNewE);
                            }
                            else
                            {
                                myInitOffsetEdge.Bind(anEdgeRef, aNewShape);
                            }
                        }
                        else
                        { // aNewShape != TopAbs_EDGE
                            NCollection_List<TopoDS_Shape> aListNewEdge = new();

                            for (ExpC.Init(aNewShape, TopAbs_EDGE); ExpC.More(); ExpC.Next())
                            {
                                TopoDS_Shape aResEdge = ExpC.Current();

                                if (IMOE.HasImage(aResEdge))
                                {
                                    NCollection_List<TopoDS_Shape> aListNewE = IMOE.Image(aResEdge);
                                    NCollection_List<TopoDS_Shape>.Iterator aNewEIter = new(aListNewE);

                                    for (; aNewEIter.More(); aNewEIter.Next())
                                    {
                                        aListNewEdge.Append(aNewEIter.Value());
                                    }
                                }
                                else
                                {
                                    aListNewEdge.Append(aResEdge);
                                }
                            }

                            myInitOffsetEdge.Bind(anEdgeRef, aListNewEdge);
                        }
                    }
                    else
                    { // Free boundary.
                        TopoDS_Shape aNewEdge = MapSF[aFaceRef].InternalGenerated(anEdgeRef);

                        if (MES.IsBound(aNewEdge))
                        {
                            aNewEdge = MES[aNewEdge];
                        }

                        if (IMOE.HasImage(aNewEdge))
                        {
                            NCollection_List<TopoDS_Shape> aListNewE = IMOE.Image(aNewEdge);

                            myInitOffsetEdge.Bind(anEdgeRef, aListNewE);
                        }
                        else
                        {
                            myInitOffsetEdge.Bind(anEdgeRef, aNewEdge);
                        }
                    }
                }
            }
        }
        //  Modified by skv - Tue Mar 15 16:20:43 2005

        //---------------------------
        // MAJ SD. for caps
        //---------------------------
        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> View;
        using Message_ProgressScope aPSHist2 = new(aPSOuter.Next(aSteps[BuildOffsetByInter_FillHistoryForDeepenings]),
                                                   "Fill history for deepening faces", myFaces.Extent());
        for (i = 1; i <= myFaces.Extent(); i++, aPSHist2.Next())
        {
            if (!aPSHist2.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Shape Cork = myFaces[i];
            NCollection_List<TopoDS_Shape> LE = AsDes.Descendant(Cork);
            for (itLF.Initialize(LE); itLF.More(); itLF.Next())
            {
                TopoDS_Edge OE = TopoDS.Edge(itLF.Value());
                if (IMOE.HasImage(OE))
                {
                    NCollection_List<TopoDS_Shape> LOE = IMOE.Image(OE);
                    NCollection_List<TopoDS_Shape>.Iterator itLOE = new(LOE);
                    for (; itLOE.More(); itLOE.Next())
                    {
                        TopoDS_Edge COE = TopoDS.Edge(itLOE.Value());
                        myAsDes.Add(Cork, COE.Oriented(OE.Orientation()));

                        if (!myAsDes.HasDescendant(COE))
                        {
                            TopExp.Vertices(COE, out TopoDS_Vertex CV1, out TopoDS_Vertex CV2);
                            if (!CV1.IsNull())
                            {
                                myAsDes.Add(COE, CV1.Oriented(TopAbs_FORWARD));
                            }
                            if (!CV2.IsNull())
                            {
                                myAsDes.Add(COE, CV2.Oriented(TopAbs_REVERSED));
                            }
                        }
                    }
                }
                else
                {
                    myAsDes.Add(Cork, OE);
                    if (AsDes.HasDescendant(OE))
                    {
                        myAsDes.Add(OE, AsDes.Descendant(OE));
                    }
                }
            }
        }

        // OCCT_DEBUG: DEBVerticesControl and the chronometer are left out.
    }

    //=================================================================================================

    private void ReplaceRoots()
    {
        // Replace the artificial faces and edges in InitOffset maps with the original ones.
        NCollection_Map<TopoDS_Shape> View = new(TopTools_ShapeMapHasher.Instance);
        for (TopExp_Explorer anExpF = new(myFaceComp, TopAbs_EDGE); anExpF.More(); anExpF.Next())
        {
            TopoDS_Shape aF = anExpF.Current();
            for (TopExp_Explorer anExpE = new(aF, TopAbs_EDGE); anExpE.More(); anExpE.Next())
            {
                TopoDS_Shape aE = anExpE.Current();
                if (!View.Add(aE))
                {
                    continue;
                }

                TopoDS_Shape aFGen = myAnalyse.Generated(aE);
                if (aFGen.IsNull())
                {
                    continue;
                }

                myInitOffsetFace.ReplaceRoot(aFGen, aE);

                for (TopoDS_Iterator itV = new(aE); itV.More(); itV.Next())
                {
                    TopoDS_Shape aV = itV.Value();
                    if (!View.Add(aV))
                    {
                        continue;
                    }

                    TopoDS_Shape aEGen = myAnalyse.Generated(aV);
                    if (aEGen.IsNull())
                    {
                        continue;
                    }

                    myInitOffsetEdge.ReplaceRoot(aEGen, aV);
                }
            }
        }
    }

    //=======================================================================
    // function : BuildFaceComp
    // purpose  : Make a compound containing actual faces (including planar faces instead of their
    // originals)
    //=======================================================================
    private void BuildFaceComp()
    {
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref myFaceComp);
        TopExp_Explorer anExplo = new(myShape, TopAbs_FACE);
        for (; anExplo.More(); anExplo.Next())
        {
            TopoDS_Shape aFace = anExplo.Current();
            TopAbs_Orientation anOr = aFace.Orientation();
            if (myFacePlanfaceMap.Find(aFace, out TopoDS_Shape aPlanface))
            {
                aFace = aPlanface;
            }
            aBB.Add(myFaceComp, aFace.Oriented(anOr));
        }
    }

    //=================================================================================================

    private void BuildOffsetByArc(Message_ProgressRange theRange)
    {
        // OCCT_DEBUG: the chronometer is left out.

        TopExp_Explorer Exp = new();
        NCollection_List<TopoDS_Shape>.Iterator itLF = new();
        NCollection_Map<TopoDS_Shape> Done = new(TopTools_ShapeMapHasher.Instance);
        using Message_ProgressScope aPSOuter = new(theRange, null, 10);
        //--------------------------------------------------------
        // Construction of faces parallel to initial faces
        //--------------------------------------------------------
        NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> MapSF = new(TopTools_ShapeMapHasher.Instance);
        MakeOffsetFaces(MapSF, aPSOuter.Next());
        if (myError != BRepOffset_NoError)
        {
            return;
        }
        //--------------------------------------------------------
        // Construction of tubes on edge.
        //--------------------------------------------------------
        ChFiDS_TypeOfConcavity OT = ChFiDS_Convex;
        if (myOffset < 0.0)
        {
            OT = ChFiDS_Concave;
        }

        using Message_ProgressScope aPS1 = new(aPSOuter.Next(4), "Constructing tubes on edges", 1, true);
        for (Exp.Init(myFaceComp, TopAbs_EDGE); Exp.More(); Exp.Next(), aPS1.Next())
        {
            if (!aPS1.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Edge E = TopoDS.Edge(Exp.Current());
            if (Done.Add(E))
            {
                NCollection_List<TopoDS_Shape> Anc = myAnalyse.Ancestors(E);
                if (Anc.Extent() == 2)
                {
                    NCollection_List<BRepOffset_Interval> L = myAnalyse.Type(E);
                    if (!L.IsEmpty() && L.First().Type() == OT)
                    {
                        double CurOffset = myOffset;
                        if (myFaceOffset.IsBound(Anc.First()))
                        {
                            CurOffset = myFaceOffset[Anc.First()];
                        }
                        TopoDS_Shape aLocalShapeGen = MapSF[Anc.First()].InternalGenerated(E);
                        TopoDS_Edge EOn1 = TopoDS.Edge(aLocalShapeGen);
                        aLocalShapeGen = MapSF[Anc.Last()].InternalGenerated(E);
                        TopoDS_Edge EOn2 = TopoDS.Edge(aLocalShapeGen);
                        //          TopoDS_Edge EOn1 = TopoDS::Edge(MapSF(Anc.First()).Generated(E));
                        //          TopoDS_Edge EOn2 = TopoDS::Edge(MapSF(Anc.Last()) .Generated(E));
                        // find if exits tangent edges in the original shape
                        TopoDS_Edge E1f = new(), E1l = new();
                        TopExp.Vertices(E, out TopoDS_Vertex V1f, out TopoDS_Vertex V1l);
                        NCollection_List<TopoDS_Shape> TangE = new();
                        myAnalyse.TangentEdges(E, V1f, TangE);
                        // find if the pipe on the tangent edges are soon created.
                        NCollection_List<TopoDS_Shape>.Iterator itl = new(TangE);
                        bool Find = false;
                        for (; itl.More() && !Find; itl.Next())
                        {
                            if (MapSF.IsBound(itl.Value()))
                            {
                                TopoDS_Shape aLocalShape = MapSF[itl.Value()].InternalGenerated(V1f);
                                E1f = TopoDS.Edge(aLocalShape);
                                //              E1f  = TopoDS::Edge(MapSF(itl.Value()).Generated(V1f));
                                Find = true;
                            }
                        }
                        TangE.Clear();
                        myAnalyse.TangentEdges(E, V1l, TangE);
                        // find if the pipe on the tangent edges are soon created.
                        itl.Initialize(TangE);
                        Find = false;
                        for (; itl.More() && !Find; itl.Next())
                        {
                            if (MapSF.IsBound(itl.Value()))
                            {
                                TopoDS_Shape aLocalShape = MapSF[itl.Value()].InternalGenerated(V1l);
                                E1l = TopoDS.Edge(aLocalShape);
                                //              E1l  = TopoDS::Edge(MapSF(itl.Value()).Generated(V1l));
                                Find = true;
                            }
                        }
                        BRepOffset_Offset OF = new(E, EOn1, EOn2, CurOffset, E1f, E1l);
                        MapSF.Bind(E, OF);
                    }
                }
                else
                {
                    // ----------------------
                    // free border.
                    // ----------------------
                    TopoDS_Shape aLocalShape = MapSF[Anc.First()].InternalGenerated(E);
                    TopoDS_Edge EOn1 = TopoDS.Edge(aLocalShape);
                    //        TopoDS_Edge EOn1 = TopoDS::Edge(MapSF(Anc.First()).Generated(E));
                    myInitOffsetEdge.SetRoot(E); // skv: supporting history.
                    myInitOffsetEdge.Bind(E, EOn1);
                }
            }
        }

        //--------------------------------------------------------
        // Construction of spheres on vertex.
        //--------------------------------------------------------
        Done.Clear();
        NCollection_List<TopoDS_Shape>.Iterator it = new();
        using Message_ProgressScope aPS2 = new(aPSOuter.Next(4), "Constructing spheres on vertices", 1, true);
        for (Exp.Init(myFaceComp, TopAbs_VERTEX); Exp.More(); Exp.Next(), aPS2.Next())
        {
            if (!aPS2.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Vertex V = TopoDS.Vertex(Exp.Current());
            if (Done.Add(V))
            {
                NCollection_List<TopoDS_Shape> LA = myAnalyse.Ancestors(V);
                NCollection_List<TopoDS_Shape> LE = new();
                myAnalyse.Edges(V, OT, LE);

                if (LE.Extent() >= 3 && LE.Extent() == LA.Extent())
                {
                    NCollection_List<TopoDS_Shape> LOE = new();
                    //--------------------------------------------------------
                    // Return connected edges on tubes.
                    //--------------------------------------------------------
                    for (it.Initialize(LE); it.More(); it.Next())
                    {
                        LOE.Append(MapSF[it.Value()].InternalGenerated(V).Reversed());
                    }
                    //----------------------
                    // construction sphere.
                    //-----------------------
                    NCollection_List<TopoDS_Shape> LLA = myAnalyse.Ancestors(LA.First());
                    TopoDS_Shape FF = LLA.First();
                    double CurOffset = myOffset;
                    if (myFaceOffset.IsBound(FF))
                    {
                        CurOffset = myFaceOffset[FF];
                    }

                    BRepOffset_Offset OF = new(V, LOE, CurOffset);
                    MapSF.Bind(V, OF);
                }
                //--------------------------------------------------------------
                // Particular processing if V is at least a free border.
                //-------------------------------------------------------------
                NCollection_List<TopoDS_Shape> LBF = new();
                myAnalyse.Edges(V, ChFiDS_FreeBound, LBF);
                if (!LBF.IsEmpty())
                {
                    bool First = true;
                    for (it.Initialize(LE); it.More(); it.Next())
                    {
                        if (First)
                        {
                            myInitOffsetEdge.SetRoot(V); // skv: supporting history.
                            myInitOffsetEdge.Bind(V, MapSF[it.Value()].InternalGenerated(V));
                            First = false;
                        }
                        else
                        {
                            myInitOffsetEdge.Add(V, MapSF[it.Value()].InternalGenerated(V));
                        }
                    }
                }
            }
        }

        //------------------------------------------------------------
        // Extension of parallel faces to the context.
        // Extended faces are ordered in DS and removed from MapSF.
        //------------------------------------------------------------
        if (!myFaces.IsEmpty())
        {
            ToContext(MapSF);
        }

        //------------------------------------------------------
        // MAJ SD.
        //------------------------------------------------------
        ChFiDS_TypeOfConcavity RT = ChFiDS_Concave;
        if (myOffset < 0.0)
        {
            RT = ChFiDS_Convex;
        }
        // OCCT.NET: OCCT iterates MapSF by its shapes' addresses; the port in insertion order (decision 9203).
        using Message_ProgressScope aPS3 = new(aPSOuter.Next(), null, MapSF.Extent());
        foreach (KeyValuePair<TopoDS_Shape, BRepOffset_Offset> It in MapSF)
        {
            if (!aPS3.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Shape SI = It.Key;
            BRepOffset_Offset SF = It.Value;
            if (SF.Status() == BRepOffset_Reversed || SF.Status() == BRepOffset_Degenerated)
            {
                //------------------------------------------------
                // Degenerated or returned faces are not stored.
                //------------------------------------------------
                aPS3.Next();
                continue;
            }

            TopoDS_Face OF = It.Value.InternalFace();
            myInitOffsetFace.Bind(SI, OF);
            myInitOffsetFace.SetRoot(SI); // Initial<-> Offset
            myImageOffset.SetRoot(OF);    // FaceOffset root of images

            if (SI.ShapeType() == TopAbs_FACE)
            {
                for (Exp.Init(SI.Oriented(TopAbs_FORWARD), TopAbs_EDGE); Exp.More(); Exp.Next())
                {
                    //--------------------------------------------------------------------
                    // To each face are associatedthe edges that restrict that
                    // The edges that do not generate tubes or are not tangent
                    // to two faces are removed.
                    //--------------------------------------------------------------------
                    TopoDS_Edge E = TopoDS.Edge(Exp.Current());
                    NCollection_List<BRepOffset_Interval> L = myAnalyse.Type(E);
                    if (!L.IsEmpty() && L.First().Type() != RT)
                    {
                        TopAbs_Orientation OO = E.Orientation();
                        TopoDS_Shape aLocalShape = It.Value.InternalGenerated(E);
                        TopoDS_Edge OE = TopoDS.Edge(aLocalShape);
                        //          TopoDS_Edge        OE  = TopoDS::Edge(It.Value().Generated(E));
                        myAsDes.Add(OF, OE.Oriented(OO));
                    }
                }
            }
            else
            {
                for (Exp.Init(OF.Oriented(TopAbs_FORWARD), TopAbs_EDGE); Exp.More(); Exp.Next())
                {
                    myAsDes.Add(OF, Exp.Current());
                }
            }
            aPS3.Next();
        }

        // OCCT_DEBUG: the chronometer is left out.
    }

    //=================================================================================================

    private void SelfInter(NCollection_Map<TopoDS_Shape> Modif)
    {
        // OCCT_DEBUG: the chronometer is left out.

        throw new Standard_NotImplemented();
    }

    //=================================================================================================

    private void ToContext(NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> MapSF)
    {
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Created = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MEF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> FacesToBuild = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        TopExp_Explorer exp = new();

        //  TopAbs_State       Side = TopAbs_IN;
        //  if (myOffset < 0.) Side = TopAbs_OUT;

        TopAbs_State Side = TopAbs_OUT;

        // OCCT has a commented-out loop here over the context faces' edges not in the initial shape.

        //--------------------------------------------------------
        // Determine the edges and faces reconstructed by
        // intersection.
        //---------------------------------------------------------
        int j;
        for (j = 1; j <= myFaces.Extent(); j++)
        {
            TopoDS_Face CF = TopoDS.Face(myFaces[j]);
            for (exp.Init(CF.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(exp.Current());
                if (myAnalyse.HasAncestor(E))
                {
                    NCollection_List<TopoDS_Shape> LEA = myAnalyse.Ancestors(E);
                    for (itl.Initialize(LEA); itl.More(); itl.Next())
                    {
                        BRepOffset_Offset OF = MapSF[itl.Value()];
                        FacesToBuild.Add(itl.Value());
                        MEF.Bind(OF.InternalGenerated(E), CF);
                    }
                    TopoDS_Vertex[] V = new TopoDS_Vertex[2];
                    TopExp.Vertices(E, out V[0], out V[1]);
                    for (int i = 0; i < 2; i++)
                    {
                        NCollection_List<TopoDS_Shape> LVA = myAnalyse.Ancestors(V[i]);
                        for (itl.Initialize(LVA); itl.More(); itl.Next())
                        {
                            TopoDS_Edge EV = TopoDS.Edge(itl.Value());
                            if (MapSF.IsBound(EV))
                            {
                                BRepOffset_Offset OF = MapSF[EV];
                                FacesToBuild.Add(EV);
                                MEF.Bind(OF.InternalGenerated(V[i]), CF);
                            }
                        }
                    }
                }
            }
        }
        //---------------------------
        // Reconstruction of faces.
        //---------------------------
        TopoDS_Face F, NF = new();
        ChFiDS_TypeOfConcavity RT = ChFiDS_Concave;
        if (myOffset < 0.0)
        {
            RT = ChFiDS_Convex;
        }
        TopoDS_Shape OE, NE;
        TopAbs_Orientation Or;

        for (j = 1; j <= FacesToBuild.Extent(); j++)
        {
            TopoDS_Shape S = FacesToBuild[j];
            // OCCT copies the offset (a value type); the port's is only read here, and unbound below.
            BRepOffset_Offset BOF = MapSF[S];
            F = TopoDS.Face(BOF.InternalFace());
            BRepOffset_Tool.ExtentFace(F, Created, MEF, Side, myTol, ref NF);
            MapSF.UnBind(S);
            //--------------
            // MAJ SD.
            //--------------
            myInitOffsetFace.Bind(S, NF);
            myInitOffsetFace.SetRoot(S); // Initial<-> Offset
            myImageOffset.SetRoot(NF);

            if (S.ShapeType() == TopAbs_FACE)
            {
                for (exp.Init(S.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
                {

                    TopoDS_Edge E = TopoDS.Edge(exp.Current());
                    NCollection_List<BRepOffset_Interval> L = myAnalyse.Type(E);
                    OE = BOF.InternalGenerated(E);
                    Or = E.Orientation();
                    OE.Orientation(Or);
                    if (!L.IsEmpty() && L.First().Type() != RT)
                    {
                        if (Created.IsBound(OE))
                        {
                            NE = Created[OE];
                            if (NE.Orientation() == TopAbs_REVERSED)
                            {
                                NE.Orientation(TopAbs.Reverse(Or));
                            }
                            else
                            {
                                NE.Orientation(Or);
                            }
                            myAsDes.Add(NF, NE);
                        }
                        else
                        {
                            myAsDes.Add(NF, OE);
                        }
                    }
                }
            }
            else
            {
                //------------------
                // Tube
                //---------------------
                for (exp.Init(NF.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
                {
                    myAsDes.Add(NF, exp.Current());
                }
            }
            MapSF.UnBind(S);
        }

        //------------------
        // MAJ free borders
        //------------------
        // OCCT.NET: OCCT iterates Created by its edges' addresses; the port in insertion order (decision 9203).
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> itc in Created)
        {
            OE = itc.Key;
            NE = itc.Value;
            if (myInitOffsetEdge.IsImage(OE))
            {
                TopoDS_Shape E = myInitOffsetEdge.ImageFrom(OE);
                Or = myInitOffsetEdge.Image(E).First().Orientation();
                if (NE.Orientation() == TopAbs_REVERSED)
                {
                    NE.Orientation(TopAbs.Reverse(Or));
                }
                else
                {
                    NE.Orientation(Or);
                }
                myInitOffsetEdge.Remove(OE);
                myInitOffsetEdge.Bind(E, NE);
            }
        }
    }

    //=================================================================================================

    private void UpdateFaceOffset()
    {
        NCollection_Map<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, double> CopiedMap = new(myFaceOffset);

        ChFiDS_TypeOfConcavity RT = ChFiDS_Convex;
        if (myOffset < 0.0)
        {
            RT = ChFiDS_Concave;
        }

        // OCCT.NET: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 9203).
        foreach (KeyValuePair<TopoDS_Shape, double> it in CopiedMap)
        {
            TopoDS_Face F = TopoDS.Face(it.Key);
            double CurOffset = CopiedMap[F];
            if (!M.Add(F))
            {
                continue;
            }
            TopoDS_Compound Co = new();
            BRep_Builder Build = new();
            Build.MakeCompound(ref Co);
            NCollection_Map<TopoDS_Shape> Dummy = new(TopTools_ShapeMapHasher.Instance);
            Build.Add(Co, F);
            if (myJoin == GeomAbs_Arc)
            {
                myAnalyse.AddFaces(F, Co, Dummy, ChFiDS_Tangential, RT);
            }
            else
            {
                myAnalyse.AddFaces(F, Co, Dummy, ChFiDS_Tangential);
            }

            TopExp_Explorer exp = new(Co, TopAbs_FACE);
            for (; exp.More(); exp.Next())
            {
                TopoDS_Face FF = TopoDS.Face(exp.Current());
                if (!M.Add(FF))
                {
                    continue;
                }
                if (myFaceOffset.IsBound(FF))
                {
                    myFaceOffset.UnBind(FF);
                }
                myFaceOffset.Bind(FF, CurOffset);
            }
        }
    }

    //=================================================================================================

    private void CorrectConicalFaces()
    {
        if (myOffsetShape.IsNull())
        {
            return;
        }
        //
        NCollection_Sequence<TopoDS_Shape> Cones = new();
        NCollection_Sequence<TopoDS_Shape> Circs = new();
        NCollection_Sequence<TopoDS_Shape> Seams = new();
        double TolApex = 1.0e-5;

        int i;

        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> FacesOfCone = new(TopTools_ShapeMapHasher.Instance);
        // NCollection_DataMap<TopoDS_Shape, TopoDS_Shape, TopTools_ShapeMapHasher> DegEdges;
        TopExp_Explorer Explo = new(myOffsetShape, TopAbs_FACE);
        if (myJoin == GeomAbs_Arc)
        {
            for (; Explo.More(); Explo.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(Explo.Current());
                Geom_Surface? aSurf = BRep_Tool.Surface(aFace);
                // if (aSurf->DynamicType() == STANDARD_TYPE(Geom_OffsetSurface))
                // aSurf = (occ::down_cast<Geom_OffsetSurface>(aSurf))->BasisSurface(); //???

                NCollection_IndexedMap<TopoDS_Shape> Emap = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapes(aFace, TopAbs_EDGE, Emap);
                for (i = 1; i <= Emap.Extent(); i++)
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(Emap[i]);
                    // double f, l;
                    // occ::handle<Geom_Curve> theCurve = BRep_Tool::Curve( anEdge, f, l );
                    // occ::handle<BRep_TEdge>& TE = *((occ::handle<BRep_TEdge>*) &anEdge.TShape());
                    if (BRep_Tool.Degenerated(anEdge))
                    {
                        // Check if anEdge is a really degenerated edge or not
                        BRepAdaptor_Curve BACurve = new(anEdge, aFace);
                        gp_Pnt Pfirst, Plast, Pmid;
                        Pfirst = BACurve.Value(BACurve.FirstParameter());
                        Plast = BACurve.Value(BACurve.LastParameter());
                        Pmid = BACurve.Value((BACurve.FirstParameter() + BACurve.LastParameter()) / 2.0);
                        if (Pfirst.Distance(Plast) <= TolApex && Pfirst.Distance(Pmid) <= TolApex)
                        {
                            continue;
                        }
                        // Cones.Append( aFace );
                        // Circs.Append( anEdge );
                        // TopoDS_Vertex Vdeg = TopExp::FirstVertex( anEdge );
                        TopoDS_Edge OrEdge = TopoDS.Edge(myInitOffsetEdge.Root(anEdge));
                        TopoDS_Vertex VF = TopExp.FirstVertex(OrEdge);
                        if (FacesOfCone.IsBound(VF))
                        {
                            // add a face to the existing list
                            NCollection_List<TopoDS_Shape> aFaces = FacesOfCone.ChangeFind(VF);
                            aFaces.Append(aFace);
                            // DegEdges.Bind(aFace, anEdge);
                        }
                        else
                        {
                            // the vertex is not in the map => create a new key and items
                            NCollection_List<TopoDS_Shape> aFaces = new();
                            aFaces.Append(aFace);
                            FacesOfCone.Bind(VF, aFaces);
                            // DegEdges.Bind(aFace, anEdge);
                        }
                    }
                } // for (i = 1; i <= Emap.Extent(); i++)
            } // for (; fexp.More(); fexp.Next())
        } // if (myJoin == GeomAbs_Arc)

        BRep_Builder BB = new();
        TopLoc_Location L = new();
        bool IsModified = false;
        // OCCT.NET: OCCT iterates the apexes by their addresses; the port in insertion order (decision 9203).
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Cone in FacesOfCone)
        {
            gp_Sphere theSphere = new();
            Geom_SphericalSurface? aSphSurf = null;
            TopoDS_Wire SphereWire = new();
            BB.MakeWire(ref SphereWire);
            TopoDS_Vertex anApex = TopoDS.Vertex(Cone.Key);
            NCollection_List<TopoDS_Shape> Faces = Cone.Value; // FacesOfCone(anApex);
            NCollection_List<TopoDS_Shape>.Iterator itFaces = new(Faces);
            bool isFirstFace = true;
            gp_Pnt FirstPoint = new();
            TopoDS_Vertex theFirstVertex = new(), CurFirstVertex = new();
            for (; itFaces.More(); itFaces.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(itFaces.Value()); // TopoDS::Face(Faces.First());
                TopoDS_Edge DegEdge = new();                         // = TopoDS::Edge(DegEdges(aFace));
                for (Explo.Init(aFace, TopAbs_EDGE); Explo.More(); Explo.Next())
                {
                    DegEdge = TopoDS.Edge(Explo.Current());
                    if (BRep_Tool.Degenerated(DegEdge))
                    {
                        TopoDS_Edge OrEdge = TopoDS.Edge(myInitOffsetEdge.Root(DegEdge));
                        TopoDS_Vertex VF = TopExp.FirstVertex(OrEdge);
                        if (VF.IsSame(anApex))
                        {
                            break;
                        }
                    }
                }
                TopoDS_Shape aLocalDegShape = DegEdge.Oriented(TopAbs_FORWARD);
                TopoDS_Edge CurEdge = TopoDS.Edge(aLocalDegShape);
                BB.Degenerated(CurEdge, false);
                BB.SameRange(CurEdge, false);
                BB.SameParameter(CurEdge, false);
                GetEdgePoints(CurEdge, aFace, out gp_Pnt fPnt, out gp_Pnt mPnt, out gp_Pnt lPnt);
                BRep_Tool.Range(CurEdge, out double f, out double l);
                if (isFirstFace)
                {
                    gp_Vec aVec1 = new(fPnt, mPnt);
                    gp_Vec aVec2 = new(fPnt, lPnt);
                    gp_Vec aNorm = aVec1.Crossed(aVec2);
                    gp_Pnt theApex = BRep_Tool.Pnt(anApex);
                    gp_Vec ApexToFpnt = new(theApex, fPnt);
                    gp_Vec Ydir = aNorm ^ ApexToFpnt;
                    gp_Vec Xdir = Ydir ^ aNorm;
                    // Xdir.Rotate(gp_Ax1(theApex, aNorm), -f);
                    gp_Ax2 anAx2 = new(theApex, new gp_Dir(aNorm), new gp_Dir(Xdir));
                    theSphere.SetRadius(myOffset);
                    theSphere.SetPosition(new gp_Ax3(anAx2) /*gp_Ax3(theApex, gp_Dir(aNorm))*/);
                    aSphSurf = new Geom_SphericalSurface(theSphere);
                    FirstPoint = fPnt;
                    theFirstVertex = new BRepLib_MakeVertex(fPnt).InternalVertex();
                    CurFirstVertex = theFirstVertex;
                }

                TopoDS_Vertex FirstVert, EndVert;
                TopExp.Vertices(CurEdge, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
                FirstVert = CurFirstVertex;
                if (lPnt.Distance(FirstPoint) <= Precision.Confusion())
                {
                    EndVert = theFirstVertex;
                }
                else
                {
                    EndVert = new BRepLib_MakeVertex(lPnt).InternalVertex();
                }
                CurEdge.Free(true);
                BB.Remove(CurEdge, v1);
                BB.Remove(CurEdge, v2);
                BB.Add(CurEdge, FirstVert.Oriented(TopAbs_FORWARD));
                BB.Add(CurEdge, EndVert.Oriented(TopAbs_REVERSED));
                // take the curve from sphere an put it to the edge
                ElSLib.Parameters(theSphere, fPnt, out double Uf, out double Vf);
                ElSLib.Parameters(theSphere, lPnt, out double Ul, out double Vl);
                if (Math.Abs(Ul) <= Precision.Confusion())
                {
                    Ul = 2.0 * M_PI;
                }
                Geom_Curve aCurv = aSphSurf!.VIso(Vf);
                // OCCT has a commented-out rotation of the circle for the faces after the first.
                Geom_TrimmedCurve aTrimCurv = new(aCurv, Uf, Ul);
                BB.UpdateEdge(CurEdge, aTrimCurv, Precision.Confusion());
                BB.Range(CurEdge, Uf, Ul, true);
                Geom2d_Line theLin2d = new(new gp_Pnt2d(0.0, Vf), gp.DX2d());
                Geom2d_TrimmedCurve theTrimLin2d = new(theLin2d, Uf, Ul);
                BB.UpdateEdge(CurEdge, theTrimLin2d, aSphSurf, L, Precision.Confusion());
                BB.Range(CurEdge, aSphSurf, L, Uf, Ul);
                BRepLib.SameParameter(CurEdge);
                BB.Add(SphereWire, CurEdge);
                // Modifying correspondent edges in aFace: substitute vertices common with CurEdge
                BRepAdaptor_Curve2d BAc2d = new(CurEdge, aFace);
                gp_Pnt2d fPnt2d, lPnt2d;
                fPnt2d = BAc2d.Value(BAc2d.FirstParameter());
                lPnt2d = BAc2d.Value(BAc2d.LastParameter());
                NCollection_IndexedMap<TopoDS_Shape> Emap = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapes(aFace, TopAbs_EDGE, Emap);
                TopoDS_Edge[] EE = [new(), new()];
                int j = 0, k;
                for (k = 1; k <= Emap.Extent(); k++)
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(Emap[k]);
                    if (!BRep_Tool.Degenerated(anEdge))
                    {
                        TopExp.Vertices(anEdge, out TopoDS_Vertex V1e, out TopoDS_Vertex V2e);
                        if (V1e.IsSame(v1) || V2e.IsSame(v1))
                        {
                            EE[j++] = anEdge;
                        }
                    }
                }
                for (k = 0; k < j; k++)
                {
                    TopoDS_Shape aLocalShape = EE[k].Oriented(TopAbs_FORWARD);
                    TopoDS_Edge Eforward = TopoDS.Edge(aLocalShape);
                    Eforward.Free(true);
                    TopExp.Vertices(Eforward, out TopoDS_Vertex V1e, out TopoDS_Vertex V2e);
                    BRepAdaptor_Curve2d EEc = new(Eforward, aFace);
                    gp_Pnt2d p2d1_, p2d2_;
                    p2d1_ = EEc.Value(EEc.FirstParameter());
                    p2d2_ = EEc.Value(EEc.LastParameter());
                    if (V1e.IsSame(v1))
                    {
                        TopoDS_Vertex NewV = (p2d1_.Distance(fPnt2d) <= Precision.Confusion()) ? FirstVert : EndVert;
                        BB.Remove(Eforward, V1e);
                        BB.Add(Eforward, NewV.Oriented(TopAbs_FORWARD));
                    }
                    else
                    {
                        TopoDS_Vertex NewV = (p2d2_.Distance(fPnt2d) <= Precision.Confusion()) ? FirstVert : EndVert;
                        BB.Remove(Eforward, V2e);
                        BB.Add(Eforward, NewV.Oriented(TopAbs_REVERSED));
                    }
                }

                isFirstFace = false;
                CurFirstVertex = EndVert;
            }
            // Building new spherical face
            double Ufirst = RealLast(), Ulast = RealFirst();
            gp_Pnt2d p2d1 = new(), p2d2 = new();
            NCollection_List<TopoDS_Shape> EdgesOfWire = new();
            TopoDS_Iterator itw = new(SphereWire);
            for (; itw.More(); itw.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(itw.Value());
                EdgesOfWire.Append(anEdge);
                Geom2d_Curve aC2d = BRep_Tool.CurveOnSurface(anEdge, aSphSurf, L, out double f, out double l)!;
                p2d1 = aC2d.Value(f);
                p2d2 = aC2d.Value(l);
                if (p2d1.X() < Ufirst)
                {
                    Ufirst = p2d1.X();
                }
                if (p2d1.X() > Ulast)
                {
                    Ulast = p2d1.X();
                }
                if (p2d2.X() < Ufirst)
                {
                    Ufirst = p2d2.X();
                }
                if (p2d2.X() > Ulast)
                {
                    Ulast = p2d2.X();
                }
            }
            NCollection_List<TopoDS_Shape> NewEdges = new();
            TopoDS_Edge FirstEdge = new();
            NCollection_List<TopoDS_Shape>.Iterator itl = new(EdgesOfWire);
            for (; itl.More(); itl.Next())
            {
                FirstEdge = TopoDS.Edge(itl.Value());
                Geom2d_Curve aC2d = BRep_Tool.CurveOnSurface(FirstEdge, aSphSurf, L, out double f, out double l)!;
                p2d1 = aC2d.Value(f);
                p2d2 = aC2d.Value(l);
                if (Math.Abs(p2d1.X() - Ufirst) <= Precision.Confusion())
                {
                    EdgesOfWire.Remove(itl);
                    break;
                }
            }
            NewEdges.Append(FirstEdge.Oriented(TopAbs_FORWARD));
            TopExp.Vertices(FirstEdge, out TopoDS_Vertex Vf1, out TopoDS_Vertex CurVertex);
            itl.Initialize(EdgesOfWire);
            while (itl.More())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(itl.Value());
                TopExp.Vertices(anEdge, out TopoDS_Vertex V1_, out TopoDS_Vertex V2_);
                if (V1_.IsSame(CurVertex) || V2_.IsSame(CurVertex))
                {
                    NewEdges.Append(anEdge.Oriented(TopAbs_FORWARD));
                    CurVertex = (V1_.IsSame(CurVertex)) ? V2_ : V1_;
                    EdgesOfWire.Remove(itl);
                }
                else
                {
                    itl.Next();
                }
            }

            double Vfirst, Vlast;
            if (p2d1.Y() > 0.0)
            {
                Vfirst = p2d1.Y();
                Vlast = M_PI / 2.0;
            }
            else
            {
                Vfirst = -M_PI / 2.0;
                Vlast = p2d1.Y();
            }
            TopoDS_Face NewSphericalFace =
                new BRepLib_MakeFace(aSphSurf!, Ufirst, Ulast, Vfirst, Vlast, Precision.Confusion()).InternalFace();
            TopoDS_Edge OldEdge = new(), DegEdge2 = new();
            for (Explo.Init(NewSphericalFace, TopAbs_EDGE); Explo.More(); Explo.Next())
            {
                DegEdge2 = TopoDS.Edge(Explo.Current());
                if (BRep_Tool.Degenerated(DegEdge2))
                {
                    break;
                }
            }
            TopoDS_Vertex DegVertex = TopExp.FirstVertex(DegEdge2);
            for (Explo.Init(NewSphericalFace, TopAbs_EDGE); Explo.More(); Explo.Next())
            {
                OldEdge = TopoDS.Edge(Explo.Current());
                TopExp.Vertices(OldEdge, out TopoDS_Vertex V1_, out TopoDS_Vertex V2_);
                if (!V1_.IsSame(DegVertex) && !V2_.IsSame(DegVertex))
                {
                    break;
                }
            }
            TopExp.Vertices(OldEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            NCollection_List<TopoDS_Shape> LV1 = new(), LV2 = new();
            LV1.Append(Vf1.Oriented(TopAbs_FORWARD));
            LV2.Append(CurVertex.Oriented(TopAbs_FORWARD));
            BRepTools_Substitution theSubstitutor = new();
            theSubstitutor.Substitute(V1.Oriented(TopAbs_FORWARD), LV1);
            if (!V1.IsSame(V2))
            {
                theSubstitutor.Substitute(V2.Oriented(TopAbs_FORWARD), LV2);
            }
            theSubstitutor.Substitute(OldEdge.Oriented(TopAbs_FORWARD), NewEdges);
            theSubstitutor.Build(NewSphericalFace);
            if (theSubstitutor.IsCopied(NewSphericalFace))
            {
                NCollection_List<TopoDS_Shape> listSh = theSubstitutor.InternalCopy(NewSphericalFace);
                NewSphericalFace = TopoDS.Face(listSh.First());
            }

            // Adding NewSphericalFace to the shell
            Explo.Init(myOffsetShape, TopAbs_SHELL);
            TopoDS_Shape theShell = Explo.Current();
            theShell.Free(true);
            BB.Add(theShell, NewSphericalFace);
            IsModified = true;
            if (!theShell.Closed())
            {
                if (BRep_Tool.IsClosed(theShell))
                {
                    theShell.Closed(true);
                }
            }
        }
        //
        if (!IsModified)
        {
            return;
        }
        //
        if (myShape.ShapeType() == TopAbs_SOLID || myThickening)
        {
            // Explo.Init( myOffsetShape, TopAbs_SHELL );

            // if (Explo.More()) {
            //   TopoDS_Shape theShell = Explo.Current();
            //   theShell.Closed( true );
            // }

            int NbShell = 0;
            TopoDS_Compound NC = new();
            TopoDS_Shape S1 = new();
            BB.MakeCompound(ref NC);

            TopoDS_Solid Sol = new();
            BB.MakeSolid(ref Sol);
            Sol.Closed(true);
            for (Explo.Init(myOffsetShape, TopAbs_SHELL); Explo.More(); Explo.Next())
            {
                TopoDS_Shell Sh = TopoDS.Shell(Explo.Current());
                // if (myThickening && myOffset > 0.)
                //   Sh.Reverse();
                NbShell++;
                if (Sh.Closed())
                {
                    BB.Add(Sol, Sh);
                }
                else
                {
                    BB.Add(NC, Sh);
                    if (NbShell == 1)
                    {
                        S1 = Sh;
                    }
                }
            }
            int nbs = Sol.NbChildren();
            bool SolIsNull = (nbs == 0);
            // Checking solid
            if (nbs > 1)
            {
                BRepCheck_Analyzer aCheck = new(Sol, false);
                if (!aCheck.IsValid())
                {
                    NCollection_List<TopoDS_Shape> aSolList = new();
                    CorrectSolid(ref Sol, aSolList);
                    if (!aSolList.IsEmpty())
                    {
                        BB.Add(NC, Sol);
                        NCollection_List<TopoDS_Shape>.Iterator aSLIt = new(aSolList);
                        for (; aSLIt.More(); aSLIt.Next())
                        {
                            BB.Add(NC, aSLIt.Value());
                        }
                        SolIsNull = true;
                    }
                }
            }
            //
            bool NCIsNull = (NC.NbChildren() == 0);
            if ((!SolIsNull) && (!NCIsNull))
            {
                BB.Add(NC, Sol);
                myOffsetShape = NC;
            }
            else if (SolIsNull && (!NCIsNull))
            {
                if (NbShell == 1)
                {
                    myOffsetShape = S1;
                }
                else
                {
                    myOffsetShape = NC;
                }
            }
            else if ((!SolIsNull) && NCIsNull)
            {
                myOffsetShape = Sol;
            }
            else
            {
                myOffsetShape = NC;
            }
        }
    }

    //=================================================================================================

    private void Intersection3D(BRepOffset_Inter3d Inter, Message_ProgressRange theRange)
    {
        // OCCT_DEBUG: the chronometer is left out.
        using Message_ProgressScope aPS = new(theRange, null, (myFaces.Extent() != 0 && myJoin == GeomAbs_Arc) ? 2 : 1);

        // In the Complete Intersection mode, implemented currently for planar
        // solids only, there is no need to intersect the faces here.
        // This intersection will be performed in the method BuildShellsCompleteInter
        // where the special treatment is applied to produced faces.
        //
        // Make sure to match the parameters in which the method
        // BuildShellsCompleteInter is called.
        if (myInter && (myJoin == GeomAbs_Intersection) && myIsPlanar && !myThickening && myFaces.IsEmpty()
            && IsSolid(myShape))
        {
            return;
        }

        NCollection_List<TopoDS_Shape> OffsetFaces = new(); // list of faces // created.
        MakeList(OffsetFaces, myInitOffsetFace, myFaces);

        if (!myFaces.IsEmpty())
        {
            bool InSide = (myOffset < 0.0); // Temporary
            // it is necessary to calculate Inside taking account of the concavity or convexity of edges
            // between the cap and the part.

            if (myJoin == GeomAbs_Arc)
            {
                Inter.ContextIntByArc(myFaces, InSide, myAnalyse, myInitOffsetFace, myInitOffsetEdge, aPS.Next());
            }
        }
        if (myInter)
        {
            //-------------
            // Complete.
            //-------------
            Inter.CompletInt(OffsetFaces, myInitOffsetFace, aPS.Next());
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            NCollection_IndexedMap<TopoDS_Shape> NewEdges = Inter.NewEdges();
            if (myJoin == GeomAbs_Intersection)
            {
                BRepOffset_Tool.CorrectOrientation(myFaceComp, NewEdges, myAsDes, myInitOffsetFace, myOffset);
            }
        }
        else
        {
            //--------------------------------
            // Only between neighbor faces.
            //--------------------------------
            Inter.ConnexIntByArc(OffsetFaces, myFaceComp, myAnalyse, myInitOffsetFace, aPS.Next());
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
        }
        // OCCT_DEBUG: the chronometer is left out.
    }

    //=================================================================================================

    private void Intersection2D(NCollection_IndexedMap<TopoDS_Shape> Modif, NCollection_IndexedMap<TopoDS_Shape> NewEdges,
                                Message_ProgressRange theRange)
    {
        // OCCT_DEBUG: the chronometer is left out.
        //--------------------------------------------------------
        // calculate intersections2d on faces concerned by
        // intersection3d
        //---------------------------------------------------------
        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>::Iterator it(Modif);
        //-----------------------------------------------
        // Intersection of edges 2 by 2.
        //-----------------------------------------------
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVV = new(TopTools_ShapeMapHasher.Instance);
        int i;
        using Message_ProgressScope aPS = new(theRange, "Intersection 2D", Modif.Extent());
        for (i = 1; i <= Modif.Extent(); i++)
        {
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Face F = TopoDS.Face(Modif[i]);
            BRepOffset_Inter2d.Compute(myAsDes, F, NewEdges, myTol, myEdgeIntEdges, aDMVV, aPS.Next());
        }
        //
        BRepOffset_Inter2d.FuseVertices(aDMVV, myAsDes, myImageVV);
        //
        // OCCT_DEBUG: DEBVerticesControl and the chronometer are left out.
    }

    //=================================================================================================

    private void MakeLoops(NCollection_IndexedMap<TopoDS_Shape> Modif, Message_ProgressRange theRange)
    {
        // OCCT_DEBUG: the chronometer is left out.
        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>::Iterator    it(Modif);
        NCollection_List<TopoDS_Shape> LF = new(), LC = new();
        //-----------------------------------------
        // unwinding of faces // modified.
        //-----------------------------------------
        int i;
        for (i = 1; i <= Modif.Extent(); i++)
        {
            if (!myFaces.Contains(Modif[i]))
            {
                LF.Append(Modif[i]);
            }
        }
        //
        using Message_ProgressScope aPS = new(theRange, null, LF.Extent() + myFaces.Extent());
        if ((myJoin == GeomAbs_Intersection) && myInter && myIsPlanar)
        {
            BuildSplitsOfTrimmedFaces(LF, myAsDes, myImageOffset, aPS.Next(LF.Extent()));
        }
        else
        {
            myMakeLoops.Build(LF, myAsDes, myImageOffset, myImageVV, aPS.Next(LF.Extent()));
        }
        if (!aPS.More())
        {
            myError = BRepOffset_UserBreak;
            return;
        }

        //-----------------------------------------
        // unwinding of caps.
        //-----------------------------------------
        for (i = 1; i <= myFaces.Extent(); i++)
        {
            LC.Append(myFaces[i]);
        }

        bool InSide = true;
        if (myOffset > 0)
        {
            InSide = false;
        }
        myMakeLoops.BuildOnContext(LC, myAnalyse, myAsDes, myImageOffset, InSide, aPS.Next(LC.Extent()));

        // OCCT_DEBUG: the chronometer is left out.
    }

    //=======================================================================
    // function : MakeFaces
    // purpose  : Reconstruction of topologically unchanged faces that
    //           share edges that were reconstructed.
    //=======================================================================
    private void MakeFaces(NCollection_IndexedMap<TopoDS_Shape> Modif, Message_ProgressRange theRange)
    {
        // OCCT_DEBUG: the chronometer is left out.
        NCollection_List<TopoDS_Shape>.Iterator itr = new();
        NCollection_List<TopoDS_Shape> Roots = myInitOffsetFace.Roots();
        NCollection_List<TopoDS_Shape> LOF = new();
        //----------------------------------
        // Loop on all faces //.
        //----------------------------------
        for (itr.Initialize(Roots); itr.More(); itr.Next())
        {
            TopoDS_Face F = TopoDS.Face(myInitOffsetFace.Image(itr.Value()).First());
            if (!myImageOffset.HasImage(F))
            {
                LOF.Append(F);
            }
        }
        //
        using Message_ProgressScope aPS = new(theRange, null, 1);
        if ((myJoin == GeomAbs_Intersection) && myInter && myIsPlanar)
        {
            BuildSplitsOfTrimmedFaces(LOF, myAsDes, myImageOffset, aPS.Next());
        }
        else
        {
            myMakeLoops.BuildFaces(LOF, myAsDes, myImageOffset, aPS.Next());
        }
        if (!aPS.More())
        {
            myError = BRepOffset_UserBreak;
            return;
        }
        // OCCT_DEBUG: the chronometer is left out.
    }

    //=======================================================================
    // function : UpdateInitOffset
    // purpose  : Update and cleaning of myInitOffset
    //=======================================================================
    private static void UpdateInitOffset(BRepAlgo_Image myInitOffset, BRepAlgo_Image myImageOffset,
                                         in TopoDS_Shape myOffsetShape, TopAbs_ShapeEnum theShapeType) // skv
    {
        BRepAlgo_Image NIOF = new();
        NCollection_List<TopoDS_Shape> Roots = myInitOffset.Roots();
        NCollection_List<TopoDS_Shape>.Iterator it = new(Roots);
        for (; it.More(); it.Next())
        {
            NIOF.SetRoot(it.Value());
        }
        for (it.Initialize(Roots); it.More(); it.Next())
        {
            TopoDS_Shape SI = it.Value();
            NCollection_List<TopoDS_Shape> LI = new();
            NCollection_List<TopoDS_Shape> L1 = new();
            myInitOffset.LastImage(SI, L1);
            NCollection_List<TopoDS_Shape>.Iterator itL1 = new(L1);
            for (; itL1.More(); itL1.Next())
            {
                TopoDS_Shape O1 = itL1.Value();
                NCollection_List<TopoDS_Shape> L2 = new();
                myImageOffset.LastImage(O1, L2);
                LI.Append(L2);
            }
            NIOF.Bind(SI, LI);
        }
        //  Modified by skv - Mon Apr  4 18:17:27 2005 Begin
        //  Supporting history.
        //   NIOF.Filter(myOffsetShape,TopAbs_FACE);
        NIOF.Filter(myOffsetShape, theShapeType);
        //  Modified by skv - Mon Apr  4 18:17:27 2005 End
        myInitOffset.Assign(NIOF);
    }

    //=================================================================================================

    private void MakeMissingWalls(Message_ProgressRange theRange)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Contours =
            new(TopTools_ShapeMapHasher.Instance); // Start vertex + list of connected edges (free boundary)
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapEF = new(TopTools_ShapeMapHasher.Instance); // Edges of contours: edge + face
        double OffsetVal = Math.Abs(myOffset);

        FillContours(myFaceComp, myAnalyse, Contours, MapEF);

        using Message_ProgressScope aPS = new(theRange, "Making missing walls", Contours.Extent());
        for (int ic = 1; ic <= Contours.Extent(); ic++, aPS.Next())
        {
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Vertex StartVertex = TopoDS.Vertex(Contours.FindKey(ic));
            TopoDS_Edge StartEdge = new();
            NCollection_List<TopoDS_Shape> aContour = Contours[ic];
            NCollection_List<TopoDS_Shape>.Iterator itl = new(aContour);
            bool FirstStep = true;
            TopoDS_Edge PrevEdge = new();
            TopoDS_Vertex PrevVertex = StartVertex;
            bool isBuildFromScratch = false; // Problems with edges.
            for (; itl.More(); itl.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(itl.Value());
                TopoDS_Face aFaceOfEdge = TopoDS.Face(MapEF[anEdge]);

                // Check for offset existence.
                if (!myInitOffsetEdge.HasImage(anEdge))
                {
                    continue;
                }

                // Check for existence of two different vertices.
                NCollection_List<TopoDS_Shape> LOE = new(), LOE2 = new();
                myInitOffsetEdge.LastImage(anEdge, LOE);
                myImageOffset.LastImage(LOE.Last(), LOE2);
                TopoDS_Edge OE = TopoDS.Edge(LOE2.Last());
                TopExp.Vertices(OE, out TopoDS_Vertex V4, out TopoDS_Vertex V3);
                TopExp.Vertices(anEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                Geom_Curve? aC = BRep_Tool.Curve(anEdge, out double aF, out double aL);
                if (V3.IsNull() && V4.IsNull())
                {
                    // Initially offset edge is created without vertices.
                    // Then edge is trimmed by intersection line between
                    // two adjacent extended offset faces and get vertices.
                    // When intersection lines are invalid for any reason,
                    // (one reason is mixed connectivity of faces)
                    // algorithm of cutting offset edge by intersection line
                    // can fail and offset edge cannot get vertices.
                    // Following workaround is only to avoid exception if V3 and V4 are Null
                    // Vertex points are invalid.
                    TopAbs_Orientation anOEOri = OE.Orientation();
                    OE.Orientation(TopAbs_FORWARD);
                    Geom_Curve anOEC = BRep_Tool.Curve(OE, out double anOEF, out double anOEL)!;
                    BRep_Builder aBB = new();
                    gp_Pnt aP1 = anOEC.Value(aF);
                    gp_Pnt aP2 = anOEC.Value(aL);
                    TopoDS_Vertex anOEV1 = new(), anOEV2 = new();
                    double aTol = Math.Max(BRep_Tool.Tolerance(V1), BRep_Tool.Tolerance(V2));
                    aBB.MakeVertex(ref anOEV1, aP1, aTol);
                    anOEV1.Orientation(TopAbs_FORWARD);
                    aBB.MakeVertex(ref anOEV2, aP2, aTol);
                    anOEV2.Orientation(TopAbs_REVERSED);
                    aBB.Add(OE, anOEV1);
                    aBB.Add(OE, anOEV2);
                    aBB.Range(OE, aF, aL);
                    OE.Orientation(anOEOri);
                    TopExp.Vertices(OE, out V4, out V3);
                }
                if (aC is not null && (!aC.IsClosed() && !aC.IsPeriodic()))
                {
                    gp_Pnt aPntF = BRep_Tool.Pnt(V1);
                    gp_Pnt aPntL = BRep_Tool.Pnt(V2);
                    double aDistE = aPntF.SquareDistance(aPntL);
                    if (aDistE < Precision.SquareConfusion())
                    {
                        // Bad case: non closed, but vertexes mapped to same 3d point.
                        continue;
                    }

                    double anEdgeTol0 = BRep_Tool.Tolerance(anEdge);
                    if (aDistE < anEdgeTol0)
                    {
                        // Potential problems not detected via checkshape.
                        gp_Pnt aPntOF = BRep_Tool.Pnt(V4);
                        gp_Pnt aPntOL = BRep_Tool.Pnt(V3);
                        if (aPntOF.SquareDistance(aPntOL) > gp.Resolution())
                        {
                            // To avoid computation of complex analytical continuation of Sin / ArcSin.
                            double aSinValue = Math.Min(2 * anEdgeTol0 / aPntOF.Distance(aPntOL), 1.0);
                            double aMaxAngle = Math.Min(Math.Abs(Math.Asin(aSinValue)), M_PI_4); // Maximal angle.
                            double aCurrentAngle = new gp_Vec(aPntF, aPntL).Angle(new gp_Vec(aPntOF, aPntOL));
                            if (aC is Geom_Line && Math.Abs(aCurrentAngle) > aMaxAngle)
                            {
                                // anEdge not collinear to offset edge.
                                isBuildFromScratch = true;
                                myIsPerformSewing = true;
                                continue;
                            }
                        }
                    }
                }

                bool ToReverse = false;
                if (!V1.IsSame(PrevVertex))
                {
                    TopoDS_Vertex aVtx = V1;
                    V1 = V2;
                    V2 = aVtx;
                    aVtx = V3;
                    V3 = V4;
                    V4 = aVtx;
                    ToReverse = true;
                }

                OE.Orientation(TopAbs.Reverse(anEdge.Orientation()));
                TopoDS_Edge E3, E4 = new();
                bool ArcOnV2 = ((myJoin == GeomAbs_Arc) && (myInitOffsetEdge.HasImage(V2)));
                if (FirstStep || isBuildFromScratch)
                {
                    E4 = new BRepLib_MakeEdge(V1, V4).InternalEdge();
                    if (FirstStep)
                    {
                        StartEdge = E4;
                    }
                }
                else
                {
                    E4 = PrevEdge;
                }
                if (V2.IsSame(StartVertex) && !ArcOnV2)
                {
                    E3 = StartEdge;
                }
                else
                {
                    E3 = new BRepLib_MakeEdge(V2, V3).InternalEdge();
                }
                E4.Reverse();

                if (isBuildFromScratch)
                {
                    E3.Reverse();
                    E4.Reverse();
                }

                TopoDS_Shape localAnEdge = anEdge.Oriented(TopAbs_FORWARD);
                TopoDS_Edge anEdgeFWD = TopoDS.Edge(localAnEdge);
                double ParV1 = BRep_Tool.Parameter(V1, anEdgeFWD);
                double ParV2 = BRep_Tool.Parameter(V2, anEdgeFWD);
                BRep_Builder BB = new();
                TopoDS_Wire theWire = new();
                BB.MakeWire(ref theWire);
                if (ToReverse)
                {
                    BB.Add(theWire, anEdge.Reversed());
                    BB.Add(theWire, E3.Reversed());
                    BB.Add(theWire, OE.Reversed());
                    BB.Add(theWire, E4.Reversed());
                }
                else
                {
                    BB.Add(theWire, anEdge);
                    BB.Add(theWire, E3);
                    BB.Add(theWire, OE);
                    BB.Add(theWire, E4);
                }

                BRepLib.BuildCurves3d(theWire, myTol);
                theWire.Closed(true);
                TopoDS_Face NewFace = new();
                Geom_Surface? theSurf = null;
                BRepAdaptor_Curve BAcurve = new(anEdge);
                BRepAdaptor_Curve BAcurveOE = new(OE);
                double fpar = BAcurve.FirstParameter();
                double lpar = BAcurve.LastParameter();
                gp_Pnt PonE = BAcurve.Value(fpar);
                gp_Pnt PonOE = BAcurveOE.Value(fpar);
                gp_Dir OffsetDir = new gce_MakeDir(PonE, PonOE).Value();
                Geom2d_Line? EdgeLine2d = null, OELine2d = null, aLine2d = null, aLine2d2 = null;
                bool IsPlanar = false;
                if (BAcurve.GetType() == GeomAbs_Circle && BAcurveOE.GetType() == GeomAbs_Circle)
                {
                    gp_Circ aCirc = BAcurve.Circle();
                    gp_Circ aCircOE = BAcurveOE.Circle();
                    gp_Lin anAxisLine = new(aCirc.Axis());
                    gp_Dir CircAxisDir = aCirc.Axis().Direction();
                    if (aCirc.Axis().IsParallel(aCircOE.Axis(), Precision.Confusion())
                        && anAxisLine.Contains(aCircOE.Location(), Precision.Confusion()))
                    { // cylinder, plane or cone
                        if (Math.Abs(aCirc.Radius() - aCircOE.Radius()) <= Precision.Confusion())
                        { // case of cylinder
                            theSurf = new GC_MakeCylindricalSurface(aCirc).Value();
                        }
                        else if (aCirc.Location().Distance(aCircOE.Location()) <= Precision.Confusion())
                        { // case of plane
                            IsPlanar = true;
                            //
                            gp_Pnt PonEL = BAcurve.Value(lpar);
                            if (PonEL.Distance(PonE) <= Precision.PConfusion())
                            {
                                bool bIsHole;
                                TopoDS_Edge aE1, aE2;
                                TopoDS_Wire aW1 = new(), aW2 = new();
                                Geom_Plane aPL;
                                IntTools_FClass2d aClsf = new();
                                //
                                if (aCirc.Radius() > aCircOE.Radius())
                                {
                                    aE1 = anEdge;
                                    aE2 = OE;
                                }
                                else
                                {
                                    aE1 = OE;
                                    aE2 = anEdge;
                                }
                                //
                                BB.MakeWire(ref aW1);
                                BB.Add(aW1, aE1);
                                BB.MakeWire(ref aW2);
                                BB.Add(aW2, aE2);
                                //
                                aPL = new Geom_Plane(aCirc.Location(), CircAxisDir);
                                for (int i = 0; i < 2; ++i)
                                {
                                    ref TopoDS_Wire aW = ref (i == 0) ? ref aW1 : ref aW2;
                                    TopoDS_Edge aE = (i == 0) ? aE1 : aE2;
                                    //
                                    TopoDS_Face aFace = new();
                                    BB.MakeFace(ref aFace, aPL, Precision.Confusion());
                                    BB.Add(aFace, aW);
                                    aClsf.Init(aFace, Precision.Confusion());
                                    bIsHole = aClsf.IsHole();
                                    if ((bIsHole && i == 0) || (!bIsHole && i != 0))
                                    {
                                        aW.Nullify();
                                        BB.MakeWire(ref aW);
                                        BB.Add(aW, aE.Reversed());
                                    }
                                }
                                //
                                BB.MakeFace(ref NewFace, aPL, Precision.Confusion());
                                BB.Add(NewFace, aW1);
                                BB.Add(NewFace, aW2);
                            }
                        }
                        else // case of cone
                        {
                            gp_Cone theCone =
                                new gce_MakeCone(aCirc.Location(), aCircOE.Location(), aCirc.Radius(), aCircOE.Radius()).Value();
                            gp_Ax3 theAx3 = new(aCirc.Position());
                            if (CircAxisDir * theCone.Axis().Direction() < 0.0)
                            {
                                theAx3.ZReverse();
                                CircAxisDir.Reverse();
                            }
                            theCone.SetPosition(theAx3);
                            theSurf = new Geom_ConicalSurface(theCone);
                        }
                        if (!IsPlanar)
                        {
                            TopLoc_Location Loc = new();
                            EdgeLine2d = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(gp_Dir2d.D.X));
                            BB.UpdateEdge(anEdge, EdgeLine2d, theSurf, Loc, Precision.Confusion());
                            double Coeff = (OffsetDir * CircAxisDir > 0.0) ? 1.0 : -1.0;
                            OELine2d = new Geom2d_Line(new gp_Pnt2d(0.0, OffsetVal * Coeff), new gp_Dir2d(gp_Dir2d.D.X));
                            BB.UpdateEdge(OE, OELine2d, theSurf, Loc, Precision.Confusion());
                            aLine2d = new Geom2d_Line(new gp_Pnt2d(ParV2, 0.0), new gp_Dir2d(0.0, Coeff));
                            aLine2d2 = new Geom2d_Line(new gp_Pnt2d(ParV1, 0.0), new gp_Dir2d(0.0, Coeff));
                            if (E3.IsSame(E4))
                            {
                                if (Coeff > 0.0)
                                {
                                    BB.UpdateEdge(E3, aLine2d, aLine2d2, theSurf, Loc, Precision.Confusion());
                                }
                                else
                                {
                                    BB.UpdateEdge(E3, aLine2d2, aLine2d, theSurf, Loc, Precision.Confusion());
                                    theWire.Nullify();
                                    BB.MakeWire(ref theWire);
                                    BB.Add(theWire, anEdge.Oriented(TopAbs_REVERSED));
                                    BB.Add(theWire, E4);
                                    BB.Add(theWire, OE.Oriented(TopAbs_FORWARD));
                                    BB.Add(theWire, E3);
                                    theWire.Closed(true);
                                }
                            }
                            else
                            {
                                BB.SameParameter(E3, false);
                                BB.SameRange(E3, false);
                                BB.SameParameter(E4, false);
                                BB.SameRange(E4, false);
                                BB.UpdateEdge(E3, aLine2d, theSurf, Loc, Precision.Confusion());
                                BB.Range(E3, theSurf, Loc, 0.0, OffsetVal);
                                BB.UpdateEdge(E4, aLine2d2, theSurf, Loc, Precision.Confusion());
                                BB.Range(E4, theSurf, Loc, 0.0, OffsetVal);
                            }
                            NewFace = new BRepLib_MakeFace(theSurf!, theWire).InternalFace();
                        }
                    } // cylinder or cone
                } // if both edges are arcs of circles
                if (NewFace.IsNull())
                {
                    double anEdgeTol = BRep_Tool.Tolerance(anEdge);
                    // Tolerances of input shape should not be increased by BRepLib_MakeFace
                    BRepLib_FindSurface aFindPlane = new(theWire, anEdgeTol, true); // only plane
                    IsPlanar = false;
                    if (aFindPlane.Found() && aFindPlane.ToleranceReached() <= anEdgeTol)
                    {
                        Geom_Curve? aGC = BRep_Tool.Curve(anEdge, out double f, out double l);
                        Geom_Plane aPln = (Geom_Plane)aFindPlane.Surface()!;
                        double aMaxDist = ComputeMaxDist(aPln.Pln(), aGC, f, l);
                        if (aMaxDist <= anEdgeTol)
                        {
                            BRepLib_MakeFace MF = new(aPln.Pln(), theWire);
                            if (MF.IsDone())
                            {
                                NewFace = MF.InternalFace();
                                TopoDS_Iterator anItE = new(theWire);
                                for (; anItE.More(); anItE.Next())
                                {
                                    TopoDS_Edge anE = TopoDS.Edge(anItE.Value());
                                    if (anE.IsSame(anEdge))
                                    {
                                        continue;
                                    }
                                    aGC = BRep_Tool.Curve(anE, out f, out l);
                                    aMaxDist = ComputeMaxDist(aPln.Pln(), aGC, f, l);
                                    BB.UpdateEdge(anE, aMaxDist);
                                }
                                IsPlanar = true;
                            }
                        }
                    }
                    //
                    if (!IsPlanar) // Extrusion (by thrusections)
                    {
                        Geom_Curve EdgeCurve = BRep_Tool.Curve(anEdge, out fpar, out lpar)!;
                        Geom_TrimmedCurve TrEdgeCurve = new(EdgeCurve, fpar, lpar);
                        Geom_Curve OffsetCurve = BRep_Tool.Curve(OE, out double fparOE0, out double lparOE0)!;
                        Geom_TrimmedCurve TrOffsetCurve = new(OffsetCurve, fparOE0, lparOE0);
                        GeomFill_Generator ThrusecGenerator = new();
                        ThrusecGenerator.AddCurve(TrEdgeCurve);
                        ThrusecGenerator.AddCurve(TrOffsetCurve);
                        ThrusecGenerator.Perform(Precision.PConfusion());
                        theSurf = ThrusecGenerator.Surface()!;
                        theSurf.Bounds(out double Uf, out double Ul, out double Vf, out double Vl);
                        TopLoc_Location Loc = new();
                        EdgeLine2d = new Geom2d_Line(new gp_Pnt2d(0.0, Vf), new gp_Dir2d(gp_Dir2d.D.X));
                        BB.UpdateEdge(anEdge, EdgeLine2d, theSurf, Loc, Precision.Confusion());
                        OELine2d = new Geom2d_Line(new gp_Pnt2d(0.0, Vl), new gp_Dir2d(gp_Dir2d.D.X));
                        BB.UpdateEdge(OE, OELine2d, theSurf, Loc, Precision.Confusion());
                        double UonV1 = (ToReverse) ? Ul : Uf;
                        double UonV2 = (ToReverse) ? Uf : Ul;
                        aLine2d = new Geom2d_Line(new gp_Pnt2d(UonV2, 0.0), new gp_Dir2d(gp_Dir2d.D.Y));
                        aLine2d2 = new Geom2d_Line(new gp_Pnt2d(UonV1, 0.0), new gp_Dir2d(gp_Dir2d.D.Y));
                        if (E3.IsSame(E4))
                        {
                            BB.UpdateEdge(E3, aLine2d, aLine2d2, theSurf, Loc, Precision.Confusion());
                            Geom_Curve BSplC34 = theSurf.UIso(Uf);
                            BB.UpdateEdge(E3, BSplC34, Precision.Confusion());
                            BB.Range(E3, Vf, Vl);
                        }
                        else
                        {
                            BB.SameParameter(E3, false);
                            BB.SameRange(E3, false);
                            BB.SameParameter(E4, false);
                            BB.SameRange(E4, false);
                            BB.UpdateEdge(E3, aLine2d, theSurf, Loc, Precision.Confusion());
                            BB.Range(E3, theSurf, Loc, Vf, Vl);
                            BB.UpdateEdge(E4, aLine2d2, theSurf, Loc, Precision.Confusion());
                            BB.Range(E4, theSurf, Loc, Vf, Vl);
                            Geom_Curve BSplC3 = theSurf.UIso(UonV2);
                            BB.UpdateEdge(E3, BSplC3, Precision.Confusion());
                            BB.Range(E3, Vf, Vl, true); // only for 3d curve
                            Geom_Curve BSplC4 = theSurf.UIso(UonV1);
                            BB.UpdateEdge(E4, BSplC4, Precision.Confusion());
                            BB.Range(E4, Vf, Vl, true); // only for 3d curve
                        }
                        NewFace = new BRepLib_MakeFace(theSurf, theWire).InternalFace();
                    }
                }
                if (!IsPlanar)
                {
                    double fparOE = BAcurveOE.FirstParameter();
                    double lparOE = BAcurveOE.LastParameter();
                    TopLoc_Location Loc = new();
                    if (Math.Abs(fpar - fparOE) > Precision.Confusion())
                    {
                        TopoDS_Edge anE4 = (ToReverse) ? E3 : E4;
                        gp_Pnt2d fp2d = EdgeLine2d!.Value(fpar);
                        gp_Pnt2d fp2dOE = OELine2d!.Value(fparOE);
                        aLine2d2 = new GC_MakeLine2d(fp2d, fp2dOE).Value();
                        Geom_Curve? aCurve = null;
                        double FirstPar = 0.0, LastPar = fp2d.Distance(fp2dOE);
                        Geom2dAdaptor_Curve AC2d = new(aLine2d2, FirstPar, LastPar);
                        GeomAdaptor_Surface GAsurf = new(theSurf!);
                        Geom2dAdaptor_Curve HC2d = AC2d; // OCCT copies the adaptors into handles
                        GeomAdaptor_Surface HSurf = GAsurf;
                        Adaptor3d_CurveOnSurface ConS = new(HC2d, HSurf);
                        GeomLib.BuildCurve3d(Precision.Confusion(), ConS, FirstPar, LastPar, ref aCurve,
                                             out double max_deviation, out double average_deviation);
                        BB.UpdateEdge(anE4, aCurve, max_deviation);
                        BB.UpdateEdge(anE4, aLine2d2, theSurf, Loc, max_deviation);
                        BB.Range(anE4, FirstPar, LastPar);
                    }
                    if (Math.Abs(lpar - lparOE) > Precision.Confusion())
                    {
                        TopoDS_Edge anE3 = (ToReverse) ? E4 : E3;
                        gp_Pnt2d lp2d = EdgeLine2d!.Value(lpar);
                        gp_Pnt2d lp2dOE = OELine2d!.Value(lparOE);
                        aLine2d = new GC_MakeLine2d(lp2d, lp2dOE).Value();
                        Geom_Curve? aCurve = null;
                        double FirstPar = 0.0, LastPar = lp2d.Distance(lp2dOE);
                        Geom2dAdaptor_Curve AC2d = new(aLine2d, FirstPar, LastPar);
                        GeomAdaptor_Surface GAsurf = new(theSurf!);
                        Geom2dAdaptor_Curve HC2d = AC2d; // OCCT copies the adaptors into handles
                        GeomAdaptor_Surface HSurf = GAsurf;
                        Adaptor3d_CurveOnSurface ConS = new(HC2d, HSurf);
                        GeomLib.BuildCurve3d(Precision.Confusion(), ConS, FirstPar, LastPar, ref aCurve,
                                             out double max_deviation, out double average_deviation);
                        BB.UpdateEdge(anE3, aCurve, max_deviation);
                        BB.UpdateEdge(anE3, aLine2d, theSurf, Loc, max_deviation);
                        BB.Range(anE3, FirstPar, LastPar);
                    }
                }

                if (!IsPlanar)
                {
                    // For planar faces these operations are useless,
                    // because there are no curves on surface
                    BRepLib.SameParameter(NewFace);
                    BRepTools.Update(NewFace);
                }
                // Check orientation
                TopAbs_Orientation anOr = OrientationOfEdgeInFace(anEdge, aFaceOfEdge);
                TopAbs_Orientation OrInNewFace = OrientationOfEdgeInFace(anEdge, NewFace);
                if (OrInNewFace != TopAbs.Reverse(anOr))
                {
                    NewFace.Reverse();
                }
                ///////////////////
                myWalls.Append(NewFace);
                if (ArcOnV2)
                {
                    TopoDS_Edge anArc = TopoDS.Edge(myInitOffsetEdge.Image(V2).First());
                    TopExp.Vertices(anArc, out TopoDS_Vertex arcV1, out TopoDS_Vertex arcV2);
                    bool ArcReverse = false;
                    if (!arcV1.IsSame(V3))
                    {
                        TopoDS_Vertex aVtx = arcV1;
                        arcV1 = arcV2;
                        arcV2 = aVtx;
                        ArcReverse = true;
                    }
                    TopoDS_Edge EA1, EA2;
                    // EA1 = (ToReverse)? E3 : TopoDS::Edge(E3.Reversed());
                    EA1 = E3;
                    EA1.Reverse();
                    if (ToReverse)
                    {
                        EA1.Reverse();
                    }
                    //////////////////////////////////////////////////////
                    if (V2.IsSame(StartVertex))
                    {
                        EA2 = StartEdge;
                    }
                    else
                    {
                        EA2 = new BRepLib_MakeEdge(V2, arcV2).InternalEdge();
                    }
                    anArc.Orientation(((ArcReverse) ? TopAbs_REVERSED : TopAbs_FORWARD));
                    if (EA1.Orientation() == TopAbs_REVERSED)
                    {
                        anArc.Reverse();
                    }
                    EA2.Orientation(TopAbs.Reverse(EA1.Orientation()));
                    TopoDS_Wire arcWire = new();
                    BB.MakeWire(ref arcWire);
                    BB.Add(arcWire, EA1);
                    BB.Add(arcWire, anArc);
                    BB.Add(arcWire, EA2);
                    BRepLib.BuildCurves3d(arcWire, myTol);
                    arcWire.Closed(true);
                    TopoDS_Face arcFace = new BRepLib_MakeFace(arcWire, true).InternalFace();
                    BRepTools.Update(arcFace);
                    myWalls.Append(arcFace);
                    TopoDS_Shape localEA2 = EA2.Oriented(TopAbs_FORWARD);
                    TopoDS_Edge CEA2 = TopoDS.Edge(localEA2);
                    PrevEdge = CEA2;
                    PrevVertex = V2;
                }
                else
                {
                    if (isBuildFromScratch)
                    {
                        PrevEdge = TopoDS.Edge(E4);
                        PrevVertex = V1;
                        isBuildFromScratch = false;
                    }
                    else
                    {
                        PrevEdge = E3;
                        PrevVertex = V2;
                    }
                }
                FirstStep = false;
            }
        }
    }

    //=================================================================================================

    private void MakeShells(Message_ProgressRange theRange)
    {
        // OCCT_DEBUG: the chronometer is left out.
        //
        using Message_ProgressScope aPS = new(theRange, "Making shells", 1);
        // Prepare list of splits of the offset faces to make the shells
        NCollection_List<TopoDS_Shape> aLSF = new();
        NCollection_List<TopoDS_Shape> R = myImageOffset.Roots();
        NCollection_List<TopoDS_Shape>.Iterator it = new(R);
        //
        for (; it.More(); it.Next())
        {
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Shape aF = it.Value();
            if (myThickening)
            { // offsetted faces must change their orientations
                aF.Reverse();
            }
            //
            NCollection_List<TopoDS_Shape> Image = new();
            myImageOffset.LastImage(aF, Image);
            NCollection_List<TopoDS_Shape>.Iterator it2 = new(Image);
            for (; it2.More(); it2.Next())
            {
                TopoDS_Shape aFIm = it2.Value();
                aLSF.Append(aFIm);
            }
        }
        //
        if (myThickening)
        {
            TopExp_Explorer Explo0 = new(myShape, TopAbs_FACE);
            for (; Explo0.More(); Explo0.Next())
            {
                TopoDS_Shape aF = Explo0.Current();
                aLSF.Append(aF);
            }
            //
            it.Initialize(myWalls);
            for (; it.More(); it.Next())
            {
                TopoDS_Shape aF = it.Value();
                aLSF.Append(aF);
            }
        }
        //
        if (aLSF.IsEmpty())
        {
            return;
        }
        //
        bool bDone = false;
        if ((myJoin == GeomAbs_Intersection) && myInter && !myThickening && myFaces.IsEmpty() && IsSolid(myShape)
            && myIsPlanar)
        {
            //
            TopoDS_Shape aShells = new();
            bDone = BuildShellsCompleteInter(aLSF, myImageOffset, ref aShells, aPS.Next());
            if (bDone)
            {
                myOffsetShape = aShells;
            }
        }
        //
        if (!bDone)
        {
            BRepTools_Quilt Glue = new();
            NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLSF);
            for (; aItLS.More(); aItLS.Next())
            {
                Glue.Add(aItLS.Value());
            }
            myOffsetShape = Glue.InternalShells();
        }
        //
        // Set correct value for closed flag
        TopExp_Explorer Explo = new(myOffsetShape, TopAbs_SHELL);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Shape aS = Explo.Current();
            if (!aS.Closed())
            {
                if (BRep_Tool.IsClosed(aS))
                {
                    aS.Closed(true);
                }
            }
        }
    }

    //=================================================================================================

    private void MakeSolid(Message_ProgressRange theRange)
    {
        if (myOffsetShape.IsNull())
        {
            return;
        }

        //  Modified by skv - Mon Apr  4 18:17:27 2005 Begin
        //  Supporting history.
        UpdateInitOffset(myInitOffsetFace, myImageOffset, myOffsetShape, TopAbs_FACE);
        UpdateInitOffset(myInitOffsetEdge, myImageOffset, myOffsetShape, TopAbs_EDGE);
        //  Modified by skv - Mon Apr  4 18:17:27 2005 End
        TopExp_Explorer exp = new();
        BRep_Builder B = new();
        int NbShell = 0;
        TopoDS_Compound NC = new();
        TopoDS_Shape S1 = new();
        B.MakeCompound(ref NC);

        using Message_ProgressScope aPS = new(theRange, "Making solid", 1);

        TopoDS_Solid Sol = new();
        B.MakeSolid(ref Sol);
        Sol.Closed(true);
        bool aMakeSolid = (myShape.ShapeType() == TopAbs_SOLID) || myThickening;
        for (exp.Init(myOffsetShape, TopAbs_SHELL); exp.More(); exp.Next())
        {
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
            TopoDS_Shell Sh = TopoDS.Shell(exp.Current());
            if (myThickening && myOffset > 0.0)
            {
                Sh.Reverse();
            }
            NbShell++;
            if (Sh.Closed() && aMakeSolid)
            {
                B.Add(Sol, Sh);
            }
            else
            {
                B.Add(NC, Sh);
                if (NbShell == 1)
                {
                    S1 = Sh;
                }
            }
        }
        int nbs = Sol.NbChildren();
        bool SolIsNull = (nbs == 0);
        // Checking solid
        if (nbs > 1)
        {
            BRepCheck_Analyzer aCheck = new(Sol, false);
            if (!aCheck.IsValid())
            {
                NCollection_List<TopoDS_Shape> aSolList = new();
                CorrectSolid(ref Sol, aSolList);
                if (!aSolList.IsEmpty())
                {
                    B.Add(NC, Sol);
                    NCollection_List<TopoDS_Shape>.Iterator aSLIt = new(aSolList);
                    for (; aSLIt.More(); aSLIt.Next())
                    {
                        B.Add(NC, aSLIt.Value());
                    }
                    SolIsNull = true;
                }
            }
        }
        bool NCIsNull = (NC.NbChildren() == 0);
        if ((!SolIsNull) && (!NCIsNull))
        {
            B.Add(NC, Sol);
            myOffsetShape = NC;
        }
        else if (SolIsNull && (!NCIsNull))
        {
            if (NbShell == 1)
            {
                myOffsetShape = S1;
            }
            else
            {
                myOffsetShape = NC;
            }
        }
        else if ((!SolIsNull) && NCIsNull)
        {
            myOffsetShape = Sol;
        }
        else
        {
            myOffsetShape = NC;
        }
    }

    //=================================================================================================

    private void SelectShells()
    {
        NCollection_Map<TopoDS_Shape> FreeEdges = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new(myFaceComp, TopAbs_EDGE);
        //-------------------------------------------------------------
        // FreeEdges all edges that can have free border in the
        // parallel shell
        // 1 - free borders of myShape .
        //-------------------------------------------------------------
        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(exp.Current());
            NCollection_List<TopoDS_Shape> LA = myAnalyse.Ancestors(E);
            if (LA.Extent() < 2)
            {
                if (myAnalyse.Type(E).First().Type() == ChFiDS_FreeBound)
                {
                    FreeEdges.Add(E);
                }
            }
        }
        // myShape has free borders and there are no caps
        // no unwinding 3d.
        if (!FreeEdges.IsEmpty() && myFaces.IsEmpty())
        {
            return;
        }

        myOffsetShape = BRepOffset_Tool.Deboucle3D(myOffsetShape, FreeEdges);
    }

    //=================================================================================================

    /// <summary>
    /// The links between the initial shapes and the offset faces: a copy whose roots are the caller's sub-shapes and
    /// whose images are the result's faces (frozen).
    /// </summary>
    public BRepAlgo_Image OffsetFacesFromShapes() => PublishImage(myInitOffsetFace);

    /// <summary>The links between the initial shapes and the offset faces, of the working copy, for ported code.</summary>
    internal BRepAlgo_Image InternalOffsetFacesFromShapes() => myInitOffsetFace;

    //  Modified by skv - Tue Mar 15 16:20:43 2005 Begin

    //=================================================================================================

    /// <summary>The join type.</summary>
    public GeomAbs_JoinType GetJoinType() => myJoin;

    //=================================================================================================

    /// <summary>
    /// The links between the initial shapes and the offset edges: a copy whose roots are the caller's sub-shapes and
    /// whose images are the result's edges (frozen).
    /// </summary>
    public BRepAlgo_Image OffsetEdgesFromShapes() => PublishImage(myInitOffsetEdge);

    /// <summary>The links between the initial shapes and the offset edges, of the working copy, for ported code.</summary>
    internal BRepAlgo_Image InternalOffsetEdgesFromShapes() => myInitOffsetEdge;

    //  Modified by skv - Tue Mar 15 16:20:43 2005 End

    //=================================================================================================

    /// <summary>The closing faces stored by <see cref="AddFace"/>, as the caller gave them.</summary>
    public NCollection_IndexedMap<TopoDS_Shape> ClosingFaces()
    {
        NCollection_IndexedMap<TopoDS_Shape> aFaces = new(TopTools_ShapeMapHasher.Instance);
        for (int i = 1; i <= myOriginalFaces.Extent(); ++i)
        {
            aFaces.Add(InternalOriginal(myOriginalFaces[i]));
        }
        return aFaces;
    }

    /// <summary>The closing faces, of the working copy, for ported code.</summary>
    internal NCollection_IndexedMap<TopoDS_Shape> InternalClosingFaces() => myOriginalFaces;

    /// <summary>The caller's sub-shape a working copy's sub-shape was made from (itself when there is none).</summary>
    private TopoDS_Shape InternalOriginal(in TopoDS_Shape theS) =>
        myEntry is null || !myEntry.IsCopied() ? theS : myEntry.Original(theS);

    /// <summary>
    /// A copy of an image whose roots are the caller's sub-shapes and whose images stand for the result's shapes, all
    /// frozen (decision 9201).
    /// </summary>
    private BRepAlgo_Image PublishImage(BRepAlgo_Image theImage)
    {
        BRepAlgo_Image aPublished = new();
        NCollection_Map<TopoDS_Shape> aDone = new(TopTools_ShapeMapHasher.Instance);
        for (NCollection_List<TopoDS_Shape>.Iterator itR = new(theImage.Roots()); itR.More(); itR.Next())
        {
            TopoDS_Shape aRoot = itR.Value();
            TopoDS_Shape aPubRoot = InternalOriginal(aRoot);
            aPubRoot.TShape()?.FreezeGraph();
            aPublished.SetRoot(aPubRoot);
            PublishImages(theImage, aRoot, aPubRoot, aPublished, aDone);
        }
        return aPublished;
    }

    private void PublishImages(BRepAlgo_Image theImage, in TopoDS_Shape theS, in TopoDS_Shape thePubS,
                               BRepAlgo_Image thePublished, NCollection_Map<TopoDS_Shape> theDone)
    {
        if (!theImage.HasImage(theS) || !theDone.Add(theS))
        {
            return;
        }
        NCollection_List<TopoDS_Shape> aLPub = new();
        NCollection_List<TopoDS_Shape> aLIm = theImage.Image(theS);
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLIm); it.More(); it.Next())
        {
            TopoDS_Shape aPub = InternalFinal(it.Value());
            aPub.TShape()?.FreezeGraph();
            aLPub.Append(aPub);
        }
        thePublished.Bind(thePubS, aLPub);
        NCollection_List<TopoDS_Shape>.Iterator itP = new(aLPub);
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLIm); it.More(); it.Next(), itP.Next())
        {
            if (!it.Value().IsSame(theS))
            {
                PublishImages(theImage, it.Value(), itP.Value(), thePublished, theDone);
            }
        }
    }

    //=================================================================================================

    private void EncodeRegularity()
    {
        // OCCT_DEBUG: the chronometer is left out.

        if (myOffsetShape.IsNull())
        {
            return;
        }
        // find edges G1 in the result
        TopExp_Explorer exp = new(myOffsetShape, TopAbs_EDGE);

        BRep_Builder B = new();
        NCollection_Map<TopoDS_Shape> MS = new(TopTools_ShapeMapHasher.Instance);

        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge OE = TopoDS.Edge(exp.Current());
            BRepLib.BuildCurve3d(OE, myTol);
            TopoDS_Edge ROE = OE;

            if (!MS.Add(OE))
            {
                continue;
            }

            if (myImageOffset.IsImage(OE))
            {
                ROE = TopoDS.Edge(myImageOffset.Root(OE));
            }

            NCollection_List<TopoDS_Shape> LofOF = myAsDes.Ascendant(ROE);

            if (LofOF.Extent() != 2)
            {
                // OCCT_DEBUG_VERB: " Edge shared by N Faces" left out.
                continue;
            }

            TopoDS_Face F1 = TopoDS.Face(LofOF.First());
            TopoDS_Face F2 = TopoDS.Face(LofOF.Last());

            if (F1.IsNull() || F2.IsNull())
            {
                continue;
            }

            TopoDS_Shape Root1 = myInitOffsetFace.Root(F1);
            TopoDS_Shape Root2 = myInitOffsetFace.Root(F2);

            TopAbs_ShapeEnum Type1 = Root1.ShapeType();
            TopAbs_ShapeEnum Type2 = Root2.ShapeType();

            if (F1.IsSame(F2))
            {
                if (BRep_Tool.IsClosed(OE, F1))
                {
                    // Temporary Debug for the Bench.
                    // Check with YFR.
                    // In mode intersection, the edges are not coded in myInitOffsetEdge
                    // so, manage case by case
                    // Note DUB; for Hidden parts, it is NECESSARY to code CN
                    // Analytic Surfaces.
                    if (myJoin == GeomAbs_Intersection)
                    {
                        BRepAdaptor_Surface BS = new(F1, false);
                        GeomAbs_SurfaceType SType = BS.GetType();
                        if (SType == GeomAbs_Cylinder || SType == GeomAbs_Cone || SType == GeomAbs_Sphere
                            || SType == GeomAbs_Torus)
                        {
                            B.Continuity(OE, F1, F1, GeomAbs_CN);
                        }
                        else
                        {
                            // See YFR : MaJ of myInitOffsetFace
                        }
                    }
                    else if (myInitOffsetEdge.IsImage(ROE))
                    {
                        if (Type1 == TopAbs_FACE && Type2 == TopAbs_FACE)
                        {
                            TopoDS_Face FRoot = TopoDS.Face(Root1);
                            TopoDS_Edge EI = TopoDS.Edge(myInitOffsetEdge.ImageFrom(ROE));
                            GeomAbs_Shape Conti = BRep_Tool.Continuity(EI, FRoot, FRoot);
                            if (Conti == GeomAbs_CN)
                            {
                                B.Continuity(OE, F1, F1, GeomAbs_CN);
                            }
                            else if (Conti > GeomAbs_C0)
                            {
                                B.Continuity(OE, F1, F1, GeomAbs_G1);
                            }
                        }
                    }
                }
                continue;
            }

            //  code regularities G1 between :
            //    - sphere and tube : one root is a vertex, the other is an edge
            //                        and the vertex is included in the edge
            //    - face and tube   : one root is a face, the other an edge
            //                        and the edge is included in the face
            //    - face and face    : if two root faces are tangent in
            //                        the initial shape, they will be tangent in the offset shape
            //    - tube and tube  : if 2 edges generating tubes are
            //                        tangents, the 2 will be tangent either.
            if (Type1 == TopAbs_EDGE && Type2 == TopAbs_VERTEX)
            {
                TopExp.Vertices(TopoDS.Edge(Root1), out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (V1.IsSame(Root2) || V2.IsSame(Root2))
                {
                    B.Continuity(OE, F1, F2, GeomAbs_G1);
                }
            }
            else if (Type1 == TopAbs_VERTEX && Type2 == TopAbs_EDGE)
            {
                TopExp.Vertices(TopoDS.Edge(Root2), out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (V1.IsSame(Root1) || V2.IsSame(Root1))
                {
                    B.Continuity(OE, F1, F2, GeomAbs_G1);
                }
            }
            else if (Type1 == TopAbs_FACE && Type2 == TopAbs_EDGE)
            {
                TopExp_Explorer exp2 = new(Root1, TopAbs_EDGE);
                for (; exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(Root2))
                    {
                        B.Continuity(OE, F1, F2, GeomAbs_G1);
                        break;
                    }
                }
            }
            else if (Type1 == TopAbs_EDGE && Type2 == TopAbs_FACE)
            {
                TopExp_Explorer exp2 = new(Root2, TopAbs_EDGE);
                for (; exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(Root1))
                    {
                        B.Continuity(OE, F1, F2, GeomAbs_G1);
                        break;
                    }
                }
            }
            else if (Type1 == TopAbs_FACE && Type2 == TopAbs_FACE)
            {
                //  if two root faces are tangent in
                //  the initial shape, they will be tangent in the offset shape
                NCollection_List<TopoDS_Shape> LE = new();
                BRepOffset_Tool.FindCommonShapes(Root1, Root2, TopAbs_EDGE, LE);
                if (LE.Extent() == 1)
                {
                    TopoDS_Edge Ed = TopoDS.Edge(LE.First());
                    if (myAnalyse.HasAncestor(Ed))
                    {
                        NCollection_List<BRepOffset_Interval> LI = myAnalyse.Type(Ed);
                        if (LI.Extent() == 1 && LI.First().Type() == ChFiDS_Tangential)
                        {
                            B.Continuity(OE, F1, F2, GeomAbs_G1);
                        }
                    }
                }
            }
            else if (Type1 == TopAbs_EDGE && Type2 == TopAbs_EDGE)
            {
                NCollection_List<TopoDS_Shape> LV = new();
                BRepOffset_Tool.FindCommonShapes(Root1, Root2, TopAbs_VERTEX, LV);
                if (LV.Extent() == 1)
                {
                    NCollection_List<TopoDS_Shape> LEdTg = new();
                    myAnalyse.TangentEdges(TopoDS.Edge(Root1), TopoDS.Vertex(LV.First()), LEdTg);
                    NCollection_List<TopoDS_Shape>.Iterator it = new(LEdTg);
                    for (; it.More(); it.Next())
                    {
                        if (it.Value().IsSame(Root2))
                        {
                            B.Continuity(OE, F1, F2, GeomAbs_G1);
                            break;
                        }
                    }
                }
            }
        }

        // OCCT_DEBUG: the chronometer is left out.
    }

    //=================================================================================================

    private static double ComputeMaxDist(in gp_Pln thePlane, Geom_Curve? theCrv, double theFirst, double theLast)
    {
        double aMaxDist = 0.0;
        int i, NCONTROL = 23;
        double aPrm, aDist2;
        gp_Pnt aP;
        for (i = 0; i < NCONTROL; i++)
        {
            aPrm = ((NCONTROL - 1 - i) * theFirst + i * theLast) / (NCONTROL - 1);
            aP = theCrv!.Value(aPrm);
            if (Precision.IsInfinite(aP.X()) || Precision.IsInfinite(aP.Y()) || Precision.IsInfinite(aP.Z()))
            {
                return Precision.Infinite();
            }
            aDist2 = thePlane.SquareDistance(aP);
            if (aDist2 > aMaxDist)
            {
                aMaxDist = aDist2;
            }
        }
        return Math.Sqrt(aMaxDist) * 1.05;
    }

    //=================================================================================================

    // OCCT.NET: Locked(false) acts on the offset's own shapes: the result's new edges and vertices, and, in the
    // thickening mode, the copy of the caller's shape made at the entry (decision 1005).
    private static void UpdateTolerance(ref TopoDS_Shape S, NCollection_IndexedMap<TopoDS_Shape> Faces,
                                        in TopoDS_Shape theInitShape)
    {
        BRep_Builder B = new();
        NCollection_Map<TopoDS_Shape> View = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Vertex[] V = new TopoDS_Vertex[2];

        // The edges of caps are not modified.
        int j;
        for (j = 1; j <= Faces.Extent(); j++)
        {
            TopoDS_Shape F = Faces[j];
            TopExp_Explorer Exp = new();
            for (Exp.Init(F, TopAbs_EDGE); Exp.More(); Exp.Next())
            {
                View.Add(Exp.Current());
            }
        }

        // The edges of initial shape are  not modified
        NCollection_Map<TopoDS_Shape> aMapInitF = new(TopTools_ShapeMapHasher.Instance);
        if (!theInitShape.IsNull())
        {
            TopExp_Explorer anExpF0 = new(theInitShape, TopAbs_FACE);
            for (; anExpF0.More(); anExpF0.Next())
            {
                aMapInitF.Add(anExpF0.Current());
                TopExp_Explorer anExpE = new();
                for (anExpE.Init(anExpF0.Current(), TopAbs_EDGE); anExpE.More(); anExpE.Next())
                {
                    View.Add(anExpE.Current());
                    TopoDS_Iterator anItV = new(anExpE.Current());
                    for (; anItV.More(); anItV.Next())
                    {
                        View.Add(anItV.Value());
                    }
                }
            }
        }

        double Tol;
        TopExp_Explorer anExpF = new(S, TopAbs_FACE);
        for (; anExpF.More(); anExpF.Next())
        {
            TopoDS_Shape F = anExpF.Current();
            if (Faces.Contains(F) || aMapInitF.Contains(F))
            {
                continue;
            }
            BRepAdaptor_Surface aBAS = new(TopoDS.Face(F), false);
            TopExp_Explorer Exp = new();
            for (Exp.Init(F, TopAbs_EDGE); Exp.More(); Exp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(Exp.Current());
                bool isUpdated = false;
                double aCurrTol = BRep_Tool.Tolerance(E);
                if (aBAS.GetType() == GeomAbs_Plane)
                {
                    // Edge does not seem to have pcurve on plane,
                    // so EdgeCorrector does not include it in tolerance calculation
                    Geom_Curve? aCrv = BRep_Tool.Curve(E, out double aFirst, out double aLast);
                    double aMaxDist = ComputeMaxDist(aBAS.Plane(), aCrv, aFirst, aLast);
                    if (aMaxDist > aCurrTol)
                    {
                        B.UpdateEdge(E, aMaxDist);
                        isUpdated = true;
                    }
                }
                if (View.Add(E))
                {
                    E.Locked(false);
                    BRepCheck_Edge EdgeCorrector = new(E);
                    Tol = EdgeCorrector.Tolerance();
                    if (Tol > aCurrTol)
                    {
                        B.UpdateEdge(E, Tol);
                        isUpdated = true;
                    }
                }
                if (isUpdated)
                {
                    Tol = BRep_Tool.Tolerance(E);
                    // Update the vertices.
                    TopExp.Vertices(E, out V[0], out V[1]);

                    for (int i = 0; i <= 1; i++)
                    {
                        V[i].Locked(false);
                        if (View.Add(V[i]))
                        {
                            BRep_TVertex TV = (BRep_TVertex)V[i].TShape()!;
                            TV.Tolerance(0.0);
                            BRepCheck_Vertex VertexCorrector = new(V[i]);
                            B.UpdateVertex(V[i], VertexCorrector.Tolerance());
                            // use the occasion to clean the vertices.
                            (TV.ChangePoints()).Clear();
                        }
                        B.UpdateVertex(V[i], Tol);
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static void CorrectSolid(ref TopoDS_Solid theSol, NCollection_List<TopoDS_Shape> theSolList)
    {
        BRep_Builder aBB = new();
        TopoDS_Shape anOuterShell = new();
        NCollection_List<double> aVols = new();
        double aVolMax = 0.0, anOuterVol = 0.0;

        TopoDS_Iterator anIt = new(theSol);
        for (; anIt.More(); anIt.Next())
        {
            TopoDS_Shape aSh = anIt.Value();
            GProp_GProps aVProps = new();
            BRepGProp.VolumeProperties(aSh, aVProps, true);
            if (Math.Abs(aVProps.Mass()) > aVolMax)
            {
                anOuterVol = aVProps.Mass();
                aVolMax = Math.Abs(anOuterVol);
                anOuterShell = aSh;
            }
            aVols.Append(aVProps.Mass());
        }
        //
        if (Math.Abs(anOuterVol) < Precision.Confusion())
        {
            return;
        }
        if (anOuterVol < 0.0)
        {
            anOuterShell.Reverse();
        }
        TopoDS_Solid aNewSol = new();
        aBB.MakeSolid(ref aNewSol);
        aNewSol.Closed(true);
        aBB.Add(aNewSol, anOuterShell);
        BRepClass3d_SolidClassifier aSolClass = new(aNewSol);
        //
        anIt.Initialize(theSol);
        NCollection_List<double>.Iterator aVIt = new(aVols);
        for (; anIt.More(); anIt.Next(), aVIt.Next())
        {
            TopoDS_Shell aSh = TopoDS.Shell(anIt.Value());
            if (aSh.IsSame(anOuterShell))
            {
                continue;
            }
            else
            {
                TopExp_Explorer aVExp = new(aSh, TopAbs_VERTEX);
                TopoDS_Vertex aV = TopoDS.Vertex(aVExp.Current());
                gp_Pnt aP = BRep_Tool.Pnt(aV);
                aSolClass.Perform(aP, BRep_Tool.Tolerance(aV));
                if (aSolClass.State() == TopAbs_IN)
                {
                    if (aVIt.Value() > 0.0)
                    {
                        aSh.Reverse();
                    }
                    aBB.Add(aNewSol, aSh);
                }
                else
                {
                    if (aVIt.Value() < 0.0)
                    {
                        aSh.Reverse();
                    }
                    TopoDS_Solid aSol = new();
                    aBB.MakeSolid(ref aSol);
                    aSol.Closed(true);
                    aBB.Add(aSol, aSh);
                    theSolList.Append(aSol);
                }
            }
        }
        theSol = aNewSol;
    }

    //=======================================================================
    // function : CheckInputData
    // purpose  : Check input data for possibility of offset perform.
    //=======================================================================

    /// <summary>
    /// The pre-analysis: whether the offset can be made (see <see cref="Error"/>). Checks, in order, that some face
    /// has a non-null offset, that the shell is connected, that the surfaces aren't C0, and that their normals exist on
    /// a grid of points.
    /// </summary>
    public bool CheckInputData(Message_ProgressRange theRange)
    {
        // Set initial error state.
        myError = BRepOffset_NoError;
        TopoDS_Shape aTmpShape = new();
        myBadShape = aTmpShape;
        using Message_ProgressScope aPS = new(theRange, null, 1);
        // Non-null offset.
        if (Math.Abs(myOffset) <= myTol)
        {
            bool isFound = false;
            foreach (KeyValuePair<TopoDS_Shape, double> anIter in myFaceOffset)
            {
                if (Math.Abs(anIter.Value) > myTol)
                {
                    isFound = true;
                    break;
                }
            }

            if (!isFound)
            {
                // No face with non-null offset found.
                myError = BRepOffset_NullOffset;
                return false;
            }
        }

        // Connectivity of input shape.
        if (!IsConnectedShell(myFaceComp))
        {
            myError = BRepOffset_NotConnectedShell;
            return false;
        }

        // Normals check and continuity check.
        const int aPntPerDim = 20; // 21 points on each dimension.
        TopExp_Explorer anExpSF = new(myFaceComp, TopAbs_FACE);
        HashSet<TopoDS_TShape> aPresenceMap = new(ReferenceEqualityComparer.Instance);
        gp_Pnt2d aPnt2d;
        for (; anExpSF.More(); anExpSF.Next())
        {
            if (!aPS.More())
            {
                myError = BRepOffset_UserBreak;
                return false;
            }
            TopoDS_Face aF = TopoDS.Face(anExpSF.Current());

            if (aPresenceMap.Contains(aF.TShape()!))
            {
                // Not perform computations with partner shapes,
                // since they are contain same geometry.
                continue;
            }
            aPresenceMap.Add(aF.TShape()!);

            Geom_Surface aSurf = BRep_Tool.Surface(aF, out TopLoc_Location L)!;
            BRepTools.UVBounds(aF, out double aUmin, out double aUmax, out double aVmin, out double aVmax);

            // Continuity check.
            if (aSurf.Continuity() == GeomAbs_C0)
            {
                myError = BRepOffset_C0Geometry;
                return false;
            }

            // Get degenerated points, to avoid check them.
            NCollection_DynamicArray<gp_Pnt> aBad3dPnts = new();
            TopExp_Explorer anExpFE = new(aF, TopAbs_EDGE);
            for (; anExpFE.More(); anExpFE.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(anExpFE.Current());
                if (BRep_Tool.Degenerated(aE))
                {
                    aBad3dPnts.Append(BRep_Tool.Pnt((TopExp.FirstVertex(aE))));
                }
            }

            // Geometry grid check.
            for (int i = 0; i <= aPntPerDim; i++)
            {
                double aUParam = aUmin + (aUmax - aUmin) * i / aPntPerDim;
                for (int j = 0; j <= aPntPerDim; j++)
                {
                    double aVParam = aVmin + (aVmax - aVmin) * j / aPntPerDim;

                    myError = checkSinglePoint(aUParam, aVParam, aSurf, aBad3dPnts);
                    if (myError != BRepOffset_NoError)
                    {
                        return false;
                    }
                }
            }

            // Vertex list check.
            TopExp_Explorer anExpFV = new(aF, TopAbs_VERTEX);
            for (; anExpFV.More(); anExpFV.Next())
            {
                TopoDS_Vertex aV = TopoDS.Vertex(anExpFV.Current());
                aPnt2d = BRep_Tool.Parameters(aV, aF);

                myError = checkSinglePoint(aPnt2d.X(), aPnt2d.Y(), aSurf, aBad3dPnts);
                if (myError != BRepOffset_NoError)
                {
                    return false;
                }
            }
        }

        return true;
    }

    //=======================================================================
    // function : GetBadShape
    // purpose  : Get shape where problems detected.
    //=======================================================================

    /// <summary>The shape where the pre-analysis found a problem (frozen).</summary>
    public TopoDS_Shape GetBadShape()
    {
        TopoDS_Shape aBad = InternalOriginal(myBadShape);
        aBad.TShape()?.FreezeGraph();
        return aBad;
    }

    //=================================================================================================

    private void RemoveInternalEdges()
    {
        bool bRemoveWire, bRemoveEdge;
        TopExp_Explorer aExpF = new(), aExpW = new(), aExpE = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMELF = new(TopTools_ShapeMapHasher.Instance);
        //
        TopExp.MapShapesAndAncestors(myOffsetShape, TopAbs_EDGE, TopAbs_FACE, aDMELF);
        //
        aExpF.Init(myOffsetShape, TopAbs_FACE);
        for (; aExpF.More(); aExpF.Next())
        {
            TopoDS_Shape aF = aExpF.Current();
            //
            NCollection_List<TopoDS_Shape> aLIW = new();
            //
            aExpW.Init(aF, TopAbs_WIRE);
            for (; aExpW.More(); aExpW.Next())
            {
                TopoDS_Shape aW = aExpW.Current();
                //
                bRemoveWire = true;
                NCollection_List<TopoDS_Shape> aLIE = new();
                //
                aExpE.Init(aW, TopAbs_EDGE);
                for (; aExpE.More(); aExpE.Next())
                {
                    TopoDS_Shape aE = aExpE.Current();
                    if (aE.Orientation() != TopAbs_INTERNAL)
                    {
                        bRemoveWire = false;
                        continue;
                    }
                    //
                    NCollection_List<TopoDS_Shape> aLF = aDMELF.FindFromKey(aE);
                    bRemoveEdge = (aLF.Extent() == 1);
                    if (bRemoveEdge)
                    {
                        aLIE.Append(aE);
                    }
                    else
                    {
                        bRemoveWire = false;
                    }
                }
                //
                if (bRemoveWire)
                {
                    aLIW.Append(aW);
                }
                else if (aLIE.Extent() != 0)
                {
                    RemoveShapes(ref aW, aLIE);
                }
            }
            //
            if (aLIW.Extent() != 0)
            {
                RemoveShapes(ref aF, aLIW);
            }
        }
    }

    //=======================================================================
    // static methods implementation
    //=======================================================================

    //=======================================================================
    // function : checkSinglePoint
    // purpose  : Check single point on surface for bad normals
    //=======================================================================
    private static BRepOffset_Error checkSinglePoint(double theUParam, double theVParam, Geom_Surface theSurf,
                                                     NCollection_DynamicArray<gp_Pnt> theBadPoints)
    {
        theSurf.D1(theUParam, theVParam, out gp_Pnt aPnt, out gp_Vec aD1U, out gp_Vec aD1V);

        if (aD1U.SquareMagnitude() < Precision.SquareConfusion() || aD1V.SquareMagnitude() < Precision.SquareConfusion())
        {
            bool isKnownBadPnt = false;
            for (int anIdx = theBadPoints.Lower(); anIdx <= theBadPoints.Upper(); ++anIdx)
            {
                if (aPnt.SquareDistance(theBadPoints[anIdx]) < Precision.SquareConfusion())
                {
                    isKnownBadPnt = true;
                    break;
                }
            } // for(int anIdx  = theBadPoints.Lower();

            if (!isKnownBadPnt)
            {
                return BRepOffset_BadNormalsOnGeometry;
            }
            else
            {
                return BRepOffset_NoError;
            }
        } //  if (aD1U.SquareMagnitude() < Precision::SquareConfusion() ||

        if (aD1U.IsParallel(aD1V, Precision.Confusion()))
        {
            // Isolines are collinear.
            return BRepOffset_BadNormalsOnGeometry;
        }

        return BRepOffset_NoError;
    }

    //=======================================================================
    // function : RemoveShapes
    // purpose  : Removes the shapes <theLS> from the shape <theS>
    //=======================================================================
    private static void RemoveShapes(ref TopoDS_Shape theS, NCollection_List<TopoDS_Shape> theLS)
    {
        BRep_Builder aBB = new();
        //
        bool bFree = theS.Free();
        theS.Free(true);
        //
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLS);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSI = aIt.Value();
            aBB.Remove(theS, aSI);
        }
        //
        theS.Free(bFree);
    }

    //=======================================================================
    // function : UpdateHistory
    // purpose  : Updates the history information
    //=======================================================================
    private static void UpdateHistory(NCollection_List<TopoDS_Shape> theLF, BOPAlgo_Builder theGF, BRepAlgo_Image theImage)
    {
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLF);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aF = aIt.Value();
            NCollection_List<TopoDS_Shape> aLFIm = new(theGF.Modified(aF));
            if (aLFIm.Extent() != 0)
            {
                if (theImage.HasImage(aF))
                {
                    theImage.Add(aF, aLFIm);
                }
                else
                {
                    theImage.Bind(aF, aLFIm);
                }
            }
        }
    }

    //=================================================================================================

    private void IntersectEdges(NCollection_List<TopoDS_Shape> theFaces,
                                NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> theMapSF,
                                NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theMES,
                                NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theBuild, ref BRepAlgo_AsDes theAsDes,
                                ref BRepAlgo_AsDes theAsDes2d, Message_ProgressRange theRange)
    {
        double aTolF;
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVV = new(TopTools_ShapeMapHasher.Instance);
        // intersect edges created from edges
        NCollection_IndexedMap<TopoDS_Shape> aMFV = new(TopTools_ShapeMapHasher.Instance);
        using Message_ProgressScope aPSOuter = new(theRange, null, 2);
        using Message_ProgressScope aPS1 = new(aPSOuter.Next(), null, theFaces.Extent());
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(theFaces); it.More(); it.Next())
        {
            TopoDS_Face aF = TopoDS.Face(it.Value());
            aTolF = BRep_Tool.Tolerance(aF);
            if (!BRepOffset_Inter2d.ConnexIntByInt(aF, theMapSF[aF], theMES, theBuild, theAsDes, theAsDes2d, myOffset,
                                                   aTolF, myAnalyse, aMFV, myImageVV, myEdgeIntEdges, aDMVV,
                                                   aPS1.Next()))
            {
                myError = BRepOffset_CannotExtentEdge;
                return;
            }
            if (!aPS1.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
        }
        // intersect edges created from vertices
        int i, aNbF = aMFV.Extent();
        using Message_ProgressScope aPS2 = new(aPSOuter.Next(), "Intersecting edges created from vertices", aNbF);
        for (i = 1; i <= aNbF; ++i)
        {
            TopoDS_Face aF = TopoDS.Face(aMFV[i]);
            aTolF = BRep_Tool.Tolerance(aF);
            BRepOffset_Inter2d.ConnexIntByIntInVert(aF, theMapSF[aF], theMES, theBuild, theAsDes, theAsDes2d, aTolF,
                                                    myAnalyse, aDMVV, aPS2.Next());
            if (!aPS2.More())
            {
                myError = BRepOffset_UserBreak;
                return;
            }
        }
        //
        // fuse vertices on edges
        if (!BRepOffset_Inter2d.FuseVertices(aDMVV, theAsDes2d, myImageVV))
        {
            myError = BRepOffset_CannotFuseVertices;
            return;
        }
    }

    //=================================================================================================

    private static bool TrimEdges(in TopoDS_Shape theShape, double theOffset, BRepOffset_Analyse Analyse,
                                  NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> theMapSF,
                                  NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theMES,
                                  NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theBuild, ref BRepAlgo_AsDes theAsDes,
                                  ref BRepAlgo_AsDes theAsDes2d, NCollection_IndexedMap<TopoDS_Shape> theNewEdges,
                                  NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theETrimEInf,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEdgesOrigins)
    {
        TopExp_Explorer Exp = new(), Exp2 = new(), ExpC = new();
        TopoDS_Shape NE;
        TopoDS_Face NF;

        NCollection_List<TopoDS_Shape> aLFaces = new();
        for (Exp.Init(theShape, TopAbs_FACE); Exp.More(); Exp.Next())
        {
            aLFaces.Append(Exp.Current());
        }

        NCollection_Map<TopoDS_Shape> aMFGenerated = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEF = new(TopTools_ShapeMapHasher.Instance);
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(Analyse.NewFaces()); it.More(); it.Next())
        {
            TopoDS_Shape aFG = it.Value();
            aLFaces.Append(aFG);
            aMFGenerated.Add(aFG);
            TopExp.MapShapesAndUniqueAncestors(aFG, TopAbs_EDGE, TopAbs_FACE, aDMEF);
        }

        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLFaces); it.More(); it.Next())
        {
            TopoDS_Face FI = TopoDS.Face(it.Value());
            NF = theMapSF[FI].InternalFace();
            if (theMES.IsBound(NF))
            {
                NF = TopoDS.Face(theMES[NF]);
            }
            //
            NCollection_Map<TopoDS_Shape> View = new(TopTools_ShapeMapHasher.Instance);
            NCollection_IndexedMap<TopoDS_Shape> VEmap = new(TopTools_ShapeMapHasher.Instance);
            int i, aNb;
            //
            TopExp.MapShapes(FI.Oriented(TopAbs_FORWARD), TopAbs_EDGE, VEmap);
            TopExp.MapShapes(FI.Oriented(TopAbs_FORWARD), TopAbs_VERTEX, VEmap);
            //
            aNb = VEmap.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                TopoDS_Shape aS = VEmap[i];
                if (!View.Add(aS))
                {
                    continue;
                }
                //
                if (theBuild.IsBound(aS))
                {
                    NE = theBuild[aS];
                    // keep connection to original edges
                    ExpC.Init(NE, TopAbs_EDGE);
                    for (; ExpC.More(); ExpC.Next())
                    {
                        TopoDS_Edge NEC = TopoDS.Edge(ExpC.Current());
                        if (!theEdgesOrigins.IsBound(NEC))
                        {
                            theEdgesOrigins.Bind(NEC, new NCollection_List<TopoDS_Shape>());
                        }
                        AppendToList(theEdgesOrigins[NEC], aS);
                    }
                    // trim edges
                    if (NE.ShapeType() == TopAbs_EDGE)
                    {
                        if (AddNew(theNewEdges, NE))
                        {
                            TopoDS_Edge aNE = TopoDS.Edge(NE);
                            if (!TrimEdge(ref aNE, theAsDes2d, ref theAsDes, theETrimEInf))
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        //------------------------------------------------------------
                        // The Intersections are on several edges.
                        // The pieces without intersections with neighbors
                        // are removed from AsDes.
                        //------------------------------------------------------------
                        for (ExpC.Init(NE, TopAbs_EDGE); ExpC.More(); ExpC.Next())
                        {
                            TopoDS_Edge NEC = TopoDS.Edge(ExpC.Current());
                            if (AddNew(theNewEdges, NEC))
                            {
                                if (!theAsDes2d.Descendant(NEC).IsEmpty())
                                {
                                    if (!TrimEdge(ref NEC, theAsDes2d, ref theAsDes, theETrimEInf))
                                    {
                                        return false;
                                    }
                                }
                                else
                                {
                                    if (theAsDes.HasAscendant(NEC))
                                    {
                                        theAsDes.Remove(NEC);
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (aS.ShapeType() != TopAbs_EDGE)
                    {
                        continue;
                    }
                    if (aMFGenerated.Contains(FI) && aDMEF.FindFromKey(aS).Extent() == 1)
                    {
                        continue;
                    }

                    NE = theMapSF[FI].InternalGenerated(aS);
                    //// modified by jgv, 19.12.03 for OCC4455 ////
                    NE.Orientation(aS.Orientation());
                    //
                    if (!theEdgesOrigins.IsBound(NE))
                    {
                        theEdgesOrigins.Bind(NE, new NCollection_List<TopoDS_Shape>());
                    }
                    AppendToList(theEdgesOrigins[NE], aS);
                    //
                    if (theMES.IsBound(NE))
                    {
                        NE = theMES[NE];
                        NE.Orientation(aS.Orientation());
                        if (AddNew(theNewEdges, NE))
                        {
                            TopoDS_Edge aNE = TopoDS.Edge(NE);
                            if (!TrimEdge(ref aNE, theAsDes2d, ref theAsDes, theETrimEInf))
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        TopoDS_Edge anEdge = TopoDS.Edge(NE);
                        BRepAdaptor_Curve aBAC = new(anEdge);
                        if (aBAC.GetType() == GeomAbs_Line)
                        {
                            BRepOffset_Inter2d.ExtentEdge(anEdge, out TopoDS_Edge aNewEdge, theOffset);
                            theETrimEInf.Bind(anEdge, aNewEdge);
                        }
                    }
                    theAsDes.Add(NF, NE);
                }
            }
        }
        return true;
    }

    // OCCT: "if (theNewEdges.Add(NE))" tests the index NCollection_IndexedMap::Add returns, never 0, so an edge already
    // in the map is trimmed again (occt-issues.md).
    private static bool AddNew(NCollection_IndexedMap<TopoDS_Shape> theMap, in TopoDS_Shape theS)
    {
        return theMap.Add(theS) != 0;
    }

    //=======================================================================
    // function : TrimEdge
    // purpose  : Trim the edge of the largest of descendants in AsDes2d.
    //           Order in AsDes two vertices that have trimmed the edge.
    //=======================================================================
    private static bool TrimEdge(ref TopoDS_Edge NE, BRepAlgo_AsDes AsDes2d, ref BRepAlgo_AsDes AsDes,
                                 NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theETrimEInf)
    {
        //
        TopExp.Vertices(NE, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        BRep_Tool.Range(NE, out double aT1, out double aT2);
        //
        BOPTools_AlgoTools.MakeSplitEdge(NE, V1, aT1, V2, aT2, out TopoDS_Edge aSourceEdge);
        //
        //
        double aSameParTol = Precision.Confusion();

        double U = 0.0;
        double UMin = Precision.Infinite();
        double UMax = -UMin;

        NCollection_List<TopoDS_Shape> LE = AsDes2d.Descendant(NE);
        //
        bool bTrim = false;
        //
        if (LE.Extent() > 1)
        {
            NCollection_List<TopoDS_Shape>.Iterator it = new(LE);
            for (; it.More(); it.Next())
            {
                TopoDS_Vertex V = TopoDS.Vertex(it.Value());
                if (NE.Orientation() == TopAbs_REVERSED)
                {
                    V.Reverse();
                }
                // V.Orientation(TopAbs_INTERNAL);
                if (!FindParameter(V, NE, ref U))
                {
                    Geom_Curve theCurve = BRep_Tool.Curve(NE, out double f, out double l)!;
                    gp_Pnt thePoint = BRep_Tool.Pnt(V);
                    GeomAPI_ProjectPointOnCurve Projector = new(thePoint, theCurve);
                    if (Projector.NbPoints() == 0)
                    {
                        return false;
                    }
                    U = Projector.LowerDistanceParameter();
                }
                if (U < UMin)
                {
                    UMin = U;
                    V1 = V;
                }
                if (U > UMax)
                {
                    UMax = U;
                    V2 = V;
                }
            }
            //
            if (V1.IsNull() || V2.IsNull())
            {
                return false;
            }
            if (!V1.IsSame(V2))
            {
                NE.Free(true);
                BRep_Builder B = new();
                TopAbs_Orientation Or = NE.Orientation();
                NE.Orientation(TopAbs_FORWARD);
                TopExp.Vertices(NE, out TopoDS_Vertex VF, out TopoDS_Vertex VL);
                B.Remove(NE, VF);
                B.Remove(NE, VL);
                B.Add(NE, V1.Oriented(TopAbs_FORWARD));
                B.Add(NE, V2.Oriented(TopAbs_REVERSED));
                B.Range(NE, UMin, UMax);
                NE.Orientation(Or);
                AsDes.Add(NE, V1.Oriented(TopAbs_FORWARD));
                AsDes.Add(NE, V2.Oriented(TopAbs_REVERSED));
                BRepLib.SameParameter(NE, aSameParTol, true);
                //
                bTrim = true;
            }
        }
        //
        if (!bTrim)
        {
            BRepAdaptor_Curve aBAC = new(NE);
            if (aBAC.GetType() == GeomAbs_Line)
            {
                if (AsDes.HasAscendant(NE))
                {
                    AsDes.Remove(NE);
                }
            }
        }
        else
        {
            if (!theETrimEInf.IsBound(NE))
            {
                theETrimEInf.Bind(NE, aSourceEdge);
            }
        }
        return true;
    }

    //=================================================================================================

    private static void GetEnlargedFaces(NCollection_List<TopoDS_Shape> theFaces,
                                         NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> theMapSF,
                                         NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theMES,
                                         NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theFacesOrigins,
                                         BRepAlgo_Image theImage, NCollection_List<TopoDS_Shape> theLSF)
    {
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(theFaces); it.More(); it.Next())
        {
            TopoDS_Shape FI = it.Value();
            TopoDS_Shape OFI = theMapSF[FI].InternalFace();
            if (theMES.IsBound(OFI))
            {
                TopoDS_Face aLocalFace = TopoDS.Face(theMES[OFI]);
                theLSF.Append(aLocalFace);
                theImage.SetRoot(aLocalFace);
                theFacesOrigins.Bind(aLocalFace, FI);
            }
        }
    }

    //=======================================================================
    // function : BuildShellsCompleteInter
    // purpose  : Make the shells from list of faces using MakerVolume algorithm.
    //           In case there will be more than just one solid, it will be
    //           rebuilt using only outer faces.
    //=======================================================================
    private static bool BuildShellsCompleteInter(NCollection_List<TopoDS_Shape> theLF, BRepAlgo_Image theImage,
                                                 ref TopoDS_Shape theShells, Message_ProgressRange theRange)
    {
        using Message_ProgressScope aPS = new(theRange, null, 5);
        // make solids
        BOPAlgo_MakerVolume aMV1 = new();
        aMV1.SetArguments(theLF);
        // we need to intersect the faces to process the tangential faces
        aMV1.SetIntersect(true);
        aMV1.SetAvoidInternalShapes(true);
        aMV1.Perform(aPS.Next(3));
        //
        bool bDone = !aMV1.HasErrors();
        if (!bDone)
        {
            return bDone;
        }
        //
        UpdateHistory(theLF, aMV1, theImage);
        //
        TopoDS_Shape aResult1 = aMV1.InternalShape();
        if (aResult1.ShapeType() == TopAbs_SOLID)
        {
            // result is the alone solid, nothing to do
            return GetSubShapes(aResult1, TopAbs_SHELL, ref theShells);
        }

        // OCCT's incremental allocators are left out.

        // Since the <theImage> object does not support multiple ancestors,
        // prepare local copy of the origins, which will be used to resolve
        // non-manifold solids produced by Maker Volume algorithm by comparison
        // of the normal directions of the split faces with their origins.
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anOrigins = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLR = new(theImage.Roots());
        for (; aItLR.More(); aItLR.Next())
        {
            TopoDS_Shape aFR = aItLR.Value();

            // Find the last splits of the root face, including the ones
            // created during MakeVolume operation
            NCollection_List<TopoDS_Shape> aLFIm = new();
            theImage.LastImage(aFR, aLFIm);

            NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFIm);
            for (; aItLFIm.More(); aItLFIm.Next())
            {
                TopoDS_Shape aFIm = aItLFIm.Value();
                if (!anOrigins.Find(aFIm, out NCollection_List<TopoDS_Shape> pLFOr))
                {
                    pLFOr = new NCollection_List<TopoDS_Shape>();
                    anOrigins.Bind(aFIm, pLFOr);
                }
                pLFOr.Append(aFR);
            }
        }

        // It is necessary to rebuild the solids, avoiding internal faces
        // Map faces to solids
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMFS = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aResult1, TopAbs_FACE, TopAbs_SOLID, aDMFS);
        //
        int i, aNb = aDMFS.Extent();
        bDone = (aNb > 0);
        if (!bDone)
        {
            // unable to build any solid
            return bDone;
        }
        //
        // get faces attached to only one solid
        NCollection_List<TopoDS_Shape> aLF = new();
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<TopoDS_Shape> aLS = aDMFS[i];
            if (aLS.Extent() == 1)
            {
                TopoDS_Shape aF = aDMFS.FindKey(i);
                aLF.Append(aF);
            }
        }
        //
        // make solids from the new list
        BOPAlgo_MakerVolume aMV2 = new();
        aMV2.SetArguments(aLF);
        // no need to intersect this time
        aMV2.SetIntersect(false);
        aMV2.SetAvoidInternalShapes(true);
        aMV2.Perform(aPS.Next());
        bDone = !aMV2.HasErrors();
        if (!bDone)
        {
            return bDone;
        }
        //
        TopoDS_Shape aResult2 = aMV2.InternalShape();
        if (aResult2.ShapeType() == TopAbs_SOLID)
        {
            return GetSubShapes(aResult2, TopAbs_SHELL, ref theShells);
        }
        //
        TopExp_Explorer aExp = new(aResult2, TopAbs_FACE);
        bDone = aExp.More();
        if (!bDone)
        {
            return bDone;
        }
        //
        aLF.Clear();
        aDMFS.Clear();

        // the result is non-manifold - resolve it comparing normal
        // directions of the offset faces and original faces
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aExp.Current());
            if (!anOrigins.Find(aF, out NCollection_List<TopoDS_Shape> pLFOr))
            {
                // Standard_ASSERT_INVOKE: "Origins map does not contain the split face" (debug builds only).
                continue;
            }
            // Check orientation
            NCollection_List<TopoDS_Shape>.Iterator aItLOr = new(pLFOr);
            for (; aItLOr.More(); aItLOr.Next())
            {
                TopoDS_Face aFOr = TopoDS.Face(aItLOr.Value());
                if (BRepOffset_Tool.CheckPlanesNormals(aF, aFOr))
                {
                    aLF.Append(aF);
                    break;
                }
            }
        }
        //
        // make solid from most outer faces with correct normal direction
        BOPAlgo_MakerVolume aMV3 = new();
        aMV3.SetArguments(aLF);
        aMV3.SetIntersect(false);
        aMV3.SetAvoidInternalShapes(true);
        aMV3.Perform(aPS.Next());
        bDone = !aMV3.HasErrors();
        if (!bDone)
        {
            return bDone;
        }
        //
        TopoDS_Shape aResult3 = aMV3.InternalShape();
        return GetSubShapes(aResult3, TopAbs_SHELL, ref theShells);
    }

    //=================================================================================================

    /// <summary>
    /// The shapes generated from the caller's sub-shape <paramref name="theS"/> that are in the result (frozen, with the
    /// result).
    /// </summary>
    public IBentoList<TopoDS_Shape> Generated(in TopoDS_Shape theS) => PublishList(InternalGenerated(theS));

    /// <summary>
    /// The shapes modified from the caller's sub-shape <paramref name="theShape"/>: the images of a closing face (frozen,
    /// with the result).
    /// </summary>
    public IBentoList<TopoDS_Shape> Modified(in TopoDS_Shape theShape) => PublishList(InternalModified(theShape));

    private IBentoList<TopoDS_Shape> PublishList(NCollection_List<TopoDS_Shape> theList)
    {
        if (theList.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        myOffsetShape.TShape()?.FreezeGraph();
        foreach (TopoDS_Shape aS in theList)
        {
            aS.TShape()?.FreezeGraph();
        }
        return PackedList.Own([.. theList]);
    }

    /// <summary>
    /// OCCT's <c>Generated</c>, without freezing: the algorithm's own list (cleared by the next call), for the caller's
    /// sub-shape <paramref name="theS0"/>, holding the shapes that stand for the result's (decision 1005).
    /// </summary>
    internal NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape theS0)
    {
        TopoDS_Shape theS = InternalCopied(theS0);
        myGenerated.Clear();
        TopAbs_ShapeEnum aType = theS.ShapeType();
        // OCCT's switch falls through from VERTEX to EDGE to FACE.
        if (aType == TopAbs_VERTEX)
        {
            if (myAnalyse.HasAncestor(theS))
            {
                NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
                NCollection_List<TopoDS_Shape> aLA = myAnalyse.Ancestors(theS);
                NCollection_List<TopoDS_Shape>.Iterator itLA = new(aLA);
                for (; myGenerated.IsEmpty() && itLA.More(); itLA.Next())
                {
                    TopoDS_Shape aE = itLA.Value();
                    if (!myInitOffsetEdge.HasImage(aE))
                    {
                        continue;
                    }
                    NCollection_List<TopoDS_Shape> aLEIm = new();
                    myInitOffsetEdge.LastImage(aE, aLEIm);
                    NCollection_List<TopoDS_Shape>.Iterator itLEIm = new(aLEIm);
                    for (; myGenerated.IsEmpty() && itLEIm.More(); itLEIm.Next())
                    {
                        TopoDS_Iterator itV = new(itLEIm.Value());
                        for (; itV.More(); itV.Next())
                        {
                            if (!aMFence.Add(itV.Value()))
                            {
                                myGenerated.Append(itV.Value());
                                break;
                            }
                        }
                    }
                }
            }
        }
        if (aType == TopAbs_VERTEX || aType == TopAbs_EDGE)
        {
            if (myInitOffsetEdge.HasImage(theS))
            {
                myInitOffsetEdge.LastImage(theS, myGenerated);
            }
        }
        if (aType == TopAbs_VERTEX || aType == TopAbs_EDGE || aType == TopAbs_FACE)
        {
            TopoDS_Shape aS = theS;
            if (myFacePlanfaceMap.Find(aS, out TopoDS_Shape aPlanface))
            {
                aS = TopoDS.Face(aPlanface);
            }

            if (!myFaces.Contains(aS) && myInitOffsetFace.HasImage(aS))
            {
                myInitOffsetFace.LastImage(aS, myGenerated);

                if (!myFaces.IsEmpty())
                {
                    // Reverse generated shapes in case of small solids.
                    // Useful only for faces without influence on others.
                    NCollection_List<TopoDS_Shape>.Iterator it = new(myGenerated);
                    for (; it.More(); it.Next())
                    {
                        it.ChangeValue().Reverse();
                    }
                }
            }
        }
        else if (aType == TopAbs_SOLID)
        {
            if (theS.IsSame(myShape))
            {
                myGenerated.Append(myOffsetShape);
            }
        }

        if (myResMap.IsEmpty())
        {
            TopExp.MapShapes(myOffsetShape, myResMap);
        }

        // OCCT.NET: the working copy's shapes are replaced by what stands for them in the result (decision 1005).
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(myGenerated); it.More(); it.Next())
        {
            it.ChangeValue() = InternalFinal(it.Value());
        }

        for (NCollection_List<TopoDS_Shape>.Iterator it = new(myGenerated); it.More();)
        {
            if (myResMap.Contains(it.Value()))
            {
                it.Next();
            }
            else
            {
                myGenerated.Remove(it);
            }
        }

        return myGenerated;
    }

    //=================================================================================================

    /// <summary>
    /// OCCT's <c>Modified</c>, without freezing: the algorithm's own list (cleared by the next call), for the caller's
    /// sub-shape <paramref name="theShape0"/> (decision 1005).
    /// </summary>
    internal NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape theShape0)
    {
        TopoDS_Shape theShape = InternalCopied(theShape0);
        myGenerated.Clear();

        if (theShape.ShapeType() == TopAbs_FACE)
        {
            TopoDS_Shape aS = theShape;
            if (myFacePlanfaceMap.Find(aS, out TopoDS_Shape aPlanface))
            {
                aS = TopoDS.Face(aPlanface);
            }

            if (myFaces.Contains(aS) && myInitOffsetFace.HasImage(aS))
            {
                myInitOffsetFace.LastImage(aS, myGenerated);

                if (!myFaces.IsEmpty())
                {
                    // Reverse generated shapes in case of small solids.
                    // Useful only for faces without influence on others.
                    NCollection_List<TopoDS_Shape>.Iterator it = new(myGenerated);
                    for (; it.More(); it.Next())
                    {
                        it.ChangeValue().Reverse();
                    }
                }
            }
        }

        // OCCT.NET: the working copy's shapes are replaced by what stands for them in the result (decision 1005).
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(myGenerated); it.More(); it.Next())
        {
            it.ChangeValue() = InternalFinal(it.Value());
        }

        return myGenerated;
    }

    //=================================================================================================

    /// <summary>True if the caller's sub-shape <paramref name="theS"/> has been deleted.</summary>
    public bool IsDeleted(in TopoDS_Shape theS)
    {
        if (myResMap.IsEmpty())
        {
            TopExp.MapShapes(myOffsetShape, myResMap);
        }

        // OCCT.NET: a sub-shape of the caller's that OCCT keeps in its result, changed in place (a thickened shell's
        // edge given a pcurve), is in the port's result as its changed copy (decision 1005).
        if (myResMap.Contains(theS) || myResMap.Contains(InternalCopied(theS)))
        {
            return false;
        }

        return InternalGenerated(theS).IsEmpty() && InternalModified(theS).IsEmpty();
    }

    //=================================================================================================

    private static bool GetSubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theSSType, ref TopoDS_Shape theResult)
    {
        TopExp_Explorer aExp = new(theShape, theSSType);
        if (!aExp.More())
        {
            return false;
        }
        //
        TopoDS_Compound aResult = new();
        new BRep_Builder().MakeCompound(ref aResult);
        //
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aSS = aExp.Current();
            new BRep_Builder().Add(aResult, aSS);
        }
        theResult = aResult;
        return true;
    }

    //=================================================================================================

    /// <summary>Analyzes the progress steps of the whole operation.</summary>
    private void analyzeProgress(double theWhole, NCollection_Array1<double> theSteps)
    {
        theSteps.Init(0.0);

        // Set, approximately, the proportions for each operation.
        // It is not a problem that the sum of the set values will not
        // be equal to 100%, as the values will be normalized.
        // The main point is to make the proportions valid relatively each other.

        // Proportions will be different for different connection types
        bool isArc = (myJoin == GeomAbs_Arc);
        bool isPlanarIntCase = myInter && !isArc && myIsPlanar && !myThickening && myFaces.IsEmpty() && IsSolid(myShape);

        theSteps[PIOperation_CheckInputData] = 1.0;
        theSteps[PIOperation_Analyse] = 2.0;
        theSteps[PIOperation_BuildOffsetBy] = isPlanarIntCase ? 70.0 : (isArc ? 20.0 : 50.0);
        theSteps[PIOperation_Intersection] = isPlanarIntCase ? 0.0 : (isArc ? 50.0 : 20.0);
        if (myThickening)
        {
            theSteps[PIOperation_MakeMissingWalls] = 5.0;
        }
        theSteps[PIOperation_MakeShells] = isPlanarIntCase ? 25.0 : 5.0;
        theSteps[PIOperation_MakeSolid] = 5.0;
        if (myIsPerformSewing && myThickening)
        {
            theSteps[PIOperation_Sewing] = 10.0;
        }

        normalizeSteps(theWhole, theSteps);
    }

    //=======================================================================
    // function : IsPlanar
    // purpose  : Checks if all the faces of the shape are planes
    //=======================================================================
    private bool IsPlanar()
    {
        bool aIsNonPlanarFound = false;
        BRep_Builder aBB = new();

        TopExp_Explorer aExp = new(myShape, TopAbs_FACE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aExp.Current());
            BRepAdaptor_Surface aBAS = new(aF, false);
            if (aBAS.GetType() == GeomAbs_Plane)
            {
                continue;
            }

            if (myIsLinearizationAllowed)
            {
                // define the toleance
                double aTolForFace = BRep_Tool.Tolerance(aF);

                // try to linearize
                Geom_Surface aSurf = BRep_Tool.Surface(aF)!;
                GeomLib_IsPlanarSurface aPlanarityChecker = new(aSurf, Precision.Confusion());
                if (aPlanarityChecker.IsPlanar())
                {
                    gp_Pln aPln = aPlanarityChecker.Plan();
                    double um, vm;
                    aSurf.Bounds(out double u1, out double u2, out double v1, out double v2);
                    bool isInf1 = Precision.IsInfinite(u1), isInf2 = Precision.IsInfinite(u2);
                    if (!isInf1 && !isInf2)
                    {
                        um = (u1 + u2) / 2.0;
                    }
                    else if (isInf1 && !isInf2)
                    {
                        um = u2 - 1.0;
                    }
                    else if (!isInf1 && isInf2)
                    {
                        um = u1 + 1.0;
                    }
                    else // isInf1 && isInf2
                    {
                        um = 0.0;
                    }
                    isInf1 = Precision.IsInfinite(v1);
                    isInf2 = Precision.IsInfinite(v2);
                    if (!isInf1 && !isInf2)
                    {
                        vm = (v1 + v2) / 2.0;
                    }
                    else if (isInf1 && !isInf2)
                    {
                        vm = v2 - 1.0;
                    }
                    else if (!isInf1 && isInf2)
                    {
                        vm = v1 + 1.0;
                    }
                    else // isInf1 && isInf2
                    {
                        vm = 0.0;
                    }
                    aBAS.D1(um, vm, out gp_Pnt aP, out gp_Vec aD1, out gp_Vec aD2);
                    gp_Vec aNorm = aD1.Crossed(aD2);
                    gp_Dir aPlnNorm = aPln.Position().Direction();
                    if (aNorm.Dot(new gp_Vec(aPlnNorm)) < 0.0)
                    {
                        aPlnNorm.Reverse();
                        gp_Ax1 anAx = new(aPln.Position().Location(), aPlnNorm);
                        aPln.SetAxis(anAx);
                    }
                    Geom_Plane aPlane = new(aPln);
                    TopoDS_Face aPlanarFace = new();
                    aBB.MakeFace(ref aPlanarFace, aPlane, aTolForFace);
                    TopoDS_Face aFaceForward = aF;
                    aFaceForward.Orientation(TopAbs_FORWARD);
                    TopoDS_Iterator anItFace = new(aFaceForward);
                    for (; anItFace.More(); anItFace.Next())
                    {
                        TopoDS_Shape aWire = anItFace.Value();
                        aBB.Add(aPlanarFace, aWire);
                    }
                    RemoveSeamAndDegeneratedEdges(aPlanarFace, aFaceForward);
                    myFacePlanfaceMap.Bind(aF, aPlanarFace);
                    if (myFaces.Contains(aF))
                    {
                        myFaces.RemoveKey(aF);
                        myFaces.Add(aPlanarFace);
                    }
                }
                else
                {
                    aIsNonPlanarFound = true;
                }
            }
            else
            {
                aIsNonPlanarFound = true;
            }
        }

        return (!aIsNonPlanarFound);
    }

    //=======================================================================
    // function : RemoveSeamAndDegeneratedEdges
    // purpose  : Removes useless seam and degenerated edges from a face that becomes planar
    //=======================================================================
    private static void RemoveSeamAndDegeneratedEdges(in TopoDS_Face theFace, in TopoDS_Face theOldFace)
    {
        TopoDS_Face aFace = theFace;
        aFace.Orientation(TopAbs_FORWARD);

        bool aIsDegOrSeamFound = false;
        NCollection_Sequence<TopoDS_Shape> aEseq = new();
        TopExp_Explorer anExplo = new(aFace, TopAbs_EDGE);
        for (; anExplo.More(); anExplo.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExplo.Current());
            if (BRep_Tool.Degenerated(anEdge) || BRepTools.IsReallyClosed(anEdge, theOldFace))
            {
                aIsDegOrSeamFound = true;
            }
            else
            {
                aEseq.Append(anEdge);
            }
        }

        if (!aIsDegOrSeamFound)
        {
            return;
        }

        // Reconstruct wires
        BRep_Builder aBB = new();
        NCollection_List<TopoDS_Shape> aWlist = new();
        TopoDS_Iterator anItFace = new(aFace);
        for (; anItFace.More(); anItFace.Next())
        {
            aWlist.Append(anItFace.Value());
        }

        aFace.Free(true);
        NCollection_List<TopoDS_Shape>.Iterator anItl = new(aWlist);
        for (; anItl.More(); anItl.Next())
        {
            aBB.Remove(aFace, anItl.Value());
        }

        while (!aEseq.IsEmpty())
        {
            TopoDS_Wire aNewWire = new();
            aBB.MakeWire(ref aNewWire);
            TopoDS_Edge aCurEdge = TopoDS.Edge(aEseq[1]);
            aBB.Add(aNewWire, aCurEdge);
            aEseq.Remove(1);
            TopExp.Vertices(aCurEdge, out TopoDS_Vertex aFirstVertex, out TopoDS_Vertex aCurVertex, true); // with orientation
            while (!aCurVertex.IsSame(aFirstVertex))
            {
                TopoDS_Vertex aV1 = new(), aV2 = new();
                int ind;
                for (ind = 1; ind <= aEseq.Length(); ind++)
                {
                    aCurEdge = TopoDS.Edge(aEseq[ind]);
                    TopExp.Vertices(aCurEdge, out aV1, out aV2, true); // with orientation
                    if (aV1.IsSame(aCurVertex))
                    {
                        break;
                    }
                }
                if (ind > aEseq.Length())
                { // error occurred: wire is not closed
                    break;
                }

                aBB.Add(aNewWire, aCurEdge);
                aEseq.Remove(ind);
                aCurVertex = aV2;
            }

            aBB.Add(aFace, aNewWire);
        }
    }

    //=================================================================================================

    private static bool IsSolid(in TopoDS_Shape theS)
    {
        TopExp_Explorer aExp = new(theS, TopAbs_SOLID);
        return aExp.More();
    }

    //=======================================================================
    // function : AppendToList
    // purpose  : Add to a list only unique elements
    //=======================================================================
    private static void AppendToList(NCollection_List<TopoDS_Shape> theList, in TopoDS_Shape theShape)
    {
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(theList);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            if (aS.IsSame(theShape))
            {
                return;
            }
        }
        theList.Append(theShape);
    }

    //=================================================================================================

    /// <summary>
    /// The analysis of the working copy (OCCT's <c>GetAnalyse</c>): internal, since its tangent treatment holds the
    /// offset's working shapes (decision 1007).
    /// </summary>
    internal BRepOffset_Analyse GetAnalyse() => myAnalyse;
}
