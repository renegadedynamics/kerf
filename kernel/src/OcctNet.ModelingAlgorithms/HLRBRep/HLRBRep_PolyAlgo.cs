// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyAlgo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using static Kerf.Engine.Geometry.DerivativeStatus;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.HLRAlgo_PolyMask;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The polygonal hidden-line removal: hides the edges of meshed shapes (their faces' triangulations, their edges'
/// polygons on them, the polygons of free edges) for a projector, with the outlines found on the meshes.
/// </summary>
/// <remarks>
/// <para>
/// A working object (a transient), as OCCT's. Load shapes, set the projector, <see cref="Update"/>, then iterate on
/// the segments with <see cref="InitHide"/>/<see cref="MoreHide"/>/<see cref="NextHide"/> and <see cref="Hide"/>
/// (or read HLRBRep_PolyHLRToShape's compounds). Faces without a triangulation are skipped: the shapes must be meshed
/// first.
/// </para>
/// <para>
/// The algorithm only reads its shapes (their triangulations, polygons and geometry); it builds its own internal
/// triangulation, so nothing is copied at the entry (unit 7Q2). The shapes the segments come from
/// (<see cref="Hide"/>, <see cref="Show"/>) are the caller's own edges and faces.
/// </para>
/// </remarks>
public class HLRBRep_PolyAlgo : Transient
{
    // Node flags of the internal triangulation (OCCT's file-scope enum)
    private const int NMsk_Vert = 1;
    private const int NMsk_OutL = 2;
    private const int NMsk_Norm = 4;
    private const int NMsk_Fuck = 8;
    private const int NMsk_Edge = 16;
    private const int NMsk_Move = 32;

    private HLRAlgo_Projector myProj = new();
    private readonly double[,] TMat = new double[3, 3];
    private readonly double[] TLoc = new double[3];
    private readonly double[,] TTMa = new double[3, 3];
    private readonly double[] TTLo = new double[3];
    private readonly double[,] TIMa = new double[3, 3];
    private readonly double[] TILo = new double[3];
    private readonly NCollection_Sequence<TopoDS_Shape> myShapes = new();
    private readonly NCollection_IndexedMap<TopoDS_Shape> myEMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myFMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly HLRAlgo_PolyAlgo myAlgo;
    private bool myDebug;
    private double myTolSta;
    private double myTolEnd;
    private double myTolAngular;
    private Geom_Surface? myGSurf;
    private readonly BRepAdaptor_Surface myBSurf = new();
    private readonly BRepAdaptor_Curve myBCurv = new();
    private readonly BRepAdaptor_Curve2d myPC = new();

    public HLRBRep_PolyAlgo()
    {
        myDebug = false;
        myTolSta = 0.1;
        myTolEnd = 0.9;
        myTolAngular = 0.001;
        myAlgo = new HLRAlgo_PolyAlgo();
    }

    /// <summary>OCCT's copy: shares the other's HLRAlgo_PolyAlgo, copies its settings, projector and shapes.</summary>
    public HLRBRep_PolyAlgo(HLRBRep_PolyAlgo theOther)
    {
        myDebug = theOther.Debug();
        myTolAngular = theOther.TolAngular();
        myTolSta = theOther.TolCoef();
        myTolEnd = 1.0 - myTolSta;
        myAlgo = theOther.Algo();
        myProj = theOther.Projector();

        int aNbShapes = theOther.NbShapes();
        for (int i = 1; i <= aNbShapes; ++i)
        {
            Load(theOther.Shape(i));
        }
    }

    public HLRBRep_PolyAlgo(in TopoDS_Shape theShape)
    {
        myDebug = false;
        myTolSta = 0.1;
        myTolEnd = 0.9;
        myTolAngular = 0.001;
        myShapes.Append(theShape);
        myAlgo = new HLRAlgo_PolyAlgo();
    }

    public int NbShapes() => myShapes.Length();

    /// <summary>The shape of index theIndex.</summary>
    public TopoDS_Shape Shape(int theIndex) => myShapes.Value(theIndex);

    /// <summary>Removes the shape of index theIndex.</summary>
    public void Remove(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex == 0 || theIndex > myShapes.Length(), "HLRBRep_PolyAlgo::Remove : unknown Shape");
        myShapes.Remove(theIndex);
        myAlgo.Clear();
        myEMap.Clear();
        myFMap.Clear();
    }

    /// <summary>The index of the shape theShape, 0 if it isn't loaded.</summary>
    public int Index(in TopoDS_Shape theShape)
    {
        int i = 1;
        for (int k = 1; k <= myShapes.Length(); ++k, ++i)
        {
            if (myShapes.Value(k).IsEqual(theShape))
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>Adds the shape theShape (meshed: its faces' triangulations are used).</summary>
    public void Load(in TopoDS_Shape theShape) => myShapes.Append(theShape);

    public HLRAlgo_PolyAlgo Algo() => myAlgo;

    public HLRAlgo_Projector Projector() => myProj;

    public void Projector(in HLRAlgo_Projector theProj) => myProj = theProj;

    public double TolAngular() => myTolAngular;

    public void TolAngular(double theTol) => myTolAngular = theTol;

    public double TolCoef() => myTolSta;

    public void TolCoef(double theTol)
    {
        myTolSta = theTol;
        myTolEnd = 1.0 - theTol;
    }

    /// <summary>Builds the polygonal data of the shapes and prepares the hiding.</summary>
    public void Update()
    {
        myAlgo.Clear();
        myEMap.Clear();
        myFMap.Clear();

        TopoDS_Shape aShape = MakeShape();
        if (aShape.IsNull())
        {
            return;
        }

        bool isIsoledF = false, isIsoledE = false;
        int aNbShell = InitShape(aShape, ref isIsoledF, ref isIsoledE);
        if (aNbShell <= 0)
        {
            return;
        }

        TopExp.MapShapes(aShape, TopAbs_EDGE, myEMap);
        TopExp.MapShapes(aShape, TopAbs_FACE, myFMap);
        int aNbEdge = myEMap.Extent();
        int aNbFace = myFMap.Extent();
        NCollection_Array1<int> anES = new(0, aNbEdge); // index of the Shell
        NCollection_Array1<HLRAlgo_PolyData?> aPD = new(0, aNbFace);
        NCollection_Array1<HLRAlgo_PolyInternalData?> aPID = new(0, aNbFace);
        NCollection_Map<TopoDS_Shape> aShapeMap1 = new(TopTools_ShapeMapHasher.Instance),
                                      aShapeMap2 = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Array1<HLRAlgo_PolyShellData?> aShell = myAlgo.ChangePolyShell();
        int iShell = 0;
        for (TopExp_Explorer aShellIter = new(aShape, TopAbs_SHELL); aShellIter.More(); aShellIter.Next())
        {
            StoreShell(aShellIter.Current(), ref iShell, aShell, false, false, anES, aPD, aPID, aShapeMap1, aShapeMap2);
        }
        if (isIsoledF)
        {
            StoreShell(aShape, ref iShell, aShell, isIsoledF, false, anES, aPD, aPID, aShapeMap1, aShapeMap2);
        }
        if (isIsoledE)
        {
            StoreShell(aShape, ref iShell, aShell, false, isIsoledE, anES, aPD, aPID, aShapeMap1, aShapeMap2);
        }
        myAlgo.Update();
    }

    public void InitHide() => myAlgo.InitHide();

    public bool MoreHide() => myAlgo.MoreHide();

    public void NextHide() => myAlgo.NextHide();

    /// <summary>
    /// Hides the current segment: sets theStatus to its visible and hidden parts and returns its points; theShape is
    /// the edge it comes from (the face, for an internal outline).
    /// </summary>
    public ref HLRAlgo_BiPoint.PointsT Hide(HLRAlgo_EdgeStatus theStatus, out TopoDS_Shape theShape, out bool theReg1,
                                           out bool theRegn, out bool theOutl, out bool theIntl)
    {
        int anIndex = 0;
        ref HLRAlgo_BiPoint.PointsT aPoints =
            ref myAlgo.Hide(theStatus, out anIndex, out theReg1, out theRegn, out theOutl, out theIntl);
        theShape = theIntl ? myFMap.FindKey(anIndex) : myEMap.FindKey(anIndex);
        return ref aPoints;
    }

    public void InitShow() => myAlgo.InitShow();

    public bool MoreShow() => myAlgo.MoreShow();

    public void NextShow() => myAlgo.NextShow();

    /// <summary>The current segment without hiding: its points, and the edge (or face) it comes from.</summary>
    public ref HLRAlgo_BiPoint.PointsT Show(out TopoDS_Shape theShape, out bool theReg1, out bool theRegn,
                                           out bool theOutl, out bool theIntl)
    {
        int anIndex = 0;
        ref HLRAlgo_BiPoint.PointsT aPoints = ref myAlgo.Show(out anIndex, out theReg1, out theRegn, out theOutl, out theIntl);
        theShape = theIntl ? myFMap.FindKey(anIndex) : myEMap.FindKey(anIndex);
        return ref aPoints;
    }

    public bool Debug() => myDebug;

    public void Debug(bool theDebug) => myDebug = theDebug;

    private TopoDS_Shape MakeShape()
    {
        if (myShapes.IsEmpty())
        {
            return new TopoDS_Shape();
        }

        BRep_Builder aBuilder = new();
        TopoDS_Compound aComp = new();
        aBuilder.MakeCompound(ref aComp);
        for (int i = 1; i <= myShapes.Length(); i++)
        {
            aBuilder.Add(aComp, myShapes.Value(i));
        }
        return aComp;
    }

    private int InitShape(in TopoDS_Shape theShape, ref bool theIsoledF, ref bool theIsoledE)
    {
        NCollection_Map<TopoDS_Shape> aShapeMap0 = new(TopTools_ShapeMapHasher.Instance);
        int aNbShell = 0;
        theIsoledF = false;
        theIsoledE = false;
        TopExp_Explorer aFaceIter = new();
        Placement aLoc;

        for (TopExp_Explorer aShellIter = new(theShape, TopAbs_SHELL); aShellIter.More(); aShellIter.Next())
        {
            bool hasTrian = false;
            for (aFaceIter.Init(aShellIter.Current(), TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
                if (BRep_Tool.Triangulation(aFace, out aLoc) is not null)
                {
                    if (aShapeMap0.Add(aFace))
                    {
                        hasTrian = true;
                    }
                }
            }
            if (hasTrian)
            {
                ++aNbShell;
            }
        }

        for (aFaceIter.Init(theShape, TopAbs_FACE, TopAbs_SHELL); aFaceIter.More() && !theIsoledF; aFaceIter.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
            if (BRep_Tool.Triangulation(aFace, out aLoc) is not null)
            {
                if (aShapeMap0.Add(aFace))
                {
                    theIsoledF = true;
                }
            }
        }
        if (theIsoledF)
        {
            ++aNbShell;
        }

        for (TopExp_Explorer anEdgeIter = new(theShape, TopAbs_EDGE,
            TopAbs_FACE); anEdgeIter.More() && !theIsoledE; anEdgeIter.Next())
        {
            theIsoledE = true;
        }
        if (theIsoledE)
        {
            ++aNbShell;
        }
        if (aNbShell > 0)
        {
            myAlgo.Init(aNbShell);
        }
        return aNbShell;
    }


    private void StoreShell(in TopoDS_Shape theShape, ref int theIShell,
        NCollection_Array1<HLRAlgo_PolyShellData?> theShell, bool theIsoledF, bool theIsoledE,
        NCollection_Array1<int> theES, NCollection_Array1<HLRAlgo_PolyData?> thePD,
        NCollection_Array1<HLRAlgo_PolyInternalData?> thePID, NCollection_Map<TopoDS_Shape> theShapeMap1,
        NCollection_Map<TopoDS_Shape> theShapeMap2)
    {
        Placement aLoc;
        TopExp_Explorer aFaceExp = new(), anEdgeExp = new();
        int aNbFaceShell = 0;
        bool isClosed = false;
        Transform3D aProjTrsf = myProj.Transformation();
        {
            Xyz aTrsfVec = aProjTrsf.TranslationPart();
            TLoc[0] = aTrsfVec.X();
            TLoc[1] = aTrsfVec.Y();
            TLoc[2] = aTrsfVec.Z();

            Matrix3x3 aTrsfMat = aProjTrsf.VectorialPart();
            TMat[0, 0] = aTrsfMat.Value(1, 1);
            TMat[0, 1] = aTrsfMat.Value(1, 2);
            TMat[0, 2] = aTrsfMat.Value(1, 3);
            TMat[1, 0] = aTrsfMat.Value(2, 1);
            TMat[1, 1] = aTrsfMat.Value(2, 2);
            TMat[1, 2] = aTrsfMat.Value(2, 3);
            TMat[2, 0] = aTrsfMat.Value(3, 1);
            TMat[2, 1] = aTrsfMat.Value(3, 2);
            TMat[2, 2] = aTrsfMat.Value(3, 3);
        }
        {
            Transform3D aTrsfInv = myProj.InvertedTransformation();
            Xyz aTrsfInvVec = aTrsfInv.TranslationPart();
            TILo[0] = aTrsfInvVec.X();
            TILo[1] = aTrsfInvVec.Y();
            TILo[2] = aTrsfInvVec.Z();

            Matrix3x3 aTrsfInvMat = aTrsfInv.VectorialPart();
            TIMa[0, 0] = aTrsfInvMat.Value(1, 1);
            TIMa[0, 1] = aTrsfInvMat.Value(1, 2);
            TIMa[0, 2] = aTrsfInvMat.Value(1, 3);
            TIMa[1, 0] = aTrsfInvMat.Value(2, 1);
            TIMa[1, 1] = aTrsfInvMat.Value(2, 2);
            TIMa[1, 2] = aTrsfInvMat.Value(2, 3);
            TIMa[2, 0] = aTrsfInvMat.Value(3, 1);
            TIMa[2, 1] = aTrsfInvMat.Value(3, 2);
            TIMa[2, 2] = aTrsfInvMat.Value(3, 3);
        }
        if (!theIsoledE)
        {
            if (!theIsoledF)
            {
                isClosed = theShape.Closed();
                if (!isClosed)
                {
                    NCollection_IndexedMap<TopoDS_Shape> anEM = new(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapes(theShape, TopAbs_EDGE, anEM);
                    int aNbEdge = anEM.Extent();
                    NCollection_Array1<int> aFlagArray = new(1, Max(aNbEdge, 1));
                    aFlagArray.Init(0);
                    for (anEdgeExp.Init(theShape, TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
                    {
                        TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
                        int anEdgeIndex = anEM.FindIndex(anEdge);
                        TopAbs_Orientation anEdgeOrient = anEdge.Orientation();
                        if (!BRep_Tool.Degenerated(anEdge))
                        {
                            if (anEdgeOrient == TopAbs_FORWARD)
                            {
                                aFlagArray[anEdgeIndex] += 1;
                            }
                            else if (anEdgeOrient == TopAbs_REVERSED)
                            {
                                aFlagArray[anEdgeIndex] -= 1;
                            }
                        }
                    }
                    isClosed = true;

                    for (int anEdgeIter = 1; anEdgeIter <= aNbEdge && isClosed; ++anEdgeIter)
                    {
                        isClosed = (aFlagArray[anEdgeIter] == 0);
                    }
                }

                aFaceExp.Init(theShape, TopAbs_FACE);
            }
            else
            {
                aFaceExp.Init(theShape, TopAbs_FACE, TopAbs_SHELL);
            }

            for (; aFaceExp.More(); aFaceExp.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());
                if (BRep_Tool.Triangulation(aFace, out aLoc) is not null)
                {
                    if (theShapeMap1.Add(aFace))
                    {
                        ++aNbFaceShell;
                    }
                }
            }
        }
        if (aNbFaceShell > 0 || theIsoledE)
        {
            ++theIShell;
            theShell.SetValue(theIShell, new HLRAlgo_PolyShellData(aNbFaceShell));
        }

        if (aNbFaceShell > 0)
        {
            HLRAlgo_PolyShellData aPsd = theShell[theIShell]!;
            int iFace = 0;
            if (!theIsoledF)
            {
                aFaceExp.Init(theShape, TopAbs_FACE);
            }
            else
            {
                aFaceExp.Init(theShape, TopAbs_FACE, TopAbs_SHELL);
            }

            for (; aFaceExp.More(); aFaceExp.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());
                Triangulation? aTr = BRep_Tool.Triangulation(aFace, out aLoc);
                if (aTr is not null)
                {
                    if (theShapeMap2.Add(aFace))
                    {
                        iFace++;
                        int aFaceIndex = myFMap.FindIndex(aFace);
                        bool isReversed = aFace.Orientation() == TopAbs_REVERSED;
                        Transform3D aTT = aLoc.Transformation();
                        aTT.PreMultiply(aProjTrsf);
                        {
                            Xyz aTTrsfVec = aTT.TranslationPart();
                            TTLo[0] = aTTrsfVec.X();
                            TTLo[1] = aTTrsfVec.Y();
                            TTLo[2] = aTTrsfVec.Z();

                            Matrix3x3 aTTrsfMat = aTT.VectorialPart();
                            TTMa[0, 0] = aTTrsfMat.Value(1, 1);
                            TTMa[0, 1] = aTTrsfMat.Value(1, 2);
                            TTMa[0, 2] = aTTrsfMat.Value(1, 3);
                            TTMa[1, 0] = aTTrsfMat.Value(2, 1);
                            TTMa[1, 1] = aTTrsfMat.Value(2, 2);
                            TTMa[1, 2] = aTTrsfMat.Value(2, 3);
                            TTMa[2, 0] = aTTrsfMat.Value(3, 1);
                            TTMa[2, 1] = aTTrsfMat.Value(3, 2);
                            TTMa[2, 2] = aTTrsfMat.Value(3, 3);
                        }
                        int aNbNodes = aTr.NodeCount;
                        int aNbTris = aTr.TriangleCount;
                        thePD.SetValue(aFaceIndex, new HLRAlgo_PolyData());
                        aPsd.PolyData().SetValue(iFace, thePD.Value(aFaceIndex));
                        thePID.SetValue(aFaceIndex, new HLRAlgo_PolyInternalData(aNbNodes, aNbTris));
                        HLRAlgo_PolyInternalData aPid = thePID[aFaceIndex]!;
                        Geom_Surface? aSurf = BRep_Tool.Surface(aFace);
                        if (aSurf is not null)
                        {
                            if (aSurf is Geom_RectangularTrimmedSurface aRectTrimSurf)
                            {
                                aSurf = aRectTrimSurf.BasisSurface();
                            }
                            GeomAdaptor_Surface aSurfAdapt = new(aSurf);
                            aPid.Planar(aSurfAdapt.GetType() == Plane);
                        }
                        else
                        {
                            aPid.Planar(false);
                        }

                        NCollection_Array1<HLRAlgo_TriangleData> aTData = aPid.TData();
                        NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg = aPid.PISeg();
                        NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod = aPid.PINod();
                        for (int aTriIter = 1; aTriIter <= aNbTris; ++aTriIter)
                        {
                            Triangle1 aPolyTri = aTr.Triangle(aTriIter);
                            ref HLRAlgo_TriangleData aTriData = ref aTData[aTriIter];
                            aTriData.Flags = 0;
                            if (isReversed)
                            {
                                aPolyTri.Get(out aTriData.Node3, out aTriData.Node2, out aTriData.Node1);
                            }
                            else
                            {
                                aPolyTri.Get(out aTriData.Node1, out aTriData.Node2, out aTriData.Node3);
                            }
                        }

                        for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                        {
                            Point3D aPnt = aTr.Node(aNodeIter);
                            HLRAlgo_PolyInternalNode aPolyINode = aPINod[aNodeIter]!;
                            ref HLRAlgo_PolyInternalNode.NodeData aNod1RValues = ref aPolyINode.Data();
                            ref HLRAlgo_PolyInternalNode.NodeIndices aNodIndices = ref aPolyINode.Indices();
                            aNodIndices.NdSg = 0;
                            aNodIndices.Flag = 0;
                            aNod1RValues.Point = aPnt.XYZ();
                            TTMultiply(ref aNod1RValues.Point);
                        }
                        aPid.UpdateLinks(aTData, aPISeg, aPINod);
                        if (aTr.HasUVNodes)
                        {
                            bool hasSurf = BRep_Tool.IsGeometric(aFace);
                            myBSurf.Initialize(aFace, false);
                            for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                            {
                                HLRAlgo_PolyInternalNode aPolyINode =
                                    aPINod[aNodeIter]!;
                                ref HLRAlgo_PolyInternalNode.NodeIndices aNodIndices = ref aPolyINode.Indices();
                                ref HLRAlgo_PolyInternalNode.NodeData aNod1RValues = ref aPolyINode.Data();
                                if (aTr.HasUVNodes)
                                {
                                    aNod1RValues.UV = aTr.UVNode(aNodeIter).XY();
                                }
                                if (aTr.HasNormals)
                                {
                                    aNod1RValues.Normal = aTr.Normal(aNodeIter).XYZ();
                                }

                                if ((aTr.HasNormals || (hasSurf && aTr.HasUVNodes)) && Normal(aNodeIter,
                                    ref aNodIndices, ref aNod1RValues, aTData, aPISeg, aPINod, false))
                                {
                                    aNodIndices.Flag |= NMsk_Norm;
                                }
                                else
                                {
                                    aNodIndices.Flag &= ~NMsk_Norm;
                                    aNod1RValues.Scal = 0;
                                }
                            }
                        }

                        for (int aTriIter = 1; aTriIter <= aNbTris; ++aTriIter)
                        {
                            ref HLRAlgo_TriangleData aTriData = ref aTData[aTriIter];
                            HLRAlgo_PolyInternalNode aPN1 = aPINod[aTriData.Node1]!;
                            HLRAlgo_PolyInternalNode aPN2 = aPINod[aTriData.Node2]!;
                            HLRAlgo_PolyInternalNode aPN3 = aPINod[aTriData.Node3]!;
                            OrientTriangle(aTriIter, ref aTriData, ref aPN1.Indices(), ref aPN1.Data(),
                                ref aPN2.Indices(), ref aPN2.Data(), ref aPN3.Indices(), ref aPN3.Data());
                        }
                    }
                }
            }

            int aNbFaces = myFMap.Extent();
            NCollection_List<HLRAlgo_BiPoint> aList = aPsd.Edges();
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> EF = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(theShape, TopAbs_EDGE, TopAbs_FACE, EF);
            for (int aFaceIter = 1; aFaceIter <= aNbFaces; ++aFaceIter)
            {
                if (thePID.Value(aFaceIter) is not null)
                {
                    for (anEdgeExp.Init(myFMap[aFaceIter], TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
                    {
                        TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
                        if (theShapeMap1.Add(anEdge))
                        {
                            int anEdgeIndex = myEMap.FindIndex(anEdge);
                            theES.SetValue(anEdgeIndex, theIShell);
                            int anIndexE = EF.FindIndex(anEdge);
                            if (anIndexE > 0)
                            {
                                NCollection_List<TopoDS_Shape> LS = EF[anIndexE];
                                InitBiPointsWithConnexity(anEdgeIndex, ref anEdge, aList, thePID, LS, true);
                            }
                            else
                            {
                                NCollection_List<TopoDS_Shape> LS = new();
                                InitBiPointsWithConnexity(anEdgeIndex, ref anEdge, aList, thePID, LS, false);
                            }
                        }
                    }
                }
            }
            InsertOnOutLine(thePID);
            CheckFrBackTriangles(aList, thePID);
            UpdateOutLines(aList, thePID);
            UpdateEdgesBiPoints(aList, thePID, isClosed);
            UpdatePolyData(thePD, thePID, isClosed);

            for (int aFaceIter = 1; aFaceIter <= aNbFaces; ++aFaceIter)
            {
                thePID[aFaceIter] = null;
            }
        }
        else if (theIsoledE)
        {
            HLRAlgo_PolyShellData aPsd = theShell[theIShell]!;
            NCollection_List<HLRAlgo_BiPoint> aList = aPsd.Edges();
            for (anEdgeExp.Init(theShape, TopAbs_EDGE, TopAbs_FACE); anEdgeExp.More(); anEdgeExp.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
                if (theShapeMap1.Add(anEdge))
                {
                    int anEdgeIndex = myEMap.FindIndex(anEdge);
                    theES.SetValue(anEdgeIndex, theIShell);
                    NCollection_List<TopoDS_Shape> aLS = new();
                    InitBiPointsWithConnexity(anEdgeIndex, ref anEdge, aList, thePID, aLS, false);
                }
            }
        }
    }


    private bool Normal(int theNodeIndex, ref HLRAlgo_PolyInternalNode.NodeIndices theNodIndices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod1RValues, NCollection_Array1<HLRAlgo_TriangleData> theTriData,
        NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg, NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod,
        bool theToOrient)
    {
        if (theNod1RValues.Normal.SquareModulus() < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Vector3D aD1U, aD1V;
            Point3D aPnt;
            DerivativeStatus aStatus = D1IsNull;
            myBSurf.D1(theNod1RValues.UV.X(), theNod1RValues.UV.Y(), out aPnt, out aD1U, out aD1V);
            Direction3D aNorm = new();
            global::Kerf.Engine.Geometry.CSLib.Normal(aD1U, aD1V, global::Kerf.Engine.Foundation.Precision.Angular(), out aStatus, ref aNorm);
            if (aStatus != Done)
            {
                Vector3D aD2U, aD2V, aD2UV;
                bool isOK = false;
                NormalStatus aNromStatus;
                myBSurf.D2(theNod1RValues.UV.X(), theNod1RValues.UV.Y(), out aPnt, out aD1U, out aD1V, out aD2U, out aD2V, out aD2UV);
                global::Kerf.Engine.Geometry.CSLib.Normal(aD1U, aD1V, aD2U, aD2V, aD2UV, global::Kerf.Engine.Foundation.Precision.Angular(), out isOK, out aNromStatus, ref aNorm);
                if (!isOK)
                {
                    return false;
                }
            }
            theNod1RValues.Normal = aNorm.XYZ();
        }

        TMultiply(ref theNod1RValues.Normal, myProj.Perspective());

        Xyz anAverNorm = new();
        if (AverageNormal(theNodeIndex, ref theNodIndices, theTriData, thePISeg, thePINod, ref anAverNorm))
        {
            if (theNod1RValues.Normal * anAverNorm < 0)
            {
                theNod1RValues.Normal.Reverse();
            }

            Xyz anEyeDir = new(0.0, 0.0, -1.0);
            if (myProj.Perspective())
            {
                anEyeDir.SetCoord(theNod1RValues.Point.X(), theNod1RValues.Point.Y(),
                    theNod1RValues.Point.Z() - myProj.Focus());
                double anEyeMod = anEyeDir.Modulus();
                if (anEyeMod > 0.0)
                {
                    anEyeDir /= anEyeMod;
                }
            }
            theNod1RValues.Scal = (theNod1RValues.Normal * anEyeDir);
        }
        else
        {
            theNod1RValues.Scal = 0;
            theNod1RValues.Normal = new Xyz(1.0, 0.0, 0.0);
        }
        if (theNod1RValues.Scal > 0)
        {
            if (theNod1RValues.Scal < myTolAngular)
            {
                theNod1RValues.Scal = 0;
                theNodIndices.Flag |= NMsk_OutL;
            }
        }
        else
        {
            if (-theNod1RValues.Scal < myTolAngular)
            {
                theNod1RValues.Scal = 0;
                theNodIndices.Flag |= NMsk_OutL;
            }
        }

        if (theToOrient)
        {
            UpdateAroundNode(theNodeIndex, ref theNodIndices, theTriData, thePISeg, thePINod);
        }
        return true;
    }


    private bool AverageNormal(int iNode, ref HLRAlgo_PolyInternalNode.NodeIndices theNodeIndices,
        NCollection_Array1<HLRAlgo_TriangleData> theTData, NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg,
        NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod, ref double theX, ref double theY, ref double theZ)
    {
        bool isOK = false;
        int jNode = 0, kNode = 0, iiii;
        theX = 0;
        theY = 0;
        theZ = 0;
        iiii = theNodeIndices.NdSg;
        while (iiii != 0 && !isOK)
        {
            ref HLRAlgo_PolyInternalSegment aSegIndices = ref thePISeg[iiii];
            int iTri1 = aSegIndices.Conex1;
            int iTri2 = aSegIndices.Conex2;
            if (iTri1 != 0)
            {
                AddNormalOnTriangle(iTri1, iNode, ref jNode, theTData, thePINod, ref theX, ref theY, ref theZ, ref isOK);
            }
            if (iTri2 != 0)
            {
                AddNormalOnTriangle(iTri2, iNode, ref jNode, theTData, thePINod, ref theX, ref theY, ref theZ, ref isOK);
            }
            if (aSegIndices.LstSg1 == iNode)
            {
                iiii = aSegIndices.NxtSg1;
            }
            else
            {
                iiii = aSegIndices.NxtSg2;
            }
        }

        if (jNode != 0)
        {
            iiii = theNodeIndices.NdSg;

            while (iiii != 0 && !isOK)
            {
                ref HLRAlgo_PolyInternalSegment aSegIndices = ref thePISeg[iiii];
                int iTri1 = aSegIndices.Conex1;
                int iTri2 = aSegIndices.Conex2;
                if (iTri1 != 0)
                {
                    AddNormalOnTriangle(iTri1, jNode, ref kNode, theTData, thePINod, ref theX, ref theY, ref theZ, ref isOK);
                }
                if (iTri2 != 0)
                {
                    AddNormalOnTriangle(iTri2, jNode, ref kNode, theTData, thePINod, ref theX, ref theY, ref theZ, ref isOK);
                }
                if (aSegIndices.LstSg1 == jNode)
                {
                    iiii = aSegIndices.NxtSg1;
                }
                else
                {
                    iiii = aSegIndices.NxtSg2;
                }
            }
        }
        double aD = Math.Sqrt(theX * theX + theY * theY + theZ * theZ);
        if (isOK && aD < 1.0e-10)
        {
            isOK = false;
        }
        return isOK;
    }


    private void AddNormalOnTriangle(int theITri, int theINode, ref int theJNode,
        NCollection_Array1<HLRAlgo_TriangleData> theTData, NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod,
        ref double theX, ref double theY, ref double theZ, ref bool theIsOK)
    {
        ref HLRAlgo_TriangleData aTriangle = ref theTData[theITri];
        ref HLRAlgo_PolyInternalNode.NodeData aNod1RValues = ref thePINod.Value(aTriangle.Node1)!.Data();
        ref HLRAlgo_PolyInternalNode.NodeData aNod2RValues = ref thePINod.Value(aTriangle.Node2)!.Data();
        ref HLRAlgo_PolyInternalNode.NodeData aNod3RValues = ref thePINod.Value(aTriangle.Node3)!.Data();
        Xyz aD1 = aNod2RValues.Point - aNod1RValues.Point;
        double aD1Norm = aD1.Modulus();
        if (aD1Norm < 1.0e-10)
        {
            if (aTriangle.Node1 == theINode)
            {
                theJNode = aTriangle.Node2;
            }
            else if (aTriangle.Node2 == theINode)
            {
                theJNode = aTriangle.Node1;
            }
        }
        else
        {
            Xyz aD2 = aNod3RValues.Point - aNod2RValues.Point;
            double aD2Norm = aD2.Modulus();
            if (aD2Norm < 1.0e-10)
            {
                if (aTriangle.Node2 == theINode)
                {
                    theJNode = aTriangle.Node3;
                }
                else if (aTriangle.Node3 == theINode)
                {
                    theJNode = aTriangle.Node2;
                }
            }
            else
            {
                Xyz aD3 = aNod1RValues.Point - aNod3RValues.Point;
                double aD3Norm = aD3.Modulus();
                if (aD3Norm < 1.0e-10)
                {
                    if (aTriangle.Node3 == theINode)
                    {
                        theJNode = aTriangle.Node1;
                    }
                    else if (aTriangle.Node1 == theINode)
                    {
                        theJNode = aTriangle.Node3;
                    }
                }
                else
                {
                    Xyz aDN = (1 / (aD1Norm * aD2Norm)) * (aD1 ^ aD2);
                    double aDNNorm = aDN.Modulus();
                    if (aDNNorm > 1.0e-10)
                    {
                        theIsOK = true;
                        theX += aDN.X();
                        theY += aDN.Y();
                        theZ += aDN.Z();
                    }
                }
            }
        }
    }


    private void InitBiPointsWithConnexity(int theIEdge, ref TopoDS_Edge theEdge,
        NCollection_List<HLRAlgo_BiPoint> theList, NCollection_Array1<HLRAlgo_PolyInternalData?> thePID,
        NCollection_List<TopoDS_Shape> theLS, bool theIsConnex)
    {
        double X1, Y1, Z1, X2, Y2, Z2;
        double XTI1, YTI1, ZTI1, XTI2, YTI2, ZTI2;
        double U1 = 0.0, U2 = 0.0;
        PolygonOnTriangulation?[] aHPol = new PolygonOnTriangulation?[2];
        Placement aLoc;
        myBCurv.Initialize(theEdge);
        if (theIsConnex)
        {
            int aNbConnex = theLS.Extent();
            if (aNbConnex == 1)
            {
                NCollection_List<TopoDS_Shape>.Iterator itn = new(theLS);
                TopoDS_Face aF1 = TopoDS.Face(itn.Value());
                int i1 = myFMap.FindIndex(aF1);
                Triangulation? aTr1 = BRep_Tool.Triangulation(aF1, out aLoc);
                aHPol[0] = BRep_Tool.PolygonOnTriangulation(theEdge, aTr1, aLoc);
                HLRAlgo_PolyInternalData pid1 = thePID.Value(i1)!;
                if (aHPol[0] is not null)
                {
                    myPC.Initialize(theEdge, aF1);
                    NCollection_HArray1<double> par = aHPol[0]!.Parameters()!;
                    NCollection_Array1<int> aPol1 = aHPol[0]!.Nodes();
                    int aNbPol = aPol1.Upper();
                    NCollection_Array1<HLRAlgo_TriangleData> aTData1 = pid1.TData();
                    NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg1 = pid1.PISeg();
                    NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod1 = pid1.PINod();
                    HLRAlgo_PolyInternalNode pi1p1 = aPINod1[aPol1[1]]!;
                    ref HLRAlgo_PolyInternalNode.NodeIndices aNode11Indices = ref pi1p1.Indices();
                    ref HLRAlgo_PolyInternalNode.NodeData aNod11RValues = ref pi1p1.Data();
                    HLRAlgo_PolyInternalNode pi1p2 = aPINod1[aPol1[aNbPol]]!;
                    ref HLRAlgo_PolyInternalNode.NodeIndices aNode12Indices = ref pi1p2.Indices();
                    ref HLRAlgo_PolyInternalNode.NodeData aNod12RValues = ref pi1p2.Data();
                    aNode11Indices.Flag |= NMsk_Vert;
                    aNode12Indices.Flag |= NMsk_Vert;

                    for (int iPol = 1; iPol <= aNbPol; iPol++)
                    {
                        HLRAlgo_PolyInternalNode pi1pA = aPINod1[aPol1[iPol]]!;
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNodeIndices1A = ref pi1pA.Indices();
                        ref HLRAlgo_PolyInternalNode.NodeData Nod1ARValues = ref pi1pA.Data();
                        if (aNodeIndices1A.Edg1 == 0 || aNodeIndices1A.Edg1 == theIEdge)
                        {
                            aNodeIndices1A.Edg1 = theIEdge;
                            Nod1ARValues.PCu1 = par.Value(iPol);
                        }
                        else
                        {
                            aNodeIndices1A.Edg2 = theIEdge;
                            Nod1ARValues.PCu2 = par.Value(iPol);
                        }
                    }

                    int i1p2 = aPol1[1];
                    aNode12Indices = ref aNode11Indices;
                    aNod12RValues = ref aNod11RValues;
                    XTI2 = X2 = aNod12RValues.Point.X();
                    YTI2 = Y2 = aNod12RValues.Point.Y();
                    ZTI2 = Z2 = aNod12RValues.Point.Z();
                    if (aNode12Indices.Edg1 == theIEdge)
                    {
                        U2 = aNod12RValues.PCu1;
                    }
                    else if (aNode12Indices.Edg2 == theIEdge)
                    {
                        U2 = aNod12RValues.PCu2;
                    }
                    aNode12Indices.Flag |= NMsk_Edge;
                    TIMultiply(ref XTI2, ref YTI2, ref ZTI2);
                    if (aPol1[1] == aPol1[aNbPol] && myPC.IsPeriodic())
                    {
                        U2 = U2 - myPC.Period();
                    }

                    if (aNbPol == 2 && BRep_Tool.Degenerated(theEdge))
                    {
                        CheckDegeneratedSegment(ref aNode11Indices, ref aNod11RValues, ref aNode12Indices, ref aNod12RValues);
                        UpdateAroundNode(aPol1[1], ref aNode11Indices, aTData1, aPISeg1, aPINod1);
                        UpdateAroundNode(aPol1[aNbPol], ref aNode12Indices, aTData1, aPISeg1, aPINod1);
                    }
                    else
                    {
                        for (int iPol = 2; iPol <= aNbPol; iPol++)
                        {
                            int i1p1 = i1p2;
                            aNode11Indices = ref aNode12Indices;
                            aNod11RValues = ref aNod12RValues;
                            i1p2 = aPol1[iPol];
                            HLRAlgo_PolyInternalNode pi1p2iPol =
                                aPINod1[aPol1[iPol]]!;
                            aNode12Indices = ref pi1p2iPol.Indices();
                            aNod12RValues = ref pi1p2iPol.Data();
                            X1 = X2;
                            Y1 = Y2;
                            Z1 = Z2;
                            XTI1 = XTI2;
                            YTI1 = YTI2;
                            ZTI1 = ZTI2;
                            U1 = U2;
                            XTI2 = X2 = aNod12RValues.Point.X();
                            YTI2 = Y2 = aNod12RValues.Point.Y();
                            ZTI2 = Z2 = aNod12RValues.Point.Z();
                            if (aNode12Indices.Edg1 == theIEdge)
                            {
                                U2 = aNod12RValues.PCu1;
                            }
                            else if (aNode12Indices.Edg2 == theIEdge)
                            {
                                U2 = aNod12RValues.PCu2;
                            }
                            aNode12Indices.Flag |= NMsk_Edge;
                            TIMultiply(ref XTI2, ref YTI2, ref ZTI2);
                            Interpolation(theList, ref X1, ref Y1, ref Z1, ref X2, ref Y2, ref Z2, ref XTI1, ref YTI1,
                                ref ZTI1, ref XTI2, ref YTI2, ref ZTI2, theIEdge, ref U1, ref U2, ref aNode11Indices,
                                ref aNod11RValues, ref aNode12Indices, ref aNod12RValues, i1p1, i1p2, i1, pid1,
                                ref aTData1, ref aPISeg1, ref aPINod1);
                        }
                    }
                }
            }
            else if (aNbConnex == 2)
            {
                NCollection_List<TopoDS_Shape>.Iterator itn = new(theLS);
                TopoDS_Face aF1 = TopoDS.Face(itn.Value());
                int i1 = myFMap.FindIndex(aF1);
                Triangulation? aTr1 = BRep_Tool.Triangulation(aF1, out aLoc);
                aHPol[0] = BRep_Tool.PolygonOnTriangulation(theEdge, aTr1, aLoc);
                itn.Next();
                TopoDS_Face aF2 = TopoDS.Face(itn.Value());
                int i2 = myFMap.FindIndex(aF2);
                if (i1 == i2)
                {
                    theEdge.Reverse();
                }
                Triangulation? aTr2 = BRep_Tool.Triangulation(aF2, out aLoc);
                aHPol[1] = BRep_Tool.PolygonOnTriangulation(theEdge, aTr2, aLoc);
                Continuity rg = BRep_Tool.Continuity(theEdge, aF1, aF2);
                HLRAlgo_PolyInternalData pid1 = thePID.Value(i1)!;
                HLRAlgo_PolyInternalData pid2 = thePID.Value(i2)!;
                if (aHPol[0] is not null && aHPol[1] is not null)
                {
                    myPC.Initialize(theEdge, aF1);
                    NCollection_Array1<int> aPol1 = aHPol[0]!.Nodes();
                    NCollection_Array1<int> aPol2 = aHPol[1]!.Nodes();
                    NCollection_HArray1<double> par = aHPol[0]!.Parameters()!;
                    int aNbPol1 = aPol1.Upper();
                    NCollection_Array1<HLRAlgo_TriangleData> aTData1 = pid1.TData();
                    NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg1 = pid1.PISeg();
                    NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod1 = pid1.PINod();
                    NCollection_Array1<HLRAlgo_TriangleData> aTData2 = pid2.TData();
                    NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg2 = pid2.PISeg();
                    NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod2 = pid2.PINod();
                    HLRAlgo_PolyInternalNode pi1p1 = aPINod1[aPol1[1]]!;
                    ref HLRAlgo_PolyInternalNode.NodeIndices aNode11Indices = ref pi1p1.Indices();
                    ref HLRAlgo_PolyInternalNode.NodeData aNod11RValues = ref pi1p1.Data();
                    HLRAlgo_PolyInternalNode pi1p2nbPol1 = aPINod1[aPol1[aNbPol1]]!;
                    ref HLRAlgo_PolyInternalNode.NodeIndices aNode12Indices = ref pi1p2nbPol1.Indices();
                    ref HLRAlgo_PolyInternalNode.NodeData aNod12RValues = ref pi1p2nbPol1.Data();
                    HLRAlgo_PolyInternalNode pi2p1 = aPINod2[aPol2[1]]!;
                    ref HLRAlgo_PolyInternalNode.NodeIndices aNod21Indices = ref pi2p1.Indices();
                    ref HLRAlgo_PolyInternalNode.NodeData aNod21RValues = ref pi2p1.Data();
                    HLRAlgo_PolyInternalNode pi2p2 = aPINod2[aPol2[aNbPol1]]!;
                    ref HLRAlgo_PolyInternalNode.NodeIndices aNod22Indices = ref pi2p2.Indices();
                    ref HLRAlgo_PolyInternalNode.NodeData aNod22RValues = ref pi2p2.Data();
                    aNode11Indices.Flag |= NMsk_Vert;
                    aNode12Indices.Flag |= NMsk_Vert;
                    aNod21Indices.Flag |= NMsk_Vert;
                    aNod22Indices.Flag |= NMsk_Vert;

                    for (int iPol = 1; iPol <= aNbPol1; iPol++)
                    {
                        HLRAlgo_PolyInternalNode pi1pA = aPINod1[aPol1[iPol]]!;
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNod1AIndices = ref pi1pA.Indices();
                        ref HLRAlgo_PolyInternalNode.NodeData aNod1ARValues = ref pi1pA.Data();
                        HLRAlgo_PolyInternalNode pi2pA = aPINod2[aPol2[iPol]]!;
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNod2AIndices = ref pi2pA.Indices();
                        ref HLRAlgo_PolyInternalNode.NodeData aNod2ARValues = ref pi2pA.Data();
                        double PCu = par.Value(iPol);
                        if (aNod1AIndices.Edg1 == 0 || aNod1AIndices.Edg1 == theIEdge)
                        {
                            aNod1AIndices.Edg1 = theIEdge;
                            aNod1ARValues.PCu1 = PCu;
                        }
                        else
                        {
                            aNod1AIndices.Edg2 = theIEdge;
                            aNod1ARValues.PCu2 = PCu;
                        }
                        if (aNod2AIndices.Edg1 == 0 || aNod2AIndices.Edg1 == theIEdge)
                        {
                            aNod2AIndices.Edg1 = theIEdge;
                            aNod2ARValues.PCu1 = PCu;
                        }
                        else
                        {
                            aNod2AIndices.Edg2 = theIEdge;
                            aNod2ARValues.PCu2 = PCu;
                        }
                    }

                    int i1p2 = aPol1[1];
                    aNode12Indices = ref aNode11Indices;
                    aNod12RValues = ref aNod11RValues;
                    int i2p2 = aPol2[1];
                    aNod22Indices = ref aNod21Indices;
                    aNod22RValues = ref aNod21RValues;
                    XTI2 = X2 = aNod12RValues.Point.X();
                    YTI2 = Y2 = aNod12RValues.Point.Y();
                    ZTI2 = Z2 = aNod12RValues.Point.Z();
                    if (aNode12Indices.Edg1 == theIEdge)
                    {
                        U2 = aNod12RValues.PCu1;
                    }
                    else if (aNode12Indices.Edg2 == theIEdge)
                    {
                        U2 = aNod12RValues.PCu2;
                    }
                    aNode12Indices.Flag |= NMsk_Edge;
                    aNod22Indices.Flag |= NMsk_Edge;
                    TIMultiply(ref XTI2, ref YTI2, ref ZTI2);
                    if (aPol1[1] == aPol1[aNbPol1] && myPC.IsPeriodic())
                    {
                        U2 = U2 - myPC.Period();
                    }

                    if (aNbPol1 == 2 && BRep_Tool.Degenerated(theEdge))
                    {
                        CheckDegeneratedSegment(ref aNode11Indices, ref aNod11RValues, ref aNode12Indices, ref aNod12RValues);
                        CheckDegeneratedSegment(ref aNod21Indices, ref aNod21RValues, ref aNod22Indices, ref aNod22RValues);
                        UpdateAroundNode(aPol1[1], ref aNode11Indices, aTData1, aPISeg1, aPINod1);
                        UpdateAroundNode(aPol1[aNbPol1], ref aNode12Indices, aTData1, aPISeg1, aPINod1);
                        UpdateAroundNode(aPol2[1], ref aNod21Indices, aTData2, aPISeg2, aPINod2);
                        UpdateAroundNode(aPol2[aNbPol1], ref aNod22Indices, aTData2, aPISeg2, aPINod2);
                    }
                    else
                    {
                        for (int iPol = 2; iPol <= aNbPol1; iPol++)
                        {
                            int i1p1 = i1p2;
                            aNode11Indices = ref aNode12Indices;
                            aNod11RValues = ref aNod12RValues;
                            int i2p1 = i2p2;
                            aNod21Indices = ref aNod22Indices;
                            aNod21RValues = ref aNod22RValues;
                            i1p2 = aPol1[iPol];
                            HLRAlgo_PolyInternalNode pi1p2iPol = aPINod1[aPol1[iPol]]!;
                            aNode12Indices = ref pi1p2iPol.Indices();
                            aNod12RValues = ref pi1p2iPol.Data();
                            i2p2 = aPol2[iPol];
                            HLRAlgo_PolyInternalNode pi2p2iPol = aPINod2[aPol2[iPol]]!;
                            aNod22Indices = ref pi2p2iPol.Indices();
                            aNod22RValues = ref pi2p2iPol.Data();
                            X1 = X2;
                            Y1 = Y2;
                            Z1 = Z2;
                            XTI1 = XTI2;
                            YTI1 = YTI2;
                            ZTI1 = ZTI2;
                            U1 = U2;
                            XTI2 = X2 = aNod12RValues.Point.X();
                            YTI2 = Y2 = aNod12RValues.Point.Y();
                            ZTI2 = Z2 = aNod12RValues.Point.Z();
                            if (aNode12Indices.Edg1 == theIEdge)
                            {
                                U2 = aNod12RValues.PCu1;
                            }
                            else if (aNode12Indices.Edg2 == theIEdge)
                            {
                                U2 = aNod12RValues.PCu2;
                            }
                            aNode12Indices.Flag |= NMsk_Edge;
                            aNode12Indices.Flag |= NMsk_Edge;
                            TIMultiply(ref XTI2, ref YTI2, ref ZTI2);
                            Interpolation(theList, ref X1, ref Y1, ref Z1, ref X2, ref Y2, ref Z2, ref XTI1, ref YTI1,
                                ref ZTI1, ref XTI2, ref YTI2, ref ZTI2, theIEdge, ref U1, ref U2, rg, ref aNode11Indices,
                                ref aNod11RValues, ref aNode12Indices, ref aNod12RValues, i1p1, i1p2, i1, pid1,
                                ref aTData1, ref aPISeg1, ref aPINod1, ref aNod21Indices, ref aNod21RValues,
                                ref aNod22Indices, ref aNod22RValues, i2p1, i2p2, i2, pid2, ref aTData2, ref aPISeg2,
                                ref aPINod2);
                        }
                    }
                }
            }
        }
        else
        { // no connexity
            Polygon3D? aPolyg = BRep_Tool.Polygon3D(theEdge, out aLoc);
            if (aPolyg is not null)
            {
                NCollection_Array1<Point3D> aPol = aPolyg.Nodes();
                Transform3D aTT = aLoc.Transformation();
                Transform3D aProjTrsf = myProj.Transformation();
                aTT.PreMultiply(aProjTrsf);
                {
                    Xyz aTTrsfVec = aTT.TranslationPart();
                    TTLo[0] = aTTrsfVec.X();
                    TTLo[1] = aTTrsfVec.Y();
                    TTLo[2] = aTTrsfVec.Z();
                    Matrix3x3 aTTrsfMat = aTT.VectorialPart();
                    TTMa[0, 0] = aTTrsfMat.Value(1, 1);
                    TTMa[0, 1] = aTTrsfMat.Value(1, 2);
                    TTMa[0, 2] = aTTrsfMat.Value(1, 3);
                    TTMa[1, 0] = aTTrsfMat.Value(2, 1);
                    TTMa[1, 1] = aTTrsfMat.Value(2, 2);
                    TTMa[1, 2] = aTTrsfMat.Value(2, 3);
                    TTMa[2, 0] = aTTrsfMat.Value(3, 1);
                    TTMa[2, 1] = aTTrsfMat.Value(3, 2);
                    TTMa[2, 2] = aTTrsfMat.Value(3, 3);
                }
                int aNbPol1 = aPol.Upper();
                Xyz aP1 = aPol[1].XYZ();
                X2 = aP1.X();
                Y2 = aP1.Y();
                Z2 = aP1.Z();
                TTMultiply(ref X2, ref Y2, ref Z2);
                XTI2 = X2;
                YTI2 = Y2;
                ZTI2 = Z2;
                TIMultiply(ref XTI2, ref YTI2, ref ZTI2);

                for (int jPol = 2; jPol <= aNbPol1; jPol++)
                {
                    X1 = X2;
                    Y1 = Y2;
                    Z1 = Z2;
                    XTI1 = XTI2;
                    YTI1 = YTI2;
                    ZTI1 = ZTI2;
                    Xyz aP2 = aPol[jPol].XYZ();
                    X2 = aP2.X();
                    Y2 = aP2.Y();
                    Z2 = aP2.Z();
                    TTMultiply(ref X2, ref Y2, ref Z2);
                    XTI2 = X2;
                    YTI2 = Y2;
                    ZTI2 = Z2;
                    TIMultiply(ref XTI2, ref YTI2, ref ZTI2);
                    theList.Prepend(new HLRAlgo_BiPoint(XTI1, YTI1, ZTI1, XTI2, YTI2, ZTI2, X1, Y1, Z1, X2, Y2, Z2,
                        theIEdge, 0));
                }
            }
        }
    }


    private void Interpolation(NCollection_List<HLRAlgo_BiPoint> theList, ref double theX1, ref double theY1,
        ref double theZ1, ref double theX2, ref double theY2, ref double theZ2, ref double theXTI1, ref double theYTI1,
        ref double theZTI1, ref double theXTI2, ref double theYTI2, ref double theZTI2, int theIEdge, ref double theU1,
        ref double theU2, ref HLRAlgo_PolyInternalNode.NodeIndices theNod11Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod11RValues, ref HLRAlgo_PolyInternalNode.NodeIndices theNod12Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod12RValues, int theI1p1, int theI1p2, int theI1,
        HLRAlgo_PolyInternalData thePid1, ref NCollection_Array1<HLRAlgo_TriangleData> theTData1,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg1,
        ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod1)
    {
        bool mP3P1 = false;
        double X3 = 0.0, Y3 = 0.0, Z3 = 0.0, XTI3 = 0.0, YTI3 = 0.0, ZTI3 = 0.0, coef3 = 0.0, U3 = 0.0;
        //  gp_Pnt P3, PT3;
        bool insP3 = Interpolation(theU1, theU2, ref theNod11RValues, ref theNod12RValues, ref X3, ref Y3, ref Z3,
            ref XTI3, ref YTI3, ref ZTI3, ref coef3, ref U3, ref mP3P1);
        MoveOrInsertPoint(theList, ref theX1, ref theY1, ref theZ1, ref theX2, ref theY2, ref theZ2, ref theXTI1,
            ref theYTI1, ref theZTI1, ref theXTI2, ref theYTI2, ref theZTI2, theIEdge, ref theU1, ref theU2,
            ref theNod11Indices, ref theNod11RValues, ref theNod12Indices, ref theNod12RValues, theI1p1, theI1p2, theI1,
            thePid1, ref theTData1, ref thePISeg1, ref thePINod1, X3, Y3, Z3, XTI3, YTI3, ZTI3, coef3, U3, insP3, mP3P1,
            0);
    }


    private void Interpolation(NCollection_List<HLRAlgo_BiPoint> theList, ref double theX1, ref double theY1,
        ref double theZ1, ref double theX2, ref double theY2, ref double theZ2, ref double theXTI1, ref double theYTI1,
        ref double theZTI1, ref double theXTI2, ref double theYTI2, ref double theZTI2, int theIEdge, ref double theU1,
        ref double theU2, Continuity theRg, ref HLRAlgo_PolyInternalNode.NodeIndices theNod11Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod11RValues, ref HLRAlgo_PolyInternalNode.NodeIndices theNod12Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod12RValues, int theI1p1, int theI1p2, int theI1,
        HLRAlgo_PolyInternalData thePid1, ref NCollection_Array1<HLRAlgo_TriangleData> theTData1,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg1,
        ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod1,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod21Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod21RValues,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod22Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod22RValues,
        int theI2p1, int theI2p2, int theI2, HLRAlgo_PolyInternalData thePid2,
        ref NCollection_Array1<HLRAlgo_TriangleData> theTData2,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg2,
        ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod2)
    {
        bool mP3P1 = false, mP4P1 = false;
        double X3 = 0.0, Y3 = 0.0, Z3 = 0.0, XTI3 = 0.0, YTI3 = 0.0, ZTI3 = 0.0, coef3 = 0.0, U3 = 0.0;
        double X4 = 0.0, Y4 = 0.0, Z4 = 0.0, XTI4 = 0.0, YTI4 = 0.0, ZTI4 = 0.0, coef4 = 0.0, U4 = 0.0;
        //  gp_Pnt P3, PT3, P4, PT4;
        int flag = 0;
        if (theRg >= G1)
        {
            flag += 1;
        }
        if (theRg >= G2)
        {
            flag += 2;
        }
        bool insP3 = Interpolation(theU1, theU2, ref theNod11RValues, ref theNod12RValues, ref X3, ref Y3, ref Z3,
            ref XTI3, ref YTI3, ref ZTI3, ref coef3, ref U3, ref mP3P1);
        bool insP4 = Interpolation(theU1, theU2, ref theNod21RValues, ref theNod22RValues, ref X4, ref Y4, ref Z4,
            ref XTI4, ref YTI4, ref ZTI4, ref coef4, ref U4, ref mP4P1);
        bool isOK = insP3 || insP4;
        if (isOK)
        {
            if (!insP4) // p1 i1p3 p2
            {
                MoveOrInsertPoint(theList, ref theX1, ref theY1, ref theZ1, ref theX2, ref theY2, ref theZ2, ref theXTI1,
                    ref theYTI1, ref theZTI1, ref theXTI2, ref theYTI2, ref theZTI2, theIEdge, ref theU1, ref theU2,
                    ref theNod11Indices, ref theNod11RValues, ref theNod12Indices, ref theNod12RValues, theI1p1, theI1p2,
                    theI1, thePid1, ref theTData1, ref thePISeg1, ref thePINod1, ref theNod21Indices,
                    ref theNod21RValues, ref theNod22Indices, ref theNod22RValues, theI2p1, theI2p2, theI2, thePid2,
                    ref theTData2, ref thePISeg2, ref thePINod2, X3, Y3, Z3, XTI3, YTI3, ZTI3, coef3, U3, insP3, mP3P1,
                    flag);
            }
            else if (!insP3) // p1 i2p4 p2
            {
                MoveOrInsertPoint(theList, ref theX1, ref theY1, ref theZ1, ref theX2, ref theY2, ref theZ2, ref theXTI1,
                    ref theYTI1, ref theZTI1, ref theXTI2, ref theYTI2, ref theZTI2, theIEdge, ref theU1, ref theU2,
                    ref theNod21Indices, ref theNod21RValues, ref theNod22Indices, ref theNod22RValues, theI2p1, theI2p2,
                    theI2, thePid2, ref theTData2, ref thePISeg2, ref thePINod2, ref theNod11Indices,
                    ref theNod11RValues, ref theNod12Indices, ref theNod12RValues, theI1p1, theI1p2, theI1, thePid1,
                    ref theTData1, ref thePISeg1, ref thePINod1, X4, Y4, Z4, XTI4, YTI4, ZTI4, coef4, U4, insP4, mP4P1,
                    flag);
            }
            else if (Math.Abs(coef4 - coef3) < myTolSta) // p1 i1p3-i2p4 p2
            {
                MoveOrInsertPoint(theList, ref theX1, ref theY1, ref theZ1, ref theX2, ref theY2, ref theZ2, ref theXTI1,
                    ref theYTI1, ref theZTI1, ref theXTI2, ref theYTI2, ref theZTI2, theIEdge, ref theU1, ref theU2,
                    ref theNod21Indices, ref theNod21RValues, ref theNod22Indices, ref theNod22RValues, theI2p1, theI2p2,
                    theI2, thePid2, ref theTData2, ref thePISeg2, ref thePINod2, ref theNod11Indices,
                    ref theNod11RValues, ref theNod12Indices, ref theNod12RValues, theI1p1, theI1p2, theI1, thePid1,
                    ref theTData1, ref thePISeg1, ref thePINod1, X4, Y4, Z4, XTI4, YTI4, ZTI4, coef4, U4, insP4, mP4P1,
                    flag);
            }
            else if (coef4 < coef3) // p1 i2p4 i1p3 p2
            {
                MoveOrInsertPoint(theList, ref theX1, ref theY1, ref theZ1, ref theX2, ref theY2, ref theZ2, ref theXTI1,
                    ref theYTI1, ref theZTI1, ref theXTI2, ref theYTI2, ref theZTI2, theIEdge, ref theU1, ref theU2,
                    ref theNod21Indices, ref theNod21RValues, ref theNod22Indices, ref theNod22RValues, theI2p1, theI2p2,
                    theI2, thePid2, ref theTData2, ref thePISeg2, ref thePINod2, ref theNod11Indices,
                    ref theNod11RValues, ref theNod12Indices, ref theNod12RValues, theI1p1, theI1p2, theI1, thePid1,
                    ref theTData1, ref thePISeg1, ref thePINod1, X4, Y4, Z4, XTI4, YTI4, ZTI4, coef4, U4, insP4, mP4P1,
                    X3, Y3, Z3, XTI3, YTI3, ZTI3, coef3, U3, insP3, mP3P1, flag);
            }
            else // p1 i1p3 i2p4 p2
            {
                MoveOrInsertPoint(theList, ref theX1, ref theY1, ref theZ1, ref theX2, ref theY2, ref theZ2, ref theXTI1,
                    ref theYTI1, ref theZTI1, ref theXTI2, ref theYTI2, ref theZTI2, theIEdge, ref theU1, ref theU2,
                    ref theNod11Indices, ref theNod11RValues, ref theNod12Indices, ref theNod12RValues, theI1p1, theI1p2,
                    theI1, thePid1, ref theTData1, ref thePISeg1, ref thePINod1, ref theNod21Indices,
                    ref theNod21RValues, ref theNod22Indices, ref theNod22RValues, theI2p1, theI2p2, theI2, thePid2,
                    ref theTData2, ref thePISeg2, ref thePINod2, X3, Y3, Z3, XTI3, YTI3, ZTI3, coef3, U3, insP3, mP3P1,
                    X4, Y4, Z4, XTI4, YTI4, ZTI4, coef4, U4, insP4, mP4P1, flag);
            }
        }
        else // p1 p2
        {
            theList.Prepend(new HLRAlgo_BiPoint(theXTI1, theYTI1, theZTI1, theXTI2, theYTI2, theZTI2, theX1, theY1,
                theZ1, theX2, theY2, theZ2, theIEdge, theI1, theI1p1, theI1p2, theI2, theI2p1, theI2p2, flag));
        }
    }


    private bool Interpolation(double theU1, double theU2, ref HLRAlgo_PolyInternalNode.NodeData theNod1RValues,
        ref HLRAlgo_PolyInternalNode.NodeData theNod2RValues, ref double theX3, ref double theY3, ref double theZ3,
        ref double theXTI3, ref double theYTI3, ref double theZTI3, ref double theCoef3, ref double theU3,
        ref bool themP3P1)
    {
        if (NewNode(ref theNod1RValues, ref theNod2RValues, ref theCoef3, ref themP3P1))
        {
            theU3 = theU1 + (theU2 - theU1) * theCoef3;
            Point3D aP3 = myBCurv.Value(theU3);
            theXTI3 = theX3 = aP3.X();
            theYTI3 = theY3 = aP3.Y();
            theZTI3 = theZ3 = aP3.Z();
            TMultiply(ref theX3, ref theY3, ref theZ3);
            return true;
        }

        theX3 = theY3 = theZ3 = theXTI3 = theYTI3 = theZTI3 = theCoef3 = theU3 = 0.0;
        return false;
    }


    private void MoveOrInsertPoint(NCollection_List<HLRAlgo_BiPoint> theList, ref double theX1, ref double theY1,
        ref double theZ1, ref double theX2, ref double theY2, ref double theZ2, ref double theXTI1, ref double theYTI1,
        ref double theZTI1, ref double theXTI2, ref double theYTI2, ref double theZTI2, int theIEdge, ref double theU1,
        ref double theU2, ref HLRAlgo_PolyInternalNode.NodeIndices theNod11Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod11RValues, ref HLRAlgo_PolyInternalNode.NodeIndices theNod12Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod12RValues, int theI1p1, int theI1p2, int theI1,
        HLRAlgo_PolyInternalData thePid1, ref NCollection_Array1<HLRAlgo_TriangleData> theTData1,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg1,
        ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod1, double theX3, double theY3, double theZ3,
        double theXTI3, double theYTI3, double theZTI3, double theCoef3, double theU3, bool theInsP3, bool themP3P1,
        int theFlag)
    {
        NCollection_Array1<HLRAlgo_TriangleData> aTData2 = null!;
        NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg2 = null!;
        NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod2 = null!;
        bool anIns3 = theInsP3;
        if (anIns3 && themP3P1) // P1 ---> P3
        {
            if (((theNod11Indices.Flag & NMsk_Vert) == 0) && theCoef3 < myTolSta)
            {
                anIns3 = false;
                ChangeNode(theI1p1, theI1p2, ref theNod11Indices, ref theNod11RValues, ref theNod12Indices,
                    ref theNod12RValues, theCoef3, theX3, theY3, theZ3, true, theTData1, thePISeg1, thePINod1);
                theX1 = theX3;
                theY1 = theY3;
                theZ1 = theZ3;
                theXTI1 = theXTI3;
                theYTI1 = theYTI3;
                theZTI1 = theZTI3;
                theU1 = theU3;
                theNod11RValues.Point = new Xyz(theX3, theY3, theZ3);
                if (theNod11Indices.Edg1 == theIEdge)
                {
                    theNod11RValues.PCu1 = theU3;
                }
                else if (theNod11Indices.Edg2 == theIEdge)
                {
                    theNod11RValues.PCu2 = theU3;
                }
                theNod11RValues.Scal = 0;
                theNod11Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI1p1, ref theNod11Indices, theTData1, thePISeg1, thePINod1);
                ref HLRAlgo_BiPoint.PointsT aPoints = ref theList.ChangeFirst().Points();
                aPoints.PntP2 = new Xyz(theX3, theY3, theZ3);
                aPoints.Pnt2 = new Xyz(theXTI3, theYTI3, theZTI3);
            }
        }
        if (anIns3 && !themP3P1) // P2 ---> P3
        {
            if (((theNod12Indices.Flag & NMsk_Vert) == 0) && theCoef3 > myTolEnd)
            {
                anIns3 = false;
                ChangeNode(theI1p1, theI1p2, ref theNod11Indices, ref theNod11RValues, ref theNod12Indices,
                    ref theNod12RValues, theCoef3, theX3, theY3, theZ3, false, theTData1, thePISeg1, thePINod1);
                theX2 = theX3;
                theY2 = theY3;
                theZ2 = theZ3;
                theXTI2 = theXTI3;
                theYTI2 = theYTI3;
                theZTI2 = theZTI3;
                theU2 = theU3;
                theNod12RValues.Point = new Xyz(theX3, theY3, theZ3);
                if (theNod12Indices.Edg1 == theIEdge)
                {
                    theNod12RValues.PCu1 = theU3;
                }
                else if (theNod12Indices.Edg2 == theIEdge)
                {
                    theNod12RValues.PCu2 = theU3;
                }
                theNod12RValues.Scal = 0;
                theNod12Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI1p2, ref theNod12Indices, theTData1, thePISeg1, thePINod1);
            }
        }
        if (anIns3) // p1 i1p3 p2
        {
            int anI1p3 = thePid1.AddNode(ref theNod11RValues, ref theNod12RValues, ref thePINod1, ref aPINod2, theCoef3,
                theX3, theY3, theZ3);
            HLRAlgo_PolyInternalNode aPi1p3 = thePINod1[anI1p3]!;
            ref HLRAlgo_PolyInternalNode.NodeIndices aNod13Indices = ref aPi1p3.Indices();
            ref HLRAlgo_PolyInternalNode.NodeData aNod13RValues = ref aPi1p3.Data();
            aNod13Indices.Edg1 = theIEdge;
            aNod13RValues.PCu1 = theU3;
            aNod13RValues.Scal = 0;
            aNod13Indices.Flag |= NMsk_OutL;
            aNod13Indices.Flag |= NMsk_Edge;
            thePid1.UpdateLinks(theI1p1, theI1p2, anI1p3, ref theTData1, ref aTData2, ref thePISeg1, ref aPISeg2,
                ref thePINod1, ref aPINod2);
            UpdateAroundNode(anI1p3, ref aNod13Indices, theTData1, thePISeg1, thePINod1);
            theList.Prepend(new HLRAlgo_BiPoint(theXTI1, theYTI1, theZTI1, theXTI3, theYTI3, theZTI3, theX1, theY1,
                theZ1, theX3, theY3, theZ3, theIEdge, theI1, theI1p1, anI1p3, theFlag));
            theList.Prepend(new HLRAlgo_BiPoint(theXTI3, theYTI3, theZTI3, theXTI2, theYTI2, theZTI2, theX3, theY3,
                theZ3, theX2, theY2, theZ2, theIEdge, theI1, anI1p3, theI1p2, theFlag));
        }
        else // p1 p2
        {
            theList.Prepend(new HLRAlgo_BiPoint(theXTI1, theYTI1, theZTI1, theXTI2, theYTI2, theZTI2, theX1, theY1,
                theZ1, theX2, theY2, theZ2, theIEdge, theI1, theI1p1, theI1p2, theFlag));
        }
    }


    private void MoveOrInsertPoint(NCollection_List<HLRAlgo_BiPoint> theList, ref double theX1, ref double theY1,
        ref double theZ1, ref double theX2, ref double theY2, ref double theZ2, ref double theXTI1, ref double theYTI1,
        ref double theZTI1, ref double theXTI2, ref double theYTI2, ref double theZTI2, int theIEdge, ref double theU1,
        ref double theU2, ref HLRAlgo_PolyInternalNode.NodeIndices theNod11Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod11RValues, ref HLRAlgo_PolyInternalNode.NodeIndices theNod12Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod12RValues, int theI1p1, int theI1p2, int theI1,
        HLRAlgo_PolyInternalData thePid1, ref NCollection_Array1<HLRAlgo_TriangleData> theTData1,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg1,
        ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod1,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod21Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod21RValues,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod22Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod22RValues,
        int theI2p1, int theI2p2, int theI2, HLRAlgo_PolyInternalData thePid2,
        ref NCollection_Array1<HLRAlgo_TriangleData> theTData2,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg2,
        ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod2, double theX3, double theY3, double theZ3,
        double theXTI3, double theYTI3, double theZTI3, double theCoef3, double theU3, bool theInsP3, bool themP3P1,
        int theFlag)
    {
        bool anIns3 = theInsP3;
        if (anIns3 && themP3P1) // P1 ---> P3
        {
            if (((theNod11Indices.Flag & NMsk_Vert) == 0) && theCoef3 < myTolSta)
            {
                anIns3 = false;
                ChangeNode(theI1p1, theI1p2, ref theNod11Indices, ref theNod11RValues, ref theNod12Indices,
                    ref theNod12RValues, theCoef3, theX3, theY3, theZ3, true, theTData1, thePISeg1, thePINod1);
                ChangeNode(theI2p1, theI2p2, ref theNod21Indices, ref theNod21RValues, ref theNod22Indices,
                    ref theNod22RValues, theCoef3, theX3, theY3, theZ3, true, theTData2, thePISeg2, thePINod2);
                theX1 = theX3;
                theY1 = theY3;
                theZ1 = theZ3;
                theXTI1 = theXTI3;
                theYTI1 = theYTI3;
                theZTI1 = theZTI3;
                theU1 = theU3;
                theNod11RValues.Point = new Xyz(theX3, theY3, theZ3);
                if (theNod11Indices.Edg1 == theIEdge)
                {
                    theNod11RValues.PCu1 = theU3;
                }
                else if (theNod11Indices.Edg2 == theIEdge)
                {
                    theNod11RValues.PCu2 = theU3;
                }
                theNod11RValues.Scal = 0;
                theNod11Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI1p1, ref theNod11Indices, theTData1, thePISeg1, thePINod1);
                theNod21RValues.Point = new Xyz(theX3, theY3, theZ3);
                if (theNod21Indices.Edg1 == theIEdge)
                {
                    theNod21RValues.PCu1 = theU3;
                }
                else if (theNod21Indices.Edg2 == theIEdge)
                {
                    theNod21RValues.PCu2 = theU3;
                }
                theNod21RValues.Scal = 0;
                theNod21Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI2p1, ref theNod21Indices, theTData2, thePISeg2, thePINod2);
                ref HLRAlgo_BiPoint.PointsT aPoints = ref theList.ChangeFirst().Points();
                aPoints.PntP2 = new Xyz(theX3, theY3, theZ3);
                aPoints.Pnt2 = new Xyz(theXTI3, theYTI3, theZTI3);
            }
        }
        if (anIns3 && !themP3P1) // P2 ---> P3
        {
            if (((theNod12Indices.Flag & NMsk_Vert) == 0) && theCoef3 > myTolEnd)
            {
                anIns3 = false;
                ChangeNode(theI1p1, theI1p2, ref theNod11Indices, ref theNod11RValues, ref theNod12Indices,
                    ref theNod12RValues, theCoef3, theX3, theY3, theZ3, false, theTData1, thePISeg1, thePINod1);
                ChangeNode(theI2p1, theI2p2, ref theNod21Indices, ref theNod21RValues, ref theNod22Indices,
                    ref theNod22RValues, theCoef3, theX3, theY3, theZ3, false, theTData2, thePISeg2, thePINod2);
                theX2 = theX3;
                theY2 = theY3;
                theZ2 = theZ3;
                theXTI2 = theXTI3;
                theYTI2 = theYTI3;
                theZTI2 = theZTI3;
                theU2 = theU3;
                theNod12RValues.Point = new Xyz(theX3, theY3, theZ3);
                if (theNod12Indices.Edg1 == theIEdge)
                {
                    theNod12RValues.PCu1 = theU3;
                }
                else if (theNod12Indices.Edg2 == theIEdge)
                {
                    theNod12RValues.PCu2 = theU3;
                }
                theNod12RValues.Scal = 0;
                theNod12Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI1p2, ref theNod12Indices, theTData1, thePISeg1, thePINod1);
                theNod22RValues.Point = new Xyz(theX3, theY3, theZ3);
                if (theNod22Indices.Edg1 == theIEdge)
                {
                    theNod22RValues.PCu1 = theU3;
                }
                else if (theNod22Indices.Edg2 == theIEdge)
                {
                    theNod22RValues.PCu2 = theU3;
                }
                theNod22RValues.Scal = 0;
                theNod22Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI2p2, ref theNod22Indices, theTData2, thePISeg2, thePINod2);
            }
        }
        if (anIns3) // p1 i1p3 p2
        {
            int anI1p3 = thePid1.AddNode(ref theNod11RValues, ref theNod12RValues, ref thePINod1, ref thePINod2,
                theCoef3, theX3, theY3, theZ3);
            int anI2p3 = thePid2.AddNode(ref theNod21RValues, ref theNod22RValues, ref thePINod2, ref thePINod1,
                theCoef3, theX3, theY3, theZ3);
            HLRAlgo_PolyInternalNode aPi1p3 = thePINod1[anI1p3]!;
            ref HLRAlgo_PolyInternalNode.NodeIndices aNod13Indices = ref aPi1p3.Indices();
            ref HLRAlgo_PolyInternalNode.NodeData aNod13RValues = ref aPi1p3.Data();
            HLRAlgo_PolyInternalNode aPi2p3 = thePINod2[anI2p3]!;
            ref HLRAlgo_PolyInternalNode.NodeIndices aNod23Indices = ref aPi2p3.Indices();
            ref HLRAlgo_PolyInternalNode.NodeData aNod23RValues = ref aPi2p3.Data();
            aNod13Indices.Edg1 = theIEdge;
            aNod13RValues.PCu1 = theU3;
            aNod13RValues.Scal = 0;
            aNod13Indices.Flag |= NMsk_OutL;
            aNod13Indices.Flag |= NMsk_Edge;
            aNod23Indices.Edg1 = theIEdge;
            aNod23RValues.PCu1 = theU3;
            aNod23RValues.Scal = 0;
            aNod23Indices.Flag |= NMsk_OutL;
            aNod23Indices.Flag |= NMsk_Edge;
            thePid1.UpdateLinks(theI1p1, theI1p2, anI1p3, ref theTData1, ref theTData2, ref thePISeg1, ref thePISeg2,
                ref thePINod1, ref thePINod2);
            thePid2.UpdateLinks(theI2p1, theI2p2, anI2p3, ref theTData2, ref theTData1, ref thePISeg2, ref thePISeg1,
                ref thePINod2, ref thePINod1);
            UpdateAroundNode(anI1p3, ref aNod13Indices, theTData1, thePISeg1, thePINod1);
            UpdateAroundNode(anI2p3, ref aNod23Indices, theTData2, thePISeg2, thePINod2);
            theList.Prepend(new HLRAlgo_BiPoint(theXTI1, theYTI1, theZTI1, theXTI3, theYTI3, theZTI3, theX1, theY1,
                theZ1, theX3, theY3, theZ3, theIEdge, theI1, theI1p1, anI1p3, theI2, theI2p1, anI2p3, theFlag));
            theList.Prepend(new HLRAlgo_BiPoint(theXTI3, theYTI3, theZTI3, theXTI2, theYTI2, theZTI2, theX3, theY3,
                theZ3, theX2, theY2, theZ2, theIEdge, theI1, anI1p3, theI1p2, theI2, anI2p3, theI2p2, theFlag));
        }
        else // p1 p2
        {
            theList.Prepend(new HLRAlgo_BiPoint(theXTI1, theYTI1, theZTI1, theXTI2, theYTI2, theZTI2, theX1, theY1,
                theZ1, theX2, theY2, theZ2, theIEdge, theI1, theI1p1, theI1p2, theI2, theI2p1, theI2p2, theFlag));
        }
    }


    private void MoveOrInsertPoint(NCollection_List<HLRAlgo_BiPoint> theList, ref double theX1, ref double theY1,
        ref double theZ1, ref double theX2, ref double theY2, ref double theZ2, ref double theXTI1, ref double theYTI1,
        ref double theZTI1, ref double theXTI2, ref double theYTI2, ref double theZTI2, int theIEdge, ref double theU1,
        ref double theU2, ref HLRAlgo_PolyInternalNode.NodeIndices theNod11Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod11RValues, ref HLRAlgo_PolyInternalNode.NodeIndices theNod12Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod12RValues, int theI1p1, int theI1p2, int theI1,
        HLRAlgo_PolyInternalData thePid1, ref NCollection_Array1<HLRAlgo_TriangleData> theTData1,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg1,
        ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod1,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod21Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod21RValues,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod22Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod22RValues,
        int theI2p1, int theI2p2, int theI2, HLRAlgo_PolyInternalData thePid2,
        ref NCollection_Array1<HLRAlgo_TriangleData> theTData2,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg2,
        ref NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod2, double theX3, double theY3, double theZ3,
        double theXTI3, double theYTI3, double theZTI3, double theCoef3, double theU3, bool theInsP3, bool themP3P1,
        double theX4, double theY4, double theZ4, double theXTI4, double theYTI4, double theZTI4, double theCoef4,
        double theU4, bool theInsP4, bool themP4P1, int theFlag)
    {
        bool anIns3 = theInsP3;
        bool anIns4 = theInsP4;
        if (anIns3 && themP3P1) // P1 ---> P3
        {
            if (((theNod11Indices.Flag & NMsk_Vert) == 0) && theCoef3 < myTolSta)
            {
                anIns3 = false;
                ChangeNode(theI1p1, theI1p2, ref theNod11Indices, ref theNod11RValues, ref theNod12Indices,
                    ref theNod12RValues, theCoef3, theX3, theY3, theZ3, true, theTData1, thePISeg1, thePINod1);
                ChangeNode(theI2p1, theI2p2, ref theNod21Indices, ref theNod21RValues, ref theNod22Indices,
                    ref theNod22RValues, theCoef3, theX3, theY3, theZ3, true, theTData2, thePISeg2, thePINod2);
                theX1 = theX3;
                theY1 = theY3;
                theZ1 = theZ3;
                theXTI1 = theXTI3;
                theYTI1 = theYTI3;
                theZTI1 = theZTI3;
                theU1 = theU3;
                theNod11RValues.Point = new Xyz(theX3, theY3, theZ3);
                if (theNod11Indices.Edg1 == theIEdge)
                {
                    theNod11RValues.PCu1 = theU3;
                }
                else if (theNod11Indices.Edg2 == theIEdge)
                {
                    theNod11RValues.PCu2 = theU3;
                }
                theNod11RValues.Scal = 0;
                theNod11Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI1p1, ref theNod11Indices, theTData1, thePISeg1, thePINod1);
                theNod21RValues.Point = new Xyz(theX3, theY3, theZ3);
                if (theNod21Indices.Edg1 == theIEdge)
                {
                    theNod21RValues.PCu1 = theU3;
                }
                else if (theNod21Indices.Edg2 == theIEdge)
                {
                    theNod21RValues.PCu2 = theU3;
                }
                theNod21RValues.Scal = 0;
                theNod21Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI2p1, ref theNod21Indices, theTData2, thePISeg2, thePINod2);
                ref HLRAlgo_BiPoint.PointsT aPoints = ref theList.ChangeFirst().Points();
                aPoints.PntP2 = new Xyz(theX3, theY3, theZ3);
                aPoints.Pnt2 = new Xyz(theXTI3, theYTI3, theZTI3);
            }
        }
        if (anIns4 && !themP4P1) // P2 ---> P4
        {
            if (((theNod12Indices.Flag & NMsk_Vert) == 0) && theCoef4 > myTolEnd)
            {
                anIns4 = false;
                ChangeNode(theI2p1, theI2p2, ref theNod21Indices, ref theNod21RValues, ref theNod22Indices,
                    ref theNod22RValues, theCoef4, theX4, theY4, theZ4, false, theTData2, thePISeg2, thePINod2);
                ChangeNode(theI1p1, theI1p2, ref theNod11Indices, ref theNod11RValues, ref theNod12Indices,
                    ref theNod12RValues, theCoef4, theX4, theY4, theZ4, false, theTData1, thePISeg1, thePINod1);
                theX2 = theX4;
                theY2 = theY4;
                theZ2 = theZ4;
                theXTI2 = theXTI4;
                theYTI2 = theYTI4;
                theZTI2 = theZTI4;
                theU2 = theU4;
                theNod12RValues.Point = new Xyz(theX4, theY4, theZ4);
                if (theNod12Indices.Edg1 == theIEdge)
                {
                    theNod12RValues.PCu1 = theU4;
                }
                else if (theNod12Indices.Edg2 == theIEdge)
                {
                    theNod12RValues.PCu2 = theU4;
                }
                theNod12RValues.Scal = 0;
                theNod12Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI1p2, ref theNod12Indices, theTData1, thePISeg1, thePINod1);
                theNod22RValues.Point = new Xyz(theX4, theY4, theZ4);
                if (theNod22Indices.Edg1 == theIEdge)
                {
                    theNod22RValues.PCu1 = theU4;
                }
                else if (theNod22Indices.Edg2 == theIEdge)
                {
                    theNod22RValues.PCu2 = theU4;
                }
                theNod22RValues.Scal = 0;
                theNod22Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(theI2p2, ref theNod22Indices, theTData2, thePISeg2, thePINod2);
            }
        }
        if (anIns3 || anIns4)
        {
            if (!anIns4) // p1 i1p3 p2
            {
                MoveOrInsertPoint(theList, ref theX1, ref theY1, ref theZ1, ref theX2, ref theY2, ref theZ2, ref theXTI1,
                    ref theYTI1, ref theZTI1, ref theXTI2, ref theYTI2, ref theZTI2, theIEdge, ref theU1, ref theU2,
                    ref theNod11Indices, ref theNod11RValues, ref theNod12Indices, ref theNod12RValues, theI1p1, theI1p2,
                    theI1, thePid1, ref theTData1, ref thePISeg1, ref thePINod1, ref theNod21Indices,
                    ref theNod21RValues, ref theNod22Indices, ref theNod22RValues, theI2p1, theI2p2, theI2, thePid2,
                    ref theTData2, ref thePISeg2, ref thePINod2, theX3, theY3, theZ3, theXTI3, theYTI3, theZTI3,
                    theCoef3, theU3, theInsP3, themP3P1, theFlag);
            }
            else if (!anIns3) // p1 i2p4 p2
            {
                MoveOrInsertPoint(theList, ref theX1, ref theY1, ref theZ1, ref theX2, ref theY2, ref theZ2, ref theXTI1,
                    ref theYTI1, ref theZTI1, ref theXTI2, ref theYTI2, ref theZTI2, theIEdge, ref theU1, ref theU2,
                    ref theNod21Indices, ref theNod21RValues, ref theNod22Indices, ref theNod22RValues, theI2p1, theI2p2,
                    theI2, thePid2, ref theTData2, ref thePISeg2, ref thePINod2, ref theNod11Indices,
                    ref theNod11RValues, ref theNod12Indices, ref theNod12RValues, theI1p1, theI1p2, theI1, thePid1,
                    ref theTData1, ref thePISeg1, ref thePINod1, theX4, theY4, theZ4, theXTI4, theYTI4, theZTI4,
                    theCoef4, theU4, theInsP4, themP4P1, theFlag);
            }
            else // p1 i1p3 i2p4 p2
            {
                int anI1p3 = thePid1.AddNode(ref theNod11RValues, ref theNod12RValues, ref thePINod1, ref thePINod2,
                    theCoef3, theX3, theY3, theZ3);
                int anI2p3 = thePid2.AddNode(ref theNod21RValues, ref theNod22RValues, ref thePINod2, ref thePINod1,
                    theCoef3, theX3, theY3, theZ3);
                int anI1p4 = thePid1.AddNode(ref theNod11RValues, ref theNod12RValues, ref thePINod1, ref thePINod2,
                    theCoef4, theX4, theY4, theZ4);
                int anI2p4 = thePid2.AddNode(ref theNod21RValues, ref theNod22RValues, ref thePINod2, ref thePINod1,
                    theCoef4, theX4, theY4, theZ4);
                HLRAlgo_PolyInternalNode aPi1p3 = thePINod1[anI1p3]!;
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod13Indices = ref aPi1p3.Indices();
                ref HLRAlgo_PolyInternalNode.NodeData aNod13RValues = ref aPi1p3.Data();
                HLRAlgo_PolyInternalNode aPi1p4 = thePINod1[anI1p4]!;
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod14Indices = ref aPi1p4.Indices();
                ref HLRAlgo_PolyInternalNode.NodeData aNod14RValues = ref aPi1p4.Data();
                HLRAlgo_PolyInternalNode aPi2p3 = thePINod2[anI2p3]!;
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod23Indices = ref aPi2p3.Indices();
                ref HLRAlgo_PolyInternalNode.NodeData aNod23RValues = ref aPi2p3.Data();
                HLRAlgo_PolyInternalNode aPi2p4 = thePINod2[anI2p4]!;
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod24Indices = ref aPi2p4.Indices();
                ref HLRAlgo_PolyInternalNode.NodeData aNod24RValues = ref aPi2p4.Data();
                aNod13Indices.Edg1 = theIEdge;
                aNod13RValues.PCu1 = theU3;
                aNod13RValues.Scal = 0;
                aNod13Indices.Flag |= NMsk_OutL;
                aNod13Indices.Flag |= NMsk_Edge;
                aNod23Indices.Edg1 = theIEdge;
                aNod23RValues.PCu1 = theU3;
                aNod23RValues.Scal = 0;
                aNod23Indices.Flag |= NMsk_OutL;
                aNod23Indices.Flag |= NMsk_Edge;
                aNod14Indices.Edg1 = theIEdge;
                aNod14RValues.PCu1 = theU4;
                aNod14RValues.Scal = 0;
                aNod14Indices.Flag |= NMsk_OutL;
                aNod14Indices.Flag |= NMsk_Edge;
                aNod24Indices.Edg1 = theIEdge;
                aNod24RValues.PCu1 = theU4;
                aNod24RValues.Scal = 0;
                aNod24Indices.Flag |= NMsk_OutL;
                aNod24Indices.Flag |= NMsk_Edge;
                thePid1.UpdateLinks(theI1p1, theI1p2, anI1p3, ref theTData1, ref theTData2, ref thePISeg1, ref thePISeg2,
                    ref thePINod1, ref thePINod2);
                thePid2.UpdateLinks(theI2p1, theI2p2, anI2p3, ref theTData2, ref theTData1, ref thePISeg2, ref thePISeg1,
                    ref thePINod2, ref thePINod1);
                thePid2.UpdateLinks(anI2p3, theI2p2, anI2p4, ref theTData2, ref theTData1, ref thePISeg2, ref thePISeg1,
                    ref thePINod2, ref thePINod1);
                thePid1.UpdateLinks(anI1p3, theI1p2, anI1p4, ref theTData1, ref theTData2, ref thePISeg1, ref thePISeg2,
                    ref thePINod1, ref thePINod2);
                UpdateAroundNode(anI1p3, ref aNod13Indices, theTData1, thePISeg1, thePINod1);
                UpdateAroundNode(anI2p3, ref aNod23Indices, theTData2, thePISeg2, thePINod2);
                UpdateAroundNode(anI1p4, ref aNod14Indices, theTData1, thePISeg1, thePINod1);
                UpdateAroundNode(anI2p4, ref aNod24Indices, theTData2, thePISeg2, thePINod2);
                theList.Prepend(new HLRAlgo_BiPoint(theXTI1, theYTI1, theZTI1, theXTI3, theYTI3, theZTI3, theX1, theY1,
                    theZ1, theX3, theY3, theZ3, theIEdge, theI1, theI1p1, anI1p3, theI2, theI2p1, anI2p3, theFlag));
                theList.Prepend(new HLRAlgo_BiPoint(theXTI3, theYTI3, theZTI3, theXTI4, theYTI4, theZTI4, theX3, theY3,
                    theZ3, theX4, theY4, theZ4, theIEdge, theI1, anI1p3, anI1p4, theI2, anI2p3, anI2p4, theFlag));
                theList.Prepend(new HLRAlgo_BiPoint(theXTI4, theYTI4, theZTI4, theXTI2, theYTI2, theZTI2, theX4, theY4,
                    theZ4, theX2, theY2, theZ2, theIEdge, theI1, anI1p4, theI1p2, theI2, anI2p4, theI2p2, theFlag));
            }
        }
        else // p1 p2
        {
            theList.Prepend(new HLRAlgo_BiPoint(theXTI1, theYTI1, theZTI1, theXTI2, theYTI2, theZTI2, theX1, theY1,
                theZ1, theX2, theY2, theZ2, theIEdge, theI1, theI1p1, theI1p2, theI2, theI2p1, theI2p2, theFlag));
        }
    }


    private void InsertOnOutLine(NCollection_Array1<HLRAlgo_PolyInternalData?> thePID)
    {
        NCollection_Array1<HLRAlgo_TriangleData> aTData2 = null!;
        NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg2 = null!;
        NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod2 = null!;

        Placement aLoc;
        bool mP3P1 = false;
        double aU3 = 0.0, aV3 = 0.0, aCoef3 = 0.0, X3 = 0.0, Y3 = 0.0, Z3 = 0.0;

        Transform3D aProjTrsf = myProj.Transformation();

        int aNbFaces = myFMap.Extent();
        for (int aFaceIter = 1; aFaceIter <= aNbFaces; ++aFaceIter)
        {
            HLRAlgo_PolyInternalData aPid = thePID[aFaceIter]!;
            if (aPid is null)
            {
                continue;
            }

            bool isIntOutL = false;
            NCollection_Array1<HLRAlgo_TriangleData> aTData1 = aPid.TData();
            NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg1 = aPid.PISeg();
            NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod1 = aPid.PINod();
            TopoDS_Shape aLocalShape = myFMap[aFaceIter];
            TopoDS_Face aFace = TopoDS.Face(aLocalShape);
            myBSurf.Initialize(aFace, false);
            myGSurf = BRep_Tool.Surface(aFace, out aLoc);
            {
                Transform3D aTT = aLoc.Transformation();
                aTT.PreMultiply(aProjTrsf);
                Xyz aTTrsfVec = aTT.TranslationPart();
                TTLo[0] = aTTrsfVec.X();
                TTLo[1] = aTTrsfVec.Y();
                TTLo[2] = aTTrsfVec.Z();
                Matrix3x3 aTTrsfMat = aTT.VectorialPart();
                TTMa[0, 0] = aTTrsfMat.Value(1, 1);
                TTMa[0, 1] = aTTrsfMat.Value(1, 2);
                TTMa[0, 2] = aTTrsfMat.Value(1, 3);
                TTMa[1, 0] = aTTrsfMat.Value(2, 1);
                TTMa[1, 1] = aTTrsfMat.Value(2, 2);
                TTMa[1, 2] = aTTrsfMat.Value(2, 3);
                TTMa[2, 0] = aTTrsfMat.Value(3, 1);
                TTMa[2, 1] = aTTrsfMat.Value(3, 2);
                TTMa[2, 2] = aTTrsfMat.Value(3, 3);
            }


            int aNbSegs = aPid.NbPISeg();
            for (int aSegIter = 1; aSegIter <= aNbSegs; ++aSegIter)
            {
                ref HLRAlgo_PolyInternalSegment aSegIndices = ref aPISeg1[aSegIter];
                //	bool Cutted = false;
                if (aSegIndices.Conex1 != 0 && aSegIndices.Conex2 != 0)
                {
                    int ip1 = aSegIndices.LstSg1;
                    int ip2 = aSegIndices.LstSg2;
                    HLRAlgo_PolyInternalNode aPip1 = aPINod1[ip1]!;
                    ref HLRAlgo_PolyInternalNode.NodeIndices aNod1Indices = ref aPip1.Indices();
                    ref HLRAlgo_PolyInternalNode.NodeData aNod1RValues = ref aPip1.Data();
                    HLRAlgo_PolyInternalNode aPip2 = aPINod1[ip2]!;
                    ref HLRAlgo_PolyInternalNode.NodeIndices aNod2Indices = ref aPip2.Indices();
                    ref HLRAlgo_PolyInternalNode.NodeData aNod2RValues = ref aPip2.Data();
                    if ((aNod1Indices.Flag & NMsk_OutL) != 0 && (aNod2Indices.Flag & NMsk_OutL) != 0)
                    {
                        isIntOutL = true;
                    }
                    else if ((aNod1RValues.Scal >= myTolAngular && aNod2RValues.Scal <= -myTolAngular) || (aNod2RValues.Scal >= myTolAngular && aNod1RValues.Scal <= -myTolAngular))
                    {
                        isIntOutL = true;
                        bool isInsP3 = NewNode(ref aNod1RValues, ref aNod2RValues, ref aCoef3, ref mP3P1);
                        if (isInsP3)
                        {
                            UVNode(ref aNod1RValues, ref aNod2RValues, aCoef3, ref aU3, ref aV3);
                            if (myGSurf is not null)
                            {
                                Point3D aPT3 = myGSurf.Value(aU3, aV3);
                                X3 = aPT3.X();
                                Y3 = aPT3.Y();
                                Z3 = aPT3.Z();
                            }
                            else
                            {
                                // simple averaging - this could be improved
                                double aCoef2 = 1.0 - aCoef3;
                                Point3D aPT3 = new Point3D(aCoef2 * aNod1RValues.Point + aCoef3 * aNod2RValues.Point);
                                X3 = aPT3.X();
                                Y3 = aPT3.Y();
                                Z3 = aPT3.Z();
                            }
                            TTMultiply(ref X3, ref Y3, ref Z3);
                        }

                        if (isInsP3 && mP3P1) // P1 ---> P3
                        {
                            if ((aNod1Indices.Flag & NMsk_Edge) == 0 && aCoef3 < myTolSta)
                            {
                                isInsP3 = false;
                                ChangeNode(ip1, ip2, ref aNod1Indices, ref aNod1RValues, ref aNod2Indices,
                                    ref aNod2RValues, aCoef3, X3, Y3, Z3, true, aTData1, aPISeg1, aPINod1);
                                aNod1RValues.Scal = 0;
                                aNod1Indices.Flag |= NMsk_OutL;
                            }
                        }
                        if (isInsP3 && !mP3P1) // P2 ---> P3
                        {
                            if ((aNod2Indices.Flag & NMsk_Edge) == 0 && aCoef3 > myTolEnd)
                            {
                                isInsP3 = false;
                                ChangeNode(ip1, ip2, ref aNod1Indices, ref aNod1RValues, ref aNod2Indices,
                                    ref aNod2RValues, aCoef3, X3, Y3, Z3, false, aTData1, aPISeg1, aPINod1);
                                aNod2RValues.Scal = 0;
                                aNod2Indices.Flag |= NMsk_OutL;
                            }
                        }
                        if (isInsP3) // p1 ip3 p2
                        {
                            int ip3 =
                                aPid.AddNode(ref aNod1RValues, ref aNod2RValues, ref aPINod1, ref aPINod2, aCoef3, X3, Y3, Z3);
                            HLRAlgo_PolyInternalNode aPip3 = aPINod1[ip3]!;
                            ref HLRAlgo_PolyInternalNode.NodeIndices aNod3Indices = ref aPip3.Indices();
                            ref HLRAlgo_PolyInternalNode.NodeData aNod3RValues = ref aPip3.Data();
                            aPid.UpdateLinks(ip1, ip2, ip3, ref aTData1, ref aTData2, ref aPISeg1, ref aPISeg2, ref aPINod1, ref aPINod2);
                            UpdateAroundNode(ip3, ref aNod3Indices, aTData1, aPISeg1, aPINod1);
                            aNod3RValues.Scal = 0;
                            aNod3Indices.Flag |= NMsk_OutL;
                        }
                    }
                }
            }
            if (isIntOutL)
            {
                aPid.IntOutL(true);
            }

        }
    }


    private void CheckFrBackTriangles(NCollection_List<HLRAlgo_BiPoint> theList,
        NCollection_Array1<HLRAlgo_PolyInternalData?> thePID)
    {
        double X1 = 0.0, Y1 = 0.0, X2 = 0.0, Y2 = 0.0, X3 = 0.0, Y3 = 0.0;
        double D1, D2, D3;
        double dd, dX, dY, nX, nY;

        NCollection_Array1<HLRAlgo_TriangleData> aTData1 = null!;
        NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg1 = null!;
        NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod1 = null!;
        NCollection_Array1<HLRAlgo_TriangleData> aTData2 = null!;
        NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg2 = null!;
        NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod2 = null!;

        int aNbFaces = myFMap.Extent();
        bool isModif = true;
        int iLoop = 0;

        while (isModif && iLoop < 4)
        {
            iLoop++;
            isModif = false;
            bool isFrBackInList = false;
            for (int aFaceIter = 1; aFaceIter <= aNbFaces; ++aFaceIter)
            {
                HLRAlgo_PolyInternalData pid = thePID[aFaceIter]!;
                if (pid is null)
                {
                    continue;
                }

                int aNbTris = pid.NbTData();
                NCollection_Array1<HLRAlgo_TriangleData> aTData = pid.TData();
                NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg = pid.PISeg();
                NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod = pid.PINod();
                for (int aTriIter = 1; aTriIter <= aNbTris; ++aTriIter)
                {
                    ref HLRAlgo_TriangleData tdata = ref aTData[aTriIter];
                    if ((tdata.Flags & HLRAlgo_PolyMask_FMskSide) == 0 && ((tdata.Flags & HLRAlgo_PolyMask_FMskFrBack) != 0))
                    {
                        isModif = true;
                        HLRAlgo_PolyInternalNode pi1p1 = aPINod[tdata.Node1]!;
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNod11Indices = ref pi1p1.Indices();
                        ref HLRAlgo_PolyInternalNode.NodeData aNod11RValues = ref pi1p1.Data();
                        HLRAlgo_PolyInternalNode pi1p2 = aPINod[tdata.Node2]!;
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNod12Indices = ref pi1p2.Indices();
                        ref HLRAlgo_PolyInternalNode.NodeData aNod12RValues = ref pi1p2.Data();
                        HLRAlgo_PolyInternalNode pi1p3 = aPINod[tdata.Node3]!;
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNod13Indices = ref pi1p3.Indices();
                        ref HLRAlgo_PolyInternalNode.NodeData aNod13RValues = ref pi1p3.Data();
                        D1 = 0.0;
                        D2 = 0.0;
                        D3 = 0.0;
                        if (((aNod11Indices.Flag & NMsk_Edge) == 0 || iLoop > 1) && ((aNod11Indices.Flag & NMsk_OutL) == 0 || iLoop > 1) && ((aNod11Indices.Flag & NMsk_Vert) == 0))
                        {
                            dX = aNod13RValues.Point.X() - aNod12RValues.Point.X();
                            dY = aNod13RValues.Point.Y() - aNod12RValues.Point.Y();
                            D1 = dX * dX + dY * dY;
                            D1 = Math.Sqrt(D1);
                            nX = -dY / D1;
                            nY = dX / D1;
                            dX = aNod11RValues.Point.X() - aNod12RValues.Point.X();
                            dY = aNod11RValues.Point.Y() - aNod12RValues.Point.Y();
                            dd = -(dX * nX + dY * nY);
                            if (dd < 0)
                            {
                                dd -= D1 * 0.01;
                            }
                            else
                            {
                                dd += D1 * 0.01;
                            }
                            X1 = nX * dd;
                            Y1 = nY * dd;
                        }
                        if (((aNod12Indices.Flag & NMsk_Edge) == 0 || iLoop > 1) && ((aNod12Indices.Flag & NMsk_OutL) == 0 || iLoop > 1) && ((aNod12Indices.Flag & NMsk_Vert) == 0))
                        {
                            dX = aNod11RValues.Point.X() - aNod13RValues.Point.X();
                            dY = aNod11RValues.Point.Y() - aNod13RValues.Point.Y();
                            D2 = dX * dX + dY * dY;
                            D2 = Math.Sqrt(D2);
                            nX = -dY / D2;
                            nY = dX / D2;
                            dX = aNod12RValues.Point.X() - aNod13RValues.Point.X();
                            dY = aNod12RValues.Point.Y() - aNod13RValues.Point.Y();
                            dd = -(dX * nX + dY * nY);
                            if (dd < 0)
                            {
                                dd -= D2 * 0.01;
                            }
                            else
                            {
                                dd += D2 * 0.01;
                            }
                            X2 = nX * dd;
                            Y2 = nY * dd;
                        }
                        if (((aNod13Indices.Flag & NMsk_Edge) == 0 || iLoop > 1) && ((aNod13Indices.Flag & NMsk_OutL) == 0 || iLoop > 1) && ((aNod13Indices.Flag & NMsk_Vert) == 0))
                        {
                            dX = aNod12RValues.Point.X() - aNod11RValues.Point.X();
                            dY = aNod12RValues.Point.Y() - aNod11RValues.Point.Y();
                            D3 = dX * dX + dY * dY;
                            D3 = Math.Sqrt(D3);
                            nX = -dY / D3;
                            nY = dX / D3;
                            dX = aNod13RValues.Point.X() - aNod11RValues.Point.X();
                            dY = aNod13RValues.Point.Y() - aNod11RValues.Point.Y();
                            dd = -(dX * nX + dY * nY);
                            if (dd < 0)
                            {
                                dd -= D3 * 0.01;
                            }
                            else
                            {
                                dd += D3 * 0.01;
                            }
                            X3 = nX * dd;
                            Y3 = nY * dd;
                        }
                        if (D1 > D2 && D1 > D3)
                        {
                            aNod11RValues.Point.ChangeCoord(1) += X1;
                            aNod11RValues.Point.ChangeCoord(2) += Y1;
                            aNod11Indices.Flag |= NMsk_Move;
                            UpdateAroundNode(tdata.Node1, ref aNod11Indices, aTData, aPISeg, aPINod);
                            isFrBackInList = true;
                        }
                        else if (D2 > D3 && D2 > D1)
                        {
                            aNod12RValues.Point.ChangeCoord(1) += X2;
                            aNod12RValues.Point.ChangeCoord(2) += Y2;
                            aNod12Indices.Flag |= NMsk_Move;
                            UpdateAroundNode(tdata.Node2, ref aNod12Indices, aTData, aPISeg, aPINod);
                            isFrBackInList = true;
                        }
                        else if (D3 > D1 && D3 > D2)
                        {
                            aNod13RValues.Point.ChangeCoord(1) += X3;
                            aNod13RValues.Point.ChangeCoord(2) += Y3;
                            aNod13Indices.Flag |= NMsk_Move;
                            UpdateAroundNode(tdata.Node3, ref aNod13Indices, aTData, aPISeg, aPINod);
                            isFrBackInList = true;
                        }
                    }
                }
            }
            if (isFrBackInList)
            {
                for (NCollection_List<HLRAlgo_BiPoint>.Iterator aBPointIter = new(theList); aBPointIter.More(); aBPointIter.Next())
                {
                    ref HLRAlgo_BiPoint BP = ref aBPointIter.ChangeValue();
                    ref HLRAlgo_BiPoint.IndicesT theIndices = ref BP.Indices();
                    if (theIndices.FaceConex1 != 0)
                    {
                        HLRAlgo_PolyInternalData pid1 = thePID.Value(theIndices.FaceConex1)!;
                        aTData1 = pid1.TData();
                        aPISeg1 = pid1.PISeg();
                        aPINod1 = pid1.PINod();
                    }
                    if (theIndices.FaceConex2 != 0)
                    {
                        if (theIndices.FaceConex1 == theIndices.FaceConex2)
                        {
                            aTData2 = aTData1;
                            aPISeg2 = aPISeg1;
                            aPINod2 = aPINod1;
                        }
                        else
                        {
                            HLRAlgo_PolyInternalData pid2 = thePID.Value(theIndices.FaceConex2)!;
                            aTData2 = pid2.TData();
                            aPISeg2 = pid2.PISeg();
                            aPINod2 = pid2.PINod();
                        }
                    }
                    if (theIndices.FaceConex1 != 0)
                    {
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNod11Indices = ref aPINod1[theIndices.Face1Pt1]!.Indices();
                        if ((aNod11Indices.Flag & NMsk_Move) != 0)
                        {
                            ref HLRAlgo_PolyInternalNode.NodeData aNod11RValues = ref
                                aPINod1.Value(theIndices.Face1Pt1)!.Data();
                            ref HLRAlgo_BiPoint.PointsT aPoints = ref BP.Points();
                            aPoints.Pnt1 = aPoints.PntP1 = aNod11RValues.Point;
                            TIMultiply(ref aPoints.Pnt1);
                            if (theIndices.FaceConex2 != 0)
                            {
                                ref HLRAlgo_PolyInternalNode.NodeIndices aNod12Indices = ref
                                    aPINod2[theIndices.Face2Pt1]!.Indices();
                                ref HLRAlgo_PolyInternalNode.NodeData aNod12RValues = ref
                                    aPINod2[theIndices.Face2Pt1]!.Data();
                                aNod12RValues.Point.ChangeCoord(1) = aNod11RValues.Point.X();
                                aNod12RValues.Point.ChangeCoord(2) = aNod11RValues.Point.Y();
                                UpdateAroundNode(theIndices.Face2Pt1, ref aNod12Indices, aTData2, aPISeg2, aPINod2);
                            }
                        }
                        aNod11Indices = ref aPINod1[theIndices.Face1Pt2]!.Indices();
                        if ((aNod11Indices.Flag & NMsk_Move) != 0)
                        {
                            ref HLRAlgo_PolyInternalNode.NodeData aNod11RValues = ref
                                aPINod1.Value(theIndices.Face1Pt2)!.Data();
                            ref HLRAlgo_BiPoint.PointsT aPoints = ref BP.Points();
                            aPoints.Pnt2 = aPoints.PntP2 = aNod11RValues.Point;
                            TIMultiply(ref aPoints.Pnt2);
                            if (theIndices.FaceConex2 != 0)
                            {
                                ref HLRAlgo_PolyInternalNode.NodeIndices aNod12Indices = ref
                                    aPINod2[theIndices.Face2Pt2]!.Indices();
                                ref HLRAlgo_PolyInternalNode.NodeData aNod12RValues = ref
                                    aPINod2[theIndices.Face2Pt2]!.Data();
                                aNod12RValues.Point.ChangeCoord(1) = aNod11RValues.Point.X();
                                aNod12RValues.Point.ChangeCoord(2) = aNod11RValues.Point.Y();
                                UpdateAroundNode(theIndices.Face2Pt2, ref aNod12Indices, aTData2, aPISeg2, aPINod2);
                            }
                        }
                    }
                    if (theIndices.FaceConex2 != 0)
                    {
                        HLRAlgo_PolyInternalData pid2 = thePID.Value(theIndices.FaceConex2)!;
                        aPINod2 = pid2.PINod();
                        ref HLRAlgo_PolyInternalNode.NodeIndices aNod11Indices = ref aPINod2[theIndices.Face2Pt1]!.Indices();
                        if ((aNod11Indices.Flag & NMsk_Move) != 0)
                        {
                            ref HLRAlgo_PolyInternalNode.NodeData aNod11RValues = ref
                                aPINod2.Value(theIndices.Face2Pt1)!.Data();
                            ref HLRAlgo_BiPoint.PointsT aPoints = ref BP.Points();
                            aPoints.Pnt1 = aPoints.PntP1 = aNod11RValues.Point;
                            TIMultiply(ref aPoints.Pnt1);
                            if (theIndices.FaceConex1 != 0)
                            {
                                ref HLRAlgo_PolyInternalNode.NodeIndices aNod12Indices = ref
                                    aPINod1[theIndices.Face1Pt1]!.Indices();
                                ref HLRAlgo_PolyInternalNode.NodeData aNod12RValues = ref
                                    aPINod1[theIndices.Face1Pt1]!.Data();
                                aNod12RValues.Point.ChangeCoord(1) = aNod11RValues.Point.X();
                                aNod12RValues.Point.ChangeCoord(2) = aNod11RValues.Point.Y();
                                UpdateAroundNode(theIndices.Face1Pt1, ref aNod12Indices, aTData1, aPISeg1, aPINod1);
                            }
                        }
                        aNod11Indices = ref aPINod2[theIndices.Face2Pt2]!.Indices();
                        if ((aNod11Indices.Flag & NMsk_Move) != 0)
                        {
                            ref HLRAlgo_PolyInternalNode.NodeData aNod11RValues = ref aPINod2.Value(theIndices.Face2Pt2)!.Data();
                            ref HLRAlgo_BiPoint.PointsT aPoints = ref BP.Points();
                            aPoints.Pnt2 = aPoints.PntP2 = aNod11RValues.Point;
                            TIMultiply(ref aPoints.Pnt2);
                            if (theIndices.FaceConex1 != 0)
                            {
                                ref HLRAlgo_PolyInternalNode.NodeIndices aNod12Indices = ref
                                    aPINod1[theIndices.Face1Pt2]!.Indices();
                                ref HLRAlgo_PolyInternalNode.NodeData aNod12RValues = ref
                                    aPINod1[theIndices.Face1Pt2]!.Data();
                                aNod12RValues.Point.ChangeCoord(1) = aNod11RValues.Point.X();
                                aNod12RValues.Point.ChangeCoord(2) = aNod11RValues.Point.Y();
                                UpdateAroundNode(theIndices.Face1Pt2, ref aNod12Indices, aTData1, aPISeg1, aPINod1);
                            }
                        }
                    }
                }

                for (int aFaceIter = 1; aFaceIter <= aNbFaces; ++aFaceIter)
                {
                    HLRAlgo_PolyInternalData aPid = thePID[aFaceIter]!;
                    if (aPid is not null)
                    {
                        int aNbNodes = aPid.NbPINod();
                        NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod = aPid.PINod();
                        for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                        {
                            ref HLRAlgo_PolyInternalNode.NodeIndices aNod11Indices = ref
                                aPINod[aNodeIter]!.Indices();
                            aNod11Indices.Flag &= ~NMsk_Move;
                        }
                    }
                }
            }
        }
    }


    private void FindEdgeOnTriangle(in HLRAlgo_TriangleData theTriangle, int theIp1, int theIp2, ref int theJtrouv,
        ref bool theIsDirect)
    {
        int n1 = theTriangle.Node1;
        int n2 = theTriangle.Node2;
        int n3 = theTriangle.Node3;
        if (theIp1 == n1 && theIp2 == n2)
        {
            theJtrouv = 0;
            theIsDirect = true;
            return;
        }
        else if (theIp2 == n1 && theIp1 == n2)
        {
            theJtrouv = 0;
            theIsDirect = false;
            return;
        }
        else if (theIp1 == n2 && theIp2 == n3)
        {
            theJtrouv = 1;
            theIsDirect = true;
            return;
        }
        else if (theIp2 == n2 && theIp1 == n3)
        {
            theJtrouv = 1;
            theIsDirect = false;
            return;
        }
        else if (theIp1 == n3 && theIp2 == n1)
        {
            theJtrouv = 2;
            theIsDirect = true;
            return;
        }
        else if (theIp2 == n3 && theIp1 == n1)
        {
            theJtrouv = 2;
            theIsDirect = false;
            return;
        }
    }


    private void ChangeNode(int theIp1, int theIp2, ref HLRAlgo_PolyInternalNode.NodeIndices theNod1Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod1RValues, ref HLRAlgo_PolyInternalNode.NodeIndices theNod2Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod2RValues, double theCoef1, double theX3, double theY3, double theZ3,
        bool theIsFirst, NCollection_Array1<HLRAlgo_TriangleData> theTData,
        NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg,
        NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod)
    {
        double aCoef2 = 1.0 - theCoef1;
        if (theIsFirst)
        {
            theNod1RValues.Point = new Xyz(theX3, theY3, theZ3);
            theNod1RValues.UV = aCoef2 * theNod1RValues.UV + theCoef1 * theNod2RValues.UV;
            theNod1RValues.Scal = theNod1RValues.Scal * aCoef2 + theNod2RValues.Scal * theCoef1;
            Xyz aXYZ = aCoef2 * theNod1RValues.Normal + theCoef1 * theNod2RValues.Normal;
            double aNorm = aXYZ.Modulus();
            if (aNorm > 0)
            {
                theNod1RValues.Normal = (1 / aNorm) * aXYZ;
            }
            else
            {
                theNod1RValues.Normal = new Xyz(1.0, 0.0, 0.0);
            }
            UpdateAroundNode(theIp1, ref theNod1Indices, theTData, thePISeg, thePINod);
        }
        else
        {
            theNod2RValues.Point = new Xyz(theX3, theY3, theZ3);
            theNod2RValues.UV = aCoef2 * theNod1RValues.UV + theCoef1 * theNod2RValues.UV;
            theNod2RValues.Scal = theNod1RValues.Scal * aCoef2 + theNod2RValues.Scal * theCoef1;
            Xyz aXYZ = aCoef2 * theNod1RValues.Normal + theCoef1 * theNod2RValues.Normal;
            double aNorm = aXYZ.Modulus();
            if (aNorm > 0)
            {
                theNod2RValues.Normal = (1 / aNorm) * aXYZ;
            }
            else
            {
                theNod2RValues.Normal = new Xyz(1.0, 0.0, 0.0);
            }
            UpdateAroundNode(theIp2, ref theNod2Indices, theTData, thePISeg, thePINod);
        }
    }


    private void UpdateAroundNode(int theINode, ref HLRAlgo_PolyInternalNode.NodeIndices theNod1Indices,
        NCollection_Array1<HLRAlgo_TriangleData> theTData, NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg,
        NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod)
    {
        int iiii = theNod1Indices.NdSg;
        while (iiii != 0)
        {
            ref HLRAlgo_PolyInternalSegment aSegIndices = ref thePISeg[iiii];
            int iTri1 = aSegIndices.Conex1;
            int iTri2 = aSegIndices.Conex2;
            if (iTri1 != 0)
            {
                ref HLRAlgo_TriangleData aTriangle = ref theTData[iTri1];
                HLRAlgo_PolyInternalNode aPN1 = thePINod[aTriangle.Node1]!;
                HLRAlgo_PolyInternalNode aPN2 = thePINod[aTriangle.Node2]!;
                HLRAlgo_PolyInternalNode aPN3 = thePINod[aTriangle.Node3]!;
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod1Indices = ref aPN1.Indices();
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod2Indices = ref aPN2.Indices();
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod3Indices = ref aPN3.Indices();
                ref HLRAlgo_PolyInternalNode.NodeData aNod1RValues = ref aPN1.Data();
                ref HLRAlgo_PolyInternalNode.NodeData aNod2RValues = ref aPN2.Data();
                ref HLRAlgo_PolyInternalNode.NodeData aNod3RValues = ref aPN3.Data();
                OrientTriangle(iTri1, ref aTriangle, ref aNod1Indices, ref aNod1RValues, ref aNod2Indices,
                    ref aNod2RValues, ref aNod3Indices, ref aNod3RValues);
            }
            if (iTri2 != 0)
            {
                ref HLRAlgo_TriangleData aTriangle2 = ref theTData[iTri2];
                HLRAlgo_PolyInternalNode aPN1 = thePINod[aTriangle2.Node1]!;
                HLRAlgo_PolyInternalNode aPN2 = thePINod[aTriangle2.Node2]!;
                HLRAlgo_PolyInternalNode aPN3 = thePINod[aTriangle2.Node3]!;
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod1Indices = ref aPN1.Indices();
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod2Indices = ref aPN2.Indices();
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod3Indices = ref aPN3.Indices();
                ref HLRAlgo_PolyInternalNode.NodeData aNod1RValues = ref aPN1.Data();
                ref HLRAlgo_PolyInternalNode.NodeData aNod2RValues = ref aPN2.Data();
                ref HLRAlgo_PolyInternalNode.NodeData aNod3RValues = ref aPN3.Data();
                OrientTriangle(iTri2, ref aTriangle2, ref aNod1Indices, ref aNod1RValues, ref aNod2Indices,
                    ref aNod2RValues, ref aNod3Indices, ref aNod3RValues);
            }
            if (aSegIndices.LstSg1 == theINode)
            {
                iiii = aSegIndices.NxtSg1;
            }
            else
            {
                iiii = aSegIndices.NxtSg2;
            }
        }
    }


    private void OrientTriangle(int theITri, ref HLRAlgo_TriangleData theTriangle,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod1Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod1RValues,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod2Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod2RValues,
        ref HLRAlgo_PolyInternalNode.NodeIndices theNod3Indices, ref HLRAlgo_PolyInternalNode.NodeData theNod3RValues)
    {
        bool o1 = (theNod1Indices.Flag & NMsk_OutL) != 0;
        bool o2 = (theNod2Indices.Flag & NMsk_OutL) != 0;
        bool o3 = (theNod3Indices.Flag & NMsk_OutL) != 0;
        theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskFlat;
        theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskOnOutL;
        if (o1 && o2 && o3)
        {
            theTriangle.Flags |= HLRAlgo_PolyMask_FMskSide;
            theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
            theTriangle.Flags |= HLRAlgo_PolyMask_FMskOnOutL;
            _ = theITri;
        }
        else
        {
            double s1 = theNod1RValues.Scal;
            double s2 = theNod2RValues.Scal;
            double s3 = theNod3RValues.Scal;
            double as1 = s1;
            double as2 = s2;
            double as3 = s3;
            if (s1 < 0)
            {
                as1 = -s1;
            }
            if (s2 < 0)
            {
                as2 = -s2;
            }
            if (s3 < 0)
            {
                as3 = -s3;
            }
            double s = 0;
            double as_ = 0;
            if (!o1)
            {
                s = s1;
                as_ = as1;
            }
            if (!o2 && as_ < as2)
            {
                s = s2;
                as_ = as2;
            }
            if (!o3 && as_ < as3)
            {
                s = s3;
                as_ = as3;
            }
            if (s > 0)
            {
                theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskSide;
                theTriangle.Flags |= HLRAlgo_PolyMask_FMskBack;
            }
            else
            {
                theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskSide;
                theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
            }
            Xyz aD12 = theNod2RValues.Point - theNod1RValues.Point;
            double aD12Norm = aD12.Modulus();
            if (aD12Norm <= 1.0e-10)
            {
                theTriangle.Flags |= HLRAlgo_PolyMask_FMskFlat;
                theTriangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
            }
            else
            {
                Xyz aD23 = theNod3RValues.Point - theNod2RValues.Point;
                double aD23Norm = aD23.Modulus();
                if (aD23Norm < 1.0e-10)
                {
                    theTriangle.Flags |= HLRAlgo_PolyMask_FMskFlat;
                    theTriangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                    theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                }
                else
                {
                    Xyz aD31 = theNod1RValues.Point - theNod3RValues.Point;
                    double aD31Norm = aD31.Modulus();
                    if (aD31Norm < 1.0e-10)
                    {
                        theTriangle.Flags |= HLRAlgo_PolyMask_FMskFlat;
                        theTriangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                        theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                    }
                    else
                    {
                        aD12 *= 1 / aD12Norm;
                        aD23 *= 1 / aD23Norm;
                        Xyz aD = aD12 ^ aD23;
                        double aDNorm = aD.Modulus();
                        if (aDNorm < 1.0e-5)
                        {
                            theTriangle.Flags |= HLRAlgo_PolyMask_FMskFlat;
                            theTriangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                            theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                        }
                        else
                        {
                            double o;
                            if (myProj.Perspective())
                            {
                                aD *= 1 / aDNorm;
                                o = aD.Z() * myProj.Focus() - aD * theNod1RValues.Point;
                            }
                            else
                            {
                                o = aD.Z() / aDNorm;
                            }
                            if (o < 0)
                            {
                                theTriangle.Flags |= HLRAlgo_PolyMask_FMskOrBack;
                                o = -o;
                            }
                            else
                            {
                                theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskOrBack;
                            }
                            if (o < 1.0e-10)
                            {
                                theTriangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                                theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                            }
                        }
                    }
                }
            }
        }

        if ((((theTriangle.Flags & HLRAlgo_PolyMask_FMskBack) == 0) && ((theTriangle.Flags & HLRAlgo_PolyMask_FMskOrBack) != 0)) || (((theTriangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0) && ((theTriangle.Flags & HLRAlgo_PolyMask_FMskOrBack) == 0)))
        {
            theTriangle.Flags |= HLRAlgo_PolyMask_FMskFrBack;
        }
        else
        {
            theTriangle.Flags &= ~HLRAlgo_PolyMask_FMskFrBack;
        }
    }


    private bool Triangles(int theIp1, int theIp2, ref HLRAlgo_PolyInternalNode.NodeIndices theNod1Indices,
        ref NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg, ref int theITri1, ref int theITri2)
    {
        int iiii = theNod1Indices.NdSg;
        while (iiii != 0)
        {
            ref HLRAlgo_PolyInternalSegment aSegIndices = ref thePISeg[iiii];
            if (aSegIndices.LstSg1 == theIp1)
            {
                if (aSegIndices.LstSg2 == theIp2)
                {
                    theITri1 = aSegIndices.Conex1;
                    theITri2 = aSegIndices.Conex2;
                    return true;
                }
                else
                {
                    iiii = aSegIndices.NxtSg1;
                }
            }
            else
            {
                if (aSegIndices.LstSg1 == theIp2)
                {
                    theITri1 = aSegIndices.Conex1;
                    theITri2 = aSegIndices.Conex2;
                    return true;
                }
                else
                {
                    iiii = aSegIndices.NxtSg2;
                }
            }
        }
        theITri1 = 0;
        theITri2 = 0;
        return false;
    }


    private bool NewNode(ref HLRAlgo_PolyInternalNode.NodeData theNod1RValues,
        ref HLRAlgo_PolyInternalNode.NodeData theNod2RValues, ref double theCoef1, ref bool theToMoveP1)
    {
        double aTolAng = myTolAngular * 0.5;
        if ((theNod1RValues.Scal >= aTolAng && theNod2RValues.Scal <= -aTolAng) || (theNod2RValues.Scal >= aTolAng && theNod1RValues.Scal <= -aTolAng))
        {
            theCoef1 = theNod1RValues.Scal / (theNod2RValues.Scal - theNod1RValues.Scal);
            if (theCoef1 < 0)
            {
                theCoef1 = -theCoef1;
            }
            theToMoveP1 = theCoef1 < 0.5;
            return true;
        }
        return false;
    }


    private void UVNode(ref HLRAlgo_PolyInternalNode.NodeData theNod1RValues,
        ref HLRAlgo_PolyInternalNode.NodeData theNod2RValues, double theCoef1, ref double theU3, ref double theV3)
    {
        double aCoef2 = 1.0 - theCoef1;
        Xy aUV3 = aCoef2 * theNod1RValues.UV + theCoef1 * theNod2RValues.UV;
        theU3 = aUV3.X();
        theV3 = aUV3.Y();
    }


    private void CheckDegeneratedSegment(ref HLRAlgo_PolyInternalNode.NodeIndices theNod1Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod1RValues, ref HLRAlgo_PolyInternalNode.NodeIndices theNod2Indices,
        ref HLRAlgo_PolyInternalNode.NodeData theNod2RValues)
    {
        theNod1Indices.Flag |= NMsk_Fuck;
        theNod2Indices.Flag |= NMsk_Fuck;
        if ((theNod1RValues.Scal >= myTolAngular && theNod2RValues.Scal <= -myTolAngular) || (theNod2RValues.Scal >= myTolAngular && theNod1RValues.Scal <= -myTolAngular))
        {
            theNod1RValues.Scal = 0.0;
            theNod1Indices.Flag |= NMsk_OutL;
            theNod2RValues.Scal = 0.0;
            theNod2Indices.Flag |= NMsk_OutL;
        }
    }


    private void UpdateOutLines(NCollection_List<HLRAlgo_BiPoint> theList,
        NCollection_Array1<HLRAlgo_PolyInternalData?> thePID)
    {
        double X1, Y1, Z1, X2, Y2, Z2;
        double XTI1, YTI1, ZTI1, XTI2, YTI2, ZTI2;
        int aNbFaces = myFMap.Extent();
        for (int aFaceIter = 1; aFaceIter <= aNbFaces; ++aFaceIter)
        {
            HLRAlgo_PolyInternalData aPid = thePID[aFaceIter]!;
            if (aPid is null)
            {
                continue;
            }

            if (!aPid.IntOutL())
            {
                continue;
            }

            NCollection_Array1<HLRAlgo_TriangleData> aTData = aPid.TData();
            NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg = aPid.PISeg();
            NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod = aPid.PINod();
            int j, it1, it2, tn1, tn2, tn3, pd, pf;
            bool isOutl = false;
            int aNbSegs = aPid.NbPISeg();
            for (int aSegIter = 1; aSegIter <= aNbSegs; ++aSegIter)
            {
                ref HLRAlgo_PolyInternalSegment psg = ref aPISeg[aSegIter];
                it1 = psg.Conex1;
                it2 = psg.Conex2;
                if (it1 != 0 && it2 != 0 && it1 != it2) // debile but sure !
                {
                    ref HLRAlgo_TriangleData aTriangle = ref aTData[it1];
                    ref HLRAlgo_TriangleData aTriangle2 = ref aTData[it2];
                    if (((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskSide) == 0))
                    {
                        isOutl = ((aTriangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0)
                                          != ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskBack) != 0);
                    }
                    else if (((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskSide) != 0))
                    {
                        isOutl = false;
                    }
                    else if ((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0)
                    {
                        isOutl = ((aTriangle.Flags & HLRAlgo_PolyMask_FMskFlat) == 0)
                                          && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskBack) == 0);
                    }
                    else
                    {
                        isOutl = ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskFlat) == 0)
                                          && ((aTriangle.Flags & HLRAlgo_PolyMask_FMskBack) == 0);
                    }

                    if (isOutl)
                    {
                        pd = psg.LstSg1;
                        pf = psg.LstSg2;
                        tn1 = aTriangle.Node1;
                        tn2 = aTriangle.Node2;
                        tn3 = aTriangle.Node3;
                        if (((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && ((aTriangle.Flags & HLRAlgo_PolyMask_FMskOrBack) != 0))
                        {
                            j = tn1;
                            tn1 = tn3;
                            tn3 = j;
                        }
                        if ((tn1 == pd && tn2 == pf) || (tn1 == pf && tn2 == pd))
                        {
                            aTriangle.Flags |= HLRAlgo_PolyMask_EMskOutLin1;
                        }
                        else if ((tn2 == pd && tn3 == pf) || (tn2 == pf && tn3 == pd))
                        {
                            aTriangle.Flags |= HLRAlgo_PolyMask_EMskOutLin2;
                        }
                        else if ((tn3 == pd && tn1 == pf) || (tn3 == pf && tn1 == pd))
                        {
                            aTriangle.Flags |= HLRAlgo_PolyMask_EMskOutLin3;
                        }
                        tn1 = aTriangle2.Node1;
                        tn2 = aTriangle2.Node2;
                        tn3 = aTriangle2.Node3;
                        if (((aTriangle2.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskOrBack) != 0))
                        {
                            j = tn1;
                            tn1 = tn3;
                            tn3 = j;
                        }
                        if ((tn1 == pd && tn2 == pf) || (tn1 == pf && tn2 == pd))
                        {
                            aTriangle2.Flags |= HLRAlgo_PolyMask_EMskOutLin1;
                        }
                        else if ((tn2 == pd && tn3 == pf) || (tn2 == pf && tn3 == pd))
                        {
                            aTriangle2.Flags |= HLRAlgo_PolyMask_EMskOutLin2;
                        }
                        else if ((tn3 == pd && tn1 == pf) || (tn3 == pf && tn1 == pd))
                        {
                            aTriangle2.Flags |= HLRAlgo_PolyMask_EMskOutLin3;
                        }
                        ref HLRAlgo_PolyInternalNode.NodeData aNod1RValues = ref aPINod.Value(pd)!.Data();
                        ref HLRAlgo_PolyInternalNode.NodeData aNod2RValues = ref aPINod.Value(pf)!.Data();
                        XTI1 = X1 = aNod1RValues.Point.X();
                        YTI1 = Y1 = aNod1RValues.Point.Y();
                        ZTI1 = Z1 = aNod1RValues.Point.Z();
                        XTI2 = X2 = aNod2RValues.Point.X();
                        YTI2 = Y2 = aNod2RValues.Point.Y();
                        ZTI2 = Z2 = aNod2RValues.Point.Z();
                        TIMultiply(ref XTI1, ref YTI1, ref ZTI1);
                        TIMultiply(ref XTI2, ref YTI2, ref ZTI2);
                        theList.Append(new HLRAlgo_BiPoint(XTI1, YTI1, ZTI1, XTI2, YTI2, ZTI2, X1, Y1, Z1, X2, Y2, Z2,
                            aFaceIter, aFaceIter, pd, pf, aFaceIter, pd, pf, 12));
                    }
                }
            }
        }
    }


    private void UpdateEdgesBiPoints(NCollection_List<HLRAlgo_BiPoint> theList,
        NCollection_Array1<HLRAlgo_PolyInternalData?> thePID, bool theIsClosed)
    {
        int itri1 = 0, itri2 = 0, tbid = 0;
        for (NCollection_List<HLRAlgo_BiPoint>.Iterator aBPntIter = new(theList); aBPntIter.More(); aBPntIter.Next())
        {
            ref HLRAlgo_BiPoint aBP = ref aBPntIter.ChangeValue();
            ref HLRAlgo_BiPoint.IndicesT aIndices = ref aBP.Indices();
            if (aIndices.FaceConex1 != 0 && aIndices.FaceConex2 != 0)
            {
                HLRAlgo_PolyInternalData aPid1 = thePID.Value(aIndices.FaceConex1)!;
                HLRAlgo_PolyInternalData aPid2 = thePID.Value(aIndices.FaceConex2)!;
                NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg1 = aPid1.PISeg();
                NCollection_Array1<HLRAlgo_PolyInternalSegment> aPISeg2 = aPid2.PISeg();
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod11Indices = ref
                    aPid1.PINod()[aIndices.Face1Pt1]!.Indices();
                ref HLRAlgo_PolyInternalNode.NodeIndices aNod21Indices = ref
                    aPid2.PINod()[aIndices.Face2Pt1]!.Indices();
                Triangles(aIndices.Face1Pt1, aIndices.Face1Pt2, ref aNod11Indices, ref aPISeg1, ref itri1, ref tbid);
                Triangles(aIndices.Face2Pt1, aIndices.Face2Pt2, ref aNod21Indices, ref aPISeg2, ref itri2, ref tbid);

                if (itri1 != 0 && itri2 != 0)
                {
                    if (aIndices.FaceConex1 != aIndices.FaceConex2 || itri1 != itri2)
                    {
                        NCollection_Array1<HLRAlgo_TriangleData> aTData1 = aPid1.TData();
                        NCollection_Array1<HLRAlgo_TriangleData> aTData2 = aPid2.TData();
                        ref HLRAlgo_TriangleData aTriangle = ref aTData1[itri1];
                        ref HLRAlgo_TriangleData aTriangle2 = ref aTData2[itri2];
                        if (theIsClosed)
                        {
                            if ((((aTriangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskBack) != 0)) || (((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskSide) != 0)) || (((aTriangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskSide) != 0)) || (((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskBack) != 0)))
                            {
                                aBP.Hidden(true);
                            }
                        }

                        bool isOutl;
                        if (((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskSide) == 0))
                        {
                            isOutl = ((aTriangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0)
                                              != ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskBack) != 0);
                        }
                        else if (((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0) && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskSide) != 0))
                        {
                            isOutl = false;
                        }
                        else if (((aTriangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0))
                        {
                            isOutl = ((aTriangle.Flags & HLRAlgo_PolyMask_FMskFlat) == 0)
                                              && ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskBack) == 0);
                        }
                        else
                        {
                            isOutl = ((aTriangle2.Flags & HLRAlgo_PolyMask_FMskFlat) == 0)
                                              && ((aTriangle.Flags & HLRAlgo_PolyMask_FMskBack) == 0);
                        }
                        aBP.OutLine(isOutl);
                    }
                }
            }
        }
    }


    private void UpdatePolyData(NCollection_Array1<HLRAlgo_PolyData?> thePD,
        NCollection_Array1<HLRAlgo_PolyInternalData?> thePID, bool theIsClosed)
    {
        int aNbFaces = myFMap.Extent();
        for (int aFaceIter = 1; aFaceIter <= aNbFaces; ++aFaceIter)
        {
            HLRAlgo_PolyInternalData aPid = thePID[aFaceIter]!;
            if (aPid is null)
            {
                continue;
            }

            int aNbNodes = aPid.NbPINod();
            int aNbTris = aPid.NbTData();
            NCollection_HArray1<Xyz> aHNodes = new NCollection_HArray1<Xyz>(1, aNbNodes);
            NCollection_HArray1<HLRAlgo_TriangleData> aHTData =
                new NCollection_HArray1<HLRAlgo_TriangleData>(1, aNbTris);
            NCollection_Array1<Xyz> aNodes = aHNodes.ChangeArray1();
            NCollection_Array1<HLRAlgo_TriangleData> aTrian = aHTData.ChangeArray1();
            NCollection_Array1<HLRAlgo_TriangleData> aTData = aPid.TData();
            NCollection_Array1<HLRAlgo_PolyInternalNode?> aPINod = aPid.PINod();
            int aNbHidden = 0;

            for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
            {
                ref HLRAlgo_PolyInternalNode.NodeData aNod1RValues = ref aPINod.Value(aNodeIter)!.Data();
                aNodes.SetValue(aNodeIter, aNod1RValues.Point);
            }

            for (int aTriIter = 1; aTriIter <= aNbTris; ++aTriIter)
            {
                ref HLRAlgo_TriangleData anOT = ref aTData[aTriIter];
                ref HLRAlgo_TriangleData aNT = ref aTrian[aTriIter];
                if (((anOT.Flags & HLRAlgo_PolyMask_FMskSide) == 0))
                {
                    if ((anOT.Flags & HLRAlgo_PolyMask_FMskOrBack) != 0)
                    {
                        int j = anOT.Node1;
                        anOT.Node1 = anOT.Node3;
                        anOT.Node3 = j;
                        anOT.Flags |= HLRAlgo_PolyMask_FMskBack;
                    }
                    else
                    {
                        anOT.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                        // Tri1Flags |= HLRAlgo_PolyMask_FMskBack;//OCC349
                    }
                }
                aNT.Node1 = anOT.Node1;
                aNT.Node2 = anOT.Node2;
                aNT.Node3 = anOT.Node3;
                aNT.Flags = anOT.Flags;
                if (((aNT.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && (((aNT.Flags & HLRAlgo_PolyMask_FMskBack) == 0) || !theIsClosed))
                {
                    aNT.Flags |= HLRAlgo_PolyMask_FMskHiding;
                    ++aNbHidden;
                }
                else
                {
                    aNT.Flags &= ~HLRAlgo_PolyMask_FMskHiding;
                }
            }

            NCollection_HArray1<HLRAlgo_PolyHidingData>? aHPHDat = null;
            if (aNbHidden > 0)
            {
                aHPHDat = new NCollection_HArray1<HLRAlgo_PolyHidingData>(1, aNbHidden);
            }

            HLRAlgo_PolyData aPd = thePD[aFaceIter]!;
            aPd.HNodes(aHNodes);
            aPd.HTData(aHTData);
            aPd.HPHDat(aHPHDat);
            aPd.FaceIndex(aFaceIter);
        }
    }


    private void TMultiply(ref double theX, ref double theY, ref double theZ, bool theVPO = false)
    {
        double Xt = TMat[0, 0] * theX + TMat[0, 1] * theY + TMat[0, 2] * theZ + (theVPO ? 0 : TLoc[0]);//OCC349
        double Yt = TMat[1, 0] * theX + TMat[1, 1] * theY + TMat[1, 2] * theZ + (theVPO ? 0 : TLoc[1]);//OCC349
        theZ = TMat[2, 0] * theX + TMat[2, 1] * theY + TMat[2, 2] * theZ + (theVPO ? 0 : TLoc[2]);//OCC349
        theX = Xt;
        theY = Yt;
    }


    private void TTMultiply(ref double theX, ref double theY, ref double theZ, bool theVPO = false)
    {
        double Xt = TTMa[0, 0] * theX + TTMa[0, 1] * theY + TTMa[0, 2] * theZ + (theVPO ? 0 : TTLo[0]);//OCC349
        double Yt = TTMa[1, 0] * theX + TTMa[1, 1] * theY + TTMa[1, 2] * theZ + (theVPO ? 0 : TTLo[1]);//OCC349
        theZ = TTMa[2, 0] * theX + TTMa[2, 1] * theY + TTMa[2, 2] * theZ + (theVPO ? 0 : TTLo[2]);//OCC349
        theX = Xt;
        theY = Yt;
    }


    private void TIMultiply(ref double theX, ref double theY, ref double theZ, bool theVPO = false)
    {
        double Xt = TIMa[0, 0] * theX + TIMa[0, 1] * theY + TIMa[0, 2] * theZ + (theVPO ? 0 : TILo[0]);//OCC349
        double Yt = TIMa[1, 0] * theX + TIMa[1, 1] * theY + TIMa[1, 2] * theZ + (theVPO ? 0 : TILo[1]);//OCC349
        theZ = TIMa[2, 0] * theX + TIMa[2, 1] * theY + TIMa[2, 2] * theZ + (theVPO ? 0 : TILo[2]);//OCC349
        theX = Xt;
        theY = Yt;
    }


    /// <summary>
    /// The shape theShape in a compound with the internal outlines found on its faces' meshes (3D segments), or
    /// theShape alone before <see cref="Update"/>.
    /// </summary>
    public TopoDS_Shape OutLinedShape(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return new TopoDS_Shape();
        }

        TopoDS_Compound aResult = new();
        BRep_Builder aBuilder = new();
        aBuilder.MakeCompound(ref aResult);
        aBuilder.Add(aResult, theShape);
        if (myFMap.IsEmpty())
        {
            aResult.TShape()?.FreezeGraph();
            return aResult;
        }

        NCollection_Map<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        {
            TopExp_Explorer aShapeExp = new();
            for (aShapeExp.Init(theShape, TopAbs_EDGE); aShapeExp.More(); aShapeExp.Next())
            {
                aMap.Add(aShapeExp.Current());
            }
            for (aShapeExp.Init(theShape, TopAbs_FACE); aShapeExp.More(); aShapeExp.Next())
            {
                aMap.Add(aShapeExp.Current());
            }
        }

        NCollection_Array1<HLRAlgo_PolyShellData?> aShell = myAlgo.PolyShell();
        int aNbShells = aShell.Upper();
        NCollection_List<HLRAlgo_BiPoint>.Iterator aBPntIter = new();
        for (int aShellIter = 1; aShellIter <= aNbShells; ++aShellIter)
        {
            NCollection_List<HLRAlgo_BiPoint> aList = aShell.Value(aShellIter)!.Edges();
            for (aBPntIter.Initialize(aList); aBPntIter.More(); aBPntIter.Next())
            {
                ref HLRAlgo_BiPoint aBP = ref aBPntIter.ChangeValue();
                if (aBP.IntLine())
                {
                    ref HLRAlgo_BiPoint.IndicesT aIndices = ref aBP.Indices();
                    if (aMap.Contains(myFMap.FindKey(aIndices.ShapeIndex)))
                    {
                        ref HLRAlgo_BiPoint.PointsT aPoints = ref aBP.Points();
                        aBuilder.Add(aResult, new BRepLib_MakeEdge(new Point3D(aPoints.Pnt1), new Point3D(aPoints.Pnt2)).InternalEdge());
                    }
                }
            }
        }

        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    private void TMultiply(ref Xyz thePoint, bool VecPartOnly = false)
    {
        TMultiply(ref thePoint.ChangeCoord(1), ref thePoint.ChangeCoord(2), ref thePoint.ChangeCoord(3), VecPartOnly);
    }

    private void TTMultiply(ref Xyz thePoint, bool VecPartOnly = false)
    {
        TTMultiply(ref thePoint.ChangeCoord(1), ref thePoint.ChangeCoord(2), ref thePoint.ChangeCoord(3), VecPartOnly);
    }

    private void TIMultiply(ref Xyz thePoint, bool VecPartOnly = false)
    {
        TIMultiply(ref thePoint.ChangeCoord(1), ref thePoint.ChangeCoord(2), ref thePoint.ChangeCoord(3), VecPartOnly);
    }

    private bool AverageNormal(int theINode, ref HLRAlgo_PolyInternalNode.NodeIndices theNodeIndices,
                               NCollection_Array1<HLRAlgo_TriangleData> theTData,
                               NCollection_Array1<HLRAlgo_PolyInternalSegment> thePISeg,
                               NCollection_Array1<HLRAlgo_PolyInternalNode?> thePINod, ref Xyz theNormal)
    {
        return AverageNormal(theINode, ref theNodeIndices, theTData, thePISeg, thePINod, ref theNormal.ChangeCoord(1),
                             ref theNormal.ChangeCoord(2), ref theNormal.ChangeCoord(3));
    }
}
