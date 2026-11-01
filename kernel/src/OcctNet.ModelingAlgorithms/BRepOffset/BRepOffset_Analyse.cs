// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Analyse.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Analyse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiDS_TypeOfConcavity;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Analyses a shape to find the parts of its edges connecting convex, concave or tangent faces.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): the lists it returns are its own, as OCCT's const references are. It never writes
/// into the analysed shape itself, and its results hold only the shape's own sub-shapes, but for the tangent faces'
/// treatment below.
/// </para>
/// <para>
/// The treatment of tangent faces with different offset values (<c>TreatTangentFaces</c>) runs only with a face-offset
/// map, which only the 3D offset sets: <see cref="SetFaceOffsetMap"/> and <see cref="SetOffsetValue"/> are internal.
/// It builds prisms on the tangent edges (<see cref="NewFaces"/>, <see cref="Generated"/>, <see cref="EdgeReplacement"/>),
/// whose sweep writes pcurves into those edges, as OCCT's does; BRepOffset_MakeOffset analyses its own copy of the
/// caller's shape (the offset family's copy at entry, unit 7E), so the edges written are the copy's. On a frozen shape
/// BRepPrimAPI_MakePrism sweeps a copy of the edges instead (its own copy at entry), so the shape is never written, but
/// the new faces then hold copies of the tangent edges where OCCT's hold the edges themselves.
/// </para>
/// </remarks>
public sealed class BRepOffset_Analyse
{
    // Inputs
    private TopoDS_Shape myShape = new(); // Input shape to analyze
    private double myAngle;               // Criteria angle to check tangency

    private double myOffset; // Offset value
    // Map to store offset values for the faces. Should be set by the calling algorithm.
    private NCollection_DataMap<TopoDS_Shape, double> myFaceOffsetMap = new(TopTools_ShapeMapHasher.Instance);

    // Results
    private bool myDone; // Status of the algorithm

    // Map containing the list of intervals on the edge
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<BRepOffset_Interval>> myMapEdgeType = new(TopTools_ShapeMapHasher.Instance);
    // Ancestors map
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myAncestors = new(TopTools_ShapeMapHasher.Instance);
    // Replacement of an edge in the face
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>> myReplacement = new(TopTools_ShapeMapHasher.Instance);
    // Map of shapes descendants built on the base of Ancestors map. Filled on the first query.
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myDescendants = new(TopTools_ShapeMapHasher.Instance);

    // New faces generated to close the gaps between adjacent tangential faces having different offset values
    private readonly NCollection_List<TopoDS_Shape> myNewFaces = new();
    // Binding between edge and face generated from the edge
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myGenerated = new(TopTools_ShapeMapHasher.Instance);

    private static void CorrectOrientationOfTangent(ref gp_Vec TangVec, in TopoDS_Vertex aVertex, in TopoDS_Edge anEdge)
    {
        TopoDS_Vertex Vlast = TopExp.LastVertex(anEdge);
        if (aVertex.IsSame(Vlast))
        {
            TangVec.Reverse();
        }
    }

    //=================================================================================================

    /// <summary>An empty analysis.</summary>
    public BRepOffset_Analyse()
    {
        myOffset = 0.0;
        myDone = false;
    }

    //=================================================================================================

    /// <summary>Analyses <paramref name="theS"/> with the tangency angle <paramref name="theAngle"/>.</summary>
    public BRepOffset_Analyse(in TopoDS_Shape theS, double theAngle)
    {
        myOffset = 0.0;
        myDone = false;
        Perform(theS, theAngle);
    }

    //=================================================================================================

    private static void EdgeAnalyse(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2, double SinTol,
                                    NCollection_List<BRepOffset_Interval> LI)
    {
        BRep_Tool.Range(E, F1, out double f, out double l);
        BRepOffset_Interval I = new();
        I.First(f);
        I.Last(l);
        //
        BRepAdaptor_Surface aBAsurf1 = new(F1, false);
        GeomAbs_SurfaceType aSurfType1 = aBAsurf1.GetType();

        BRepAdaptor_Surface aBAsurf2 = new(F2, false);
        GeomAbs_SurfaceType aSurfType2 = aBAsurf2.GetType();

        bool isTwoPlanes = (aSurfType1 == GeomAbs_Plane && aSurfType2 == GeomAbs_Plane);

        ChFiDS_TypeOfConcavity ConnectType = ChFiDS_Other;

        if (isTwoPlanes) // then use only strong condition
        {
            if (BRep_Tool.Continuity(E, F1, F2) > GeomAbs_C0)
            {
                ConnectType = ChFiDS_Tangential;
            }
            else
            {
                ConnectType = ChFi3d.DefineConnectType(E, F1, F2, SinTol, false);
            }
        }
        else
        {
            bool isTwoSplines =
                (aSurfType1 == GeomAbs_BSplineSurface || aSurfType1 == GeomAbs_BezierSurface)
                && (aSurfType2 == GeomAbs_BSplineSurface || aSurfType2 == GeomAbs_BezierSurface);
            bool isMixedConcavity = false;
            if (isTwoSplines)
            {
                double anAngTol = 0.1;
                isMixedConcavity = CheckMixedContinuity(E, F1, F2, anAngTol);
            }

            if (!isMixedConcavity)
            {
                if (ChFi3d.IsTangentFaces(E, F1, F2)) // weak condition
                {
                    ConnectType = ChFiDS_Tangential;
                }
                else
                {
                    ConnectType = ChFi3d.DefineConnectType(E, F1, F2, SinTol, false);
                }
            }
            else
            {
                ConnectType = ChFiDS_Mixed;
            }
        }

        I.Type(ConnectType);
        LI.Append(I);
    }

    //=================================================================================================

    private static bool CheckMixedContinuity(in TopoDS_Edge theEdge, in TopoDS_Face theFace1, in TopoDS_Face theFace2,
                                             double theAngTol)
    {
        bool aMixedCont = false;
        GeomAbs_Shape aCurrOrder = BRep_Tool.Continuity(theEdge, theFace1, theFace2);
        if (aCurrOrder > GeomAbs_C0)
        {
            // Method BRep_Tool::Continuity(...) always returns minimal continuity between faces
            // so, if aCurrOrder > C0 it means that faces are tangent along whole edge.
            return aMixedCont;
        }
        // But we cannot trust result, if it is C0, because this value is set by default.
        double TolC0 = Max(0.001, 1.5 * BRep_Tool.Tolerance(theEdge));

        double aFirst;
        double aLast;

        Geom2d_Curve? aC2d1, aC2d2;

        if (!theFace1.IsSame(theFace2) && BRep_Tool.IsClosed(theEdge, theFace1)
            && BRep_Tool.IsClosed(theEdge, theFace2))
        {
            // Find the edge in the face 1: this edge will have correct orientation
            TopoDS_Edge anEdgeInFace1 = new();
            TopoDS_Face aFace1 = theFace1;
            aFace1.Orientation(TopAbs_FORWARD);
            TopExp_Explorer anExplo = new(aFace1, TopAbs_EDGE);
            for (; anExplo.More(); anExplo.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anExplo.Current());
                if (anEdge.IsSame(theEdge))
                {
                    anEdgeInFace1 = anEdge;
                    break;
                }
            }
            if (anEdgeInFace1.IsNull())
            {
                return aMixedCont;
            }

            aC2d1 = BRep_Tool.CurveOnSurface(anEdgeInFace1, aFace1, out aFirst, out aLast);
            TopoDS_Face aFace2 = theFace2;
            aFace2.Orientation(TopAbs_FORWARD);
            anEdgeInFace1.Reverse();
            aC2d2 = BRep_Tool.CurveOnSurface(anEdgeInFace1, aFace2, out aFirst, out aLast);
        }
        else
        {
            // Obtaining of pcurves of edge on two faces.
            aC2d1 = BRep_Tool.CurveOnSurface(theEdge, theFace1, out aFirst, out aLast);
            // For the case of seam edge
            TopoDS_Edge EE = theEdge;
            if (theFace1.IsSame(theFace2))
            {
                EE.Reverse();
            }
            aC2d2 = BRep_Tool.CurveOnSurface(EE, theFace2, out aFirst, out aLast);
        }

        if (aC2d1 is null || aC2d2 is null)
        {
            return aMixedCont;
        }

        // Obtaining of two surfaces from adjacent faces.
        Geom_Surface? aSurf1 = BRep_Tool.Surface(theFace1);
        Geom_Surface? aSurf2 = BRep_Tool.Surface(theFace2);

        if (aSurf1 is null || aSurf2 is null)
        {
            return aMixedCont;
        }

        int aNbSamples = 23;

        // Check for mixed concavity: convex in some regions, concave in others.
        double aDelta = (aLast - aFirst) / (aNbSamples - 1);
        bool aHasConvex = false;
        bool aHasConcave = false;
        int aNbValid = 0;

        for (int i = 1; i <= aNbSamples; i++)
        {
            double aPar = (i == aNbSamples) ? aLast : aFirst + (i - 1) * aDelta;

            LocalAnalysis_SurfaceContinuity aCont = new(aC2d1, aC2d2, aPar, aSurf1, aSurf2, GeomAbs_G1, 0.001, TolC0,
                                                        theAngTol, theAngTol, theAngTol);
            if (!aCont.IsDone())
            {
                continue;
            }

            aNbValid++;

            if (!aCont.IsG1() && (!aHasConvex || !aHasConcave))
            {
                double anAngle = aCont.C0Value();
                aHasConvex = aHasConvex || (anAngle > M_PI_2 + theAngTol);
                aHasConcave = aHasConcave || (anAngle < M_PI_2 - theAngTol);
            }
        }

        if (aNbValid < aNbSamples / 2)
        {
            return aMixedCont;
        }

        // Mixed connectivity: both convex and concave regions exist.
        aMixedCont = aHasConvex && aHasConcave;

        return aMixedCont;
    }

    //=================================================================================================

    private static void BuildAncestors(in TopoDS_Shape S, NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MA)
    {
        MA.Clear();
        TopExp.MapShapesAndUniqueAncestors(S, TopAbs_VERTEX, TopAbs_EDGE, MA);
        TopExp.MapShapesAndUniqueAncestors(S, TopAbs_EDGE, TopAbs_FACE, MA);
    }

    //=================================================================================================

    /// <summary>Performs the analysis of <paramref name="theS"/> with the tangency angle <paramref name="theAngle"/>.</summary>
    public void Perform(in TopoDS_Shape theS, double theAngle, Message_ProgressRange theRange = default)
    {
        TopoDS_Shape S = theS;
        double Angle = theAngle;
        myShape = S;
        myNewFaces.Clear();
        myGenerated.Clear();
        myReplacement.Clear();
        myDescendants.Clear();

        myAngle = Angle;
        double SinTol = Math.Abs(Math.Sin(Angle));

        // Build ancestors.
        BuildAncestors(S, myAncestors);

        NCollection_List<TopoDS_Shape> aLETang = new();
        TopExp_Explorer Exp = new(S.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
        using Message_ProgressScope aPSOuter = new(theRange, null, 2);
        using Message_ProgressScope aPS = new(aPSOuter.Next(), "Performing edges analysis", 1, true);
        for (; Exp.More(); Exp.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Edge E = TopoDS.Edge(Exp.Current());
            if (!myMapEdgeType.IsBound(E))
            {
                NCollection_List<BRepOffset_Interval> LI = new();
                myMapEdgeType.Bind(E, LI);

                NCollection_List<TopoDS_Shape> L = Ancestors(E);
                if (L.IsEmpty())
                {
                    continue;
                }

                if (L.Extent() == 2)
                {
                    TopoDS_Face F1 = TopoDS.Face(L.First());
                    TopoDS_Face F2 = TopoDS.Face(L.Last());
                    EdgeAnalyse(E, F1, F2, SinTol, myMapEdgeType[E]);

                    // For tangent faces add artificial perpendicular face
                    // to close the gap between them (if they have different offset values)
                    if (myMapEdgeType[E].Last().Type() == ChFiDS_Tangential)
                    {
                        aLETang.Append(E);
                    }
                }
                else if (L.Extent() == 1)
                {
                    TopoDS_Face F = TopoDS.Face(L.First());
                    BRep_Tool.Range(E, F, out double U1, out double U2);
                    BRepOffset_Interval Inter = new(U1, U2, ChFiDS_Other);

                    if (!BRepTools.IsReallyClosed(E, F))
                    {
                        Inter.Type(ChFiDS_FreeBound);
                    }
                    myMapEdgeType[E].Append(Inter);
                }
                else
                {
                    // edge shared by more than two faces (OCCT_DEBUG trace left out)
                }
            }
        }

        TreatTangentFaces(aLETang, aPSOuter.Next());
        if (!aPSOuter.More())
        {
            return;
        }
        myDone = true;
    }

    //=================================================================================================

    // Treatment of the tangential cases: theLE are the edges connecting tangent faces
    private void TreatTangentFaces(NCollection_List<TopoDS_Shape> theLE, Message_ProgressRange theRange)
    {
        if (theLE.IsEmpty() || myFaceOffsetMap.IsEmpty())
        {
            // Noting to do: either there are no tangent faces in the shape or
            //               the face offset map has not been provided
            return;
        }

        // Select the edges which connect faces with different offset values
        TopoDS_Compound aCETangent = new();
        new BRep_Builder().MakeCompound(ref aCETangent);
        // Bind to each tangent edge a max offset value of its faces
        NCollection_DataMap<TopoDS_Shape, double> anEdgeOffsetMap = new(TopTools_ShapeMapHasher.Instance);
        // Bind vertices of the tangent edges with connected edges
        // of the face with smaller offset value
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aDMVEMin = new(TopTools_ShapeMapHasher.Instance);
        using Message_ProgressScope aPSOuter = new(theRange, null, 3);
        using Message_ProgressScope aPS1 = new(aPSOuter.Next(), "Binding vertices with connected edges", theLE.Extent());
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(theLE); it.More(); it.Next(), aPS1.Next())
        {
            if (!aPS1.More())
            {
                return;
            }
            TopoDS_Shape aE = it.Value();
            NCollection_List<TopoDS_Shape> aLA = Ancestors(aE);

            TopoDS_Shape aF1 = aLA.First(), aF2 = aLA.Last();

            bool pOffsetVal1 = myFaceOffsetMap.Find(aF1, out double anOffsetFound1);
            bool pOffsetVal2 = myFaceOffsetMap.Find(aF2, out double anOffsetFound2);
            double anOffsetVal1 = pOffsetVal1 ? Math.Abs(anOffsetFound1) : myOffset;
            double anOffsetVal2 = pOffsetVal2 ? Math.Abs(anOffsetFound2) : myOffset;
            if (anOffsetVal1 != anOffsetVal2)
            {
                new BRep_Builder().Add(aCETangent, aE);
                anEdgeOffsetMap.Bind(aE, Max(anOffsetVal1, anOffsetVal2));

                TopoDS_Shape aFMin = anOffsetVal1 < anOffsetVal2 ? aF1 : aF2;
                for (TopoDS_Iterator itV = new(aE); itV.More(); itV.Next())
                {
                    TopoDS_Shape aV = itV.Value();
                    if (Ancestors(aV).Extent() == 3)
                    {
                        for (TopExp_Explorer expE = new(aFMin, TopAbs_EDGE); expE.More(); expE.Next())
                        {
                            TopoDS_Shape aEMin = expE.Current();
                            if (aEMin.IsSame(aE))
                            {
                                continue;
                            }
                            for (TopoDS_Iterator itV1 = new(aEMin); itV1.More(); itV1.Next())
                            {
                                TopoDS_Shape aVx = itV1.Value();
                                if (aV.IsSame(aVx))
                                {
                                    aDMVEMin.Bind(aV, aEMin);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (anEdgeOffsetMap.IsEmpty())
        {
            return;
        }

        // Create map of Face ancestors for the vertices on tangent edges
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVFAnc = new(TopTools_ShapeMapHasher.Instance);

        using Message_ProgressScope aPS2 = new(aPSOuter.Next(), "Creating map of Face ancestors", theLE.Extent());
        for (NCollection_List<TopoDS_Shape>.Iterator itE = new(theLE); itE.More(); itE.Next(), aPS2.Next())
        {
            if (!aPS2.More())
            {
                return;
            }
            TopoDS_Shape aE = itE.Value();
            if (!anEdgeOffsetMap.IsBound(aE))
            {
                continue;
            }

            NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
            {
                NCollection_List<TopoDS_Shape> aLEA = Ancestors(aE);
                for (NCollection_List<TopoDS_Shape>.Iterator itLEA = new(aLEA); itLEA.More(); itLEA.Next())
                {
                    aMFence.Add(itLEA.Value());
                }
            }

            for (TopoDS_Iterator itV = new(aE); itV.More(); itV.Next())
            {
                TopoDS_Shape aV = itV.Value();
                NCollection_List<TopoDS_Shape> pLFA = aDMVFAnc.Bound(aV, new NCollection_List<TopoDS_Shape>());
                NCollection_List<TopoDS_Shape> aLVA = Ancestors(aV);
                for (NCollection_List<TopoDS_Shape>.Iterator itLVA = new(aLVA); itLVA.More(); itLVA.Next())
                {
                    TopoDS_Edge aEA = TopoDS.Edge(itLVA.Value());
                    if (!myMapEdgeType.Find(aEA, out NCollection_List<BRepOffset_Interval> pIntervals)
                        || pIntervals.IsEmpty())
                    {
                        continue;
                    }
                    if (pIntervals.First().Type() == ChFiDS_Tangential)
                    {
                        continue;
                    }

                    NCollection_List<TopoDS_Shape> aLEA = Ancestors(aEA);
                    for (NCollection_List<TopoDS_Shape>.Iterator itLEA = new(aLEA); itLEA.More(); itLEA.Next())
                    {
                        TopoDS_Shape aFA = itLEA.Value();
                        if (aMFence.Add(aFA))
                        {
                            pLFA.Append(aFA);
                        }
                    }
                }
            }
        }

        IntTools_Context aCtx = new();
        // Tangency criteria
        double aSinTol = Math.Abs(Math.Sin(myAngle));

        // Make blocks of connected edges
        NCollection_List<NCollection_List<TopoDS_Shape>> aLCB = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVEMap = new(TopTools_ShapeMapHasher.Instance);

        BOPTools_AlgoTools.MakeConnexityBlocks(aCETangent, TopAbs_VERTEX, TopAbs_EDGE, aLCB, aMVEMap);

        // Analyze each block to find co-planar edges
        using Message_ProgressScope aPS3 = new(aPSOuter.Next(), "Analyzing blocks to find co-planar edges", aLCB.Extent());
        for (NCollection_List<NCollection_List<TopoDS_Shape>>.Iterator itLCB = new(aLCB); itLCB.More();
             itLCB.Next(), aPS3.Next())
        {
            if (!aPS3.More())
            {
                return;
            }
            NCollection_List<TopoDS_Shape> aCB = itLCB.Value();

            NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
            for (NCollection_List<TopoDS_Shape>.Iterator itCB1 = new(aCB); itCB1.More(); itCB1.Next())
            {
                TopoDS_Edge aE1 = TopoDS.Edge(itCB1.Value());
                if (!aMFence.Add(aE1))
                {
                    continue;
                }

                TopoDS_Compound aBlock = new();
                new BRep_Builder().MakeCompound(ref aBlock);
                new BRep_Builder().Add(aBlock, aE1.Oriented(TopAbs_FORWARD));

                double anOffset = anEdgeOffsetMap.Find(aE1);
                NCollection_List<TopoDS_Shape> aLF1 = Ancestors(aE1);

                BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE1, TopoDS.Face(aLF1.First()), out gp_Dir aDN1);

                NCollection_List<TopoDS_Shape>.Iterator itCB2 = new();
                itCB2.Reset(aCB, itCB1.Index); // itCB2 = itCB1
                for (itCB2.Next(); itCB2.More(); itCB2.Next())
                {
                    TopoDS_Edge aE2 = TopoDS.Edge(itCB2.Value());
                    if (aMFence.Contains(aE2))
                    {
                        continue;
                    }

                    NCollection_List<TopoDS_Shape> aLF2 = Ancestors(aE2);

                    BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE2, TopoDS.Face(aLF2.First()), out gp_Dir aDN2);

                    if (aDN1.XYZ().Crossed(aDN2.XYZ()).Modulus() < aSinTol)
                    {
                        new BRep_Builder().Add(aBlock, aE2.Oriented(TopAbs_FORWARD));
                        aMFence.Add(aE2);
                        anOffset = Max(anOffset, anEdgeOffsetMap.Find(aE2));
                    }
                }

                // Make the prism
                BRepPrimAPI_MakePrism aMP = new(aBlock, new gp_Vec(aDN1.XYZ()) * anOffset);
                if (!aMP.IsDone())
                {
                    continue;
                }

                TopoDS_Shape aPrism = aMP.InternalShape();
                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aPrismAncestors = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapesAndAncestors(aPrism, TopAbs_EDGE, TopAbs_FACE, aPrismAncestors);
                TopExp.MapShapesAndAncestors(aPrism, TopAbs_VERTEX, TopAbs_EDGE, aPrismAncestors);

                for (TopoDS_Iterator itE = new(aBlock); itE.More(); itE.Next())
                {
                    TopoDS_Edge aE = TopoDS.Edge(itE.Value());
                    NCollection_List<TopoDS_Shape> aLG = aMP.InternalGenerated(aE);
                    TopoDS_Face aFNew = TopoDS.Face(aLG.First());

                    NCollection_List<TopoDS_Shape> aLA = myAncestors.ChangeFromKey(aE);

                    TopoDS_Shape aF1 = aLA.First();
                    TopoDS_Shape aF2 = aLA.Last();

                    bool pOffsetVal1 = myFaceOffsetMap.Find(aF1, out double anOffsetFound1);
                    bool pOffsetVal2 = myFaceOffsetMap.Find(aF2, out double anOffsetFound2);
                    double anOffsetVal1 = pOffsetVal1 ? Math.Abs(anOffsetFound1) : myOffset;
                    double anOffsetVal2 = pOffsetVal2 ? Math.Abs(anOffsetFound2) : myOffset;

                    TopoDS_Shape aFToRemove = anOffsetVal1 > anOffsetVal2 ? aF1 : aF2;
                    TopoDS_Shape aFOpposite = anOffsetVal1 > anOffsetVal2 ? aF2 : aF1;

                    // Orient the face so its normal is directed to smaller offset face
                    {
                        // get normal of the new face
                        BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE, aFNew, out gp_Dir aDN);

                        // get bi-normal for the aFOpposite
                        TopoDS_Edge aEInF = new();
                        for (TopExp_Explorer aExpE = new(aFOpposite, TopAbs_EDGE); aExpE.More(); aExpE.Next())
                        {
                            if (aE.IsSame(aExpE.Current()))
                            {
                                aEInF = TopoDS.Edge(aExpE.Current());
                                break;
                            }
                        }

                        gp_Pnt2d aP2d = new();
                        gp_Pnt aPInF = new();
                        Geom_Curve aC3D = BRep_Tool.Curve(aEInF, out double f, out double l)!;
                        gp_Pnt aPOnE = aC3D.Value((f + l) / 2.0);
                        BOPTools_AlgoTools3D.PointNearEdge(aEInF, TopoDS.Face(aFOpposite), (f + l) / 2.0, 1.0e-5,
                                                           ref aP2d, ref aPInF);

                        gp_Vec aBN = new(aPOnE, aPInF);

                        if (aBN.Dot(new gp_Vec(aDN)) < 0)
                        {
                            aFNew.Reverse();
                        }
                    }

                    // Remove the face with bigger offset value from edge ancestors
                    for (NCollection_List<TopoDS_Shape>.Iterator itA = new(aLA); itA.More(); itA.Next())
                    {
                        if (itA.Value().IsSame(aFToRemove))
                        {
                            aLA.Remove(itA);
                            break;
                        }
                    }
                    aLA.Append(aFNew);

                    myMapEdgeType[aE].Clear();
                    // Analyze edge again
                    EdgeAnalyse(aE, TopoDS.Face(aFOpposite), aFNew, aSinTol, myMapEdgeType[aE]);

                    // Analyze vertices
                    NCollection_Map<TopoDS_Shape> aFNewEdgeMap = new(TopTools_ShapeMapHasher.Instance);
                    aFNewEdgeMap.Add(aE);
                    for (TopoDS_Iterator itV = new(aE); itV.More(); itV.Next())
                    {
                        TopoDS_Shape aV = itV.Value();
                        // Add Side edge to map of Ancestors with the correct orientation
                        TopoDS_Edge aEG = TopoDS.Edge(aMP.InternalGenerated(aV).First());
                        myGenerated.Bind(aV, aEG);
                        {
                            for (TopExp_Explorer anExpEg = new(aFNew, TopAbs_EDGE); anExpEg.More(); anExpEg.Next())
                            {
                                if (anExpEg.Current().IsSame(aEG))
                                {
                                    aEG = TopoDS.Edge(anExpEg.Current());
                                    break;
                                }
                            }
                        }

                        if (aDMVEMin.IsBound(aV))
                        {
                            if (aDMVFAnc.Find(aV, out NCollection_List<TopoDS_Shape> pSA) && pSA.Extent() == 1)
                            {
                                // Adjust orientation of generated edge to its new ancestor
                                TopoDS_Edge aEMin = TopoDS.Edge(aDMVEMin.Find(aV));
                                for (TopExp_Explorer expEx = new(pSA.First(), TopAbs_EDGE); expEx.More(); expEx.Next())
                                {
                                    if (expEx.Current().IsSame(aEMin))
                                    {
                                        aEMin = TopoDS.Edge(expEx.Current());
                                        break;
                                    }
                                }

                                TopAbs_Orientation anOriInEMin = TopAbs_FORWARD, anOriInEG = TopAbs_FORWARD;

                                for (TopoDS_Iterator itx = new(aEMin); itx.More(); itx.Next())
                                {
                                    if (itx.Value().IsSame(aV))
                                    {
                                        anOriInEMin = itx.Value().Orientation();
                                        break;
                                    }
                                }

                                for (TopoDS_Iterator itx = new(aEG); itx.More(); itx.Next())
                                {
                                    if (itx.Value().IsSame(aV))
                                    {
                                        anOriInEG = itx.Value().Orientation();
                                        break;
                                    }
                                }

                                if (anOriInEG == anOriInEMin)
                                {
                                    aEG.Reverse();
                                }
                            }
                        }

                        NCollection_List<TopoDS_Shape> aLVA = myAncestors.ChangeFromKey(aV);
                        if (!aLVA.Contains(aEG))
                        {
                            aLVA.Append(aEG);
                        }
                        aFNewEdgeMap.Add(aEG);

                        NCollection_List<TopoDS_Shape> aLEGA =
                            myAncestors[myAncestors.Add(aEG, new NCollection_List<TopoDS_Shape>(aPrismAncestors.FindFromKey(aEG)))];
                        {
                            // Add ancestors from the shape
                            if (aDMVFAnc.Find(aV, out NCollection_List<TopoDS_Shape> pSA) && !pSA.IsEmpty())
                            {
                                NCollection_List<TopoDS_Shape> aLSA = new(pSA);
                                aLEGA.Append(aLSA);
                            }
                        }

                        myMapEdgeType.Bind(aEG, new NCollection_List<BRepOffset_Interval>());
                        if (aLEGA.Extent() == 2)
                        {
                            EdgeAnalyse(aEG, TopoDS.Face(aLEGA.First()), TopoDS.Face(aLEGA.Last()), aSinTol,
                                        myMapEdgeType[aEG]);
                        }
                    }

                    // Find an edge opposite to tangential one and add ancestors for it
                    TopoDS_Edge aEOpposite = new();
                    for (TopExp_Explorer anExpE = new(aFNew, TopAbs_EDGE); anExpE.More(); anExpE.Next())
                    {
                        if (!aFNewEdgeMap.Contains(anExpE.Current()))
                        {
                            aEOpposite = TopoDS.Edge(anExpE.Current());
                            break;
                        }
                    }

                    {
                        // Find it in aFOpposite
                        for (TopExp_Explorer anExpE = new(aFToRemove, TopAbs_EDGE); anExpE.More(); anExpE.Next())
                        {
                            TopoDS_Shape aEInFToRem = anExpE.Current();
                            if (aE.IsSame(aEInFToRem))
                            {
                                if (BOPTools_AlgoTools.IsSplitToReverse(aEOpposite, aEInFToRem, aCtx))
                                {
                                    aEOpposite.Reverse();
                                }
                                break;
                            }
                        }
                    }

                    NCollection_List<TopoDS_Shape> aLFOpposite = new();
                    aLFOpposite.Append(aFNew);
                    aLFOpposite.Append(aFToRemove);
                    myAncestors.Add(aEOpposite, aLFOpposite);
                    myMapEdgeType.Bind(aEOpposite, new NCollection_List<BRepOffset_Interval>());
                    EdgeAnalyse(aEOpposite, aFNew, TopoDS.Face(aFToRemove), aSinTol, myMapEdgeType[aEOpposite]);

                    if (!myReplacement.Find(aFToRemove, out NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> pEEMap))
                    {
                        pEEMap = myReplacement.Bound(aFToRemove,
                                                     new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance));
                    }
                    pEEMap.Bind(aE, aEOpposite);

                    // Add ancestors for the vertices
                    for (TopoDS_Iterator itV = new(aEOpposite); itV.More(); itV.Next())
                    {
                        TopoDS_Shape aV = itV.Value();
                        NCollection_List<TopoDS_Shape> aLVA = aPrismAncestors.FindFromKey(aV);
                        myAncestors.Add(aV, new NCollection_List<TopoDS_Shape>(aLVA));
                    }

                    myNewFaces.Append(aFNew);
                    myGenerated.Bind(aE, aFNew);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>True if the analysis is done.</summary>
    public bool IsDone() => myDone;

    /// <summary>
    /// The replacement of the edge <paramref name="theEdge"/> in the face <paramref name="theFace"/>: the edge itself
    /// if it has none.
    /// </summary>
    public TopoDS_Edge EdgeReplacement(in TopoDS_Face theFace, in TopoDS_Edge theEdge)
    {
        if (!myReplacement.Find(theFace, out NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> pEE))
        {
            return theEdge;
        }

        if (!pEE.Find(theEdge, out TopoDS_Shape pE))
        {
            return theEdge;
        }

        return TopoDS.Edge(pE);
    }

    //=================================================================================================

    /// <summary>
    /// The face generated for the edge connecting two tangent faces with different offset values, or the edge
    /// generated from a vertex of such an edge; a null shape if none.
    /// </summary>
    public TopoDS_Shape Generated(in TopoDS_Shape theS)
    {
        return myGenerated.Find(theS, out TopoDS_Shape pGenS) ? pGenS : new TopoDS_Shape();
    }

    /// <summary>True if the shape has generated a new face (or edge).</summary>
    public bool HasGenerated(in TopoDS_Shape theS) => myGenerated.IsBound(theS);

    /// <summary>
    /// The new faces built between tangent faces with different offset values (the 3D offset's per-face values).
    /// </summary>
    public NCollection_List<TopoDS_Shape> NewFaces() => myNewFaces;

    //=================================================================================================

    /// <summary>
    /// The shape's descendants (the inverse of <see cref="Ancestors"/>), or null if it has none; the map is built on the
    /// first query, and again with <paramref name="theUpdate"/>.
    /// </summary>
    public NCollection_List<TopoDS_Shape>? Descendants(in TopoDS_Shape theS, bool theUpdate = false)
    {
        if (myDescendants.IsEmpty() || theUpdate)
        {
            myDescendants.Clear();
            int aNbA = myAncestors.Extent();
            for (int i = 1; i <= aNbA; ++i)
            {
                TopoDS_Shape aSS = myAncestors.FindKey(i);
                NCollection_List<TopoDS_Shape> aLA = myAncestors[i];

                for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLA); it.More(); it.Next())
                {
                    TopoDS_Shape aSA = it.Value();

                    if (!myDescendants.Find(aSA, out NCollection_List<TopoDS_Shape> pLD))
                    {
                        pLD = myDescendants.Bound(aSA, new NCollection_List<TopoDS_Shape>());
                    }
                    if (!pLD.Contains(aSS))
                    {
                        pLD.Append(aSS);
                    }
                }
            }
        }

        return myDescendants.Find(theS, out NCollection_List<TopoDS_Shape> aLD) ? aLD : null;
    }

    //=================================================================================================

    /// <summary>Clears the content of the algorithm.</summary>
    public void Clear()
    {
        myDone = false;
        myShape = new TopoDS_Shape();
        myMapEdgeType.Clear();
        myAncestors.Clear();
        myFaceOffsetMap.Clear();
        myReplacement.Clear();
        myDescendants.Clear();
        myNewFaces.Clear();
        myGenerated.Clear();
    }

    //=======================================================================
    // function : NCollection_List<BRepOffset_Interval>&
    // purpose  :
    //=======================================================================

    /// <summary>The connectivity type of the edge: its intervals (throws if the edge wasn't analysed).</summary>
    public NCollection_List<BRepOffset_Interval> Type(in TopoDS_Edge E)
    {
        return myMapEdgeType.Find(E);
    }

    //=================================================================================================

    /// <summary>Stores in <paramref name="LE"/> all the edges of type <paramref name="T"/> on the vertex <paramref name="V"/>.</summary>
    public void Edges(in TopoDS_Vertex V, ChFiDS_TypeOfConcavity T, NCollection_List<TopoDS_Shape> LE)
    {
        LE.Clear();
        NCollection_List<TopoDS_Shape> L = Ancestors(V);
        NCollection_List<TopoDS_Shape>.Iterator it = new(L);

        for (; it.More(); it.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(it.Value());
            if (myMapEdgeType.Find(E, out NCollection_List<BRepOffset_Interval> pIntervals) && pIntervals.Extent() > 0)
            {
                BRepOffset_Tool.EdgeVertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (V1.IsSame(V))
                {
                    if (pIntervals.Last().Type() == T)
                    {
                        LE.Append(E);
                    }
                }
                if (V2.IsSame(V))
                {
                    if (pIntervals.First().Type() == T)
                    {
                        LE.Append(E);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Stores in <paramref name="LE"/> all the edges of type <paramref name="T"/> on the face <paramref name="F"/>.</summary>
    public void Edges(in TopoDS_Face F, ChFiDS_TypeOfConcavity T, NCollection_List<TopoDS_Shape> LE)
    {
        LE.Clear();
        TopExp_Explorer exp = new(F, TopAbs_EDGE);

        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(exp.Current());

            NCollection_List<BRepOffset_Interval> Lint = Type(E);
            NCollection_List<BRepOffset_Interval>.Iterator it = new(Lint);
            for (; it.More(); it.Next())
            {
                if (it.Value().Type() == T)
                {
                    LE.Append(E);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Sets in <paramref name="Edges"/> all the edges of the shape tangent to <paramref name="Edge"/> at the vertex
    /// <paramref name="Vertex"/>.
    /// </summary>
    public void TangentEdges(in TopoDS_Edge Edge, in TopoDS_Vertex Vertex, NCollection_List<TopoDS_Shape> Edges)
    {
        gp_Vec V, VRef;

        double U, URef;
        BRepAdaptor_Curve C3d, C3dRef;

        URef = BRep_Tool.Parameter(Vertex, Edge);
        C3dRef = new BRepAdaptor_Curve(Edge);
        VRef = C3dRef.DN(URef, 1);
        CorrectOrientationOfTangent(ref VRef, Vertex, Edge);
        if (VRef.SquareMagnitude() < gp.Resolution())
        {
            return;
        }

        Edges.Clear();

        NCollection_List<TopoDS_Shape> Anc = Ancestors(Vertex);
        NCollection_List<TopoDS_Shape>.Iterator it = new(Anc);
        for (; it.More(); it.Next())
        {
            TopoDS_Edge CurE = TopoDS.Edge(it.Value());
            if (CurE.IsSame(Edge))
            {
                continue;
            }
            U = BRep_Tool.Parameter(Vertex, CurE);
            C3d = new BRepAdaptor_Curve(CurE);
            V = C3d.DN(U, 1);
            CorrectOrientationOfTangent(ref V, Vertex, CurE);
            if (V.SquareMagnitude() < gp.Resolution())
            {
                continue;
            }
            if (V.IsOpposite(VRef, myAngle))
            {
                Edges.Append(CurE);
            }
        }
    }

    /// <summary>True if the shape has ancestors.</summary>
    public bool HasAncestor(in TopoDS_Shape theS) => myAncestors.Contains(theS);

    /// <summary>The ancestors of the shape (throws if it has none).</summary>
    public NCollection_List<TopoDS_Shape> Ancestors(in TopoDS_Shape theS) => myAncestors.FindFromKey(theS);

    //=================================================================================================

    /// <summary>
    /// Explodes the shape into compounds of faces where all the connecting edges are of type <paramref name="T"/>.
    /// </summary>
    public void Explode(NCollection_List<TopoDS_Shape> List, ChFiDS_TypeOfConcavity T)
    {
        List.Clear();
        BRep_Builder B = new();
        NCollection_Map<TopoDS_Shape> Map = new(TopTools_ShapeMapHasher.Instance);

        TopExp_Explorer Fexp = new();
        for (Fexp.Init(myShape, TopAbs_FACE); Fexp.More(); Fexp.Next())
        {
            if (Map.Add(Fexp.Current()))
            {
                TopoDS_Face Face = TopoDS.Face(Fexp.Current());
                TopoDS_Compound Co = new();
                B.MakeCompound(ref Co);
                B.Add(Co, Face);
                // add to Co all faces from the cloud of faces
                // G1 created from <Face>
                AddFaces(Face, Co, Map, T);
                List.Append(Co);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Explodes the shape into compounds of faces where all the connecting edges are of type <paramref name="T1"/> or
    /// <paramref name="T2"/>.
    /// </summary>
    public void Explode(NCollection_List<TopoDS_Shape> List, ChFiDS_TypeOfConcavity T1, ChFiDS_TypeOfConcavity T2)
    {
        List.Clear();
        BRep_Builder B = new();
        NCollection_Map<TopoDS_Shape> Map = new(TopTools_ShapeMapHasher.Instance);

        TopExp_Explorer Fexp = new();
        for (Fexp.Init(myShape, TopAbs_FACE); Fexp.More(); Fexp.Next())
        {
            if (Map.Add(Fexp.Current()))
            {
                TopoDS_Face Face = TopoDS.Face(Fexp.Current());
                TopoDS_Compound Co = new();
                B.MakeCompound(ref Co);
                B.Add(Co, Face);
                // add to Co all faces from the cloud of faces
                // G1 created from  <Face>
                AddFaces(Face, Co, Map, T1, T2);
                List.Append(Co);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds to <paramref name="Co"/> the faces of the shell holding <paramref name="Face"/> connected to it through
    /// edges of type <paramref name="T"/>.
    /// </summary>
    public void AddFaces(in TopoDS_Face Face, in TopoDS_Compound Co, NCollection_Map<TopoDS_Shape> Map,
                         ChFiDS_TypeOfConcavity T)
    {
        BRep_Builder B = new();
        NCollection_List<TopoDS_Shape>? pLE = Descendants(Face);
        if (pLE is null)
        {
            return;
        }
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(pLE); it.More(); it.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(it.Value());
            NCollection_List<BRepOffset_Interval> LI = Type(E);
            if (!LI.IsEmpty() && LI.First().Type() == T)
            {
                // so <NewFace> is attached to G1 by <Face>
                NCollection_List<TopoDS_Shape> L = Ancestors(E);
                if (L.Extent() == 2)
                {
                    TopoDS_Face F1 = TopoDS.Face(L.First());
                    if (F1.IsSame(Face))
                    {
                        F1 = TopoDS.Face(L.Last());
                    }
                    if (Map.Add(F1))
                    {
                        B.Add(Co, F1);
                        AddFaces(F1, Co, Map, T);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds to <paramref name="Co"/> the faces of the shell holding <paramref name="Face"/> connected to it through
    /// edges of type <paramref name="T1"/> or <paramref name="T2"/>.
    /// </summary>
    public void AddFaces(in TopoDS_Face Face, in TopoDS_Compound Co, NCollection_Map<TopoDS_Shape> Map,
                         ChFiDS_TypeOfConcavity T1, ChFiDS_TypeOfConcavity T2)
    {
        BRep_Builder B = new();
        NCollection_List<TopoDS_Shape>? pLE = Descendants(Face);
        if (pLE is null)
        {
            return;
        }
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(pLE); it.More(); it.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(it.Value());
            NCollection_List<BRepOffset_Interval> LI = Type(E);
            if (!LI.IsEmpty() && (LI.First().Type() == T1 || LI.First().Type() == T2))
            {
                // so <NewFace> is attached to G1 by <Face>
                NCollection_List<TopoDS_Shape> L = Ancestors(E);
                if (L.Extent() == 2)
                {
                    TopoDS_Face F1 = TopoDS.Face(L.First());
                    if (F1.IsSame(Face))
                    {
                        F1 = TopoDS.Face(L.Last());
                    }
                    if (Map.Add(F1))
                    {
                        B.Add(Co, F1);
                        AddFaces(F1, Co, Map, T1, T2);
                    }
                }
            }
        }
    }

    /// <summary>Sets the offset value, for the treatment of tangent faces (the 3D offset only).</summary>
    internal void SetOffsetValue(double theOffset) => myOffset = theOffset;

    /// <summary>
    /// Sets the faces' offset values, to treat tangent faces with different values (the 3D offset only). The map is
    /// copied, as OCCT assigns it.
    /// </summary>
    internal void SetFaceOffsetMap(NCollection_DataMap<TopoDS_Shape, double> theMap)
    {
        myFaceOffsetMap = new NCollection_DataMap<TopoDS_Shape, double>(TopTools_ShapeMapHasher.Instance);
        foreach (KeyValuePair<TopoDS_Shape, double> aPair in theMap)
        {
            myFaceOffsetMap.Bind(aPair.Key, aPair.Value);
        }
    }
}
