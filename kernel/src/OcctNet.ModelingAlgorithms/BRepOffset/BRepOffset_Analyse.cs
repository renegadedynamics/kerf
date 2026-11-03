// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Analyse.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Analyse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFiDS_TypeOfConcavity;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private Shape myShape = new(); // Input shape to analyze
    private double myAngle;               // Criteria angle to check tangency

    private double myOffset; // Offset value
    // Map to store offset values for the faces. Should be set by the calling algorithm.
    private NCollection_DataMap<Shape, double> myFaceOffsetMap = new(ShapeHasher.Instance);

    // Results
    private bool myDone; // Status of the algorithm

    // Map containing the list of intervals on the edge
    private readonly NCollection_DataMap<Shape, NCollection_List<BRepOffset_Interval>> myMapEdgeType = new(ShapeHasher.Instance);
    // Ancestors map
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myAncestors = new(ShapeHasher.Instance);
    // Replacement of an edge in the face
    private readonly NCollection_DataMap<Shape, NCollection_DataMap<Shape, Shape>> myReplacement = new(ShapeHasher.Instance);
    // Map of shapes descendants built on the base of Ancestors map. Filled on the first query.
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myDescendants = new(ShapeHasher.Instance);

    // New faces generated to close the gaps between adjacent tangential faces having different offset values
    private readonly NCollection_List<Shape> myNewFaces = new();
    // Binding between edge and face generated from the edge
    private readonly NCollection_DataMap<Shape, Shape> myGenerated = new(ShapeHasher.Instance);

    private static void CorrectOrientationOfTangent(ref Vector3D TangVec, in Vertex aVertex, in Edge anEdge)
    {
        Vertex Vlast = Exploration.LastVertex(anEdge);
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
    public BRepOffset_Analyse(in Shape theS, double theAngle)
    {
        myOffset = 0.0;
        myDone = false;
        Perform(theS, theAngle);
    }

    //=================================================================================================

    private static void EdgeAnalyse(in Edge E, in Face F1, in Face F2, double SinTol,
                                    NCollection_List<BRepOffset_Interval> LI)
    {
        Tool.Range(E, F1, out double f, out double l);
        BRepOffset_Interval I = new();
        I.First(f);
        I.Last(l);
        //
        FaceSurfaceAdaptor aBAsurf1 = new(F1, false);
        SurfaceKind aSurfType1 = aBAsurf1.GetType();

        FaceSurfaceAdaptor aBAsurf2 = new(F2, false);
        SurfaceKind aSurfType2 = aBAsurf2.GetType();

        bool isTwoPlanes = (aSurfType1 == Plane && aSurfType2 == Plane);

        ChFiDS_TypeOfConcavity ConnectType = ChFiDS_Other;

        if (isTwoPlanes) // then use only strong condition
        {
            if (Tool.Continuity(E, F1, F2) > C0)
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
                (aSurfType1 == SurfaceKind.BSplineSurface || aSurfType1 == SurfaceKind.BezierSurface)
                && (aSurfType2 == SurfaceKind.BSplineSurface || aSurfType2 == SurfaceKind.BezierSurface);
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

    private static bool CheckMixedContinuity(in Edge theEdge, in Face theFace1, in Face theFace2,
                                             double theAngTol)
    {
        bool aMixedCont = false;
        Continuity aCurrOrder = Tool.Continuity(theEdge, theFace1, theFace2);
        if (aCurrOrder > C0)
        {
            // Method BRep_Tool::Continuity(...) always returns minimal continuity between faces
            // so, if aCurrOrder > C0 it means that faces are tangent along whole edge.
            return aMixedCont;
        }
        // But we cannot trust result, if it is C0, because this value is set by default.
        double TolC0 = Max(0.001, 1.5 * Tool.Tolerance(theEdge));

        double aFirst;
        double aLast;

        Curve2D? aC2d1, aC2d2;

        if (!theFace1.IsSame(theFace2) && Tool.IsClosed(theEdge, theFace1)
            && Tool.IsClosed(theEdge, theFace2))
        {
            // Find the edge in the face 1: this edge will have correct orientation
            Edge anEdgeInFace1 = new();
            Face aFace1 = theFace1;
            aFace1.Orientation(Forward);
            Explorer anExplo = new(aFace1, ShapeType.Edge);
            for (; anExplo.More(); anExplo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anExplo.Current());
                if (anEdge.IsSame(theEdge))
                {
                    anEdgeInFace1 = anEdge;
                    break;
                }
            }
            if (anEdgeInFace1.IsNull)
            {
                return aMixedCont;
            }

            aC2d1 = Tool.CurveOnSurface(anEdgeInFace1, aFace1, out aFirst, out aLast);
            Face aFace2 = theFace2;
            aFace2.Orientation(Forward);
            anEdgeInFace1.Reverse();
            aC2d2 = Tool.CurveOnSurface(anEdgeInFace1, aFace2, out aFirst, out aLast);
        }
        else
        {
            // Obtaining of pcurves of edge on two faces.
            aC2d1 = Tool.CurveOnSurface(theEdge, theFace1, out aFirst, out aLast);
            // For the case of seam edge
            Edge EE = theEdge;
            if (theFace1.IsSame(theFace2))
            {
                EE.Reverse();
            }
            aC2d2 = Tool.CurveOnSurface(EE, theFace2, out aFirst, out aLast);
        }

        if (aC2d1 is null || aC2d2 is null)
        {
            return aMixedCont;
        }

        // Obtaining of two surfaces from adjacent faces.
        Surface? aSurf1 = Tool.Surface(theFace1);
        Surface? aSurf2 = Tool.Surface(theFace2);

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

            LocalAnalysis_SurfaceContinuity aCont = new(aC2d1, aC2d2, aPar, aSurf1, aSurf2, G1, 0.001, TolC0,
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

    private static void BuildAncestors(in Shape S, NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> MA)
    {
        MA.Clear();
        Exploration.MapShapesAndUniqueAncestors(S, ShapeType.Vertex, ShapeType.Edge, MA);
        Exploration.MapShapesAndUniqueAncestors(S, ShapeType.Edge, ShapeType.Face, MA);
    }

    //=================================================================================================

    /// <summary>Performs the analysis of <paramref name="theS"/> with the tangency angle <paramref name="theAngle"/>.</summary>
    public void Perform(in Shape theS, double theAngle, ProgressRange theRange = default)
    {
        Shape S = theS;
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

        NCollection_List<Shape> aLETang = new();
        Explorer Exp = new(S.Oriented(Forward), ShapeType.Edge);
        using ProgressScope aPSOuter = new(theRange, null, 2);
        using ProgressScope aPS = new(aPSOuter.Next(), "Performing edges analysis", 1, true);
        for (; Exp.More(); Exp.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            Edge E = ShapeCasts.Edge(Exp.Current());
            if (!myMapEdgeType.IsBound(E))
            {
                NCollection_List<BRepOffset_Interval> LI = new();
                myMapEdgeType.Bind(E, LI);

                NCollection_List<Shape> L = Ancestors(E);
                if (L.IsEmpty())
                {
                    continue;
                }

                if (L.Extent() == 2)
                {
                    Face F1 = ShapeCasts.Face(L.First());
                    Face F2 = ShapeCasts.Face(L.Last());
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
                    Face F = ShapeCasts.Face(L.First());
                    Tool.Range(E, F, out double U1, out double U2);
                    BRepOffset_Interval Inter = new(U1, U2, ChFiDS_Other);

                    if (!BrepTools.IsReallyClosed(E, F))
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
    private void TreatTangentFaces(NCollection_List<Shape> theLE, ProgressRange theRange)
    {
        if (theLE.IsEmpty() || myFaceOffsetMap.IsEmpty())
        {
            // Noting to do: either there are no tangent faces in the shape or
            //               the face offset map has not been provided
            return;
        }

        // Select the edges which connect faces with different offset values
        Compound aCETangent = new();
        new BrepBuilder().MakeCompound(ref aCETangent);
        // Bind to each tangent edge a max offset value of its faces
        NCollection_DataMap<Shape, double> anEdgeOffsetMap = new(ShapeHasher.Instance);
        // Bind vertices of the tangent edges with connected edges
        // of the face with smaller offset value
        NCollection_DataMap<Shape, Shape> aDMVEMin = new(ShapeHasher.Instance);
        using ProgressScope aPSOuter = new(theRange, null, 3);
        using ProgressScope aPS1 = new(aPSOuter.Next(), "Binding vertices with connected edges", theLE.Extent());
        for (NCollection_List<Shape>.Iterator it = new(theLE); it.More(); it.Next(), aPS1.Next())
        {
            if (!aPS1.More())
            {
                return;
            }
            Shape aE = it.Value();
            NCollection_List<Shape> aLA = Ancestors(aE);

            Shape aF1 = aLA.First(), aF2 = aLA.Last();

            bool pOffsetVal1 = myFaceOffsetMap.Find(aF1, out double anOffsetFound1);
            bool pOffsetVal2 = myFaceOffsetMap.Find(aF2, out double anOffsetFound2);
            double anOffsetVal1 = pOffsetVal1 ? Math.Abs(anOffsetFound1) : myOffset;
            double anOffsetVal2 = pOffsetVal2 ? Math.Abs(anOffsetFound2) : myOffset;
            if (anOffsetVal1 != anOffsetVal2)
            {
                new BrepBuilder().Add(aCETangent, aE);
                anEdgeOffsetMap.Bind(aE, Max(anOffsetVal1, anOffsetVal2));

                Shape aFMin = anOffsetVal1 < anOffsetVal2 ? aF1 : aF2;
                for (Iterator itV = new(aE); itV.More(); itV.Next())
                {
                    Shape aV = itV.Value();
                    if (Ancestors(aV).Extent() == 3)
                    {
                        for (Explorer expE = new(aFMin, ShapeType.Edge); expE.More(); expE.Next())
                        {
                            Shape aEMin = expE.Current();
                            if (aEMin.IsSame(aE))
                            {
                                continue;
                            }
                            for (Iterator itV1 = new(aEMin); itV1.More(); itV1.Next())
                            {
                                Shape aVx = itV1.Value();
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
        NCollection_DataMap<Shape, NCollection_List<Shape>> aDMVFAnc = new(ShapeHasher.Instance);

        using ProgressScope aPS2 = new(aPSOuter.Next(), "Creating map of Face ancestors", theLE.Extent());
        for (NCollection_List<Shape>.Iterator itE = new(theLE); itE.More(); itE.Next(), aPS2.Next())
        {
            if (!aPS2.More())
            {
                return;
            }
            Shape aE = itE.Value();
            if (!anEdgeOffsetMap.IsBound(aE))
            {
                continue;
            }

            NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
            {
                NCollection_List<Shape> aLEA = Ancestors(aE);
                for (NCollection_List<Shape>.Iterator itLEA = new(aLEA); itLEA.More(); itLEA.Next())
                {
                    aMFence.Add(itLEA.Value());
                }
            }

            for (Iterator itV = new(aE); itV.More(); itV.Next())
            {
                Shape aV = itV.Value();
                NCollection_List<Shape> pLFA = aDMVFAnc.Bound(aV, new NCollection_List<Shape>());
                NCollection_List<Shape> aLVA = Ancestors(aV);
                for (NCollection_List<Shape>.Iterator itLVA = new(aLVA); itLVA.More(); itLVA.Next())
                {
                    Edge aEA = ShapeCasts.Edge(itLVA.Value());
                    if (!myMapEdgeType.Find(aEA, out NCollection_List<BRepOffset_Interval> pIntervals)
                        || pIntervals.IsEmpty())
                    {
                        continue;
                    }
                    if (pIntervals.First().Type() == ChFiDS_Tangential)
                    {
                        continue;
                    }

                    NCollection_List<Shape> aLEA = Ancestors(aEA);
                    for (NCollection_List<Shape>.Iterator itLEA = new(aLEA); itLEA.More(); itLEA.Next())
                    {
                        Shape aFA = itLEA.Value();
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
        NCollection_List<NCollection_List<Shape>> aLCB = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMVEMap = new(ShapeHasher.Instance);

        BOPTools_AlgoTools.MakeConnexityBlocks(aCETangent, ShapeType.Vertex, ShapeType.Edge, aLCB, aMVEMap);

        // Analyze each block to find co-planar edges
        using ProgressScope aPS3 = new(aPSOuter.Next(), "Analyzing blocks to find co-planar edges", aLCB.Extent());
        for (NCollection_List<NCollection_List<Shape>>.Iterator itLCB = new(aLCB); itLCB.More();
             itLCB.Next(), aPS3.Next())
        {
            if (!aPS3.More())
            {
                return;
            }
            NCollection_List<Shape> aCB = itLCB.Value();

            NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
            for (NCollection_List<Shape>.Iterator itCB1 = new(aCB); itCB1.More(); itCB1.Next())
            {
                Edge aE1 = ShapeCasts.Edge(itCB1.Value());
                if (!aMFence.Add(aE1))
                {
                    continue;
                }

                Compound aBlock = new();
                new BrepBuilder().MakeCompound(ref aBlock);
                new BrepBuilder().Add(aBlock, aE1.Oriented(Forward));

                double anOffset = anEdgeOffsetMap.Find(aE1);
                NCollection_List<Shape> aLF1 = Ancestors(aE1);

                BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE1, ShapeCasts.Face(aLF1.First()), out Direction3D aDN1);

                NCollection_List<Shape>.Iterator itCB2 = new();
                itCB2.Reset(aCB, itCB1.Index); // itCB2 = itCB1
                for (itCB2.Next(); itCB2.More(); itCB2.Next())
                {
                    Edge aE2 = ShapeCasts.Edge(itCB2.Value());
                    if (aMFence.Contains(aE2))
                    {
                        continue;
                    }

                    NCollection_List<Shape> aLF2 = Ancestors(aE2);

                    BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE2, ShapeCasts.Face(aLF2.First()), out Direction3D aDN2);

                    if (aDN1.XYZ().Crossed(aDN2.XYZ()).Modulus() < aSinTol)
                    {
                        new BrepBuilder().Add(aBlock, aE2.Oriented(Forward));
                        aMFence.Add(aE2);
                        anOffset = Max(anOffset, anEdgeOffsetMap.Find(aE2));
                    }
                }

                // Make the prism
                BRepPrimAPI_MakePrism aMP = new(aBlock, new Vector3D(aDN1.XYZ()) * anOffset);
                if (!aMP.IsDone())
                {
                    continue;
                }

                Shape aPrism = aMP.InternalShape();
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aPrismAncestors = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(aPrism, ShapeType.Edge, ShapeType.Face, aPrismAncestors);
                Exploration.MapShapesAndAncestors(aPrism, ShapeType.Vertex, ShapeType.Edge, aPrismAncestors);

                for (Iterator itE = new(aBlock); itE.More(); itE.Next())
                {
                    Edge aE = ShapeCasts.Edge(itE.Value());
                    NCollection_List<Shape> aLG = aMP.InternalGenerated(aE);
                    Face aFNew = ShapeCasts.Face(aLG.First());

                    NCollection_List<Shape> aLA = myAncestors.ChangeFromKey(aE);

                    Shape aF1 = aLA.First();
                    Shape aF2 = aLA.Last();

                    bool pOffsetVal1 = myFaceOffsetMap.Find(aF1, out double anOffsetFound1);
                    bool pOffsetVal2 = myFaceOffsetMap.Find(aF2, out double anOffsetFound2);
                    double anOffsetVal1 = pOffsetVal1 ? Math.Abs(anOffsetFound1) : myOffset;
                    double anOffsetVal2 = pOffsetVal2 ? Math.Abs(anOffsetFound2) : myOffset;

                    Shape aFToRemove = anOffsetVal1 > anOffsetVal2 ? aF1 : aF2;
                    Shape aFOpposite = anOffsetVal1 > anOffsetVal2 ? aF2 : aF1;

                    // Orient the face so its normal is directed to smaller offset face
                    {
                        // get normal of the new face
                        BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE, aFNew, out Direction3D aDN);

                        // get bi-normal for the aFOpposite
                        Edge aEInF = new();
                        for (Explorer aExpE = new(aFOpposite, ShapeType.Edge); aExpE.More(); aExpE.Next())
                        {
                            if (aE.IsSame(aExpE.Current()))
                            {
                                aEInF = ShapeCasts.Edge(aExpE.Current());
                                break;
                            }
                        }

                        Point2D aP2d = new();
                        Point3D aPInF = new();
                        Curve3D aC3D = Tool.Curve(aEInF, out double f, out double l)!;
                        Point3D aPOnE = aC3D.Value((f + l) / 2.0);
                        BOPTools_AlgoTools3D.PointNearEdge(aEInF, ShapeCasts.Face(aFOpposite), (f + l) / 2.0, 1.0e-5,
                                                           ref aP2d, ref aPInF);

                        Vector3D aBN = new(aPOnE, aPInF);

                        if (aBN.Dot(new Vector3D(aDN)) < 0)
                        {
                            aFNew.Reverse();
                        }
                    }

                    // Remove the face with bigger offset value from edge ancestors
                    for (NCollection_List<Shape>.Iterator itA = new(aLA); itA.More(); itA.Next())
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
                    EdgeAnalyse(aE, ShapeCasts.Face(aFOpposite), aFNew, aSinTol, myMapEdgeType[aE]);

                    // Analyze vertices
                    NCollection_Map<Shape> aFNewEdgeMap = new(ShapeHasher.Instance);
                    aFNewEdgeMap.Add(aE);
                    for (Iterator itV = new(aE); itV.More(); itV.Next())
                    {
                        Shape aV = itV.Value();
                        // Add Side edge to map of Ancestors with the correct orientation
                        Edge aEG = ShapeCasts.Edge(aMP.InternalGenerated(aV).First());
                        myGenerated.Bind(aV, aEG);
                        {
                            for (Explorer anExpEg = new(aFNew, ShapeType.Edge); anExpEg.More(); anExpEg.Next())
                            {
                                if (anExpEg.Current().IsSame(aEG))
                                {
                                    aEG = ShapeCasts.Edge(anExpEg.Current());
                                    break;
                                }
                            }
                        }

                        if (aDMVEMin.IsBound(aV))
                        {
                            if (aDMVFAnc.Find(aV, out NCollection_List<Shape> pSA) && pSA.Extent() == 1)
                            {
                                // Adjust orientation of generated edge to its new ancestor
                                Edge aEMin = ShapeCasts.Edge(aDMVEMin.Find(aV));
                                for (Explorer expEx = new(pSA.First(), ShapeType.Edge); expEx.More(); expEx.Next())
                                {
                                    if (expEx.Current().IsSame(aEMin))
                                    {
                                        aEMin = ShapeCasts.Edge(expEx.Current());
                                        break;
                                    }
                                }

                                Orientation anOriInEMin = Forward, anOriInEG = Forward;

                                for (Iterator itx = new(aEMin); itx.More(); itx.Next())
                                {
                                    if (itx.Value().IsSame(aV))
                                    {
                                        anOriInEMin = itx.Value().Orientation();
                                        break;
                                    }
                                }

                                for (Iterator itx = new(aEG); itx.More(); itx.Next())
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

                        NCollection_List<Shape> aLVA = myAncestors.ChangeFromKey(aV);
                        if (!aLVA.Contains(aEG))
                        {
                            aLVA.Append(aEG);
                        }
                        aFNewEdgeMap.Add(aEG);

                        NCollection_List<Shape> aLEGA =
                            myAncestors[myAncestors.Add(aEG, new NCollection_List<Shape>(aPrismAncestors.FindFromKey(aEG)))];
                        {
                            // Add ancestors from the shape
                            if (aDMVFAnc.Find(aV, out NCollection_List<Shape> pSA) && !pSA.IsEmpty())
                            {
                                NCollection_List<Shape> aLSA = new(pSA);
                                aLEGA.Append(aLSA);
                            }
                        }

                        myMapEdgeType.Bind(aEG, new NCollection_List<BRepOffset_Interval>());
                        if (aLEGA.Extent() == 2)
                        {
                            EdgeAnalyse(aEG, ShapeCasts.Face(aLEGA.First()), ShapeCasts.Face(aLEGA.Last()), aSinTol,
                                        myMapEdgeType[aEG]);
                        }
                    }

                    // Find an edge opposite to tangential one and add ancestors for it
                    Edge aEOpposite = new();
                    for (Explorer anExpE = new(aFNew, ShapeType.Edge); anExpE.More(); anExpE.Next())
                    {
                        if (!aFNewEdgeMap.Contains(anExpE.Current()))
                        {
                            aEOpposite = ShapeCasts.Edge(anExpE.Current());
                            break;
                        }
                    }

                    {
                        // Find it in aFOpposite
                        for (Explorer anExpE = new(aFToRemove, ShapeType.Edge); anExpE.More(); anExpE.Next())
                        {
                            Shape aEInFToRem = anExpE.Current();
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

                    NCollection_List<Shape> aLFOpposite = new();
                    aLFOpposite.Append(aFNew);
                    aLFOpposite.Append(aFToRemove);
                    myAncestors.Add(aEOpposite, aLFOpposite);
                    myMapEdgeType.Bind(aEOpposite, new NCollection_List<BRepOffset_Interval>());
                    EdgeAnalyse(aEOpposite, aFNew, ShapeCasts.Face(aFToRemove), aSinTol, myMapEdgeType[aEOpposite]);

                    if (!myReplacement.Find(aFToRemove, out NCollection_DataMap<Shape, Shape> pEEMap))
                    {
                        pEEMap = myReplacement.Bound(aFToRemove,
                                                     new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance));
                    }
                    pEEMap.Bind(aE, aEOpposite);

                    // Add ancestors for the vertices
                    for (Iterator itV = new(aEOpposite); itV.More(); itV.Next())
                    {
                        Shape aV = itV.Value();
                        NCollection_List<Shape> aLVA = aPrismAncestors.FindFromKey(aV);
                        myAncestors.Add(aV, new NCollection_List<Shape>(aLVA));
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
    public Edge EdgeReplacement(in Face theFace, in Edge theEdge)
    {
        if (!myReplacement.Find(theFace, out NCollection_DataMap<Shape, Shape> pEE))
        {
            return theEdge;
        }

        if (!pEE.Find(theEdge, out Shape pE))
        {
            return theEdge;
        }

        return ShapeCasts.Edge(pE);
    }

    //=================================================================================================

    /// <summary>
    /// The face generated for the edge connecting two tangent faces with different offset values, or the edge
    /// generated from a vertex of such an edge; a null shape if none.
    /// </summary>
    public Shape Generated(in Shape theS)
    {
        return myGenerated.Find(theS, out Shape pGenS) ? pGenS : new Shape();
    }

    /// <summary>True if the shape has generated a new face (or edge).</summary>
    public bool HasGenerated(in Shape theS) => myGenerated.IsBound(theS);

    /// <summary>
    /// The new faces built between tangent faces with different offset values (the 3D offset's per-face values).
    /// </summary>
    public NCollection_List<Shape> NewFaces() => myNewFaces;

    //=================================================================================================

    /// <summary>
    /// The shape's descendants (the inverse of <see cref="Ancestors"/>), or null if it has none; the map is built on the
    /// first query, and again with <paramref name="theUpdate"/>.
    /// </summary>
    public NCollection_List<Shape>? Descendants(in Shape theS, bool theUpdate = false)
    {
        if (myDescendants.IsEmpty() || theUpdate)
        {
            myDescendants.Clear();
            int aNbA = myAncestors.Extent();
            for (int i = 1; i <= aNbA; ++i)
            {
                Shape aSS = myAncestors.FindKey(i);
                NCollection_List<Shape> aLA = myAncestors[i];

                for (NCollection_List<Shape>.Iterator it = new(aLA); it.More(); it.Next())
                {
                    Shape aSA = it.Value();

                    if (!myDescendants.Find(aSA, out NCollection_List<Shape> pLD))
                    {
                        pLD = myDescendants.Bound(aSA, new NCollection_List<Shape>());
                    }
                    if (!pLD.Contains(aSS))
                    {
                        pLD.Append(aSS);
                    }
                }
            }
        }

        return myDescendants.Find(theS, out NCollection_List<Shape> aLD) ? aLD : null;
    }

    //=================================================================================================

    /// <summary>Clears the content of the algorithm.</summary>
    public void Clear()
    {
        myDone = false;
        myShape = new Shape();
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
    public NCollection_List<BRepOffset_Interval> Type(in Edge E)
    {
        return myMapEdgeType.Find(E);
    }

    //=================================================================================================

    /// <summary>Stores in <paramref name="LE"/> all the edges of type <paramref name="T"/> on the vertex <paramref name="V"/>.</summary>
    public void Edges(in Vertex V, ChFiDS_TypeOfConcavity T, NCollection_List<Shape> LE)
    {
        LE.Clear();
        NCollection_List<Shape> L = Ancestors(V);
        NCollection_List<Shape>.Iterator it = new(L);

        for (; it.More(); it.Next())
        {
            Edge E = ShapeCasts.Edge(it.Value());
            if (myMapEdgeType.Find(E, out NCollection_List<BRepOffset_Interval> pIntervals) && pIntervals.Extent() > 0)
            {
                BRepOffset_Tool.EdgeVertices(E, out Vertex V1, out Vertex V2);
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
    public void Edges(in Face F, ChFiDS_TypeOfConcavity T, NCollection_List<Shape> LE)
    {
        LE.Clear();
        Explorer exp = new(F, ShapeType.Edge);

        for (; exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());

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
    public void TangentEdges(in Edge Edge, in Vertex Vertex, NCollection_List<Shape> Edges)
    {
        Vector3D V, VRef;

        double U, URef;
        EdgeCurveAdaptor C3d, C3dRef;

        URef = Tool.Parameter(Vertex, Edge);
        C3dRef = new EdgeCurveAdaptor(Edge);
        VRef = C3dRef.DN(URef, 1);
        CorrectOrientationOfTangent(ref VRef, Vertex, Edge);
        if (VRef.SquareMagnitude() < Constants.Resolution())
        {
            return;
        }

        Edges.Clear();

        NCollection_List<Shape> Anc = Ancestors(Vertex);
        NCollection_List<Shape>.Iterator it = new(Anc);
        for (; it.More(); it.Next())
        {
            Edge CurE = ShapeCasts.Edge(it.Value());
            if (CurE.IsSame(Edge))
            {
                continue;
            }
            U = Tool.Parameter(Vertex, CurE);
            C3d = new EdgeCurveAdaptor(CurE);
            V = C3d.DN(U, 1);
            CorrectOrientationOfTangent(ref V, Vertex, CurE);
            if (V.SquareMagnitude() < Constants.Resolution())
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
    public bool HasAncestor(in Shape theS) => myAncestors.Contains(theS);

    /// <summary>The ancestors of the shape (throws if it has none).</summary>
    public NCollection_List<Shape> Ancestors(in Shape theS) => myAncestors.FindFromKey(theS);

    //=================================================================================================

    /// <summary>
    /// Explodes the shape into compounds of faces where all the connecting edges are of type <paramref name="T"/>.
    /// </summary>
    public void Explode(NCollection_List<Shape> List, ChFiDS_TypeOfConcavity T)
    {
        List.Clear();
        BrepBuilder B = new();
        NCollection_Map<Shape> Map = new(ShapeHasher.Instance);

        Explorer Fexp = new();
        for (Fexp.Init(myShape, ShapeType.Face); Fexp.More(); Fexp.Next())
        {
            if (Map.Add(Fexp.Current()))
            {
                Face Face = ShapeCasts.Face(Fexp.Current());
                Compound Co = new();
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
    public void Explode(NCollection_List<Shape> List, ChFiDS_TypeOfConcavity T1, ChFiDS_TypeOfConcavity T2)
    {
        List.Clear();
        BrepBuilder B = new();
        NCollection_Map<Shape> Map = new(ShapeHasher.Instance);

        Explorer Fexp = new();
        for (Fexp.Init(myShape, ShapeType.Face); Fexp.More(); Fexp.Next())
        {
            if (Map.Add(Fexp.Current()))
            {
                Face Face = ShapeCasts.Face(Fexp.Current());
                Compound Co = new();
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
    public void AddFaces(in Face Face, in Compound Co, NCollection_Map<Shape> Map,
                         ChFiDS_TypeOfConcavity T)
    {
        BrepBuilder B = new();
        NCollection_List<Shape>? pLE = Descendants(Face);
        if (pLE is null)
        {
            return;
        }
        for (NCollection_List<Shape>.Iterator it = new(pLE); it.More(); it.Next())
        {
            Edge E = ShapeCasts.Edge(it.Value());
            NCollection_List<BRepOffset_Interval> LI = Type(E);
            if (!LI.IsEmpty() && LI.First().Type() == T)
            {
                // so <NewFace> is attached to G1 by <Face>
                NCollection_List<Shape> L = Ancestors(E);
                if (L.Extent() == 2)
                {
                    Face F1 = ShapeCasts.Face(L.First());
                    if (F1.IsSame(Face))
                    {
                        F1 = ShapeCasts.Face(L.Last());
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
    public void AddFaces(in Face Face, in Compound Co, NCollection_Map<Shape> Map,
                         ChFiDS_TypeOfConcavity T1, ChFiDS_TypeOfConcavity T2)
    {
        BrepBuilder B = new();
        NCollection_List<Shape>? pLE = Descendants(Face);
        if (pLE is null)
        {
            return;
        }
        for (NCollection_List<Shape>.Iterator it = new(pLE); it.More(); it.Next())
        {
            Edge E = ShapeCasts.Edge(it.Value());
            NCollection_List<BRepOffset_Interval> LI = Type(E);
            if (!LI.IsEmpty() && (LI.First().Type() == T1 || LI.First().Type() == T2))
            {
                // so <NewFace> is attached to G1 by <Face>
                NCollection_List<Shape> L = Ancestors(E);
                if (L.Extent() == 2)
                {
                    Face F1 = ShapeCasts.Face(L.First());
                    if (F1.IsSame(Face))
                    {
                        F1 = ShapeCasts.Face(L.Last());
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
    internal void SetFaceOffsetMap(NCollection_DataMap<Shape, double> theMap)
    {
        myFaceOffsetMap = new NCollection_DataMap<Shape, double>(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, double> aPair in theMap)
        {
            myFaceOffsetMap.Bind(aPair.Key, aPair.Value);
        }
    }
}
