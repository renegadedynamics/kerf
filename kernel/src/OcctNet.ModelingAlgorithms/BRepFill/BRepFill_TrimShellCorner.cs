// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimShellCorner.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimShellCorner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2014 OPEN CASCADE SAS; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepFill_TransitionStyle;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Trims the two shells of faces meeting at a corner of a sweep's path along their intersection (right corner) or
/// cuts them before it (round corner), with the history of the faces and edges it rebuilt.
/// </summary>
/// <remarks>
/// A working object of <see cref="BRepFill_Sweep"/> (decision 21). It runs a pave filler on the sweep's own unfrozen
/// faces, so OCCT's destructive mode applies as it does natively (6P's notes), and edits the section edges the filler
/// made.
/// </remarks>
internal sealed class BRepFill_TrimShellCorner
{
    private readonly BRepFill_TransitionStyle myTransition;
    private readonly Frame3D myAxeOfBisPlane;
    private readonly Vector3D myIntPointCrossDir;
    private Shape myShape1 = new();
    private Shape myShape2 = new();
    private NCollection_HArray2<Shape>? myBounds;
    private NCollection_HArray2<Shape>? myUEdges;
    private NCollection_HArray1<Shape>? myVEdges;
    private readonly NCollection_HArray2<Shape> myFaces;
    private bool myDone;
    private bool myHasSection;
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myHistMap = new(ShapeHasher.Instance);

    //=================================================================================================

    /// <summary>
    /// The corner between the two columns of <paramref name="theFaces"/>, with the bisector plane's axis and the
    /// direction the cross product of the u-edges' tangents must follow at their intersection.
    /// </summary>
    public BRepFill_TrimShellCorner(NCollection_HArray2<Shape> theFaces,
                                    BRepFill_TransitionStyle theTransition,
                                    in Frame3D theAxeOfBisPlane,
                                    in Vector3D theIntPointCrossDir)
    {
        myTransition = theTransition;
        myAxeOfBisPlane = theAxeOfBisPlane;
        myIntPointCrossDir = theIntPointCrossDir;
        myDone = false;
        myHasSection = false;
        myFaces = new NCollection_HArray2<Shape>(theFaces.LowerRow(),
                                                        theFaces.UpperRow(),
                                                        theFaces.LowerCol(),
                                                        theFaces.UpperCol());
        myFaces.ChangeArray2().Assign(theFaces.Array2());
    }

    //=================================================================================================

    /// <summary>The section edges bounding the two shells at the corner.</summary>
    public void AddBounds(NCollection_HArray2<Shape> theBounds)
    {
        myBounds = new NCollection_HArray2<Shape>(theBounds.LowerRow(),
                                                         theBounds.UpperRow(),
                                                         theBounds.LowerCol(),
                                                         theBounds.UpperCol());
        myBounds.ChangeArray2().Assign(theBounds.Array2());
    }

    //=================================================================================================

    /// <summary>The u-edges of the two shells.</summary>
    public void AddUEdges(NCollection_HArray2<Shape> theUEdges)
    {
        myUEdges = new NCollection_HArray2<Shape>(theUEdges.LowerRow(),
                                                         theUEdges.UpperRow(),
                                                         theUEdges.LowerCol(),
                                                         theUEdges.UpperCol());
        myUEdges.ChangeArray2().Assign(theUEdges.Array2());
    }

    //=================================================================================================

    /// <summary>The column <paramref name="theIndex"/> of the v-edges: the section edges at the corner.</summary>
    public void AddVEdges(NCollection_HArray2<Shape> theVEdges, int theIndex)
    {
        myVEdges = new NCollection_HArray1<Shape>(theVEdges.LowerRow(), theVEdges.UpperRow());

        for (int i = theVEdges.LowerRow(); i <= theVEdges.UpperRow(); i++)
        {
            myVEdges.SetValue(i, theVEdges.Value(i, theIndex));
        }
    }

    //=================================================================================================

    /// <summary>Trims the shells.</summary>
    public void Perform()
    {
        int anIndex1, anIndex2, nF1, nF2, i, j, aNbP, aNbC;
        bool bhassec;

        myDone = false;
        myHistMap.Clear();

        if (myFaces.RowLength() != 2)
        {
            return;
        }
        int ii = 0, jj = 0;
        BrepBuilder aBB = new();

        for (jj = myFaces.LowerCol(); jj <= myFaces.UpperCol(); jj++)
        {
            Shell aShell = new();
            aBB.MakeShell(ref aShell);

            for (ii = myFaces.LowerRow(); ii <= myFaces.UpperRow(); ii++)
            {
                aBB.Add(aShell, myFaces.Value(ii, jj));
            }
            aShell.Closed(Tool.IsClosed(aShell));

            if (jj == myFaces.LowerCol())
            {
                myShape1 = aShell;
            }
            else
            {
                myShape2 = aShell;
            }
        }

        double aMaxTol = 0.0;
        Explorer anExp = new(myShape1, ShapeType.Vertex);
        for (; anExp.More(); anExp.Next())
        {
            aMaxTol = Max(aMaxTol, Tool.Tolerance(ShapeCasts.Vertex(anExp.Current())));
        }

        anExp.Init(myShape2, ShapeType.Vertex);
        for (; anExp.More(); anExp.Next())
        {
            aMaxTol = Max(aMaxTol, Tool.Tolerance(ShapeCasts.Vertex(anExp.Current())));
        }

        double aFuzzy = 4.0 * global::Kerf.Engine.Foundation.Precision.Confusion();
        BOPAlgo_PaveFiller aPF = new();
        NCollection_List<Shape> aLS = new();
        aLS.Append(myShape1);
        aLS.Append(myShape2);
        aPF.SetArguments(aLS);
        if (aMaxTol < 1.005 * global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            aFuzzy = Max(aPF.FuzzyValue(), aFuzzy);
            aPF.SetFuzzyValue(aFuzzy);
        }
        //
        aPF.Perform();
        if (aPF.HasErrors())
        {
            return;
        }
        //
        BOPDS_DS theDS = aPF.PDS()!;
        //
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = theDS.InterfFF();
        int aNbFFs = aFFs.Length();

        if (!SplitUEdges(myUEdges!, theDS, myIntPointCrossDir, myHistMap))
        {
            return;
        }

        for (ii = myFaces.LowerRow(); ii <= myFaces.UpperRow(); ii++)
        {
            Shape aF1 = myFaces.Value(ii, myFaces.LowerCol());
            Shape aF2 = myFaces.Value(ii, myFaces.UpperCol());
            //
            anIndex1 = theDS.Index(aF1);
            anIndex2 = theDS.Index(aF2);

            if ((anIndex1 == -1) || (anIndex2 == -1))
            {
                continue;
            }

            for (i = 0; i < aNbFFs; ++i)
            {
                BOPDS_InterfFF aFFi = aFFs[i];
                aFFi.Indices(out nF1, out nF2);
                //
                NCollection_DynamicArray<BOPDS_Point> aVP = aFFi.ChangePoints();
                aNbP = aVP.Length();
                NCollection_DynamicArray<BOPDS_Curve> aVC = aFFi.Curves();
                aNbC = aVC.Length();
                if (aNbP == 0 && aNbC == 0)
                {
                    if (!theDS.HasInterfSubShapes(nF1, nF2))
                    {
                        continue;
                    }
                }
                //
                if ((nF1 == anIndex1) && (nF2 == anIndex2))
                {
                    bhassec = false;
                    //
                    for (j = 0; j < aNbC; ++j)
                    {
                        BOPDS_Curve aBCurve = aVC[j];
                        NCollection_List<BOPDS_PaveBlock> aSectEdges = aBCurve.PaveBlocks();
                        //
                        if (aSectEdges.Extent() != 0)
                        {
                            bhassec = true;
                            break;
                        }
                    }

                    if (!bhassec)
                    {
                        if (!MakeFacesNonSec(ii, theDS, anIndex1, anIndex2))
                        {
                            myHistMap.Clear();
                            return;
                        }
                    }
                    else
                    {
                        if (!MakeFacesSec(ii, theDS, anIndex1, anIndex2, i))
                        {
                            myHistMap.Clear();
                            return;
                        }
                    }
                    break;
                }
            }
        }
        myDone = true;
    }

    //=================================================================================================

    /// <summary>True if the shells are trimmed.</summary>
    public bool IsDone() => myDone;

    //=================================================================================================

    /// <summary>True if the shells intersect (set nowhere: always false, as in OCCT).</summary>
    public bool HasSection() => myHasSection; // OCCT: myHasSection is never set

    //=================================================================================================

    /// <summary>The shapes that replace <paramref name="theShape"/> (a face, u-edge or v-edge), or none.</summary>
    public void Modified(in Shape theShape, NCollection_List<Shape> theModified)
    {
        theModified.Clear();

        if (myHistMap.IsBound(theShape))
        {
            theModified.Assign(myHistMap.Find(theShape));
        }
    }

    // ----------------------------------------------------------------------------------------------------
    // function: MakeFacesNonSec
    // purpose:         Updates <myHistMap> by new faces in the case when old faces do not intersect
    // ----------------------------------------------------------------------------------------------------
    private bool MakeFacesNonSec(int theIndex, BOPDS_DS theDS, int theFaceIndex1, int theFaceIndex2)
    {
        bool bHasNewEdge = false;
        Edge aNewEdge = new();

        BrepBuilder aBB = new();
        Shape aE1 = myBounds!.Value(theIndex, 1);
        Shape aE2 = myBounds.Value(theIndex, 2);

        // search common vertex between bounds. begin
        int anIndex1 = theDS.Index(aE1);
        int anIndex2 = theDS.Index(aE2);

        bool bvertexfound = FindCommonVertex(theDS,
                                             anIndex1,
                                             anIndex2,
                                             myIntPointCrossDir,
                                             out Vertex aCommonVertex,
                                             out _,
                                             out _);
        // search common vertex between bounds. end

        ReShape aSubstitutor = new();

        // search common vertices between uedges. begin
        NCollection_List<Shape> aCommonVertices = new();
        int acommonflag = 0; // 0 - no, 1 - first pair, 2 - second pair, 3 - both
        int ueit = 0, eindex = 0;

        for (ueit = 1, eindex = theIndex; ueit <= 2; ueit++, eindex++)
        {
            Shape aShape1 = myUEdges!.Value(eindex, myUEdges.LowerCol());
            Shape aShape2 = myUEdges.Value(eindex, myUEdges.UpperCol());
            Edge aUE1 = ShapeCasts.Edge(aShape1);
            Edge aUE2 = ShapeCasts.Edge(aShape2);

            if (myHistMap.IsBound(aShape1))
            {
                NCollection_List<Shape> lst = myHistMap.Find(aShape1);

                if (!lst.IsEmpty())
                {
                    aUE1 = ShapeCasts.Edge(lst.First());
                }
            }

            if (myHistMap.IsBound(aShape2))
            {
                NCollection_List<Shape> lst = myHistMap.Find(aShape2);

                if (!lst.IsEmpty())
                {
                    aUE2 = ShapeCasts.Edge(lst.First());
                }
            }

            if (!aShape1.IsSame(aUE1))
            {
                aSubstitutor.Replace(aShape1.Oriented(Forward), aUE1.Oriented(Forward));
            }

            if (!aShape2.IsSame(aUE2))
            {
                aSubstitutor.Replace(aShape2.Oriented(Forward), aUE2.Oriented(Forward));
            }

            Vertex V1 = Exploration.LastVertex(aUE1);
            Vertex V2 = Exploration.FirstVertex(aUE2);

            if (V1.IsSame(V2))
            {
                acommonflag = (acommonflag == 0) ? ueit : 3;
                aCommonVertices.Append(V1);
            }
        }
        // search common vertices between uedges. end

        if (bvertexfound)
        {
            if (aCommonVertices.Extent() != 1)
            {
                return false;
            }

            if (acommonflag == 1)
            {
                aNewEdge = new BRepLib_MakeEdge(ShapeCasts.Vertex(aCommonVertices.First()), aCommonVertex).InternalEdge();
            }
            else
            {
                aNewEdge = new BRepLib_MakeEdge(aCommonVertex, ShapeCasts.Vertex(aCommonVertices.First())).InternalEdge();
            }

            bHasNewEdge = true;
        }

        if (aCommonVertices.Extent() == 2)
        {
            aNewEdge = new BRepLib_MakeEdge(ShapeCasts.Vertex(aCommonVertices.First()),
                                            ShapeCasts.Vertex(aCommonVertices.Last())).InternalEdge();
            bHasNewEdge = true;
        }
        int fit = 0;

        for (fit = 1; fit <= 2; fit++)
        {
            Compound aComp = new();
            NCollection_Map<Shape> aMapV = new(ShapeHasher.Instance);
            aBB.MakeCompound(ref aComp);

            for (ueit = 1, eindex = theIndex; ueit <= 2; ueit++, eindex++)
            {
                Shape aShape = myUEdges!.Value(eindex, myUEdges.LowerCol() + fit - 1);
                Shape aUE = aShape;

                if (myHistMap.IsBound(aShape))
                {
                    NCollection_List<Shape> lst = myHistMap.Find(aShape);

                    if (!lst.IsEmpty())
                    {
                        aUE = ShapeCasts.Edge(lst.First());
                    }
                }
                Shape aV = (fit == 1) ? Exploration.FirstVertex(ShapeCasts.Edge(aUE)) : Exploration.LastVertex(ShapeCasts.Edge(aUE));
                aMapV.Add(aV);
                aBB.Add(aComp, aUE);
            }

            if (bHasNewEdge)
            {
                aBB.Add(aComp, aNewEdge);
                StoreVedgeInHistMap(myVEdges!, theIndex, aNewEdge, myHistMap);
            }

            NCollection_List<Shape> alonevertices = new();
            FindFreeVertices(aComp, aMapV, alonevertices);

            if (!alonevertices.IsEmpty() && (alonevertices.Extent() != 2))
            {
                return false;
            }

            int aFaceIndex = (fit == 1) ? theFaceIndex1 : theFaceIndex2;
            Shape aFace = theDS.Shape(aFaceIndex);
            Orientation aFaceOri = aFace.Orientation();
            aFace.Orientation(Forward);

            Explorer anExpE = new(aFace, ShapeType.Edge);

            if (bHasNewEdge)
            {
                aNewEdge.Orientation(Forward);

                // Refer to BrepFill_Sweep.cxx BuildEdge Construct an edge via an iso
                Point3D P1, P2;
                double p11, p12, p21, p22;
                P1 = Tool.Pnt(Exploration.FirstVertex(ShapeCasts.Edge(aNewEdge)));
                P2 = Tool.Pnt(Exploration.LastVertex(ShapeCasts.Edge(aNewEdge)));

                Edge aERef = ShapeCasts.Edge(fit == 1 ? aE1 : aE2);
                p11 = P1.Distance(Tool.Pnt(Exploration.FirstVertex(aERef)));
                p22 = P2.Distance(Tool.Pnt(Exploration.LastVertex(aERef)));
                p12 = P1.Distance(Tool.Pnt(Exploration.LastVertex(aERef)));
                p21 = P2.Distance(Tool.Pnt(Exploration.FirstVertex(aERef)));

                if (p11 > p12 && p22 > p21)
                {
                    aNewEdge.Reverse();
                }

                // for nonPlane surface, we should add pCurve
                ShapeFix_Edge sfe = new();
                sfe.FixAddPCurve(aNewEdge, ShapeCasts.Face(aFace), false);
            }

            NCollection_List<Shape> aOrderedList = new();

            if (!alonevertices.IsEmpty())
            {
                int anEIndex = (fit == 1) ? anIndex1 : anIndex2;
                bool bfound1 = false;
                bool bfound2 = false;
                double aparam1 = 0.0, aparam2 = 0.0;

                NCollection_List<BOPDS_Pave> aLP = new();
                theDS.Paves(anEIndex, aLP);
                NCollection_List<BOPDS_Pave>.Iterator aIt = new();
                aIt.Initialize(aLP);
                for (; aIt.More(); aIt.Next())
                {
                    BOPDS_Pave aPave = aIt.Value();
                    Shape aV = theDS.Shape(aPave.Index());

                    if (aV.IsSame(alonevertices.First()))
                    {
                        if (!bfound1)
                        {
                            aparam1 = aPave.Parameter();
                            bfound1 = true;
                        }
                    }

                    if (aV.IsSame(alonevertices.Last()))
                    {
                        if (!bfound2)
                        {
                            aparam2 = aPave.Parameter();
                            bfound2 = true;
                        }
                    }
                }

                if (bfound1 && bfound2)
                {
                    Edge aNewBoundE;

                    if (fit == 1)
                    {
                        aNewBoundE = ShapeCasts.Edge(aE1.EmptyCopied());
                    }
                    else
                    {
                        aNewBoundE = ShapeCasts.Edge(aE2.EmptyCopied());
                    }
                    Vertex aV1, aV2;

                    if (aparam1 < aparam2)
                    {
                        aV1 = ShapeCasts.Vertex(alonevertices.First());
                        aV2 = ShapeCasts.Vertex(alonevertices.Last());
                    }
                    else
                    {
                        aV1 = ShapeCasts.Vertex(alonevertices.Last());
                        aV2 = ShapeCasts.Vertex(alonevertices.First());
                        double tmp = aparam1;
                        aparam1 = aparam2;
                        aparam2 = tmp;
                    }
                    aV1.Orientation(Forward);
                    aV2.Orientation(Reversed);
                    aBB.Add(aNewBoundE, aV1);
                    aBB.Add(aNewBoundE, aV2);
                    aBB.Range(aNewBoundE, aparam1, aparam2);
                    aNewBoundE.Orientation(Forward);

                    aOrderedList.Append(aNewBoundE);

                    if (bHasNewEdge)
                    {
                        Explorer anExpV = new(aNewEdge, ShapeType.Vertex);
                        bool bfoundv = false;

                        for (; !bfoundv && anExpV.More(); anExpV.Next())
                        {
                            if (aV2.IsSame(anExpV.Current()))
                            {
                                bfoundv = true;
                            }
                        }

                        if (bfoundv)
                        {
                            aOrderedList.Append(aNewEdge);
                        }
                        else
                        {
                            aOrderedList.Prepend(aNewEdge);
                        }
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                if (bHasNewEdge)
                {
                    aOrderedList.Append(aNewEdge);
                }
            }

            if (!aOrderedList.IsEmpty())
            {
                Wire aW = new();
                aBB.MakeWire(ref aW);
                NCollection_List<Shape>.Iterator anItE = new(aOrderedList);

                for (; anItE.More(); anItE.Next())
                {
                    aBB.Add(aW, anItE.Value());
                }
                if (fit == 1)
                {
                    aSubstitutor.Replace(aE1.Oriented(Forward), aW);
                }
                else
                {
                    aSubstitutor.Replace(aE2.Oriented(Forward), aW);
                }
            }

            aSubstitutor.InternalApply(aFace);
            Shape aNewFace = aSubstitutor.InternalValue(aFace);
            aNewFace.Orientation(aFaceOri);
            NCollection_List<Shape> atmpList = new();
            atmpList.Append(aNewFace);
            myHistMap.Bind(aFace, atmpList);

            anExpE.Init(aFace, ShapeType.Edge);

            for (; anExpE.More(); anExpE.Next())
            {
                Shape aNewValue = aSubstitutor.InternalValue(anExpE.Current());

                if (aNewValue.IsNull || aNewValue.IsSame(anExpE.Current()))
                {
                    continue;
                }

                if (myHistMap.IsBound(anExpE.Current()))
                {
                    continue;
                }
                NCollection_List<Shape> aListOfNewEdge = new();
                Explorer anExpE2 = new(aNewValue, ShapeType.Edge);

                for (; anExpE2.More(); anExpE2.Next())
                {
                    aListOfNewEdge.Append(anExpE2.Current());
                }
                myHistMap.Bind(anExpE.Current(), aListOfNewEdge);
            }
        }

        return true;
    }

    // ----------------------------------------------------------------------------------------------------
    // function: MakeFacesSec
    // purpose:  Updates <myHistMap> by new faces in the case when old faces intersect each other
    // ----------------------------------------------------------------------------------------------------
    private bool MakeFacesSec(int theIndex, BOPDS_DS theDS, int theFaceIndex1, int theFaceIndex2, int theSSInterfIndex)
    {
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = theDS.InterfFF();
        BOPDS_InterfFF aFFi = aFFs[theSSInterfIndex];
        NCollection_DynamicArray<BOPDS_Curve> aBCurves = aFFi.Curves();

        Face aSecPlane = new();

        if (!FilterSectionEdges(aBCurves, aSecPlane, theDS, out Compound aSecEdges))
        {
            return false;
        }

        // Extract vertices on the intersection of correspondent U-edges
        Shape LeftE1 = myUEdges!.Value(theIndex, 1);
        Shape LeftE2 = myUEdges.Value(theIndex, 2);
        Shape RightE1 = myUEdges.Value(theIndex + 1, 1);
        Shape RightE2 = myUEdges.Value(theIndex + 1, 2);

        int IndexOfLeftE1 = theDS.Index(LeftE1);
        int IndexOfLeftE2 = theDS.Index(LeftE2);
        int IndexOfRightE1 = theDS.Index(RightE1);
        int IndexOfRightE2 = theDS.Index(RightE2);

        FindCommonVertex(theDS,
                         IndexOfLeftE1,
                         IndexOfLeftE2,
                         myIntPointCrossDir,
                         out Vertex FirstVertex,
                         out _,
                         out _);
        FindCommonVertex(theDS,
                         IndexOfRightE1,
                         IndexOfRightE2,
                         myIntPointCrossDir,
                         out Vertex LastVertex,
                         out _,
                         out _);

        Shape SecWire = new();
        Plane3D SecPlane = new();
        bool WireFound = ChooseSection(aSecEdges, FirstVertex, LastVertex, ref SecWire, ref SecPlane, out _);

        if (WireFound)
        {
            // aSecEdges = SecWire;
            Compound aComp = new();
            BrepBuilder BB = new();
            BB.MakeCompound(ref aComp);
            Explorer explo = new(SecWire, ShapeType.Edge);

            for (; explo.More(); explo.Next())
            {
                BB.Add(aComp, explo.Current());
            }
            aSecEdges = aComp;

            StoreVedgeInHistMap(myVEdges!, theIndex, SecWire, myHistMap);
        }

        int fit = 0;
        ReShape aSubstitutor = new();

        for (fit = 0; fit < 2; fit++)
        {
            int aFaceIndex = (fit == 0) ? theFaceIndex1 : theFaceIndex2;
            Face aFace = ShapeCasts.Face(theDS.Shape(aFaceIndex));
            Orientation aFaceOri = aFace.Orientation();
            Face aFaceF = aFace;
            aFaceF.Orientation(Forward);
            Edge aBoundEdge = ShapeCasts.Edge(myBounds!.Value(theIndex, myBounds.LowerCol() + fit));
            int aBoundEdgeIndex = theDS.Index(aBoundEdge);

            if (!GetUEdges(theIndex, fit, myUEdges, aBoundEdge, aFaceF, out Edge aUE1, out Edge aUE2))
            {
                return false;
            }

            Edge aUE1old = aUE1;
            Edge aUE2old = aUE2;

            if (myHistMap.IsBound(aUE1))
            {
                NCollection_List<Shape> lst = myHistMap.Find(aUE1);

                if (!lst.IsEmpty())
                {
                    Shape anEdge = lst.First().Oriented(aUE1.Orientation());

                    if (!aUE1.IsSame(anEdge))
                    {
                        aSubstitutor.Replace(aUE1.Oriented(Forward), anEdge.Oriented(Forward));
                    }
                    aUE1 = ShapeCasts.Edge(anEdge);
                }
            }

            if (myHistMap.IsBound(aUE2))
            {
                NCollection_List<Shape> lst = myHistMap.Find(aUE2);

                if (!lst.IsEmpty())
                {
                    Shape anEdge = lst.First().Oriented(aUE2.Orientation());

                    if (!aUE2.IsSame(anEdge))
                    {
                        aSubstitutor.Replace(aUE2.Oriented(Forward), anEdge.Oriented(Forward));
                    }
                    aUE2 = ShapeCasts.Edge(anEdge);
                }
            }
            Compound aCompOfSecEdges = aSecEdges;
            NCollection_List<Shape> aListOfWireEdges = new();
            BrepBuilder aBB = new();
            BOPDS_Pave aPave1 = new();
            bool isPave1OnUEdge = true;

            if (FindFromUEdge(aUE1old,
                              aUE2old,
                              aUE1,
                              aUE2,
                              aFace,
                              aSecEdges,
                              fit,
                              aBoundEdge,
                              aBoundEdgeIndex,
                              theDS,
                              myHistMap,
                              ref aCompOfSecEdges,
                              aListOfWireEdges,
                              ref aPave1,
                              out isPave1OnUEdge))
            {
                NCollection_List<Shape> aSecondListOfEdges = new();

                if (!FindFromVEdge(aPave1,
                                   isPave1OnUEdge,
                                   aUE1old,
                                   aUE2old,
                                   aFace,
                                   aCompOfSecEdges,
                                   fit,
                                   aBoundEdge,
                                   aBoundEdgeIndex,
                                   theDS,
                                   myHistMap,
                                   aSecondListOfEdges,
                                   out bool bisSectionFound))
                {
                    return false;
                }

                if (!bisSectionFound && aListOfWireEdges.IsEmpty())
                {
                    return false;
                }
                aListOfWireEdges.Append(aSecondListOfEdges);
            }
            else
            {
                return false;
            }

            if (!aListOfWireEdges.IsEmpty())
            {
                Wire aW = new();
                aBB.MakeWire(ref aW);
                NCollection_List<Shape>.Iterator aEIt = new(aListOfWireEdges);

                for (; aEIt.More(); aEIt.Next())
                {
                    if (!aBoundEdge.IsSame(aEIt.Value()))
                    {
                        aBB.Add(aW, aEIt.Value());
                    }
                }
                aSubstitutor.Replace(aBoundEdge.Oriented(Forward), aW);
            }

            aSubstitutor.InternalApply(aFace);
            Shape aNewFace = aSubstitutor.InternalValue(aFace);
            aNewFace.Orientation(aFaceOri);
            NCollection_List<Shape> atmpList = new();
            atmpList.Append(aNewFace);
            myHistMap.Bind(aFace, atmpList);

            Explorer anExpE = new(aFace, ShapeType.Edge);

            for (; anExpE.More(); anExpE.Next())
            {
                Shape aNewValue = aSubstitutor.InternalValue(anExpE.Current());

                if (aNewValue.IsNull || aNewValue.IsSame(anExpE.Current()))
                {
                    continue;
                }

                if (myHistMap.IsBound(anExpE.Current()))
                {
                    continue;
                }
                NCollection_List<Shape> aListOfNewEdge = new();
                Explorer anExpE2 = new(aNewValue, ShapeType.Edge);

                for (; anExpE2.More(); anExpE2.Next())
                {
                    aListOfNewEdge.Append(anExpE2.Current());
                }
                myHistMap.Bind(anExpE.Current(), aListOfNewEdge);
            }
        }
        return true;
    }

    //=================================================================================================

    private bool ChooseSection(in Shape Comp,
                               in Vertex theFirstVertex,
                               in Vertex theLastVertex,
                               ref Shape resWire,
                               ref Plane3D resPlane,
                               out bool IsSingular)
    {
        IsSingular = false;

        int ind, i, j;
        BrepBuilder BB = new();

        if (myTransition == BRepFill_Right && !theFirstVertex.IsNull
            && !theLastVertex.IsNull) // the case where section wire goes from
                                         // its known first vertex to its known last vertex
        {
            Wire NewWire = new();
            BB.MakeWire(ref NewWire);

            Compound OldComp = new();
            BB.MakeCompound(ref OldComp);
            Iterator iter = new(Comp);
            for (; iter.More(); iter.Next())
            {
                BB.Add(OldComp, iter.Value());
            }

            Edge FirstEdge = FindEdgeCloseToBisectorPlane(theFirstVertex, OldComp, myAxeOfBisPlane.Axis());
            if (FirstEdge.IsNull)
            {
                return false;
            }

            iter.Initialize(OldComp);
            if (!iter.More())
            {
                iter.Initialize(Comp);
                BB.Add(OldComp, iter.Value());
            }
            Edge LastEdge = FindEdgeCloseToBisectorPlane(theLastVertex, OldComp, myAxeOfBisPlane.Axis());
            if (LastEdge.IsNull)
            {
                return false;
            }

            if (FirstEdge.IsNull || LastEdge.IsNull)
            {
                return false;
            }

            BB.Add(NewWire, FirstEdge);

            if (!FirstEdge.IsSame(LastEdge))
            {
                Vertex aCommonVertex = new();
                bool CommonVertexExists = FindCommonVertex(FirstEdge, LastEdge, theFirstVertex, theLastVertex, ref aCommonVertex);
                if (CommonVertexExists)
                {
                    BB.Add(NewWire, LastEdge);
                }
                else
                {
                    Vertex Vertex1, Vertex2, V1, V2;
                    Exploration.Vertices(FirstEdge, out V1, out V2);
                    Vertex1 = (theFirstVertex.IsSame(V1)) ? V2 : V1;
                    Exploration.Vertices(LastEdge, out V1, out V2);
                    Vertex2 = (theLastVertex.IsSame(V1)) ? V2 : V1;

                    NCollection_List<Shape> MiddleEdges = new();
                    if (FindMiddleEdges(Vertex1, Vertex2, myAxeOfBisPlane.Axis(), OldComp, MiddleEdges))
                    {
                        NCollection_List<Shape>.Iterator itl = new(MiddleEdges);
                        for (; itl.More(); itl.Next())
                        {
                            BB.Add(NewWire, itl.Value());
                        }
                        BB.Add(NewWire, LastEdge);
                    }
                    else
                    {
                        // trim <FirstEdge> and <LastEdge> in the points of extrema
                        // these points become new vertex with centre between them
                        BRepExtrema_ExtCC Extrema = new(FirstEdge, LastEdge);
                        if (Extrema.IsDone() && Extrema.NbExt() > 0)
                        {
                            int imin = 1;
                            for (i = 2; i <= Extrema.NbExt(); i++)
                            {
                                if (Extrema.SquareDistance(i) < Extrema.SquareDistance(imin))
                                {
                                    imin = i;
                                }
                            }

                            double aMinDist = Math.Sqrt(Extrema.SquareDistance(imin));
                            double ParamOnFirstEdge = Extrema.ParameterOnE1(imin);
                            double ParamOnLastEdge = Extrema.ParameterOnE2(imin);
                            Point3D PointOnFirstEdge = Extrema.PointOnE1(imin);
                            Point3D PointOnLastEdge = Extrema.PointOnE2(imin);
                            Point3D MidPnt = new((PointOnFirstEdge.XYZ() + PointOnLastEdge.XYZ()) / 2);
                            aCommonVertex = new BRepLib_MakeVertex(MidPnt).InternalVertex();
                            BB.UpdateVertex(aCommonVertex, 1.001 * aMinDist / 2);

                            UpdateSectionEdge(FirstEdge, theFirstVertex, ref aCommonVertex, ParamOnFirstEdge);
                            UpdateSectionEdge(LastEdge, theLastVertex, ref aCommonVertex, ParamOnLastEdge);

                            BB.Add(NewWire, LastEdge);
                        }
                    }
                }
            }

            resWire = NewWire;
            resPlane = new Plane3D(new CoordinateSystem(myAxeOfBisPlane));
            return true;
        }

        // General case: try to find continuous section closest to bisector plane
        Compound OldComp2 = new();
        BrepBuilder B = new();
        B.MakeCompound(ref OldComp2);
        Iterator iter2 = new(Comp);
        for (; iter2.More(); iter2.Next())
        {
            B.Add(OldComp2, iter2.Value());
        }

        bool anError = false;
        NCollection_Sequence<Shape> Wseq = new();
        for (;;)
        {
            Explorer explo = new(OldComp2, ShapeType.Edge);
            if (!explo.More())
            {
                break;
            }
            Edge FirstEdge = ShapeCasts.Edge(explo.Current());
            Wire NewWire = new BRepLib_MakeWire(FirstEdge).InternalWire();
            B.Remove(OldComp2, FirstEdge);
            if (NewWire.Closed())
            {
                Wseq.Append(NewWire);
                continue;
            }

            for (;;)
            {
                Vertex[] Extremity = new Vertex[2];
                Exploration.Vertices(NewWire, out Extremity[0], out Extremity[1]);
                if (Extremity[0].IsNull || Extremity[1].IsNull)
                {
                    anError = true;
                    break;
                }
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(OldComp2, ShapeType.Vertex, ShapeType.Edge, VEmap);
                NCollection_List<Shape>[] Vedges = [new(), new()];
                for (j = 0; j < 2; j++)
                {
                    if (VEmap.Contains(Extremity[j]))
                    {
                        Vedges[j].Assign(VEmap.FindFromKey(Extremity[j]));
                    }
                }
                if (Vedges[0].IsEmpty() && Vedges[1].IsEmpty())
                {
                    // no more edges in OldComp to continue NewWire
                    break;
                }
                bool Modified = false;
                for (j = 0; j < 2; j++)
                {
                    if (Vedges[j].Extent() == 1)
                    {
                        Edge anEdge = ShapeCasts.Edge(Vedges[j].First());
                        NewWire = new BRepLib_MakeWire(NewWire, anEdge).InternalWire();
                        B.Remove(OldComp2, anEdge);
                        Modified = true;
                    }
                }
                if (!Modified) // only multiple connections
                {
                    ind = (Vedges[0].IsEmpty()) ? 1 : 0;
                    NCollection_Sequence<Shape> Edges = new();
                    NCollection_List<Shape>.Iterator itl = new(Vedges[ind]);
                    for (; itl.More(); itl.Next())
                    {
                        Edges.Append(itl.Value());
                    }
                    int theind = 0;
                    double MinDeviation = RealLast();
                    for (j = 1; j <= Edges.Length(); j++)
                    {
                        Wire aWire = new BRepLib_MakeWire(NewWire, ShapeCasts.Edge(Edges[j])).InternalWire();
                        Plane3D aPlane = new();
                        double Deviation = ComputeAveragePlaneAndMaxDeviation(aWire, ref aPlane, out _);
                        if (Deviation < MinDeviation)
                        {
                            MinDeviation = Deviation;
                            theind = j;
                        }
                    }
                    NewWire = new BRepLib_MakeWire(NewWire, ShapeCasts.Edge(Edges[theind])).InternalWire();
                    B.Remove(OldComp2, Edges[theind]);
                }
                if (NewWire.Closed())
                {
                    break;
                }
            }
            Wseq.Append(NewWire);
            if (anError)
            {
                break;
            }
        }

        double MinAngle = RealLast();
        Explorer Explo = new(OldComp2, ShapeType.Edge);
        if (!anError && !Explo.More()) // wires are built successfully and compound <OldComp> is empty
        {
            if (Wseq.Length() == 1) // only one wire => it becomes result
            {
                resWire = Wseq.First();
                ComputeAveragePlaneAndMaxDeviation(resWire, ref resPlane, out IsSingular);
                return true;
            }
            else // we must choose the wire which average plane is closest to bisector plane
            {    //(check angle between axes)
                for (i = 1; i <= Wseq.Length(); i++)
                {
                    Wire aWire = ShapeCasts.Wire(Wseq[i]);
                    Plane3D aPln = new();
                    ComputeAveragePlaneAndMaxDeviation(aWire, ref aPln, out bool issing);
                    if (issing)
                    {
                        continue;
                    }

                    double Angle = aPln.Axis().Angle(myAxeOfBisPlane.Axis());
                    if (Angle > M_PI / 2)
                    {
                        Angle = M_PI - Angle;
                    }

                    if (Angle < MinAngle)
                    {
                        MinAngle = Angle;
                        resWire = aWire;
                        resPlane = aPln;
                    }
                }
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    private static bool SplitUEdges(NCollection_HArray2<Shape> theUEdges,
                                    BOPDS_DS theDS,
                                    in Vector3D theCrossDirection,
                                    NCollection_DataMap<Shape, NCollection_List<Shape>> theHistMap)
    {
        NCollection_DynamicArray<BOPDS_InterfVV> aVVs = theDS.InterfVV();

        BrepBuilder aBB = new();
        int ueit = 0, upRow, lowCol, upCol;
        //
        upRow = theUEdges.UpperRow();
        lowCol = theUEdges.LowerCol();
        upCol = theUEdges.UpperCol();
        //
        for (ueit = theUEdges.LowerRow(); ueit <= upRow; ueit++)
        {
            Shape aE1 = theUEdges.Value(ueit, lowCol);
            Shape aE2 = theUEdges.Value(ueit, upCol);

            if (theHistMap.IsBound(aE1) || theHistMap.IsBound(aE2))
            {
                continue;
            }

            int anEIndex1 = theDS.Index(aE1);
            int anEIndex2 = theDS.Index(aE2);

            bool bvertexfound = FindCommonVertex(theDS,
                                                 anEIndex1,
                                                 anEIndex2,
                                                 theCrossDirection,
                                                 out Vertex aCommonVertex,
                                                 out double apar1,
                                                 out double apar2);
            //
            if (!bvertexfound)
            {
                Vertex V1 = Exploration.LastVertex(ShapeCasts.Edge(aE1));
                Vertex V2 = Exploration.FirstVertex(ShapeCasts.Edge(aE2));
                int vindex1 = theDS.Index(V1);
                int vindex2 = theDS.Index(V2);
                int vvit = 0;
                int aNbVVs = aVVs.Length();

                for (vvit = 0; !bvertexfound && (vvit < aNbVVs); vvit++)
                {
                    BOPDS_InterfVV aVV = aVVs[vvit];

                    if (((vindex1 == aVV.Index1()) && (vindex2 == aVV.Index2()))
                        || ((vindex1 == aVV.Index2()) && (vindex2 == aVV.Index1())))
                    {
                        if (!aVV.HasIndexNew())
                        {
                            continue;
                        }
                        aCommonVertex = ShapeCasts.Vertex(theDS.Shape(aVV.IndexNew()));
                        bvertexfound = true;
                        apar1 = Tool.Parameter(V1, ShapeCasts.Edge(aE1));
                        apar2 = Tool.Parameter(V2, ShapeCasts.Edge(aE2));
                    }
                }
            }

            if (bvertexfound)
            {
                Vertex aV1, aV2;
                double f = 0.0, l = 0.0;
                //
                Edge aNewE1 = ShapeCasts.Edge(aE1.EmptyCopied());
                Exploration.Vertices(ShapeCasts.Edge(aE1), out aV1, out aV2);
                aNewE1.Orientation(Forward);
                aV1.Orientation(Forward);
                aBB.Add(aNewE1, aV1);
                aCommonVertex.Orientation(Reversed);
                aBB.Add(aNewE1, aCommonVertex);
                Tool.Range(ShapeCasts.Edge(aE1), out f, out l);
                aBB.Range(aNewE1, f, apar1);

                //
                Edge aNewE2 = ShapeCasts.Edge(aE2.EmptyCopied());
                Exploration.Vertices(ShapeCasts.Edge(aE2), out aV1, out aV2);
                aNewE2.Orientation(Forward);
                aCommonVertex.Orientation(Forward);
                aBB.Add(aNewE2, aCommonVertex);
                aBB.Add(aNewE2, aV2);
                Tool.Range(ShapeCasts.Edge(aE2), out f, out l);
                aBB.Range(aNewE2, apar2, l);

                NCollection_List<Shape> lst = new();
                lst.Append(aNewE1);
                theHistMap.Bind(aE1, lst);
                lst = new NCollection_List<Shape>();
                lst.Append(aNewE2);
                theHistMap.Bind(aE2, lst);
            }
        }
        return true;
    }

    //=================================================================================================

    private static void StoreVedgeInHistMap(NCollection_HArray1<Shape> theVEdges,
                                            int theIndex,
                                            in Shape theNewVshape,
                                            NCollection_DataMap<Shape, NCollection_List<Shape>> theHistMap)
    {
        // Replace default value in the map (v-iso edge of face)
        // by intersection of two consecutive faces
        Shape aVEdge = theVEdges.Value(theIndex);

        theHistMap.Bound(aVEdge, new NCollection_List<Shape>()).Append(theNewVshape);
    }

    //=================================================================================================

    private static void FindFreeVertices(in Shape theShape,
                                         NCollection_Map<Shape> theVerticesToAvoid,
                                         NCollection_List<Shape> theListOfVertex)
    {
        theListOfVertex.Clear();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theShape, ShapeType.Vertex, ShapeType.Edge, aMap);
        int i = 0;

        for (i = 1; i <= aMap.Extent(); i++)
        {
            Shape aKey = aMap.FindKey(i);

            if (theVerticesToAvoid.Contains(aKey))
            {
                continue;
            }
            NCollection_List<Shape> aList = aMap.FindFromIndex(i);

            if (aList.Extent() < 2)
            {
                theListOfVertex.Append(aKey);
            }
        }
    }

    //=================================================================================================

    private static bool FindCommonVertex(BOPDS_DS theDS,
                                         int theEIndex1,
                                         int theEIndex2,
                                         in Vector3D theCrossDirection,
                                         out Vertex theCommonVertex,
                                         out double theParamOnE1,
                                         out double theParamOnE2)
    {
        // OCCT leaves the outputs unwritten when no vertex is found; its callers initialise them
        theCommonVertex = new Vertex();
        theParamOnE1 = 0.0;
        theParamOnE2 = 0.0;

        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = theDS.InterfEE();

        bool bvertexfound = false;
        int eeit = 0;

        Edge theE1 = ShapeCasts.Edge(theDS.Shape(theEIndex1));
        Edge theE2 = ShapeCasts.Edge(theDS.Shape(theEIndex2));
        EdgeCurveAdaptor aBC1 = new(theE1), aBC2 = new(theE2);

        int aNbEEs;
        aNbEEs = aEEs.Length();
        for (eeit = 0; eeit < aNbEEs; ++eeit)
        {
            BOPDS_InterfEE aEE = aEEs[eeit];

            if ((theEIndex1 == aEE.Index1() && theEIndex2 == aEE.Index2())
                || (theEIndex1 == aEE.Index2() && theEIndex2 == aEE.Index1()))
            {
                if (!aEE.HasIndexNew())
                {
                    continue;
                }

                IntTools_CommonPrt aCP = aEE.CommonPart();
                if (aCP.Type() == ShapeType.Vertex)
                {
                    theCommonVertex = ShapeCasts.Vertex(theDS.Shape(aEE.IndexNew()));

                    if (theEIndex1 == aEE.Index1())
                    {
                        IntTools_Tools.VertexParameters(aCP, out theParamOnE1, out theParamOnE2);
                    }
                    else
                    {
                        IntTools_Tools.VertexParameters(aCP, out theParamOnE2, out theParamOnE1);
                    }

                    Direction3D aIntersectPointCrossDir;

                    // intersect point aDirOnE1.cross(aDirOnE2) should same direction with path
                    // theCrossDirection
                    aBC1.D1(theParamOnE1, out _, out Vector3D aDirOnE1);
                    aBC2.D1(theParamOnE2, out _, out Vector3D aDirOnE2);
                    aIntersectPointCrossDir = new Direction3D(aDirOnE1.Crossed(aDirOnE2));

                    if (aIntersectPointCrossDir.Dot(new Direction3D(theCrossDirection)) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        bvertexfound = true;
                        break;
                    }
                }
            }
        }
        return bvertexfound;
    }

    //=================================================================================================

    private static bool GetUEdges(int theIndex,
                                  int theRank,
                                  NCollection_HArray2<Shape> theUEdges,
                                  in Edge theBoundEdge,
                                  in Face theFace,
                                  out Edge theFirstUEdge,
                                  out Edge theSecondUEdge)
    {
        theFirstUEdge = new Edge();
        theSecondUEdge = new Edge();
        Shape aUE1 = theUEdges.Value(theIndex, theUEdges.LowerCol() + theRank);
        Shape aUE2 = theUEdges.Value(theIndex + 1, theUEdges.LowerCol() + theRank);

        Face aFace = theFace;
        aFace.Orientation(Forward);
        Edge E1 = new(), E2 = new();
        Explorer anExp = new(aFace, ShapeType.Edge);

        for (; anExp.More(); anExp.Next())
        {
            if (E1.IsNull && aUE1.IsSame(anExp.Current()))
            {
                E1 = ShapeCasts.Edge(anExp.Current());
            }
            else if (E2.IsNull && aUE2.IsSame(anExp.Current()))
            {
                E2 = ShapeCasts.Edge(anExp.Current());
            }
        }

        if (E1.IsNull || E2.IsNull)
        {
            return false;
        }

        Curve2D? C1 = Tool.CurveOnSurface(E1, aFace, out double f, out double l);

        if (C1 is null)
        {
            return false;
        }
        Point2D PU1 = (theRank == 0) ? C1.Value(l) : C1.Value(f);
        Curve2D? C2 = Tool.CurveOnSurface(theBoundEdge, aFace, out f, out l);

        if (C2 is null)
        {
            return false;
        }
        Tool.Range(theBoundEdge, out f, out l);
        Point2D pf = C2.Value(f);
        Vertex aV = (theRank == 0) ? Exploration.LastVertex(E1) : Exploration.FirstVertex(E1);
        double aTolerance = Tool.Tolerance(aV);
        FaceSurfaceAdaptor aBAS = new(aFace, false);

        if (pf.Distance(PU1) > aBAS.UResolution(aTolerance))
        {
            Edge atmpE = E1;
            E1 = E2;
            E2 = atmpE;
        }
        theFirstUEdge = E1;
        theSecondUEdge = E2;
        return true;
    }

    //=================================================================================================

    private static bool FillGap(in Vertex theFirstVertex,
                                in Vertex theLastVertex,
                                in Point2D theFirstPoint,
                                in Point2D theLastPoint,
                                in Face theFace,
                                in Compound theSectionEdges,
                                NCollection_List<Shape> theOrderedList)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theSectionEdges, ShapeType.Vertex, ShapeType.Edge, aMap);

        if (aMap.IsEmpty())
        {
            return false;
        }

        if (!aMap.Contains(theFirstVertex) || !aMap.Contains(theLastVertex))
        {
            return false;
        }
        NCollection_List<Shape> aListOfEdge = new();
        NCollection_Map<Shape> aMapToAvoid = new(ShapeHasher.Instance);

        if (FindNextEdge(theFirstVertex, theLastVertex, aMap, aMapToAvoid, aListOfEdge))
        {
            if (!aListOfEdge.IsEmpty())
            {
                return CheckAndOrientEdges(aListOfEdge, theFirstPoint, theLastPoint, theFace, theOrderedList);
            }
        }
        return false;
    }

    //=================================================================================================

    private static bool FindNextEdge(in Vertex theFirstVertex,
                                     in Vertex theLastVertex,
                                     NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapVE,
                                     NCollection_Map<Shape> theMapToAvoid,
                                     NCollection_List<Shape> theOrderedList)
    {
        Vertex aCurVertex = theFirstVertex;
        NCollection_Map<Shape> aMapToAvoid = new(theMapToAvoid);
        NCollection_List<Shape> aListOfEdge = new();
        int i = 0;

        for (i = 1; i <= theMapVE.Extent(); i++)
        {
            if (!theMapVE.Contains(aCurVertex))
            {
                break;
            }
            NCollection_List<Shape> lste = theMapVE.FindFromKey(aCurVertex);
            bool befound = false;

            NCollection_List<Shape>.Iterator anIt = new(lste);

            for (; anIt.More(); anIt.Next())
            {
                Shape anEdge = anIt.Value();
                Vertex aSaveCurVertex = aCurVertex;

                if (!aMapToAvoid.Contains(anEdge))
                {
                    Exploration.Vertices(ShapeCasts.Edge(anEdge), out Vertex V1, out Vertex V2);

                    if (!aCurVertex.IsSame(V1))
                    {
                        aCurVertex = V1;
                    }
                    else if (!aCurVertex.IsSame(V2))
                    {
                        aCurVertex = V2;
                    }
                    aMapToAvoid.Add(anEdge);
                    befound = true;
                    aListOfEdge.Append(anEdge);

                    if (!aCurVertex.IsSame(theLastVertex))
                    {
                        NCollection_List<Shape> aListtmp = new();

                        if (!FindNextEdge(aCurVertex, theLastVertex, theMapVE, aMapToAvoid, aListtmp))
                        {
                            aListOfEdge.Clear();
                            aCurVertex = aSaveCurVertex;
                            continue;
                        }
                        else
                        {
                            aListOfEdge.Append(aListtmp);
                            theOrderedList.Append(aListOfEdge);
                            return true;
                        }
                    }
                    break;
                }
            }

            if (aCurVertex.IsSame(theLastVertex))
            {
                break;
            }

            if (!befound)
            {
                return false;
            }
        }

        if (aCurVertex.IsSame(theLastVertex))
        {
            theOrderedList.Append(aListOfEdge);
            return true;
        }
        return false;
    }

    //=================================================================================================

    private static bool CheckAndOrientEdges(NCollection_List<Shape> theOrderedList,
                                            in Point2D theFirstPoint,
                                            in Point2D theLastPoint,
                                            in Face theFace,
                                            NCollection_List<Shape> theOrientedList)
    {
        NCollection_List<Shape>.Iterator anIt = new(theOrderedList);

        if (!anIt.More())
        {
            return true;
        }

        Edge aEPrev = ShapeCasts.Edge(anIt.Value());
        anIt.Next();

        Curve2D aCurve = Tool.CurveOnSurface(aEPrev, theFace, out double f, out double l)!;
        Exploration.Vertices(aEPrev, out Vertex Vf, out Vertex Vl);
        FaceSurfaceAdaptor aBAS = new(theFace, false);

        double aTolerance1 = (Vf.IsNull) ? global::Kerf.Engine.Foundation.Precision.Confusion() : Tool.Tolerance(Vf);
        double aTolerance2 = (Vl.IsNull) ? global::Kerf.Engine.Foundation.Precision.Confusion() : Tool.Tolerance(Vl);
        double utol = aBAS.UResolution(aTolerance1);
        double vtol = aBAS.VResolution(aTolerance1);
        aTolerance1 = (utol > vtol) ? utol : vtol;
        utol = aBAS.UResolution(aTolerance2);
        vtol = aBAS.VResolution(aTolerance2);
        aTolerance2 = (utol > vtol) ? utol : vtol;

        Point2D ap = aCurve.Value(f);
        bool bFirstFound = false;
        bool bLastFound = false;

        if (ap.Distance(theFirstPoint) < aTolerance1)
        {
            if (theOrientedList.IsEmpty())
            {
                theOrientedList.Append(aEPrev.Oriented(Forward));
            }
            bFirstFound = true;
        }
        else if (ap.Distance(theLastPoint) < aTolerance1)
        {
            if (theOrientedList.IsEmpty())
            {
                theOrientedList.Append(aEPrev.Oriented(Reversed));
            }
            bLastFound = true;
        }
        ap = aCurve.Value(l);

        if (ap.Distance(theLastPoint) < aTolerance2)
        {
            if (theOrientedList.IsEmpty())
            {
                theOrientedList.Append(aEPrev.Oriented(Forward));
            }
            bLastFound = true;
        }
        else if (ap.Distance(theFirstPoint) < aTolerance2)
        {
            if (theOrientedList.IsEmpty())
            {
                theOrientedList.Append(aEPrev.Oriented(Reversed));
            }
            bFirstFound = true;
        }

        if (!theOrientedList.IsEmpty())
        {
            aEPrev = ShapeCasts.Edge(theOrientedList.Last());
        }

        for (; anIt.More(); anIt.Next())
        {
            Edge aE = ShapeCasts.Edge(anIt.Value());
            Exploration.Vertices(aEPrev, out Vertex aV11, out Vertex aV12, true);
            Exploration.Vertices(aE, out Vertex aV21, out Vertex aV22, false);

            Orientation anOri = (aV12.IsSame(aV21) || aV11.IsSame(aV22)) ? Forward : Reversed;
            theOrientedList.Append(aE.Oriented(anOri));
            aEPrev = ShapeCasts.Edge(theOrientedList.Last());

            aTolerance1 = (aV21.IsNull) ? global::Kerf.Engine.Foundation.Precision.Confusion() : Tool.Tolerance(aV21);
            aTolerance2 = (aV22.IsNull) ? global::Kerf.Engine.Foundation.Precision.Confusion() : Tool.Tolerance(aV22);
            utol = aBAS.UResolution(aTolerance1);
            vtol = aBAS.VResolution(aTolerance1);
            aTolerance1 = (utol > vtol) ? utol : vtol;
            utol = aBAS.UResolution(aTolerance2);
            vtol = aBAS.VResolution(aTolerance2);
            aTolerance2 = (utol > vtol) ? utol : vtol;
            aCurve = Tool.CurveOnSurface(aE, theFace, out f, out l)!;
            ap = aCurve.Value(f);

            if (ap.Distance(theFirstPoint) < aTolerance1)
            {
                bFirstFound = true;
            }
            else if (ap.Distance(theLastPoint) < aTolerance1)
            {
                bLastFound = true;
            }
            ap = aCurve.Value(l);

            if (ap.Distance(theFirstPoint) < aTolerance2)
            {
                bFirstFound = true;
            }
            else if (ap.Distance(theLastPoint) < aTolerance2)
            {
                bLastFound = true;
            }
        }

        return bFirstFound && bLastFound;
    }

    //=================================================================================================

    private static bool FindVertex(in Edge theEdge,
                                   int theRank,
                                   BOPDS_DS theDS,
                                   NCollection_DataMap<Shape, NCollection_List<Shape>> theHistMap,
                                   ref Vertex theVertex,
                                   ref BOPDS_Pave thePave)
    {
        if (!theHistMap.IsBound(theEdge))
        {
            return false;
        }

        NCollection_List<Shape> lst = theHistMap.Find(theEdge);

        if (lst.IsEmpty())
        {
            return false;
        }

        Edge aNewEdge = ShapeCasts.Edge(lst.First());
        Tool.Range(aNewEdge, out double f, out double l);

        if (theRank == 0)
        {
            thePave.SetParameter(l);
            theVertex = Exploration.LastVertex(aNewEdge);
        }
        else
        {
            thePave.SetParameter(f);
            theVertex = Exploration.FirstVertex(aNewEdge);
        }
        int anIndex = theDS.Index(theVertex);
        if (anIndex == -1)
        {
            int i, i1, i2;
            i1 = theDS.NbSourceShapes();
            i2 = theDS.NbShapes();
            for (i = i1; i < i2; ++i)
            {
                Shape aSx = theDS.Shape(i);
                if (aSx.IsSame(theVertex))
                {
                    anIndex = i;
                    break;
                }
            }
        }

        thePave.SetIndex(anIndex);

        return true;
    }

    //=================================================================================================

    private static bool FindNextVertex(int theEdgeIndex,
                                       in BOPDS_Pave thePrevPave,
                                       BOPDS_DS theDS,
                                       ref Vertex theNextVertex,
                                       ref BOPDS_Pave thePave)
    {
        bool bTakePave, bFound;
        BOPDS_Pave aTmpPave;
        NCollection_List<BOPDS_Pave>.Iterator aItP = new();
        //
        BOPDS_Pave anullpave = new();
        bFound = false;
        bTakePave = thePrevPave.IsEqual(anullpave);

        NCollection_List<BOPDS_Pave> aLP = new();
        theDS.Paves(theEdgeIndex, aLP);
        aItP.Initialize(aLP);
        for (; aItP.More(); aItP.Next())
        {
            aTmpPave = aItP.Value();
            //
            if (bTakePave)
            {
                if (theDS.IsNewShape(aTmpPave.Index()))
                {
                    theNextVertex = ShapeCasts.Vertex(theDS.Shape(aTmpPave.Index()));
                    thePave = aTmpPave;
                    bFound = true;
                    break;
                }
            }
            //
            else if (aTmpPave.IsEqual(thePrevPave))
            {
                bTakePave = true;
            }
        }

        return bFound;
    }

    //=================================================================================================

    private static bool GetPave(int theEdgeIndex, bool isFirst, BOPDS_DS theDS, ref BOPDS_Pave thePave)
    {
        NCollection_List<BOPDS_Pave> aLP = new();

        theDS.Paves(theEdgeIndex, aLP);
        if (aLP.Extent() == 0)
        {
            return false;
        }
        //
        if (isFirst)
        {
            thePave = aLP.First();
        }
        else
        {
            thePave = aLP.Last();
        }

        return true;
    }

    //=================================================================================================

    private static bool FindFromUEdge(in Edge theUE1Old,
                                      in Edge theUE2Old,
                                      in Edge theUE1New,
                                      in Edge theUE2New,
                                      in Face theFace,
                                      in Compound theSecEdges,
                                      int theRank,
                                      in Edge theBoundEdge,
                                      int theBoundEdgeIndex,
                                      BOPDS_DS theDS,
                                      NCollection_DataMap<Shape, NCollection_List<Shape>> theHistMap,
                                      ref Compound theSecEdgesNew,
                                      NCollection_List<Shape> theListOfWireEdges,
                                      ref BOPDS_Pave theFoundPave,
                                      out bool isOnUEdge)
    {
        theFoundPave.SetIndex(0);
        theFoundPave.SetParameter(0.0);
        isOnUEdge = true;

        Face aFaceF = theFace;
        aFaceF.Orientation(Forward);
        Vertex aPrevVertex = new(), aNextVertex = new();
        Compound aCompOfSecEdges = theSecEdges;
        NCollection_List<Shape> aListOfWireEdges = new();

        BOPDS_Pave aPave1 = new(), aPave2 = new();
        Point2D p1, p2;
        Vertex aFirstV, aLastV;
        BOPDS_Pave atmpPave = new();

        if (!FindVertex(theUE1Old, theRank, theDS, theHistMap, ref aPrevVertex, ref atmpPave))
        {
            return true;
        }

        if (aPrevVertex.IsNull)
        {
            return false;
        }

        aFirstV = aPrevVertex;
        bool bSecFound = false;
        Curve2D aC1 = Tool.CurveOnSurface(theUE1New, aFaceF, out double f, out double l)!;
        p1 = (theRank == 0) ? aC1.Value(l) : aC1.Value(f);
        BOPDS_Pave afoundpave = new();
        NCollection_List<BOPDS_Pave> aLP = new();
        theDS.Paves(theBoundEdgeIndex, aLP);
        int nbpave = aLP.Extent();
        int pit = 0;

        while (FindNextVertex(theBoundEdgeIndex, aPave1, theDS, ref aNextVertex, ref aPave2) && (pit < nbpave))
        {
            aLastV = aNextVertex;
            Curve2D aC2 = Tool.CurveOnSurface(theBoundEdge, aFaceF, out f, out l)!;
            p2 = aC2.Value(aPave2.Parameter());
            NCollection_List<Shape> aOrderedList = new();

            if (FillGap(aFirstV, aLastV, p1, p2, aFaceF, aCompOfSecEdges, aOrderedList))
            {
                // remove found edges...
                RemoveEdges(aCompOfSecEdges, aOrderedList, out Compound aComp);
                aCompOfSecEdges = aComp;
                aListOfWireEdges.Append(aOrderedList);
                afoundpave = aPave2;
                isOnUEdge = false;
                bSecFound = true;
                break;
            }
            aPrevVertex = aNextVertex;
            aPave1 = aPave2;
            pit++;
        }

        if (!bSecFound && FindVertex(theUE2Old, theRank, theDS, theHistMap, ref aNextVertex, ref aPave2))
        {
            aLastV = aNextVertex;
            Curve2D aC2 = Tool.CurveOnSurface(theUE2New, aFaceF, out f, out l)!;
            p2 = aC2.Value(aPave2.Parameter());
            NCollection_List<Shape> aOrderedList = new();

            if (FillGap(aFirstV, aLastV, p1, p2, aFaceF, aCompOfSecEdges, aOrderedList))
            {
                // remove found edges...
                RemoveEdges(aCompOfSecEdges, aOrderedList, out Compound aComp);
                aCompOfSecEdges = aComp;
                aListOfWireEdges.Append(aOrderedList);
                afoundpave = aPave2;
                bSecFound = true;
                isOnUEdge = true;
            }
        }

        if (bSecFound)
        {
            theFoundPave = afoundpave;
            theListOfWireEdges.Assign(aListOfWireEdges);
            theSecEdgesNew = aCompOfSecEdges;
        }
        return true;
    }

    //=================================================================================================

    // Finds the split of the bound edge between aPave1 and aPave2 (the repeated "get split" blocks of
    // FindFromVEdge, OCCT.NET)
    private static Edge BoundSplit(BOPDS_DS theDS, int theBoundEdgeIndex, in BOPDS_Pave aPave1, in BOPDS_Pave aPave2)
    {
        Edge aESplit = new();
        NCollection_List<BOPDS_PaveBlock>.Iterator aPBIt = new(theDS.PaveBlocks(theBoundEdgeIndex));

        for (; aPBIt.More(); aPBIt.Next())
        {
            BOPDS_PaveBlock aPB1 = aPBIt.Value();
            if (aPB1.OriginalEdge() == theBoundEdgeIndex && aPB1.Pave1().IsEqual(aPave1) && aPB1.Pave2().IsEqual(aPave2))
            {
                if (aPB1.Edge() > 0)
                {
                    aESplit = ShapeCasts.Edge(theDS.Shape(aPB1.Edge()));
                    break;
                }
            }
        }
        return aESplit;
    }

    private static bool FindFromVEdge(in BOPDS_Pave thePrevPave,
                                      bool isOnUEdge,
                                      in Edge theUE1Old,
                                      in Edge theUE2Old,
                                      in Face theFace,
                                      in Compound theSecEdges,
                                      int theRank,
                                      in Edge theBoundEdge,
                                      int theBoundEdgeIndex,
                                      BOPDS_DS theDS,
                                      NCollection_DataMap<Shape, NCollection_List<Shape>> theHistMap,
                                      NCollection_List<Shape> theListOfWireEdges,
                                      out bool isSectionFound)
    {
        theListOfWireEdges.Clear();
        isSectionFound = false;
        //
        Face aFaceF = theFace;
        aFaceF.Orientation(Forward);
        Vertex aPrevVertex, aNextVertex = new();
        Compound aCompOfSecEdges = theSecEdges;
        NCollection_List<Shape> aListOfWireEdges = new();

        BOPDS_Pave aPave1, aPave2 = new();

        if (isOnUEdge)
        {
            Vertex atmpVertex = new();
            BOPDS_Pave aPaveOfE2 = new();

            if (FindVertex(theUE2Old, theRank, theDS, theHistMap, ref atmpVertex, ref aPaveOfE2))
            {
                if (thePrevPave.IsEqual(aPaveOfE2))
                {
                    return true;
                }
            }
        }

        Point2D p1 = new(0.0, 0.0), p2 = new(0.0, 0.0);
        Vertex aFirstV, aLastV;
        Curve2D aC1 = Tool.CurveOnSurface(theUE1Old, aFaceF, out double f, out double l)!;
        _ = aC1;
        Curve2D aC2 = Tool.CurveOnSurface(theBoundEdge, aFaceF, out f, out l)!;
        bool bSecFound = false;

        aPave1 = thePrevPave;

        if (isOnUEdge)
        {
            BOPDS_Pave atmpPave = new();

            if (!GetPave(theBoundEdgeIndex, true, theDS, ref atmpPave))
            {
                return false;
            }
            aPave1 = atmpPave;
        }
        p1 = aC2.Value(aPave1.Parameter());
        aPrevVertex = ShapeCasts.Vertex(theDS.Shape(aPave1.Index()));

        NCollection_List<BOPDS_Pave> aLP = new();
        theDS.Paves(theBoundEdgeIndex, aLP);
        int nbpave = aLP.Extent();
        int pit = 0;
        NCollection_Array1<NCollection_List<Shape>> anArrayOfListOfSec = new(1, nbpave);
        for (int iArr = 1; iArr <= nbpave; iArr++)
        {
            anArrayOfListOfSec[iArr] = new NCollection_List<Shape>();
        }

        // by pairs non continuously. begin
        int k = 0;
        BOPDS_Pave aFirstPave = aPave1;
        Vertex aFirstVertex = aPrevVertex;
        Point2D apfirst = p1;
        NCollection_List<BOPDS_Pave> aFirstPaves = new(), aLastPaves = new();
        NCollection_List<int> aListOfFlags = new();
        int apaircounter = 1;

        for (k = 0; k < nbpave; k++)
        {
            aPave1 = aFirstPave;
            p1 = apfirst;
            aPrevVertex = aFirstVertex;
            bool bfound = false;
            pit = 0;

            while (FindNextVertex(theBoundEdgeIndex, aPave1, theDS, ref aNextVertex, ref aPave2) && (pit < nbpave))
            {
                aFirstV = aPrevVertex;
                aLastV = aNextVertex;
                p2 = aC2.Value(aPave2.Parameter());

                NCollection_List<Shape> aOrderedList = new();

                if (FillGap(aFirstV, aLastV, p1, p2, aFaceF, aCompOfSecEdges, aOrderedList))
                {
                    RemoveEdges(aCompOfSecEdges, aOrderedList, out Compound aComp);
                    aCompOfSecEdges = aComp;

                    anArrayOfListOfSec[apaircounter++].Append(aOrderedList);
                    aFirstPaves.Append(aFirstPave);
                    aLastPaves.Append(aPave2);
                    aListOfFlags.Append(1);
                    aFirstPave = aPave2;
                    aFirstVertex = aNextVertex;
                    apfirst = p2;
                    aPrevVertex = aNextVertex;
                    bSecFound = true;
                    bfound = true;
                }
                aPave1 = aPave2;
                pit++;
            }

            if (FindVertex(theUE2Old, theRank, theDS, theHistMap, ref aNextVertex, ref aPave2))
            {
                aFirstV = aPrevVertex;
                aLastV = aNextVertex;
                Curve2D aC3 = Tool.CurveOnSurface(theUE2Old, aFaceF, out f, out l)!;
                p2 = aC3.Value(aPave2.Parameter());

                NCollection_List<Shape> aOrderedList = new();

                if (FillGap(aFirstV, aLastV, p1, p2, aFaceF, aCompOfSecEdges, aOrderedList))
                {
                    RemoveEdges(aCompOfSecEdges, aOrderedList, out Compound aComp);
                    aCompOfSecEdges = aComp;
                    anArrayOfListOfSec[apaircounter++].Append(aOrderedList);
                    aFirstPaves.Append(aFirstPave);
                    aLastPaves.Append(aPave2);
                    aListOfFlags.Append(0);
                    bSecFound = true;
                    break;
                }
            }

            if (!bfound)
            {
                if (!FindNextVertex(theBoundEdgeIndex, aFirstPave, theDS, ref aNextVertex, ref aPave2))
                {
                    break;
                }
                aFirstPave = aPave2;
                apfirst = aC2.Value(aPave2.Parameter());
                aFirstVertex = aNextVertex;
            }
        }
        // by pairs non continuously. end

        // by pairs continuously. begin
        aPave1 = thePrevPave;

        if (isOnUEdge)
        {
            BOPDS_Pave atmpPave = new();

            if (!GetPave(theBoundEdgeIndex, true, theDS, ref atmpPave))
            {
                return false;
            }
            aPave1 = atmpPave;
        }
        p1 = aC2.Value(aPave1.Parameter());
        aPrevVertex = ShapeCasts.Vertex(theDS.Shape(aPave1.Index()));

        pit = 0;

        while (FindNextVertex(theBoundEdgeIndex, aPave1, theDS, ref aNextVertex, ref aPave2) && (pit < nbpave))
        {
            aFirstV = aPrevVertex;
            aLastV = aNextVertex;
            p2 = aC2.Value(aPave2.Parameter());

            bool bisinside = false;
            int apbindex = 0;
            int apbcounter = 1;
            NCollection_List<BOPDS_Pave>.Iterator aPIt1 = new(), aPIt2 = new();
            NCollection_List<int>.Iterator aFlagIt = new();

            for (aPIt1.Initialize(aFirstPaves), aPIt2.Initialize(aLastPaves), aFlagIt.Initialize(aListOfFlags);
                 aPIt1.More() && aPIt2.More() && aFlagIt.More();
                 aPIt1.Next(), aPIt2.Next(), aFlagIt.Next(), apbcounter++)
            {
                bool bfin = false;
                bool blin = false;

                if (aPave1.IsEqual(aPIt1.Value()))
                {
                    bfin = true;
                }
                else
                {
                    bfin = (aPave1.Parameter() > aPIt1.Value().Parameter());
                }

                if (aFlagIt.Value() != 0)
                {
                    if (aPave2.IsEqual(aPIt2.Value()))
                    {
                        blin = true;
                    }
                    else
                    {
                        blin = (aPave2.Parameter() < aPIt2.Value().Parameter());
                    }
                }
                else
                {
                    if ((aPave2.Index() == aPIt2.Value().Index()) && (aPave2.Index() > 0))
                    {
                        Curve2D pc = Tool.CurveOnSurface(theUE2Old, aFaceF, out f, out l)!;
                        Point2D p3 = pc.Value(aPIt2.Value().Parameter());
                        Vertex aV = ShapeCasts.Vertex(theDS.Shape(aPave2.Index()));
                        FaceSurfaceAdaptor aBAS = new(aFaceF, false);
                        double aTolerance = Tool.Tolerance(aV);
                        double utol = aBAS.UResolution(aTolerance);
                        double vtol = aBAS.VResolution(aTolerance);
                        aTolerance = (utol > vtol) ? utol : vtol;

                        if (p2.Distance(p3) < aTolerance)
                        {
                            blin = true;
                        }
                    }
                }

                if (bfin && blin)
                {
                    apbindex = apbcounter;
                    bisinside = true;
                    break;
                }
            }

            if (!bisinside)
            {
                NCollection_List<Shape> aOrderedList = new();

                if (FillGap(aFirstV, aLastV, p1, p2, aFaceF, aCompOfSecEdges, aOrderedList))
                {
                    RemoveEdges(aCompOfSecEdges, aOrderedList, out Compound aComp);
                    aCompOfSecEdges = aComp;
                    aListOfWireEdges.Append(aOrderedList);

                    bSecFound = true;
                }
                else
                {
                    // get split
                    Edge aESplit = BoundSplit(theDS, theBoundEdgeIndex, aPave1, aPave2);

                    if (!aESplit.IsNull)
                    {
                        aListOfWireEdges.Append(aESplit);
                    }
                }
            }
            else
            {
                if (apbindex > 0)
                {
                    NCollection_List<Shape> aListOfSec = anArrayOfListOfSec[apbindex];
                    aListOfWireEdges.Append(aListOfSec);
                }
            }
            aPave1 = aPave2;
            aPrevVertex = aNextVertex;
            p1 = p2;
            pit++;
        }

        if (FindVertex(theUE2Old, theRank, theDS, theHistMap, ref aNextVertex, ref aPave2))
        {
            aFirstV = aPrevVertex;
            aLastV = aNextVertex;
            Curve2D aC3 = Tool.CurveOnSurface(theUE2Old, aFaceF, out f, out l)!;
            p2 = aC3.Value(aPave2.Parameter());

            bool bisinside = false;
            int apbindex = 0;
            int apbcounter = 1;
            NCollection_List<BOPDS_Pave>.Iterator aPIt1 = new(), aPIt2 = new();
            NCollection_List<int>.Iterator aFlagIt = new();

            for (aPIt1.Initialize(aFirstPaves), aPIt2.Initialize(aLastPaves), aFlagIt.Initialize(aListOfFlags);
                 aPIt1.More() && aPIt2.More() && aFlagIt.More();
                 aPIt1.Next(), aPIt2.Next(), aFlagIt.Next(), apbcounter++)
            {
                bool bfin = false;
                bool blin = false;

                if (aPave1.IsEqual(aPIt1.Value()))
                {
                    bfin = true;
                }
                else
                {
                    bfin = (aPave1.Parameter() > aPIt1.Value().Parameter());
                }

                if (aFlagIt.Value() != 0)
                {
                    if (aPave2.IsEqual(aPIt2.Value()))
                    {
                        blin = true;
                    }
                    else
                    {
                        blin = (aPave2.Parameter() < aPIt2.Value().Parameter());
                    }
                }
                else
                {
                    blin = true;
                }

                if (bfin && blin)
                {
                    apbindex = apbcounter;
                    bisinside = true;
                    break;
                }
            }

            if (!bisinside)
            {
                NCollection_List<Shape> aOrderedList = new();

                if (FillGap(aFirstV, aLastV, p1, p2, aFaceF, aCompOfSecEdges, aOrderedList))
                {
                    RemoveEdges(aCompOfSecEdges, aOrderedList, out Compound aComp);
                    aCompOfSecEdges = aComp;
                    aListOfWireEdges.Append(aOrderedList);

                    bSecFound = true;
                }
                else
                {
                    // add split
                    // get split
                    if (!GetPave(theBoundEdgeIndex, false, theDS, ref aPave2))
                    {
                        return false;
                    }
                    //
                    Edge aESplit = BoundSplit(theDS, theBoundEdgeIndex, aPave1, aPave2);

                    if (!aESplit.IsNull)
                    {
                        aListOfWireEdges.Append(aESplit);
                    }
                }
            }
            else
            {
                if (apbindex > 0)
                {
                    NCollection_List<Shape> aListOfSec = anArrayOfListOfSec[apbindex];
                    aListOfWireEdges.Append(aListOfSec);
                }
            }
        }
        else
        {
            // add split
            // get split
            if (!GetPave(theBoundEdgeIndex, false, theDS, ref aPave2))
            {
                return false;
            }

            Edge aESplit = BoundSplit(theDS, theBoundEdgeIndex, aPave1, aPave2);

            if (!aESplit.IsNull)
            {
                aListOfWireEdges.Append(aESplit);
            }
        }

        // by pairs continuously. end
        theListOfWireEdges.Assign(aListOfWireEdges);
        isSectionFound = bSecFound;
        return true;
    }

    //=================================================================================================

    private static void RemoveEdges(in Compound theSourceComp,
                                    NCollection_List<Shape> theListToRemove,
                                    out Compound theResultComp)
    {
        BrepBuilder aBB = new();
        Compound aComp = new();
        aBB.MakeCompound(ref aComp);
        Explorer anExp = new(theSourceComp, ShapeType.Edge);

        for (; anExp.More(); anExp.Next())
        {
            bool bfound = false;
            NCollection_List<Shape>.Iterator anIt = new(theListToRemove);

            for (; !bfound && anIt.More(); anIt.Next())
            {
                bfound = anExp.Current().IsSame(anIt.Value());
            }

            if (!bfound)
            {
                aBB.Add(aComp, anExp.Current());
            }
        }
        theResultComp = aComp;
    }

    //=================================================================================================

    private static bool FilterSectionEdges(NCollection_DynamicArray<BOPDS_Curve> theBCurves,
                                           in Face theSecPlane,
                                           BOPDS_DS theDS,
                                           out Compound theResult)
    {
        theResult = new Compound();
        theResult.Nullify();

        BrepBuilder aBB = new();
        aBB.MakeCompound(ref theResult);
        int aNbCurves = theBCurves.Length();
        int cit = 0;
        NCollection_List<BOPDS_PaveBlock>.Iterator aPBIt = new();

        for (cit = 0; cit < aNbCurves; ++cit)
        {
            BOPDS_Curve aBCurve = theBCurves[cit];
            NCollection_List<BOPDS_PaveBlock> aSectEdges = aBCurve.PaveBlocks();

            aPBIt.Initialize(aSectEdges);
            for (; aPBIt.More(); aPBIt.Next())
            {
                BOPDS_PaveBlock aPB = aPBIt.Value();
                int nSect = aPB.Edge();
                Shape aS = theDS.Shape(nSect);
                Edge anEdge = ShapeCasts.Edge(aS);
                bool bAddEdge = true;

                if (!theSecPlane.IsNull)
                {
                    IntTools_BeanFaceIntersector anIntersector = new(anEdge, theSecPlane);
                    Tool.Range(anEdge, out double f, out double l);
                    anIntersector.SetBeanParameters(f, l);
                    //
                    IntTools_Context aContext = new();
                    anIntersector.SetContext(aContext);
                    //
                    anIntersector.Perform();

                    if (anIntersector.IsDone())
                    {
                        bAddEdge = false;
                        int r = 0;

                        for (r = 1; r <= anIntersector.Result().Length(); r++)
                        {
                            IntTools_Range aRange = anIntersector.Result().Value(r);

                            if (((aRange.First() - f) < global::Kerf.Engine.Foundation.Precision.PConfusion()) && ((l - aRange.Last()) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            {
                                bAddEdge = true;
                                break;
                            }
                        }
                    }
                }

                if (bAddEdge)
                {
                    aBB.Add(theResult, aS);
                }
            }
        }

        return true;
    }

    //=================================================================================================

    private static double ComputeAveragePlaneAndMaxDeviation(in Shape aWire, ref Plane3D thePlane, out bool IsSingular)
    {
        int N = 40;
        int nedges = aWire.ChildCount;

        NCollection_Array1<Point3D> Pnts = new(1, nedges * N);
        int ind = 1, i;
        for (Iterator iter = new(aWire); iter.More(); iter.Next())
        {
            Edge anEdge = ShapeCasts.Edge(iter.Value());
            EdgeCurveAdaptor aCurve = new(anEdge);
            UniformAbscissa Distribution = new(aCurve, N + 1);
            for (i = 1; i <= N; i++)
            {
                double par = Distribution.Parameter(i);
                Pnts[ind++] = aCurve.Value(par);
            }
        }

        global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(Pnts, out Frame3D Axe, out IsSingular);
        if (IsSingular)
        {
            return -1;
        }

        thePlane = new Plane3D(new CoordinateSystem(Axe));
        double MaxDeviation = 0;
        for (i = 1; i <= Pnts.Length(); i++)
        {
            double dist = thePlane.Distance(Pnts[i]);
            if (dist > MaxDeviation)
            {
                MaxDeviation = dist;
            }
        }
        return MaxDeviation;
    }

    private static void UpdateSectionEdge(in Edge theEdge,
                                          in Vertex theConstVertex,
                                          ref Vertex theVertex,
                                          double theParam)
    {
        Edge F_Edge = theEdge;
        F_Edge.Orientation(Forward);

        Orientation OrOfVertex;
        Vertex AnotherVertex;
        Exploration.Vertices(F_Edge, out Vertex V1, out Vertex V2);
        if (theConstVertex.IsSame(V1))
        {
            OrOfVertex = Reversed;
            AnotherVertex = V2;
        }
        else
        {
            OrOfVertex = Forward;
            AnotherVertex = V1;
        }

        BrepBuilder BB = new();
        Tool.Range(F_Edge, out double fpar, out double lpar);
        if (OrOfVertex == Forward)
        {
            fpar = theParam;
        }
        else
        {
            lpar = theParam;
        }
        BB.Range(F_Edge, fpar, lpar);

        F_Edge.Free(true);
        BB.Remove(F_Edge, AnotherVertex);
        theVertex.Orientation(OrOfVertex);
        BB.Add(F_Edge, theVertex);
    }

    // Finds the edge connected to <theVertex> in the compound <theComp>
    // that is closest to bisector plane angularly.
    // Removes found edge from <theComp>
    //<theAxis> is the axis of bisector plane
    private static Edge FindEdgeCloseToBisectorPlane(in Vertex theVertex,
                                                            in Compound theComp,
                                                            in Axis3D theAxis)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theComp, ShapeType.Vertex, ShapeType.Edge, VEmap);

        Edge MinEdge = new();
        if (!VEmap.Contains(theVertex))
        {
            return MinEdge;
        }

        BrepBuilder BB = new();

        NCollection_List<Shape> Edges = VEmap.FindFromKey(theVertex);
        if (Edges.Extent() == 1)
        {
            MinEdge = ShapeCasts.Edge(Edges.First());
        }
        else
        {
            NCollection_List<Shape>.Iterator itl = new(Edges);
            double MinAngle = RealLast();
            for (; itl.More(); itl.Next())
            {
                Edge anEdge = ShapeCasts.Edge(itl.Value());
                Wire aWire = new();
                BB.MakeWire(ref aWire);
                BB.Add(aWire, anEdge);
                Plane3D aPln = new();
                ComputeAveragePlaneAndMaxDeviation(aWire, ref aPln, out bool issing);
                double anAngle;
                if (issing) // edge is a segment of line
                {
                    //<anAngle> is angle between <anEdge> and its projection on bisector plane
                    EdgeCurveAdaptor BAcurve = new(anEdge);
                    Point3D FirstPnt = BAcurve.Value(BAcurve.FirstParameter());
                    Point3D LastPnt = BAcurve.Value(BAcurve.LastParameter());
                    Vector3D EdgeVec = new(FirstPnt, LastPnt);
                    Axis3D EdgeAxis = new(FirstPnt, new Direction3D(EdgeVec));
                    anAngle = EdgeAxis.Direction().Angle(theAxis.Direction());
                    if (anAngle > M_PI / 2)
                    {
                        anAngle = M_PI - anAngle;
                    }
                    anAngle = M_PI / 2 - anAngle;
                }
                else
                {
                    anAngle = aPln.Axis().Angle(theAxis);
                    if (anAngle > M_PI / 2)
                    {
                        anAngle = M_PI - anAngle;
                    }
                }

                if (anAngle < MinAngle)
                {
                    MinAngle = anAngle;
                    MinEdge = anEdge;
                }
            }
        } // else (more than one edge)

        BB.Remove(theComp, MinEdge);
        return MinEdge;
    }

    private static bool FindMiddleEdges(in Vertex theVertex1,
                                        in Vertex theVertex2,
                                        in Axis3D theAxis,
                                        in Compound theComp,
                                        NCollection_List<Shape> theElist)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theComp, ShapeType.Vertex, ShapeType.Edge, VEmap);
        if (VEmap.IsEmpty())
        {
            return false;
        }

        if (!VEmap.Contains(theVertex1) || !VEmap.Contains(theVertex2))
        {
            return false;
        }

        Vertex CurVertex = theVertex1;
        for (;;)
        {
            Edge CurEdge;

            CurEdge = FindEdgeCloseToBisectorPlane(CurVertex, theComp, theAxis);
            if (CurEdge.IsNull)
            {
                return false;
            }

            Exploration.Vertices(CurEdge, out Vertex V1, out Vertex V2);
            CurVertex = (V1.IsSame(CurVertex)) ? V2 : V1;

            theElist.Append(CurEdge);
            if (CurVertex.IsSame(theVertex2))
            {
                return true;
            }
        }
    }

    private static bool FindCommonVertex(in Edge theFirstEdge,
                                         in Edge theLastEdge,
                                         in Vertex theFirstVertex,
                                         in Vertex theLastVertex,
                                         ref Vertex theCommonVertex)
    {
        if (!theFirstVertex.IsSame(theLastVertex))
        {
            bool CommonVertexExists = Exploration.CommonVertex(theFirstEdge, theLastEdge, ref theCommonVertex);
            return CommonVertexExists;
        }

        Exploration.Vertices(theFirstEdge, out Vertex V1, out Vertex V2);
        Exploration.Vertices(theLastEdge, out Vertex V3, out Vertex V4);

        if (V1.IsSame(theFirstVertex))
        {
            if (V2.IsSame(V3) || V2.IsSame(V4))
            {
                theCommonVertex = V2;
                return true;
            }
        }
        else
        {
            if (V1.IsSame(V3) || V1.IsSame(V4))
            {
                theCommonVertex = V1;
                return true;
            }
        }

        return false;
    }
}
