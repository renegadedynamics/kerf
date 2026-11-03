// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FuseEdges.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FuseEdges.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Fuses the connected edges of a shape that lie on the same curve (line, circle, ellipse, B-spline, Bezier): each
/// vertex shared by exactly two such edges, bounding the same faces, is removed and the edges become one.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape it is given: the fused edges, and the faces, wires and
/// parents that hold them, are new shapes (BRepTools_Substitution). <see cref="Shape"/>, <see cref="ResultEdges()"/>
/// and <see cref="Faces()"/> freeze what they return; ported code reads <see cref="InternalShape"/> and the internal
/// map forms.
/// </remarks>
public sealed class BRepLib_FuseEdges
{
    private Shape myShape;
    private bool myShapeDone;
    private bool myEdgesDone;
    private bool myResultEdgesDone;
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myMapVerLstEdg =
        new(ShapeHasher.Instance);
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myMapEdgLstFac =
        new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<int, NCollection_List<Shape>> myMapLstEdg = new();
    private readonly NCollection_DataMap<int, Shape> myMapEdg = new();
    private readonly NCollection_DataMap<Shape, Shape> myMapFaces = new(ShapeHasher.Instance);
    private int myNbConnexEdge;
    private NCollection_IndexedMap<Shape> myAvoidEdg = new(ShapeHasher.Instance);
    private bool myConcatBSpl;

    private static void BCSmoothing(BSplineCurve3D theC, int theCont, double theTol)
    {
        int aNbIter = 5;
        bool bContinue = true;
        int iter = 1;
        var aKnotIndex = new NCollection_Sequence<int>();
        var aKnotIns = new NCollection_Sequence<double>();

        while (bContinue && iter <= aNbIter)
        {
            int aNbKnots = theC.KnotCount;
            NCollection_Array1<int> aMults = new(theC.InternalMultiplicities());
            NCollection_Array1<double> aKnots = new(theC.InternalKnots());
            int i, m = theC.Degree();
            m = m - theCont;

            if (m < 1)
            {
                return;
            }

            aKnotIndex.Clear();

            for (i = 2; i < aNbKnots; i++)
            {
                if (aMults[i] > m)
                {
                    if (!(theC.RemoveKnot(i, m, theTol)))
                    {
                        aKnotIndex.Append(i);
                    }
                }
            }

            // Prepare knots for inserting;

            int aNbAdd = aKnotIndex.Length();

            if (aNbAdd == 0)
            {
                return;
            }

            aKnotIns.Clear();

            double aLastKnot = aKnots[1];
            for (i = 1; i <= aNbAdd; i++)
            {
                int anInd = aKnotIndex[i];

                double aK1 = 0.5 * (aKnots[anInd] + aKnots[anInd - 1]);
                if (Math.Abs(aK1 - aLastKnot) > 1.0e-3)
                {
                    aKnotIns.Append(aK1);
                    aLastKnot = aK1;
                }

                double aK2 = 0.5 * (aKnots[anInd + 1] + aKnots[anInd]);

                if (Math.Abs(aK2 - aLastKnot) > 1.0e-3)
                {
                    aKnotIns.Append(aK2);
                    aLastKnot = aK2;
                }
            }

            aNbAdd = aKnotIns.Length();

            for (i = 1; i <= aNbAdd; i++)
            {
                theC.InsertKnot(aKnotIns[i], m, 1.0e-3);
            }

            iter++;
        }

        if (iter > aNbIter)
        {
            BCSmoothing(theC, theCont, 1.0e10);
        }
    }

    private static void MakeClosedCurve(ref Curve3D C, in Point3D PF, ref double f, ref double l)
    {
        BSplineCurve3D aBC = (BSplineCurve3D)C;
        // OCCT.NET: frozen. OCCT makes the curve periodic and moves its origin in place, and the curve can be the old
        // edge's own (or the basis curve of its trimmed curve), shared with the caller's shape: the port works on a copy.
        if (aBC.IsFrozen)
        {
            aBC = (BSplineCurve3D)aBC.Copy();
        }
        Continuity aCont = aBC.Continuity();
        // Find new origin
        aBC.SetPeriodic();
        int fk = aBC.FirstUKnotIndex();
        int lk = aBC.LastUKnotIndex();
        int k;
        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        eps *= eps;
        double porig = 2.0 * l;
        double dmin = 1.0e100, pmin = f;
        for (k = fk; k <= lk; k++)
        {
            Point3D aP = aBC.Value(aBC.Knot(k));
            double d = PF.SquareDistance(aP);
            if (PF.SquareDistance(aP) > eps)
            {
                if (d < dmin)
                {
                    pmin = aBC.Knot(k);
                    dmin = d;
                }
                continue;
            }

            porig = aBC.Knot(k);
            break;
        }
        if (porig > l)
        {
            // try to project
            var aGAC = new CurveAdaptor(aBC);
            var aPrj = new LocateExtPC(PF, aGAC, pmin, global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (aPrj.IsDone)
            {
                porig = aPrj.Point().Parameter();
            }
            else
            {
                throw new ConstructionErrorException("FuseEdges : Projection failed for closed curve");
            }
        }

        aBC.SetOrigin(porig, global::Kerf.Engine.Foundation.Precision.PConfusion());
        f = aBC.FirstParameter();
        l = aBC.LastParameter();
        aBC.Segment(f, l);
        if (aCont > Continuity.C0 && aBC.Continuity() == Continuity.C0)
        {
            BCSmoothing(aBC, 1, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        C = aBC;
        f = C.FirstParameter();
        l = C.LastParameter();
    }

    /// <summary>
    /// Initialises the algorithm on <paramref name="theShape"/>. As in OCCT, <paramref name="PerformNow"/> is ignored:
    /// the work is done on demand.
    /// </summary>
    /// <exception cref="NullObjectException">The shape is null.</exception>
    public BRepLib_FuseEdges(in Shape theShape, bool PerformNow = false)
    {
        _ = PerformNow;
        myShape = theShape;
        myShapeDone = false;
        myEdgesDone = false;
        myResultEdgesDone = false;
        myNbConnexEdge = 0;
        myConcatBSpl = false;
        //  if (theShape.ShapeType() != TopAbs_SHELL && theShape.ShapeType() != TopAbs_SOLID)
        //    throw Standard_ConstructionError("FuseEdges");
        Standard_NullObject_Raise_if(theShape.IsNull, "FuseEdges");
        myMapFaces.Clear();
    }

    /// <summary>Sets the edges that must not be fused (a copy of the map is kept).</summary>
    public void AvoidEdges(NCollection_IndexedMap<Shape> theMapEdg)
    {
        myAvoidEdg = new NCollection_IndexedMap<Shape>(theMapEdg);
    }

    /// <summary>Sets whether connected B-spline edges that are only G1 at their common vertex may be concatenated.</summary>
    public void SetConcatBSpl(bool theConcatBSpl = true)
    {
        myConcatBSpl = theConcatBSpl;
    }

    /// <summary>
    /// The lists of connected edges to be fused, by index: each list is a set of connected edges, in the forward
    /// direction, that can be fused into one. The edges are the input shape's own.
    /// </summary>
    public IReadOnlyDictionary<int, ISchematiqList<Shape>> Edges()
    {
        var aMap = new NCollection_DataMap<int, NCollection_List<Shape>>();
        Edges(aMap);
        var aResult = new Dictionary<int, ISchematiqList<Shape>>();
        foreach (KeyValuePair<int, NCollection_List<Shape>> anItem in aMap)
        {
            aResult.Add(anItem.Key, PackedList.Own([.. anItem.Value]));
        }
        return aResult;
    }

    /// <summary>OCCT's form: copies the lists of edges to be fused into <paramref name="theMapLstEdg"/> (ported code).</summary>
    internal void Edges(NCollection_DataMap<int, NCollection_List<Shape>> theMapLstEdg)
    {
        if (!myEdgesDone)
        {
            BuildListEdges();
        }

        // theMapLstEdg = myMapLstEdg: C++ copies the map and its lists
        theMapLstEdg.Clear();
        foreach (KeyValuePair<int, NCollection_List<Shape>> anItem in myMapLstEdg)
        {
            theMapLstEdg.Bind(anItem.Key, new NCollection_List<Shape>(anItem.Value));
        }
    }

    /// <summary>The fused edges (frozen), by the index of the list of edges each replaces (see <see cref="Edges()"/>).</summary>
    public IReadOnlyDictionary<int, Shape> ResultEdges()
    {
        var aMap = new NCollection_DataMap<int, Shape>();
        ResultEdges(aMap);
        var aResult = new Dictionary<int, Shape>();
        foreach (KeyValuePair<int, Shape> anItem in aMap)
        {
            anItem.Value.TShape()?.FreezeGraph();
            aResult.Add(anItem.Key, anItem.Value);
        }
        return aResult;
    }

    /// <summary>OCCT's form: copies the fused edges into <paramref name="theMapEdg"/>, unfrozen (ported code).</summary>
    internal void ResultEdges(NCollection_DataMap<int, Shape> theMapEdg)
    {
        if (!myEdgesDone)
        {
            BuildListEdges();
        }

        if (!myResultEdgesDone)
        {
            BuildListResultEdges();
        }

        theMapEdg.Clear();
        foreach (KeyValuePair<int, Shape> anItem in myMapEdg)
        {
            theMapEdg.Bind(anItem.Key, anItem.Value);
        }
    }

    /// <summary>The faces modified by the fusion: each face of the input mapped to its new face (frozen).</summary>
    public IReadOnlyDictionary<Shape, Shape> Faces()
    {
        var aMap = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Faces(aMap);
        var aResult = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, Shape> anItem in aMap)
        {
            anItem.Value.TShape()?.FreezeGraph();
            aResult.Add(anItem.Key, anItem.Value);
        }
        return aResult;
    }

    /// <summary>OCCT's form: copies the modified faces into <paramref name="theMapFac"/>, unfrozen (ported code).</summary>
    internal void Faces(NCollection_DataMap<Shape, Shape> theMapFac)
    {
        if (!myEdgesDone)
        {
            BuildListEdges();
        }

        if (!myResultEdgesDone)
        {
            BuildListResultEdges();
        }

        if (!myShapeDone)
        {
            Perform();
        }

        theMapFac.Clear();
        foreach (KeyValuePair<Shape, Shape> anItem in myMapFaces)
        {
            theMapFac.Bind(anItem.Key, anItem.Value);
        }
    }

    /// <summary>The number of vertices removed by the fusion.</summary>
    public int NbVertices()
    {
        Standard_NullObject_Raise_if(myShape.IsNull, "FuseEdges : No Shape");
        int nbedges, nbvertices = 0;

        if (!myEdgesDone)
        {
            BuildListEdges();
        }

        if ((nbedges = myMapLstEdg.Extent()) > 0)
        {
            foreach (KeyValuePair<int, NCollection_List<Shape>> itEdg in myMapLstEdg)
            {
                int iLst = itEdg.Key;
                NCollection_List<Shape> LmapEdg = myMapLstEdg.Find(iLst);
                nbvertices += LmapEdg.Extent() - 1;
            }
        }

        return nbvertices;
    }

    /// <summary>The shape with its edges fused (performed on demand), frozen.</summary>
    public Shape Shape()
    {
        Shape aShape = InternalShape();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>The shape with its edges fused, without freezing it (ported code).</summary>
    internal Shape InternalShape()
    {
        Standard_NullObject_Raise_if(myShape.IsNull, "FuseEdges : No Shape");

        if (!myEdgesDone)
        {
            BuildListEdges();
        }

        if (!myResultEdgesDone)
        {
            BuildListResultEdges();
        }

        if (!myShapeDone)
        {
            Perform();
        }

        return myShape;
    }

    // Build the all the lists of edges that are to be fused
    private void BuildListEdges()
    {
        //--------------------------------------------------------
        // Step One : Build the map ancestors
        //--------------------------------------------------------

        // Clear the maps
        myMapLstEdg.Clear();
        myMapVerLstEdg.Clear();
        myMapEdgLstFac.Clear();

        Exploration.MapShapesAndUniqueAncestors(myShape, ShapeType.Vertex, ShapeType.Edge, myMapVerLstEdg);
        Exploration.MapShapesAndAncestors(myShape, ShapeType.Edge, ShapeType.Face, myMapEdgLstFac);

        int iEdg;
        var mapUniqEdg = new NCollection_Map<Shape>(ShapeHasher.Instance);

        // for each edge of myMapEdgLstFac
        for (iEdg = 1; iEdg <= myMapEdgLstFac.Extent(); iEdg++)
        {
            Shape edgecur = myMapEdgLstFac.FindKey(iEdg);
            var LstEdg = new NCollection_List<Shape>();

            // if edge not already treated
            if (!mapUniqEdg.Contains(edgecur)
                && (edgecur.Orientation() == Forward || edgecur.Orientation() == Reversed))
            {
                if (myAvoidEdg.Contains(edgecur))
                {
                    continue; // edge is not allowed to be fused
                }
                BuildListConnexEdge(edgecur, mapUniqEdg, LstEdg);
                if (LstEdg.Extent() > 1)
                {
                    myNbConnexEdge++;
                    myMapLstEdg.Bind(myNbConnexEdge, LstEdg);
                }
            }
        }

        myEdgesDone = true;
        myResultEdgesDone = false;
    }

    private void BuildListResultEdges()
    {
        // if we have edges to fuse
        if (myMapLstEdg.Extent() > 0)
        {
            Vertex VF, VL;
            Curve3D C;
            Placement loc;
            double f, l;
            Edge NewEdge;

            myMapEdg.Clear();

            foreach (KeyValuePair<int, NCollection_List<Shape>> itLstEdg in myMapLstEdg)
            {
                int iLst = itLstEdg.Key;
                NCollection_List<Shape> LmapEdg = myMapLstEdg.Find(iLst);

                Edge OldEdge = ShapeCasts.Edge(LmapEdg.First());

                // the first edge of the list will be replaced by the result fusion edge
                if (OldEdge.Orientation() == Reversed)
                {
                    VL = Exploration.FirstVertex(ShapeCasts.Edge(LmapEdg.First()), true);
                    VF = Exploration.LastVertex(ShapeCasts.Edge(LmapEdg.Last()), true);
                }
                else
                {
                    VF = Exploration.FirstVertex(ShapeCasts.Edge(LmapEdg.First()), true);
                    VL = Exploration.LastVertex(ShapeCasts.Edge(LmapEdg.Last()), true);
                }
                C = Tool.Curve(OldEdge, out loc, out f, out l)!;

                if (!loc.IsIdentity)
                {
                    C = (Curve3D)C.Transformed(loc.Transformation());
                }
                // if the curve is trimmed we get the basis curve to fit the new vertices
                // otherwise the makeedge will fail.
                if (C.GetType() == typeof(TrimmedCurve3D))
                {
                    C = ((TrimmedCurve3D)C).BasisCurve();
                }

                if (myConcatBSpl)
                {
                    // Prepare common BSpline curve
                    if (C.GetType() == typeof(BSplineCurve3D))
                    {
                        var anEdgIter = new NCollection_List<Shape>.Iterator(LmapEdg);
                        var aTC = new TrimmedCurve3D(C, f, l);
                        var Concat = new CompCurveToBSplineCurve(aTC);

                        anEdgIter.Next();
                        for (; anEdgIter.More(); anEdgIter.Next())
                        {
                            Curve3D aC = Tool.Curve(ShapeCasts.Edge(anEdgIter.Value()), out f, out l)!;
                            aTC = new TrimmedCurve3D(aC, f, l);
                            if (!Concat.Add(aTC, global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                // cannot merge curves
                                throw new ConstructionErrorException("FuseEdges : Concatenation failed");
                            }
                        }
                        // the concatenation is discarded at once: read it unfrozen (MakeClosedCurve may change it)
                        C = Concat.InternalBSplineCurve()!;
                    }
                }

                var ME = new BRepLib_MakeEdge();

                bool isBSpline = C.GetType() == typeof(BSplineCurve3D);

                if (VF.IsSame(VL) && isBSpline)
                {
                    // closed edge
                    f = C.FirstParameter();
                    l = C.LastParameter();
                    Point3D aPf = C.Value(f);
                    Point3D aPl = C.Value(l);
                    if (aPf.Distance(aPl) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        throw new ConstructionErrorException("FuseEdges : Curve must be closed");
                    }
                    Point3D PF = Tool.Pnt(VF);
                    if (PF.Distance(aPf) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        MakeClosedCurve(ref C, PF, ref f, ref l);
                    }
                    //
                    ME.Init(C, VF, VL, f, l);
                    if (!ME.IsDone())
                    {
                        throw new ConstructionErrorException("FuseEdges : MakeEdge failed for closed curve");
                    }
                }
                else
                {
                    ME.Init(C, VF, VL);
                }
                //      BRepLib_MakeEdge ME(C,VF,VL);

                if (!ME.IsDone())
                {
                    // the MakeEdge has fails, one reason could be that the new Vertices are outside
                    // the curve which is not infinite and limited to old vertices
                    // we try to use ExtendCurveToPoint, then rebuild the NewEdge

                    BoundedCurve3D? ExtC = C.Copy() as BoundedCurve3D;
                    if (ExtC is not null)
                    {
                        Point3D PF = Tool.Pnt(VF);
                        Point3D PL = Tool.Pnt(VL);
                        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref ExtC, PF, 1, false);
                        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref ExtC, PL, 1, true);

                        ME.Init(ExtC, VF, VL);
                        if (!ME.IsDone())
                        {
                            throw new ConstructionErrorException("FuseEdges : Fusion failed");
                        }
                    }
                    else
                    {
                        throw new ConstructionErrorException("FuseEdges : Fusion failed");
                    }
                }

                NewEdge = ME.InternalEdge();

                if (UpdatePCurve(OldEdge, NewEdge, LmapEdg))
                {
                    myMapEdg.Bind(iLst, NewEdge);
                }
            }

            myResultEdgesDone = true;
        }
    }

    /// <summary>Performs the fusion (on demand from the accessors).</summary>
    public void Perform()
    {
        if (!myResultEdgesDone)
        {
            BuildListResultEdges();
        }

        // if we have fused edges
        if (myMapEdg.Extent() > 0)
        {
            NCollection_List<Shape> EmptyList = new(), EdgeToSubs = new();
            var Bsub = new Substitution();

            foreach (KeyValuePair<int, NCollection_List<Shape>> itLstEdg in myMapLstEdg)
            {
                int iLst = itLstEdg.Key;
                if (!myMapEdg.IsBound(iLst))
                {
                    continue;
                }
                NCollection_List<Shape> LmapEdg = myMapLstEdg.Find(iLst);
                var itEdg = new NCollection_List<Shape>.Iterator();

                EdgeToSubs.Clear();
                Edge OldEdge = ShapeCasts.Edge(LmapEdg.First());

                EdgeToSubs.Append(myMapEdg[iLst]);
                Bsub.Substitute(OldEdge, EdgeToSubs);

                itEdg.Initialize(LmapEdg);

                // the other edges of the list will be removed
                while (itEdg.More())
                {
                    if (!OldEdge.IsSame(ShapeCasts.Edge(itEdg.Value())))
                    {
                        Bsub.Substitute(itEdg.Value(), EmptyList);
                    }
                    itEdg.Next();
                }
            }

            // perform the effective substitution
            Bsub.Build(myShape);

            // before copying the resulting shape, map the modified faces into myMapFaces
            var exp = new Explorer(myShape, ShapeType.Face);

            for (; exp.More(); exp.Next())
            {
                Shape facecur = exp.Current();
                if (Bsub.IsCopied(facecur))
                {
                    myMapFaces.Bind(facecur, Bsub.InternalCopy(facecur).First());
                }
            }

            if (Bsub.IsCopied(myShape))
            {
                myShape = Bsub.InternalCopy(myShape).First();
            }
        }

        myShapeDone = true;
    }

    // giving one edge, build the list of connex edges which have vertices that have only two connex edges. All the
    // edges that are added to the list must be added also to the mapUniq, in order for the caller to not treat again
    // these edges. This list is always oriented in the "Forward" direction.
    private void BuildListConnexEdge(in Shape theEdge,
                                     NCollection_Map<Shape> theMapUniq,
                                     NCollection_List<Shape> theLstEdg)
    {
        Vertex VF, VL;

        VL = Exploration.LastVertex(ShapeCasts.Edge(theEdge), true);
        Shape edgeconnex = new();
        Shape edgecur = theEdge;
        theLstEdg.Clear();
        theLstEdg.Append(edgecur);
        theMapUniq.Add(edgecur);
        Orientation ori2;

        // we first build the list of edges connex to edgecur by looking from the last Vertex VL
        while (NextConnexEdge(VL, edgecur, ref edgeconnex))
        {
            if (theMapUniq.Contains(edgeconnex))
            {
                break;
            }
            theLstEdg.Append(edgeconnex);
            edgecur = edgeconnex;
            // here take care about internal or external edges. It is non-sense to build
            // the connex list with such edges.
            ori2 = edgecur.Orientation();
            if (ori2 == External || ori2 == Internal)
            {
                break;
            }
            VL = Exploration.LastVertex(ShapeCasts.Edge(edgecur), true);
            theMapUniq.Add(edgecur);
        }

        edgecur = theEdge;
        VF = Exploration.FirstVertex(ShapeCasts.Edge(theEdge), true);

        // then we build the list of edges connex to edgecur by looking from the first Vertex VF
        while (NextConnexEdge(VF, edgecur, ref edgeconnex))
        {
            if (theMapUniq.Contains(edgeconnex))
            {
                break;
            }
            theLstEdg.Prepend(edgeconnex);
            edgecur = edgeconnex;
            // here take care about internal or external edges. It is non-sense to build
            // the connex list with such edges.
            ori2 = edgecur.Orientation();
            if (ori2 == External || ori2 == Internal)
            {
                break;
            }
            VF = Exploration.FirstVertex(ShapeCasts.Edge(edgecur), true);
            theMapUniq.Add(edgecur);
        }
    }

    // Look for an edge connex to theEdge at theVertex. the connex edge must satisfies the following criteria :
    //   * theVertex must have exactly 2 connex edges.
    //   * the 2 connex edges must have exactly the 2 same connex faces
    //   * the 2 connex edges must lie on the same support.
    // theEdgeConnex is written only when theVertex has two edges, as in OCCT.
    private bool NextConnexEdge(in Vertex theVertex, in Shape theEdge, ref Shape theEdgeConnex)
    {
        NCollection_List<Shape> LmapEdg = myMapVerLstEdg.FindFromKey(theVertex);
        bool HasConnex = true;
        var itEdg = new NCollection_List<Shape>.Iterator();
        var itFac1 = new NCollection_List<Shape>.Iterator();
        var itFac2 = new NCollection_List<Shape>.Iterator();

        // 1st condition
        if (LmapEdg.Extent() == 2)
        {
            itEdg.Initialize(LmapEdg);
            theEdgeConnex = itEdg.Value();
            if (theEdge.IsSame(theEdgeConnex))
            {
                itEdg.Next();
                theEdgeConnex = itEdg.Value();
            }

            if (myAvoidEdg.Contains(theEdgeConnex))
            {
                HasConnex = false; // edge is not allowed to be fused
            }

            // 2nd condition
            if (HasConnex)
            {
                NCollection_List<Shape> LmapFac1 = myMapEdgLstFac.FindFromKey(theEdge);
                NCollection_List<Shape> LmapFac2 = myMapEdgLstFac.FindFromKey(theEdgeConnex);

                if (LmapFac1.Extent() == LmapFac2.Extent() && LmapFac1.Extent() < 3)
                {
                    itFac1.Initialize(LmapFac1);

                    // for each face in LmapFac1 we look in LmapFac2 if it exists
                    while (itFac1.More() && HasConnex)
                    {
                        Shape face1 = itFac1.Value();
                        for (itFac2.Initialize(LmapFac2); itFac2.More(); itFac2.Next())
                        {
                            Shape face2 = itFac2.Value();
                            HasConnex = false;
                            if (face1.IsSame(face2))
                            {
                                HasConnex = true;
                                break;
                            }
                        }
                        itFac1.Next();
                    }

                    // 3rd condition : same support
                    if (HasConnex)
                    {
                        HasConnex = SameSupport(ShapeCasts.Edge(theEdge), ShapeCasts.Edge(theEdgeConnex));
                    }
                }
                else
                {
                    HasConnex = false;
                }
            }
        }
        else
        {
            HasConnex = false;
        }

        return HasConnex;
    }

    private bool SameSupport(in Edge E1, in Edge E2)
    {
        if (E1.IsNull || E2.IsNull)
        {
            return false;
        }

        Curve3D? C1, C2;
        Placement loc;
        double f1, l1, f2, l2;
        Type typC1, typC2;

        C1 = Tool.Curve(E1, out loc, out f1, out l1);
        // modified by NIZNHY-PKV Mon Nov 15 16:24:10 1999
        // degenerated edges has no 3D curve
        if (C1 is null)
        {
            return false;
        }

        if (!loc.IsIdentity)
        {
            Geometry3D GG1 = C1.Transformed(loc.Transformation());
            C1 = (Curve3D)GG1;
        }
        C2 = Tool.Curve(E2, out loc, out f2, out l2);
        // modified by NIZNHY-PKV Mon Nov 15 16:24:38 1999
        // degenerated edges has no 3D curve
        if (C2 is null)
        {
            return false;
        }

        if (!loc.IsIdentity)
        {
            Geometry3D GG2 = C2.Transformed(loc.Transformation());
            C2 = (Curve3D)GG2;
        }

        typC1 = C1.GetType();
        typC2 = C2.GetType();

        if (typC1 == typeof(TrimmedCurve3D))
        {
            C1 = ((TrimmedCurve3D)C1).BasisCurve();
            typC1 = C1.GetType();
        }

        if (typC2 == typeof(TrimmedCurve3D))
        {
            C2 = ((TrimmedCurve3D)C2).BasisCurve();
            typC2 = C2.GetType();
        }

        if (typC1 != typC2)
        {
            return false;
        }

        if (typC1 != typeof(LineCurve3D) && typC1 != typeof(CircleCurve3D)
            && typC1 != typeof(EllipseCurve3D) && typC1 != typeof(BSplineCurve3D)
            && typC1 != typeof(BezierCurve3D))
        {
            return false;
        }

        // On a presomption de confusion
        double tollin = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolang = global::Kerf.Engine.Foundation.Precision.Angular();
        if (typC1 == typeof(LineCurve3D))
        {
            Line3D li1 = ((LineCurve3D)C1).Lin();
            Line3D li2 = ((LineCurve3D)C2).Lin();
            Direction3D dir1 = li1.Direction();
            Direction3D dir2 = li2.Direction();

            if (dir1.IsParallel(dir2, tolang))
            {
                // on verifie que l'on n'a pas de cas degenere. Par exemple E1 et E2 connexes
                // mais bouclant l'un sur l'autre (cas tres rare)
                Point3D pf1 = Tool.Pnt(Exploration.FirstVertex(E1, true));
                Point3D pl1 = Tool.Pnt(Exploration.LastVertex(E1, true));
                Point3D pf2 = Tool.Pnt(Exploration.FirstVertex(E2, true));
                Point3D pl2 = Tool.Pnt(Exploration.LastVertex(E2, true));
                return pl1.Distance(pf2) >= tollin || pl2.Distance(pf1) >= tollin;
            }
            return false;
        }
        else if (typC1 == typeof(CircleCurve3D))
        {
            Circle3D ci1 = ((CircleCurve3D)C1).Circ();
            Circle3D ci2 = ((CircleCurve3D)C2).Circ();
            return Math.Abs(ci1.Radius() - ci2.Radius()) <= tollin
                   && ci1.Location().SquareDistance(ci2.Location()) <= tollin * tollin
                   && ci1.Axis().IsParallel(ci2.Axis(), tolang);
        }
        else if (typC1 == typeof(EllipseCurve3D))
        {
            Ellipse3D ci1 = ((EllipseCurve3D)C1).Elips();
            Ellipse3D ci2 = ((EllipseCurve3D)C2).Elips();

            return Math.Abs(ci1.MajorRadius() - ci2.MajorRadius()) <= tollin
                   && Math.Abs(ci1.MinorRadius() - ci2.MinorRadius()) <= tollin
                   && ci1.Location().SquareDistance(ci2.Location()) <= tollin * tollin
                   && ci1.Axis().IsParallel(ci2.Axis(), tolang);
        }
        else if (typC1 == typeof(BSplineCurve3D))
        {
            if (myConcatBSpl)
            {
                // Check G1 continuity
                Point3D aPf1, aPl1, aPf2, aPl2;
                Vector3D aDf1, aDl1, aDf2, aDl2;

                C1.D1(f1, out aPf1, out aDf1);
                C1.D1(l1, out aPl1, out aDl1);

                C2.D1(f2, out aPf2, out aDf2);
                C2.D1(l2, out aPl2, out aDl2);

                if (aPl1.Distance(aPf2) <= tollin && aDl1.IsParallel(aDf2, tolang))
                {
                    return true;
                }
                if (aPl2.Distance(aPf1) <= tollin && aDl2.IsParallel(aDf1, tolang))
                {
                    return true;
                }
                if (aPf1.Distance(aPf2) <= tollin && aDf1.IsParallel(aDf2, tolang))
                {
                    return true;
                }
                if (aPl1.Distance(aPl2) <= tollin && aDl1.IsParallel(aDl2, tolang))
                {
                    return true;
                }
            }
            // we must ensure that before fuse two bsplines, the end of one curve does not
            // corresponds to the beginning of the second.
            // we could add a special treatment for periodic bspline. This is not done for the moment.
            if (Math.Abs(f2 - l1) > tollin && Math.Abs(f1 - l2) > tollin)
            {
                return false;
            }

            BSplineCurve3D B1 = (BSplineCurve3D)C1;
            BSplineCurve3D B2 = (BSplineCurve3D)C2;

            int nbpoles = B1.PoleCount;
            if (nbpoles != B2.PoleCount)
            {
                return false;
            }

            int nbknots = B1.KnotCount;
            if (nbknots != B2.KnotCount)
            {
                return false;
            }

            NCollection_Array1<Point3D> P1 = B1.InternalPoles();
            NCollection_Array1<Point3D> P2 = B2.InternalPoles();

            double tol3d = Tool.Tolerance(E1);
            for (int p = 1; p <= nbpoles; p++)
            {
                if ((P1[p]).Distance(P2[p]) > tol3d)
                {
                    return false;
                }
            }

            NCollection_Array1<double> K1 = B1.InternalKnots();
            NCollection_Array1<double> K2 = B2.InternalKnots();

            NCollection_Array1<int> M1 = B1.InternalMultiplicities();
            NCollection_Array1<int> M2 = B2.InternalMultiplicities();

            for (int k = 1; k <= nbknots; k++)
            {
                // OCCT compares the signed difference only: a knot of B1 smaller than B2's by any amount passes.
                if ((K1[k] - K2[k]) > tollin)
                {
                    return false;
                }
                if (Math.Abs(M1[k] - M2[k]) > tollin)
                {
                    return false;
                }
            }

            if (!B1.IsRational)
            {
                if (B2.IsRational)
                {
                    return false;
                }
            }
            else
            {
                if (!B2.IsRational)
                {
                    return false;
                }
            }

            if (B1.IsRational)
            {
                NCollection_Array1<double> W1 = B1.InternalWeightsArray();
                NCollection_Array1<double> W2 = B2.InternalWeightsArray();

                for (int w = 1; w <= nbpoles; w++)
                {
                    if (Math.Abs(W1[w] - W2[w]) > tollin)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
        else if (typC1 == typeof(BezierCurve3D))
        {
            // we must ensure that before fuse two bezier, the end of one curve does not
            // corresponds to the beginning of the second.
            if (Math.Abs(f2 - l1) > tollin && Math.Abs(f1 - l2) > tollin)
            {
                return false;
            }

            BezierCurve3D B1 = (BezierCurve3D)C1;
            BezierCurve3D B2 = (BezierCurve3D)C2;

            int nbpoles = B1.PoleCount;
            if (nbpoles != B2.PoleCount)
            {
                return false;
            }

            NCollection_Array1<Point3D> P1 = B1.InternalPoles();
            NCollection_Array1<Point3D> P2 = B2.InternalPoles();

            for (int p = 1; p <= nbpoles; p++)
            {
                if ((P1[p]).Distance(P2[p]) > tollin)
                {
                    return false;
                }
            }

            if (!B1.IsRational)
            {
                if (B2.IsRational)
                {
                    return false;
                }
            }
            else
            {
                if (!B2.IsRational)
                {
                    return false;
                }
            }

            if (B1.IsRational)
            {
                NCollection_Array1<double> W1 = B1.InternalWeightsArray();
                NCollection_Array1<double> W2 = B2.InternalWeightsArray();

                for (int w = 1; w <= nbpoles; w++)
                {
                    if (Math.Abs(W1[w] - W2[w]) > tollin)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
        return false;
    }

    private bool UpdatePCurve(in Edge theOldEdge, in Edge theNewEdge, NCollection_List<Shape> theLstEdg)
    {
        // get the pcurve of edge to substitute (theOldEdge)
        // using CurveOnSurface with Index syntax, so we can update the pcurve
        // on all the faces
        BrepBuilder B = new();
        Curve2D? Curv2d = null;
        Surface? Surf = null;
        Placement loc = new(), locbid;
        double ef, el, cf = 0.0, cl = 0.0;
        int iedg = 1;

        // take care that we want only Pcurve that maps on the surface where the 3D edges lies.
        NCollection_List<Shape> LmapFac = myMapEdgLstFac.FindFromKey(theOldEdge);

        Tool.CurveOnSurface(theOldEdge, ref Curv2d, ref Surf, ref loc, ref cf, ref cl, iedg);

        bool pcurveRebuilt = false;

        while (Curv2d is not null)
        {
            // we look for a face that contains the same surface as the one that cames
            // from CurveOnSurface
            bool SameSurf = false;
            var itFac = new NCollection_List<Shape>.Iterator();

            for (itFac.Initialize(LmapFac); itFac.More(); itFac.Next())
            {
                Shape face = itFac.Value();
                Surface? S = Tool.Surface(ShapeCasts.Face(face), out locbid);
                if (ReferenceEquals(S, Surf))
                {
                    SameSurf = true;
                    break;
                }
            }

            if (SameSurf)
            {
                Tool.Range(theNewEdge, out ef, out el);

                // modified by NIZNHY-PKV Mon Nov 15 14:59:48 1999 _from
                Edge aFEdge = theOldEdge;
                aFEdge.Orientation(Forward);

                // take care if the edge is on the closing curve of a closed surface. In that case
                // we get the second pcurve by reversing the edge and calling again CurveOnSurface method

                Tool.CurveOnSurface(aFEdge, ref Curv2d, ref Surf, ref loc, ref cf, ref cl, iedg);
                if (Tool.IsClosed(theOldEdge, Surf, loc))
                {
                    aFEdge.Reverse();
                    Face aFFace = ShapeCasts.Face(itFac.Value());
                    aFFace.Orientation(Forward);
                    Curve2D Curv2dR = Tool.CurveOnSurface(aFEdge, aFFace, out cf, out cl)!;
                    if (Curv2d!.GetType() == typeof(TrimmedCurve2D))
                    {
                        Curv2d = ((TrimmedCurve2D)Curv2d).BasisCurve();
                    }
                    if (Curv2dR.GetType() == typeof(TrimmedCurve2D))
                    {
                        Curv2dR = ((TrimmedCurve2D)Curv2dR).BasisCurve();
                    }

                    B.UpdateEdge(theNewEdge, Curv2d, Curv2dR, Surf, loc, Tool.Tolerance(theNewEdge));
                }
                else
                {
                    // update the new edge
                    if (Curv2d!.GetType() == typeof(TrimmedCurve2D))
                    {
                        Curv2d = ((TrimmedCurve2D)Curv2d).BasisCurve();
                    }
                    double f, l;
                    f = Curv2d.FirstParameter();
                    l = Curv2d.LastParameter();
                    if (l - f + 2.0 * Epsilon(l - f) < el - ef)
                    {
                        BoundedCurve2D? bcurve = Curv2d as BoundedCurve2D;
                        if (bcurve is null)
                        {
                            bcurve = new TrimmedCurve2D(Curv2d, cf, cl);
                        }
                        var Concat = new CompCurveToBSplineCurve2D(bcurve);
                        var iter = new NCollection_List<Shape>.Iterator(theLstEdg);
                        iter.Next();
                        for (; iter.More(); iter.Next())
                        {
                            Edge E = ShapeCasts.Edge(iter.Value());
                            Curve2D C = Tool.CurveOnSurface(E, Surf, loc, out double first, out double last)!;
                            BoundedCurve2D? BC = C as BoundedCurve2D;
                            if (BC is null)
                            {
                                BC = new TrimmedCurve2D(C, first, last);
                            }
                            if (!Concat.Add(BC, global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            {
                                // cannot merge pcurves
                                return false;
                            }
                        }
                        // the concatenation is discarded at once: read it unfrozen (its knots may be reset below)
                        Curv2d = Concat.InternalBSplineCurve()!;

                        // check that new curve 2d is same range
                        double first2 = Curv2d.FirstParameter();
                        double last2 = Curv2d.LastParameter();
                        if (Math.Abs(first2 - ef) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                            || Math.Abs(last2 - el) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            BSplineCurve2D bc = (BSplineCurve2D)Curv2d;
                            NCollection_Array1<double> Knots = new(bc.InternalKnots());
                            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(ef, el, Knots);
                            bc.SetKnots(Knots);
                        }
                        pcurveRebuilt = true;
                    }

                    B.UpdateEdge(theNewEdge, Curv2d, Surf, loc, Tool.Tolerance(theNewEdge));
                }

                // the old pcurve range is cf,cl. The new 3d edge range is ef,el. if we want
                // the pcurve to be samerange we must adapt the parameter of the edge. In general
                // cases cf=ef and cl=el expect for periodic curve if the new edge is going over
                // the value 0.
                if (!pcurveRebuilt)
                {
                    if (theOldEdge.Orientation() == Reversed)
                    {
                        B.Range(theNewEdge, cl - el + ef, cl);
                    }
                    else
                    {
                        B.Range(theNewEdge, cf, cf + el - ef);
                    }
                }
            }

            // get next pcurve
            iedg++;
            Tool.CurveOnSurface(theOldEdge, ref Curv2d, ref Surf, ref loc, ref cf, ref cl, iedg);
        }

        if (pcurveRebuilt)
        {
            // force same parameter
            B.SameParameter(theNewEdge, false);
            BRepLib.SameParameter(theNewEdge, Tool.Tolerance(theNewEdge));
        }

        return true;
    }
}
