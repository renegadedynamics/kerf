// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.Extrema_ExtFlag;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Edges and wires lying on a shape, bound to the faces (or edges) they lie on, and how their vertices lie on the
/// shape's vertices and edges: the input of <see cref="LocOpe_Spliter"/>.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, binding the edges (<see cref="BindAll"/>, which the edge iterator runs) writes into them and into the
/// shape: pcurves and same-parameter on the edges, raised vertex tolerances, and the edges crossing others are split.
/// So the shape and every edge given are worked on as copies when frozen (decision 1013): <see cref="LocOpe_Spliter"/>
/// goes on with the same copies, and gives back the caller's unchanged sub-shapes at its end. A WiresOnShape serves one
/// splitting.
/// </para>
/// <para>
/// The public accessors answer with the caller's shapes: an edge, face or vertex the operation copied is given back as
/// the caller's own; the edges it made (splits of a given edge) are its working shapes, frozen with the spliter's
/// result. Ported code reads the <c>Internal</c> forms, which give the working shapes.
/// </para>
/// </remarks>
public sealed class LocOpe_WiresOnShape : Transient
{
    private TopoDS_Shape myShape = new();
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myMapEF = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_Map<TopoDS_Shape> myFacesWithSection = new(TopTools_ShapeMapHasher.Instance);
    private bool myCheckInterior;
    // OCCT iterates this map keyed by the shapes' addresses (in BindAll); the port in insertion order
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);
    private bool myDone;
    private int myIndex;
    private LocOpe_WorkingShapes myWork;

    public LocOpe_WiresOnShape(in TopoDS_Shape S)
    {
        myWork = new LocOpe_WorkingShapes();
        myShape = myWork.Working(S);
        myCheckInterior = true;
        myDone = false;
        myIndex = -1;
    }

    /// <summary>For ported operations that own the working shapes (LocOpe_Gluer): <paramref name="S"/> is one.</summary>
    internal LocOpe_WiresOnShape(in TopoDS_Shape S, LocOpe_WorkingShapes theWork)
    {
        myWork = theWork;
        myShape = theWork.Working(S);
        myCheckInterior = true;
        myDone = false;
        myIndex = -1;
    }

    /// <summary>The working shapes of the splitting (shared with the spliter).</summary>
    internal LocOpe_WorkingShapes WorkingShapes() => myWork;

    /// <summary>The shape, as worked on (its copy if it was frozen).</summary>
    internal TopoDS_Shape InternalShape() => myShape;

    public void Init(in TopoDS_Shape S)
    {
        myWork = new LocOpe_WorkingShapes();
        myShape = myWork.Working(S);
        myCheckInterior = true;
        myDone = false;
        myMap.Clear();
        myMapEF.Clear();
    }

    /// <summary>
    /// Adds splitting edges or wires for the whole initial shape, without saying on which faces or edges they lie: each
    /// edge is bound to the faces of the shape it lies in.
    /// </summary>
    public bool Add(NCollection_Sequence<TopoDS_Shape> theEdges)
    {
        NCollection_Sequence<TopoDS_Shape> anEdges = new();
        int i = 1, nb = theEdges.Length();
        NCollection_Array1<Box> anEdgeBoxes = new(1, nb);
        for (; i <= nb; i++)
        {
            TopoDS_Shape aCurSplit = myWork.Working(theEdges[i]);
            TopExp_Explorer anExpE = new(aCurSplit, TopAbs_EDGE);
            for (; anExpE.More(); anExpE.Next())
            {
                TopoDS_Shape aCurE = anExpE.Current();

                Box aBoxE = new();
                BRepBndLib.AddClose(aCurE, ref aBoxE);
                if (aBoxE.IsVoid)
                {
                    continue;
                }
                double aTolE = BRep_Tool.Tolerance(TopoDS.Edge(aCurE));
                aBoxE.SetGap(aTolE);
                // OCCT: the box goes at the argument's index, the edge at the next free one; a wire of several edges
                // leaves them out of step (occt-issues.md)
                anEdgeBoxes.SetValue(i, aBoxE);
                anEdges.Append(aCurE);
            }
        }
        TopExp_Explorer anExpFaces = new(myShape, TopAbs_FACE);
        int numF = 1;
        NCollection_PackedMapOfInteger anUsedEdges = new();
        for (; anExpFaces.More(); anExpFaces.Next(), numF++)
        {
            TopoDS_Face aCurF = TopoDS.Face(anExpFaces.Current());
            Box aBoxF = new();
            BRepBndLib.Add(aCurF, ref aBoxF);
            if (aBoxF.IsVoid)
            {
                continue;
            }
            BRepAdaptor_Surface anAdF = new(aCurF, false);
            BRepTopAdaptor_FClass2d? aCheckStateTool = null;

            // Initialize extrema projector once per face
            Extrema_ExtPS anExtr = new();
            anExtr.Initialize(anAdF,
                              anAdF.FirstUParameter(),
                              anAdF.LastUParameter(),
                              anAdF.FirstVParameter(),
                              anAdF.LastVParameter(),
                              global::Kerf.Engine.Foundation.Precision.Confusion(),
                              global::Kerf.Engine.Foundation.Precision.Confusion());
            anExtr.SetFlag(Extrema_ExtFlag_MIN);

            i = 1;
            nb = anEdgeBoxes.Length();
            for (; i <= nb; i++)
            {
                if (anUsedEdges.Contains(i))
                {
                    continue;
                }

                if (aBoxF.IsOut(anEdgeBoxes[i]))
                {
                    continue;
                }

                TopoDS_Edge aCurE = TopoDS.Edge(anEdges[i]);

                Geom_Curve? aC = BRep_Tool.Curve(aCurE, out double aF, out double aL);
                if (aC is null)
                {
                    anUsedEdges.Add(i);
                    continue;
                }
                Point3D aP = aC.Value((aF + aL) * 0.5);
                anExtr.Perform(aP);

                if (!anExtr.IsDone() || anExtr.NbExt() == 0)
                {
                    continue;
                }
                double aTolE = BRep_Tool.Tolerance(TopoDS.Edge(aCurE));
                double aTol2 = (aTolE + global::Kerf.Engine.Foundation.Precision.Confusion()) * (aTolE + global::Kerf.Engine.Foundation.Precision.Confusion());
                int n = 1;
                for (; n <= anExtr.NbExt(); n++)
                {
                    double aDist2 = anExtr.SquareDistance(n);
                    if (aDist2 > aTol2)
                    {
                        continue;
                    }
                    Extrema_POnSurf aPS = anExtr.Point(n);
                    aPS.Parameter(out double aU, out double aV);

                    aCheckStateTool ??= new BRepTopAdaptor_FClass2d(aCurF, global::Kerf.Engine.Foundation.Precision.PConfusion());
                    if (aCheckStateTool.Perform(new Point2D(aU, aV)) == TopAbs_IN)
                    {
                        bind(aCurE, aCurF);
                        anUsedEdges.Add(i);
                    }
                }
            }
        }
        _ = numF;
        return !anUsedEdges.IsEmpty();
    }

    /// <summary>Sets the flag of check of internal intersections (true by default).</summary>
    public void SetCheckInterior(bool ToCheckInterior)
    {
        myCheckInterior = ToCheckInterior;
    }

    public void Bind(in TopoDS_Wire W, in TopoDS_Face F)
    {
        TopoDS_Shape aW = myWork.Working(W);
        TopoDS_Face aF = TopoDS.Face(myWork.Working(F));
        for (TopExp_Explorer exp = new(aW, TopAbs_EDGE); exp.More(); exp.Next())
        {
            bind(TopoDS.Edge(exp.Current()), aF);
        }
    }

    public void Bind(in TopoDS_Compound Comp, in TopoDS_Face F)
    {
        TopoDS_Shape aComp = myWork.Working(Comp);
        TopoDS_Face aF = TopoDS.Face(myWork.Working(F));
        for (TopExp_Explorer exp = new(aComp, TopAbs_EDGE); exp.More(); exp.Next())
        {
            bind(TopoDS.Edge(exp.Current()), aF);
        }
        myFacesWithSection.Add(aF);
    }

    public void Bind(in TopoDS_Edge E, in TopoDS_Face F)
    {
        bind(TopoDS.Edge(myWork.Working(E)), TopoDS.Face(myWork.Working(F)));
    }

    // Bind(E, F) on working shapes
    private void bind(in TopoDS_Edge E, in TopoDS_Face F)
    {
        if (!myMapEF.Contains(E))
        {
            TopExp_Explorer exp = new(F, TopAbs_EDGE);
            for (; exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(E))
                {
                    break;
                }
            }
            if (!exp.More())
            {
                myMapEF.Add(E, F);
            }
        }
        else
        {
            throw new ConstructionErrorException();
        }
    }

    public void Bind(in TopoDS_Edge EfromW, in TopoDS_Edge EonFace)
    {
        myMap.Bind(myWork.Working(EfromW), myWork.Working(EonFace));
    }

    public void BindAll()
    {
        if (myDone)
        {
            return;
        }
        NCollection_Map<TopoDS_Shape> theMap = new(TopTools_ShapeMapHasher.Instance);

        // Detection des vertex a projeter ou a "binder" avec des vertex existants
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> mapV = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new(), exp2 = new();
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> ite in myMap)
        {
            TopoDS_Edge eref = TopoDS.Edge(ite.Key);
            TopoDS_Edge eimg = TopoDS.Edge(ite.Value);

            PutPCurves(eref, eimg, myShape);

            for (exp.Init(eref, TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
                if (!theMap.Contains(vtx))
                { // pas deja traite
                    for (exp2.Init(eimg, TopAbs_VERTEX); exp2.More(); exp2.Next())
                    {
                        TopoDS_Vertex vtx2 = TopoDS.Vertex(exp2.Current());
                        if (vtx2.IsSame(vtx))
                        {
                            break;
                        }
                        else if (BRepTools.Compare(vtx, vtx2))
                        {
                            mapV.Bind(vtx, vtx2);
                            break;
                        }
                    }
                    if (!exp2.More())
                    {
                        mapV.Bind(vtx, eimg);
                    }
                    theMap.Add(vtx);
                }
            }
        }

        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> ite in mapV)
        {
            myMap.Bind(ite.Key, ite.Value);
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Splits = new(TopTools_ShapeMapHasher.Instance);
        int Ind;
        NCollection_Map<TopoDS_Shape> anOverlappedEdges = new(TopTools_ShapeMapHasher.Instance);
        for (Ind = 1; Ind <= myMapEF.Extent(); Ind++)
        {
            TopoDS_Edge edg = TopoDS.Edge(myMapEF.FindKey(Ind));
            TopoDS_Face fac = TopoDS.Face(myMapEF[Ind]);

            PutPCurve(edg, fac);
            Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(edg, fac, out _, out _);
            if (aPCurve is null)
            {
                continue;
            }

            if (myCheckInterior)
            {
                bool isOverlapped = false;
                FindInternalIntersections(edg, fac, Splits, ref isOverlapped);
                if (isOverlapped)
                {
                    anOverlappedEdges.Add(edg);
                }
            }
        }

        for (Ind = 1; Ind <= Splits.Extent(); Ind++)
        {
            TopoDS_Shape anEdge = Splits.FindKey(Ind);
            if (anOverlappedEdges.Contains(anEdge))
            {
                continue;
            }
            TopoDS_Shape aFace = myMapEF.FindFromKey(anEdge);
            // Remove "anEdge" from "myMapEF"
            myMapEF.RemoveKey(anEdge);
            NCollection_List<TopoDS_Shape>.Iterator itl = new(Splits[Ind]);
            for (; itl.More(); itl.Next())
            {
                myMapEF.Add(itl.Value(), aFace);
            }
        }

        NCollection_DataMap<TopoDS_Shape, double> aVertParam = new(TopTools_ShapeMapHasher.Instance);

        for (Ind = 1; Ind <= myMapEF.Extent(); Ind++)
        {
            TopoDS_Edge edg = TopoDS.Edge(myMapEF.FindKey(Ind));
            TopoDS_Face fac = TopoDS.Face(myMapEF[Ind]);
            // JAG 02.02.96 : On verifie les pcurves...

            for (exp.Init(edg, TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
                if (theMap.Contains(vtx))
                {
                    continue;
                }

                double vtx_param = BRep_Tool.Parameter(vtx, edg);
                BRepAdaptor_Curve2d BAcurve2d = new(edg, fac);

                Point2D p2d = BAcurve2d.Curve() is not null
                                   ? BAcurve2d.Value(vtx_param)
                                   : new Point2D(global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());

                TopoDS_Edge Epro = new();
                double prm = global::Kerf.Engine.Foundation.Precision.Infinite();
                bool isProjected = myMap.IsBound(vtx);

                // if vertex was already projected on the current edge on the previous face
                // it is necessary to check tolerance of the vertex in the 2D space on the current
                // face without projection and update tolerance of vertex if it is necessary
                if (isProjected)
                {
                    TopoDS_Shape aSh = myMap.Find(vtx);
                    if (aSh.ShapeType() != TopAbs_EDGE)
                    {
                        continue;
                    }
                    Epro = TopoDS.Edge(myMap.Find(vtx));
                    if (aVertParam.IsBound(vtx))
                    {
                        prm = aVertParam.Find(vtx);
                    }
                }
                bool ok = Project(vtx, p2d, fac, ref Epro, ref prm);
                if (ok && !isProjected)
                {
                    for (exp2.Init(Epro, TopAbs_VERTEX); exp2.More(); exp2.Next())
                    {
                        TopoDS_Vertex vtx2 = TopoDS.Vertex(exp2.Current());
                        if (vtx2.IsSame(vtx))
                        {
                            myMap.Bind(vtx, vtx2);
                            theMap.Add(vtx);
                            break;
                        }
                        else if (BRepTools.Compare(vtx, vtx2))
                        {
                            BRep_Tool.Range(Epro, fac, out double aF1, out double aL1);
                            if (!BRep_Tool.Degenerated(Epro)
                                && (Math.Abs(prm - aF1) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                                    || Math.Abs(prm - aL1) <= global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            {
                                myMap.Bind(vtx, vtx2);
                                theMap.Add(vtx);
                                break;
                            }
                        }
                    }
                    if (!exp2.More())
                    {
                        myMap.Bind(vtx, Epro);
                        aVertParam.Bind(vtx, prm);
                    }
                }
            }
        }

        //  Modified by Sergey KHROMOV - Mon Feb 12 16:26:50 2001 Begin
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> ite in myMap)
        {
            if (ite.Key.ShapeType() == TopAbs_EDGE)
            {
                myMapEF.Add(ite.Key, ite.Value);
            }
        }
        //  Modified by Sergey KHROMOV - Mon Feb 12 16:26:52 2001 End

        myDone = true;
    }

    public bool IsDone() => myDone;

    public void InitEdgeIterator()
    {
        BindAll();
        myIndex = 1;
    }

    public bool MoreEdge() => myIndex <= myMapEF.Extent();

    /// <summary>The current edge (the caller's edge if it was copied).</summary>
    public TopoDS_Edge Edge() => TopoDS.Edge(myWork.Original(InternalEdge()));

    /// <summary><see cref="Edge"/> as worked on, for ported code.</summary>
    internal TopoDS_Edge InternalEdge() => TopoDS.Edge(myMapEF.FindKey(myIndex));

    /// <summary>Returns the face of the shape on which the current edge is projected (the caller's face).</summary>
    public TopoDS_Face OnFace() => TopoDS.Face(myWork.Original(InternalOnFace()));

    /// <summary><see cref="OnFace"/> as worked on, for ported code.</summary>
    internal TopoDS_Face InternalOnFace() => TopoDS.Face(myMapEF[myIndex]);

    /// <summary>
    /// If the current edge is projected on an edge, returns true and sets the edge (the caller's edge) in
    /// <paramref name="E"/>. Otherwise, returns false.
    /// </summary>
    public bool OnEdge(out TopoDS_Edge E)
    {
        bool isOn = InternalOnEdge(out E);
        E = TopoDS.Edge(myWork.Original(E));
        return isOn;
    }

    /// <summary><see cref="OnEdge(out TopoDS_Edge)"/> as worked on, for ported code.</summary>
    internal bool InternalOnEdge(out TopoDS_Edge E)
    {
        E = new TopoDS_Edge();
        if (myMap.IsBound(myMapEF.FindKey(myIndex)))
        {
            E = TopoDS.Edge(myMap[myMapEF.FindKey(myIndex)]);
            return true;
        }
        return false;
    }

    public void NextEdge()
    {
        myIndex++;
    }

    /// <summary>
    /// If the vertex <paramref name="Vw"/> of a wire lies on a vertex of the shape, returns true and sets that vertex
    /// (the caller's) in <paramref name="Vs"/>.
    /// </summary>
    public bool OnVertex(in TopoDS_Vertex Vw, out TopoDS_Vertex Vs)
    {
        bool isOn = InternalOnVertex(TopoDS.Vertex(myWork.Copied(Vw)), out Vs);
        Vs = TopoDS.Vertex(myWork.Original(Vs));
        return isOn;
    }

    /// <summary><see cref="OnVertex"/> on working shapes, for ported code.</summary>
    internal bool InternalOnVertex(in TopoDS_Vertex Vw, out TopoDS_Vertex Vs)
    {
        Vs = new TopoDS_Vertex();
        if (myMap.IsBound(Vw))
        {
            if (myMap[Vw].ShapeType() == TopAbs_VERTEX)
            {
                Vs = TopoDS.Vertex(myMap[Vw]);
                return true;
            }
            return false;
        }
        return false;
    }

    /// <summary>
    /// If the vertex <paramref name="V"/> lies on an edge of the original shape, returns true and sets the concerned
    /// edge (the caller's) in <paramref name="Ed"/>, and the parameter on the edge in <paramref name="prm"/>. Else
    /// returns false.
    /// </summary>
    public bool OnEdge(in TopoDS_Vertex V, out TopoDS_Edge Ed, out double prm)
    {
        bool isOn = InternalOnEdge(TopoDS.Vertex(myWork.Copied(V)), out Ed, out prm);
        Ed = TopoDS.Edge(myWork.Original(Ed));
        return isOn;
    }

    /// <summary><see cref="OnEdge(in TopoDS_Vertex, out TopoDS_Edge, out double)"/> on working shapes.</summary>
    internal bool InternalOnEdge(in TopoDS_Vertex V, out TopoDS_Edge Ed, out double prm)
    {
        Ed = new TopoDS_Edge();
        prm = 0.0; // OCCT: unchanged
        if (!myMap.IsBound(V) || myMap[V].ShapeType() == TopAbs_VERTEX)
        {
            return false;
        }

        Ed = TopoDS.Edge(myMap[V]);
        prm = Project(V, Ed);
        return true;
    }

    /// <summary>
    /// <see cref="OnEdge(in TopoDS_Vertex, out TopoDS_Edge, out double)"/> for the vertex <paramref name="V"/> of the
    /// edge <paramref name="EdgeFrom"/>, projected through its pcurve where the shape's edge has no 3D curve.
    /// </summary>
    public bool OnEdge(in TopoDS_Vertex V, in TopoDS_Edge EdgeFrom, out TopoDS_Edge Ed, out double prm)
    {
        bool isOn = InternalOnEdge(TopoDS.Vertex(myWork.Copied(V)), TopoDS.Edge(myWork.Copied(EdgeFrom)), out Ed, out prm);
        Ed = TopoDS.Edge(myWork.Original(Ed));
        return isOn;
    }

    /// <summary><see cref="OnEdge(in TopoDS_Vertex, in TopoDS_Edge, out TopoDS_Edge, out double)"/> on working shapes.</summary>
    internal bool InternalOnEdge(in TopoDS_Vertex V, in TopoDS_Edge EdgeFrom, out TopoDS_Edge Ed, out double prm)
    {
        Ed = new TopoDS_Edge();
        prm = 0.0; // OCCT: unchanged
        if (!myMap.IsBound(V) || myMap[V].ShapeType() == TopAbs_VERTEX)
        {
            return false;
        }

        Ed = TopoDS.Edge(myMap[V]);
        if (!myMapEF.Contains(EdgeFrom))
        {
            return false;
        }

        TopoDS_Shape aShape = myMapEF.FindFromKey(EdgeFrom);
        Geom_Curve? aC = BRep_Tool.Curve(Ed, out _, out _);
        if (aC is null && aShape.ShapeType() == TopAbs_FACE)
        {
            TopoDS_Face aFace = TopoDS.Face(aShape);
            double vtx_param = BRep_Tool.Parameter(V, EdgeFrom);
            BRepAdaptor_Curve2d BAcurve2d = new(EdgeFrom, aFace);
            Point2D p2d = BAcurve2d.Value(vtx_param);

            prm = Project(V, p2d, Ed, aFace);
        }
        else
        {
            prm = Project(V, TopoDS.Edge(Ed));
        }

        return true;
    }

    /// <summary>Tells if the face is to be split by a section (a compound bound to it).</summary>
    public bool IsFaceWithSection(in TopoDS_Shape aFace) => InternalIsFaceWithSection(myWork.Copied(aFace));

    /// <summary><see cref="IsFaceWithSection"/> on working shapes.</summary>
    internal bool InternalIsFaceWithSection(in TopoDS_Shape aFace) => myFacesWithSection.Contains(aFace);

    //=================================================================================================

    private static bool Project(in TopoDS_Vertex V, in Point2D p2d, in TopoDS_Face F, ref TopoDS_Edge theEdge, ref double param)
    {
        double aTolV = BRep_Tool.Tolerance(V);
        double dmin = (theEdge.IsNull() ? RealLast() : aTolV * aTolV);

        bool valret = false;

        Geom_Surface aSurf = BRep_Tool.Surface(F)!;

        if (theEdge.IsNull())
        {
            Point3D toproj = BRep_Tool.Pnt(V);
            for (TopExp_Explorer exp = new(F.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                Geom_Curve? C = BRep_Tool.Curve(edg, out double f, out double l);
                double aCurDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                double aCurPar = global::Kerf.Engine.Foundation.Precision.Infinite();
                if (C is not null)
                {
                    aCurPar = Project(V, edg);
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurPar))
                    {
                        continue;
                    }
                    C.D0(aCurPar, out Point3D aCurPBound);
                    aCurDist = aCurPBound.SquareDistance(toproj);
                }

                else if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(p2d.X()))
                {
                    Geom2d_Curve? aC2d = BRep_Tool.CurveOnSurface(edg, F, out f, out l);
                    if (aC2d is null)
                    {
                        continue;
                    }

                    aCurPar = Project(V, p2d, edg, F);
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurPar))
                    {
                        continue;
                    }
                    Geom2d_Curve PC = BRep_Tool.CurveOnSurface(edg, F, out f, out l)!;
                    PC.D0(aCurPar, out Point2D aPProj);
                    aSurf.D0(aPProj.X(), aPProj.Y(), out Point3D aCurPBound);
                    aCurDist = aCurPBound.SquareDistance(toproj);
                }

                if (aCurDist < dmin)
                {
                    theEdge = edg;
                    theEdge.Orientation(edg.Orientation());
                    dmin = aCurDist;
                    param = aCurPar;
                }
            }
            if (theEdge.IsNull())
            {
                return false;
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(param))
        {
            Geom_Curve? C = BRep_Tool.Curve(theEdge, out _, out _);
            param = (C is not null ? Project(V, theEdge) : Project(V, p2d, theEdge, F));
        }

        double ttol = aTolV + BRep_Tool.Tolerance(theEdge);
        if (dmin <= ttol * ttol)
        {
            valret = true;
            GeomAdaptor_Surface adSurf = new(aSurf);

            double anUResolution = adSurf.UResolution(1.0);
            double aVResolution = adSurf.UResolution(1.0); // OCCT: UResolution for V too (occt-issues.md)

            Geom2d_Curve? aCrvBound = BRep_Tool.CurveOnSurface(theEdge, F, out _, out _);
            if (aCrvBound is not null)
            {
                aCrvBound.D0(param, out Point2D aPBound2d);

                // distance in 2D space recomputed in the 3D space in order to tolerance of vertex
                // cover gap in 2D space. For consistency with check of the validity in the  BRepCheck_Wire
                double dumax = 0.01 * (adSurf.LastUParameter() - adSurf.FirstUParameter());
                double dvmax = 0.01 * (adSurf.LastVParameter() - adSurf.FirstVParameter());

                Point2D aPcur = p2d;
                double dumin = Math.Abs(aPcur.X() - aPBound2d.X());
                double dvmin = Math.Abs(aPcur.Y() - aPBound2d.Y());
                if (dumin > dumax && adSurf.IsUPeriodic())
                {
                    double aX1 = aPBound2d.X();
                    double aShift = ShapeAnalysis.AdjustToPeriod(aX1, adSurf.FirstUParameter(), adSurf.LastUParameter());
                    aX1 += aShift;
                    aPBound2d.SetX(aX1);
                    double aX2 = p2d.X();
                    aShift = ShapeAnalysis.AdjustToPeriod(aX2, adSurf.FirstUParameter(), adSurf.LastUParameter());
                    aX2 += aShift;
                    dumin = Math.Abs(aX2 - aX1);
                    if (dumin > dumax && (Math.Abs(dumin - adSurf.UPeriod()) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    {
                        aX2 = aX1;
                        dumin = 0.0;
                    }
                    aPcur.SetX(aX2);
                }

                if (dvmin > dvmax && adSurf.IsVPeriodic())
                {
                    double aY1 = aPBound2d.Y();
                    double aShift = ShapeAnalysis.AdjustToPeriod(aY1, adSurf.FirstVParameter(), adSurf.LastVParameter());
                    aY1 += aShift;
                    aPBound2d.SetY(aY1);
                    double aY2 = p2d.Y();
                    aShift = ShapeAnalysis.AdjustToPeriod(aY2, adSurf.FirstVParameter(), adSurf.LastVParameter());
                    aY2 += aShift;
                    dvmin = Math.Abs(aY1 - aY2);
                    if (dvmin > dvmax && (Math.Abs(dvmin - adSurf.VPeriod()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        aY2 = aY1;
                        dvmin = 0.0;
                    }
                    aPcur.SetY(aY2);
                }
                double aDist3d = aTolV;
                if ((dumin > dumax) || (dvmin > dvmax))
                {
                    dumax = adSurf.UResolution(aTolV);
                    dvmax = adSurf.VResolution(aTolV);
                    double aTol2d = 2.0 * Max(dumax, dvmax);
                    double aDist2d = Max(dumin, dvmin);

                    if (aDist2d > aTol2d)
                    {
                        double aDist3d1 = aDist2d / Max(anUResolution, aVResolution);
                        if (aDist3d1 > aDist3d)
                        {
                            aDist3d = aDist3d1;
                        }
                    }
                }

                // added check by 3D the same as in the BRepCheck_Wire::SelfIntersect
                aSurf.D0(aPBound2d.X(), aPBound2d.Y(), out Point3D aPBound);
                aSurf.D0(p2d.X(), p2d.Y(), out Point3D aPV2d);
                double aDistPoints_3D = aPV2d.SquareDistance(aPBound);
                double aMaxDist = Max(aDistPoints_3D, aDist3d * aDist3d);

                BRep_Builder B = new();
                if (aTolV * aTolV < aMaxDist)
                {
                    double aNewTol = Math.Sqrt(aMaxDist);
                    B.UpdateVertex(V, aNewTol);
                }
            }
        }
        // OCCT_DEBUG_MESH trace left out
        return valret;
    }

    //=================================================================================================

    private static double Project(in TopoDS_Vertex V, in TopoDS_Edge theEdge)
    {
        Point3D toproj = BRep_Tool.Pnt(V);
        GeomAPI_ProjectPointOnCurve proj = new();

        Geom_Curve C = BRep_Tool.Curve(theEdge, out Placement Loc, out double f, out double l)!;
        if (!Loc.IsIdentity)
        {
            Geom_Geometry GG = C.Transformed(Loc.Transformation());
            C = (Geom_Curve)GG;
        }
        proj.Init(toproj, C, f, l);

        return (proj.NbPoints() > 0 ? proj.LowerDistanceParameter() : global::Kerf.Engine.Foundation.Precision.Infinite());
    }

    //=================================================================================================

    private static double Project(in TopoDS_Vertex V, in Point2D p2d, in TopoDS_Edge theEdge, in TopoDS_Face theFace)
    {
        _ = V;
        Geom2dAPI_ProjectPointOnCurve proj = new();

        Geom2d_Curve PC = BRep_Tool.CurveOnSurface(theEdge, theFace, out double f, out double l)!;

        proj.Init(p2d, PC, f, l);

        return proj.NbPoints() > 0 ? proj.LowerDistanceParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    //=================================================================================================

    // OCCT.NET: OCCT translates the projected pcurve in place; a frozen one (shared) is translated on a copy
    private static Geom2d_Curve Translatable(Geom2d_Curve theCurve) =>
        theCurve.IsFrozen ? (Geom2d_Curve)theCurve.Copy() : theCurve;

    private static void PutPCurve(in TopoDS_Edge Edg, in TopoDS_Face Fac)
    {
        BRep_Builder B = new();

        Geom_Surface S = BRep_Tool.Surface(Fac, out Placement LocFac)!;
        _ = LocFac;

        if (S is Geom_RectangularTrimmedSurface aTrimmed)
        {
            S = aTrimmed.BasisSurface();
        }

        if (S.GetType() == typeof(Geom_Plane))
        {
            return;
        }

        BRepTools.UVBounds(Fac, out double Umin, out double Umax, out double Vmin, out double Vmax);

        double f, l;

        Geom2d_Curve? aC2d = BRep_Tool.CurveOnSurface(Edg, Fac, out f, out l);
        if (aC2d is not null)
        {
            aC2d.D0((f + l) * 0.5, out Point2D p2d);
            bool IsIn = true;
            if ((p2d.X() < Umin - global::Kerf.Engine.Foundation.Precision.PConfusion()) || (p2d.X() > Umax + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                IsIn = false;
            }
            if ((p2d.Y() < Vmin - global::Kerf.Engine.Foundation.Precision.PConfusion()) || (p2d.Y() > Vmax + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                IsIn = false;
            }

            if (IsIn)
            {
                return;
            }
        }

        Geom_Curve C = BRep_Tool.Curve(Edg, out Placement Loc, out f, out l)!;
        if (!Loc.IsIdentity)
        {
            Geom_Geometry GG = C.Transformed(Loc.Transformation());
            C = (Geom_Curve)GG;
        }

        if (C.GetType() != typeof(Geom_TrimmedCurve))
        {
            C = new Geom_TrimmedCurve(C, f, l);
        }

        S = BRep_Tool.Surface(Fac)!;

        double TolFirst = -1, TolLast = -1;
        TopExp.Vertices(Edg, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        if (!V1.IsNull())
        {
            TolFirst = BRep_Tool.Tolerance(V1);
        }
        if (!V2.IsNull())
        {
            TolLast = BRep_Tool.Tolerance(V2);
        }

        double tol2d = global::Kerf.Engine.Foundation.Precision.Confusion();
        ShapeConstruct_ProjectCurveOnSurface aToolProj = new();
        aToolProj.Init(S, tol2d);

        aToolProj.InternalPerform(C, f, l, out Geom2d_Curve? C2d, TolFirst, TolLast);
        if (C2d is null)
        {
            return;
        }

        Point2D pf = C2d.Value(f);
        Point2D pl = C2d.Value(l);
        S.D0(pf.X(), pf.Y(), out Point3D PF);
        S.D0(pl.X(), pl.Y(), out Point3D PL);
        if (Edg.Orientation() == TopAbs_REVERSED)
        {
            V1 = TopExp.LastVertex(Edg);
            V1.Reverse();
        }
        else
        {
            V1 = TopExp.FirstVertex(Edg);
        }
        if (Edg.Orientation() == TopAbs_REVERSED)
        {
            V2 = TopExp.FirstVertex(Edg);
            V2.Reverse();
        }
        else
        {
            V2 = TopExp.LastVertex(Edg);
        }

        if (!V1.IsNull() && V2.IsNull())
        {
            // Handling of internal vertices
            // OCCT: V2 is null here, so BRep_Tool::Tolerance(V2) dereferences a null shape (occt-issues.md)
            double old1 = BRep_Tool.Tolerance(V1);
            double old2 = BRep_Tool.Tolerance(V2);
            Point3D pnt1 = BRep_Tool.Pnt(V1);
            Point3D pnt2 = BRep_Tool.Pnt(V2);
            double tol1 = pnt1.Distance(PF);
            double tol2 = pnt2.Distance(PL);
            B.UpdateVertex(V1, Max(old1, tol1));
            B.UpdateVertex(V2, Max(old2, tol2));
        }

        if (S.IsUPeriodic())
        {
            double up = S.UPeriod();
            double tolu = global::Kerf.Engine.Foundation.Precision.PConfusion(); // Epsilon(up);
            int nbtra = 0;
            double theUmin = Min(pf.X(), pl.X());
            double theUmax = Max(pf.X(), pl.X());

            if (theUmin < Umin - tolu)
            {
                while (theUmin < Umin - tolu)
                {
                    theUmin += up;
                    nbtra++;
                }
            }
            else if (theUmax > Umax + tolu)
            {
                while (theUmax > Umax + tolu)
                {
                    theUmax -= up;
                    nbtra--;
                }
            }

            if (nbtra != 0)
            {
                C2d = Translatable(C2d);
                C2d.Translate(new Vector2D(nbtra * up, 0.0));
            }
        }

        if (S.IsVPeriodic())
        {
            double vp = S.VPeriod();
            double tolv = global::Kerf.Engine.Foundation.Precision.PConfusion(); // Epsilon(vp);
            int nbtra = 0;
            double theVmin = Min(pf.Y(), pl.Y());
            double theVmax = Max(pf.Y(), pl.Y());

            if (theVmin < Vmin - tolv)
            {
                while (theVmin < Vmin - tolv)
                {
                    theVmin += vp;
                    theVmax += vp;
                    nbtra++;
                }
            }
            else if (theVmax > Vmax + tolv)
            {
                while (theVmax > Vmax + tolv)
                {
                    theVmax -= vp;
                    theVmin -= vp;
                    nbtra--;
                }
            }

            if (nbtra != 0)
            {
                C2d = Translatable(C2d);
                C2d.Translate(new Vector2D(0.0, nbtra * vp));
            }
        }
        B.UpdateEdge(Edg, C2d, Fac, BRep_Tool.Tolerance(Edg));

        B.SameParameter(Edg, false);
        BRepLib.SameParameter(Edg, tol2d);
    }

    //=================================================================================================

    private static void PutPCurves(in TopoDS_Edge Efrom, in TopoDS_Edge Eto, in TopoDS_Shape myShape)
    {
        NCollection_List<TopoDS_Shape> Lfaces = new();
        TopExp_Explorer exp = new(), exp2 = new();

        for (exp.Init(myShape, TopAbs_FACE); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), TopAbs_EDGE); exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(Eto))
                {
                    Lfaces.Append(exp.Current());
                }
            }
        }

        if (Lfaces.Extent() != 1 && Lfaces.Extent() != 2)
        {
            throw new ConstructionErrorException();
        }

        // soit bord libre, soit connexite entre 2 faces, eventuellement edge closed

        if (Lfaces.Extent() == 1)
        {
            return; // sera fait par PutPCurve.... on l`espere
        }

        BRep_Builder B = new();
        Geom_Surface S;
        Geom_Curve C;
        double Umin, Umax, Vmin, Vmax;
        double f, l;
        Placement Loc;

        if (!Lfaces.First().IsSame(Lfaces.Last()))
        {
            NCollection_List<TopoDS_Shape>.Iterator itl = new(Lfaces);
            for (; itl.More(); itl.Next())
            {
                TopoDS_Face Fac = TopoDS.Face(itl.Value());

                if (BRep_Tool.CurveOnSurface(Efrom, Fac, out f, out l) is not null)
                {
                    continue;
                }
                S = BRep_Tool.Surface(Fac, out _)!;
                if (S is Geom_RectangularTrimmedSurface aTrimmed)
                {
                    S = aTrimmed.BasisSurface();
                }
                if (S.GetType() == typeof(Geom_Plane))
                {
                    continue;
                }

                BRepTools.UVBounds(Fac, out Umin, out Umax, out Vmin, out Vmax);
                C = BRep_Tool.Curve(Efrom, out Loc, out f, out l)!;
                if (!Loc.IsIdentity)
                {
                    Geom_Geometry GG = C.Transformed(Loc.Transformation());
                    C = (Geom_Curve)GG;
                }

                if (C.GetType() != typeof(Geom_TrimmedCurve))
                {
                    C = new Geom_TrimmedCurve(C, f, l);
                }

                S = BRep_Tool.Surface(Fac)!;

                // Compute the tol2d
                double tol3d = Max(BRep_Tool.Tolerance(Efrom), BRep_Tool.Tolerance(Fac));
                GeomAdaptor_Surface Gas = new(S, Umin, Umax, Vmin, Vmax);
                double TolU = Gas.UResolution(tol3d);
                double TolV = Gas.VResolution(tol3d);
                double tol2d = Max(TolU, TolV);

                Geom2d_Curve? C2d = GeomProjLib.Curve2d(C, S, Umin, Umax, Vmin, Vmax, ref tol2d);
                if (C2d is null)
                {
                    return;
                }

                Point2D pf = C2d.Value(f);
                Point2D pl = C2d.Value(l);

                if (S.IsUPeriodic())
                {
                    double up = S.UPeriod();
                    double tolu = global::Kerf.Engine.Foundation.Precision.PConfusion(); // Epsilon(up);
                    int nbtra = 0;
                    double theUmin = Min(pf.X(), pl.X());
                    double theUmax = Max(pf.X(), pl.X());

                    if (theUmin < Umin - tolu)
                    {
                        while (theUmin < Umin - tolu)
                        {
                            theUmin += up;
                            theUmax += up;
                            nbtra++;
                        }
                    }
                    else if (theUmax > Umax + tolu)
                    {
                        while (theUmax > Umax + tolu)
                        {
                            theUmax -= up;
                            theUmin -= up;
                            nbtra--;
                        }
                    }
                    if (nbtra != 0)
                    {
                        C2d = Translatable(C2d);
                        C2d.Translate(new Vector2D(nbtra * up, 0.0));
                    }
                }

                if (S.IsVPeriodic())
                {
                    double vp = S.VPeriod();
                    double tolv = global::Kerf.Engine.Foundation.Precision.PConfusion(); // Epsilon(vp);
                    int nbtra = 0;
                    double theVmin = Min(pf.Y(), pl.Y());
                    double theVmax = Max(pf.Y(), pl.Y());

                    if (theVmin < Vmin - tolv)
                    {
                        while (theVmin < Vmin - tolv)
                        {
                            theVmin += vp;
                            theVmax += vp;
                            nbtra++;
                        }
                    }
                    else if (theVmax > Vmax + tolv)
                    {
                        while (theVmax > Vmax + tolv)
                        {
                            theVmax -= vp;
                            theVmin -= vp;
                            nbtra--;
                        }
                    }
                    if (nbtra != 0)
                    {
                        C2d = Translatable(C2d);
                        C2d.Translate(new Vector2D(0.0, nbtra * vp));
                    }
                }
                B.UpdateEdge(Efrom, C2d, Fac, BRep_Tool.Tolerance(Efrom));
            }
        }

        else
        {
            TopoDS_Face Fac = TopoDS.Face(Lfaces.First());
            if (!BRep_Tool.IsClosed(Eto, Fac))
            {
                throw new ConstructionErrorException();
            }

            TopoDS_Shape aLocalE = Efrom.Oriented(TopAbs_FORWARD);
            TopoDS_Shape aLocalF = Fac.Oriented(TopAbs_FORWARD);
            Geom2d_Curve? c2dff = BRep_Tool.CurveOnSurface(TopoDS.Edge(aLocalE), TopoDS.Face(aLocalF), out f, out l);

            aLocalE = Efrom.Oriented(TopAbs_REVERSED);
            aLocalF = Fac.Oriented(TopAbs_FORWARD);
            Geom2d_Curve? c2dfr = BRep_Tool.CurveOnSurface(TopoDS.Edge(aLocalE), TopoDS.Face(aLocalF), out f, out l);

            aLocalE = Eto.Oriented(TopAbs_FORWARD);
            aLocalF = Fac.Oriented(TopAbs_FORWARD);
            Geom2d_Curve c2dtf = BRep_Tool.CurveOnSurface(TopoDS.Edge(aLocalE), TopoDS.Face(aLocalF), out f, out l)!;

            aLocalE = Eto.Oriented(TopAbs_REVERSED);
            aLocalF = Fac.Oriented(TopAbs_FORWARD);
            Geom2d_Curve c2dtr = BRep_Tool.CurveOnSurface(TopoDS.Edge(aLocalE), TopoDS.Face(aLocalF), out f, out l)!;

            Point2D ptf = c2dtf.Value(f); // sur courbe frw
            Point2D ptr = c2dtr.Value(f); // sur courbe rev

            bool isoU = (Math.Abs(ptf.Y() - ptr.Y()) < Epsilon(ptf.X())); // meme V

            // Efrom et Eto dans le meme sens???

            C = BRep_Tool.Curve(Efrom, out Loc, out f, out l)!;
            if (!Loc.IsIdentity)
            {
                Geom_Geometry GG = C.Transformed(Loc.Transformation());
                C = (Geom_Curve)GG;
            }

            C.D1(f, out Point3D pt, out Vector3D d1f);

            TopoDS_Vertex FirstVertex = TopExp.FirstVertex(Efrom);
            double vtx_param = BRep_Tool.Parameter(FirstVertex, Efrom);
            BRepAdaptor_Curve2d BAcurve2d = new(Efrom, Fac);
            Point2D p2d = BAcurve2d.Value(vtx_param);

            double prmproj = Project(TopExp.FirstVertex(Efrom), p2d, Eto, Fac);

            C = BRep_Tool.Curve(Eto, out Loc, out f, out l)!;
            if (!Loc.IsIdentity)
            {
                Geom_Geometry GG = C.Transformed(Loc.Transformation());
                C = (Geom_Curve)GG;
            }

            C.D1(prmproj, out pt, out Vector3D d1t);

            double SameOri = (d1t.Dot(d1f) > 0.0) ? 1.0 : 0.0;

            if (c2dff is null && c2dfr is null)
            {
                S = BRep_Tool.Surface(Fac)!;
                if (S is Geom_RectangularTrimmedSurface aTrimmed)
                {
                    S = aTrimmed.BasisSurface();
                }

                C = BRep_Tool.Curve(Efrom, out Loc, out f, out l)!;
                if (!Loc.IsIdentity)
                {
                    Geom_Geometry GG = C.Transformed(Loc.Transformation());
                    C = (Geom_Curve)GG;
                }

                if (C.GetType() != typeof(Geom_TrimmedCurve))
                {
                    C = new Geom_TrimmedCurve(C, f, l);
                }

                // Compute the tol2d
                BRepTools.UVBounds(Fac, out Umin, out Umax, out Vmin, out Vmax);

                double tol3d = Max(BRep_Tool.Tolerance(Efrom), BRep_Tool.Tolerance(Fac));
                GeomAdaptor_Surface Gas = new(S, Umin, Umax, Vmin, Vmax);
                double TolU = Gas.UResolution(tol3d);
                double TolV = Gas.VResolution(tol3d);
                double tol2d = Max(TolU, TolV);

                Geom2d_Curve? C2d = GeomProjLib.Curve2d(C, S, Umin, Umax, Vmin, Vmax, ref tol2d);
                c2dff = C2d;
                c2dfr = C2d;
            }
            else if (c2dfr is null)
            {
                c2dfr = c2dff;
            }
            else if (c2dff is null)
            {
                c2dff = c2dfr;
            }

            BRep_Tool.Range(Efrom, out f, out l);

            Point2D p2f = c2dff!.Value(f);
            Point2D p2r = c2dfr!.Value(f);

            if (isoU)
            {
                if (SameOri != 0.0)
                {
                    if (Math.Abs(ptf.X() - p2f.X()) > Epsilon(ptf.X()))
                    {
                        c2dff = (Geom2d_Curve)c2dff.Translated(new Vector2D(ptf.X() - p2f.X(), 0.0));
                    }
                    if (Math.Abs(ptr.X() - p2r.X()) > Epsilon(ptr.X()))
                    {
                        c2dfr = (Geom2d_Curve)c2dfr.Translated(new Vector2D(ptr.X() - p2r.X(), 0.0));
                    }
                }
                else
                {
                    if (Math.Abs(ptr.X() - p2f.X()) > Epsilon(ptr.X()))
                    {
                        c2dff = (Geom2d_Curve)c2dff.Translated(new Vector2D(ptr.X() - p2f.X(), 0.0));
                    }

                    if (Math.Abs(ptf.X() - p2r.X()) > Epsilon(ptf.X()))
                    {
                        c2dfr = (Geom2d_Curve)c2dfr.Translated(new Vector2D(ptf.X() - p2r.X(), 0.0));
                    }
                }

                // on est bien en U, recalage si periodique en V a faire
            }

            else
            { // !isoU soit isoV
                if (SameOri != 0.0)
                {
                    if (Math.Abs(ptf.Y() - p2f.Y()) > Epsilon(ptf.Y()))
                    {
                        c2dff = (Geom2d_Curve)c2dff.Translated(new Vector2D(0.0, ptf.Y() - p2f.Y()));
                    }
                    if (Math.Abs(ptr.Y() - p2r.Y()) > Epsilon(ptr.Y()))
                    {
                        c2dfr = (Geom2d_Curve)c2dfr.Translated(new Vector2D(0.0, ptr.Y() - p2r.Y()));
                    }
                }
                else
                {
                    if (Math.Abs(ptr.Y() - p2f.Y()) > Epsilon(ptr.Y()))
                    {
                        c2dff = (Geom2d_Curve)c2dff.Translated(new Vector2D(0.0, ptr.Y() - p2f.Y()));
                    }
                    if (Math.Abs(ptf.Y() - p2r.Y()) > Epsilon(ptf.Y()))
                    {
                        c2dfr = (Geom2d_Curve)c2dfr.Translated(new Vector2D(0.0, ptf.Y() - p2r.Y()));
                    }
                }
                // on est bien en V, recalage si periodique en U a faire
            }
            B.UpdateEdge(Efrom, c2dff, c2dfr, Fac, BRep_Tool.Tolerance(Efrom));
        }
    }

    //=================================================================================================

    private static void FindInternalIntersections(in TopoDS_Edge theEdge,
                                                  in TopoDS_Face theFace,
                                                  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Splits,
                                                  ref bool isOverlapped)
    {
        double TolExt = global::Kerf.Engine.Foundation.Precision.PConfusion();
        int i, j;

        BRepAdaptor_Surface anAdSurf = new(theFace, false);
        _ = anAdSurf;
        NCollection_Sequence<double> SplitPars = new();

        TopoDS_Vertex[] theVertices = new TopoDS_Vertex[2];
        TopExp.Vertices(theEdge, out theVertices[0], out theVertices[1]);
        Point3D[] thePnt = new Point3D[2];
        thePnt[0] = BRep_Tool.Pnt(theVertices[0]);
        thePnt[1] = BRep_Tool.Pnt(theVertices[1]);
        double[] aTolV = new double[2];
        aTolV[0] = BRep_Tool.Tolerance(theVertices[0]);
        aTolV[1] = BRep_Tool.Tolerance(theVertices[1]);
        double ext = 16.0; // = 4 * 4 - to avoid creating microedges, area around vertices is increased
                           // up to 4 vertex tolerance. Such approach is usual for other topological
                           // algorithms, for example, Boolean Operations.
        double[] aTolVExt = [ext * aTolV[0] * aTolV[0], ext * aTolV[1] * aTolV[1]];

        BRepAdaptor_Curve2d thePCurve = new(theEdge, theFace);
        Box2d theBox = new();
        BndLib_Add2dCurve.Add(thePCurve, BRep_Tool.Tolerance(theEdge), ref theBox);

        double[] thePar = new double[2];
        Geom_Curve theCurve = BRep_Tool.Curve(theEdge, out thePar[0], out thePar[1])!;
        GeomAdaptor_Curve theGAcurve = new(theCurve, thePar[0], thePar[1]);
        double[] aTolV2d = [theGAcurve.Resolution(aTolV[0]), theGAcurve.Resolution(aTolV[1])];
        aTolV2d[0] = Max(aTolV2d[0], global::Kerf.Engine.Foundation.Precision.PConfusion());
        aTolV2d[1] = Max(aTolV2d[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
        double aDistMax = global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion();
        TopExp_Explorer Explo = new(theFace, TopAbs_EDGE);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(Explo.Current());
            BRepAdaptor_Curve2d aPCurve = new(anEdge, theFace);
            Box2d aBox = new();
            BndLib_Add2dCurve.Add(aPCurve, BRep_Tool.Tolerance(anEdge), ref aBox);
            if (theBox.IsOut(aBox))
            {
                continue;
            }

            Geom_Curve? aCurve = BRep_Tool.Curve(anEdge, out double aFpar, out double aLpar);
            GeomAdaptor_Curve aGAcurve = new(aCurve ?? throw new NullObjectException(), aFpar, aLpar); // OCCT's Load throws on a null curve
            Extrema_ExtCC anExtrema = new(theGAcurve, aGAcurve, TolExt, TolExt);

            if (!anExtrema.IsDone() || anExtrema.NbExt() == 0)
            {
                continue;
            }

            int aNbExt = anExtrema.NbExt();
            double MaxTol = BRep_Tool.Tolerance(anEdge);
            double aMaxTol2 = MaxTol * MaxTol;
            if (anExtrema.IsParallel() && anExtrema.SquareDistance(1) <= aMaxTol2)
            {
                isOverlapped = true;
                return;
            }
            // Check extremity distances
            double[] dists = new double[4];
            anExtrema.TrimmedSquareDistances(out dists[0], out dists[1], out dists[2], out dists[3],
                                             out Point3D aP11, out Point3D aP12, out Point3D aP21, out Point3D aP22);
            for (i = 0; i < 4; ++i)
            {
                if (i < 2)
                {
                    j = 0;
                }
                else
                {
                    j = 1;
                }
                if (dists[i] < aTolVExt[j] / ext)
                {
                    return;
                }
            }

            for (i = 1; i <= aNbExt; i++)
            {
                double aDist = anExtrema.SquareDistance(i);
                if (aDist > aMaxTol2)
                {
                    continue;
                }

                anExtrema.Points(i, out Extrema_POnCurv aPOnC1, out Extrema_POnCurv aPOnC2);
                double theIntPar = aPOnC1.Parameter();
                double anIntPar = aPOnC2.Parameter();
                for (j = 0; j < 2; j++) // try to find intersection on an extremity of "theEdge"
                {
                    if (Math.Abs(theIntPar - thePar[j]) <= aTolV2d[j])
                    {
                        break;
                    }
                }
                // intersection found in the middle of the edge
                if (j >= 2) // intersection is inside "theEdge" => split
                {
                    Point3D aPoint = aCurve!.Value(anIntPar);
                    Point3D aPointInt = theCurve.Value(theIntPar);

                    if (aPointInt.SquareDistance(thePnt[0]) > aTolVExt[0]
                        && aPointInt.SquareDistance(thePnt[1]) > aTolVExt[1]
                        && aPoint.SquareDistance(thePnt[0]) > aTolVExt[0]
                        && aPoint.SquareDistance(thePnt[1]) > aTolVExt[1])
                    {
                        SplitPars.Append(theIntPar);
                        if (aDist > aDistMax)
                        {
                            aDistMax = aDist;
                        }
                    }
                }
            }
        }

        if (SplitPars.IsEmpty())
        {
            return;
        }

        // Sort
        for (i = 1; i < SplitPars.Length(); i++)
        {
            for (j = i + 1; j <= SplitPars.Length(); j++)
            {
                if (SplitPars[i] > SplitPars[j])
                {
                    double Tmp = SplitPars[i];
                    SplitPars[i] = SplitPars[j];
                    SplitPars[j] = Tmp;
                }
            }
        }
        // Remove repeating points
        i = 1;
        while (i < SplitPars.Length())
        {
            Point3D Pnt1 = theCurve.Value(SplitPars[i]);
            Point3D Pnt2 = theCurve.Value(SplitPars[i + 1]);
            if (Pnt1.SquareDistance(Pnt2) <= global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                SplitPars.Remove(i + 1);
            }
            else
            {
                i++;
            }
        }

        // Split
        NCollection_List<TopoDS_Shape> NewEdges = new();
        BRep_Builder BB = new();

        TopoDS_Vertex FirstVertex = theVertices[0], LastVertex;
        double FirstPar = thePar[0], LastPar;
        for (i = 1; i <= SplitPars.Length() + 1; i++)
        {
            FirstVertex.Orientation(TopAbs_FORWARD);
            if (i <= SplitPars.Length())
            {
                LastPar = SplitPars[i];
                Point3D LastPoint = theCurve.Value(LastPar);
                LastVertex = new BRepLib_MakeVertex(LastPoint).InternalVertex();
                BRep_Builder aB = new();
                aB.UpdateVertex(LastVertex, Math.Sqrt(aDistMax));
            }
            else
            {
                LastPar = thePar[1];
                LastVertex = theVertices[1];
            }
            LastVertex.Orientation(TopAbs_REVERSED);

            TopoDS_Shape aLocalShape = theEdge.EmptyCopied();
            TopAbs_Orientation anOrient = aLocalShape.Orientation();
            aLocalShape.Orientation(TopAbs_FORWARD);
            TopoDS_Edge NewEdge = TopoDS.Edge(aLocalShape);
            BB.Range(NewEdge, FirstPar, LastPar);
            BB.Add(NewEdge, FirstVertex);
            BB.Add(NewEdge, LastVertex);
            NewEdge.Orientation(anOrient);
            ShapeAnalysis_Edge aSae = new();
            double amaxdev = 0.0;
            if (aSae.CheckSameParameter(NewEdge, theFace, ref amaxdev))
            {
                BRep_Builder aB = new();
                aB.UpdateEdge(NewEdge, amaxdev);
            }

            if (anOrient == TopAbs_FORWARD)
            {
                NewEdges.Append(NewEdge);
            }
            else
            {
                NewEdges.Prepend(NewEdge);
            }
            FirstVertex = LastVertex;
            FirstPar = LastPar;
        }

        if (!NewEdges.IsEmpty())
        {
            Splits.Add(theEdge, NewEdges);
        }
    }
}
