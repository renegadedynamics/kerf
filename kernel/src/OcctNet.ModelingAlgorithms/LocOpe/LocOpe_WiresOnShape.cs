// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    private Shape myShape = new();
    private readonly NCollection_IndexedDataMap<Shape, Shape> myMapEF = new(ShapeHasher.Instance);
    private readonly NCollection_Map<Shape> myFacesWithSection = new(ShapeHasher.Instance);
    private bool myCheckInterior;
    // OCCT iterates this map keyed by the shapes' addresses (in BindAll); the port in insertion order
    private readonly NCollection_DataMap<Shape, Shape> myMap = new(ShapeHasher.Instance);
    private bool myDone;
    private int myIndex;
    private LocOpe_WorkingShapes myWork;

    public LocOpe_WiresOnShape(in Shape S)
    {
        myWork = new LocOpe_WorkingShapes();
        myShape = myWork.Working(S);
        myCheckInterior = true;
        myDone = false;
        myIndex = -1;
    }

    /// <summary>For ported operations that own the working shapes (LocOpe_Gluer): <paramref name="S"/> is one.</summary>
    internal LocOpe_WiresOnShape(in Shape S, LocOpe_WorkingShapes theWork)
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
    internal Shape InternalShape() => myShape;

    public void Init(in Shape S)
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
    public bool Add(NCollection_Sequence<Shape> theEdges)
    {
        NCollection_Sequence<Shape> anEdges = new();
        int i = 1, nb = theEdges.Length();
        NCollection_Array1<Box> anEdgeBoxes = new(1, nb);
        for (; i <= nb; i++)
        {
            Shape aCurSplit = myWork.Working(theEdges[i]);
            Explorer anExpE = new(aCurSplit, global::Kerf.Topology.ShapeType.Edge);
            for (; anExpE.More(); anExpE.Next())
            {
                Shape aCurE = anExpE.Current();

                Box aBoxE = new();
                BRepBndLib.AddClose(aCurE, ref aBoxE);
                if (aBoxE.IsVoid)
                {
                    continue;
                }
                double aTolE = Tool.Tolerance(ShapeCasts.Edge(aCurE));
                aBoxE.SetGap(aTolE);
                // OCCT: the box goes at the argument's index, the edge at the next free one; a wire of several edges
                // leaves them out of step (occt-issues.md)
                anEdgeBoxes.SetValue(i, aBoxE);
                anEdges.Append(aCurE);
            }
        }
        Explorer anExpFaces = new(myShape, ShapeType.Face);
        int numF = 1;
        NCollection_PackedMapOfInteger anUsedEdges = new();
        for (; anExpFaces.More(); anExpFaces.Next(), numF++)
        {
            Face aCurF = ShapeCasts.Face(anExpFaces.Current());
            Box aBoxF = new();
            BRepBndLib.Add(aCurF, ref aBoxF);
            if (aBoxF.IsVoid)
            {
                continue;
            }
            FaceSurfaceAdaptor anAdF = new(aCurF, false);
            BRepTopAdaptor_FClass2d? aCheckStateTool = null;

            // Initialize extrema projector once per face
            ExtPS anExtr = new();
            anExtr.Initialize(anAdF,
                              anAdF.FirstUParameter(),
                              anAdF.LastUParameter(),
                              anAdF.FirstVParameter(),
                              anAdF.LastVParameter(),
                              global::Kerf.Engine.Foundation.Precision.Confusion(),
                              global::Kerf.Engine.Foundation.Precision.Confusion());
            anExtr.SetFlag(ExtFlag_MIN);

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

                Edge aCurE = ShapeCasts.Edge(anEdges[i]);

                Curve3D? aC = Tool.Curve(aCurE, out double aF, out double aL);
                if (aC is null)
                {
                    anUsedEdges.Add(i);
                    continue;
                }
                Point3D aP = aC.Value((aF + aL) * 0.5);
                anExtr.Perform(aP);

                if (!anExtr.IsDone || anExtr.ExtCount == 0)
                {
                    continue;
                }
                double aTolE = Tool.Tolerance(ShapeCasts.Edge(aCurE));
                double aTol2 = (aTolE + global::Kerf.Engine.Foundation.Precision.Confusion()) * (aTolE + global::Kerf.Engine.Foundation.Precision.Confusion());
                int n = 1;
                for (; n <= anExtr.ExtCount; n++)
                {
                    double aDist2 = anExtr.SquareDistance(n);
                    if (aDist2 > aTol2)
                    {
                        continue;
                    }
                    POnSurf aPS = anExtr.Point(n);
                    aPS.Parameter(out double aU, out double aV);

                    aCheckStateTool ??= new BRepTopAdaptor_FClass2d(aCurF, global::Kerf.Engine.Foundation.Precision.PConfusion());
                    if (aCheckStateTool.Perform(new Point2D(aU, aV)) == In)
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

    public void Bind(in Wire W, in Face F)
    {
        Shape aW = myWork.Working(W);
        Face aF = ShapeCasts.Face(myWork.Working(F));
        for (Explorer exp = new(aW, global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
        {
            bind(ShapeCasts.Edge(exp.Current()), aF);
        }
    }

    public void Bind(in Compound Comp, in Face F)
    {
        Shape aComp = myWork.Working(Comp);
        Face aF = ShapeCasts.Face(myWork.Working(F));
        for (Explorer exp = new(aComp, global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
        {
            bind(ShapeCasts.Edge(exp.Current()), aF);
        }
        myFacesWithSection.Add(aF);
    }

    public void Bind(in Edge E, in Face F)
    {
        bind(ShapeCasts.Edge(myWork.Working(E)), ShapeCasts.Face(myWork.Working(F)));
    }

    // Bind(E, F) on working shapes
    private void bind(in Edge E, in Face F)
    {
        if (!myMapEF.Contains(E))
        {
            Explorer exp = new(F, global::Kerf.Topology.ShapeType.Edge);
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

    public void Bind(in Edge EfromW, in Edge EonFace)
    {
        myMap.Bind(myWork.Working(EfromW), myWork.Working(EonFace));
    }

    public void BindAll()
    {
        if (myDone)
        {
            return;
        }
        NCollection_Map<Shape> theMap = new(ShapeHasher.Instance);

        // Detection des vertex a projeter ou a "binder" avec des vertex existants
        NCollection_DataMap<Shape, Shape> mapV = new(ShapeHasher.Instance);
        Explorer exp = new(), exp2 = new();
        foreach (KeyValuePair<Shape, Shape> ite in myMap)
        {
            Edge eref = ShapeCasts.Edge(ite.Key);
            Edge eimg = ShapeCasts.Edge(ite.Value);

            PutPCurves(eref, eimg, myShape);

            for (exp.Init(eref, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (!theMap.Contains(vtx))
                { // pas deja traite
                    for (exp2.Init(eimg, ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        Vertex vtx2 = ShapeCasts.Vertex(exp2.Current());
                        if (vtx2.IsSame(vtx))
                        {
                            break;
                        }
                        else if (BrepTools.Compare(vtx, vtx2))
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

        foreach (KeyValuePair<Shape, Shape> ite in mapV)
        {
            myMap.Bind(ite.Key, ite.Value);
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Splits = new(ShapeHasher.Instance);
        int Ind;
        NCollection_Map<Shape> anOverlappedEdges = new(ShapeHasher.Instance);
        for (Ind = 1; Ind <= myMapEF.Extent(); Ind++)
        {
            Edge edg = ShapeCasts.Edge(myMapEF.FindKey(Ind));
            Face fac = ShapeCasts.Face(myMapEF[Ind]);

            PutPCurve(edg, fac);
            Curve2D? aPCurve = Tool.CurveOnSurface(edg, fac, out _, out _);
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
            Shape anEdge = Splits.FindKey(Ind);
            if (anOverlappedEdges.Contains(anEdge))
            {
                continue;
            }
            Shape aFace = myMapEF.FindFromKey(anEdge);
            // Remove "anEdge" from "myMapEF"
            myMapEF.RemoveKey(anEdge);
            NCollection_List<Shape>.Iterator itl = new(Splits[Ind]);
            for (; itl.More(); itl.Next())
            {
                myMapEF.Add(itl.Value(), aFace);
            }
        }

        NCollection_DataMap<Shape, double> aVertParam = new(ShapeHasher.Instance);

        for (Ind = 1; Ind <= myMapEF.Extent(); Ind++)
        {
            Edge edg = ShapeCasts.Edge(myMapEF.FindKey(Ind));
            Face fac = ShapeCasts.Face(myMapEF[Ind]);
            // JAG 02.02.96 : On verifie les pcurves...

            for (exp.Init(edg, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (theMap.Contains(vtx))
                {
                    continue;
                }

                double vtx_param = Tool.Parameter(vtx, edg);
                EdgeCurve2DAdaptor BAcurve2d = new(edg, fac);

                Point2D p2d = BAcurve2d.Curve() is not null
                                   ? BAcurve2d.Value(vtx_param)
                                   : new Point2D(global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());

                Edge Epro = new();
                double prm = global::Kerf.Engine.Foundation.Precision.Infinite();
                bool isProjected = myMap.IsBound(vtx);

                // if vertex was already projected on the current edge on the previous face
                // it is necessary to check tolerance of the vertex in the 2D space on the current
                // face without projection and update tolerance of vertex if it is necessary
                if (isProjected)
                {
                    Shape aSh = myMap.Find(vtx);
                    if (aSh.ShapeType() != global::Kerf.Topology.ShapeType.Edge)
                    {
                        continue;
                    }
                    Epro = ShapeCasts.Edge(myMap.Find(vtx));
                    if (aVertParam.IsBound(vtx))
                    {
                        prm = aVertParam.Find(vtx);
                    }
                }
                bool ok = Project(vtx, p2d, fac, ref Epro, ref prm);
                if (ok && !isProjected)
                {
                    for (exp2.Init(Epro, ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        Vertex vtx2 = ShapeCasts.Vertex(exp2.Current());
                        if (vtx2.IsSame(vtx))
                        {
                            myMap.Bind(vtx, vtx2);
                            theMap.Add(vtx);
                            break;
                        }
                        else if (BrepTools.Compare(vtx, vtx2))
                        {
                            Tool.Range(Epro, fac, out double aF1, out double aL1);
                            if (!Tool.Degenerated(Epro)
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
        foreach (KeyValuePair<Shape, Shape> ite in myMap)
        {
            if (ite.Key.ShapeType() == global::Kerf.Topology.ShapeType.Edge)
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
    public Edge Edge() => ShapeCasts.Edge(myWork.Original(InternalEdge()));

    /// <summary><see cref="Edge"/> as worked on, for ported code.</summary>
    internal Edge InternalEdge() => ShapeCasts.Edge(myMapEF.FindKey(myIndex));

    /// <summary>Returns the face of the shape on which the current edge is projected (the caller's face).</summary>
    public Face OnFace() => ShapeCasts.Face(myWork.Original(InternalOnFace()));

    /// <summary><see cref="OnFace"/> as worked on, for ported code.</summary>
    internal Face InternalOnFace() => ShapeCasts.Face(myMapEF[myIndex]);

    /// <summary>
    /// If the current edge is projected on an edge, returns true and sets the edge (the caller's edge) in
    /// <paramref name="E"/>. Otherwise, returns false.
    /// </summary>
    public bool OnEdge(out Edge E)
    {
        bool isOn = InternalOnEdge(out E);
        E = ShapeCasts.Edge(myWork.Original(E));
        return isOn;
    }

    /// <summary><see cref="OnEdge(out Edge)"/> as worked on, for ported code.</summary>
    internal bool InternalOnEdge(out Edge E)
    {
        E = new Edge();
        if (myMap.IsBound(myMapEF.FindKey(myIndex)))
        {
            E = ShapeCasts.Edge(myMap[myMapEF.FindKey(myIndex)]);
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
    public bool OnVertex(in Vertex Vw, out Vertex Vs)
    {
        bool isOn = InternalOnVertex(ShapeCasts.Vertex(myWork.Copied(Vw)), out Vs);
        Vs = ShapeCasts.Vertex(myWork.Original(Vs));
        return isOn;
    }

    /// <summary><see cref="OnVertex"/> on working shapes, for ported code.</summary>
    internal bool InternalOnVertex(in Vertex Vw, out Vertex Vs)
    {
        Vs = new Vertex();
        if (myMap.IsBound(Vw))
        {
            if (myMap[Vw].ShapeType() == ShapeType.Vertex)
            {
                Vs = ShapeCasts.Vertex(myMap[Vw]);
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
    public bool OnEdge(in Vertex V, out Edge Ed, out double prm)
    {
        bool isOn = InternalOnEdge(ShapeCasts.Vertex(myWork.Copied(V)), out Ed, out prm);
        Ed = ShapeCasts.Edge(myWork.Original(Ed));
        return isOn;
    }

    /// <summary><see cref="OnEdge(in Vertex, out Edge, out double)"/> on working shapes.</summary>
    internal bool InternalOnEdge(in Vertex V, out Edge Ed, out double prm)
    {
        Ed = new Edge();
        prm = 0.0; // OCCT: unchanged
        if (!myMap.IsBound(V) || myMap[V].ShapeType() == ShapeType.Vertex)
        {
            return false;
        }

        Ed = ShapeCasts.Edge(myMap[V]);
        prm = Project(V, Ed);
        return true;
    }

    /// <summary>
    /// <see cref="OnEdge(in Vertex, out Edge, out double)"/> for the vertex <paramref name="V"/> of the
    /// edge <paramref name="EdgeFrom"/>, projected through its pcurve where the shape's edge has no 3D curve.
    /// </summary>
    public bool OnEdge(in Vertex V, in Edge EdgeFrom, out Edge Ed, out double prm)
    {
        bool isOn = InternalOnEdge(ShapeCasts.Vertex(myWork.Copied(V)), ShapeCasts.Edge(myWork.Copied(EdgeFrom)), out Ed, out prm);
        Ed = ShapeCasts.Edge(myWork.Original(Ed));
        return isOn;
    }

    /// <summary><see cref="OnEdge(in Vertex, in Edge, out Edge, out double)"/> on working shapes.</summary>
    internal bool InternalOnEdge(in Vertex V, in Edge EdgeFrom, out Edge Ed, out double prm)
    {
        Ed = new Edge();
        prm = 0.0; // OCCT: unchanged
        if (!myMap.IsBound(V) || myMap[V].ShapeType() == ShapeType.Vertex)
        {
            return false;
        }

        Ed = ShapeCasts.Edge(myMap[V]);
        if (!myMapEF.Contains(EdgeFrom))
        {
            return false;
        }

        Shape aShape = myMapEF.FindFromKey(EdgeFrom);
        Curve3D? aC = Tool.Curve(Ed, out _, out _);
        if (aC is null && aShape.ShapeType() == ShapeType.Face)
        {
            Face aFace = ShapeCasts.Face(aShape);
            double vtx_param = Tool.Parameter(V, EdgeFrom);
            EdgeCurve2DAdaptor BAcurve2d = new(EdgeFrom, aFace);
            Point2D p2d = BAcurve2d.Value(vtx_param);

            prm = Project(V, p2d, Ed, aFace);
        }
        else
        {
            prm = Project(V, ShapeCasts.Edge(Ed));
        }

        return true;
    }

    /// <summary>Tells if the face is to be split by a section (a compound bound to it).</summary>
    public bool IsFaceWithSection(in Shape aFace) => InternalIsFaceWithSection(myWork.Copied(aFace));

    /// <summary><see cref="IsFaceWithSection"/> on working shapes.</summary>
    internal bool InternalIsFaceWithSection(in Shape aFace) => myFacesWithSection.Contains(aFace);

    //=================================================================================================

    private static bool Project(in Vertex V, in Point2D p2d, in Face F, ref Edge theEdge, ref double param)
    {
        double aTolV = Tool.Tolerance(V);
        double dmin = (theEdge.IsNull ? RealLast() : aTolV * aTolV);

        bool valret = false;

        Surface aSurf = Tool.Surface(F)!;

        if (theEdge.IsNull)
        {
            Point3D toproj = Tool.Pnt(V);
            for (Explorer exp = new(F.Oriented(Forward), global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                Curve3D? C = Tool.Curve(edg, out double f, out double l);
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
                    Curve2D? aC2d = Tool.CurveOnSurface(edg, F, out f, out l);
                    if (aC2d is null)
                    {
                        continue;
                    }

                    aCurPar = Project(V, p2d, edg, F);
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurPar))
                    {
                        continue;
                    }
                    Curve2D PC = Tool.CurveOnSurface(edg, F, out f, out l)!;
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
            if (theEdge.IsNull)
            {
                return false;
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(param))
        {
            Curve3D? C = Tool.Curve(theEdge, out _, out _);
            param = (C is not null ? Project(V, theEdge) : Project(V, p2d, theEdge, F));
        }

        double ttol = aTolV + Tool.Tolerance(theEdge);
        if (dmin <= ttol * ttol)
        {
            valret = true;
            SurfaceAdaptor adSurf = new(aSurf);

            double anUResolution = adSurf.UResolution(1.0);
            double aVResolution = adSurf.UResolution(1.0); // OCCT: UResolution for V too (occt-issues.md)

            Curve2D? aCrvBound = Tool.CurveOnSurface(theEdge, F, out _, out _);
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
                if (dumin > dumax && adSurf.IsUPeriodic)
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

                if (dvmin > dvmax && adSurf.IsVPeriodic)
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

                BrepBuilder B = new();
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

    private static double Project(in Vertex V, in Edge theEdge)
    {
        Point3D toproj = Tool.Pnt(V);
        GeomAPI_ProjectPointOnCurve proj = new();

        Curve3D C = Tool.Curve(theEdge, out Placement Loc, out double f, out double l)!;
        if (!Loc.IsIdentity)
        {
            Geometry3D GG = C.Transformed(Loc.Transformation());
            C = (Curve3D)GG;
        }
        proj.Init(toproj, C, f, l);

        return (proj.NbPoints() > 0 ? proj.LowerDistanceParameter() : global::Kerf.Engine.Foundation.Precision.Infinite());
    }

    //=================================================================================================

    private static double Project(in Vertex V, in Point2D p2d, in Edge theEdge, in Face theFace)
    {
        _ = V;
        Geom2dAPI_ProjectPointOnCurve proj = new();

        Curve2D PC = Tool.CurveOnSurface(theEdge, theFace, out double f, out double l)!;

        proj.Init(p2d, PC, f, l);

        return proj.NbPoints() > 0 ? proj.LowerDistanceParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    //=================================================================================================

    // OCCT.NET: OCCT translates the projected pcurve in place; a frozen one (shared) is translated on a copy
    private static Curve2D Translatable(Curve2D theCurve) =>
        theCurve.IsFrozen ? (Curve2D)theCurve.Copy() : theCurve;

    private static void PutPCurve(in Edge Edg, in Face Fac)
    {
        BrepBuilder B = new();

        Surface S = Tool.Surface(Fac, out Placement LocFac)!;
        _ = LocFac;

        if (S is TrimmedSurface aTrimmed)
        {
            S = aTrimmed.BasisSurface();
        }

        if (S.GetType() == typeof(PlaneSurface))
        {
            return;
        }

        BrepTools.UVBounds(Fac, out double Umin, out double Umax, out double Vmin, out double Vmax);

        double f, l;

        Curve2D? aC2d = Tool.CurveOnSurface(Edg, Fac, out f, out l);
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

        Curve3D C = Tool.Curve(Edg, out Placement Loc, out f, out l)!;
        if (!Loc.IsIdentity)
        {
            Geometry3D GG = C.Transformed(Loc.Transformation());
            C = (Curve3D)GG;
        }

        if (C.GetType() != typeof(TrimmedCurve3D))
        {
            C = new TrimmedCurve3D(C, f, l);
        }

        S = Tool.Surface(Fac)!;

        double TolFirst = -1, TolLast = -1;
        Exploration.Vertices(Edg, out Vertex V1, out Vertex V2);
        if (!V1.IsNull)
        {
            TolFirst = Tool.Tolerance(V1);
        }
        if (!V2.IsNull)
        {
            TolLast = Tool.Tolerance(V2);
        }

        double tol2d = global::Kerf.Engine.Foundation.Precision.Confusion();
        ShapeConstruct_ProjectCurveOnSurface aToolProj = new();
        aToolProj.Init(S, tol2d);

        aToolProj.InternalPerform(C, f, l, out Curve2D? C2d, TolFirst, TolLast);
        if (C2d is null)
        {
            return;
        }

        Point2D pf = C2d.Value(f);
        Point2D pl = C2d.Value(l);
        S.D0(pf.X(), pf.Y(), out Point3D PF);
        S.D0(pl.X(), pl.Y(), out Point3D PL);
        if (Edg.Orientation() == Reversed)
        {
            V1 = Exploration.LastVertex(Edg);
            V1.Reverse();
        }
        else
        {
            V1 = Exploration.FirstVertex(Edg);
        }
        if (Edg.Orientation() == Reversed)
        {
            V2 = Exploration.FirstVertex(Edg);
            V2.Reverse();
        }
        else
        {
            V2 = Exploration.LastVertex(Edg);
        }

        if (!V1.IsNull && V2.IsNull)
        {
            // Handling of internal vertices
            // OCCT: V2 is null here, so BRep_Tool::Tolerance(V2) dereferences a null shape (occt-issues.md)
            double old1 = Tool.Tolerance(V1);
            double old2 = Tool.Tolerance(V2);
            Point3D pnt1 = Tool.Pnt(V1);
            Point3D pnt2 = Tool.Pnt(V2);
            double tol1 = pnt1.Distance(PF);
            double tol2 = pnt2.Distance(PL);
            B.UpdateVertex(V1, Max(old1, tol1));
            B.UpdateVertex(V2, Max(old2, tol2));
        }

        if (S.IsUPeriodic)
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

        if (S.IsVPeriodic)
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
        B.UpdateEdge(Edg, C2d, Fac, Tool.Tolerance(Edg));

        B.SameParameter(Edg, false);
        BRepLib.SameParameter(Edg, tol2d);
    }

    //=================================================================================================

    private static void PutPCurves(in Edge Efrom, in Edge Eto, in Shape myShape)
    {
        NCollection_List<Shape> Lfaces = new();
        Explorer exp = new(), exp2 = new();

        for (exp.Init(myShape, ShapeType.Face); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), global::Kerf.Topology.ShapeType.Edge); exp2.More(); exp2.Next())
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

        BrepBuilder B = new();
        Surface S;
        Curve3D C;
        double Umin, Umax, Vmin, Vmax;
        double f, l;
        Placement Loc;

        if (!Lfaces.First().IsSame(Lfaces.Last()))
        {
            NCollection_List<Shape>.Iterator itl = new(Lfaces);
            for (; itl.More(); itl.Next())
            {
                Face Fac = ShapeCasts.Face(itl.Value());

                if (Tool.CurveOnSurface(Efrom, Fac, out f, out l) is not null)
                {
                    continue;
                }
                S = Tool.Surface(Fac, out _)!;
                if (S is TrimmedSurface aTrimmed)
                {
                    S = aTrimmed.BasisSurface();
                }
                if (S.GetType() == typeof(PlaneSurface))
                {
                    continue;
                }

                BrepTools.UVBounds(Fac, out Umin, out Umax, out Vmin, out Vmax);
                C = Tool.Curve(Efrom, out Loc, out f, out l)!;
                if (!Loc.IsIdentity)
                {
                    Geometry3D GG = C.Transformed(Loc.Transformation());
                    C = (Curve3D)GG;
                }

                if (C.GetType() != typeof(TrimmedCurve3D))
                {
                    C = new TrimmedCurve3D(C, f, l);
                }

                S = Tool.Surface(Fac)!;

                // Compute the tol2d
                double tol3d = Max(Tool.Tolerance(Efrom), Tool.Tolerance(Fac));
                SurfaceAdaptor Gas = new(S, Umin, Umax, Vmin, Vmax);
                double TolU = Gas.UResolution(tol3d);
                double TolV = Gas.VResolution(tol3d);
                double tol2d = Max(TolU, TolV);

                Curve2D? C2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, S, Umin, Umax, Vmin, Vmax, ref tol2d);
                if (C2d is null)
                {
                    return;
                }

                Point2D pf = C2d.Value(f);
                Point2D pl = C2d.Value(l);

                if (S.IsUPeriodic)
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

                if (S.IsVPeriodic)
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
                B.UpdateEdge(Efrom, C2d, Fac, Tool.Tolerance(Efrom));
            }
        }

        else
        {
            Face Fac = ShapeCasts.Face(Lfaces.First());
            if (!Tool.IsClosed(Eto, Fac))
            {
                throw new ConstructionErrorException();
            }

            Shape aLocalE = Efrom.Oriented(Forward);
            Shape aLocalF = Fac.Oriented(Forward);
            Curve2D? c2dff = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalE), ShapeCasts.Face(aLocalF), out f, out l);

            aLocalE = Efrom.Oriented(Reversed);
            aLocalF = Fac.Oriented(Forward);
            Curve2D? c2dfr = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalE), ShapeCasts.Face(aLocalF), out f, out l);

            aLocalE = Eto.Oriented(Forward);
            aLocalF = Fac.Oriented(Forward);
            Curve2D c2dtf = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalE), ShapeCasts.Face(aLocalF), out f, out l)!;

            aLocalE = Eto.Oriented(Reversed);
            aLocalF = Fac.Oriented(Forward);
            Curve2D c2dtr = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalE), ShapeCasts.Face(aLocalF), out f, out l)!;

            Point2D ptf = c2dtf.Value(f); // sur courbe frw
            Point2D ptr = c2dtr.Value(f); // sur courbe rev

            bool isoU = (Math.Abs(ptf.Y() - ptr.Y()) < Epsilon(ptf.X())); // meme V

            // Efrom et Eto dans le meme sens???

            C = Tool.Curve(Efrom, out Loc, out f, out l)!;
            if (!Loc.IsIdentity)
            {
                Geometry3D GG = C.Transformed(Loc.Transformation());
                C = (Curve3D)GG;
            }

            C.D1(f, out Point3D pt, out Vector3D d1f);

            Vertex FirstVertex = Exploration.FirstVertex(Efrom);
            double vtx_param = Tool.Parameter(FirstVertex, Efrom);
            EdgeCurve2DAdaptor BAcurve2d = new(Efrom, Fac);
            Point2D p2d = BAcurve2d.Value(vtx_param);

            double prmproj = Project(Exploration.FirstVertex(Efrom), p2d, Eto, Fac);

            C = Tool.Curve(Eto, out Loc, out f, out l)!;
            if (!Loc.IsIdentity)
            {
                Geometry3D GG = C.Transformed(Loc.Transformation());
                C = (Curve3D)GG;
            }

            C.D1(prmproj, out pt, out Vector3D d1t);

            double SameOri = (d1t.Dot(d1f) > 0.0) ? 1.0 : 0.0;

            if (c2dff is null && c2dfr is null)
            {
                S = Tool.Surface(Fac)!;
                if (S is TrimmedSurface aTrimmed)
                {
                    S = aTrimmed.BasisSurface();
                }

                C = Tool.Curve(Efrom, out Loc, out f, out l)!;
                if (!Loc.IsIdentity)
                {
                    Geometry3D GG = C.Transformed(Loc.Transformation());
                    C = (Curve3D)GG;
                }

                if (C.GetType() != typeof(TrimmedCurve3D))
                {
                    C = new TrimmedCurve3D(C, f, l);
                }

                // Compute the tol2d
                BrepTools.UVBounds(Fac, out Umin, out Umax, out Vmin, out Vmax);

                double tol3d = Max(Tool.Tolerance(Efrom), Tool.Tolerance(Fac));
                SurfaceAdaptor Gas = new(S, Umin, Umax, Vmin, Vmax);
                double TolU = Gas.UResolution(tol3d);
                double TolV = Gas.VResolution(tol3d);
                double tol2d = Max(TolU, TolV);

                Curve2D? C2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, S, Umin, Umax, Vmin, Vmax, ref tol2d);
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

            Tool.Range(Efrom, out f, out l);

            Point2D p2f = c2dff!.Value(f);
            Point2D p2r = c2dfr!.Value(f);

            if (isoU)
            {
                if (SameOri != 0.0)
                {
                    if (Math.Abs(ptf.X() - p2f.X()) > Epsilon(ptf.X()))
                    {
                        c2dff = (Curve2D)c2dff.Translated(new Vector2D(ptf.X() - p2f.X(), 0.0));
                    }
                    if (Math.Abs(ptr.X() - p2r.X()) > Epsilon(ptr.X()))
                    {
                        c2dfr = (Curve2D)c2dfr.Translated(new Vector2D(ptr.X() - p2r.X(), 0.0));
                    }
                }
                else
                {
                    if (Math.Abs(ptr.X() - p2f.X()) > Epsilon(ptr.X()))
                    {
                        c2dff = (Curve2D)c2dff.Translated(new Vector2D(ptr.X() - p2f.X(), 0.0));
                    }

                    if (Math.Abs(ptf.X() - p2r.X()) > Epsilon(ptf.X()))
                    {
                        c2dfr = (Curve2D)c2dfr.Translated(new Vector2D(ptf.X() - p2r.X(), 0.0));
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
                        c2dff = (Curve2D)c2dff.Translated(new Vector2D(0.0, ptf.Y() - p2f.Y()));
                    }
                    if (Math.Abs(ptr.Y() - p2r.Y()) > Epsilon(ptr.Y()))
                    {
                        c2dfr = (Curve2D)c2dfr.Translated(new Vector2D(0.0, ptr.Y() - p2r.Y()));
                    }
                }
                else
                {
                    if (Math.Abs(ptr.Y() - p2f.Y()) > Epsilon(ptr.Y()))
                    {
                        c2dff = (Curve2D)c2dff.Translated(new Vector2D(0.0, ptr.Y() - p2f.Y()));
                    }
                    if (Math.Abs(ptf.Y() - p2r.Y()) > Epsilon(ptf.Y()))
                    {
                        c2dfr = (Curve2D)c2dfr.Translated(new Vector2D(0.0, ptf.Y() - p2r.Y()));
                    }
                }
                // on est bien en V, recalage si periodique en U a faire
            }
            B.UpdateEdge(Efrom, c2dff, c2dfr, Fac, Tool.Tolerance(Efrom));
        }
    }

    //=================================================================================================

    private static void FindInternalIntersections(in Edge theEdge,
                                                  in Face theFace,
                                                  NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Splits,
                                                  ref bool isOverlapped)
    {
        double TolExt = global::Kerf.Engine.Foundation.Precision.PConfusion();
        int i, j;

        FaceSurfaceAdaptor anAdSurf = new(theFace, false);
        _ = anAdSurf;
        NCollection_Sequence<double> SplitPars = new();

        Vertex[] theVertices = new Vertex[2];
        Exploration.Vertices(theEdge, out theVertices[0], out theVertices[1]);
        Point3D[] thePnt = new Point3D[2];
        thePnt[0] = Tool.Pnt(theVertices[0]);
        thePnt[1] = Tool.Pnt(theVertices[1]);
        double[] aTolV = new double[2];
        aTolV[0] = Tool.Tolerance(theVertices[0]);
        aTolV[1] = Tool.Tolerance(theVertices[1]);
        double ext = 16.0; // = 4 * 4 - to avoid creating microedges, area around vertices is increased
                           // up to 4 vertex tolerance. Such approach is usual for other topological
                           // algorithms, for example, Boolean Operations.
        double[] aTolVExt = [ext * aTolV[0] * aTolV[0], ext * aTolV[1] * aTolV[1]];

        EdgeCurve2DAdaptor thePCurve = new(theEdge, theFace);
        Box2d theBox = new();
        Add2dCurve.Add(thePCurve, Tool.Tolerance(theEdge), ref theBox);

        double[] thePar = new double[2];
        Curve3D theCurve = Tool.Curve(theEdge, out thePar[0], out thePar[1])!;
        CurveAdaptor theGAcurve = new(theCurve, thePar[0], thePar[1]);
        double[] aTolV2d = [theGAcurve.Resolution(aTolV[0]), theGAcurve.Resolution(aTolV[1])];
        aTolV2d[0] = Max(aTolV2d[0], global::Kerf.Engine.Foundation.Precision.PConfusion());
        aTolV2d[1] = Max(aTolV2d[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
        double aDistMax = global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion();
        Explorer Explo = new(theFace, global::Kerf.Topology.ShapeType.Edge);
        for (; Explo.More(); Explo.Next())
        {
            Edge anEdge = ShapeCasts.Edge(Explo.Current());
            EdgeCurve2DAdaptor aPCurve = new(anEdge, theFace);
            Box2d aBox = new();
            Add2dCurve.Add(aPCurve, Tool.Tolerance(anEdge), ref aBox);
            if (theBox.IsOut(aBox))
            {
                continue;
            }

            Curve3D? aCurve = Tool.Curve(anEdge, out double aFpar, out double aLpar);
            CurveAdaptor aGAcurve = new(aCurve ?? throw new NullObjectException(), aFpar, aLpar); // OCCT's Load throws on a null curve
            ExtCC anExtrema = new(theGAcurve, aGAcurve, TolExt, TolExt);

            if (!anExtrema.IsDone || anExtrema.ExtCount == 0)
            {
                continue;
            }

            int aNbExt = anExtrema.ExtCount;
            double MaxTol = Tool.Tolerance(anEdge);
            double aMaxTol2 = MaxTol * MaxTol;
            if (anExtrema.IsParallel && anExtrema.SquareDistance(1) <= aMaxTol2)
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

                anExtrema.Points(i, out POnCurv aPOnC1, out POnCurv aPOnC2);
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
        NCollection_List<Shape> NewEdges = new();
        BrepBuilder BB = new();

        Vertex FirstVertex = theVertices[0], LastVertex;
        double FirstPar = thePar[0], LastPar;
        for (i = 1; i <= SplitPars.Length() + 1; i++)
        {
            FirstVertex.Orientation(Forward);
            if (i <= SplitPars.Length())
            {
                LastPar = SplitPars[i];
                Point3D LastPoint = theCurve.Value(LastPar);
                LastVertex = new BRepLib_MakeVertex(LastPoint).InternalVertex();
                BrepBuilder aB = new();
                aB.UpdateVertex(LastVertex, Math.Sqrt(aDistMax));
            }
            else
            {
                LastPar = thePar[1];
                LastVertex = theVertices[1];
            }
            LastVertex.Orientation(Reversed);

            Shape aLocalShape = theEdge.EmptyCopied();
            Orientation anOrient = aLocalShape.Orientation();
            aLocalShape.Orientation(Forward);
            Edge NewEdge = ShapeCasts.Edge(aLocalShape);
            BB.Range(NewEdge, FirstPar, LastPar);
            BB.Add(NewEdge, FirstVertex);
            BB.Add(NewEdge, LastVertex);
            NewEdge.Orientation(anOrient);
            ShapeAnalysis_Edge aSae = new();
            double amaxdev = 0.0;
            if (aSae.CheckSameParameter(NewEdge, theFace, ref amaxdev))
            {
                BrepBuilder aB = new();
                aB.UpdateEdge(NewEdge, amaxdev);
            }

            if (anOrient == Forward)
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
