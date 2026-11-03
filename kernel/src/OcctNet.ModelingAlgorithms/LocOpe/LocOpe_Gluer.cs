// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.Continuity;
using static Occt.LocOpe_Operation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Glues a new shape on a basis shape by some of its faces, which lie on faces of the basis shape (a fuse or a cut,
/// depending on the faces' orientations).
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, it writes into both shapes: it splits the basis shape (point representations of its vertices), adds
/// pcurves to the new shape's edges, and sets the continuities of the edges. So both are worked on as copies when
/// frozen, sharing their copies (decision 1013): <see cref="Perform"/> splits, generates and recodes the regularities
/// on the copies, then gives back the caller's sub-shapes left unchanged. The shapes it hands out are frozen; ported
/// callers (BRepFeat) read the <c>Internal</c> forms.
/// </para>
/// </remarks>
public sealed class LocOpe_Gluer
{
    private bool myDone;
    private Shape mySb = new();
    private Shape mySn = new();
    private Shape myRes = new();
    private Orientation myOri;
    private LocOpe_Operation myOpe;
    private readonly NCollection_IndexedDataMap<Shape, Shape> myMapEF = new(ShapeHasher.Instance);
    // OCCT iterates this map keyed by the edges' addresses; the port in insertion order
    private readonly NCollection_DataMap<Shape, Shape> myMapEE = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myDescF = new(ShapeHasher.Instance);
    private NCollection_List<Shape> myEdges = new();
    private NCollection_List<Shape> myTgtEdges = new();
    private LocOpe_WorkingShapes myWork = new();

    public LocOpe_Gluer()
    {
        myDone = false;
        myOri = Internal;
        myOpe = LocOpe_INVALID;
    }

    public LocOpe_Gluer(in Shape Sbase, in Shape Snew)
    {
        myDone = false;
        mySb = Sbase;
        mySn = Snew;
        myOri = Internal;
        myOpe = LocOpe_INVALID;
    }

    public void Init(in Shape Sbase, in Shape Snew)
    {
        mySb = Sbase;
        mySn = Snew;
        myMapEF.Clear();
        myMapEE.Clear();
        myDescF.Clear();
        myDone = false;
        myOri = Internal;
        myOpe = LocOpe_INVALID;
    }

    /// <summary>Binds the face <paramref name="Fnew"/> of the new shape to the face <paramref name="Fbase"/> of the basis shape it lies on.</summary>
    public void Bind(in Face Fnew, in Face Fbase)
    {
        Explorer exp = new(mySn, ShapeType.Face);

        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Fnew))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        Shape aLocalFace = Fnew.Oriented(exp.Current().Orientation());
        Face Fnor = ShapeCasts.Face(aLocalFace);

        for (exp.Init(mySb, ShapeType.Face); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Fbase))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        aLocalFace = Fbase.Oriented(exp.Current().Orientation());
        Face Fbor = ShapeCasts.Face(aLocalFace);
        Orientation Ori = GetOrientation(Fnor, Fbor);

        if (myOri == Internal)
        {
            myOri = Ori;
            if (myOri == Reversed)
            {
                mySn.Reverse();
                myOpe = LocOpe_CUT;
            }
            else
            {
                myOpe = LocOpe_FUSE;
            }
        }
        else if (Ori != Forward)
        {
            myOpe = LocOpe_INVALID;
        }

        for (exp.Init(Fnor, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!myMapEF.Contains(edg))
            {
                myMapEF.Add(edg, Fbor);
            }
            else if (!myMapEF.FindFromKey(edg).IsSame(Fbor))
            {
                myMapEF.ChangeFromKey(edg).Nullify();
                // edg sur 2 face. a binder avec l`edge commun
            }
        }
        myMapEF.Add(Fnor, Fbor);
    }

    /// <summary>Binds the edge <paramref name="Enew"/> of the new shape to the edge <paramref name="Ebase"/> of the basis shape.</summary>
    public void Bind(in Edge Enew, in Edge Ebase)
    {
        if (myMapEE.IsBound(Enew) && !myMapEE[Enew].IsSame(Ebase))
        {
            throw new ConstructionErrorException();
        }
        myMapEE.Bind(Enew, Ebase);
    }

    public LocOpe_Operation OpeType() => myOpe;

    public void Perform()
    {
        int ind;
        if (myDone)
        {
            return;
        }
        if (mySb.IsNull || mySn.IsNull || myMapEF.IsEmpty() || myOpe == LocOpe_INVALID)
        {
            throw new ConstructionErrorException();
        }

        // OCCT.NET: the basis shape and the new shape as worked on (their copies if frozen); the bindings are mapped
        myWork = new LocOpe_WorkingShapes();
        Shape aSb = myWork.Working(mySb);
        Shape aSn = myWork.Working(mySn);

        LocOpe_WiresOnShape theWOnS = new(aSb, myWork);
        LocOpe_GluedShape theGS = new(aSn);

        int lmap = myMapEF.Extent();

        for (ind = 1; ind <= lmap; ind++)
        {
            Shape S = myWork.Working(myMapEF.FindKey(ind));
            if (S.ShapeType() == ShapeType.Edge)
            {
                Shape S2 = myWork.Working(myMapEF[ind]);
                if (!S2.IsNull)
                {
                    theWOnS.Bind(ShapeCasts.Edge(S), ShapeCasts.Face(S2));
                }
            }
            else
            { // TopAbs_FACE
                theGS.GlueOnFace(ShapeCasts.Face(S));
            }
        }

        foreach (KeyValuePair<Shape, Shape> itm in myMapEE)
        {
            theWOnS.Bind(ShapeCasts.Edge(myWork.Working(itm.Key)), ShapeCasts.Edge(myWork.Working(itm.Value)));
        }

        theWOnS.BindAll();

        if (!theWOnS.IsDone())
        {
            return;
        }

        LocOpe_Spliter theSplit = new(aSb);
        theSplit.InternalPerform(theWOnS);
        if (!theSplit.IsDone())
        {
            return;
        }
        // Mise a jour des descendants
        Explorer exp = new(aSb, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            myDescF.Bind(exp.Current(), new NCollection_List<Shape>(theSplit.InternalDescendantShapes(exp.Current())));
        }

        for (exp.Init(aSn, ShapeType.Face); exp.More(); exp.Next())
        {
            NCollection_List<Shape> thelist = new();
            myDescF.Bind(exp.Current(), thelist);
            if (Contains(theGS.OrientedFaces(), exp.Current()))
            {
                myDescF[exp.Current()].Append(exp.Current());
            }
        }

        LocOpe_Generator theGen = new(theSplit.InternalResultingShape(), myWork);
        theGen.InternalPerform(theGS);

        myDone = theGen.IsDone();
        if (myDone)
        {
            myRes = theGen.InternalResultingShape();

            AddEdges(aSn);

            // Mise a jour des descendants
            foreach (Shape aKey in myDescF.Keys.ToList())
            {
                NCollection_List<Shape> newDesc = new();
                NCollection_List<Shape>.Iterator itl = new();
                for (itl.Initialize(myDescF[aKey]); itl.More(); itl.Next())
                {
                    NCollection_List<Shape>.Iterator itl2 = new(theGen.InternalDescendantFace(ShapeCasts.Face(itl.Value())));
                    for (; itl2.More(); itl2.Next())
                    {
                        Face descface = ShapeCasts.Face(itl2.Value());
                        if (!descface.IsNull)
                        { // sinon la face a disparu
                            newDesc.Append(descface);
                        }
                    }
                }
                myDescF[aKey] = newDesc;
            }
        }

        // recodage des regularites
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEF1 = new(ShapeHasher.Instance),
                                                                                   theMapEF2 = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aSn, ShapeType.Edge, ShapeType.Face, theMapEF1);
        Exploration.MapShapesAndAncestors(myRes, ShapeType.Edge, ShapeType.Face, theMapEF2);

        for (ind = 1; ind <= theMapEF1.Extent(); ind++)
        {
            Edge edg = ShapeCasts.Edge(theMapEF1.FindKey(ind));
            NCollection_List<Shape> LL = theMapEF1[ind];
            if (LL.Extent() == 2)
            {
                Face fac1 = ShapeCasts.Face(LL.First());
                Face fac2 = ShapeCasts.Face(LL.Last());
                Continuity thecont = Tool.Continuity(edg, fac1, fac2);
                if (thecont >= G1)
                {
                    // on essaie de recoder
                    int ind2 = theMapEF2.FindIndex(edg);
                    if (ind2 != 0)
                    {
                        NCollection_List<Shape> LL2 = theMapEF2[ind2];
                        if (LL2.Extent() == 2)
                        {
                            Face ff1 = ShapeCasts.Face(LL2.First());
                            Face ff2 = ShapeCasts.Face(LL2.Last());
                            if ((ff1.IsSame(fac1) && ff2.IsSame(fac2)) || (ff1.IsSame(fac2) && ff2.IsSame(fac1)))
                            {
                            }
                            else
                            {
                                BrepBuilder B = new();
                                B.Continuity(edg, ff1, ff2, thecont);
                            }
                        }
                    }
                }
            }
        }
        // creation de la liste d`edge
        theWOnS.InitEdgeIterator();
        while (theWOnS.MoreEdge())
        {
            Edge edg = theWOnS.InternalEdge();
            for (ind = 1; ind <= theMapEF2.Extent(); ind++)
            {
                Edge edg1 = ShapeCasts.Edge(theMapEF2.FindKey(ind));
                if (edg1.IsSame(edg))
                {
                    myEdges.Append(edg);
                    // recodage eventuel des regularites sur cet edge
                    NCollection_List<Shape> L = theMapEF2[ind];
                    if (L.Extent() == 2)
                    {
                        Face fac1 = ShapeCasts.Face(L.First());
                        Face fac2 = ShapeCasts.Face(L.Last());
                        if (LocOpe.TgtFaces(edg, fac1, fac2))
                        {
                            myTgtEdges.Append(edg);
                            Continuity thecont = Tool.Continuity(edg, fac1, fac2);
                            if (thecont < G1)
                            {
                                BrepBuilder B = new();
                                B.Continuity(edg, fac1, fac2, G1);
                            }
                        }
                    }
                }
            }
            theWOnS.NextEdge();
        }

        // recodage eventuel des regularites sur cet edge

        // OCCT.NET: the end of the working shapes: the caller's sub-shapes left unchanged come back
        if (myWork.IsCopied())
        {
            var aShapes = new List<Shape> { myRes };
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> anItem in myDescF)
            {
                aShapes.AddRange(anItem.Value);
            }
            aShapes.AddRange(myEdges);
            aShapes.AddRange(myTgtEdges);
            myWork.Finish(aShapes);
            myRes = myWork.Final(myRes);
            foreach (Shape aKey in myDescF.Keys.ToList())
            {
                myDescF[aKey] = myWork.Final(myDescF[aKey]);
            }
            myEdges = myWork.Final(myEdges);
            myTgtEdges = myWork.Final(myTgtEdges);
        }
    }

    public bool IsDone() => myDone;

    /// <summary>The glued shape (frozen).</summary>
    public Shape ResultingShape() => LocOpe_WorkingShapes.Published(InternalResultingShape());

    /// <summary><see cref="ResultingShape"/> without freezing, for ported code.</summary>
    internal Shape InternalResultingShape()
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        return myRes;
    }

    /// <summary>
    /// The faces of the result descending from <paramref name="F"/>, a face of the basis shape or of the new shape
    /// (frozen).
    /// </summary>
    public NCollection_List<Shape> DescendantFaces(in Face F) =>
        LocOpe_WorkingShapes.Published(InternalDescendantFaces(F));

    /// <summary>
    /// <see cref="DescendantFaces"/> without freezing, for ported code (of the caller's face or of its working copy).
    /// </summary>
    internal NCollection_List<Shape> InternalDescendantFaces(in Face F)
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        Shape aF = myWork.Copied(F);
        if (myDescF.IsBound(aF))
        {
            return myDescF[aF];
        }
        return new NCollection_List<Shape>();
    }

    /// <summary>The basis shape (the caller's).</summary>
    public Shape BasisShape() => mySb;

    /// <summary>The new shape (the caller's, reversed for a cut).</summary>
    public Shape GluedShape() => mySn;

    /// <summary>The edges of the result along which the new shape is glued (frozen).</summary>
    public NCollection_List<Shape> Edges() => LocOpe_WorkingShapes.Published(myEdges);

    /// <summary><see cref="Edges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalEdges() => myEdges;

    /// <summary>The edges of <see cref="Edges"/> where the faces are tangent (frozen).</summary>
    public NCollection_List<Shape> TgtEdges() => LocOpe_WorkingShapes.Published(myTgtEdges);

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalTgtEdges() => myTgtEdges;

    /// <summary>The working shapes of the last <see cref="Perform"/>, for ported code (BRepFeat's descendants).</summary>
    internal LocOpe_WorkingShapes WorkingShapes() => myWork;

    private static Orientation GetOrientation(in Face Fn, in Face Fb)
    {
        Surface Sn = Tool.Surface(Fn)!;
        Surface Sb = Tool.Surface(Fb)!;

        // Find a point on Sb
        Explorer exp = new();
        Vector3D n1, n2;

        // Initialize extrema projector once for the target surface
        SurfaceAdaptor GAS = new(Sb);
        double TolU = GAS.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        double TolV = GAS.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        ExtPS anExtPS = new();
        anExtPS.Initialize(GAS,
                           GAS.FirstUParameter(),
                           GAS.LastUParameter(),
                           GAS.FirstVParameter(),
                           GAS.LastVParameter(),
                           TolU,
                           TolV);
        anExtPS.SetFlag(ExtFlag_MIN);

        for (exp.Init(Fn, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            Curve2D C2d = Tool.CurveOnSurface(edg, Fn, out double f, out double l)!;
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
            {
                f = -100.0;
                l = 100.0;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
            {
                f = l - 200.0;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
            {
                l = f + 200.0;
            }
            double deltau = (l - f) / 20.0;
            for (int i = 1; i <= 21; i++)
            {
                C2d.D0(f + (i - 1) * deltau, out Point2D ptvtx);
                Sn.D1(ptvtx.X(), ptvtx.Y(), out Point3D pvt, out Vector3D d1u, out Vector3D d1v);
                n1 = d1u.Crossed(d1v);
                if (n1.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    n1.Normalize();
                    if (Fn.Orientation() == Reversed)
                    {
                        n1.Reverse();
                    }

                    // Projection sur Sb
                    anExtPS.Perform(pvt);
                    if (anExtPS.IsDone)
                    {
                        double dist2min = RealLast();
                        int jmin = 0;
                        for (int j = 1; j <= anExtPS.ExtCount; j++)
                        {
                            if (anExtPS.SquareDistance(j) < dist2min)
                            {
                                jmin = j;
                                dist2min = anExtPS.SquareDistance(j);
                            }
                        }
                        if (jmin != 0)
                        {
                            anExtPS.Point(jmin).Parameter(out double uu, out double vv);
                            Sb.D1(uu, vv, out pvt, out d1u, out d1v);
                            n2 = d1u.Crossed(d1v);
                            if (n2.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                n2.Normalize();
                                if (Fb.Orientation() == Reversed)
                                {
                                    n2.Reverse();
                                }
                                if (n1.Dot(n2) > 0.0)
                                {
                                    return Reversed;
                                }
                                return Forward;
                            }
                        }
                    }
                }
            }
        }
        return Internal;
    }

    private static bool Contains(NCollection_List<Shape> L, in Shape S)
    {
        NCollection_List<Shape>.Iterator it = new();
        for (it.Initialize(L); it.More(); it.Next())
        {
            if (it.Value().IsSame(S))
            {
                return true;
            }
        }
        return false;
    }

    // OCCT computes, for each face of the result not of the new shape, which edges of the new shape touch it, and
    // keeps nothing of it: ported for its cost and its exceptions. On the working new shape.
    private void AddEdges(in Shape theSn)
    {
        Explorer exp = new(), expsb = new();
        exp.Init(theSn, ShapeType.Edge);

        NCollection_IndexedMap<Shape> MapV = new(ShapeHasher.Instance),
                                             MapFPrism = new(ShapeHasher.Instance),
                                             MapE = new(ShapeHasher.Instance);
        Explorer vexp = new();
        int flag, i;

        Exploration.MapShapes(theSn, ShapeType.Face, MapFPrism);

        for (expsb.Init(myRes, ShapeType.Face); expsb.More(); expsb.Next())
        {
            if (!MapFPrism.Contains(expsb.Current()))
            {
                MapV.Clear();
                Exploration.MapShapes(expsb.Current(), ShapeType.Vertex, MapV);
                Exploration.MapShapes(expsb.Current(), ShapeType.Edge, MapE);
                for (exp.Init(theSn, ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge e = ShapeCasts.Edge(exp.Current());
                    if (MapE.Contains(e))
                    {
                        continue;
                    }
                    flag = 0;
                    vexp.Init(e, ShapeType.Vertex);
                    for (; vexp.More(); vexp.Next())
                    {
                        Vertex v = ShapeCasts.Vertex(vexp.Current());
                        if (MapV.Contains(v))
                        {
                            flag = 1;
                        }
                    }
                    if (flag == 1)
                    {
                        vexp.Init(e, ShapeType.Vertex);
                        BRepExtrema_ExtPF ext = new();
                        ext.Initialize(ShapeCasts.Face(expsb.Current()));
                        flag = 0;
                        for (; vexp.More(); vexp.Next())
                        {
                            Vertex v = ShapeCasts.Vertex(vexp.Current());
                            if (!MapV.Contains(v))
                            {
                                ext.Perform(v, ShapeCasts.Face(expsb.Current()));
                                if (!ext.IsDone() || ext.NbExt() == 0)
                                {
                                    flag = 0;
                                    break;
                                }
                                else
                                {
                                    double dist2min = ext.SquareDistance(1);
                                    for (i = 2; i <= ext.NbExt(); i++)
                                    {
                                        dist2min = Min(dist2min, ext.SquareDistance(i));
                                    }
                                    if (dist2min >= Tool.Tolerance(v) * Tool.Tolerance(v))
                                    {
                                        flag = 0;
                                        break;
                                    }
                                    else
                                    {
                                        flag = 1;
                                    }
                                }
                            }
                            else
                            {
                                flag = 1;
                            }
                        }
                        if (flag == 1)
                        {
                        }
                    }
                }
            }
        }
    }
}
