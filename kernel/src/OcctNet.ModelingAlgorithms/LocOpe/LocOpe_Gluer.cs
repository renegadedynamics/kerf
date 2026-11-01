// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Gluer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Extrema_ExtFlag;
using static Occt.GeomAbs_Shape;
using static Occt.LocOpe_Operation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Glues a new shape on a basis shape by some of its faces, which lie on faces of the basis shape (a fuse or a cut,
/// depending on the faces' orientations).
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, it writes into both shapes: it splits the basis shape (point representations of its vertices), adds
/// pcurves to the new shape's edges, and sets the continuities of the edges. So both are worked on as copies when
/// frozen, sharing their copies (decision 8901): <see cref="Perform"/> splits, generates and recodes the regularities
/// on the copies, then gives back the caller's sub-shapes left unchanged. The shapes it hands out are frozen; ported
/// callers (BRepFeat) read the <c>Internal</c> forms.
/// </para>
/// </remarks>
public sealed class LocOpe_Gluer
{
    private bool myDone;
    private TopoDS_Shape mySb = new();
    private TopoDS_Shape mySn = new();
    private TopoDS_Shape myRes = new();
    private TopAbs_Orientation myOri;
    private LocOpe_Operation myOpe;
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myMapEF = new(TopTools_ShapeMapHasher.Instance);
    // OCCT iterates this map keyed by the edges' addresses; the port in insertion order
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMapEE = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myDescF = new(TopTools_ShapeMapHasher.Instance);
    private NCollection_List<TopoDS_Shape> myEdges = new();
    private NCollection_List<TopoDS_Shape> myTgtEdges = new();
    private LocOpe_WorkingShapes myWork = new();

    public LocOpe_Gluer()
    {
        myDone = false;
        myOri = TopAbs_INTERNAL;
        myOpe = LocOpe_INVALID;
    }

    public LocOpe_Gluer(in TopoDS_Shape Sbase, in TopoDS_Shape Snew)
    {
        myDone = false;
        mySb = Sbase;
        mySn = Snew;
        myOri = TopAbs_INTERNAL;
        myOpe = LocOpe_INVALID;
    }

    public void Init(in TopoDS_Shape Sbase, in TopoDS_Shape Snew)
    {
        mySb = Sbase;
        mySn = Snew;
        myMapEF.Clear();
        myMapEE.Clear();
        myDescF.Clear();
        myDone = false;
        myOri = TopAbs_INTERNAL;
        myOpe = LocOpe_INVALID;
    }

    /// <summary>Binds the face <paramref name="Fnew"/> of the new shape to the face <paramref name="Fbase"/> of the basis shape it lies on.</summary>
    public void Bind(in TopoDS_Face Fnew, in TopoDS_Face Fbase)
    {
        TopExp_Explorer exp = new(mySn, TopAbs_FACE);

        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Fnew))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }

        TopoDS_Shape aLocalFace = Fnew.Oriented(exp.Current().Orientation());
        TopoDS_Face Fnor = TopoDS.Face(aLocalFace);

        for (exp.Init(mySb, TopAbs_FACE); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Fbase))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }

        aLocalFace = Fbase.Oriented(exp.Current().Orientation());
        TopoDS_Face Fbor = TopoDS.Face(aLocalFace);
        TopAbs_Orientation Ori = GetOrientation(Fnor, Fbor);

        if (myOri == TopAbs_INTERNAL)
        {
            myOri = Ori;
            if (myOri == TopAbs_REVERSED)
            {
                mySn.Reverse();
                myOpe = LocOpe_CUT;
            }
            else
            {
                myOpe = LocOpe_FUSE;
            }
        }
        else if (Ori != TopAbs_FORWARD)
        {
            myOpe = LocOpe_INVALID;
        }

        for (exp.Init(Fnor, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
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
    public void Bind(in TopoDS_Edge Enew, in TopoDS_Edge Ebase)
    {
        if (myMapEE.IsBound(Enew) && !myMapEE[Enew].IsSame(Ebase))
        {
            throw new Standard_ConstructionError();
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
        if (mySb.IsNull() || mySn.IsNull() || myMapEF.IsEmpty() || myOpe == LocOpe_INVALID)
        {
            throw new Standard_ConstructionError();
        }

        // OCCT.NET: the basis shape and the new shape as worked on (their copies if frozen); the bindings are mapped
        myWork = new LocOpe_WorkingShapes();
        TopoDS_Shape aSb = myWork.Working(mySb);
        TopoDS_Shape aSn = myWork.Working(mySn);

        LocOpe_WiresOnShape theWOnS = new(aSb, myWork);
        LocOpe_GluedShape theGS = new(aSn);

        int lmap = myMapEF.Extent();

        for (ind = 1; ind <= lmap; ind++)
        {
            TopoDS_Shape S = myWork.Working(myMapEF.FindKey(ind));
            if (S.ShapeType() == TopAbs_EDGE)
            {
                TopoDS_Shape S2 = myWork.Working(myMapEF[ind]);
                if (!S2.IsNull())
                {
                    theWOnS.Bind(TopoDS.Edge(S), TopoDS.Face(S2));
                }
            }
            else
            { // TopAbs_FACE
                theGS.GlueOnFace(TopoDS.Face(S));
            }
        }

        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> itm in myMapEE)
        {
            theWOnS.Bind(TopoDS.Edge(myWork.Working(itm.Key)), TopoDS.Edge(myWork.Working(itm.Value)));
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
        TopExp_Explorer exp = new(aSb, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            myDescF.Bind(exp.Current(), new NCollection_List<TopoDS_Shape>(theSplit.InternalDescendantShapes(exp.Current())));
        }

        for (exp.Init(aSn, TopAbs_FACE); exp.More(); exp.Next())
        {
            NCollection_List<TopoDS_Shape> thelist = new();
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
            foreach (TopoDS_Shape aKey in myDescF.Keys.ToList())
            {
                NCollection_List<TopoDS_Shape> newDesc = new();
                NCollection_List<TopoDS_Shape>.Iterator itl = new();
                for (itl.Initialize(myDescF[aKey]); itl.More(); itl.Next())
                {
                    NCollection_List<TopoDS_Shape>.Iterator itl2 = new(theGen.InternalDescendantFace(TopoDS.Face(itl.Value())));
                    for (; itl2.More(); itl2.Next())
                    {
                        TopoDS_Face descface = TopoDS.Face(itl2.Value());
                        if (!descface.IsNull())
                        { // sinon la face a disparu
                            newDesc.Append(descface);
                        }
                    }
                }
                myDescF[aKey] = newDesc;
            }
        }

        // recodage des regularites
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapEF1 = new(TopTools_ShapeMapHasher.Instance),
                                                                                   theMapEF2 = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aSn, TopAbs_EDGE, TopAbs_FACE, theMapEF1);
        TopExp.MapShapesAndAncestors(myRes, TopAbs_EDGE, TopAbs_FACE, theMapEF2);

        for (ind = 1; ind <= theMapEF1.Extent(); ind++)
        {
            TopoDS_Edge edg = TopoDS.Edge(theMapEF1.FindKey(ind));
            NCollection_List<TopoDS_Shape> LL = theMapEF1[ind];
            if (LL.Extent() == 2)
            {
                TopoDS_Face fac1 = TopoDS.Face(LL.First());
                TopoDS_Face fac2 = TopoDS.Face(LL.Last());
                GeomAbs_Shape thecont = BRep_Tool.Continuity(edg, fac1, fac2);
                if (thecont >= GeomAbs_G1)
                {
                    // on essaie de recoder
                    int ind2 = theMapEF2.FindIndex(edg);
                    if (ind2 != 0)
                    {
                        NCollection_List<TopoDS_Shape> LL2 = theMapEF2[ind2];
                        if (LL2.Extent() == 2)
                        {
                            TopoDS_Face ff1 = TopoDS.Face(LL2.First());
                            TopoDS_Face ff2 = TopoDS.Face(LL2.Last());
                            if ((ff1.IsSame(fac1) && ff2.IsSame(fac2)) || (ff1.IsSame(fac2) && ff2.IsSame(fac1)))
                            {
                            }
                            else
                            {
                                BRep_Builder B = new();
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
            TopoDS_Edge edg = theWOnS.InternalEdge();
            for (ind = 1; ind <= theMapEF2.Extent(); ind++)
            {
                TopoDS_Edge edg1 = TopoDS.Edge(theMapEF2.FindKey(ind));
                if (edg1.IsSame(edg))
                {
                    myEdges.Append(edg);
                    // recodage eventuel des regularites sur cet edge
                    NCollection_List<TopoDS_Shape> L = theMapEF2[ind];
                    if (L.Extent() == 2)
                    {
                        TopoDS_Face fac1 = TopoDS.Face(L.First());
                        TopoDS_Face fac2 = TopoDS.Face(L.Last());
                        if (LocOpe.TgtFaces(edg, fac1, fac2))
                        {
                            myTgtEdges.Append(edg);
                            GeomAbs_Shape thecont = BRep_Tool.Continuity(edg, fac1, fac2);
                            if (thecont < GeomAbs_G1)
                            {
                                BRep_Builder B = new();
                                B.Continuity(edg, fac1, fac2, GeomAbs_G1);
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
            var aShapes = new List<TopoDS_Shape> { myRes };
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myDescF)
            {
                aShapes.AddRange(anItem.Value);
            }
            aShapes.AddRange(myEdges);
            aShapes.AddRange(myTgtEdges);
            myWork.Finish(aShapes);
            myRes = myWork.Final(myRes);
            foreach (TopoDS_Shape aKey in myDescF.Keys.ToList())
            {
                myDescF[aKey] = myWork.Final(myDescF[aKey]);
            }
            myEdges = myWork.Final(myEdges);
            myTgtEdges = myWork.Final(myTgtEdges);
        }
    }

    public bool IsDone() => myDone;

    /// <summary>The glued shape (frozen).</summary>
    public TopoDS_Shape ResultingShape() => LocOpe_WorkingShapes.Published(InternalResultingShape());

    /// <summary><see cref="ResultingShape"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalResultingShape()
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        return myRes;
    }

    /// <summary>
    /// The faces of the result descending from <paramref name="F"/>, a face of the basis shape or of the new shape
    /// (frozen).
    /// </summary>
    public NCollection_List<TopoDS_Shape> DescendantFaces(in TopoDS_Face F) =>
        LocOpe_WorkingShapes.Published(InternalDescendantFaces(F));

    /// <summary>
    /// <see cref="DescendantFaces"/> without freezing, for ported code (of the caller's face or of its working copy).
    /// </summary>
    internal NCollection_List<TopoDS_Shape> InternalDescendantFaces(in TopoDS_Face F)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        TopoDS_Shape aF = myWork.Copied(F);
        if (myDescF.IsBound(aF))
        {
            return myDescF[aF];
        }
        return new NCollection_List<TopoDS_Shape>();
    }

    /// <summary>The basis shape (the caller's).</summary>
    public TopoDS_Shape BasisShape() => mySb;

    /// <summary>The new shape (the caller's, reversed for a cut).</summary>
    public TopoDS_Shape GluedShape() => mySn;

    /// <summary>The edges of the result along which the new shape is glued (frozen).</summary>
    public NCollection_List<TopoDS_Shape> Edges() => LocOpe_WorkingShapes.Published(myEdges);

    /// <summary><see cref="Edges"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalEdges() => myEdges;

    /// <summary>The edges of <see cref="Edges"/> where the faces are tangent (frozen).</summary>
    public NCollection_List<TopoDS_Shape> TgtEdges() => LocOpe_WorkingShapes.Published(myTgtEdges);

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalTgtEdges() => myTgtEdges;

    /// <summary>The working shapes of the last <see cref="Perform"/>, for ported code (BRepFeat's descendants).</summary>
    internal LocOpe_WorkingShapes WorkingShapes() => myWork;

    private static TopAbs_Orientation GetOrientation(in TopoDS_Face Fn, in TopoDS_Face Fb)
    {
        Geom_Surface Sn = BRep_Tool.Surface(Fn)!;
        Geom_Surface Sb = BRep_Tool.Surface(Fb)!;

        // Find a point on Sb
        TopExp_Explorer exp = new();
        gp_Vec n1, n2;

        // Initialize extrema projector once for the target surface
        GeomAdaptor_Surface GAS = new(Sb);
        double TolU = GAS.UResolution(Precision.Confusion());
        double TolV = GAS.VResolution(Precision.Confusion());
        Extrema_ExtPS anExtPS = new();
        anExtPS.Initialize(GAS,
                           GAS.FirstUParameter(),
                           GAS.LastUParameter(),
                           GAS.FirstVParameter(),
                           GAS.LastVParameter(),
                           TolU,
                           TolV);
        anExtPS.SetFlag(Extrema_ExtFlag_MIN);

        for (exp.Init(Fn, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            Geom2d_Curve C2d = BRep_Tool.CurveOnSurface(edg, Fn, out double f, out double l)!;
            if (Precision.IsNegativeInfinite(f) && Precision.IsPositiveInfinite(l))
            {
                f = -100.0;
                l = 100.0;
            }
            else if (Precision.IsNegativeInfinite(f))
            {
                f = l - 200.0;
            }
            else if (Precision.IsPositiveInfinite(l))
            {
                l = f + 200.0;
            }
            double deltau = (l - f) / 20.0;
            for (int i = 1; i <= 21; i++)
            {
                C2d.D0(f + (i - 1) * deltau, out gp_Pnt2d ptvtx);
                Sn.D1(ptvtx.X(), ptvtx.Y(), out gp_Pnt pvt, out gp_Vec d1u, out gp_Vec d1v);
                n1 = d1u.Crossed(d1v);
                if (n1.Magnitude() > Precision.Confusion())
                {
                    n1.Normalize();
                    if (Fn.Orientation() == TopAbs_REVERSED)
                    {
                        n1.Reverse();
                    }

                    // Projection sur Sb
                    anExtPS.Perform(pvt);
                    if (anExtPS.IsDone())
                    {
                        double dist2min = RealLast();
                        int jmin = 0;
                        for (int j = 1; j <= anExtPS.NbExt(); j++)
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
                            if (n2.Magnitude() > Precision.Confusion())
                            {
                                n2.Normalize();
                                if (Fb.Orientation() == TopAbs_REVERSED)
                                {
                                    n2.Reverse();
                                }
                                if (n1.Dot(n2) > 0.0)
                                {
                                    return TopAbs_REVERSED;
                                }
                                return TopAbs_FORWARD;
                            }
                        }
                    }
                }
            }
        }
        return TopAbs_INTERNAL;
    }

    private static bool Contains(NCollection_List<TopoDS_Shape> L, in TopoDS_Shape S)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new();
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
    private void AddEdges(in TopoDS_Shape theSn)
    {
        TopExp_Explorer exp = new(), expsb = new();
        exp.Init(theSn, TopAbs_EDGE);

        NCollection_IndexedMap<TopoDS_Shape> MapV = new(TopTools_ShapeMapHasher.Instance),
                                             MapFPrism = new(TopTools_ShapeMapHasher.Instance),
                                             MapE = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer vexp = new();
        int flag, i;

        TopExp.MapShapes(theSn, TopAbs_FACE, MapFPrism);

        for (expsb.Init(myRes, TopAbs_FACE); expsb.More(); expsb.Next())
        {
            if (!MapFPrism.Contains(expsb.Current()))
            {
                MapV.Clear();
                TopExp.MapShapes(expsb.Current(), TopAbs_VERTEX, MapV);
                TopExp.MapShapes(expsb.Current(), TopAbs_EDGE, MapE);
                for (exp.Init(theSn, TopAbs_EDGE); exp.More(); exp.Next())
                {
                    TopoDS_Edge e = TopoDS.Edge(exp.Current());
                    if (MapE.Contains(e))
                    {
                        continue;
                    }
                    flag = 0;
                    vexp.Init(e, TopAbs_VERTEX);
                    for (; vexp.More(); vexp.Next())
                    {
                        TopoDS_Vertex v = TopoDS.Vertex(vexp.Current());
                        if (MapV.Contains(v))
                        {
                            flag = 1;
                        }
                    }
                    if (flag == 1)
                    {
                        vexp.Init(e, TopAbs_VERTEX);
                        BRepExtrema_ExtPF ext = new();
                        ext.Initialize(TopoDS.Face(expsb.Current()));
                        flag = 0;
                        for (; vexp.More(); vexp.Next())
                        {
                            TopoDS_Vertex v = TopoDS.Vertex(vexp.Current());
                            if (!MapV.Contains(v))
                            {
                                ext.Perform(v, TopoDS.Face(expsb.Current()));
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
                                    if (dist2min >= BRep_Tool.Tolerance(v) * BRep_Tool.Tolerance(v))
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
