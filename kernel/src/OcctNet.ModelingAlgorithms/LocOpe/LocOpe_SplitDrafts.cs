// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitDrafts.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitDrafts.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.IntAna_ResultType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Splits a planar face of a shape by a wire lying on it and puts a draft angle on one or both parts.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, the splitting and the rebuilding write into the shape and the wire (vertex point representations,
/// pcurves, continuities, wires added to the faces it reuses). So the shape, the face and the wire are worked on as
/// copies when frozen, sharing their copies (decision 1013), and the caller's sub-shapes left unchanged come back in
/// the result. <see cref="Shape"/> and <see cref="ShapesFromShape"/> freeze what they hand out.
/// </para>
/// <para>
/// OCCT writes its failures to std::cout ("LocOpe_SplitDrafts:!Fins ...", "fin newplane return standard_false",
/// "LocOpe_SplitDrafts: betite probleme", "Ca merde violemment", "Not yet implemented : nbwire >= 2"); the port leaves
/// the messages out.
/// </para>
/// </remarks>
public sealed class LocOpe_SplitDrafts
{
    private TopoDS_Shape myShape = new();
    private TopoDS_Shape myResult = new();
    // OCCT iterates this map keyed by the shapes' addresses to store the descendants; each entry is handled alone,
    // apart from the last pass collecting the faces of the result (whose order follows the map's: see the remarks of
    // LocOpe_BuildShape)
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(TopTools_ShapeMapHasher.Instance);
    private LocOpe_WorkingShapes myWork = new();

    /// <summary>Empty constructor.</summary>
    public LocOpe_SplitDrafts()
    {
    }

    /// <summary>Creates the algorithm on the shape <paramref name="S"/>.</summary>
    public LocOpe_SplitDrafts(in TopoDS_Shape S)
    {
        myShape = S;
    }

    /// <summary>Initializes the algorithm with the shape <paramref name="S"/>.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
        myResult.Nullify();
        myMap.Clear();
    }

    /// <summary>
    /// Splits the face <paramref name="F"/> with the wire <paramref name="W"/> and puts the draft angle
    /// <paramref name="Angle"/>, measured from <paramref name="Extr"/> with the neutral plane <paramref name="NPl"/>, on
    /// the left part.
    /// </summary>
    public void Perform(in TopoDS_Face F, in TopoDS_Wire W, in Direction3D Extr, in Plane3D NPl, double Angle)
    {
        Perform(F, W, Extr, NPl, Angle, Extr, NPl, Angle, true, false);
    }

    /// <summary>
    /// Splits the face <paramref name="F"/> of the shape with the wire <paramref name="W"/>, lying on it, and puts a
    /// draft angle on both parts: <paramref name="Extrg"/>, <paramref name="NPlg"/>, <paramref name="Angleg"/> for the
    /// left part, <paramref name="Extrd"/>, <paramref name="NPld"/>, <paramref name="Angled"/> for the right part, each
    /// part left as it is when its Modify flag is false.
    /// </summary>
    public void Perform(in TopoDS_Face F,
                        in TopoDS_Wire W,
                        in Direction3D Extrg,
                        in Plane3D NPlg,
                        double Angleg,
                        in Direction3D Extrd,
                        in Plane3D NPld,
                        double Angled,
                        bool ModLeft = true,
                        bool ModRight = true)
    {
        int j;

        myResult.Nullify();
        myMap.Clear();
        if (myShape.IsNull() || F.IsNull() || W.IsNull())
        {
            throw new NullObjectException();
        }

        if (!ModLeft && !ModRight)
        {
            throw new ConstructionErrorException();
        }

        // OCCT.NET: the shape, the face and the wire as worked on (their copies if frozen, decision 1013)
        myWork = new LocOpe_WorkingShapes();
        TopoDS_Shape aShape = myWork.Working(myShape);
        TopoDS_Face aF = TopoDS.Face(myWork.Working(F));
        TopoDS_Wire aW = TopoDS.Wire(myWork.Working(W));

        TopAbs_Orientation OriF = TopAbs_FORWARD;

        bool FinS = false;
        TopExp_Explorer exp = new(), exp2 = new();
        for (exp.Init(aShape, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Shape fac = exp.Current();
            NCollection_List<TopoDS_Shape> thelist = new();
            myMap.Bind(fac, thelist);
            if (fac.IsSame(aF))
            {
                OriF = fac.Orientation();
                FinS = true;
            }
        }

        if (!FinS)
        {
            throw new ConstructionErrorException();
        }

        TopoDS_Shape aLocalFace = aF.Oriented(OriF);

        if (!NewPlane(TopoDS.Face(aLocalFace), Extrg, NPlg, Angleg, out Plane3D NewPlg, out Axis3D NormalFg, ModLeft)
            || !NewPlane(TopoDS.Face(aLocalFace), Extrd, NPld, Angled, out Plane3D NewPld, out Axis3D NormalFd, ModRight))
        {
            return;
        }
        _ = NormalFd;

        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        BRep_Builder B = new();

        Geom_Surface NewSg = new Geom_Plane(NewPlg);
        Geom_Surface NewSd = new Geom_Plane(NewPld);
        Geom_Line theLinePipe = new(NormalFg); // ou NormalFd
        GeomInt_IntSS i2s = new(NewSg, NewSd, global::Kerf.Engine.Foundation.Precision.Confusion());

        NCollection_Map<TopoDS_Shape> theMap = new(TopTools_ShapeMapHasher.Instance);
        GeomAdaptor_Curve HAC = new();
        GeomAdaptor_Surface HAS = new();
        IntCurveSurface_HInter intcs = new();

        TopoDS_Wire theW = aW;
        if (i2s.IsDone() && i2s.NbLines() > 0)
        {
            // on split le wire

            GeomFill_Pipe thePipe = new();
            thePipe.GenerateParticularCase(true);
            // OCCT: the planes' intersection is an infinite line, which GeomFill_UniformSection can't convert to a
            // B-spline: Standard_DomainError (occt-issues.md)
            thePipe.Init(theLinePipe, i2s.Line(1));
            thePipe.Perform(true);
            if (!thePipe.IsDone())
            {
                throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
            }

            Geom_Surface Spl = thePipe.InternalSurface()!;
            HAS.Load(Spl);

            LocOpe_SplitShape splw = new(aW, myWork);

            for (exp.Init(aW, TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                if (theMap.Add(edg))
                {
                    Geom_Curve? C = BRep_Tool.Curve(edg, out double f, out double l);
                    HAC.Load(C!);
                    intcs.Perform(HAC, HAS);
                    if (!intcs.IsDone())
                    {
                        continue; // voir ce qu`on peut faire de mieux
                    }

                    if (intcs.NbSegments() >= 2)
                    {
                        continue; // Not yet implemented...and probably never"
                    }

                    if (intcs.NbSegments() == 1)
                    {
                        IntCurveSurface_IntersectionPoint P1 = intcs.Segment(1).FirstPoint();
                        IntCurveSurface_IntersectionPoint P2 = intcs.Segment(1).SecondPoint();
                        Point3D pf = P1.Pnt();
                        Point3D pl = P2.Pnt();
                        TopExp.Vertices(edg, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
                        Point3D Pf = BRep_Tool.Pnt(Vf);
                        Point3D Pl = BRep_Tool.Pnt(Vl);
                        double Tolf = BRep_Tool.Tolerance(Vf);
                        double Toll = BRep_Tool.Tolerance(Vl);
                        Tolf *= Tolf;
                        Toll *= Toll;

                        double dff = pf.SquareDistance(Pf);
                        double dfl = pf.SquareDistance(Pl);
                        double dlf = pl.SquareDistance(Pf);
                        double dll = pl.SquareDistance(Pl);

                        if ((dff <= Tolf && dll <= Toll) || (dlf <= Tolf && dfl <= Toll))
                        {
                            continue;
                        }
                        else
                        {
                            // on segmente edg en pf et pl
                            TopoDS_Vertex Vnewf = new(), Vnewl = new();
                            B.MakeVertex(ref Vnewf, pf, global::Kerf.Engine.Foundation.Precision.Confusion());
                            B.MakeVertex(ref Vnewl, pl, global::Kerf.Engine.Foundation.Precision.Confusion());
                            if (P1.W() >= f && P1.W() <= l && P2.W() >= f && P2.W() <= l)
                            {
                                splw.InternalAdd(Vnewf, P1.W(), edg);
                                splw.InternalAdd(Vnewl, P2.W(), edg);
                            }
                            else
                            {
                                continue;
                            }
                        }
                    }
                    else if (intcs.NbPoints() != 0)
                    {
                        TopExp.Vertices(edg, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
                        Point3D Pf = BRep_Tool.Pnt(Vf);
                        Point3D Pl = BRep_Tool.Pnt(Vl);
                        double Tolf = BRep_Tool.Tolerance(Vf);
                        double Toll = BRep_Tool.Tolerance(Vl);
                        Tolf *= Tolf;
                        Toll *= Toll;

                        for (int i = 1; i <= intcs.NbPoints(); i++)
                        {
                            IntCurveSurface_IntersectionPoint Pi = intcs.Point(i);
                            Point3D pi = Pi.Pnt();
                            double dif = pi.SquareDistance(Pf);
                            double dil = pi.SquareDistance(Pl);
                            if (dif <= Tolf)
                            {
                            }
                            else if (dil <= Toll)
                            {
                            }
                            else
                            {
                                if (Pi.W() >= f && Pi.W() <= l)
                                {
                                    TopoDS_Vertex Vnew = new();
                                    B.MakeVertex(ref Vnew, pi, global::Kerf.Engine.Foundation.Precision.Confusion());
                                    splw.InternalAdd(Vnew, Pi.W(), edg);
                                }
                            }
                        }
                    }
                }
            }

            NCollection_List<TopoDS_Shape> lres = splw.InternalDescendantShapes(aW);
            if (lres.Extent() != 1)
            {
                return;
            }

            if (!aW.IsSame(lres.First()))
            {
                theW.Nullify();
                theW = TopoDS.Wire(lres.First());
            }

            for (exp.ReInit(); exp.More(); exp.Next())
            {
                if (!myMap.IsBound(exp.Current()))
                {
                    NCollection_List<TopoDS_Shape> thelist1 = new();
                    myMap.Bind(exp.Current(), thelist1);
                    for (itl.Initialize(splw.InternalDescendantShapes(exp.Current())); itl.More(); itl.Next())
                    {
                        myMap[exp.Current()].Append(itl.Value());
                    }
                    for (exp2.Init(exp.Current(), TopAbs_VERTEX); exp2.More(); exp2.Next())
                    {
                        if (!myMap.IsBound(exp2.Current()))
                        {
                            NCollection_List<TopoDS_Shape> thelist2 = new();
                            myMap.Bind(exp2.Current(), thelist2);
                            myMap[exp2.Current()].Append(exp2.Current());
                        }
                    }
                }
            }
        }
        else
        {
            for (exp.Init(aW, TopAbs_EDGE); exp.More(); exp.Next())
            {
                if (!myMap.IsBound(exp.Current()))
                {
                    NCollection_List<TopoDS_Shape> thelist3 = new();
                    myMap.Bind(exp.Current(), thelist3);
                    myMap[exp.Current()].Append(exp.Current());
                    for (exp2.Init(exp.Current(), TopAbs_VERTEX); exp2.More(); exp2.Next())
                    {
                        if (!myMap.IsBound(exp2.Current()))
                        {
                            NCollection_List<TopoDS_Shape> thelist4 = new();
                            myMap.Bind(exp2.Current(), thelist4);
                            myMap[exp2.Current()].Append(exp2.Current());
                        }
                    }
                }
            }
        }

        // On split la face par le wire

        LocOpe_WiresOnShape WonS = new(aShape, myWork);
        LocOpe_Spliter Spls = new(aShape);
        WonS.Bind(theW, aF);

        // JAG Le code suivant marchera apres integration de thick0 (LocOpe_FindEdges, commented out in OCCT)

        Spls.InternalPerform(WonS);
        if (!Spls.IsDone())
        {
            return;
        }

        TopoDS_Shape Res = Spls.InternalResultingShape();
        NCollection_List<TopoDS_Shape> theLeft = Spls.InternalDirectLeft();

        // Descendants
        for (exp.Init(aShape, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Shape fac = exp.Current();
            for (itl.Initialize(Spls.InternalDescendantShapes(fac)); itl.More(); itl.Next())
            {
                myMap[fac].Append(itl.Value());
            }
        }

        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapW = new(TopTools_ShapeMapHasher.Instance);
        for (exp.Init(theW, TopAbs_EDGE); exp.More(); exp.Next())
        {
            if (!MapW.IsBound(exp.Current()))
            {
                MapW.Bind(exp.Current(), new TopoDS_Shape());
                for (exp2.Init(exp.Current(), TopAbs_VERTEX); exp2.More(); exp2.Next())
                {
                    if (!MapW.IsBound(exp2.Current()))
                    {
                        MapW.Bind(exp2.Current(), new TopoDS_Shape());
                    }
                }
            }
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(Res, TopAbs_EDGE, TopAbs_FACE, theMapEF);

        // On stocke les geometries potentiellement generees par les edges
        NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> MapEV = new(TopTools_ShapeMapHasher.Instance); // genere
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MapSg = new(TopTools_ShapeMapHasher.Instance),
                                                                          MapSd = new(TopTools_ShapeMapHasher.Instance); // image a gauche et a droite

        int Nbedges, index;
        for (itl.Initialize(myMap[aF]); itl.More(); itl.Next())
        {
            TopoDS_Shape fac = TopoDS.Face(itl.Value());
            for (exp.Init(fac, TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Shape edg = exp.Current();
                if (MapEV.FindIndex(edg) != 0)
                {
                    continue;
                }
                if (MapW.IsBound(edg))
                { // edge du wire initial
                    Geom_Curve C = BRep_Tool.Curve(TopoDS.Edge(edg), out Placement Loc, out _, out _)!;
                    if (C.GetType() == typeof(Geom_TrimmedCurve))
                    {
                        C = ((Geom_TrimmedCurve)C).BasisCurve();
                    }
                    C = (Geom_Curve)C.Transformed(Loc.Transformation());

                    GeomFill_Pipe thePipe = new();
                    thePipe.GenerateParticularCase(true);
                    thePipe.Init(theLinePipe, C);
                    thePipe.Perform(true);
                    if (!thePipe.IsDone())
                    {
                        throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
                    }

                    Geom_Surface thePS = thePipe.InternalSurface()!;
                    if (thePS.GetType() == typeof(Geom_RectangularTrimmedSurface))
                    {
                        thePS = ((Geom_RectangularTrimmedSurface)thePS).BasisSurface();
                    }

                    TopoDS_Face NewFace = new();
                    B.MakeFace(ref NewFace, thePS, global::Kerf.Engine.Foundation.Precision.Confusion());
                    MapEV.Add(edg, NewFace);
                }
                else
                { // on recupere la face.
                    index = theMapEF.FindIndex(edg);
                    if (theMapEF[index].Extent() != 2)
                    {
                        return; // NotDone
                    }
                    if (theMapEF[index].First().IsSame(fac))
                    {
                        MapEV.Add(edg, theMapEF[index].Last());
                    }
                    else
                    {
                        MapEV.Add(edg, theMapEF[index].First());
                    }
                }
            }
        }

        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapSonS = new(TopTools_ShapeMapHasher.Instance);

        Nbedges = MapEV.Extent();
        for (index = 1; index <= Nbedges; index++)
        {
            for (exp.Init(MapEV.FindKey(index), TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
                if (MapEV.FindIndex(vtx) != 0)
                {
                    continue;
                }

                // Localisation du vertex :
                //    - entre 2 edges d`origine : on recupere l`edge qui n`est
                //                                pas dans F
                //    - entre 2 edges du wire   : droite
                //    - mixte                   : intersection de surfaces
                for (j = 1; j <= Nbedges; j++)
                {
                    if (j == index)
                    {
                        continue;
                    }
                    for (exp2.Init(MapEV.FindKey(j), TopAbs_VERTEX); exp2.More(); exp2.Next())
                    {
                        TopoDS_Shape vtx2 = exp2.Current();
                        if (vtx2.IsSame(vtx))
                        {
                            break;
                        }
                    }
                    if (exp2.More())
                    {
                        break;
                    }
                }
                int Choice = 0;
                TopoDS_Shape edg1 = MapEV.FindKey(index);
                TopoDS_Shape edg2;
                if (j <= Nbedges)
                {
                    edg2 = MapEV.FindKey(j);
                }
                else
                {
                    edg2 = edg1;
                }
                if (MapW.IsBound(edg1))
                {
                    if (j > Nbedges)
                    {             // doit correspondre a edge ferme
                        Choice = 2; // droite
                    }
                    else if (MapW.IsBound(MapEV.FindKey(j)))
                    {
                        Choice = 2; // droite
                    }
                    else
                    {
                        Choice = 3; // mixte
                    }
                }
                else
                {
                    if (j > Nbedges)
                    {             // doit correspondre a edge ferme
                        Choice = 1; // edge a retrouver
                    }
                    else if (!MapW.IsBound(MapEV.FindKey(j)))
                    {
                        Choice = 1; // edge a retrouver
                    }
                    else
                    {
                        Choice = 3; // mixte
                    }
                }
                Geom_Curve? Newc = null;
                Geom2d_Curve? newCs1 = null, newCs2 = null;
                double knownp = 0;
                TopoDS_Edge Ebind = new();
                switch (Choice)
                {
                    case 1:
                    {
                        for (exp2.Init(Res, TopAbs_EDGE); exp2.More(); exp2.Next())
                        {
                            if (exp2.Current().IsSame(edg1) || exp2.Current().IsSame(edg2))
                            {
                                continue;
                            }
                            TopExp_Explorer exp3 = new(exp2.Current().Oriented(TopAbs_FORWARD), TopAbs_VERTEX);
                            for (; exp3.More(); exp3.Next())
                            {
                                if (exp3.Current().IsSame(vtx))
                                {
                                    break;
                                }
                            }
                            if (exp3.More())
                            {
                                break;
                            }
                        }
                        if (exp2.More())
                        {
                            Newc = BRep_Tool.Curve(TopoDS.Edge(exp2.Current()), out Placement Loc, out _, out _)!;
                            Newc = (Geom_Curve)Newc.Transformed(Loc.Transformation());
                            Ebind = TopoDS.Edge(exp2.Current());
                            knownp = BRep_Tool.Parameter(vtx, Ebind);
                        }
                        else
                        { // droite ??? il vaudrait mieux sortir
                            return;
                        }
                    }
                    break;
                    case 2:
                    {
                        Line3D theLine = new(NormalFg);
                        theLine.Translate(NormalFg.Location(), BRep_Tool.Pnt(vtx));
                        Newc = new Geom_Line(theLine);
                        knownp = 0.0;
                    }
                    break;
                    case 3:
                    {
                        TopoDS_Face F1 = TopoDS.Face(MapEV.FindFromKey(edg1));
                        TopoDS_Face F2 = TopoDS.Face(MapEV.FindFromKey(edg2));
                        Geom_Surface S1 = BRep_Tool.Surface(F1)!;
                        Geom_Surface S2 = BRep_Tool.Surface(F2)!;
                        bool AppS1 = false;
                        bool AppS2 = false;
                        if (S1.GetType() != typeof(Geom_Plane))
                        {
                            AppS1 = true;
                        }
                        if (S2.GetType() != typeof(Geom_Plane))
                        {
                            AppS2 = true;
                        }
                        i2s.Perform(S1, S2, global::Kerf.Engine.Foundation.Precision.Confusion(), true, AppS1, AppS2);
                        if (!i2s.IsDone() || i2s.NbLines() <= 0)
                        {
                            return;
                        }

                        double pmin = 0, Dist2 = 0, Dist2Min = 0, Glob2Min = RealLast();
                        GeomAdaptor_Curve TheCurve = new();

                        int i, imin, k;
                        Point3D pv = BRep_Tool.Pnt(vtx);
                        imin = 0;
                        for (i = 1; i <= i2s.NbLines(); i++)
                        {
                            TheCurve.Load(i2s.Line(i));
                            Extrema_ExtPC myExtPC = new(pv, TheCurve);

                            if (myExtPC.IsDone())
                            {
                                double thepmin = TheCurve.FirstParameter();
                                myExtPC.TrimmedSquareDistances(out Dist2Min, out Dist2, out Point3D p1b, out Point3D p2b);
                                if (Dist2 < Dist2Min)
                                {
                                    thepmin = TheCurve.LastParameter();
                                }
                                for (k = 1; k <= myExtPC.NbExt(); k++)
                                {
                                    Dist2 = myExtPC.SquareDistance(k);
                                    if (Dist2 < Dist2Min)
                                    {
                                        Dist2Min = Dist2;
                                        thepmin = myExtPC.Point(k).Parameter();
                                    }
                                }

                                if (Dist2Min < Glob2Min)
                                {
                                    Glob2Min = Dist2Min;
                                    pmin = thepmin;
                                    imin = i;
                                }
                            }
                        }
                        if (imin == 0)
                        {
                            return;
                        }

                        Newc = i2s.Line(imin);
                        knownp = pmin;
                        if (AppS1)
                        {
                            newCs1 = i2s.LineOnS1(imin);
                        }
                        if (AppS2)
                        {
                            newCs2 = i2s.LineOnS2(imin);
                        }
                    }
                    break;
                }

                // Determination des vertex par intersection sur Plg ou/et Pld

                HAC.Load(Newc!);
                int nbfois = 2;
                TopoDS_Vertex vtx1 = new(), vtx2b = new();
                double p1 = 0, p2 = 0;
                bool IsLeft = false;
                if (Choice == 1)
                {
                    // edge retrouve : on ne fait qu`une seule intersection
                    // il faut utiliser Plg ou Pld

                    int indedgf = theMapEF.FindIndex(edg1);
                    for (itl.Initialize(theMapEF[indedgf]); itl.More(); itl.Next())
                    {
                        if (Contains(myMap[aF], itl.Value()))
                        {
                            if (Contains(theLeft, itl.Value()))
                            {
                                HAS.Load(NewSg);
                                IsLeft = true;
                            }
                            else
                            {
                                HAS.Load(NewSd);
                                IsLeft = false;
                            }

                            nbfois = 1;
                            vtx2b = vtx;
                            p2 = knownp;
                            break;
                        }
                    }
                    if (!itl.More())
                    {
                        // OCCT writes "LocOpe_SplitDrafts: betite probleme" to std::cout (left out)
                        return;
                    }
                }
                else
                {
                    HAS.Load(NewSg);
                }

                for (int it = 1; it <= nbfois; it++)
                {
                    if (it == 2)
                    {
                        HAS.Load(NewSd);
                    }

                    intcs.Perform(HAC, HAS);
                    if (!intcs.IsDone())
                    {
                        return; // voir ce qu`on peut faire de mieux
                    }
                    int imin = 1;
                    double delta = Math.Abs(knownp - intcs.Point(1).W());
                    for (int i = 2; i <= intcs.NbPoints(); i++)
                    {
                        double newdelta = Math.Abs(knownp - intcs.Point(i).W());
                        if (newdelta < delta)
                        {
                            imin = i;
                            delta = newdelta;
                        }
                    }
                    if (it == 1)
                    {
                        B.MakeVertex(ref vtx1, intcs.Point(imin).Pnt(), global::Kerf.Engine.Foundation.Precision.Confusion());
                        p1 = intcs.Point(imin).W();
                        knownp = p1;
                    }
                    else
                    {
                        B.MakeVertex(ref vtx2b, intcs.Point(imin).Pnt(), global::Kerf.Engine.Foundation.Precision.Confusion());
                        p2 = intcs.Point(imin).W();
                    }
                }
                if (Math.Abs(p1 - p2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    TopoDS_Edge NewEdge = new();
                    B.MakeEdge(ref NewEdge, Newc, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (p1 < p2)
                    {
                        B.Add(NewEdge, vtx1.Oriented(TopAbs_FORWARD));
                        B.Add(NewEdge, vtx2b.Oriented(TopAbs_REVERSED));
                    }
                    else
                    {
                        B.Add(NewEdge, vtx1.Oriented(TopAbs_REVERSED));
                        B.Add(NewEdge, vtx2b.Oriented(TopAbs_FORWARD));
                    }
                    B.UpdateVertex(vtx1, p1, NewEdge, global::Kerf.Engine.Foundation.Precision.Confusion());
                    B.UpdateVertex(vtx2b, p2, NewEdge, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (newCs1 is not null)
                    {
                        B.UpdateEdge(NewEdge, newCs1, TopoDS.Face(MapEV.FindFromKey(edg1)), global::Kerf.Engine.Foundation.Precision.Confusion());
                    }

                    if (newCs2 is not null)
                    {
                        B.UpdateEdge(NewEdge, newCs2, TopoDS.Face(MapEV.FindFromKey(edg2)), global::Kerf.Engine.Foundation.Precision.Confusion());
                    }

                    MapEV.Add(vtx, NewEdge);

                    if (Choice == 1)
                    {
                        TopoDS_Shape aLocalEdge = Ebind.EmptyCopied();
                        TopoDS_Edge NE = TopoDS.Edge(aLocalEdge);
                        for (exp2.Init(Ebind, TopAbs_VERTEX); exp2.More(); exp2.Next())
                        {
                            TopoDS_Vertex thevtx = TopoDS.Vertex(exp2.Current());
                            if (thevtx.IsSame(vtx))
                            {
                                B.Add(NE, vtx1.Oriented(thevtx.Orientation()));
                                B.UpdateVertex(vtx1, p1, NE, global::Kerf.Engine.Foundation.Precision.Confusion());
                            }
                            else
                            {
                                B.Add(NE, thevtx);
                                double theprm = BRep_Tool.Parameter(thevtx, Ebind);
                                B.UpdateVertex(thevtx, theprm, NE, BRep_Tool.Tolerance(thevtx));
                            }
                        }
                        MapSonS.Bind(Ebind, NE.Oriented(TopAbs_FORWARD));
                        if (IsLeft)
                        {
                            NCollection_List<TopoDS_Shape> thelist5 = new();
                            MapSg.Bind(vtx, thelist5);
                            MapSg[vtx].Append(vtx1);
                        }
                        else
                        {
                            NCollection_List<TopoDS_Shape> thelist6 = new();
                            MapSd.Bind(vtx, thelist6);
                            MapSd[vtx].Append(vtx1);
                        }
                    }
                    else
                    {
                        NCollection_List<TopoDS_Shape> thelist7 = new(), thelist8 = new();
                        MapSg.Bind(vtx, thelist7);
                        MapSd.Bind(vtx, thelist8);
                        MapSg[vtx].Append(vtx1);
                        MapSd[vtx].Append(vtx2b);
                    }
                }
                else
                {
                    MapEV.Add(vtx, vtx2b); // on peut avoir vtx2 = vtx si choix == 1
                    if (Choice == 1)
                    {
                        if (IsLeft)
                        {
                            NCollection_List<TopoDS_Shape> thelist9 = new();
                            MapSg.Bind(vtx, thelist9);
                            MapSg[vtx].Append(vtx);
                        }
                        else
                        {
                            NCollection_List<TopoDS_Shape> thelist10 = new();
                            MapSd.Bind(vtx, thelist10);
                            MapSd[vtx].Append(vtx);
                        }
                    }
                    else
                    {
                        NCollection_List<TopoDS_Shape> thelist11 = new(), thelist12 = new();
                        MapSg.Bind(vtx, thelist11);
                        MapSd.Bind(vtx, thelist12);
                        MapSg[vtx].Append(vtx2b);
                        MapSd[vtx].Append(vtx2b);
                    }
                }
            }
        }

        theMap.Clear();
        for (exp.Init(theW, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            if (!theMap.Add(edg))
            { // precaution sans doute inutile...
                continue;
            }
            int indedg = MapEV.FindIndex(edg);
            ref TopoDS_Shape GenFRef = ref MapEV[indedg];
            TopoDS_Face GenF = TopoDS.Face(GenFRef);
            NCollection_List<TopoDS_Shape> thelist13 = new(), thelist14 = new();
            MapSg.Bind(edg, thelist13); // genere a gauche
            MapSd.Bind(edg, thelist14); // genere a droite
            TopoDS_Shape aLocalEdge = edg.Oriented(TopAbs_FORWARD);
            TopExp.Vertices(TopoDS.Edge(aLocalEdge), out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
            TopoDS_Shape Gvf = MapEV.FindFromKey(Vf);
            TopoDS_Shape Gvl = MapEV.FindFromKey(Vl);

            // (OCCT keeps its former construction here, commented out; nouveau code)

            TopoDS_Vertex Vfd, Vld, Vfg, Vlg;
            if (Gvf.ShapeType() == TopAbs_VERTEX)
            {
                Vfg = TopoDS.Vertex(Gvf);
                Vfd = Vfg;
            }
            else
            {
                Vfg = TopoDS.Vertex(MapSg[Vf].First());
                Vfd = TopoDS.Vertex(MapSd[Vf].First());
            }
            if (Gvl.ShapeType() == TopAbs_VERTEX)
            {
                Vlg = TopoDS.Vertex(Gvl);
                Vld = Vlg;
            }
            else
            {
                Vlg = TopoDS.Vertex(MapSg[Vl].First());
                Vld = TopoDS.Vertex(MapSd[Vl].First());
            }

            TopoDS_Edge NewEdgg = NewEdge(edg, GenF, NewSg, Vfg, Vlg);
            if (NewEdgg.IsNull())
            {
                return;
            }

            TopoDS_Edge NewEdgd = NewEdge(edg, GenF, NewSd, Vfd, Vld);
            if (NewEdgg.IsNull()) // OCCT: tests NewEdgg again
            {
                return;
            }

            bool isedg = false;
            if (Gvf.ShapeType() == TopAbs_VERTEX && Gvl.ShapeType() == TopAbs_VERTEX)
            {
                // edg ou face a 2 cotes

                // Comparaison NewEdgg et NewEdgd
                Geom_Curve Cg = BRep_Tool.Curve(NewEdgg, out double fg, out double lg)!;
                Geom_Curve Cd = BRep_Tool.Curve(NewEdgd, out double fd, out double ld)!;
                double prmg = (fg + lg) / 2.0;
                double prmd = (fd + ld) / 2.0;
                Point3D pg = Cg.Value(prmg);
                Point3D pd = Cd.Value(prmd);
                double Tol = Max(BRep_Tool.Tolerance(NewEdgg), BRep_Tool.Tolerance(NewEdgg)); // OCCT: NewEdgg twice
                if (pg.SquareDistance(pd) <= Tol * Tol)
                {
                    isedg = true;
                    // raffinement pour essayer de partager l`edge de depart...
                    bool modified = true;
                    if (Gvf.IsSame(Vf) && Gvl.IsSame(Vl))
                    {
                        // Comparaison avec l`edge de depart
                        Cd = BRep_Tool.Curve(edg, out fd, out ld)!;
                        prmd = (fd + ld) / 2.0;
                        pd = Cd.Value(prmd);
                        Tol = Max(BRep_Tool.Tolerance(NewEdgg), BRep_Tool.Tolerance(edg));
                        if (pg.SquareDistance(pd) <= Tol * Tol)
                        {
                            modified = false;
                        }
                    }

                    if (!modified)
                    {
                        MapW[edg] = edg;
                        MapSg[edg].Append(edg);
                        MapSd[edg].Append(edg);
                    }
                    else
                    {
                        MapW[edg] = NewEdgg;
                        MapSg[edg].Append(NewEdgg);
                        MapSd[edg].Append(NewEdgg);
                    }
                }
            }

            if (!isedg)
            {
                // face a 2 ou 3 ou 4 cotes
                MapSg[edg].Append(NewEdgg);
                MapSd[edg].Append(NewEdgd);

                NCollection_List<TopoDS_Shape> theedges = new();
                theedges.Append(NewEdgg);
                theedges.Append(NewEdgd);
                if (Gvf.ShapeType() == TopAbs_EDGE)
                {
                    theedges.Append(Gvf);
                }
                if (Gvl.ShapeType() == TopAbs_EDGE)
                {
                    theedges.Append(Gvl);
                }
                MakeFace(ref GenF, theedges);
                GenFRef = GenF; // OCCT changes the face held in MapEV (its orientation) through a reference
                MapW[edg] = GenF;
            }
        }

        NCollection_Map<TopoDS_Shape> mapedgadded = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> thefaces = new();

        for (itl.Initialize(myMap[aF]); itl.More(); itl.Next())
        {
            TopoDS_Face fac = TopoDS.Face(itl.Value());
            theMap.Clear();
            TopoDS_Face DrftFace = new(); // elle est FORWARD
            bool IsLeft;
            if (Contains(theLeft, fac))
            {
                B.MakeFace(ref DrftFace, NewSg, BRep_Tool.Tolerance(fac));
                IsLeft = true;
            }
            else
            {
                B.MakeFace(ref DrftFace, NewSd, BRep_Tool.Tolerance(fac));
                IsLeft = false;
            }

            TopExp_Explorer exp3 = new();
            for (exp3.Init(fac.Oriented(TopAbs_FORWARD), TopAbs_WIRE); exp3.More(); exp3.Next())
            {
                TopoDS_Shape wir = exp3.Current();
                TopoDS_Wire NewWireOnF = new();
                B.MakeWire(ref NewWireOnF);
                for (exp.Init(wir.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
                {
                    TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                    if (!theMap.Add(edg))
                    { // precaution sans doute inutile...
                        continue;
                    }
                    if (MapW.IsBound(edg))
                    { // edge du wire d`origine
                        NCollection_List<TopoDS_Shape>.Iterator itld = new();
                        TopAbs_Orientation ored = edg.Orientation();
                        if (IsLeft)
                        {
                            itld.Initialize(MapSg[edg]);
                        }
                        else
                        {
                            itld.Initialize(MapSd[edg]);
                        }
                        for (; itld.More(); itld.Next())
                        {
                            if (itld.Value().Orientation() == TopAbs_REVERSED)
                            {
                                ored = TopAbs.Reverse(ored);
                            }
                            TopoDS_Shape aLocalEdge = itld.Value().Oriented(ored);
                            B.Add(NewWireOnF, TopoDS.Edge(aLocalEdge));
                        }
                    }
                    else
                    {
                        Geom_Surface NewS;
                        if (IsLeft)
                        {
                            NewS = NewSg;
                        }
                        else
                        {
                            NewS = NewSd;
                        }
                        int indedg = MapEV.FindIndex(edg);
                        TopoDS_Face GenF = TopoDS.Face(MapEV[indedg]);
                        TopoDS_Shape aLocalEdge = edg.Oriented(TopAbs_FORWARD);
                        TopExp.Vertices(TopoDS.Edge(aLocalEdge), out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
                        TopoDS_Shape Gvf = MapEV.FindFromKey(Vf);
                        TopoDS_Shape Gvl = MapEV.FindFromKey(Vl);
                        if (Gvf.ShapeType() == TopAbs_VERTEX && Gvl.ShapeType() == TopAbs_VERTEX)
                        {
                            if (!Gvf.IsSame(Vf) || !Gvl.IsSame(Vl))
                            {
                                TopoDS_Edge NewEdg = NewEdge(edg, GenF, NewS, TopoDS.Vertex(Gvf), TopoDS.Vertex(Gvl));
                                if (NewEdg.IsNull())
                                {
                                    return;
                                }

                                MapSonS.Bind(edg, NewEdg);

                                if (NewEdg.Orientation() == TopAbs_REVERSED)
                                {
                                    NewEdg.Orientation(TopAbs.Reverse(edg.Orientation()));
                                }
                                else
                                {
                                    NewEdg.Orientation(edg.Orientation());
                                }
                                B.Add(NewWireOnF, NewEdg);
                            }
                            else
                            { // Frozen???
                                B.Add(NewWireOnF, edg);
                            }
                        }
                        else
                        {
                            TopoDS_Vertex Vff, Vll;
                            if (Gvf.ShapeType() == TopAbs_VERTEX)
                            {
                                Vff = TopoDS.Vertex(Gvf);
                            }
                            else
                            {
                                if (IsLeft)
                                {
                                    Vff = TopoDS.Vertex(MapSg[Vf].First());
                                }
                                else
                                {
                                    Vff = TopoDS.Vertex(MapSd[Vf].First());
                                }
                            }
                            if (Gvl.ShapeType() == TopAbs_VERTEX)
                            {
                                Vll = TopoDS.Vertex(Gvl);
                            }
                            else
                            {
                                if (IsLeft)
                                {
                                    Vll = TopoDS.Vertex(MapSg[Vl].First());
                                }
                                else
                                {
                                    Vll = TopoDS.Vertex(MapSd[Vl].First());
                                }
                            }

                            TopoDS_Edge NewEdg = NewEdge(edg, GenF, NewS, Vff, Vll);
                            if (NewEdg.IsNull())
                            {
                                return;
                            }

                            if (!MapW.IsBound(Vf) && !MapW.IsBound(Vl))
                            {
                                MapSonS.Bind(edg, NewEdg);
                            }
                            else
                            {
                                if (MapW.IsBound(Vf))
                                {
                                    if (Gvf.ShapeType() != TopAbs_EDGE || mapedgadded.Contains(Gvf))
                                    {
                                        MapSonS.Bind(edg, NewEdg);
                                    }
                                    else
                                    {
                                        TopoDS_Wire NewWir = new();
                                        B.MakeWire(ref NewWir);
                                        B.Add(NewWir, NewEdg);

                                        TopExp.Vertices(TopoDS.Edge(Gvf), out TopoDS_Vertex Vf2, out TopoDS_Vertex Vl2);

                                        // ici bug orientation : voir tspdrft6

                                        if (Vl2.IsSame(Vff))
                                        {
                                            B.Add(NewWir, Gvf.Oriented(TopAbs_FORWARD));
                                        }
                                        else
                                        {
                                            B.Add(NewWir, Gvf.Oriented(TopAbs_REVERSED));
                                        }
                                        mapedgadded.Add(Gvf);
                                        MapSonS.Bind(edg, NewWir); // NewWire est FORWARD
                                        _ = Vf2;
                                    }
                                }
                                else
                                {
                                    if (Gvl.ShapeType() != TopAbs_EDGE || mapedgadded.Contains(Gvl))
                                    {
                                        MapSonS.Bind(edg, NewEdg);
                                    }
                                    else
                                    {
                                        TopoDS_Wire NewWir = new();
                                        B.MakeWire(ref NewWir);
                                        B.Add(NewWir, NewEdg);

                                        TopExp.Vertices(TopoDS.Edge(Gvl), out TopoDS_Vertex Vf2, out TopoDS_Vertex Vl2);

                                        // ici bug orientation : voir tspdrft6

                                        if (Vf2.IsSame(Vll))
                                        {
                                            B.Add(NewWir, Gvl.Oriented(TopAbs_FORWARD));
                                        }
                                        else
                                        {
                                            B.Add(NewWir, Gvl.Oriented(TopAbs_REVERSED));
                                        }
                                        mapedgadded.Add(Gvl);
                                        MapSonS.Bind(edg, NewWir); // NewWire est FORWARD
                                        _ = Vl2;
                                    }
                                }
                            }
                            if (NewEdg.Orientation() == TopAbs_REVERSED)
                            {
                                NewEdg.Orientation(TopAbs.Reverse(edg.Orientation()));
                            }
                            else
                            {
                                NewEdg.Orientation(edg.Orientation());
                            }
                            B.Add(NewWireOnF, NewEdg);
                        }
                    }
                }
                B.Add(DrftFace, NewWireOnF.Oriented(wir.Orientation()));
            }
            thefaces.Append(DrftFace);
        }

        BRepTools_Substitution theSubs = new();
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> itdmss in MapSonS)
        {
            NCollection_List<TopoDS_Shape> lsubs = new();
            for (exp.Init(itdmss.Value, TopAbs_EDGE); exp.More(); exp.Next())
            {
                lsubs.Append(exp.Current());
            }
            theSubs.Substitute(itdmss.Key, lsubs);
        }

        // on reconstruit les faces
        for (exp.Init(Res, TopAbs_FACE); exp.More(); exp.Next())
        {
            if (Contains(myMap[aF], exp.Current()))
            {
                continue;
            }
            theSubs.Build(exp.Current());
        }

        // Stockage des descendants
        foreach (TopoDS_Shape aKey in myMap.Keys.ToList())
        {
            NCollection_List<TopoDS_Shape> aValue = myMap[aKey];
            if (aKey.ShapeType() == TopAbs_EDGE)
            {
                NCollection_List<TopoDS_Shape> thedesc = new();
                theMap.Clear();
                for (itl.Initialize(aValue); itl.More(); itl.Next())
                {
                    if (theMap.Add(MapW[itl.Value()]))
                    {
                        thedesc.Append(MapW[itl.Value()]);
                    }
                }
                myMap[aKey] = thedesc;
            }
            else if (aKey.IsSame(aF))
            {
                myMap[aF].Clear();
                for (itl.Initialize(thefaces); itl.More(); itl.Next())
                {
                    myMap[aF].Append(itl.Value());
                }
            }
            else
            {
                NCollection_List<TopoDS_Shape> thedesc = new();
                theMap.Clear();
                for (itl.Initialize(aValue); itl.More(); itl.Next())
                {
                    if (theSubs.IsCopied(itl.Value()))
                    {
                        if (theSubs.InternalCopy(itl.Value()).Extent() != 1)
                        {
                            // OCCT_DEBUG trace left out ("Invalid number of descendant")
                            return;
                        }
                        else
                        {
                            if (theMap.Add(theSubs.InternalCopy(itl.Value()).First()))
                            {
                                thedesc.Append(theSubs.InternalCopy(itl.Value()).First());
                            }
                        }
                    }
                    else if (theMap.Add(itl.Value()))
                    {
                        thedesc.Append(itl.Value());
                    }
                }
                myMap[aKey] = thedesc;
            }
        }

        theMap.Clear();
        thefaces.Clear();
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itdmsls in myMap)
        {
            for (itl.Initialize(itdmsls.Value); itl.More(); itl.Next())
            {
                if (itl.Value().ShapeType() == TopAbs_FACE && theMap.Add(itl.Value()))
                {
                    thefaces.Append(itl.Value());
                }
            }
        }
        LocOpe_BuildShape BS = new(thefaces);
        myResult = BS.InternalShape();

        // OCCT.NET: the copies left unchanged come back as the caller's sub-shapes
        if (myWork.IsCopied())
        {
            var aShapes = new List<TopoDS_Shape> { myResult };
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myMap)
            {
                aShapes.AddRange(anItem.Value);
            }
            myWork.Finish(aShapes);
            myResult = myWork.Final(myResult);
            foreach (TopoDS_Shape aKey in myMap.Keys.ToList())
            {
                myMap[aKey] = myWork.Final(myMap[aKey]);
            }
        }
    }

    /// <summary>Returns true if the modification has been successfully performed.</summary>
    public bool IsDone() => !myResult.IsNull();

    /// <summary>The shape given (the caller's).</summary>
    public TopoDS_Shape OriginalShape() => myShape;

    /// <summary>Returns the modified shape (frozen).</summary>
    public TopoDS_Shape Shape() => LocOpe_WorkingShapes.Published(InternalShape());

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalShape()
    {
        if (myResult.IsNull())
        {
            throw new NotDoneException();
        }
        return myResult;
    }

    /// <summary>The descendants of <paramref name="S"/>, a face of the shape or an edge of the wire (frozen).</summary>
    public NCollection_List<TopoDS_Shape> ShapesFromShape(in TopoDS_Shape S) => LocOpe_WorkingShapes.Published(InternalShapesFromShape(S));

    /// <summary><see cref="ShapesFromShape"/> without freezing, for ported code (of the caller's shape or its working copy).</summary>
    internal NCollection_List<TopoDS_Shape> InternalShapesFromShape(in TopoDS_Shape S)
    {
        if (myResult.IsNull())
        {
            throw new NotDoneException();
        }
        return myMap[myWork.Copied(S)];
    }

    private static bool NewPlane(in TopoDS_Face F, in Direction3D Extr, in Plane3D Neutr, double Ang, out Plane3D Newpl, out Axis3D NormalF,
                                 bool Modify)
    {
        Newpl = new Plane3D();
        NormalF = new Axis3D();

        // Determination du nouveau plan incline
        Geom_Surface S = BRep_Tool.Surface(F)!;
        if (S.GetType() == typeof(Geom_RectangularTrimmedSurface))
        {
            S = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
        }
        if (S is not Geom_Plane P)
        {
            return false;
        }

        Plane3D Plorig = P.Pln();
        if (!Modify)
        {
            Newpl = Plorig;
            NormalF = Newpl.Axis();
            if ((Newpl.Direct() && F.Orientation() == TopAbs_REVERSED) || (!Newpl.Direct() && F.Orientation() == TopAbs_FORWARD))
            {
                NormalF.Reverse();
            }
            return true;
        }

        Axis3D Axe;
        double Theta;

        IntAna_QuadQuadGeo i2pl = new(Plorig, Neutr, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());

        if (i2pl.IsDone() && i2pl.TypeInter() == IntAna_Line)
        {
            Line3D LinInters = i2pl.Line(1);
            Direction3D nx = LinInters.Direction();
            NormalF = Plorig.Axis();
            Direction3D ny = NormalF.Direction().Crossed(nx);
            double a = Extr.Dot(nx);
            if (Math.Abs(a) <= 1 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                double b = Extr.Dot(ny);
                double c = Extr.Dot(NormalF.Direction());
                bool direct = Plorig.Direct();
                TopAbs_Orientation Oris = F.Orientation();
                if ((direct && Oris == TopAbs_REVERSED) || (!direct && Oris == TopAbs_FORWARD))
                {
                    b = -b;
                    c = -c;
                    NormalF.Reverse();
                }
                double denom = Math.Sqrt(1 - a * a);
                double Sina = Math.Sin(Ang);
                if (denom > Math.Abs(Sina))
                {
                    double phi = Math.Atan2(b / denom, c / denom);
                    double theta0 = Math.Acos(Sina / denom);
                    Theta = theta0 - phi;
                    if (Math.Cos(Theta) < 0.0)
                    {
                        Theta = -theta0 - phi;
                    }
                    Axe = LinInters.Position();
                    Newpl = Plorig.Rotated(Axe, Theta);
                    return true;
                }
            }
        }
        // OCCT writes "fin newplane return standard_false" to std::cout (left out)
        return false;
    }

    private static void MakeFace(ref TopoDS_Face F, NCollection_List<TopoDS_Shape> ledg)
    {
        // ledg est une liste d'edge

        BRep_Builder B = new();

        // Verification de l`existence des p-curves. Celles qui manquent
        // correspondent necessairement a des isos (et meme des iso u).

        double f, l;
        NCollection_List<TopoDS_Shape>.Iterator itl = new(ledg);
        for (; itl.More(); itl.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(itl.Value());
            Geom2d_Curve? C2d = BRep_Tool.CurveOnSurface(edg, F, out f, out l);
            if (C2d is null)
            {
                BRep_Tool.Range(edg, out f, out l);
                TopExp.Vertices(edg, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                NCollection_List<TopoDS_Shape>.Iterator itl2 = new();
                for (itl2.Initialize(ledg); itl2.More(); itl2.Next())
                {
                    TopoDS_Edge edg2 = TopoDS.Edge(itl2.Value());
                    if (edg2.IsSame(edg))
                    {
                        continue;
                    }
                    TopExp.Vertices(edg2, out TopoDS_Vertex Vp1, out TopoDS_Vertex Vp2);
                    if (Vp1.IsSame(V1) || Vp2.IsSame(V1) || Vp1.IsSame(V2) || Vp2.IsSame(V2))
                    {
                        Geom2d_Curve? C22d = BRep_Tool.CurveOnSurface(edg2, F, out double f2, out double l2);
                        if (C22d is not null)
                        {
                            Point2D pt2d = new();
                            if (Vp1.IsSame(V1))
                            {
                                pt2d = C22d.Value(f2);
                                pt2d.SetY(pt2d.Y() - f);
                            }
                            else if (Vp2.IsSame(V1))
                            {
                                pt2d = C22d.Value(l2);
                                pt2d.SetY(pt2d.Y() - f);
                            }
                            else if (Vp1.IsSame(V2))
                            {
                                pt2d = C22d.Value(f2);
                                pt2d.SetY(pt2d.Y() - l);
                            }
                            else if (Vp2.IsSame(V2))
                            {
                                pt2d = C22d.Value(l2);
                                pt2d.SetY(pt2d.Y() - l);
                            }
                            C2d = new Geom2d_Line(pt2d, Constants.DY2d());
                            B.UpdateEdge(edg, C2d, F, BRep_Tool.Tolerance(edg));
                            break;
                        }
                    }
                }
                // OCCT writes "Ca merde violemment" to std::cout when C2d is still null (left out)
            }
        }

        NCollection_List<TopoDS_Shape> lwires = new();
        bool alldone = ledg.IsEmpty();
        while (!alldone)
        {
            TopoDS_Wire Wnew = new();
            B.MakeWire(ref Wnew);
            TopoDS_Shape aLocalShapeCur = ledg.First();
            TopoDS_Edge edg = TopoDS.Edge(aLocalShapeCur);
            TopoDS_Vertex VFirst, VLast;
            if (edg.Orientation() == TopAbs_FORWARD)
            {
                TopExp.Vertices(edg, out VFirst, out VLast);
            }
            else
            {
                TopExp.Vertices(edg, out VLast, out VFirst);
            }
            B.Add(Wnew, edg);
            ledg.RemoveFirst();
            // on suppose VFirst et VLast non nuls
            bool wdone = (ledg.IsEmpty() || VFirst.IsSame(VLast));
            while (!wdone)
            {
                TopAbs_Orientation oredg = TopAbs_FORWARD;

                for (itl.Initialize(ledg); itl.More(); itl.Next())
                {
                    TopoDS_Edge edg2 = TopoDS.Edge(itl.Value());
                    TopoDS_Shape aLocalShape = edg2.Oriented(TopAbs_FORWARD);
                    TopExp.Vertices(TopoDS.Edge(aLocalShape), out TopoDS_Vertex VF, out TopoDS_Vertex VL);
                    if (VF.IsSame(VLast))
                    {
                        VLast = VL;
                        oredg = TopAbs_FORWARD;
                        break;
                    }
                    else if (VL.IsSame(VFirst))
                    {
                        VFirst = VF;
                        oredg = TopAbs_FORWARD;
                        break;
                    }
                    else if (VF.IsSame(VFirst))
                    {
                        VFirst = VL;
                        oredg = TopAbs_REVERSED;
                        break;
                    }
                    else if (VL.IsSame(VLast))
                    {
                        VLast = VF;
                        oredg = TopAbs_REVERSED;
                        break;
                    }
                }
                if (!itl.More())
                {
                    wdone = true;
                }
                else
                {
                    TopoDS_Shape aLocalShape = itl.Value().Oriented(oredg);
                    B.Add(Wnew, TopoDS.Edge(aLocalShape));
                    ledg.Remove(itl);
                    wdone = (ledg.IsEmpty() || VFirst.IsSame(VLast));
                }
            }
            lwires.Append(Wnew);
            alldone = ledg.IsEmpty();
        }

        F.Orientation(TopAbs_FORWARD);
        for (itl.Initialize(lwires); itl.More(); itl.Next())
        {
            TopoDS_Shape aLocalShape = F.EmptyCopied();
            TopoDS_Face NewFace = TopoDS.Face(aLocalShape);
            B.Add(NewFace, itl.Value());
            GProp_GProps GP = new();
            BRepGProp.SurfaceProperties(NewFace, GP);
            if (GP.Mass() < 0)
            {
                itl.ChangeValue().Reverse();
            }
        }
        if (lwires.Extent() == 1)
        {
            B.Add(F, lwires.First());
        }
        else
        {
            // OCCT writes "Not yet implemented : nbwire >= 2" to std::cout (left out)
        }
    }

    private static bool Contains(NCollection_List<TopoDS_Shape> ll, in TopoDS_Shape s)
    {
        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        for (itl.Initialize(ll); itl.More(); itl.Next())
        {
            if (itl.Value().IsSame(s))
            {
                return true;
            }
        }
        return false;
    }

    // OCCT.NET: OCCT translates the intersection's pcurve in place; a frozen one is translated on a copy
    private static Geom2d_Curve Translatable(Geom2d_Curve theCurve) =>
        theCurve.IsFrozen ? (Geom2d_Curve)theCurve.Copy() : theCurve;

    private static TopoDS_Edge NewEdge(in TopoDS_Edge edg, in TopoDS_Face F, Geom_Surface NewS, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        TopoDS_Edge NewEdg = new();
        Geom_Surface S1 = BRep_Tool.Surface(F)!;
        bool AppS1 = false;
        if (S1.GetType() != typeof(Geom_Plane))
        {
            AppS1 = true;
        }

        GeomInt_IntSS i2s = new(S1, NewS, global::Kerf.Engine.Foundation.Precision.Confusion(), true, AppS1);
        if (!i2s.IsDone() || i2s.NbLines() <= 0)
        {
            return NewEdg;
        }

        BRep_Builder B = new();
        double Dist2, Dist2Min;
        double prmf = 0, prml = 0;
        GeomAdaptor_Curve TheCurve = new();

        int i, k;
        Point3D pvf = BRep_Tool.Pnt(V1);
        Point3D pvl = BRep_Tool.Pnt(V2);
        for (i = 1; i <= i2s.NbLines(); i++)
        {
            TheCurve.Load(i2s.Line(i));
            Extrema_ExtPC myExtPC = new(pvf, TheCurve);

            if (myExtPC.IsDone())
            {
                double thepmin = TheCurve.FirstParameter();
                myExtPC.TrimmedSquareDistances(out Dist2Min, out Dist2, out Point3D p1b, out Point3D p2b);
                if (Dist2 < Dist2Min && !TheCurve.IsPeriodic())
                {
                    Dist2Min = Dist2;
                    thepmin = TheCurve.LastParameter();
                }
                for (k = 1; k <= myExtPC.NbExt(); k++)
                {
                    Dist2 = myExtPC.SquareDistance(k);
                    if (Dist2 < Dist2Min)
                    {
                        Dist2Min = Dist2;
                        thepmin = myExtPC.Point(k).Parameter();
                    }
                }

                if (Dist2Min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    prmf = thepmin;
                    myExtPC.Perform(pvl);
                    if (myExtPC.IsDone())
                    {
                        thepmin = TheCurve.LastParameter();
                        myExtPC.TrimmedSquareDistances(out Dist2, out Dist2Min, out p1b, out p2b);
                        if (Dist2 < Dist2Min && !TheCurve.IsClosed())
                        {
                            Dist2Min = Dist2;
                            thepmin = TheCurve.FirstParameter();
                        }
                        for (k = 1; k <= myExtPC.NbExt(); k++)
                        {
                            Dist2 = myExtPC.SquareDistance(k);
                            if (Dist2 < Dist2Min)
                            {
                                Dist2Min = Dist2;
                                thepmin = myExtPC.Point(k).Parameter();
                            }
                        }

                        if (Dist2Min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                        {
                            prml = thepmin;
                            break;
                        }
                    }
                }
            }
        }

        if (i <= i2s.NbLines())
        {
            bool rev = false;
            TopoDS_Vertex Vf = V1;
            TopoDS_Vertex Vl = V2;
            Geom_Curve Cimg = i2s.Line(i);
            Geom2d_Curve? Cimg2d = null;
            if (AppS1)
            {
                Cimg2d = i2s.LineOnS1(i);
            }

            if (Cimg.IsPeriodic())
            {
                double period = Cimg.Period();
                double imf = Cimg.FirstParameter();
                double iml = Cimg.LastParameter();

                BRep_Tool.Range(edg, out double f, out double l);
                double delt = l - f;
                double delt1 = Math.Abs(prml - prmf);
                double delt2 = Math.Abs(period - delt1);

                if (delt1 == 0 || delt2 == 0)
                {
                    prmf = imf;
                    prml = iml;
                }
                else
                {
                    if (Math.Abs(delt1 - delt) > Math.Abs(delt2 - delt))
                    {
                        // le bon ecart est delt2...
                        if (prml > prmf)
                        {
                            if (prml < iml)
                            {
                                prmf += period;
                            }
                            else
                            {
                                prml -= period;
                            }
                        }
                        else
                        {
                            if (prmf < iml)
                            {
                                prml += period;
                            }
                            else
                            {
                                prmf -= period;
                            }
                        }
                    }
                    else if (Math.Abs(delt1 - delt) < Math.Abs(delt2 - delt))
                    {
                        if (prmf >= iml && prml >= iml)
                        {
                            prmf -= period;
                            prml -= period;
                        }
                        else if (prmf <= imf && prml <= imf)
                        {
                            prmf += period;
                            prml += period;
                        }
                    }
                    else
                    { // egalite; on priveligie l'ordre f,l
                        if (prmf > prml)
                        {
                            prmf -= period;
                        }
                        if (prmf >= iml && prml >= iml)
                        {
                            prmf -= period;
                            prml -= period;
                        }
                        else if (prmf <= imf && prml <= imf)
                        {
                            prmf += period;
                            prml += period;
                        }
                    }
                }
                // OCCT_DEBUG check left out ("Ca ne va pas aller")
            }

            if (S1.IsUPeriodic())
            {
                double speriod = S1.UPeriod();
                Point2D pf, pl;
                pf = Cimg2d!.Value(prmf);
                pl = Cimg2d.Value(prml);

                double Uf = pf.X();
                double Ul = pl.X();
                double ptra = 0.0;

                double Ustart = Min(Uf, Ul);
                while (Ustart < -global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    Ustart += speriod;
                    ptra += speriod;
                }
                while (Ustart > speriod - global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    Ustart -= speriod;
                    ptra -= speriod;
                }
                if (ptra != 0)
                {
                    Cimg2d = Translatable(Cimg2d);
                    Cimg2d.Translate(new Vector2D(ptra, 0.0));
                }
            }
            if (prmf < prml)
            {
                Vf.Orientation(TopAbs_FORWARD);
                Vl.Orientation(TopAbs_REVERSED);
            }
            else
            {
                Vf.Orientation(TopAbs_REVERSED);
                Vl.Orientation(TopAbs_FORWARD);
                rev = true;
            }

            B.MakeEdge(ref NewEdg, Cimg, global::Kerf.Engine.Foundation.Precision.Confusion());

            B.Add(NewEdg, Vf);
            B.Add(NewEdg, Vl);
            B.UpdateVertex(Vf, prmf, NewEdg, global::Kerf.Engine.Foundation.Precision.Confusion());
            B.UpdateVertex(Vl, prml, NewEdg, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (AppS1)
            {
                B.UpdateEdge(NewEdg, Cimg2d, F, global::Kerf.Engine.Foundation.Precision.Confusion());
            }

            if (rev)
            {
                NewEdg.Orientation(TopAbs_REVERSED);
            }
        }
        return NewEdg;
    }
}
