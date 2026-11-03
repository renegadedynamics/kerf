// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitDrafts.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitDrafts.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Intersection.ResultType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private Shape myShape = new();
    private Shape myResult = new();
    // OCCT iterates this map keyed by the shapes' addresses to store the descendants; each entry is handled alone,
    // apart from the last pass collecting the faces of the result (whose order follows the map's: see the remarks of
    // LocOpe_BuildShape)
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myMap = new(ShapeHasher.Instance);
    private LocOpe_WorkingShapes myWork = new();

    /// <summary>Empty constructor.</summary>
    public LocOpe_SplitDrafts()
    {
    }

    /// <summary>Creates the algorithm on the shape <paramref name="S"/>.</summary>
    public LocOpe_SplitDrafts(in Shape S)
    {
        myShape = S;
    }

    /// <summary>Initializes the algorithm with the shape <paramref name="S"/>.</summary>
    public void Init(in Shape S)
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
    public void Perform(in Face F, in Wire W, in Direction3D Extr, in Plane3D NPl, double Angle)
    {
        Perform(F, W, Extr, NPl, Angle, Extr, NPl, Angle, true, false);
    }

    /// <summary>
    /// Splits the face <paramref name="F"/> of the shape with the wire <paramref name="W"/>, lying on it, and puts a
    /// draft angle on both parts: <paramref name="Extrg"/>, <paramref name="NPlg"/>, <paramref name="Angleg"/> for the
    /// left part, <paramref name="Extrd"/>, <paramref name="NPld"/>, <paramref name="Angled"/> for the right part, each
    /// part left as it is when its Modify flag is false.
    /// </summary>
    public void Perform(in Face F,
                        in Wire W,
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
        if (myShape.IsNull || F.IsNull || W.IsNull)
        {
            throw new NullObjectException();
        }

        if (!ModLeft && !ModRight)
        {
            throw new ConstructionErrorException();
        }

        // OCCT.NET: the shape, the face and the wire as worked on (their copies if frozen, decision 1013)
        myWork = new LocOpe_WorkingShapes();
        Shape aShape = myWork.Working(myShape);
        Face aF = ShapeCasts.Face(myWork.Working(F));
        Wire aW = ShapeCasts.Wire(myWork.Working(W));

        Orientation OriF = Forward;

        bool FinS = false;
        Explorer exp = new(), exp2 = new();
        for (exp.Init(aShape, ShapeType.Face); exp.More(); exp.Next())
        {
            Shape fac = exp.Current();
            NCollection_List<Shape> thelist = new();
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

        Shape aLocalFace = aF.Oriented(OriF);

        if (!NewPlane(ShapeCasts.Face(aLocalFace), Extrg, NPlg, Angleg, out Plane3D NewPlg, out Axis3D NormalFg, ModLeft)
            || !NewPlane(ShapeCasts.Face(aLocalFace), Extrd, NPld, Angled, out Plane3D NewPld, out Axis3D NormalFd, ModRight))
        {
            return;
        }
        _ = NormalFd;

        NCollection_List<Shape>.Iterator itl = new();
        BrepBuilder B = new();

        Surface NewSg = new PlaneSurface(NewPlg);
        Surface NewSd = new PlaneSurface(NewPld);
        LineCurve3D theLinePipe = new(NormalFg); // ou NormalFd
        GeomInt_IntSS i2s = new(NewSg, NewSd, global::Kerf.Engine.Foundation.Precision.Confusion());

        NCollection_Map<Shape> theMap = new(ShapeHasher.Instance);
        CurveAdaptor HAC = new();
        SurfaceAdaptor HAS = new();
        IntCurveSurface_HInter intcs = new();

        Wire theW = aW;
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

            Surface Spl = thePipe.InternalSurface()!;
            HAS.Load(Spl);

            LocOpe_SplitShape splw = new(aW, myWork);

            for (exp.Init(aW, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                if (theMap.Add(edg))
                {
                    Curve3D? C = Tool.Curve(edg, out double f, out double l);
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
                        Exploration.Vertices(edg, out Vertex Vf, out Vertex Vl);
                        Point3D Pf = Tool.Pnt(Vf);
                        Point3D Pl = Tool.Pnt(Vl);
                        double Tolf = Tool.Tolerance(Vf);
                        double Toll = Tool.Tolerance(Vl);
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
                            Vertex Vnewf = new(), Vnewl = new();
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
                        Exploration.Vertices(edg, out Vertex Vf, out Vertex Vl);
                        Point3D Pf = Tool.Pnt(Vf);
                        Point3D Pl = Tool.Pnt(Vl);
                        double Tolf = Tool.Tolerance(Vf);
                        double Toll = Tool.Tolerance(Vl);
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
                                    Vertex Vnew = new();
                                    B.MakeVertex(ref Vnew, pi, global::Kerf.Engine.Foundation.Precision.Confusion());
                                    splw.InternalAdd(Vnew, Pi.W(), edg);
                                }
                            }
                        }
                    }
                }
            }

            NCollection_List<Shape> lres = splw.InternalDescendantShapes(aW);
            if (lres.Extent() != 1)
            {
                return;
            }

            if (!aW.IsSame(lres.First()))
            {
                theW.Nullify();
                theW = ShapeCasts.Wire(lres.First());
            }

            for (exp.ReInit(); exp.More(); exp.Next())
            {
                if (!myMap.IsBound(exp.Current()))
                {
                    NCollection_List<Shape> thelist1 = new();
                    myMap.Bind(exp.Current(), thelist1);
                    for (itl.Initialize(splw.InternalDescendantShapes(exp.Current())); itl.More(); itl.Next())
                    {
                        myMap[exp.Current()].Append(itl.Value());
                    }
                    for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        if (!myMap.IsBound(exp2.Current()))
                        {
                            NCollection_List<Shape> thelist2 = new();
                            myMap.Bind(exp2.Current(), thelist2);
                            myMap[exp2.Current()].Append(exp2.Current());
                        }
                    }
                }
            }
        }
        else
        {
            for (exp.Init(aW, ShapeType.Edge); exp.More(); exp.Next())
            {
                if (!myMap.IsBound(exp.Current()))
                {
                    NCollection_List<Shape> thelist3 = new();
                    myMap.Bind(exp.Current(), thelist3);
                    myMap[exp.Current()].Append(exp.Current());
                    for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        if (!myMap.IsBound(exp2.Current()))
                        {
                            NCollection_List<Shape> thelist4 = new();
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

        Shape Res = Spls.InternalResultingShape();
        NCollection_List<Shape> theLeft = Spls.InternalDirectLeft();

        // Descendants
        for (exp.Init(aShape, ShapeType.Face); exp.More(); exp.Next())
        {
            Shape fac = exp.Current();
            for (itl.Initialize(Spls.InternalDescendantShapes(fac)); itl.More(); itl.Next())
            {
                myMap[fac].Append(itl.Value());
            }
        }

        NCollection_DataMap<Shape, Shape> MapW = new(ShapeHasher.Instance);
        for (exp.Init(theW, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!MapW.IsBound(exp.Current()))
            {
                MapW.Bind(exp.Current(), new Shape());
                for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
                {
                    if (!MapW.IsBound(exp2.Current()))
                    {
                        MapW.Bind(exp2.Current(), new Shape());
                    }
                }
            }
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(Res, ShapeType.Edge, ShapeType.Face, theMapEF);

        // On stocke les geometries potentiellement generees par les edges
        NCollection_IndexedDataMap<Shape, Shape> MapEV = new(ShapeHasher.Instance); // genere
        NCollection_DataMap<Shape, NCollection_List<Shape>> MapSg = new(ShapeHasher.Instance),
                                                                          MapSd = new(ShapeHasher.Instance); // image a gauche et a droite

        int Nbedges, index;
        for (itl.Initialize(myMap[aF]); itl.More(); itl.Next())
        {
            Shape fac = ShapeCasts.Face(itl.Value());
            for (exp.Init(fac, ShapeType.Edge); exp.More(); exp.Next())
            {
                Shape edg = exp.Current();
                if (MapEV.FindIndex(edg) != 0)
                {
                    continue;
                }
                if (MapW.IsBound(edg))
                { // edge du wire initial
                    Curve3D C = Tool.Curve(ShapeCasts.Edge(edg), out Placement Loc, out _, out _)!;
                    if (C.GetType() == typeof(TrimmedCurve3D))
                    {
                        C = ((TrimmedCurve3D)C).BasisCurve();
                    }
                    C = (Curve3D)C.Transformed(Loc.Transformation());

                    GeomFill_Pipe thePipe = new();
                    thePipe.GenerateParticularCase(true);
                    thePipe.Init(theLinePipe, C);
                    thePipe.Perform(true);
                    if (!thePipe.IsDone())
                    {
                        throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
                    }

                    Surface thePS = thePipe.InternalSurface()!;
                    if (thePS.GetType() == typeof(TrimmedSurface))
                    {
                        thePS = ((TrimmedSurface)thePS).BasisSurface();
                    }

                    Face NewFace = new();
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

        NCollection_DataMap<Shape, Shape> MapSonS = new(ShapeHasher.Instance);

        Nbedges = MapEV.Extent();
        for (index = 1; index <= Nbedges; index++)
        {
            for (exp.Init(MapEV.FindKey(index), ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
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
                    for (exp2.Init(MapEV.FindKey(j), ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        Shape vtx2 = exp2.Current();
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
                Shape edg1 = MapEV.FindKey(index);
                Shape edg2;
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
                Curve3D? Newc = null;
                Curve2D? newCs1 = null, newCs2 = null;
                double knownp = 0;
                Edge Ebind = new();
                switch (Choice)
                {
                    case 1:
                    {
                        for (exp2.Init(Res, ShapeType.Edge); exp2.More(); exp2.Next())
                        {
                            if (exp2.Current().IsSame(edg1) || exp2.Current().IsSame(edg2))
                            {
                                continue;
                            }
                            Explorer exp3 = new(exp2.Current().Oriented(Forward), ShapeType.Vertex);
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
                            Newc = Tool.Curve(ShapeCasts.Edge(exp2.Current()), out Placement Loc, out _, out _)!;
                            Newc = (Curve3D)Newc.Transformed(Loc.Transformation());
                            Ebind = ShapeCasts.Edge(exp2.Current());
                            knownp = Tool.Parameter(vtx, Ebind);
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
                        theLine.Translate(NormalFg.Location(), Tool.Pnt(vtx));
                        Newc = new LineCurve3D(theLine);
                        knownp = 0.0;
                    }
                    break;
                    case 3:
                    {
                        Face F1 = ShapeCasts.Face(MapEV.FindFromKey(edg1));
                        Face F2 = ShapeCasts.Face(MapEV.FindFromKey(edg2));
                        Surface S1 = Tool.Surface(F1)!;
                        Surface S2 = Tool.Surface(F2)!;
                        bool AppS1 = false;
                        bool AppS2 = false;
                        if (S1.GetType() != typeof(PlaneSurface))
                        {
                            AppS1 = true;
                        }
                        if (S2.GetType() != typeof(PlaneSurface))
                        {
                            AppS2 = true;
                        }
                        i2s.Perform(S1, S2, global::Kerf.Engine.Foundation.Precision.Confusion(), true, AppS1, AppS2);
                        if (!i2s.IsDone() || i2s.NbLines() <= 0)
                        {
                            return;
                        }

                        double pmin = 0, Dist2 = 0, Dist2Min = 0, Glob2Min = RealLast();
                        CurveAdaptor TheCurve = new();

                        int i, imin, k;
                        Point3D pv = Tool.Pnt(vtx);
                        imin = 0;
                        for (i = 1; i <= i2s.NbLines(); i++)
                        {
                            TheCurve.Load(i2s.Line(i));
                            CurvePointExtrema myExtPC = new(pv, TheCurve);

                            if (myExtPC.IsDone)
                            {
                                double thepmin = TheCurve.FirstParameter();
                                myExtPC.TrimmedSquareDistances(out Dist2Min, out Dist2, out Point3D p1b, out Point3D p2b);
                                if (Dist2 < Dist2Min)
                                {
                                    thepmin = TheCurve.LastParameter();
                                }
                                for (k = 1; k <= myExtPC.ExtCount; k++)
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
                Vertex vtx1 = new(), vtx2b = new();
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
                    Edge NewEdge = new();
                    B.MakeEdge(ref NewEdge, Newc, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (p1 < p2)
                    {
                        B.Add(NewEdge, vtx1.Oriented(Forward));
                        B.Add(NewEdge, vtx2b.Oriented(Reversed));
                    }
                    else
                    {
                        B.Add(NewEdge, vtx1.Oriented(Reversed));
                        B.Add(NewEdge, vtx2b.Oriented(Forward));
                    }
                    B.UpdateVertex(vtx1, p1, NewEdge, global::Kerf.Engine.Foundation.Precision.Confusion());
                    B.UpdateVertex(vtx2b, p2, NewEdge, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (newCs1 is not null)
                    {
                        B.UpdateEdge(NewEdge, newCs1, ShapeCasts.Face(MapEV.FindFromKey(edg1)), global::Kerf.Engine.Foundation.Precision.Confusion());
                    }

                    if (newCs2 is not null)
                    {
                        B.UpdateEdge(NewEdge, newCs2, ShapeCasts.Face(MapEV.FindFromKey(edg2)), global::Kerf.Engine.Foundation.Precision.Confusion());
                    }

                    MapEV.Add(vtx, NewEdge);

                    if (Choice == 1)
                    {
                        Shape aLocalEdge = Ebind.EmptyCopied();
                        Edge NE = ShapeCasts.Edge(aLocalEdge);
                        for (exp2.Init(Ebind, ShapeType.Vertex); exp2.More(); exp2.Next())
                        {
                            Vertex thevtx = ShapeCasts.Vertex(exp2.Current());
                            if (thevtx.IsSame(vtx))
                            {
                                B.Add(NE, vtx1.Oriented(thevtx.Orientation()));
                                B.UpdateVertex(vtx1, p1, NE, global::Kerf.Engine.Foundation.Precision.Confusion());
                            }
                            else
                            {
                                B.Add(NE, thevtx);
                                double theprm = Tool.Parameter(thevtx, Ebind);
                                B.UpdateVertex(thevtx, theprm, NE, Tool.Tolerance(thevtx));
                            }
                        }
                        MapSonS.Bind(Ebind, NE.Oriented(Forward));
                        if (IsLeft)
                        {
                            NCollection_List<Shape> thelist5 = new();
                            MapSg.Bind(vtx, thelist5);
                            MapSg[vtx].Append(vtx1);
                        }
                        else
                        {
                            NCollection_List<Shape> thelist6 = new();
                            MapSd.Bind(vtx, thelist6);
                            MapSd[vtx].Append(vtx1);
                        }
                    }
                    else
                    {
                        NCollection_List<Shape> thelist7 = new(), thelist8 = new();
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
                            NCollection_List<Shape> thelist9 = new();
                            MapSg.Bind(vtx, thelist9);
                            MapSg[vtx].Append(vtx);
                        }
                        else
                        {
                            NCollection_List<Shape> thelist10 = new();
                            MapSd.Bind(vtx, thelist10);
                            MapSd[vtx].Append(vtx);
                        }
                    }
                    else
                    {
                        NCollection_List<Shape> thelist11 = new(), thelist12 = new();
                        MapSg.Bind(vtx, thelist11);
                        MapSd.Bind(vtx, thelist12);
                        MapSg[vtx].Append(vtx2b);
                        MapSd[vtx].Append(vtx2b);
                    }
                }
            }
        }

        theMap.Clear();
        for (exp.Init(theW, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!theMap.Add(edg))
            { // precaution sans doute inutile...
                continue;
            }
            int indedg = MapEV.FindIndex(edg);
            ref Shape GenFRef = ref MapEV[indedg];
            Face GenF = ShapeCasts.Face(GenFRef);
            NCollection_List<Shape> thelist13 = new(), thelist14 = new();
            MapSg.Bind(edg, thelist13); // genere a gauche
            MapSd.Bind(edg, thelist14); // genere a droite
            Shape aLocalEdge = edg.Oriented(Forward);
            Exploration.Vertices(ShapeCasts.Edge(aLocalEdge), out Vertex Vf, out Vertex Vl);
            Shape Gvf = MapEV.FindFromKey(Vf);
            Shape Gvl = MapEV.FindFromKey(Vl);

            // (OCCT keeps its former construction here, commented out; nouveau code)

            Vertex Vfd, Vld, Vfg, Vlg;
            if (Gvf.ShapeType() == ShapeType.Vertex)
            {
                Vfg = ShapeCasts.Vertex(Gvf);
                Vfd = Vfg;
            }
            else
            {
                Vfg = ShapeCasts.Vertex(MapSg[Vf].First());
                Vfd = ShapeCasts.Vertex(MapSd[Vf].First());
            }
            if (Gvl.ShapeType() == ShapeType.Vertex)
            {
                Vlg = ShapeCasts.Vertex(Gvl);
                Vld = Vlg;
            }
            else
            {
                Vlg = ShapeCasts.Vertex(MapSg[Vl].First());
                Vld = ShapeCasts.Vertex(MapSd[Vl].First());
            }

            Edge NewEdgg = NewEdge(edg, GenF, NewSg, Vfg, Vlg);
            if (NewEdgg.IsNull)
            {
                return;
            }

            Edge NewEdgd = NewEdge(edg, GenF, NewSd, Vfd, Vld);
            if (NewEdgg.IsNull) // OCCT: tests NewEdgg again
            {
                return;
            }

            bool isedg = false;
            if (Gvf.ShapeType() == ShapeType.Vertex && Gvl.ShapeType() == ShapeType.Vertex)
            {
                // edg ou face a 2 cotes

                // Comparaison NewEdgg et NewEdgd
                Curve3D Cg = Tool.Curve(NewEdgg, out double fg, out double lg)!;
                Curve3D Cd = Tool.Curve(NewEdgd, out double fd, out double ld)!;
                double prmg = (fg + lg) / 2.0;
                double prmd = (fd + ld) / 2.0;
                Point3D pg = Cg.Value(prmg);
                Point3D pd = Cd.Value(prmd);
                double Tol = Max(Tool.Tolerance(NewEdgg), Tool.Tolerance(NewEdgg)); // OCCT: NewEdgg twice
                if (pg.SquareDistance(pd) <= Tol * Tol)
                {
                    isedg = true;
                    // raffinement pour essayer de partager l`edge de depart...
                    bool modified = true;
                    if (Gvf.IsSame(Vf) && Gvl.IsSame(Vl))
                    {
                        // Comparaison avec l`edge de depart
                        Cd = Tool.Curve(edg, out fd, out ld)!;
                        prmd = (fd + ld) / 2.0;
                        pd = Cd.Value(prmd);
                        Tol = Max(Tool.Tolerance(NewEdgg), Tool.Tolerance(edg));
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

                NCollection_List<Shape> theedges = new();
                theedges.Append(NewEdgg);
                theedges.Append(NewEdgd);
                if (Gvf.ShapeType() == ShapeType.Edge)
                {
                    theedges.Append(Gvf);
                }
                if (Gvl.ShapeType() == ShapeType.Edge)
                {
                    theedges.Append(Gvl);
                }
                MakeFace(ref GenF, theedges);
                GenFRef = GenF; // OCCT changes the face held in MapEV (its orientation) through a reference
                MapW[edg] = GenF;
            }
        }

        NCollection_Map<Shape> mapedgadded = new(ShapeHasher.Instance);
        NCollection_List<Shape> thefaces = new();

        for (itl.Initialize(myMap[aF]); itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());
            theMap.Clear();
            Face DrftFace = new(); // elle est FORWARD
            bool IsLeft;
            if (Contains(theLeft, fac))
            {
                B.MakeFace(ref DrftFace, NewSg, Tool.Tolerance(fac));
                IsLeft = true;
            }
            else
            {
                B.MakeFace(ref DrftFace, NewSd, Tool.Tolerance(fac));
                IsLeft = false;
            }

            Explorer exp3 = new();
            for (exp3.Init(fac.Oriented(Forward), ShapeType.Wire); exp3.More(); exp3.Next())
            {
                Shape wir = exp3.Current();
                Wire NewWireOnF = new();
                B.MakeWire(ref NewWireOnF);
                for (exp.Init(wir.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge edg = ShapeCasts.Edge(exp.Current());
                    if (!theMap.Add(edg))
                    { // precaution sans doute inutile...
                        continue;
                    }
                    if (MapW.IsBound(edg))
                    { // edge du wire d`origine
                        NCollection_List<Shape>.Iterator itld = new();
                        Orientation ored = edg.Orientation();
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
                            if (itld.Value().Orientation() == Reversed)
                            {
                                ored = global::Kerf.Engine.Topology.TopAbs.Reverse(ored);
                            }
                            Shape aLocalEdge = itld.Value().Oriented(ored);
                            B.Add(NewWireOnF, ShapeCasts.Edge(aLocalEdge));
                        }
                    }
                    else
                    {
                        Surface NewS;
                        if (IsLeft)
                        {
                            NewS = NewSg;
                        }
                        else
                        {
                            NewS = NewSd;
                        }
                        int indedg = MapEV.FindIndex(edg);
                        Face GenF = ShapeCasts.Face(MapEV[indedg]);
                        Shape aLocalEdge = edg.Oriented(Forward);
                        Exploration.Vertices(ShapeCasts.Edge(aLocalEdge), out Vertex Vf, out Vertex Vl);
                        Shape Gvf = MapEV.FindFromKey(Vf);
                        Shape Gvl = MapEV.FindFromKey(Vl);
                        if (Gvf.ShapeType() == ShapeType.Vertex && Gvl.ShapeType() == ShapeType.Vertex)
                        {
                            if (!Gvf.IsSame(Vf) || !Gvl.IsSame(Vl))
                            {
                                Edge NewEdg = NewEdge(edg, GenF, NewS, ShapeCasts.Vertex(Gvf), ShapeCasts.Vertex(Gvl));
                                if (NewEdg.IsNull)
                                {
                                    return;
                                }

                                MapSonS.Bind(edg, NewEdg);

                                if (NewEdg.Orientation() == Reversed)
                                {
                                    NewEdg.Orientation(global::Kerf.Engine.Topology.TopAbs.Reverse(edg.Orientation()));
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
                            Vertex Vff, Vll;
                            if (Gvf.ShapeType() == ShapeType.Vertex)
                            {
                                Vff = ShapeCasts.Vertex(Gvf);
                            }
                            else
                            {
                                if (IsLeft)
                                {
                                    Vff = ShapeCasts.Vertex(MapSg[Vf].First());
                                }
                                else
                                {
                                    Vff = ShapeCasts.Vertex(MapSd[Vf].First());
                                }
                            }
                            if (Gvl.ShapeType() == ShapeType.Vertex)
                            {
                                Vll = ShapeCasts.Vertex(Gvl);
                            }
                            else
                            {
                                if (IsLeft)
                                {
                                    Vll = ShapeCasts.Vertex(MapSg[Vl].First());
                                }
                                else
                                {
                                    Vll = ShapeCasts.Vertex(MapSd[Vl].First());
                                }
                            }

                            Edge NewEdg = NewEdge(edg, GenF, NewS, Vff, Vll);
                            if (NewEdg.IsNull)
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
                                    if (Gvf.ShapeType() != ShapeType.Edge || mapedgadded.Contains(Gvf))
                                    {
                                        MapSonS.Bind(edg, NewEdg);
                                    }
                                    else
                                    {
                                        Wire NewWir = new();
                                        B.MakeWire(ref NewWir);
                                        B.Add(NewWir, NewEdg);

                                        Exploration.Vertices(ShapeCasts.Edge(Gvf), out Vertex Vf2, out Vertex Vl2);

                                        // ici bug orientation : voir tspdrft6

                                        if (Vl2.IsSame(Vff))
                                        {
                                            B.Add(NewWir, Gvf.Oriented(Forward));
                                        }
                                        else
                                        {
                                            B.Add(NewWir, Gvf.Oriented(Reversed));
                                        }
                                        mapedgadded.Add(Gvf);
                                        MapSonS.Bind(edg, NewWir); // NewWire est FORWARD
                                        _ = Vf2;
                                    }
                                }
                                else
                                {
                                    if (Gvl.ShapeType() != ShapeType.Edge || mapedgadded.Contains(Gvl))
                                    {
                                        MapSonS.Bind(edg, NewEdg);
                                    }
                                    else
                                    {
                                        Wire NewWir = new();
                                        B.MakeWire(ref NewWir);
                                        B.Add(NewWir, NewEdg);

                                        Exploration.Vertices(ShapeCasts.Edge(Gvl), out Vertex Vf2, out Vertex Vl2);

                                        // ici bug orientation : voir tspdrft6

                                        if (Vf2.IsSame(Vll))
                                        {
                                            B.Add(NewWir, Gvl.Oriented(Forward));
                                        }
                                        else
                                        {
                                            B.Add(NewWir, Gvl.Oriented(Reversed));
                                        }
                                        mapedgadded.Add(Gvl);
                                        MapSonS.Bind(edg, NewWir); // NewWire est FORWARD
                                        _ = Vl2;
                                    }
                                }
                            }
                            if (NewEdg.Orientation() == Reversed)
                            {
                                NewEdg.Orientation(global::Kerf.Engine.Topology.TopAbs.Reverse(edg.Orientation()));
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

        Substitution theSubs = new();
        foreach (KeyValuePair<Shape, Shape> itdmss in MapSonS)
        {
            NCollection_List<Shape> lsubs = new();
            for (exp.Init(itdmss.Value, ShapeType.Edge); exp.More(); exp.Next())
            {
                lsubs.Append(exp.Current());
            }
            theSubs.Substitute(itdmss.Key, lsubs);
        }

        // on reconstruit les faces
        for (exp.Init(Res, ShapeType.Face); exp.More(); exp.Next())
        {
            if (Contains(myMap[aF], exp.Current()))
            {
                continue;
            }
            theSubs.Build(exp.Current());
        }

        // Stockage des descendants
        foreach (Shape aKey in myMap.Keys.ToList())
        {
            NCollection_List<Shape> aValue = myMap[aKey];
            if (aKey.ShapeType() == ShapeType.Edge)
            {
                NCollection_List<Shape> thedesc = new();
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
                NCollection_List<Shape> thedesc = new();
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
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdmsls in myMap)
        {
            for (itl.Initialize(itdmsls.Value); itl.More(); itl.Next())
            {
                if (itl.Value().ShapeType() == ShapeType.Face && theMap.Add(itl.Value()))
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
            var aShapes = new List<Shape> { myResult };
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> anItem in myMap)
            {
                aShapes.AddRange(anItem.Value);
            }
            myWork.Finish(aShapes);
            myResult = myWork.Final(myResult);
            foreach (Shape aKey in myMap.Keys.ToList())
            {
                myMap[aKey] = myWork.Final(myMap[aKey]);
            }
        }
    }

    /// <summary>Returns true if the modification has been successfully performed.</summary>
    public bool IsDone() => !myResult.IsNull;

    /// <summary>The shape given (the caller's).</summary>
    public Shape OriginalShape() => myShape;

    /// <summary>Returns the modified shape (frozen).</summary>
    public Shape Shape() => LocOpe_WorkingShapes.Published(InternalShape());

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal Shape InternalShape()
    {
        if (myResult.IsNull)
        {
            throw new NotDoneException();
        }
        return myResult;
    }

    /// <summary>The descendants of <paramref name="S"/>, a face of the shape or an edge of the wire (frozen).</summary>
    public NCollection_List<Shape> ShapesFromShape(in Shape S) => LocOpe_WorkingShapes.Published(InternalShapesFromShape(S));

    /// <summary><see cref="ShapesFromShape"/> without freezing, for ported code (of the caller's shape or its working copy).</summary>
    internal NCollection_List<Shape> InternalShapesFromShape(in Shape S)
    {
        if (myResult.IsNull)
        {
            throw new NotDoneException();
        }
        return myMap[myWork.Copied(S)];
    }

    private static bool NewPlane(in Face F, in Direction3D Extr, in Plane3D Neutr, double Ang, out Plane3D Newpl, out Axis3D NormalF,
                                 bool Modify)
    {
        Newpl = new Plane3D();
        NormalF = new Axis3D();

        // Determination du nouveau plan incline
        Surface S = Tool.Surface(F)!;
        if (S.GetType() == typeof(TrimmedSurface))
        {
            S = ((TrimmedSurface)S).BasisSurface();
        }
        if (S is not PlaneSurface P)
        {
            return false;
        }

        Plane3D Plorig = P.Pln();
        if (!Modify)
        {
            Newpl = Plorig;
            NormalF = Newpl.Axis();
            if ((Newpl.Direct() && F.Orientation() == Reversed) || (!Newpl.Direct() && F.Orientation() == Forward))
            {
                NormalF.Reverse();
            }
            return true;
        }

        Axis3D Axe;
        double Theta;

        QuadQuadGeo i2pl = new(Plorig, Neutr, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());

        if (i2pl.IsDone && i2pl.TypeInter() == Line)
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
                Orientation Oris = F.Orientation();
                if ((direct && Oris == Reversed) || (!direct && Oris == Forward))
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

    private static void MakeFace(ref Face F, NCollection_List<Shape> ledg)
    {
        // ledg est une liste d'edge

        BrepBuilder B = new();

        // Verification de l`existence des p-curves. Celles qui manquent
        // correspondent necessairement a des isos (et meme des iso u).

        double f, l;
        NCollection_List<Shape>.Iterator itl = new(ledg);
        for (; itl.More(); itl.Next())
        {
            Edge edg = ShapeCasts.Edge(itl.Value());
            Curve2D? C2d = Tool.CurveOnSurface(edg, F, out f, out l);
            if (C2d is null)
            {
                Tool.Range(edg, out f, out l);
                Exploration.Vertices(edg, out Vertex V1, out Vertex V2);
                NCollection_List<Shape>.Iterator itl2 = new();
                for (itl2.Initialize(ledg); itl2.More(); itl2.Next())
                {
                    Edge edg2 = ShapeCasts.Edge(itl2.Value());
                    if (edg2.IsSame(edg))
                    {
                        continue;
                    }
                    Exploration.Vertices(edg2, out Vertex Vp1, out Vertex Vp2);
                    if (Vp1.IsSame(V1) || Vp2.IsSame(V1) || Vp1.IsSame(V2) || Vp2.IsSame(V2))
                    {
                        Curve2D? C22d = Tool.CurveOnSurface(edg2, F, out double f2, out double l2);
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
                            C2d = new LineCurve2D(pt2d, Constants.DY2d());
                            B.UpdateEdge(edg, C2d, F, Tool.Tolerance(edg));
                            break;
                        }
                    }
                }
                // OCCT writes "Ca merde violemment" to std::cout when C2d is still null (left out)
            }
        }

        NCollection_List<Shape> lwires = new();
        bool alldone = ledg.IsEmpty();
        while (!alldone)
        {
            Wire Wnew = new();
            B.MakeWire(ref Wnew);
            Shape aLocalShapeCur = ledg.First();
            Edge edg = ShapeCasts.Edge(aLocalShapeCur);
            Vertex VFirst, VLast;
            if (edg.Orientation() == Forward)
            {
                Exploration.Vertices(edg, out VFirst, out VLast);
            }
            else
            {
                Exploration.Vertices(edg, out VLast, out VFirst);
            }
            B.Add(Wnew, edg);
            ledg.RemoveFirst();
            // on suppose VFirst et VLast non nuls
            bool wdone = (ledg.IsEmpty() || VFirst.IsSame(VLast));
            while (!wdone)
            {
                Orientation oredg = Forward;

                for (itl.Initialize(ledg); itl.More(); itl.Next())
                {
                    Edge edg2 = ShapeCasts.Edge(itl.Value());
                    Shape aLocalShape = edg2.Oriented(Forward);
                    Exploration.Vertices(ShapeCasts.Edge(aLocalShape), out Vertex VF, out Vertex VL);
                    if (VF.IsSame(VLast))
                    {
                        VLast = VL;
                        oredg = Forward;
                        break;
                    }
                    else if (VL.IsSame(VFirst))
                    {
                        VFirst = VF;
                        oredg = Forward;
                        break;
                    }
                    else if (VF.IsSame(VFirst))
                    {
                        VFirst = VL;
                        oredg = Reversed;
                        break;
                    }
                    else if (VL.IsSame(VLast))
                    {
                        VLast = VF;
                        oredg = Reversed;
                        break;
                    }
                }
                if (!itl.More())
                {
                    wdone = true;
                }
                else
                {
                    Shape aLocalShape = itl.Value().Oriented(oredg);
                    B.Add(Wnew, ShapeCasts.Edge(aLocalShape));
                    ledg.Remove(itl);
                    wdone = (ledg.IsEmpty() || VFirst.IsSame(VLast));
                }
            }
            lwires.Append(Wnew);
            alldone = ledg.IsEmpty();
        }

        F.Orientation(Forward);
        for (itl.Initialize(lwires); itl.More(); itl.Next())
        {
            Shape aLocalShape = F.EmptyCopied();
            Face NewFace = ShapeCasts.Face(aLocalShape);
            B.Add(NewFace, itl.Value());
            Properties GP = new();
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

    private static bool Contains(NCollection_List<Shape> ll, in Shape s)
    {
        NCollection_List<Shape>.Iterator itl = new();
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
    private static Curve2D Translatable(Curve2D theCurve) =>
        theCurve.IsFrozen ? (Curve2D)theCurve.Copy() : theCurve;

    private static Edge NewEdge(in Edge edg, in Face F, Surface NewS, in Vertex V1, in Vertex V2)
    {
        Edge NewEdg = new();
        Surface S1 = Tool.Surface(F)!;
        bool AppS1 = false;
        if (S1.GetType() != typeof(PlaneSurface))
        {
            AppS1 = true;
        }

        GeomInt_IntSS i2s = new(S1, NewS, global::Kerf.Engine.Foundation.Precision.Confusion(), true, AppS1);
        if (!i2s.IsDone() || i2s.NbLines() <= 0)
        {
            return NewEdg;
        }

        BrepBuilder B = new();
        double Dist2, Dist2Min;
        double prmf = 0, prml = 0;
        CurveAdaptor TheCurve = new();

        int i, k;
        Point3D pvf = Tool.Pnt(V1);
        Point3D pvl = Tool.Pnt(V2);
        for (i = 1; i <= i2s.NbLines(); i++)
        {
            TheCurve.Load(i2s.Line(i));
            CurvePointExtrema myExtPC = new(pvf, TheCurve);

            if (myExtPC.IsDone)
            {
                double thepmin = TheCurve.FirstParameter();
                myExtPC.TrimmedSquareDistances(out Dist2Min, out Dist2, out Point3D p1b, out Point3D p2b);
                if (Dist2 < Dist2Min && !TheCurve.IsPeriodic)
                {
                    Dist2Min = Dist2;
                    thepmin = TheCurve.LastParameter();
                }
                for (k = 1; k <= myExtPC.ExtCount; k++)
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
                    if (myExtPC.IsDone)
                    {
                        thepmin = TheCurve.LastParameter();
                        myExtPC.TrimmedSquareDistances(out Dist2, out Dist2Min, out p1b, out p2b);
                        if (Dist2 < Dist2Min && !TheCurve.IsClosed)
                        {
                            Dist2Min = Dist2;
                            thepmin = TheCurve.FirstParameter();
                        }
                        for (k = 1; k <= myExtPC.ExtCount; k++)
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
            Vertex Vf = V1;
            Vertex Vl = V2;
            Curve3D Cimg = i2s.Line(i);
            Curve2D? Cimg2d = null;
            if (AppS1)
            {
                Cimg2d = i2s.LineOnS1(i);
            }

            if (Cimg.IsPeriodic)
            {
                double period = Cimg.Period();
                double imf = Cimg.FirstParameter();
                double iml = Cimg.LastParameter();

                Tool.Range(edg, out double f, out double l);
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

            if (S1.IsUPeriodic)
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
                Vf.Orientation(Forward);
                Vl.Orientation(Reversed);
            }
            else
            {
                Vf.Orientation(Reversed);
                Vl.Orientation(Forward);
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
                NewEdg.Orientation(Reversed);
            }
        }
        return NewEdg;
    }
}
