// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitDrafts.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitDrafts.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Intersection.ResultType;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// Splits a planar face of a shape by a wire lying on it and puts a draft angle on one or both parts.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, the splitting and the rebuilding write into the shape and the wire (vertex point representations,
/// pcurves, continuities, wires added to the faces it reuses). So they are written in a working overlay (decision
/// 1013, unit 10H5: <see cref="WorkingShapes"/>), and the result holds new TShapes only for the caller's sub-shapes
/// that changed. <see cref="Shape"/> and <see cref="ShapesFromShape"/> freeze what they hand out.
/// </para>
/// <para>
/// OCCT writes its failures to std::cout ("LocOpe_SplitDrafts:!Fins ...", "fin newplane return standard_false",
/// "LocOpe_SplitDrafts: betite probleme", "Ca merde violemment", "Not yet implemented : nbwire >= 2"); the port leaves
/// the messages out.
/// </para>
/// </remarks>
internal sealed class SplitDrafts
{
    private Shape _shape = new();
    private Shape _result = new();
    // OCCT iterates this map keyed by the shapes' addresses to store the descendants; each entry is handled alone,
    // apart from the last pass collecting the faces of the result (whose order follows the map's: see the remarks of
    // LocOpe_BuildShape)
    private readonly Dictionary<Shape, NCollection_List<Shape>> _map = new(ShapeHasher.Instance);
    private WorkingShapes _work = new();

    /// <summary>Empty constructor.</summary>
    public SplitDrafts()
    {
    }

    /// <summary>Creates the algorithm on the shape <paramref name="s"/>.</summary>
    public SplitDrafts(in Shape s)
    {
        _shape = s;
    }

    /// <summary>Initializes the algorithm with the shape <paramref name="s"/>.</summary>
    public void Init(in Shape s)
    {
        _shape = s;
        _result.Nullify();
        _map.Clear();
    }

    /// <summary>
    /// Splits the face <paramref name="f"/> with the wire <paramref name="w"/> and puts the draft angle
    /// <paramref name="angle"/>, measured from <paramref name="extr"/> with the neutral plane <paramref name="nPl"/>, on
    /// the left part.
    /// </summary>
    public void Perform(in Face f, in Wire w, in Direction3D extr, in Plane3D nPl, double angle)
    {
        Perform(f, w, extr, nPl, angle, extr, nPl, angle, true, false);
    }

    /// <summary>
    /// Splits the face <paramref name="F"/> of the shape with the wire <paramref name="w"/>, lying on it, and puts a
    /// draft angle on both parts: <paramref name="extrg"/>, <paramref name="nPlg"/>, <paramref name="angleg"/> for the
    /// left part, <paramref name="extrd"/>, <paramref name="nPld"/>, <paramref name="angled"/> for the right part, each
    /// part left as it is when its Modify flag is false.
    /// </summary>
    public void Perform(in Face F,
                        in Wire w,
                        in Direction3D extrg,
                        in Plane3D nPlg,
                        double angleg,
                        in Direction3D extrd,
                        in Plane3D nPld,
                        double angled,
                        bool modLeft = true,
                        bool modRight = true)
    {
        int j;

        _result.Nullify();
        _map.Clear();
        if (_shape.IsNull || F.IsNull || w.IsNull)
        {
            throw new NullObjectException();
        }

        if (!modLeft && !modRight)
        {
            throw new ConstructionErrorException();
        }

        // Kerf: the shape, the face and the wire are written in the working overlay (decision 1013, unit 10H5)
        _work = new WorkingShapes();
        using WorkingOverlay.Scope scope = _work.Enter();
        Shape shape = _shape;
        Face aF = F;
        Wire aW = w;

        Orientation oriF = Forward;

        bool finS = false;
        Explorer exp = new(), exp2 = new();
        for (exp.Init(shape, ShapeType.Face); exp.More(); exp.Next())
        {
            Shape fac = exp.Current();
            NCollection_List<Shape> thelist = new();
            _map[fac] = thelist;
            if (fac.IsSame(aF))
            {
                oriF = fac.Orientation;
                finS = true;
            }
        }

        if (!finS)
        {
            throw new ConstructionErrorException();
        }

        Shape localFace = aF.Oriented(oriF);

        if (!NewPlane(ShapeCasts.Face(localFace), extrg, nPlg, angleg, out Plane3D newPlg, out Axis3D normalFg, modLeft)
            || !NewPlane(ShapeCasts.Face(localFace), extrd, nPld, angled, out Plane3D newPld, out Axis3D normalFd, modRight))
        {
            return;
        }
        _ = normalFd;

        NCollection_List<Shape>.Iterator itl = new();
        BrepBuilder b = new();

        Surface newSg = new PlaneSurface(newPlg);
        Surface newSd = new PlaneSurface(newPld);
        LineCurve3D theLinePipe = new(normalFg); // ou NormalFd
        IntSS i2s = new(newSg, newSd, global::Kerf.Engine.Foundation.Precision.Confusion());

        HashSet<Shape> theMap = new(ShapeHasher.Instance);
        CurveAdaptor hac = new();
        SurfaceAdaptor has = new();
        HInter intcs = new();

        Wire theW = aW;
        if (i2s.IsDone && i2s.LineCount > 0)
        {
            // on split le wire

            SurfacePipe thePipe = new();
            thePipe.GenerateParticularCase(true);
            // OCCT: the planes' intersection is an infinite line, which GeomFill_UniformSection can't convert to a
            // B-spline: Standard_DomainError (occt-issues.md)
            thePipe.Init(theLinePipe, i2s.Line(1));
            thePipe.Perform(true);
            if (!thePipe.IsDone)
            {
                throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
            }

            Surface spl = thePipe.InternalSurface()!;
            has.Load(spl);

            LocalSplitShape splw = new(aW, _work);

            for (exp.Init(aW, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                if (theMap.Add(edg))
                {
                    Curve3D? c = Tool.Curve(edg, out double f, out double l);
                    hac.Load(c!);
                    intcs.Perform(hac, has);
                    if (!intcs.IsDone)
                    {
                        continue; // voir ce qu`on peut faire de mieux
                    }

                    if (intcs.SegmentCount >= 2)
                    {
                        continue; // Not yet implemented...and probably never"
                    }

                    if (intcs.SegmentCount == 1)
                    {
                        CurveSurfaceIntersectionPoint P1 = intcs.Segment(1).FirstPoint();
                        CurveSurfaceIntersectionPoint P2 = intcs.Segment(1).SecondPoint();
                        Point3D pf = P1.Pnt();
                        Point3D pl = P2.Pnt();
                        Exploration.Vertices(edg, out Vertex vf, out Vertex vl);
                        Point3D Pf = Tool.Pnt(vf);
                        Point3D Pl = Tool.Pnt(vl);
                        double tolf = Tool.Tolerance(vf);
                        double toll = Tool.Tolerance(vl);
                        tolf *= tolf;
                        toll *= toll;

                        double dff = pf.SquareDistanceTo(Pf);
                        double dfl = pf.SquareDistanceTo(Pl);
                        double dlf = pl.SquareDistanceTo(Pf);
                        double dll = pl.SquareDistanceTo(Pl);

                        if ((dff <= tolf && dll <= toll) || (dlf <= tolf && dfl <= toll))
                        {
                            continue;
                        }
                        else
                        {
                            // on segmente edg en pf et pl
                            Vertex vnewf = new(), vnewl = new();
                            b.MakeVertex(ref vnewf, pf, global::Kerf.Engine.Foundation.Precision.Confusion());
                            b.MakeVertex(ref vnewl, pl, global::Kerf.Engine.Foundation.Precision.Confusion());
                            if (P1.W() >= f && P1.W() <= l && P2.W() >= f && P2.W() <= l)
                            {
                                splw.InternalAdd(vnewf, P1.W(), edg);
                                splw.InternalAdd(vnewl, P2.W(), edg);
                            }
                            else
                            {
                                continue;
                            }
                        }
                    }
                    else if (intcs.PointCount != 0)
                    {
                        Exploration.Vertices(edg, out Vertex vf, out Vertex vl);
                        Point3D Pf = Tool.Pnt(vf);
                        Point3D Pl = Tool.Pnt(vl);
                        double tolf = Tool.Tolerance(vf);
                        double toll = Tool.Tolerance(vl);
                        tolf *= tolf;
                        toll *= toll;

                        for (int i = 1; i <= intcs.PointCount; i++)
                        {
                            CurveSurfaceIntersectionPoint Pi = intcs.Point(i);
                            Point3D pi = Pi.Pnt();
                            double dif = pi.SquareDistanceTo(Pf);
                            double dil = pi.SquareDistanceTo(Pl);
                            if (dif <= tolf)
                            {
                            }
                            else if (dil <= toll)
                            {
                            }
                            else
                            {
                                if (Pi.W() >= f && Pi.W() <= l)
                                {
                                    Vertex vnew = new();
                                    b.MakeVertex(ref vnew, pi, global::Kerf.Engine.Foundation.Precision.Confusion());
                                    splw.InternalAdd(vnew, Pi.W(), edg);
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
                if (!_map.ContainsKey(exp.Current()))
                {
                    NCollection_List<Shape> thelist1 = new();
                    _map[exp.Current()] = thelist1;
                    for (itl.Initialize(splw.InternalDescendantShapes(exp.Current())); itl.More(); itl.Next())
                    {
                        _map.GetValue(exp.Current()).Append(itl.Value());
                    }
                    for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        if (!_map.ContainsKey(exp2.Current()))
                        {
                            NCollection_List<Shape> thelist2 = new();
                            _map[exp2.Current()] = thelist2;
                            _map.GetValue(exp2.Current()).Append(exp2.Current());
                        }
                    }
                }
            }
        }
        else
        {
            for (exp.Init(aW, ShapeType.Edge); exp.More(); exp.Next())
            {
                if (!_map.ContainsKey(exp.Current()))
                {
                    NCollection_List<Shape> thelist3 = new();
                    _map[exp.Current()] = thelist3;
                    _map.GetValue(exp.Current()).Append(exp.Current());
                    for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        if (!_map.ContainsKey(exp2.Current()))
                        {
                            NCollection_List<Shape> thelist4 = new();
                            _map[exp2.Current()] = thelist4;
                            _map.GetValue(exp2.Current()).Append(exp2.Current());
                        }
                    }
                }
            }
        }

        // On split la face par le wire

        WiresOnShape wonS = new(shape, _work);
        Spliter spls = new(shape);
        wonS.Bind(theW, aF);

        // JAG Le code suivant marchera apres integration de thick0 (LocOpe_FindEdges, commented out in OCCT)

        spls.InternalPerform(wonS);
        if (!spls.IsDone)
        {
            return;
        }

        Shape res = spls.InternalResultingShape();
        NCollection_List<Shape> theLeft = spls.InternalDirectLeft();

        // Descendants
        for (exp.Init(shape, ShapeType.Face); exp.More(); exp.Next())
        {
            Shape fac = exp.Current();
            for (itl.Initialize(spls.InternalDescendantShapes(fac)); itl.More(); itl.Next())
            {
                _map.GetValue(fac).Append(itl.Value());
            }
        }

        Dictionary<Shape, Shape> mapW = new(ShapeHasher.Instance);
        for (exp.Init(theW, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!mapW.ContainsKey(exp.Current()))
            {
                mapW[exp.Current()] = new Shape();
                for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
                {
                    if (!mapW.ContainsKey(exp2.Current()))
                    {
                        mapW[exp2.Current()] = new Shape();
                    }
                }
            }
        }

        IndexedMap<Shape, NCollection_List<Shape>> theMapEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(res, ShapeType.Edge, ShapeType.Face, theMapEF);

        // On stocke les geometries potentiellement generees par les edges
        IndexedMap<Shape, Shape> mapEV = new(ShapeHasher.Instance); // genere
        Dictionary<Shape, NCollection_List<Shape>> mapSg = new(ShapeHasher.Instance),
                                                                          mapSd = new(ShapeHasher.Instance); // image a gauche et a droite

        int nbedges, index;
        for (itl.Initialize(_map.GetValue(aF)); itl.More(); itl.Next())
        {
            Shape fac = ShapeCasts.Face(itl.Value());
            for (exp.Init(fac, ShapeType.Edge); exp.More(); exp.Next())
            {
                Shape edg = exp.Current();
                if (mapEV.ContainsKey(edg))
                {
                    continue;
                }
                if (mapW.ContainsKey(edg))
                { // edge du wire initial
                    Curve3D c = Tool.Curve(ShapeCasts.Edge(edg), out Placement loc, out _, out _)!;
                    if (c.GetType() == typeof(TrimmedCurve3D))
                    {
                        c = ((TrimmedCurve3D)c).BasisCurve;
                    }
                    c = (Curve3D)c.Transformed(loc.Transformation);

                    SurfacePipe thePipe = new();
                    thePipe.GenerateParticularCase(true);
                    thePipe.Init(theLinePipe, c);
                    thePipe.Perform(true);
                    if (!thePipe.IsDone)
                    {
                        throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
                    }

                    Surface thePS = thePipe.InternalSurface()!;
                    if (thePS.GetType() == typeof(TrimmedSurface))
                    {
                        thePS = ((TrimmedSurface)thePS).BasisSurface;
                    }

                    Face newFace = new();
                    b.MakeFace(ref newFace, thePS, global::Kerf.Engine.Foundation.Precision.Confusion());
                    mapEV.Add(edg, newFace);
                }
                else
                { // on recupere la face.
                    index = theMapEF.IndexOf(edg) + 1;
                    if (theMapEF.ValueAt(index - 1).Extent() != 2)
                    {
                        return; // NotDone
                    }
                    if (theMapEF.ValueAt(index - 1).First().IsSame(fac))
                    {
                        mapEV.Add(edg, theMapEF.ValueAt(index - 1).Last());
                    }
                    else
                    {
                        mapEV.Add(edg, theMapEF.ValueAt(index - 1).First());
                    }
                }
            }
        }

        Dictionary<Shape, Shape> mapSonS = new(ShapeHasher.Instance);

        nbedges = mapEV.Count;
        for (index = 1; index <= nbedges; index++)
        {
            for (exp.Init(mapEV.KeyAt(index - 1), ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (mapEV.ContainsKey(vtx))
                {
                    continue;
                }

                // Localisation du vertex :
                //    - entre 2 edges d`origine : on recupere l`edge qui n`est
                //                                pas dans F
                //    - entre 2 edges du wire   : droite
                //    - mixte                   : intersection de surfaces
                for (j = 1; j <= nbedges; j++)
                {
                    if (j == index)
                    {
                        continue;
                    }
                    for (exp2.Init(mapEV.KeyAt(j - 1), ShapeType.Vertex); exp2.More(); exp2.Next())
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
                int choice = 0;
                Shape edg1 = mapEV.KeyAt(index - 1);
                Shape edg2;
                if (j <= nbedges)
                {
                    edg2 = mapEV.KeyAt(j - 1);
                }
                else
                {
                    edg2 = edg1;
                }
                if (mapW.ContainsKey(edg1))
                {
                    if (j > nbedges)
                    {             // doit correspondre a edge ferme
                        choice = 2; // droite
                    }
                    else if (mapW.ContainsKey(mapEV.KeyAt(j - 1)))
                    {
                        choice = 2; // droite
                    }
                    else
                    {
                        choice = 3; // mixte
                    }
                }
                else
                {
                    if (j > nbedges)
                    {             // doit correspondre a edge ferme
                        choice = 1; // edge a retrouver
                    }
                    else if (!mapW.ContainsKey(mapEV.KeyAt(j - 1)))
                    {
                        choice = 1; // edge a retrouver
                    }
                    else
                    {
                        choice = 3; // mixte
                    }
                }
                Curve3D? newc = null;
                Curve2D? newCs1 = null, newCs2 = null;
                double knownp = 0;
                Edge ebind = new();
                switch (choice)
                {
                    case 1:
                    {
                        for (exp2.Init(res, ShapeType.Edge); exp2.More(); exp2.Next())
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
                            newc = Tool.Curve(ShapeCasts.Edge(exp2.Current()), out Placement loc, out _, out _)!;
                            newc = (Curve3D)newc.Transformed(loc.Transformation);
                            ebind = ShapeCasts.Edge(exp2.Current());
                            knownp = Tool.Parameter(vtx, ebind);
                        }
                        else
                        { // droite ??? il vaudrait mieux sortir
                            return;
                        }
                    }
                    break;
                    case 2:
                    {
                        Line3D theLine = new(normalFg);
                        theLine = theLine.Translated(normalFg.Location, Tool.Pnt(vtx));
                        newc = new LineCurve3D(theLine);
                        knownp = 0.0;
                    }
                    break;
                    case 3:
                    {
                        Face f1 = ShapeCasts.Face(mapEV.GetValue(edg1));
                        Face f2 = ShapeCasts.Face(mapEV.GetValue(edg2));
                        Surface s1 = Tool.Surface(f1)!;
                        Surface s2 = Tool.Surface(f2)!;
                        bool appS1 = false;
                        bool appS2 = false;
                        if (s1.GetType() != typeof(PlaneSurface))
                        {
                            appS1 = true;
                        }
                        if (s2.GetType() != typeof(PlaneSurface))
                        {
                            appS2 = true;
                        }
                        i2s.Perform(s1, s2, global::Kerf.Engine.Foundation.Precision.Confusion(), true, appS1, appS2);
                        if (!i2s.IsDone || i2s.LineCount <= 0)
                        {
                            return;
                        }

                        double pmin = 0, dist2 = 0, dist2Min = 0, glob2Min = RealLast();
                        CurveAdaptor theCurve = new();

                        int i, imin, k;
                        Point3D pv = Tool.Pnt(vtx);
                        imin = 0;
                        for (i = 1; i <= i2s.LineCount; i++)
                        {
                            theCurve.Load(i2s.Line(i));
                            CurvePointExtrema myExtPC = new(pv, theCurve);

                            if (myExtPC.IsDone)
                            {
                                double thepmin = theCurve.FirstParameter();
                                myExtPC.TrimmedSquareDistances(out dist2Min, out dist2, out Point3D p1b, out Point3D p2b);
                                if (dist2 < dist2Min)
                                {
                                    thepmin = theCurve.LastParameter();
                                }
                                for (k = 1; k <= myExtPC.ExtCount; k++)
                                {
                                    dist2 = myExtPC.SquareDistance(k);
                                    if (dist2 < dist2Min)
                                    {
                                        dist2Min = dist2;
                                        thepmin = myExtPC.Point(k).Parameter();
                                    }
                                }

                                if (dist2Min < glob2Min)
                                {
                                    glob2Min = dist2Min;
                                    pmin = thepmin;
                                    imin = i;
                                }
                            }
                        }
                        if (imin == 0)
                        {
                            return;
                        }

                        newc = i2s.Line(imin);
                        knownp = pmin;
                        if (appS1)
                        {
                            newCs1 = i2s.LineOnS1(imin);
                        }
                        if (appS2)
                        {
                            newCs2 = i2s.LineOnS2(imin);
                        }
                    }
                    break;
                }

                // Determination des vertex par intersection sur Plg ou/et Pld

                hac.Load(newc!);
                int nbfois = 2;
                Vertex vtx1 = new(), vtx2b = new();
                double p1 = 0, p2 = 0;
                bool isLeft = false;
                if (choice == 1)
                {
                    // edge retrouve : on ne fait qu`une seule intersection
                    // il faut utiliser Plg ou Pld

                    int indedgf = theMapEF.IndexOf(edg1) + 1;
                    for (itl.Initialize(theMapEF.ValueAt(indedgf - 1)); itl.More(); itl.Next())
                    {
                        if (Contains(_map.GetValue(aF), itl.Value()))
                        {
                            if (Contains(theLeft, itl.Value()))
                            {
                                has.Load(newSg);
                                isLeft = true;
                            }
                            else
                            {
                                has.Load(newSd);
                                isLeft = false;
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
                    has.Load(newSg);
                }

                for (int it = 1; it <= nbfois; it++)
                {
                    if (it == 2)
                    {
                        has.Load(newSd);
                    }

                    intcs.Perform(hac, has);
                    if (!intcs.IsDone)
                    {
                        return; // voir ce qu`on peut faire de mieux
                    }
                    int imin = 1;
                    double delta = Math.Abs(knownp - intcs.Point(1).W());
                    for (int i = 2; i <= intcs.PointCount; i++)
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
                        b.MakeVertex(ref vtx1, intcs.Point(imin).Pnt(), global::Kerf.Engine.Foundation.Precision.Confusion());
                        p1 = intcs.Point(imin).W();
                        knownp = p1;
                    }
                    else
                    {
                        b.MakeVertex(ref vtx2b, intcs.Point(imin).Pnt(), global::Kerf.Engine.Foundation.Precision.Confusion());
                        p2 = intcs.Point(imin).W();
                    }
                }
                if (Math.Abs(p1 - p2) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    Edge newEdge = new();
                    b.MakeEdge(ref newEdge, newc, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (p1 < p2)
                    {
                        b.Add(newEdge, vtx1.Oriented(Forward));
                        b.Add(newEdge, vtx2b.Oriented(Reversed));
                    }
                    else
                    {
                        b.Add(newEdge, vtx1.Oriented(Reversed));
                        b.Add(newEdge, vtx2b.Oriented(Forward));
                    }
                    b.UpdateVertex(vtx1, p1, newEdge, global::Kerf.Engine.Foundation.Precision.Confusion());
                    b.UpdateVertex(vtx2b, p2, newEdge, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (newCs1 is not null)
                    {
                        b.UpdateEdge(newEdge, newCs1, ShapeCasts.Face(mapEV.GetValue(edg1)), global::Kerf.Engine.Foundation.Precision.Confusion());
                    }

                    if (newCs2 is not null)
                    {
                        b.UpdateEdge(newEdge, newCs2, ShapeCasts.Face(mapEV.GetValue(edg2)), global::Kerf.Engine.Foundation.Precision.Confusion());
                    }

                    mapEV.Add(vtx, newEdge);

                    if (choice == 1)
                    {
                        Shape localEdge = ebind.EmptyCopied();
                        Edge ne = ShapeCasts.Edge(localEdge);
                        for (exp2.Init(ebind, ShapeType.Vertex); exp2.More(); exp2.Next())
                        {
                            Vertex thevtx = ShapeCasts.Vertex(exp2.Current());
                            if (thevtx.IsSame(vtx))
                            {
                                b.Add(ne, vtx1.Oriented(thevtx.Orientation));
                                b.UpdateVertex(vtx1, p1, ne, global::Kerf.Engine.Foundation.Precision.Confusion());
                            }
                            else
                            {
                                b.Add(ne, thevtx);
                                double theprm = Tool.Parameter(thevtx, ebind);
                                b.UpdateVertex(thevtx, theprm, ne, Tool.Tolerance(thevtx));
                            }
                        }
                        mapSonS[ebind] = ne.Oriented(Forward);
                        if (isLeft)
                        {
                            NCollection_List<Shape> thelist5 = new();
                            mapSg[vtx] = thelist5;
                            mapSg.GetValue(vtx).Append(vtx1);
                        }
                        else
                        {
                            NCollection_List<Shape> thelist6 = new();
                            mapSd[vtx] = thelist6;
                            mapSd.GetValue(vtx).Append(vtx1);
                        }
                    }
                    else
                    {
                        NCollection_List<Shape> thelist7 = new(), thelist8 = new();
                        mapSg[vtx] = thelist7;
                        mapSd[vtx] = thelist8;
                        mapSg.GetValue(vtx).Append(vtx1);
                        mapSd.GetValue(vtx).Append(vtx2b);
                    }
                }
                else
                {
                    mapEV.Add(vtx, vtx2b); // on peut avoir vtx2 = vtx si choix == 1
                    if (choice == 1)
                    {
                        if (isLeft)
                        {
                            NCollection_List<Shape> thelist9 = new();
                            mapSg[vtx] = thelist9;
                            mapSg.GetValue(vtx).Append(vtx);
                        }
                        else
                        {
                            NCollection_List<Shape> thelist10 = new();
                            mapSd[vtx] = thelist10;
                            mapSd.GetValue(vtx).Append(vtx);
                        }
                    }
                    else
                    {
                        NCollection_List<Shape> thelist11 = new(), thelist12 = new();
                        mapSg[vtx] = thelist11;
                        mapSd[vtx] = thelist12;
                        mapSg.GetValue(vtx).Append(vtx2b);
                        mapSd.GetValue(vtx).Append(vtx2b);
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
            int indedg = mapEV.IndexOf(edg) + 1;
            ref Shape genFRef = ref mapEV.ValueAt(indedg - 1);
            Face genF = ShapeCasts.Face(genFRef);
            NCollection_List<Shape> thelist13 = new(), thelist14 = new();
            mapSg[edg] = thelist13; // genere a gauche
            mapSd[edg] = thelist14; // genere a droite
            Shape localEdge = edg.Oriented(Forward);
            Exploration.Vertices(ShapeCasts.Edge(localEdge), out Vertex vf, out Vertex vl);
            Shape gvf = mapEV.GetValue(vf);
            Shape gvl = mapEV.GetValue(vl);

            // (OCCT keeps its former construction here, commented out; nouveau code)

            Vertex vfd, vld, vfg, vlg;
            if (gvf.Type == ShapeType.Vertex)
            {
                vfg = ShapeCasts.Vertex(gvf);
                vfd = vfg;
            }
            else
            {
                vfg = ShapeCasts.Vertex(mapSg.GetValue(vf).First());
                vfd = ShapeCasts.Vertex(mapSd.GetValue(vf).First());
            }
            if (gvl.Type == ShapeType.Vertex)
            {
                vlg = ShapeCasts.Vertex(gvl);
                vld = vlg;
            }
            else
            {
                vlg = ShapeCasts.Vertex(mapSg.GetValue(vl).First());
                vld = ShapeCasts.Vertex(mapSd.GetValue(vl).First());
            }

            Edge newEdgg = NewEdge(edg, genF, newSg, vfg, vlg);
            if (newEdgg.IsNull)
            {
                return;
            }

            Edge newEdgd = NewEdge(edg, genF, newSd, vfd, vld);
            if (newEdgg.IsNull) // OCCT: tests NewEdgg again
            {
                return;
            }

            bool isedg = false;
            if (gvf.Type == ShapeType.Vertex && gvl.Type == ShapeType.Vertex)
            {
                // edg ou face a 2 cotes

                // Comparaison NewEdgg et NewEdgd
                Curve3D cg = Tool.Curve(newEdgg, out double fg, out double lg)!;
                Curve3D cd = Tool.Curve(newEdgd, out double fd, out double ld)!;
                double prmg = (fg + lg) / 2.0;
                double prmd = (fd + ld) / 2.0;
                Point3D pg = cg.Value(prmg);
                Point3D pd = cd.Value(prmd);
                double tol = Max(Tool.Tolerance(newEdgg), Tool.Tolerance(newEdgg)); // OCCT: NewEdgg twice
                if (pg.SquareDistanceTo(pd) <= tol * tol)
                {
                    isedg = true;
                    // raffinement pour essayer de partager l`edge de depart...
                    bool modified = true;
                    if (gvf.IsSame(vf) && gvl.IsSame(vl))
                    {
                        // Comparaison avec l`edge de depart
                        cd = Tool.Curve(edg, out fd, out ld)!;
                        prmd = (fd + ld) / 2.0;
                        pd = cd.Value(prmd);
                        tol = Max(Tool.Tolerance(newEdgg), Tool.Tolerance(edg));
                        if (pg.SquareDistanceTo(pd) <= tol * tol)
                        {
                            modified = false;
                        }
                    }

                    if (!modified)
                    {
                        mapW.GetValueRef(edg) = edg;
                        mapSg.GetValue(edg).Append(edg);
                        mapSd.GetValue(edg).Append(edg);
                    }
                    else
                    {
                        mapW.GetValueRef(edg) = newEdgg;
                        mapSg.GetValue(edg).Append(newEdgg);
                        mapSd.GetValue(edg).Append(newEdgg);
                    }
                }
            }

            if (!isedg)
            {
                // face a 2 ou 3 ou 4 cotes
                mapSg.GetValue(edg).Append(newEdgg);
                mapSd.GetValue(edg).Append(newEdgd);

                NCollection_List<Shape> theedges = new();
                theedges.Append(newEdgg);
                theedges.Append(newEdgd);
                if (gvf.Type == ShapeType.Edge)
                {
                    theedges.Append(gvf);
                }
                if (gvl.Type == ShapeType.Edge)
                {
                    theedges.Append(gvl);
                }
                MakeFace(ref genF, theedges);
                genFRef = genF; // OCCT changes the face held in MapEV (its orientation) through a reference
                mapW.GetValueRef(edg) = genF;
            }
        }

        HashSet<Shape> mapedgadded = new(ShapeHasher.Instance);
        NCollection_List<Shape> thefaces = new();

        for (itl.Initialize(_map.GetValue(aF)); itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());
            theMap.Clear();
            Face drftFace = new(); // elle est FORWARD
            bool isLeft;
            if (Contains(theLeft, fac))
            {
                b.MakeFace(ref drftFace, newSg, Tool.Tolerance(fac));
                isLeft = true;
            }
            else
            {
                b.MakeFace(ref drftFace, newSd, Tool.Tolerance(fac));
                isLeft = false;
            }

            Explorer exp3 = new();
            for (exp3.Init(fac.Oriented(Forward), ShapeType.Wire); exp3.More(); exp3.Next())
            {
                Shape wir = exp3.Current();
                Wire newWireOnF = new();
                b.MakeWire(ref newWireOnF);
                for (exp.Init(wir.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge edg = ShapeCasts.Edge(exp.Current());
                    if (!theMap.Add(edg))
                    { // precaution sans doute inutile...
                        continue;
                    }
                    if (mapW.ContainsKey(edg))
                    { // edge du wire d`origine
                        NCollection_List<Shape>.Iterator itld = new();
                        Orientation ored = edg.Orientation;
                        if (isLeft)
                        {
                            itld.Initialize(mapSg.GetValue(edg));
                        }
                        else
                        {
                            itld.Initialize(mapSd.GetValue(edg));
                        }
                        for (; itld.More(); itld.Next())
                        {
                            if (itld.Value().Orientation == Reversed)
                            {
                                ored = global::Kerf.Engine.Topology.TopAbs.Reverse(ored);
                            }
                            Shape localEdge = itld.Value().Oriented(ored);
                            b.Add(newWireOnF, ShapeCasts.Edge(localEdge));
                        }
                    }
                    else
                    {
                        Surface newS;
                        if (isLeft)
                        {
                            newS = newSg;
                        }
                        else
                        {
                            newS = newSd;
                        }
                        int indedg = mapEV.IndexOf(edg) + 1;
                        Face genF = ShapeCasts.Face(mapEV.ValueAt(indedg - 1));
                        Shape localEdge = edg.Oriented(Forward);
                        Exploration.Vertices(ShapeCasts.Edge(localEdge), out Vertex vf, out Vertex vl);
                        Shape gvf = mapEV.GetValue(vf);
                        Shape gvl = mapEV.GetValue(vl);
                        if (gvf.Type == ShapeType.Vertex && gvl.Type == ShapeType.Vertex)
                        {
                            if (!gvf.IsSame(vf) || !gvl.IsSame(vl))
                            {
                                Edge newEdg = NewEdge(edg, genF, newS, ShapeCasts.Vertex(gvf), ShapeCasts.Vertex(gvl));
                                if (newEdg.IsNull)
                                {
                                    return;
                                }

                                mapSonS[edg] = newEdg;

                                if (newEdg.Orientation == Reversed)
                                {
                                    newEdg.Orient(global::Kerf.Engine.Topology.TopAbs.Reverse(edg.Orientation));
                                }
                                else
                                {
                                    newEdg.Orient(edg.Orientation);
                                }
                                b.Add(newWireOnF, newEdg);
                            }
                            else
                            { // Frozen???
                                b.Add(newWireOnF, edg);
                            }
                        }
                        else
                        {
                            Vertex vff, vll;
                            if (gvf.Type == ShapeType.Vertex)
                            {
                                vff = ShapeCasts.Vertex(gvf);
                            }
                            else
                            {
                                if (isLeft)
                                {
                                    vff = ShapeCasts.Vertex(mapSg.GetValue(vf).First());
                                }
                                else
                                {
                                    vff = ShapeCasts.Vertex(mapSd.GetValue(vf).First());
                                }
                            }
                            if (gvl.Type == ShapeType.Vertex)
                            {
                                vll = ShapeCasts.Vertex(gvl);
                            }
                            else
                            {
                                if (isLeft)
                                {
                                    vll = ShapeCasts.Vertex(mapSg.GetValue(vl).First());
                                }
                                else
                                {
                                    vll = ShapeCasts.Vertex(mapSd.GetValue(vl).First());
                                }
                            }

                            Edge newEdg = NewEdge(edg, genF, newS, vff, vll);
                            if (newEdg.IsNull)
                            {
                                return;
                            }

                            if (!mapW.ContainsKey(vf) && !mapW.ContainsKey(vl))
                            {
                                mapSonS[edg] = newEdg;
                            }
                            else
                            {
                                if (mapW.ContainsKey(vf))
                                {
                                    if (gvf.Type != ShapeType.Edge || mapedgadded.Contains(gvf))
                                    {
                                        mapSonS[edg] = newEdg;
                                    }
                                    else
                                    {
                                        Wire newWir = new();
                                        b.MakeWire(ref newWir);
                                        b.Add(newWir, newEdg);

                                        Exploration.Vertices(ShapeCasts.Edge(gvf), out Vertex vf2, out Vertex vl2);

                                        // ici bug orientation : voir tspdrft6

                                        if (vl2.IsSame(vff))
                                        {
                                            b.Add(newWir, gvf.Oriented(Forward));
                                        }
                                        else
                                        {
                                            b.Add(newWir, gvf.Oriented(Reversed));
                                        }
                                        mapedgadded.Add(gvf);
                                        mapSonS[edg] = newWir; // NewWire est FORWARD
                                        _ = vf2;
                                    }
                                }
                                else
                                {
                                    if (gvl.Type != ShapeType.Edge || mapedgadded.Contains(gvl))
                                    {
                                        mapSonS[edg] = newEdg;
                                    }
                                    else
                                    {
                                        Wire newWir = new();
                                        b.MakeWire(ref newWir);
                                        b.Add(newWir, newEdg);

                                        Exploration.Vertices(ShapeCasts.Edge(gvl), out Vertex vf2, out Vertex vl2);

                                        // ici bug orientation : voir tspdrft6

                                        if (vf2.IsSame(vll))
                                        {
                                            b.Add(newWir, gvl.Oriented(Forward));
                                        }
                                        else
                                        {
                                            b.Add(newWir, gvl.Oriented(Reversed));
                                        }
                                        mapedgadded.Add(gvl);
                                        mapSonS[edg] = newWir; // NewWire est FORWARD
                                        _ = vl2;
                                    }
                                }
                            }
                            if (newEdg.Orientation == Reversed)
                            {
                                newEdg.Orient(global::Kerf.Engine.Topology.TopAbs.Reverse(edg.Orientation));
                            }
                            else
                            {
                                newEdg.Orient(edg.Orientation);
                            }
                            b.Add(newWireOnF, newEdg);
                        }
                    }
                }
                b.Add(drftFace, newWireOnF.Oriented(wir.Orientation));
            }
            thefaces.Append(drftFace);
        }

        Substitution theSubs = new();
        foreach (KeyValuePair<Shape, Shape> itdmss in mapSonS)
        {
            NCollection_List<Shape> lsubs = new();
            for (exp.Init(itdmss.Value, ShapeType.Edge); exp.More(); exp.Next())
            {
                lsubs.Append(exp.Current());
            }
            theSubs.Substitute(itdmss.Key, lsubs);
        }

        // on reconstruit les faces
        for (exp.Init(res, ShapeType.Face); exp.More(); exp.Next())
        {
            if (Contains(_map.GetValue(aF), exp.Current()))
            {
                continue;
            }
            theSubs.Build(exp.Current());
        }

        // Stockage des descendants
        foreach (Shape key in _map.Keys.ToList())
        {
            NCollection_List<Shape> value = _map.GetValue(key);
            if (key.Type == ShapeType.Edge)
            {
                NCollection_List<Shape> thedesc = new();
                theMap.Clear();
                for (itl.Initialize(value); itl.More(); itl.Next())
                {
                    if (theMap.Add(mapW.GetValue(itl.Value())))
                    {
                        thedesc.Append(mapW.GetValue(itl.Value()));
                    }
                }
                _map.GetValueRef(key) = thedesc;
            }
            else if (key.IsSame(aF))
            {
                _map.GetValue(aF).Clear();
                for (itl.Initialize(thefaces); itl.More(); itl.Next())
                {
                    _map.GetValue(aF).Append(itl.Value());
                }
            }
            else
            {
                NCollection_List<Shape> thedesc = new();
                theMap.Clear();
                for (itl.Initialize(value); itl.More(); itl.Next())
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
                _map.GetValueRef(key) = thedesc;
            }
        }

        theMap.Clear();
        thefaces.Clear();
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdmsls in _map)
        {
            for (itl.Initialize(itdmsls.Value); itl.More(); itl.Next())
            {
                if (itl.Value().Type == ShapeType.Face && theMap.Add(itl.Value()))
                {
                    thefaces.Append(itl.Value());
                }
            }
        }
        BuildShape bs = new(thefaces);
        _result = bs.InternalShape();

        // Kerf: the end of the working overlay: new TShapes for the caller's sub-shapes changed (unit 10H5)
        if (_work.IsWritten)
        {
            var shapes = new List<Shape> { _result };
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _map)
            {
                shapes.AddRange(item.Value);
            }
            _work.Finish(shapes);
            _result = _work.Final(_result);
            foreach (Shape key in _map.Keys.ToList())
            {
                _map.GetValueRef(key) = _work.Final(_map.GetValue(key));
            }
        }
    }

    /// <summary>Returns true if the modification has been successfully performed.</summary>
    public bool IsDone => !_result.IsNull;

    /// <summary>The shape given (the caller's).</summary>
    public Shape OriginalShape() => _shape;

    /// <summary>Returns the modified shape (frozen).</summary>
    public Shape Shape() => WorkingShapes.Published(InternalShape());

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal Shape InternalShape()
    {
        if (_result.IsNull)
        {
            throw new NotDoneException();
        }
        return _result;
    }

    /// <summary>The descendants of <paramref name="s"/>, a face of the shape or an edge of the wire (frozen).</summary>
    public NCollection_List<Shape> ShapesFromShape(in Shape s) => WorkingShapes.Published(InternalShapesFromShape(s));

    /// <summary><see cref="ShapesFromShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalShapesFromShape(in Shape s)
    {
        if (_result.IsNull)
        {
            throw new NotDoneException();
        }
        return _map.GetValue(s);
    }

    private static bool NewPlane(in Face f, in Direction3D extr, in Plane3D neutr, double ang, out Plane3D newpl, out Axis3D normalF,
                                 bool modify)
    {
        newpl = new Plane3D();
        normalF = new Axis3D();

        // Determination du nouveau plan incline
        Surface s = Tool.Surface(f)!;
        if (s.GetType() == typeof(TrimmedSurface))
        {
            s = ((TrimmedSurface)s).BasisSurface;
        }
        if (s is not PlaneSurface p)
        {
            return false;
        }

        Plane3D plorig = p.Plane;
        if (!modify)
        {
            newpl = plorig;
            normalF = newpl.Axis;
            if ((newpl.IsDirect && f.Orientation == Reversed) || (!newpl.IsDirect && f.Orientation == Forward))
            {
                normalF = normalF.Reversed();
            }
            return true;
        }

        Axis3D axe;
        double theta;

        QuadQuadGeo i2pl = new(plorig, neutr, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());

        if (i2pl.IsDone && i2pl.TypeInter() == Line)
        {
            Line3D linInters = i2pl.Line(1);
            Direction3D nx = linInters.Direction;
            normalF = plorig.Axis;
            Direction3D ny = normalF.Direction.Crossed(nx);
            double a = extr.Dot(nx);
            if (Math.Abs(a) <= 1 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                double b = extr.Dot(ny);
                double c = extr.Dot(normalF.Direction);
                bool direct = plorig.IsDirect;
                Orientation oris = f.Orientation;
                if ((direct && oris == Reversed) || (!direct && oris == Forward))
                {
                    b = -b;
                    c = -c;
                    normalF = normalF.Reversed();
                }
                double denom = Math.Sqrt(1 - a * a);
                double sina = Math.Sin(ang);
                if (denom > Math.Abs(sina))
                {
                    double phi = Math.Atan2(b / denom, c / denom);
                    double theta0 = Math.Acos(sina / denom);
                    theta = theta0 - phi;
                    if (Math.Cos(theta) < 0.0)
                    {
                        theta = -theta0 - phi;
                    }
                    axe = linInters.Position;
                    newpl = plorig.Rotated(axe, theta);
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

        BrepBuilder b = new();

        // Verification de l`existence des p-curves. Celles qui manquent
        // correspondent necessairement a des isos (et meme des iso u).

        double f, l;
        NCollection_List<Shape>.Iterator itl = new(ledg);
        for (; itl.More(); itl.Next())
        {
            Edge edg = ShapeCasts.Edge(itl.Value());
            Curve2D? c2d = Tool.CurveOnSurface(edg, F, out f, out l);
            if (c2d is null)
            {
                Tool.Range(edg, out f, out l);
                Exploration.Vertices(edg, out Vertex v1, out Vertex v2);
                NCollection_List<Shape>.Iterator itl2 = new();
                for (itl2.Initialize(ledg); itl2.More(); itl2.Next())
                {
                    Edge edg2 = ShapeCasts.Edge(itl2.Value());
                    if (edg2.IsSame(edg))
                    {
                        continue;
                    }
                    Exploration.Vertices(edg2, out Vertex vp1, out Vertex vp2);
                    if (vp1.IsSame(v1) || vp2.IsSame(v1) || vp1.IsSame(v2) || vp2.IsSame(v2))
                    {
                        Curve2D? c22d = Tool.CurveOnSurface(edg2, F, out double f2, out double l2);
                        if (c22d is not null)
                        {
                            Point2D pt2d = new();
                            if (vp1.IsSame(v1))
                            {
                                pt2d = c22d.Value(f2);
                                pt2d = pt2d.WithY(pt2d.Y - f);
                            }
                            else if (vp2.IsSame(v1))
                            {
                                pt2d = c22d.Value(l2);
                                pt2d = pt2d.WithY(pt2d.Y - f);
                            }
                            else if (vp1.IsSame(v2))
                            {
                                pt2d = c22d.Value(f2);
                                pt2d = pt2d.WithY(pt2d.Y - l);
                            }
                            else if (vp2.IsSame(v2))
                            {
                                pt2d = c22d.Value(l2);
                                pt2d = pt2d.WithY(pt2d.Y - l);
                            }
                            c2d = new LineCurve2D(pt2d, Constants.DY2d());
                            b.UpdateEdge(edg, c2d, F, Tool.Tolerance(edg));
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
            Wire wnew = new();
            b.MakeWire(ref wnew);
            Shape localShapeCur = ledg.First();
            Edge edg = ShapeCasts.Edge(localShapeCur);
            Vertex vFirst, vLast;
            if (edg.Orientation == Forward)
            {
                Exploration.Vertices(edg, out vFirst, out vLast);
            }
            else
            {
                Exploration.Vertices(edg, out vLast, out vFirst);
            }
            b.Add(wnew, edg);
            ledg.RemoveFirst();
            // on suppose VFirst et VLast non nuls
            bool wdone = (ledg.IsEmpty() || vFirst.IsSame(vLast));
            while (!wdone)
            {
                Orientation oredg = Forward;

                for (itl.Initialize(ledg); itl.More(); itl.Next())
                {
                    Edge edg2 = ShapeCasts.Edge(itl.Value());
                    Shape localShape = edg2.Oriented(Forward);
                    Exploration.Vertices(ShapeCasts.Edge(localShape), out Vertex vf, out Vertex vl);
                    if (vf.IsSame(vLast))
                    {
                        vLast = vl;
                        oredg = Forward;
                        break;
                    }
                    else if (vl.IsSame(vFirst))
                    {
                        vFirst = vf;
                        oredg = Forward;
                        break;
                    }
                    else if (vf.IsSame(vFirst))
                    {
                        vFirst = vl;
                        oredg = Reversed;
                        break;
                    }
                    else if (vl.IsSame(vLast))
                    {
                        vLast = vf;
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
                    Shape localShape = itl.Value().Oriented(oredg);
                    b.Add(wnew, ShapeCasts.Edge(localShape));
                    ledg.Remove(itl);
                    wdone = (ledg.IsEmpty() || vFirst.IsSame(vLast));
                }
            }
            lwires.Append(wnew);
            alldone = ledg.IsEmpty();
        }

        F.Orient(Forward);
        for (itl.Initialize(lwires); itl.More(); itl.Next())
        {
            Shape localShape = F.EmptyCopied();
            Face newFace = ShapeCasts.Face(localShape);
            b.Add(newFace, itl.Value());
            Properties gp = new();
            MassPropertiesBuilder.SurfaceProperties(newFace, gp);
            if (gp.Mass() < 0)
            {
                itl.ChangeValue().Reverse();
            }
        }
        if (lwires.Extent() == 1)
        {
            b.Add(F, lwires.First());
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

    // Kerf: OCCT translates the intersection's pcurve in place; a frozen one is translated on a copy
    private static Curve2D Translatable(Curve2D curve) =>
        curve.IsFrozen ? (Curve2D)curve.Copy() : curve;

    private static Edge NewEdge(in Edge edg, in Face F, Surface newS, in Vertex v1, in Vertex v2)
    {
        Edge newEdg = new();
        Surface s1 = Tool.Surface(F)!;
        bool appS1 = false;
        if (s1.GetType() != typeof(PlaneSurface))
        {
            appS1 = true;
        }

        IntSS i2s = new(s1, newS, global::Kerf.Engine.Foundation.Precision.Confusion(), true, appS1);
        if (!i2s.IsDone || i2s.LineCount <= 0)
        {
            return newEdg;
        }

        BrepBuilder b = new();
        double dist2, dist2Min;
        double prmf = 0, prml = 0;
        CurveAdaptor theCurve = new();

        int i, k;
        Point3D pvf = Tool.Pnt(v1);
        Point3D pvl = Tool.Pnt(v2);
        for (i = 1; i <= i2s.LineCount; i++)
        {
            theCurve.Load(i2s.Line(i));
            CurvePointExtrema myExtPC = new(pvf, theCurve);

            if (myExtPC.IsDone)
            {
                double thepmin = theCurve.FirstParameter();
                myExtPC.TrimmedSquareDistances(out dist2Min, out dist2, out Point3D p1b, out Point3D p2b);
                if (dist2 < dist2Min && !theCurve.IsPeriodic)
                {
                    dist2Min = dist2;
                    thepmin = theCurve.LastParameter();
                }
                for (k = 1; k <= myExtPC.ExtCount; k++)
                {
                    dist2 = myExtPC.SquareDistance(k);
                    if (dist2 < dist2Min)
                    {
                        dist2Min = dist2;
                        thepmin = myExtPC.Point(k).Parameter();
                    }
                }

                if (dist2Min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    prmf = thepmin;
                    myExtPC.Perform(pvl);
                    if (myExtPC.IsDone)
                    {
                        thepmin = theCurve.LastParameter();
                        myExtPC.TrimmedSquareDistances(out dist2, out dist2Min, out p1b, out p2b);
                        if (dist2 < dist2Min && !theCurve.IsClosed)
                        {
                            dist2Min = dist2;
                            thepmin = theCurve.FirstParameter();
                        }
                        for (k = 1; k <= myExtPC.ExtCount; k++)
                        {
                            dist2 = myExtPC.SquareDistance(k);
                            if (dist2 < dist2Min)
                            {
                                dist2Min = dist2;
                                thepmin = myExtPC.Point(k).Parameter();
                            }
                        }

                        if (dist2Min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                        {
                            prml = thepmin;
                            break;
                        }
                    }
                }
            }
        }

        if (i <= i2s.LineCount)
        {
            bool rev = false;
            Vertex vf = v1;
            Vertex vl = v2;
            Curve3D cimg = i2s.Line(i);
            Curve2D? cimg2d = null;
            if (appS1)
            {
                cimg2d = i2s.LineOnS1(i);
            }

            if (cimg.IsPeriodic)
            {
                double period = cimg.Period;
                double imf = cimg.FirstParameter;
                double iml = cimg.LastParameter;

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

            if (s1.IsUPeriodic)
            {
                double speriod = s1.UPeriod;
                Point2D pf, pl;
                pf = cimg2d!.Value(prmf);
                pl = cimg2d.Value(prml);

                double uf = pf.X;
                double ul = pl.X;
                double ptra = 0.0;

                double ustart = Min(uf, ul);
                while (ustart < -global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    ustart += speriod;
                    ptra += speriod;
                }
                while (ustart > speriod - global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    ustart -= speriod;
                    ptra -= speriod;
                }
                if (ptra != 0)
                {
                    cimg2d = Translatable(cimg2d);
                    cimg2d.Translate(new Vector2D(ptra, 0.0));
                }
            }
            if (prmf < prml)
            {
                vf.Orient(Forward);
                vl.Orient(Reversed);
            }
            else
            {
                vf.Orient(Reversed);
                vl.Orient(Forward);
                rev = true;
            }

            b.MakeEdge(ref newEdg, cimg, global::Kerf.Engine.Foundation.Precision.Confusion());

            b.Add(newEdg, vf);
            b.Add(newEdg, vl);
            b.UpdateVertex(vf, prmf, newEdg, global::Kerf.Engine.Foundation.Precision.Confusion());
            b.UpdateVertex(vl, prml, newEdg, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (appS1)
            {
                b.UpdateEdge(newEdg, cimg2d, F, global::Kerf.Engine.Foundation.Precision.Confusion());
            }

            if (rev)
            {
                newEdg.Orient(Reversed);
            }
        }
        return newEdg;
    }
}
