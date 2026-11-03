// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Pipe.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Pipe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Filling.TransitionStyle;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Construction.Surfaces.ApproxStyle;
using static Kerf.Engine.Construction.Surfaces.Trihedron;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Creates a shape by sweeping a shape (the profile) along a wire (the spine). For each edge or vertex of the spine,
/// the shape generated from each sub-shape of the profile can be asked for.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21), as BRepSweep_Prism under BRepPrimAPI_MakePrism: it sweeps the profile in place, as
/// OCCT does, and the sweep writes into the profile's sub-shapes it shares in the result (the faces at the ends of a
/// face's pipe, the section edges). Given a frozen profile, such a write throws <see cref="ImmutableObjectException"/>;
/// <see cref="ApiMakePipe"/> gives it a copy of a frozen profile and maps its answers back to the caller's
/// sub-shapes (decision 910).
/// </para>
/// <para>
/// The accessors hand out frozen shapes; ported callers that keep building (BRepFill_Evolved) use the
/// <c>Internal</c> forms (decision 911).
/// </para>
/// </remarks>
public sealed class Pipe
{
    private Wire _spine = new();
    private Shape _profile = new();
    private Shape _shape = new();
    private Transform3D _trsf = new();
    private LocationLaw? _loc;
    private NCollection_HArray2<Shape>? _sections;
    private NCollection_HArray2<Shape>? _faces;
    private NCollection_HArray2<Shape>? _edges;
    private readonly HashSet<Shape> _reversedEdges = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_HArray2<Shape>> _tapes = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_HArray2<Shape>> _rails = new(ShapeHasher.Instance);
    private int _curIndexOfSectionEdge;
    private Shape _first = new();
    private Shape _last = new();
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> _genMap = new(ShapeHasher.Instance);
    private int _degmax;
    private int _segmax;
    private Continuity _continuity;
    private Trihedron _mode;
    private bool _forceApproxC1;
    private double _errorOnSurf; // OCCT: uninitialised until a sweep

    //=================================================================================================

    private static bool UpdateMap(in Shape key,
                                  in Shape value,
                                  NCollection_DataMap<Shape, NCollection_List<Shape>> map)
    {
        if (!map.IsBound(key))
        {
            NCollection_List<Shape> thelist = new();
            map.Bind(key, thelist);
        }
        NCollection_List<Shape> list = map.ChangeFind(key);
        NCollection_List<Shape>.Iterator it = new(list);
        bool found = false;

        for (; it.More(); it.Next())
        {
            if (value.IsSame(it.Value()))
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            list.Append(value);
        }

        return !found;
    }

    private static void UpdateTolFromTopOrBottomPCurve(in Face aFace, in Edge anEdge)
    {
        Curve2D? pCurve = Tool.CurveOnSurface(anEdge, aFace, out double fpar, out double lpar);
        if (pCurve is null)
        {
            return;
        }

        Curve3D? curve = Tool.Curve(anEdge, out fpar, out lpar);
        if (curve is null)
        {
            return;
        }

        CurveAdaptor2D gahc2d = new(pCurve, fpar, lpar);
        Surface surf = Tool.Surface(aFace)!;
        SurfaceAdaptor gahs = new(surf);
        CurveOnSurface conS = new(gahc2d, gahs);

        double tol = Tool.Tolerance(anEdge);
        double initTol = tol;
        double tolTol = tol * tol;
        const int ncontrol = 22;
        double delta = (lpar - fpar) / ncontrol;
        for (int i = 0; i <= ncontrol; i++)
        {
            double par = fpar + i * delta;
            Point3D pnt = curve.Value(par);
            Point3D prj = conS.Value(par);
            double sqdist = pnt.SquareDistance(prj);
            if (sqdist > tolTol)
            {
                tolTol = sqdist;
            }
        }
        tol = 1.00005 * Math.Sqrt(tolTol);
        if (tol >= initTol)
        {
            BrepBuilder bb = new();
            bb.UpdateEdge(anEdge, tol);
            Iterator itv = new(anEdge);
            for (; itv.More(); itv.Next())
            {
                Vertex vertex = ShapeCasts.Vertex(itv.Value());
                bb.UpdateVertex(vertex, tol);
            }
        }
    }

    //=================================================================================================

    /// <summary>An empty pipe; <see cref="Perform"/> builds it.</summary>
    public Pipe()
    {
        _degmax = 11;
        _segmax = 100;
        _continuity = C2;
        _mode = Trihedron.IsCorrectedFrenet;
        _forceApproxC1 = false;

        _curIndexOfSectionEdge = 1;
    }

    //=================================================================================================

    /// <summary>
    /// The pipe of <paramref name="profile"/> along <paramref name="spine"/>, with the trihedron
    /// <paramref name="aMode"/> (Frenet, corrected Frenet or discrete; any other mode is corrected Frenet), and an
    /// attempt at C1 surfaces where a swept surface is C0 with <paramref name="forceApproxC1"/>.
    /// </summary>
    public Pipe(in Wire spine,
                         in Shape profile,
                         Trihedron aMode = Trihedron.IsCorrectedFrenet,
                         bool forceApproxC1 = false,
                         bool kPart = false)

    {
        _degmax = 11;
        _segmax = 100;

        _mode = Trihedron.IsCorrectedFrenet;
        if (aMode == Trihedron.IsFrenet || aMode == Trihedron.IsCorrectedFrenet
            || aMode == Trihedron.IsDiscreteTrihedron)
        {
            _mode = aMode;
        }

        _continuity = C2;
        if (_mode == Trihedron.IsDiscreteTrihedron)
        {
            _continuity = C0;
        }

        _forceApproxC1 = forceApproxC1;

        _curIndexOfSectionEdge = 1;

        Perform(spine, profile, kPart);
    }

    //=================================================================================================

    /// <summary>Builds the pipe of <paramref name="profile"/> along <paramref name="spine"/>.</summary>
    /// <exception cref="DomainErrorException">The profile contains solids.</exception>
    public void Perform(in Wire spine,
                        in Shape profile,
                        bool kPart = false)

    {
        _ = kPart;
        _sections = null;
        _faces = null;
        _edges = null;

        _spine = spine;
        _profile = profile;

        DefineRealSegmax();

        Shape theProf;

        TrihedronLaw? tLaw = null;
        switch (_mode)
        {
            case Trihedron.IsFrenet:
                tLaw = new Frenet();
                break;
            case Trihedron.IsCorrectedFrenet:
                tLaw = new CorrectedFrenet();
                break;
            case Trihedron.IsDiscreteTrihedron:
                tLaw = new DiscreteTrihedron();
                break;
            default:
                break;
        }
        CurveAndTrihedron loc = new(tLaw!);
        _loc = new Edge3DLaw(_spine, loc);
        if (_loc.LawCount == 0)
        {
            return; // Degenerated case
        }
        _loc.TransformInG0Law(); // Set into continuity

        SectionPlacement place = new(_loc, profile);
        _trsf = place.Transformation();

        Placement loc2 = new(_trsf), loc1;
        loc1 = profile.Location();
        theProf = _profile;
        theProf.Location(loc2.Multiplied(loc1));

        // Construct First && Last Shape
        Matrix3x3 m = new();
        Vector3D v = new();
        Transform3D fila = new();
        _loc.Law(1).GetDomain(out double first, out double last);
        _loc.Law(1).D0(first, ref m, ref v);
        fila = fila.WithValues(m[1, 1],
                       m[1, 2],
                       m[1, 3],
                       v.X(),
                       m[2, 1],
                       m[2, 2],
                       m[2, 3],
                       v.Y(),
                       m[3, 1],
                       m[3, 2],
                       m[3, 3],
                       v.Z());

        fila = fila.Multiplied(_trsf);
        Placement locFirst = new(fila);
        _first = _profile;
        if (!locFirst.IsIdentity)
        {
            // myFirst.Location( LocFirst.Multiplied(myProfile.Location()) );
            _first = new Transform(_profile, fila, true).InternalShape(); // copy
        }

        RemoveLocations remLoc = new();
        remLoc.SetRemoveLevel(ShapeType.Compound);
        remLoc.Remove(_first);
        _first = remLoc.InternalGetResult();

        _loc.Law(_loc.LawCount).GetDomain(out first, out last);
        _loc.Law(_loc.LawCount).D0(last, ref m, ref v);
        //    try { // Not good, but there are no other means to test SetValues
        fila = fila.WithValues(m[1, 1],
                       m[1, 2],
                       m[1, 3],
                       v.X(),
                       m[2, 1],
                       m[2, 2],
                       m[2, 3],
                       v.Y(),
                       m[3, 1],
                       m[3, 2],
                       m[3, 3],
                       v.Z());
        fila = fila.Multiplied(_trsf);
        Placement locLast = new(fila);
        if (!_loc.IsClosed() || locFirst != locLast)
        {
            _last = _profile;
            if (!locLast.IsIdentity)
            {
                // myLast.Location(LocLast.Multiplied(myProfile.Location()) );
                _last = new Transform(_profile, fila, true).InternalShape(); // copy
            }
        }
        else
        {
            _last = _first;
        }

        remLoc.Remove(_last);
        _last = remLoc.InternalGetResult();

        _shape = MakeShape(theProf, _profile, _first, _last);
    }

    //=================================================================================================

    /// <summary>The spine (frozen).</summary>
    public Shape Spine() => Published(_spine);

    /// <summary>The profile (frozen).</summary>
    public Shape Profile() => Published(_profile);

    /// <summary>The pipe (frozen).</summary>
    public Shape Shape() => Published(_shape);

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal Shape InternalShape() => _shape;

    /// <summary>The approximation error of the swept surfaces.</summary>
    public double ErrorOnSurface() => _errorOnSurf;

    /// <summary>The shape at the start of the pipe (frozen).</summary>
    public Shape FirstShape() => Published(_first);

    /// <summary><see cref="FirstShape"/> without freezing, for ported callers.</summary>
    internal Shape InternalFirstShape() => _first;

    /// <summary>The shape at the end of the pipe (frozen).</summary>
    public Shape LastShape() => Published(_last);

    /// <summary><see cref="LastShape"/> without freezing, for ported callers.</summary>
    internal Shape InternalLastShape() => _last;

    private static Shape Published(Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="list"/> with the shapes generated from <paramref name="shape"/>, a sub-shape of
    /// the profile or the spine (frozen).
    /// </summary>
    public void Generated(in Shape shape, NCollection_List<Shape> list)
    {
        InternalGenerated(shape, list);
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            it.Value().TShape()?.FreezeGraph();
        }
    }

    /// <summary><see cref="Generated"/> without freezing, for ported callers.</summary>
    internal void InternalGenerated(in Shape shape, NCollection_List<Shape> list)
    {
        list.Clear();

        if (_genMap.IsBound(shape))
        {
            list.Assign(_genMap.Find(shape));
        }
    }

    //=================================================================================================

    /// <summary>
    /// The face generated by the edge <paramref name="eSpine"/> of the spine and the edge <paramref name="eProfile"/>
    /// of the profile (frozen; null for a degenerated profile edge).
    /// </summary>
    /// <exception cref="DomainErrorException">An edge isn't in the spine or the profile.</exception>
    public Face Face(in Edge eSpine, in Edge eProfile)
    {
        Face face = InternalFace(eSpine, eProfile);
        face.TShape()?.FreezeGraph();
        return face;
    }

    /// <summary><see cref="Face"/> without freezing, for ported callers.</summary>
    internal Face InternalFace(in Edge eSpine, in Edge eProfile)
    {
        Face theFace = new();

        if (Tool.Degenerated(eProfile))
        {
            return theFace;
        }

        int ii, ispin = 0, iprof = 0, count = 0;

        // *************************************************
        // Search if EProfile is an edge of myProfile
        // *************************************************
        iprof = FindEdge(_profile, eProfile, ref count);

        if (iprof == 0)
        {
            throw new DomainErrorException("BRepFill_Pipe::Face : Edge not in the Profile");
        }

        // *************************************************
        // Search if ESpine  is an edge of mySpine and find
        // the index of the corresponding Filler
        // *************************************************
        for (ii = 1; ii <= _loc!.LawCount && (ispin == 0); ii++)
        {
            if (eSpine.IsSame(_loc.Edge(ii)))
            {
                ispin = ii;
            }
        }

        if (ispin == 0)
        {
            throw new DomainErrorException("BRepFill_Pipe::Edge  : Edge not in the Spine");
        }

        theFace = ShapeCasts.Face(_faces!.Value(iprof, ispin));
        return theFace;
    }

    //=================================================================================================

    /// <summary>
    /// The edge generated by the edge <paramref name="eSpine"/> of the spine and the vertex
    /// <paramref name="vProfile"/> of the profile (frozen).
    /// </summary>
    /// <exception cref="DomainErrorException">The edge or the vertex isn't in the spine or the profile.</exception>
    public Edge Edge(in Edge eSpine, in Vertex vProfile)
    {
        Edge edge = InternalEdge(eSpine, vProfile);
        edge.TShape()?.FreezeGraph();
        return edge;
    }

    /// <summary><see cref="Edge"/> without freezing, for ported callers.</summary>
    internal Edge InternalEdge(in Edge eSpine, in Vertex vProfile)
    {
        int ii, ispin = 0, iprof = 0, count = 0;

        // *************************************************
        // Search if VProfile is a Vertex of myProfile
        // *************************************************
        iprof = FindVertex(_profile, vProfile, ref count);
        if (iprof == 0)
        {
            throw new DomainErrorException("BRepFill_Pipe::Edge : Vertex not in the Profile");
        }

        // *************************************************
        // Search if ESpine  is an edge of mySpine and find
        // the index of the corresponding Filler
        // *************************************************

        for (ii = 1; ii <= _loc!.LawCount && (ispin == 0); ii++)
        {
            if (eSpine.IsSame(_loc.Edge(ii)))
            {
                ispin = ii;
            }
        }

        if (ispin == 0)
        {
            throw new DomainErrorException("BRepFill_Pipe::Edge  : Edge not in the Spine");
        }

        // *************************************************
        // Generate the corresponding Shape
        // *************************************************
        Edge theEdge;
        theEdge = ShapeCasts.Edge(_edges!.Value(iprof, ispin));

        return theEdge;
    }

    //=================================================================================================

    /// <summary>The profile at the position of the spine's vertex <paramref name="vSpine"/>, as a compound (frozen).</summary>
    /// <exception cref="DomainErrorException">The vertex isn't in the spine.</exception>
    public Shape Section(in Vertex vSpine) => Published(InternalSection(vSpine));

    /// <summary><see cref="Section"/> without freezing, for ported callers.</summary>
    internal Shape InternalSection(in Vertex vSpine)
    {
        int ii, ispin = 0;

        // *************************************************
        // Search if ESpine  is an edge of mySpine and find
        // the index of the corresponding Filler
        // *************************************************

        // iterate on all the edges of mySpine
        for (ii = 1; ii <= _loc!.LawCount + 1 && (ispin == 0); ii++)
        {
            if (vSpine.IsSame(_loc.Vertex(ii)))
            {
                ispin = ii;
            }
        }

        if (ispin == 0)
        {
            throw new DomainErrorException("BRepFill_Pipe::Section  : Vertex not in the Spine");
        }

        BrepBuilder b = new();
        Compound comp = new();
        b.MakeCompound(ref comp);
        for (ii = 1; ii <= _sections!.ColLength(); ii++)
        {
            b.Add(comp, _sections.Value(ii, ispin));
        }

        return comp;
    }

    //=======================================================================
    // function : PipeLine
    // purpose  : Construct a wire by sweeping of a point
    //=======================================================================

    /// <summary>The wire swept by <paramref name="point"/> (placed with the profile) along the spine (frozen).</summary>
    public Wire PipeLine(in Point3D point)
    {
        Wire wire = InternalPipeLine(point);
        wire.TShape()?.FreezeGraph();
        return wire;
    }

    /// <summary><see cref="PipeLine"/> without freezing, for ported callers.</summary>
    internal Wire InternalPipeLine(in Point3D point)
    {
        // Positioning
        Point3D p;
        p = point;
        p = p.Transformed(_trsf);

        Vertex vertexSection = new LibraryMakeVertex(p).InternalVertex();
        ShapeLaw section = new(vertexSection);

        // Sweeping
        Sweep mkSw = new(section, _loc!, true);
        mkSw.SetForceApproxC1(_forceApproxC1);
        mkSw.Build(_reversedEdges,
                   _tapes,
                   _rails,
                   TransitionStyle.Modified,
                   _continuity,
                   ApproxStyle.Location,
                   _degmax,
                   _segmax);
        Shape localShape = mkSw.Shape();
        _errorOnSurf = mkSw.ErrorOnSurface();
        BuildHistory(mkSw, vertexSection);
        return ShapeCasts.Wire(localShape);
        // return TopoDS::Wire(MkSw.Shape());
    }

    //=================================================================================================

    private Shape MakeShape(in Shape s,
                                   in Shape originalS,
                                   in Shape firstShape,
                                   in Shape lastShape)
    {
        Shape result = new();
        BrepBuilder b = new();
        bool explode = false;
        Shape theS, theFirst, theLast;
        int initialLength = 0;
        theS = s;
        theFirst = firstShape;
        theLast = lastShape;
        if (_faces is not null)
        {
            initialLength = _faces.ColLength();
        }

        Placement backLoc = new(_trsf.Inverted());
        _ = backLoc;

        // there are two kinds of generation
        //  1. generate with S from each Filler (Vertex, Edge)
        //  2. call MakeShape recursively on the subshapes of S
        //
        // explode is True in the second case

        // create the result empty

        switch (s.ShapeType())
        {

            case ShapeType.Vertex:
            {
                Wire aW = new();
                b.MakeWire(ref aW);
                result = aW;
                break;
            }

            case global::Kerf.Topology.ShapeType.Edge:
            {
                Wire w = new();
                Shell sh = new();
                b.MakeShell(ref sh);
                result = sh;
                b.MakeWire(ref w);
                b.Add(w, s);
                w.Closed(Tool.IsClosed(s));
                theS = w;
                if (!firstShape.IsNull)
                {
                    b.MakeWire(ref w);
                    b.Add(w, firstShape);
                    w.Closed(Tool.IsClosed(firstShape));
                    theFirst = w;
                }
                if (!lastShape.IsNull)
                {
                    b.MakeWire(ref w);
                    b.Add(w, lastShape);
                    w.Closed(Tool.IsClosed(lastShape));
                    theLast = w;
                }
                result.Closed(Tool.IsClosed(result));
                break;
            }

            case ShapeType.Wire:
            {
                Shell sh = new();
                b.MakeShell(ref sh);
                result = sh;
                break;
            }

            case global::Kerf.Topology.ShapeType.Face:
            {
                Shell sh = new();
                b.MakeShell(ref sh);
                result = sh;
                explode = true;
                if (!_spine.Closed() && !theFirst.IsNull)
                {
                    b.Add(result, theFirst.Reversed());
                }
                result.Closed(Tool.IsClosed(result));
                break;
            }

            case ShapeType.Shell:
            {
                CompSolid cs = new();
                b.MakeCompSolid(ref cs);
                result = cs;
                explode = true;
                break;
            }

            case ShapeType.Solid:
            case ShapeType.CompSolid:
                throw new DomainErrorException("BRepFill_Pipe::profile contains solids");

            case ShapeType.Compound:
            {
                Compound c = new();
                b.MakeCompound(ref c);
                result = c;
                explode = true;
                break;
            }
            default:
                break;
        }

        if (explode)
        {
            // add the subshapes
            Iterator itFirst = new(), itLast = new();
            Shape first = new(), last = new();
            if (!theFirst.IsNull)
            {
                itFirst.Initialize(theFirst);
            }
            if (!theLast.IsNull)
            {
                itLast.Initialize(theLast);
            }

            Iterator it = new(s);
            Iterator itorig = new(originalS);
            for (; it.More(); it.Next(), itorig.Next())
            {
                if (!theFirst.IsNull)
                {
                    first = itFirst.Value();
                }
                if (!theLast.IsNull)
                {
                    last = itLast.Value();
                }
                if (theS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
                {
                    MakeShape(it.Value(), itorig.Value(), first, last);
                }
                else
                {
                    b.Add(result, MakeShape(it.Value(), itorig.Value(), first, last));
                }

                if (!theFirst.IsNull)
                {
                    itFirst.Next();
                }
                if (!theLast.IsNull)
                {
                    itLast.Next();
                }
            }
        }

        else
        {
            if (theS.ShapeType() == ShapeType.Vertex)
            {
                ShapeLaw section = new(ShapeCasts.Vertex(theS));
                Sweep mkSw = new(section, _loc!, true);
                mkSw.SetForceApproxC1(_forceApproxC1);
                mkSw.Build(_reversedEdges,
                           _tapes,
                           _rails,
                           TransitionStyle.Modified,
                           _continuity,
                           ApproxStyle.Location,
                           _degmax,
                           _segmax);
                result = mkSw.Shape();
                UpdateMap(originalS, result, _genMap);
                _errorOnSurf = mkSw.ErrorOnSurface();
                // the edges the vertex sweeps along the spine's edges, for Edge (OCCT left myEdges null; decision 16508)
                _edges ??= mkSw.SubShape();

                NCollection_HArray2<Shape>? sections = mkSw.Sections();

                if (sections is not null)
                {
                    int vLast = sections.UpperCol();

                    _first = sections.Value(1, 1);
                    _last = sections.Value(1, vLast);
                }

                BuildHistory(mkSw, originalS);
            }

            if (theS.ShapeType() == ShapeType.Wire)
            {
                ShapeLaw section = new(ShapeCasts.Wire(theS));
                Sweep mkSw = new(section, _loc!, true);
                mkSw.SetBounds(ShapeCasts.Wire(theFirst), ShapeCasts.Wire(theLast));
                mkSw.SetForceApproxC1(_forceApproxC1);
                mkSw.Build(_reversedEdges,
                           _tapes,
                           _rails,
                           TransitionStyle.Modified,
                           _continuity,
                           ApproxStyle.Location,
                           _degmax,
                           _segmax);
                result = mkSw.Shape();
                _errorOnSurf = mkSw.ErrorOnSurface();

                // Labeling of elements
                if (_sections is null)
                {
                    _faces = mkSw.SubShape();
                    _sections = mkSw.Sections();
                    _edges = mkSw.InterFaces();
                }
                else
                {
                    NCollection_HArray2<Shape> aux, somme;
                    int length;
                    int ii, jj, kk;

                    aux = mkSw.SubShape()!;
                    length = aux.ColLength() + _faces!.ColLength();
                    somme = new NCollection_HArray2<Shape>(1, length, 1, aux.RowLength());
                    for (jj = 1; jj <= _faces.RowLength(); jj++)
                    {
                        for (ii = 1; ii <= _faces.ColLength(); ii++)
                        {
                            somme.SetValue(ii, jj, _faces.Value(ii, jj));
                        }

                        for (kk = 1, ii = _faces.ColLength() + 1; kk <= aux.ColLength(); kk++, ii++)
                        {
                            somme.SetValue(ii, jj, aux.Value(kk, jj));
                        }
                    }
                    _faces = somme;

                    aux = mkSw.Sections()!;
                    length = aux.ColLength() + _sections.ColLength();
                    somme = new NCollection_HArray2<Shape>(1, length, 1, aux.RowLength());
                    for (jj = 1; jj <= _sections.RowLength(); jj++)
                    {
                        for (ii = 1; ii <= _sections.ColLength(); ii++)
                        {
                            somme.SetValue(ii, jj, _sections.Value(ii, jj));
                        }

                        _curIndexOfSectionEdge = _sections.ColLength() + 1;

                        for (kk = 1, ii = _sections.ColLength() + 1; kk <= aux.ColLength(); kk++, ii++)
                        {
                            somme.SetValue(ii, jj, aux.Value(kk, jj));
                        }
                    }
                    _sections = somme;

                    aux = mkSw.InterFaces()!;
                    length = aux.ColLength() + _edges!.ColLength();
                    somme = new NCollection_HArray2<Shape>(1, length, 1, aux.RowLength());
                    for (jj = 1; jj <= _edges.RowLength(); jj++)
                    {
                        for (ii = 1; ii <= _edges.ColLength(); ii++)
                        {
                            somme.SetValue(ii, jj, _edges.Value(ii, jj));
                        }

                        for (kk = 1, ii = _edges.ColLength() + 1; kk <= aux.ColLength(); kk++, ii++)
                        {
                            somme.SetValue(ii, jj, aux.Value(kk, jj));
                        }
                    }

                    _edges = somme;
                }

                BuildHistory(mkSw, originalS);
            }
        }

        if (theS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
        {
            int ii, jj;
            // jgv
            Explorer explo = new(result, global::Kerf.Topology.ShapeType.Face);
            for (; explo.More(); explo.Next())
            {
                Shape face = explo.Current();
                RebuildTopOrBottomFace(face.Reversed(), true); // top face was reversed
            }
            /////
            Face f;
            for (ii = initialLength + 1; ii <= _faces!.ColLength(); ii++)
            {
                for (jj = 1; jj <= _faces.RowLength(); jj++)
                {
                    if (_faces.Value(ii, jj).ShapeType() == global::Kerf.Topology.ShapeType.Face)
                    {
                        f = ShapeCasts.Face(_faces.Value(ii, jj));
                        if (!f.IsNull)
                        {
                            b.Add(result, f);
                        }
                    }
                }
            }

            if (!_spine.Closed())
            {
                // if Spine is not closed
                // add the last face of the solid

                // jgv
                RebuildTopOrBottomFace(theLast, false); // bottom face
                /////
                b.Add(result, ShapeCasts.Face(theLast));
            }

            Solid solid = new();
            BrepBuilder bs = new();
            bs.MakeSolid(ref solid);

            result.Closed(true);
            bs.Add(solid, ShapeCasts.Shell(result));

            SolidClassifier sc = new(solid);
            sc.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
            if (sc.State() == In)
            {
                bs.MakeSolid(ref solid);
                Shape localShape = result.Reversed();
                bs.Add(solid, ShapeCasts.Shell(localShape));
                //      BS.Add(solid,TopoDS::Shell(result.Reversed()));
            }
            UpdateMap(originalS, solid, _genMap);
            return solid;
        }
        else
        {
            return result;
        }
    }

    //============================================================================
    // function : FindEdge
    // purpose  : Find the number of edge corresponding to the edge of the profile.
    //============================================================================

    private int FindEdge(in Shape s, in Edge e, ref int initialLength)
    {
        int result = 0;

        switch (s.ShapeType())
        {

            case global::Kerf.Topology.ShapeType.Edge:
            {
                initialLength++;
                if (s.IsSame(e))
                {
                    result = initialLength;
                }
                break;
            }

            case ShapeType.Wire:
            {
                ShapeLaw section = new(ShapeCasts.Wire(s), false);
                int nbLaw = section.LawCount;

                for (int ii = 1; (ii <= nbLaw) && (result == 0); ii++)
                {
                    if (e.IsSame(section.Edge(ii)))
                    {
                        result = initialLength + ii;
                    }
                }
                initialLength += nbLaw;
                break;
            }

            case global::Kerf.Topology.ShapeType.Face:
            case ShapeType.Shell:
            case ShapeType.Compound:
            {
                for (Iterator it = new(s); it.More() && (result == 0); it.Next())
                {
                    result = FindEdge(it.Value(), e, ref initialLength);
                }
                break;
            }

            case ShapeType.Solid:
            case ShapeType.CompSolid:
                throw new DomainErrorException("BRepFill_Pipe::SOLID or COMPSOLID");
            default:
                break;
        }

        return result;
    }

    //=======================================================================
    // function : FindVertex
    // purpose  : Find the number of edge corresponding to an edge of the profile.
    //=======================================================================

    private int FindVertex(in Shape s, in Vertex v, ref int initialLength)
    {
        int result = 0;

        switch (s.ShapeType())
        {
            case ShapeType.Vertex:
            {
                initialLength++;
                if (s.IsSame(v))
                {
                    result = initialLength;
                }
                break;
            }

            case global::Kerf.Topology.ShapeType.Edge:
            {
                Exploration.Vertices(ShapeCasts.Edge(s), out Vertex vf, out Vertex vl);
                if (s.Orientation() == Reversed)
                {
                    Vertex aux;
                    aux = vf;
                    vf = vl;
                    vl = aux;
                }
                if (vf.IsSame(v))
                {
                    result = initialLength + 1;
                }
                else if (vl.IsSame(v))
                {
                    result = initialLength + 2;
                }
                initialLength += 2;
                break;
            }

            case ShapeType.Wire:
            {
                int ii = initialLength + 1;
                ShapeLaw section = new(ShapeCasts.Wire(s), false);
                initialLength += section.LawCount + 1;

                for (; (ii <= initialLength) && (result == 0); ii++)
                {
                    if (v.IsSame(section.Vertex(ii, 0.0)))
                    {
                        result = ii;
                    }
                }
                break;
            }

            case global::Kerf.Topology.ShapeType.Face:
            case ShapeType.Shell:
            case ShapeType.Compound:
            {
                for (Iterator it = new(s); it.More() && (result == 0); it.Next())
                {
                    result = FindVertex(it.Value(), v, ref initialLength);
                }
                break;
            }

            case ShapeType.Solid:
            case ShapeType.CompSolid:
                throw new DomainErrorException("BRepFill_Pipe::SOLID or COMPSOLID");
            default:
                break;
        }

        return result;
    }

    //=======================================================================
    // function : DefineRealSegmax
    // purpose  : Defines the real number of segments
    //           required in the case of bspline spine
    //=======================================================================

    private void DefineRealSegmax()
    {
        int realSegmax = 0;

        Iterator iter = new(_spine);
        for (; iter.More(); iter.Next())
        {
            Edge e = ShapeCasts.Edge(iter.Value());
            Curve3D? c = Tool.Curve(e, out double first, out double last);
            if (c is null)
            {
                continue;
            }
            while (c.GetType() == typeof(TrimmedCurve3D)
                   || c.GetType() == typeof(OffsetCurve3D))
            {
                if (c.GetType() == typeof(TrimmedCurve3D))
                {
                    c = ((TrimmedCurve3D)c).BasisCurve();
                }
                if (c.GetType() == typeof(OffsetCurve3D))
                {
                    c = ((OffsetCurve3D)c).BasisCurve();
                }
            }
            if (c.GetType() == typeof(BSplineCurve3D))
            {
                BSplineCurve3D bc = (BSplineCurve3D)c;
                int nbKnots = bc.KnotCount;
                int realNbKnots = nbKnots;
                if (first > bc.FirstParameter())
                {
                    bc.LocateU(first, global::Kerf.Engine.Foundation.Precision.PConfusion(), out int i1, out _);
                    realNbKnots -= i1 - 1;
                }
                if (last < bc.LastParameter())
                {
                    bc.LocateU(last, global::Kerf.Engine.Foundation.Precision.PConfusion(), out _, out int i2);
                    realNbKnots -= nbKnots - i2;
                }
                realSegmax += realNbKnots - 1;
            }
        }

        if (_segmax < realSegmax)
        {
            _segmax = realSegmax;
        }
    }

    //=======================================================================
    // function : RebuildTopOrBottomFace
    // purpose  : Correct orientation of v-iso edges
    //           according to new 3d and 2d curves taken from swept surfaces
    //=======================================================================

    private void RebuildTopOrBottomFace(in Shape aFace, bool isTop)
    {
        int indexOfSection = (isTop) ? 1 : _sections!.RowLength();

        int ii;
        BrepBuilder bb = new();
        Iterator itf = new(aFace);
        for (; itf.More(); itf.Next())
        {
            Shape wire = itf.Value();
            NCollection_Sequence<Shape> initEdges = new();
            NCollection_Sequence<Shape> resEdges = new();
            Iterator itw = new(wire);
            for (; itw.More(); itw.Next())
            {
                Shape edge = itw.Value();
                for (ii = _curIndexOfSectionEdge; ii <= _sections!.ColLength(); ii++)
                {
                    Shape visoEdge = _sections.Value(ii, indexOfSection);
                    if (edge.IsSame(visoEdge))
                    {
                        initEdges.Append(edge);
                        resEdges.Append(visoEdge);
                        break;
                    }
                }
            }
            // the writes below reach the profile's own face when the pipe shares it: MakePipe's working overlay takes them
            wire.Free(true);
            for (ii = 1; ii <= initEdges.Length(); ii++)
            {
                bb.Remove(wire, initEdges[ii]);
                UpdateTolFromTopOrBottomPCurve(ShapeCasts.Face(aFace), ShapeCasts.Edge(resEdges[ii]));
                bb.Add(wire, resEdges[ii]);
            }
        }
    }

    //=======================================================================
    // function : BuildHistory
    // purpose  : Builds history for edges and vertices
    //           of section and path
    //=======================================================================
    private void BuildHistory(Sweep sweep, in Shape section)
    {
        // Filling of <myGenMap>
        NCollection_HArray2<Shape> uEdges = sweep.InterFaces()!;

        int inde;

        if (section.ShapeType() == ShapeType.Wire)
        {
            Wire aSection = ShapeCasts.Wire(section);
            WireExplorer wexp_sec = new(aSection);
            for (inde = 0; wexp_sec.More(); wexp_sec.Next())
            {
                inde++;
                Edge edge = ShapeCasts.Edge(wexp_sec.Current());
                if (Tool.Degenerated(edge))
                {
                    continue;
                }
                if (_genMap.IsBound(edge))
                {
                    continue;
                }

                Vertex[] vertex = new Vertex[2];
                Exploration.Vertices(edge, out vertex[0], out vertex[1]);

                // For an edge generated shape is a "tape" -
                // a shell usually containing this edge and
                // passing from beginning of path to its end
                Shell tape = ShapeCasts.Shell(sweep.Tape(inde));

                // Processing of vertices of <anEdge>
                // We should choose right index in <anUEdges>
                // for each vertex of edge
                int[] uIndex = new int[2];
                uIndex[0] = inde;
                uIndex[1] = inde + 1;

                if (edge.Orientation() == Reversed)
                {
                    int tmp = uIndex[0];
                    uIndex[0] = uIndex[1];
                    uIndex[1] = tmp;
                }

                for (int kk = 0; kk < 2; kk++)
                {
                    if (_genMap.IsBound(vertex[kk]))
                    {
                        continue;
                    }

                    // Assemble the list of edges ("rail" along the path)
                    NCollection_List<Shape> elist = _genMap.Bound(vertex[kk], new NCollection_List<Shape>());
                    int jj;
                    for (jj = 1; jj <= uEdges.UpperCol(); jj++)
                    {
                        Shape uedge = uEdges.Value(uIndex[kk], jj);
                        if (!uedge.IsNull)
                        {
                            elist.Append(uedge);
                        }
                    }
                } // for (int kk = 0; kk < 2; kk++)

                NCollection_List<Shape> flist = _genMap.Bound(edge, new NCollection_List<Shape>());
                Iterator itsh = new(tape);
                for (; itsh.More(); itsh.Next())
                {
                    flist.Append(itsh.Value());
                }
            } // for (inde = 0; wexp_sec.More(); wexp_sec.Next())
        } // if (theSection.ShapeType() == TopAbs_WIRE)

        // For subshapes of spine
        NCollection_HArray2<Shape> faces = sweep.SubShape()!;
        NCollection_HArray2<Shape> vEdges = sweep.Sections()!;

        WireExplorer wexp = new(_spine);
        inde = 0;
        bool toExit = false;
        for (;;)
        {
            if (!wexp.More())
            {
                toExit = true;
            }

            inde++;

            if (!toExit)
            {
                Edge edgeOfSpine = wexp.Current();
                for (int i = 1; i <= faces.UpperRow(); i++)
                {
                    Shape face = faces.Value(i, inde);
                    UpdateMap(edgeOfSpine, face, _genMap);
                }
            }

            Vertex vertexOfSpine = wexp.CurrentVertex();
            for (int i = 1; i <= vEdges.UpperRow(); i++)
            {
                Shape vedge = vEdges.Value(i, inde);
                UpdateMap(vertexOfSpine, vedge, _genMap);
            }

            if (toExit)
            {
                break;
            }

            if (wexp.More())
            {
                wexp.Next();
            }
        }
    }
}
