// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_ThruSections.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_ThruSections.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.ParametrizationType;
using static Kerf.Engine.Construction.Filling.ThruSectionErrorStatus;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds a loft: a shell or a solid passing through a set of sections (wires; a first or last section may be a
/// point). The loft is ruled (planar, cylindrical, conical or ruled faces between consecutive sections) or smoothed
/// (B-spline faces approximated through all the sections).
/// </summary>
/// <remarks>
/// <para>
/// OCCT's loft writes into the sections it is given: pcurves, ranges and flags into their edges, tolerances into their
/// vertices (by default, <see cref="IsMutableInput"/> true). The sections a caller gives (whose TShapes are frozen)
/// are written into in a working overlay at <see cref="Build"/> (unit 10H3); the result holds the caller's sub-shapes
/// wherever the loft left them unchanged, and the history (<see cref="MakeShape.Generated"/>,
/// <see cref="GeneratedFace"/>) answers for the caller's edges and vertices directly. The caller's shapes are never
/// written.
/// </para>
/// <para>
/// The result, <see cref="FirstShape"/>, <see cref="LastShape"/>, <see cref="GeneratedFace"/> and the history are
/// frozen when returned.
/// </para>
/// </remarks>
internal class ThruSections : MakeShape
{
    private readonly NCollection_List<Shape> _inputWires = new(); //!< List of input wires
    private NCollection_Sequence<Shape> _wires = new();           //!< Working wires
    private readonly NCollection_Sequence<Shape> _sections = new(); // OCCT.NET: the sections as added
    private readonly Dictionary<Shape, NCollection_List<int>> _edgeNewIndices = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, int> _vertexIndex = new(ShapeHasher.Instance);
    private int _nbEdgesInSection;
    private bool _isSolid;
    private bool _isRuled;
    private bool _wCheck;
    private double _pres3d;
    private Face _first = new();
    private Face _last = new();
    private bool _degen1;
    private bool _degen2;
    private readonly Dictionary<Shape, Shape> _edgeFace = new(ShapeHasher.Instance);
    private Continuity _continuity;
    private global::Kerf.Geometry.ParametrizationType _paramType;
    private int _degMax;
    private readonly double[] _critWeights = new double[3];
    private bool _useSmoothing;
    private bool _mutableInput;
    private Generator? _bfGenerator;
    private ThruSectionErrorStatus _status;
    private WorkingOverlay? _overlay; // Kerf: the overlay the sections were written in, finished (unit 10H3)

    //=======================================================================
    // function : PreciseUpar
    // purpose  : pins the u-parameter of surface close to U-knot
    //           to this U-knot
    //=======================================================================
    private static double PreciseUpar(double anUpar, global::Kerf.Geometry.BSplineSurface aSurface)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        aSurface.LocateU(anUpar, tol, out int i1, out int i2);
        double u1 = aSurface.UKnot(i1);
        double u2 = aSurface.UKnot(i2);

        double newU = anUpar;

        newU = (anUpar - u1 < u2 - anUpar) ? u1 : u2;
        return newU;
    }

    //=======================================================================
    // function :  PerformPlan
    // purpose  : Construct a plane of filling if exists
    //=======================================================================

    private static bool PerformPlan(in Wire w, double presPln, ref Face face)
    {
        bool isDegen = true;
        Iterator iter = new(w);
        for (; iter.More(); iter.Next())
        {
            Edge edge = ShapeCasts.Edge(iter.Value());
            if (!Tool.Degenerated(edge))
            {
                isDegen = false;
            }
        }
        if (isDegen)
        {
            return true;
        }

        bool ok = false;
        if (!w.IsNull)
        {
            FindPlane searcher = new(w, presPln);
            if (searcher.Found())
            {
                face = new MakeFace(searcher.Plane()!, w).InternalFace();
                ok = true;
            }
            else // try to find another surface
            {
                MakeFace mf = new(w);
                if (mf.IsDone)
                {
                    face = mf.InternalFace();
                    ok = true;
                }
            }
        }

        return ok;
    }

    //=============================================================================
    // function :  IsSameOriented
    // purpose  : Checks whether aFace is oriented to the same side as aShell or not
    //=============================================================================

    private static bool IsSameOriented(in Shape aFace, in Shape aShell)
    {
        Explorer explo = new(aFace, ShapeType.Edge);
        Shape edge = explo.Current();
        Orientation or1 = edge.Orientation;

        IndexedMap<Shape, NCollection_List<Shape>> eFmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aShell, ShapeType.Edge, ShapeType.Face, eFmap);

        Shape adjacentFace = eFmap.GetValue(edge).First();
        Shape theEdge = new();
        for (explo.Init(adjacentFace, ShapeType.Edge); explo.More(); explo.Next())
        {
            theEdge = explo.Current();
            if (theEdge.IsSame(edge))
            {
                break;
            }
        }

        Orientation or2 = theEdge.Orientation;
        return or1 != or2;
    }

    //=================================================================================================

    private static Solid MakeSolid(ref Shell shell,
                                          in Wire wire1,
                                          in Wire wire2,
                                          double presPln,
                                          ref Face face1,
                                          ref Face face2)
    {
        if (shell.IsNull)
        {
            throw new NotDoneException("Thrusections is not build");
        }
        bool b = shell.ClosedFlag;
        BrepBuilder bb = new();

        if (!b)
        {
            // It is necessary to close the extremities
            b = PerformPlan(wire1, presPln, ref face1);
            if (b)
            {
                b = PerformPlan(wire2, presPln, ref face2);
                if (b)
                {
                    if (!face1.IsNull && !IsSameOriented(face1, shell))
                    {
                        face1.Reverse();
                    }
                    if (!face2.IsNull && !IsSameOriented(face2, shell))
                    {
                        face2.Reverse();
                    }

                    if (!face1.IsNull)
                    {
                        bb.Add(shell, face1);
                    }
                    if (!face2.IsNull)
                    {
                        bb.Add(shell, face2);
                    }

                    shell.Closed(true);
                }
            }
        }

        Solid solid = new();
        bb.MakeSolid(ref solid);
        bb.Add(solid, shell);

        // verify the orientation the solid
        SolidClassifier clas3d = new(solid);
        clas3d.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
        if (clas3d.State() == In)
        {
            bb.MakeSolid(ref solid);
            Shape localShape = shell.Reversed();
            bb.Add(solid, ShapeCasts.Shell(localShape));
        }

        solid.Closed(true);
        return solid;
    }

    //=================================================================================================

    /// <summary>
    /// A loft: a solid (<paramref name="isSolid"/>) or a shell, ruled (<paramref name="ruled"/>) or smoothed, with
    /// the tolerance <paramref name="pres3d"/> of the approximation and of the end planes.
    /// </summary>
    public ThruSections(bool isSolid = false, bool ruled = false, double pres3d = 1.0e-06)
    {
        _nbEdgesInSection = 0;
        _isSolid = isSolid;
        _isRuled = ruled;
        _pres3d = pres3d;
        _degen1 = false;
        _degen2 = false;
        _wCheck = true;
        _mutableInput = true;
        //----------------------------
        _paramType = ChordLength;
        _degMax = 8;
        _continuity = C2;
        _critWeights[0] = .4;
        _critWeights[1] = .2;
        _critWeights[2] = .4;
        _useSmoothing = false;
        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>
    /// Resets the options (the sections stay). As in OCCT, the maximal degree becomes 6, where the constructor sets 8.
    /// </summary>
    public void Init(bool isSolid = false, bool ruled = false, double pres3d = 1.0e-06)
    {
        _isSolid = isSolid;
        _isRuled = ruled;
        _pres3d = pres3d;
        _wCheck = true;
        _mutableInput = true;
        //----------------------------
        _paramType = ChordLength;
        _degMax = 6; // OCCT: 8 in the constructor (occt-issues.md)
        _continuity = C2;
        _critWeights[0] = .4;
        _critWeights[1] = .2;
        _critWeights[2] = .4;
        _useSmoothing = false;
        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>Adds a section.</summary>
    public void AddWire(in Wire wire)
    {
        _sections.Append(wire);
        _inputWires.Append(wire);
    }

    //=================================================================================================

    /// <summary>Adds a point section (first or last only), as a closed wire of one degenerated edge.</summary>
    public void AddVertex(in Vertex aVertex)
    {
        BrepBuilder bb = new();

        Edge degEdge = new();
        bb.MakeEdge(ref degEdge);
        bb.Add(degEdge, aVertex.Oriented(Forward));
        bb.Add(degEdge, aVertex.Oriented(Reversed));
        bb.Degenerated(degEdge, true);

        Wire degWire = new();
        bb.MakeWire(ref degWire);
        bb.Add(degWire, degEdge);
        degWire.Closed(true);
        if (aVertex.TShape()!.IsFrozen)
        {
            // Kerf: the point section of a caller's vertex is the caller's too (unit 10H3): frozen, so the loft's
            // writes into its edge go to the working overlay and the section Wires() hands out stays as it was given
            degWire.TShape()!.FreezeGraph();
        }

        _sections.Append(degWire);
        _inputWires.Append(degWire);
    }

    //=================================================================================================

    /// <summary>
    /// Sets whether the sections are made compatible (same number of edges, origins and orientations chosen to avoid
    /// twisting) by BRepFill_CompatibleWires; true by default.
    /// </summary>
    public void CheckCompatibility(bool check = true)
    {
        _wCheck = check;
    }

    //=================================================================================================

    /// <summary>Builds the loft.</summary>
    public override void Build(ProgressRange range = default)
    {
        // Kerf: the working sections are the caller's, written into in a working overlay (unit 10H3)
        _overlay = null;
        _wires = new NCollection_Sequence<Shape>(_sections);
        _first = new Face();
        _last = new Face();
        _shape = new Shape();

        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(new Shape());
        try
        {
            BuildCore();
        }
        finally
        {
            if (overlay is not null)
            {
                // the result holds the caller's sub-shapes the loft left unchanged; so do the working sections,
                // whose edges keep sharing their vertices with the result's (Generated reads them)
                List<Shape> others = [_first, _last];
                for (int k = 1; k <= _wires.Length(); k++)
                {
                    others.Add(_wires.Value(k));
                }
                _shape = overlay.Finish(_shape, others);
                _first = ShapeCasts.Face(overlay.Final(_first));
                _last = ShapeCasts.Face(overlay.Final(_last));
                _overlay = overlay;
            }
        }
    }

    private void BuildCore()
    {
        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Done;
        _bfGenerator = null;
        // Check set of section for right configuration of punctual sections
        int i;
        Explorer explo = new();
        for (i = 2; i <= _wires.Length() - 1; i++)
        {
            bool wdeg = true;
            for (explo.Init(_wires.Value(i), ShapeType.Edge); explo.More(); explo.Next())
            {
                Edge edge = ShapeCasts.Edge(explo.Current());
                wdeg = wdeg && (Tool.Degenerated(edge));
            }
            if (wdeg)
            {
                _status = ThruSectionErrorStatus.ThruSectionErrorStatus_WrongUsage;
                return;
            }
        }
        if (_wires.Length() <= 2)
        {
            bool wdeg = true;
            for (i = 1; i <= _wires.Length(); i++)
            {
                for (explo.Init(_wires.Value(i), ShapeType.Edge); explo.More(); explo.Next())
                {
                    Edge edge = ShapeCasts.Edge(explo.Current());
                    wdeg = wdeg && (Tool.Degenerated(edge));
                }
            }
            if (wdeg)
            {
                _status = ThruSectionErrorStatus.ThruSectionErrorStatus_WrongUsage;
                return;
            }
        }

        _nbEdgesInSection = 0;

        if (_wCheck)
        {
            // compute origin and orientation on wires to avoid twisted results
            // and update wires to have same number of edges

            // use BRepFill_CompatibleWires
            NCollection_Sequence<Shape> workingSections = new();
            workingSections.Clear();
            // (OCCT's WorkingMap, a copy of Georges' generated map never read, is left out)

            // Calculate the working sections
            CompatibleWires georges = new(_wires);
            georges.Perform();
            if (georges.IsDone)
            {
                workingSections = new NCollection_Sequence<Shape>(georges.InternalShape());
                _degen1 = georges.IsDegeneratedFirstSection;
                _degen2 = georges.IsDegeneratedLastSection;
                // For each sub-edge of each section
                // we save its splits
                int indFirstSec = 1;
                if (georges.IsDegeneratedFirstSection)
                {
                    indFirstSec = 2;
                }
                Wire workingSection = ShapeCasts.Wire(workingSections.Value(indFirstSec));
                _nbEdgesInSection += workingSection.ChildCount;
                for (int ii = 1; ii <= _wires.Length(); ii++)
                {
                    Iterator itw = new(_wires.Value(ii));
                    for (; itw.More(); itw.Next())
                    {
                        Edge edge = ShapeCasts.Edge(itw.Value());
                        int sign = 1;
                        Exploration.Vertices(edge, out Vertex vfirst, out Vertex vlast);
                        NCollection_List<Shape> newEdges = georges.InternalGeneratedShapes(edge);
                        NCollection_List<int> iList = new();
                        workingSection = ShapeCasts.Wire(workingSections.Value(ii));
                        int nbNewEdges = newEdges.Extent();
                        NCollection_List<Shape>.Iterator itl = new(newEdges);
                        for (int kk = 1; itl.More(); itl.Next(), kk++)
                        {
                            Edge newEdge = ShapeCasts.Edge(itl.Value());
                            int inde = 1;
                            WireExplorer wexp = new(workingSection);
                            for (; wexp.More(); wexp.Next(), inde++)
                            {
                                Shape workingEdge = wexp.Current();
                                if (workingEdge.IsSame(newEdge))
                                {
                                    sign = (workingEdge.Orientation == Forward) ? 1 : -1;
                                    break;
                                }
                            }
                            iList.Append(inde);
                            if (kk == 1 || kk == nbNewEdges)
                            {
                                // For each sub-vertex of each section
                                // we save its index of new edge
                                Exploration.Vertices(newEdge, out Vertex newVfirst, out Vertex newVlast);
                                if (newVfirst.IsSame(vfirst) && !_vertexIndex.ContainsKey(vfirst))
                                {
                                    _vertexIndex[vfirst] = sign * inde;
                                }
                                if (newVlast.IsSame(vlast) && !_vertexIndex.ContainsKey(vlast))
                                {
                                    _vertexIndex[vlast] = sign * (-inde);
                                }
                            }
                        }
                        _edgeNewIndices[edge] = iList;
                    }
                }
            }
            else
            {
                _status = georges.GetStatus();
                NotDone();
                return;
            }

            _wires = workingSections;
        } // if (myWCheck)
        else // no check
        {
            Edge edge = new();
            for (int ii = 1; ii <= _wires.Length(); ii++)
            {
                Explorer Explo = new(_wires.Value(ii), ShapeType.Edge);
                int inde = 1;
                for (; Explo.More(); Explo.Next(), inde++)
                {
                    edge = ShapeCasts.Edge(Explo.Current());
                    NCollection_List<int> iList = new();
                    iList.Append(inde);
                    _edgeNewIndices[edge] = iList;
                    Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                    if (!_vertexIndex.ContainsKey(v1))
                    {
                        _vertexIndex[v1] = inde;
                    }
                    if (!_vertexIndex.ContainsKey(v2))
                    {
                        _vertexIndex[v2] = -inde;
                    }
                }
                inde--;
                if (inde > _nbEdgesInSection)
                {
                    _nbEdgesInSection = inde;
                }
                if (inde == 1 && Tool.Degenerated(edge))
                {
                    if (ii == 1)
                    {
                        _degen1 = true;
                    }
                    else
                    {
                        _degen2 = true;
                    }
                }
            }
        }

        try
        {
            // Calculate the resulting shape
            if (_wires.Length() == 2 || _isRuled)
            {
                // create a ruled shell
                CreateRuled();
            }
            else
            {
                // create a smoothed shell
                CreateSmoothed();
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            NotDone();
            return;
        }

        if (_status != ThruSectionErrorStatus.ThruSectionErrorStatus_Done)
        {
            NotDone();
            return;
        }
        // Encode the Regularities
        BrepLibrary.EncodeRegularity(_shape);
    }

    //=================================================================================================

    private void CreateRuled()
    {
        int nbSects = _wires.Length();
        _bfGenerator = new Generator();
        _bfGenerator.SetMutableInput(IsMutableInput);
        //  for (int i=1; i<=nbSects; i++) {
        int i;
        for (i = 1; i <= nbSects; i++)
        {
            _bfGenerator.AddWire(ShapeCasts.Wire(_wires.Value(i)));
        }
        _bfGenerator.Perform();
        ThruSectionErrorStatus status = _bfGenerator.GetStatus();
        if (status != ThruSectionErrorStatus.ThruSectionErrorStatus_Done)
        {
            _status = status;
            return;
        }
        Shell shell = _bfGenerator.InternalShell();

        if (_isSolid)
        {

            // check if the first wire is the same as the last
            bool vClosed = (_wires.Value(1).IsSame(_wires.Value(nbSects)));

            if (vClosed)
            {

                Solid solid = new();
                BrepBuilder b = new();
                b.MakeSolid(ref solid);
                b.Add(solid, shell);

                // verify the orientation of the solid
                SolidClassifier clas3d = new(solid);
                clas3d.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
                if (clas3d.State() == In)
                {
                    b.MakeSolid(ref solid);
                    Shape localShape = shell.Reversed();
                    b.Add(solid, ShapeCasts.Shell(localShape));
                }
                _shape = solid;
            }

            else
            {
                // myBFGenerator stores the same 'myWires'
                Wire wire1 = ShapeCasts.Wire(_bfGenerator.InternalResultShape(_wires.First()));
                Wire wire2 = ShapeCasts.Wire(_bfGenerator.InternalResultShape(_wires.Last()));

                _shape = MakeSolid(ref shell, wire1, wire2, _pres3d, ref _first, ref _last);
            }

            Done();
        }

        else
        {
            _shape = shell;
            Done();
        }

        // history
        WireExplorer exp1 = new(), exp2 = new();
        IndexedMap<Shape, NCollection_List<Shape>> m = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shell, ShapeType.Edge, ShapeType.Face, m);
        NCollection_List<Shape>.Iterator it = new();

        IndexedMap<Shape, NCollection_List<Shape>> mv = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shell, ShapeType.Vertex, ShapeType.Face, mv);

        for (i = 1; i <= nbSects - 1; i++)
        {

            Wire wire1 = ShapeCasts.Wire(_wires.Value(i));
            Wire wire2 = ShapeCasts.Wire(_wires.Value(i + 1));

            exp1.Init(wire1);
            exp2.Init(wire2);

            bool tantque = exp1.More() && exp2.More();

            while (tantque)
            {

                Shape edge1 = exp1.Current();
                Shape edge2 = exp2.Current();
                bool degen1 = Tool.Degenerated(exp1.Current());
                bool degen2 = Tool.Degenerated(exp2.Current());

                HashSet<Shape> mapFaces = new(ShapeHasher.Instance);
                if (degen2)
                {
                    Vertex vdegen =
                      ShapeCasts.Vertex(_bfGenerator.InternalResultShape(Exploration.FirstVertex(ShapeCasts.Edge(edge2))));
                    for (it.Initialize(mv.GetValue(vdegen)); it.More(); it.Next())
                    {
                        mapFaces.Add(it.Value());
                    }
                }
                else
                {
                    for (it.Initialize(m.GetValue(_bfGenerator.InternalResultShape(edge2))); it.More(); it.Next())
                    {
                        mapFaces.Add(it.Value());
                    }
                }

                if (degen1)
                {
                    Vertex vdegen =
                      ShapeCasts.Vertex(_bfGenerator.InternalResultShape(Exploration.FirstVertex(ShapeCasts.Edge(edge1))));
                    for (it.Initialize(mv.GetValue(vdegen)); it.More(); it.Next())
                    {
                        Shape face = it.Value();
                        if (mapFaces.Contains(face))
                        {
                            _edgeFace[edge1] = face;
                            break;
                        }
                    }
                }
                else
                {
                    for (it.Initialize(m.GetValue(_bfGenerator.InternalResultShape(edge1))); it.More(); it.Next())
                    {
                        Shape face = it.Value();
                        if (mapFaces.Contains(face))
                        {
                            _edgeFace[edge1] = face;
                            break;
                        }
                    }
                }

                if (!degen1)
                {
                    exp1.Next();
                }
                if (!degen2)
                {
                    exp2.Next();
                }

                tantque = exp1.More() && exp2.More();
                if (degen1)
                {
                    tantque = exp2.More();
                }
                if (degen2)
                {
                    tantque = exp1.More();
                }
            }
        }
    }

    //=================================================================================================


    private void CreateSmoothed()
    {
        // initialisation
        int nbSects = _wires.Length();
        WireExplorer exp = new();

        bool w1Point = true;
        // check if the first wire is punctual
        for (exp.Init(ShapeCasts.Wire(_wires.Value(1))); exp.More(); exp.Next())
        {
            w1Point = w1Point && (Tool.Degenerated(exp.Current()));
        }

        bool w2Point = true;
        // check if the last wire is punctual
        for (exp.Init(ShapeCasts.Wire(_wires.Value(nbSects))); exp.More(); exp.Next())
        {
            w2Point = w2Point && (Tool.Degenerated(exp.Current()));
        }

        bool vClosed = false;
        // check if the first wire is the same as last
        if (_wires.Value(1).IsSame(_wires.Value(_wires.Length())))
        {
            vClosed = true;
        }

        // find the dimension
        int nbEdges = 0;
        if (!w1Point)
        {
            for (exp.Init(ShapeCasts.Wire(_wires.Value(1))); exp.More(); exp.Next())
            {
                nbEdges++;
            }
        }
        else
        {
            for (exp.Init(ShapeCasts.Wire(_wires.Value(2))); exp.More(); exp.Next())
            {
                nbEdges++;
            }
        }

        // recover the shapes
        bool uClosed = true;
        NCollection_Array1<Shape> shapes = new(1, nbSects * nbEdges);
        int nb = 0, i, j;

        for (i = 1; i <= nbSects; i++)
        {
            Wire wire = ShapeCasts.Wire(_wires.Value(i));
            if (!wire.ClosedFlag)
            {
                // check if the vertices are the same
                Exploration.Vertices(wire, out Vertex v1, out Vertex v2);
                if (!v1.IsSame(v2))
                {
                    uClosed = false;
                }
            }
            if ((i == 1 && w1Point) || (i == nbSects && w2Point))
            {
                // if the wire is punctual
                exp.Init(ShapeCasts.Wire(wire));
                for (j = 1; j <= nbEdges; j++)
                {
                    nb++;
                    shapes[nb] = exp.Current();
                }
            }
            else
            {
                // otherwise
                for (exp.Init(ShapeCasts.Wire(wire)); exp.More(); exp.Next())
                {
                    nb++;
                    shapes[nb] = exp.Current();
                }
            }
        }

        // create the new surface
        Shell shell = new();
        Face face = new();
        Wire w = new();
        Edge edge, edge1 = new(), edge2 = new(), edge3 = new(), edge4 = new(), couture = new();
        NCollection_Array1<Shape> vcouture = new(1, nbEdges);

        BrepBuilder b = new();
        b.MakeShell(ref shell);

        Wire newW1 = new(), newW2 = new();
        BrepBuilder bw1 = new(), bw2 = new();
        bw1.MakeWire(ref newW1);
        bw2.MakeWire(ref newW2);

        Vertex v1f, v1l, v2f, v2l;

        // (OCCT's unused 21 x nbSects array of points is left out)

        // concatenate each section to get a total surface that will be segmented
        global::Kerf.Geometry.BSplineSurface? ts;
        ts = TotalSurf(shapes, nbSects, nbEdges, w1Point, w2Point, vClosed);

        if (ts is null)
        {
            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Failed;
            return;
        }

        Shape firstEdge;
        for (i = 1; i <= nbEdges; i++)
        {

            // segmentation of TS
            global::Kerf.Geometry.BSplineSurface surface;
            surface = (global::Kerf.Geometry.BSplineSurface)ts.Copy();
            double ui1, ui2, v0, v1;
            ui1 = i - 1;
            ui2 = i;
            ui1 = PreciseUpar(ui1, surface);
            ui2 = PreciseUpar(ui2, surface);
            v0 = surface.VKnot(surface.FirstVKnotIndex());
            v1 = surface.VKnot(surface.LastVKnotIndex());
            surface.Segment(ui1, ui2, v0, v1);

            // return vertices
            edge = ShapeCasts.Edge(shapes[i]);
            Exploration.Vertices(edge, out v1f, out v1l);
            if (edge.Orientation == Reversed)
            {
                Exploration.Vertices(edge, out v1l, out v1f);
            }
            firstEdge = edge;

            edge = ShapeCasts.Edge(shapes[(nbSects - 1) * nbEdges + i]);
            Exploration.Vertices(edge, out v2f, out v2l);
            if (edge.Orientation == Reversed)
            {
                Exploration.Vertices(edge, out v2l, out v2f);
            }

            // make the face
            b.MakeFace(ref face, surface, global::Kerf.Engine.Foundation.Precision.Confusion());

            // make the wire
            b.MakeWire(ref w);

            // make the missing edges
            surface.ParameterBounds(out double f1, out double l1, out double f2, out double l2);

            // --- edge 1
            if (w1Point)
            {
                // copy the degenerated edge
                Shape localShape = shapes[1].EmptyCopied();
                edge1 = ShapeCasts.Edge(localShape);
                edge1.Orient(Forward);
            }
            else
            {
                b.MakeEdge(ref edge1, surface.VIso(f2), global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            v1f.Orient(Forward);
            b.Add(edge1, v1f);
            v1l.Orient(Reversed);
            b.Add(edge1, v1l);
            b.Range(edge1, f1, l1);
            // processing of looping sections
            // store edges of the 1st section
            if (vClosed)
            {
                vcouture[i] = edge1;
            }

            // --- edge 2
            if (vClosed)
            {
                edge2 = ShapeCasts.Edge(vcouture[i]);
            }
            else
            {
                if (w2Point)
                {
                    // copy of the degenerated edge
                    Shape localShape = shapes[nbSects * nbEdges].EmptyCopied();
                    edge2 = ShapeCasts.Edge(localShape);
                    edge2.Orient(Forward);
                }
                else
                {
                    b.MakeEdge(ref edge2, surface.VIso(l2), global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                v2f.Orient(Forward);
                b.Add(edge2, v2f);
                v2l.Orient(Reversed);
                b.Add(edge2, v2l);
                b.Range(edge2, f1, l1);
            }
            edge2.Reverse();

            // --- edge 3
            if (i == 1)
            {
                b.MakeEdge(ref edge3, surface.UIso(f1), global::Kerf.Engine.Foundation.Precision.Confusion());
                v1f.Orient(Forward);
                b.Add(edge3, v1f);
                v2f.Orient(Reversed);
                b.Add(edge3, v2f);
                b.Range(edge3, f2, l2);
                if (uClosed)
                {
                    couture = edge3;
                }
            }
            else
            {
                edge3 = edge4;
            }
            edge3.Reverse();

            // --- edge 4
            if (uClosed && i == nbEdges)
            {
                edge4 = couture;
            }
            else
            {
                b.MakeEdge(ref edge4, surface.UIso(l1), global::Kerf.Engine.Foundation.Precision.Confusion());
                v1l.Orient(Forward);
                b.Add(edge4, v1l);
                v2l.Orient(Reversed);
                b.Add(edge4, v2l);
                b.Range(edge4, f2, l2);
            }

            b.Add(w, edge1);
            b.Add(w, edge4);
            b.Add(w, edge2);
            b.Add(w, edge3);

            // set PCurve
            if (vClosed)
            {
                b.UpdateEdge(edge1,
                             new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X)),
                             new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                b.Range(edge1, face, f1, l1);
            }
            else
            {
                b.UpdateEdge(edge1,
                             new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                b.Range(edge1, face, f1, l1);
                b.UpdateEdge(edge2,
                             new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                b.Range(edge2, face, f1, l1);
            }

            if (uClosed && nbEdges == 1)
            {
                b.UpdateEdge(edge3,
                             new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)),
                             new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                b.Range(edge3, face, f2, l2);
            }
            else
            {
                b.UpdateEdge(edge3,
                             new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                b.Range(edge3, face, f2, l2);
                b.UpdateEdge(edge4,
                             new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                b.Range(edge4, face, f2, l2);
            }
            b.Add(face, w);
            b.Add(shell, face);

            // complete newW1 newW2
            Edge edge12 = edge1;
            Edge edge22 = edge2;
            edge12.Reverse();
            edge22.Reverse();
            bw1.Add(newW1, edge12);
            bw2.Add(newW2, edge22);

            // history
            _edgeFace[firstEdge] = face;
        }

        if (uClosed && w1Point && w2Point)
        {
            shell.Closed(true);
        }

        if (_isSolid)
        {

            if (vClosed)
            {

                Solid solid = new();
                b.MakeSolid(ref solid);
                b.Add(solid, shell);

                // verify the orientation the solid
                SolidClassifier clas3d = new(solid);
                clas3d.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
                if (clas3d.State() == In)
                {
                    b.MakeSolid(ref solid);
                    Shape localShape = shell.Reversed();
                    b.Add(solid, ShapeCasts.Shell(localShape));
                }
                _shape = solid;
            }

            else
            {
                _shape = MakeSolid(ref shell, newW1, newW2, _pres3d, ref _first, ref _last);
            }
        }

        else
        {
            _shape = shell;
        }
        Done();

        Dictionary<Shape, double> vertexToleranceMap = new(ShapeHasher.Instance);
        Explorer topExplorer = new(_shape, ShapeType.Edge);
        while (topExplorer.More())
        {
            Edge curEdge = ShapeCasts.Edge(topExplorer.Current());
            b.SameRange(curEdge, false);
            b.SameParameter(curEdge, false);
            double tolerance = Tool.Tolerance(curEdge);
            if (_mutableInput)
            {
                BrepLibrary.SameParameter(curEdge, tolerance);
            }
            else
            {
                // all edges from myShape can be safely updated/changed
                // all vertices from myShape are the part of the original wires
                double newTolerance = -1;
                BrepLibrary.SameParameter(curEdge, tolerance, ref newTolerance, true);
                if (newTolerance > 0)
                {
                    Exploration.Vertices(curEdge, out Vertex vertex1, out Vertex vertex2);
                    if (!vertex1.IsNull)
                    {
                        if (!vertexToleranceMap.TryGetValue(vertex1, out double oldTolerance) || oldTolerance < newTolerance)
                        {
                            vertexToleranceMap[vertex1] = newTolerance;
                        }
                    }
                    if (!vertex2.IsNull)
                    {
                        if (!vertexToleranceMap.TryGetValue(vertex2, out double oldTolerance) || oldTolerance < newTolerance)
                        {
                            vertexToleranceMap[vertex2] = newTolerance;
                        }
                    }
                }
            }
            topExplorer.Next();
        }

        if (!_mutableInput)
        {
            ReShape reshaper = new();
            foreach (KeyValuePair<Shape, double> mapIterator in vertexToleranceMap)
            {
                Vertex vertex = ShapeCasts.Vertex(mapIterator.Key);
                double newTolerance = mapIterator.Value;
                if (Tool.Tolerance(vertex) < newTolerance)
                {
                    Vertex nnewVertex = ShapeCasts.Vertex(vertex.EmptyCopied());
                    b.UpdateVertex(nnewVertex, newTolerance);
                    reshaper.Replace(vertex, nnewVertex);
                }
            }
            _shape = reshaper.InternalApply(_shape);
        }
    }

    //=======================================================================
    // function : EdgeToBSpline
    // purpose  : auxiliary -- get curve from edge and convert it to bspline
    //           parameterized from 0 to 1
    //=======================================================================

    // NOTE: this code duplicates the same function in BRepFill_NSections.cxx
    private static BSplineCurve3D? EdgeToBSpline(in Edge edge)
    {
        BSplineCurve3D? bsCurve = null;
        if (Tool.Degenerated(edge))
        {
            // degenerated edge : construction of a point curve
            NCollection_Array1<double> knots = new(1, 2);
            knots[1] = 0.0;
            knots[2] = 1.0;

            NCollection_Array1<int> mults = new(1, 2);
            mults[1] = 2;
            mults[2] = 2;

            NCollection_Array1<Point3D> poles = new(1, 2);
            Exploration.Vertices(edge, out Vertex vl, out Vertex vf);
            poles[1] = Tool.Pnt(vf);
            poles[2] = Tool.Pnt(vl);

            bsCurve = new BSplineCurve3D(poles, knots, mults, 1);
        }
        else
        {
            // get the curve of the edge
            Curve3D? curve = Tool.Curve(edge, out Placement loc, out double first, out double last);
            if (curve is null)
            {
                return null;
            }

            // convert its part used by edge to bspline; note that if edge curve is bspline,
            // conversion made via trimmed curve is still needed -- it will copy it, segment
            // as appropriate, and remove periodicity if it is periodic (deadly for approximator)
            TrimmedCurve3D trimCurve = new(curve, first, last);

            // special treatment of conic curve
            if (trimCurve.BasisCurve is ConicCurve3D)
            {
                Curve3D curveTrimmed = trimCurve; // to avoid ambiguity
                ApproxCurve appr = new(curveTrimmed, global::Kerf.Engine.Foundation.Precision.Confusion(), C1, 16, 14);
                if (appr.HasResult)
                {
                    bsCurve = appr.InternalCurve();
                }
            }

            // general case
            if (bsCurve is null)
            {
                bsCurve = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(trimCurve);
            }

            // apply transformation if needed
            if (!loc.IsIdentity)
            {
                bsCurve.Transform(loc.Transformation);
            }

            // reparameterize to [0,1]
            NCollection_Array1<double> knots = new(bsCurve.InternalKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, knots);
            bsCurve.SetKnots(knots);
        }

        // reverse curve if edge is reversed
        if (edge.Orientation == Reversed)
        {
            bsCurve.Reverse();
        }

        return bsCurve;
    }

    //=================================================================================================

    private global::Kerf.Geometry.BSplineSurface? TotalSurf(NCollection_Array1<Shape> shapes,
                                           int nbSects,
                                           int nbEdges,
                                           bool w1Point,
                                           bool w2Point,
                                           bool vClosed)
    {
        int i, j, jdeb = 1, jfin = nbSects;
        Vertex vf, vl;

        SectionGenerator section = new();
        global::Kerf.Geometry.BSplineSurface? surface = null;
        BSplineCurve3D? bs, bs1 = null;

        if (w1Point)
        {
            jdeb++;
            Edge edge = ShapeCasts.Edge(shapes[1]);
            Exploration.Vertices(edge, out vl, out vf);
            NCollection_Array1<Point3D> extremities = new(1, 2);
            extremities[1] = Tool.Pnt(vf);
            extremities[2] = Tool.Pnt(vl);
            NCollection_Array1<double> bounds = new(1, 2);
            bounds[1] = 0.0;
            bounds[2] = 1.0;
            NCollection_Array1<int> mult = new(1, 2);
            mult[1] = 2;
            mult[2] = 2;
            BSplineCurve3D bsPoint = new(extremities, bounds, mult, 1);
            section.AddCurve(bsPoint);
        }

        if (w2Point)
        {
            jfin--;
        }

        for (j = jdeb; j <= jfin; j++)
        {

            // case of looping sections
            if (j == jfin && vClosed)
            {
                section.AddCurve(bs1!);
            }

            else
            {
                // read the first edge to initialise CompBS;
                Edge prevEdge = ShapeCasts.Edge(shapes[(j - 1) * nbEdges + 1]);
                BSplineCurve3D? curvBS = EdgeToBSpline(prevEdge);
                if (curvBS is null)
                {
                    return null;
                }

                // initialization
                CompCurveToBSplineCurve compBS = new(curvBS);

                for (i = 2; i <= nbEdges; i++)
                {
                    // read the edge
                    Edge nextEdge = ShapeCasts.Edge(shapes[(j - 1) * nbEdges + i]);
                    double tolV = global::Kerf.Engine.Foundation.Precision.Confusion();
                    Exploration.Vertices(nextEdge, out vf, out vl);
                    tolV = Max(tolV, Tool.Tolerance(vf));
                    tolV = Max(tolV, Tool.Tolerance(vl));
                    tolV = Min(tolV, 1.0e-3);
                    curvBS = EdgeToBSpline(nextEdge);
                    if (curvBS is null)
                    {
                        return null;
                    }

                    // concatenation
                    compBS.Add(curvBS, tolV, true, false, 1);
                }

                // return the final section
                bs = compBS.InternalBSplineCurve();
                section.AddCurve(bs!);

                // case of looping sections
                if (j == jdeb && vClosed)
                {
                    bs1 = bs;
                }
            }
        }

        if (w2Point)
        {
            Edge edge = ShapeCasts.Edge(shapes[nbSects * nbEdges]);
            Exploration.Vertices(edge, out vl, out vf);
            NCollection_Array1<Point3D> extremities = new(1, 2);
            extremities[1] = Tool.Pnt(vf);
            extremities[2] = Tool.Pnt(vl);
            NCollection_Array1<double> bounds = new(1, 2);
            bounds[1] = 0.0;
            bounds[2] = 1.0;
            NCollection_Array1<int> mult = new(1, 2);
            mult[1] = 2;
            mult[2] = 2;
            BSplineCurve3D bsPoint = new(extremities, bounds, mult, 1);
            section.AddCurve(bsPoint);
        }

        section.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        SurfaceLine line = new(nbSects);

        int nbIt = 3;
        if (_pres3d <= 1.0e-3)
        {
            nbIt = 0;
        }

        int degmin = 2, degmax = Max(_degMax, degmin);
        bool spApprox = true;

        SurfaceAppSurf approx = new(degmin, degmax, _pres3d, _pres3d, nbIt);
        approx.SetContinuity(_continuity);

        if (_useSmoothing)
        {
            approx.SetCriteriumWeight(_critWeights[0], _critWeights[1], _critWeights[2]);
            approx.PerformSmoothing(line, section);
        }
        else
        {
            approx.SetParType(_paramType);
            approx.Perform(line, section, spApprox);
        }

        if (approx.IsDone)
        {
            surface = new global::Kerf.Geometry.BSplineSurface(approx.SurfPoles(),
                                              approx.SurfWeights(),
                                              approx.SurfUKnots(),
                                              approx.SurfVKnots(),
                                              approx.SurfUMults(),
                                              approx.SurfVMults(),
                                              approx.UDegree(),
                                              approx.VDegree());
        }

        return surface;
    }

    //=================================================================================================

    /// <summary>The face closing the solid at the first section (frozen; null for a shell or a point section).</summary>
    public Shape FirstShape() => Published(_first);

    /// <summary>The face closing the solid at the last section (frozen; null for a shell or a point section).</summary>
    public Shape LastShape() => Published(_last);

    private static Shape Published(Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    // Kerf: a shape the loft worked on (the caller's own) as it stands in the result (unit 10H3)
    private Shape Final(in Shape shape) => _overlay is null ? shape : _overlay.Final(shape);

    private Shape ResultShape(in Shape shape) => Final(_bfGenerator!.InternalResultShape(shape));

    private protected override void BeforePublishingLists() => _shape.TShape()?.FreezeGraph();

    //=================================================================================================

    /// <summary>
    /// The shapes generated from <paramref name="s"/>: for an edge of a section, the lateral faces built from it (from
    /// every section for a ruled loft); for a vertex, the lateral edges built from it.
    /// </summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();

        NCollection_Sequence<Shape> allFaces = new();
        Explorer explo = new(_shape, ShapeType.Face);
        for (; explo.More(); explo.Next())
        {
            allFaces.Append(explo.Current());
        }

        Shape sw = s;         // the key of the maps
        Shape sf = Final(sw); // Kerf: as it stands in the result

        if (s.Type == ShapeType.Edge)
        {
            if (!_edgeNewIndices.ContainsKey(sw))
            {
                return _generated;
            }

            NCollection_List<int> indices = _edgeNewIndices.GetValue(sw);
            // Append the faces corresponding to <Indices>
            // These faces "grow" from the first section
            NCollection_List<int>.Iterator itl = new(indices);
            for (; itl.More(); itl.Next())
            {
                int indOfFace = itl.Value();
                if (allFaces.Length() < indOfFace)
                {
                    continue;
                }
                _generated.Append(allFaces.Value(indOfFace));
            }

            if (_isRuled)
            {
                // Append the next faces corresponding to <Indices>
                for (int i = 2; i < _wires.Length(); i++)
                {
                    for (itl.Initialize(indices); itl.More(); itl.Next())
                    {
                        int indOfFace = itl.Value();
                        indOfFace += (i - 1) * _nbEdgesInSection;
                        if (allFaces.Length() < indOfFace)
                        {
                            continue;
                        }
                        _generated.Append(allFaces.Value(indOfFace));
                    }
                }
            }
        }
        else if (s.Type == ShapeType.Vertex)
        {
            if (!_vertexIndex.ContainsKey(sw))
            {
                return _generated;
            }

            IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);

            bool[] isDegen = [false, false];
            if (_degen1 || _degen2)
            {
                Shape[] endSections = new Shape[2];
                endSections[0] = _wires.Value(1);
                endSections[1] = _wires.Value(_wires.Length());
                for (int i = 0; i < 2; i++)
                {
                    if (i == 0 && !_degen1)
                    {
                        continue;
                    }
                    if (i == 1 && !_degen2)
                    {
                        continue;
                    }

                    explo.Init(endSections[i], ShapeType.Vertex);
                    Shape vertex = Final(explo.Current());
                    if (sf.IsSame(vertex))
                    {
                        isDegen[i] = true;
                        break;
                    }
                }
            }
            // Only one of <IsDegen> can be True:
            // in case of one vertex for start and end degenerated sections
            // IsDegen[0] is True;
            if (isDegen[0] || isDegen[1])
            {
                // For start or end degenerated section
                // we return the whole bunch of longitudinal edges
                Exploration.MapShapesAndAncestors(_shape, ShapeType.Vertex, ShapeType.Edge, vEmap);
                IndexedSet<Shape> emap = new(ShapeHasher.Instance);
                Shape newShape = sf;
                if ((_isRuled || !_mutableInput) && _bfGenerator is not null)
                {
                    newShape = ResultShape(s);
                }

                NCollection_List<Shape> edgeList = vEmap.GetValue(newShape);
                NCollection_List<Shape>.Iterator listIterator = new(edgeList);
                for (; listIterator.More(); listIterator.Next())
                {
                    Edge edge = ShapeCasts.Edge(listIterator.Value());
                    if (!Tool.Degenerated(edge))
                    {
                        Vertex[] vv = new Vertex[2];
                        Exploration.Vertices(edge, out vv[0], out vv[1]);
                        // Comprehensive check for possible case of
                        // one vertex for start and end degenerated sections:
                        // we must take only outgoing or only ingoing edges
                        if ((isDegen[0] && newShape.IsSame(vv[0])) || (isDegen[1] && newShape.IsSame(vv[1])))
                        {
                            emap.Add(edge);
                        }
                    }
                }
                for (int j = 0; j < emap.Count; j++)
                {
                    Edge edge = ShapeCasts.Edge(emap[j]);
                    _generated.Append(edge);
                    if (_isRuled)
                    {
                        int i, k;
                        for (i = 2, k = _wires.Length() - 1; i < _wires.Length(); i++, k--)
                        {
                            int indOfSec = (isDegen[0]) ? i : k;
                            Vertex vertex =
                              (isDegen[0]) ? Exploration.LastVertex(edge) : Exploration.FirstVertex(edge);
                            NCollection_List<Shape> eElist = vEmap.GetValue(vertex);
                            IndexedSet<Shape> emapOfSection = new(ShapeHasher.Instance);
                            Shape wireSection = Final(_wires.Value(indOfSec));
                            if ((_isRuled || !_mutableInput) && _bfGenerator is not null)
                            {
                                wireSection = ResultShape(_wires.Value(indOfSec));
                            }
                            Exploration.MapShapes(wireSection, ShapeType.Edge, emapOfSection);
                            if (_overlay is not null)
                            {
                                // Kerf: the section's edges as they stand in the result
                                IndexedSet<Shape> finalEdges = new(ShapeHasher.Instance);
                                for (int e = 0; e < emapOfSection.Count; e++)
                                {
                                    finalEdges.Add(Final(emapOfSection[e]));
                                }
                                emapOfSection = finalEdges;
                            }
                            Edge nextEdge = new();
                            for (listIterator.Initialize(eElist); listIterator.More(); listIterator.Next())
                            {
                                nextEdge = ShapeCasts.Edge(listIterator.Value());
                                if (!nextEdge.IsSame(edge) && !emapOfSection.Contains(nextEdge))
                                {
                                    break;
                                }
                            }
                            _generated.Append(nextEdge);
                            edge = nextEdge;
                        }
                    }
                }
                return _generated;
            } // end of if (IsDegen[0] || IsDegen[1])

            int eindex = _vertexIndex.GetValue(sw);
            int vindex = (eindex > 0) ? 0 : 1;
            eindex = Math.Abs(eindex);

            // Find the first longitudinal edge
            Face firstFace = ShapeCasts.Face(allFaces.Value(eindex));
            firstFace.Orient(Forward);
            explo.Init(firstFace, ShapeType.Edge);
            Edge edge2 = new();
            FaceSurfaceAdaptor bAsurf = new(firstFace, false);
            Vertex firstVertex = new();
            Exploration.MapShapesAndAncestors(firstFace, ShapeType.Vertex, ShapeType.Edge, vEmap);
            if (_degen1 && bAsurf.GetType() == Plane)
            {
                // There are only 3 edges in the face in this case:
                // we take 1-st or 3-rd edge
                if (vindex == 0)
                {
                    explo.Next();
                    explo.Next();
                }
                edge2 = ShapeCasts.Edge(explo.Current());
            }
            else
            {
                Edge firstEdge = new();
                Vertex firstVertexOfFirstEdge = new();
                Wire firstSection = ShapeCasts.Wire(_wires.Value(1));
                WireExplorer wireExplorer = new(firstSection);
                for (int i = 1; wireExplorer.More(); wireExplorer.Next(), i++)
                {
                    firstEdge = wireExplorer.Current();
                    if (i == eindex)
                    {
                        if ((_isRuled || !_mutableInput) && _bfGenerator is not null)
                        {
                            firstEdge = ShapeCasts.Edge(ResultShape(firstEdge));
                        }
                        else
                        {
                            firstEdge = ShapeCasts.Edge(Final(firstEdge));
                        }
                        firstVertexOfFirstEdge = ShapeCasts.Vertex(Final(wireExplorer.CurrentVertex()));
                        break;
                    }
                }

                Shape firstEdgeInFace;
                firstEdgeInFace = explo.Current();
                Vertex[] vv = new Vertex[2];
                Exploration.Vertices(firstEdge, out vv[0], out vv[1]);
                vv[0] = ShapeCasts.Vertex(Final(vv[0])); // OCCT.NET: a section's edge may be outside the result
                vv[1] = ShapeCasts.Vertex(Final(vv[1]));
                if (vindex == 0)
                {
                    if (vv[0].IsSame(firstVertexOfFirstEdge))
                    {
                        firstVertex = vv[0];
                    }
                    else
                    {
                        firstVertex = vv[1];
                    }
                }
                else // Vindex == 1
                {
                    if (vv[0].IsSame(firstVertexOfFirstEdge))
                    {
                        firstVertex = vv[1];
                    }
                    else
                    {
                        firstVertex = vv[0];
                    }
                }
                NCollection_List<Shape> elist = vEmap.GetValue(firstVertex);
                NCollection_List<Shape>.Iterator itl = new(elist);
                Orientation edgeOr = (vindex == 0) ? Reversed : Forward;
                for (; itl.More(); itl.Next())
                {
                    edge2 = ShapeCasts.Edge(itl.Value());
                    if (!edge2.IsSame(firstEdgeInFace) && !Tool.Degenerated(edge2)
                        && edge2.Orientation == edgeOr)
                    {
                        break;
                    }
                }
            }
            _generated.Append(edge2);
            if (_isRuled)
            {
                // Find the chain of longitudinal edges from first to last
                for (int i = 2; i < _wires.Length(); i++)
                {
                    firstVertex = Exploration.LastVertex(edge2);
                    NCollection_List<Shape> elist1 = vEmap.GetValue(firstVertex);
                    Edge firstEdge = (edge2.IsSame(elist1.First())) ? ShapeCasts.Edge(elist1.Last())
                                                                             : ShapeCasts.Edge(elist1.First());
                    eindex += _nbEdgesInSection;
                    firstFace = ShapeCasts.Face(allFaces.Value(eindex));
                    firstFace.Orient(Forward);
                    vEmap.Clear();
                    Exploration.MapShapesAndAncestors(firstFace, ShapeType.Vertex, ShapeType.Edge, vEmap);
                    NCollection_List<Shape> elist2 = vEmap.GetValue(firstVertex);
                    edge2 = (firstEdge.IsSame(elist2.First())) ? ShapeCasts.Edge(elist2.Last())
                                                                 : ShapeCasts.Edge(elist2.First());
                    _generated.Append(edge2);
                }
            }
        }

        return _generated;
    }

    //=================================================================================================

    /// <summary>
    /// The face of the loft generated by the edge <paramref name="edge"/> of the first section (or of a section, for a
    /// ruled loft), frozen; a null shape for another edge.
    /// </summary>
    public Shape GeneratedFace(in Shape edge)
    {
        BeforePublishingLists();
        return Published(InternalGeneratedFace(edge));
    }

    /// <summary><see cref="GeneratedFace"/> without freezing.</summary>
    internal Shape InternalGeneratedFace(in Shape edge)
    {
        Shape bid = new();
        Shape key = edge;
        if (_edgeFace.ContainsKey(key))
        {
            return _edgeFace.GetValue(key);
        }
        else
        {
            return bid;
        }
    }

    //=======================================================================
    // function : CriteriumWeight
    // purpose  : returns the Weights associated  to the criterium used in
    //           the  optimization.
    //=======================================================================
    //
    /// <summary>The weights of the smoothing criterion.</summary>
    public void CriteriumWeight(out double w1, out double w2, out double w3)
    {
        w1 = _critWeights[0];
        w2 = _critWeights[1];
        w3 = _critWeights[2];
    }

    //=================================================================================================

    /// <summary>
    /// Sets the weights of the smoothing criterion (length, curvature, torsion); a negative weight sets the status
    /// to Failed and changes nothing (as in OCCT, the next <see cref="Build"/> resets the status).
    /// </summary>
    public void SetCriteriumWeight(double w1, double w2, double w3)
    {
        if (w1 < 0 || w2 < 0 || w3 < 0)
        {
            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Failed;
            return;
        }
        _critWeights[0] = w1;
        _critWeights[1] = w2;
        _critWeights[2] = w3;
    }

    //=================================================================================================

    /// <summary>Sets the continuity of the smoothed surfaces (C2 by default).</summary>
    public void SetContinuity(Continuity theCont)
    {
        _continuity = theCont;
    }

    /// <summary>The continuity of the smoothed surfaces.</summary>
    public Continuity Continuity()
    {
        return _continuity;
    }

    //=================================================================================================

    /// <summary>Sets the parametrisation of the approximation (chord length by default).</summary>
    public void SetParType(global::Kerf.Geometry.ParametrizationType parType)
    {
        _paramType = parType;
    }

    /// <summary>The parametrisation of the approximation.</summary>
    public global::Kerf.Geometry.ParametrizationType ParType()
    {
        return _paramType;
    }

    //=================================================================================================

    /// <summary>Sets the maximal U degree of the smoothed surfaces (8 by default).</summary>
    public void SetMaxDegree(int maxDeg)
    {
        _degMax = maxDeg;
    }

    /// <summary>The maximal U degree of the smoothed surfaces.</summary>
    public int MaxDegree()
    {
        return _degMax;
    }

    //=================================================================================================

    /// <summary>Sets whether the approximation uses the smoothing criterion (false by default).</summary>
    public void SetSmoothing(bool useVar)
    {
        _useSmoothing = useVar;
    }

    /// <summary>True if the approximation uses the smoothing criterion.</summary>
    public bool UseSmoothing()
    {
        return _useSmoothing;
    }

    //=================================================================================================

    /// <summary>
    /// Sets the mutable input state: when true (the default), the loft writes into its (working) sections' edges and
    /// vertices; when false, into copies of them. The caller's shapes are never written either way.
    /// </summary>
    public void SetMutableInput(bool isMutableInput)
    {
        _mutableInput = isMutableInput;
    }

    /// <summary>The mutable input state.</summary>
    public bool IsMutableInput
    {
        get
        {
            return _mutableInput;
        }
    }

    /// <summary>The sections as given (a point section as its wire of one degenerated edge), frozen.</summary>
    public IReadOnlyList<Shape> Wires() => PublishedList(_inputWires);

    /// <summary>The status of the last <see cref="Build"/>.</summary>
    public ThruSectionErrorStatus GetStatus() => _status;
}
