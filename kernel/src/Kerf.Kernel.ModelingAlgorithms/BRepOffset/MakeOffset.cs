// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Tue Mar 15 16:20:43 2005
// Add methods for supporting history.
//  Modified by skv - Mon Jan 12 11:50:02 2004 OCC4455

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Construction.Values;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Offsets.Error;
using static Kerf.Engine.Offsets.OffsetMode;
using static Kerf.Engine.Offsets.OffsetStatus;
using static Kerf.Engine.Fillets.Data.TypeOfConcavity;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Offsets;

/// <summary>
/// The 3D offset of a shape (a face, a shell, a solid or a compound of them): the offset shell or solid, and the thick
/// solid (<see cref="MakeThickSolid"/>), with the faces connected by arcs (pipes and spheres) or by intersection.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). OCCT's algorithm writes into the shape it works on: tolerances
/// (<c>UpdateTolerance</c> unlocks the shape with <c>Locked(false)</c>), pcurves on the free boundary edges
/// (<c>MakeMissingWalls</c>), <c>Free(true)</c> before re-adding sub-shapes. So <see cref="MakeOffsetShape"/> and
/// <see cref="MakeThickSolid"/> run OCCT's code on the caller's shape in a working overlay (unit 10H5,
/// <see cref="WorkingOverlay"/>; in place inside an operation's overlay that takes writes): the writes into the
/// caller's frozen TShapes go to working copies, and at the end the result holds new TShapes only for the sub-shapes
/// that changed (a thick solid holds the caller's faces it left alone). The thickening mode too (unit 10H6): its walls,
/// made by BRepLib_MakeFace on the caller's free edges, write into the same overlay and share those edges. The
/// history (<see cref="Generated"/>, <see cref="Modified"/>, <see cref="IsDeleted"/>) answers for the caller's
/// sub-shapes, as OCCT's.
/// </para>
/// <para>
/// <see cref="Shape"/>, <see cref="Generated"/>, <see cref="Modified"/>, <see cref="ClosingFaces"/>,
/// <see cref="GetBadShape"/> and the images freeze what they hand out (publication points); ported code reads the
/// <c>Internal</c> forms. <see cref="GetAnalyse"/> is internal (decision 1007): its tangent treatment holds the
/// offset's working shapes.
/// </para>
/// </remarks>
public sealed partial class MakeOffset
{
    private double _offset;
    private double _tol;
    private Shape _initialShape = new();
    private Shape _shape = new();
    private Compound _faceComp = new();
    private OffsetMode _mode;
    private bool _isLinearizationAllowed;
    private bool _inter;
    private bool _selfInter;
    private global::Kerf.Geometry.JoinKind _join;
    private bool _thickening;
    private bool _removeIntEdges;
    private readonly Dictionary<Shape, double> _faceOffset = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _faces = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _originalFaces = new(ShapeHasher.Instance);
    private readonly Analyse _analyse = new();
    private Shape _offsetShape = new();
    private readonly ShapeImage _initOffsetFace = new();
    private readonly ShapeImage _initOffsetEdge = new();
    private readonly ShapeImage _imageOffset = new();
    private readonly ShapeImage _imageVV = new();
    private readonly NCollection_List<Shape> _walls = new();
    private AsDes _asDes;
    private readonly Dictionary<Shape, NCollection_List<Shape>> _edgeIntEdges =
        new(ShapeHasher.Instance);
    private bool _done;
    private Error _error;
    private readonly OffsetMakeLoops _makeLoops = new();
    private bool _isPerformSewing; // Handle bad walls in thicksolid mode.
    private bool _isPlanar;
    private Shape _badShape = new();
    private readonly Dictionary<Shape, Shape> _facePlanfaceMap = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> _generated = new();
    private readonly HashSet<Shape> _resMap = new(ShapeHasher.Instance);

    // Kerf: the working overlay of the last run (unit 10H5); null until a run starts, after Initialize, and for a run
    // in place inside an operation's overlay.
    private WorkingOverlay? _overlay;
    private Dictionary<TShape, TShape>? _copies;    // a caller's unfrozen TShapes -> their copies (BeginOverlay)
    private Dictionary<TShape, TShape>? _originals; // the inverse

    // OCCT_DEBUG: the globals AffichInt2d, AffichOffC, ChronBuild, NbAE, NbAF, NVP, NVM, NVN, the chronometer and
    // DEBVerticesControl are left out.

    //=======================================================================
    // function : BRepOffset_PIOperation
    // purpose  : List of operations to be supported by the Progress Indicator
    //=======================================================================
    private const int PIOperation_CheckInputData = 0;
    private const int PIOperation_Analyse = 1;
    private const int PIOperation_BuildOffsetBy = 2;
    private const int PIOperation_Intersection = 3;
    private const int PIOperation_MakeMissingWalls = 4;
    private const int PIOperation_MakeShells = 5;
    private const int PIOperation_MakeSolid = 6;
    private const int PIOperation_Sewing = 7;
    private const int PIOperation_Last = 8;

    //=======================================================================
    // function : normalizeSteps
    // purpose  : Normalization of progress steps
    //=======================================================================
    private static void NormalizeSteps(double whole, NCollection_Array1<double> steps)
    {
        double sum = 0.0;
        for (int i = steps.Lower(); i <= steps.Upper(); ++i)
        {
            sum += steps[i];
        }

        // Normalize steps
        for (int i = steps.Lower(); i <= steps.Upper(); ++i)
        {
            steps[i] = whole * steps[i] / sum;
        }
    }

    //=================================================================================================

    private static Orientation OrientationOfEdgeInFace(in Edge edge, in Face face)
    {
        Orientation or = External;

        Explorer explo = new(face, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Shape anEdge = explo.Current();
            if (anEdge.IsSame(edge))
            {
                or = anEdge.Orientation;
                break;
            }
        }

        return or;
    }

    //=================================================================================================

    private static bool FindParameter(in Vertex v, in Edge e, ref double u)
    {
        // Search the vertex in the edge

        bool rev = false;
        Shape vf = new();
        Orientation orient = Internal;

        Iterator itv = new(e.Oriented(Forward));

        // if the edge has no vertices
        // and is degenerated use the vertex orientation
        // RLE, june 94

        if (!itv.More() && Tool.Degenerated(e))
        {
            orient = v.Orientation;
        }

        while (itv.More())
        {
            Shape vcur = itv.Value();
            if (v.IsSame(vcur))
            {
                if (vf.IsNull)
                {
                    vf = vcur;
                }
                else
                {
                    rev = e.Orientation == global::Kerf.Topology.Orientation.Reversed;
                    if (vcur.Orientation == v.Orientation)
                    {
                        vf = vcur;
                    }
                }
            }
            itv.Next();
        }

        if (!vf.IsNull)
        {
            orient = vf.Orientation;
        }

        double f, l;

        if (orient == Forward)
        {
            Tool.Range(e, out f, out l);
            // return (rev) ? l : f;
            u = (rev) ? l : f;
            return true;
        }

        else if (orient == global::Kerf.Topology.Orientation.Reversed)
        {
            Tool.Range(e, out f, out l);
            // return (rev) ? f : l;
            u = (rev) ? f : l;
            return true;
        }

        else
        {
            Curve3D? c = Tool.Curve(e, out Placement L, out f, out l);
            L = L.Predivided(v.Placement);
            if (c is not null || Tool.Degenerated(e))
            {
                NCollection_List<PointRepresentation>.Iterator itpr = new(((BrepTVertex)v.TShape()!).Points());

                while (itpr.More())
                {
                    PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnCurve(c, L))
                    {
                        double p = pr.Parameter();
                        double res = p; // SVV 4 nov 99 - to avoid warnings on Linux
                        if (c is not null)
                        {
                            // Closed curves RLE 16 june 94
                            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
                            {
                                // return pr->Parameter();//p;
                                u = pr.Parameter();
                                return true;
                            }
                            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
                            {
                                // return pr->Parameter();//p;
                                u = pr.Parameter();
                                return true;
                            }
                            Point3D pf = c.Value(f).Transformed(L.Transformation);
                            Point3D pl = c.Value(l).Transformed(L.Transformation);
                            double tol = Tool.Tolerance(v);
                            if (pf.DistanceTo(pl) < tol)
                            {
                                if (pf.DistanceTo(Tool.Pnt(v)) < tol)
                                {
                                    if (v.Orientation == Forward)
                                    {
                                        res = f; // p = f;
                                    }
                                    else
                                    {
                                        res = l; // p = l;
                                    }
                                }
                            }
                        }
                        // return res;//p;
                        u = res;
                        return true;
                    }
                    itpr.Next();
                }
            }
            else
            {
                // no 3d curve !!
                // let us try with the first pcurve
                Tool.CurveOnSurface(e, out Curve2D? pc, out Surface? s, out L, out f, out l);
                L = L.Predivided(v.Placement);
                NCollection_List<PointRepresentation>.Iterator itpr = new(((BrepTVertex)v.TShape()!).Points());

                while (itpr.More())
                {
                    PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnCurveOnSurface(pc, s, L))
                    {
                        double p = pr.Parameter();
                        // Closed curves RLE 16 june 94
                        if (pc!.IsClosed)
                        {
                            if ((p == pc.FirstParameter) || (p == pc.LastParameter))
                            {
                                if (v.Orientation == Forward)
                                {
                                    p = pc.FirstParameter;
                                }
                                else
                                {
                                    p = pc.LastParameter;
                                }
                            }
                        }
                        // return p;
                        u = p;
                        return true;
                    }
                    itpr.Next();
                }
            }
        }

        // throw Standard_NoSuchObject("BRep_Tool:: no parameter on edge");
        return false;
    }

    //=======================================================================
    // function : GetEdgePoints
    // purpose  : gets the first, last and middle points of the edge
    //=======================================================================
    private static void GetEdgePoints(in Edge anEdge, in Face aFace, out Point3D fPnt, out Point3D mPnt,
                                      out Point3D lPnt)
    {
        Curve2D theCurve = Tool.CurveOnSurface(anEdge, aFace, out double f, out double l)!;
        Point2D fPnt2d = theCurve.Value(f);
        Point2D lPnt2d = theCurve.Value(l);
        Point2D mPnt2d = theCurve.Value(0.5 * (f + l));
        Surface surf = Tool.Surface(aFace)!;
        fPnt = surf.Value(fPnt2d.X, fPnt2d.Y);
        lPnt = surf.Value(lPnt2d.X, lPnt2d.Y);
        mPnt = surf.Value(mPnt2d.X, mPnt2d.Y);
    }

    //=======================================================================
    // function : FillContours
    // purpose  : fills free boundary contours and faces connected (MapEF)
    //=======================================================================
    private static void FillContours(in Shape aShape, Analyse analyser,
                                     IndexedMap<Shape, NCollection_List<Shape>> contours,
                                     Dictionary<Shape, Shape> mapEF)
    {
        NCollection_List<Shape> edges = new();

        Explorer explo = new(aShape, ShapeType.Face);
        WireExplorer wexp = new();

        for (; explo.More(); explo.Next())
        {
            Face face = ShapeCasts.Face(explo.Current());
            Iterator itf = new(face);
            for (; itf.More(); itf.Next())
            {
                Wire wire = ShapeCasts.Wire(itf.Value());
                for (wexp.Init(wire, face); wexp.More(); wexp.Next())
                {
                    Edge edge = wexp.Current();
                    if (Tool.Degenerated(edge))
                    {
                        continue;
                    }
                    NCollection_List<OffsetInterval> lint = analyser.Type(edge);
                    if (!lint.IsEmpty() && lint.First().Type() == TypeOfConcavity.FreeBound)
                    {
                        mapEF[edge] = face;
                        edges.Append(edge);
                    }
                }
            }
        }

        NCollection_List<Shape>.Iterator itl = new();
        while (!edges.IsEmpty())
        {
            Edge startEdge = ShapeCasts.Edge(edges.First());
            edges.RemoveFirst();
            Exploration.Vertices(startEdge, out Vertex startVertex, out Vertex curVertex, true);
            NCollection_List<Shape> contour = new();
            contour.Append(startEdge);
            while (!curVertex.IsSame(startVertex))
            {
                for (itl.Initialize(edges); itl.More(); itl.Next())
                {
                    Edge edge = ShapeCasts.Edge(itl.Value());
                    Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                    if (v1.IsSame(curVertex) || v2.IsSame(curVertex))
                    {
                        contour.Append(edge);
                        curVertex = (v1.IsSame(curVertex)) ? v2 : v1;
                        edges.Remove(itl);
                        break;
                    }
                }
            }
            contours.Add(startVertex, contour);
        }
    }

    //
    //-----------------------------------------------------------------------
    //
    //=================================================================================================

    /// <summary>An empty algorithm; call <see cref="Initialize"/>.</summary>
    public MakeOffset()
    {
        _asDes = new AsDes();
    }

    //=================================================================================================

    /// <summary>Initializes the algorithm and makes the offset shape (<see cref="MakeOffsetShape"/>).</summary>
    public MakeOffset(in Shape s, double offset, double tol, OffsetMode mode = OffsetMode.Skin,
                                 bool inter = false, bool selfInter = false, global::Kerf.Geometry.JoinKind Join = Arc,
                                 bool thickening = false, bool removeIntEdges = false,
                                 ProgressRange range = default)
    {
        _offset = offset;
        _tol = tol;
        _initialShape = s;
        _shape = s;
        _mode = mode;
        _inter = inter;
        _selfInter = selfInter;
        _join = Join;
        _thickening = thickening;
        _removeIntEdges = removeIntEdges;
        _done = false;
        _asDes = new AsDes();
        _isLinearizationAllowed = true;

        MakeOffsetShape(range);
    }

    //=================================================================================================

    /// <summary>Sets the arguments of the offset and clears the results of an earlier run.</summary>
    public void Initialize(in Shape s, double offset, double tol, OffsetMode mode = OffsetMode.Skin,
                           bool inter = false, bool selfInter = false, global::Kerf.Geometry.JoinKind Join = Arc,
                           bool thickening = false, bool removeIntEdges = false)
    {
        _offset = offset;
        _initialShape = s;
        _shape = s;
        _tol = tol;
        _mode = mode;
        _inter = inter;
        _selfInter = selfInter;
        _join = Join;
        _thickening = thickening;
        _removeIntEdges = removeIntEdges;
        _isLinearizationAllowed = true;
        _done = false;
        _isPerformSewing = false;
        _isPlanar = false;
        _overlay = null;
        _copies = null;
        _originals = null;
        // Kerf: OCCT's analysis keeps its edge types and ancestors from the last run, which a run on the same shapes
        // then doubles (occt-issues.md); with the copies at the entry each run had new shapes (unit 10H5)
        _analyse.Clear();
        Clear();
    }

    //=================================================================================================

    /// <summary>Clears the results.</summary>
    public void Clear()
    {
        _offsetShape.Nullify();
        _initOffsetFace.Clear();
        _initOffsetEdge.Clear();
        _imageOffset.Clear();
        _imageVV.Clear();
        _faces.Clear();
        _originalFaces.Clear();
        _faceOffset.Clear();
        _edgeIntEdges.Clear();
        _asDes.Clear();
        _done = false;
        _generated.Clear();
        _resMap.Clear();
    }

    //=================================================================================================

    /// <summary>Changes the flag allowing the linearization.</summary>
    public void AllowLinearization(bool isAllowed)
    {
        _isLinearizationAllowed = isAllowed;
    }

    //=================================================================================================

    /// <summary>Adds a closing face; <paramref name="f"/> has to be in the initial shape.</summary>
    public void AddFace(in Face f)
    {

        _originalFaces.Add(f);
    }

    //=================================================================================================

    /// <summary>Sets the offset <paramref name="off"/> on the face <paramref name="f"/>.</summary>
    public void SetOffsetOnFace(in Face f, double off)
    {
        _faceOffset[f] = off;
    }

    //=================================================================================================

    private static void RemoveCorks(ref Shape s, IndexedSet<Shape> faces)
    {
        Compound ss = new();
        BrepBuilder b = new();
        b.MakeCompound(ref ss);
        //-----------------------------------------------------
        // Construction of a shape without caps.
        // and Orientation of caps as in shape S.
        //-----------------------------------------------------
        Explorer exp = new(s, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Shape cork = exp.Current();
            if (!faces.Contains(cork))
            {
                b.Add(ss, cork);
            }
            else
            {
                faces.Remove(cork);
                faces.Add(cork); // to reset it with proper orientation.
            }
        }
        s = ss;
    }

    //=================================================================================================

    private static bool IsConnectedShell(in Shape s)
    {
        Quilt glue = new();
        glue.Add(s);

        Shape ss = glue.InternalShells();
        Explorer explo = new(ss, ShapeType.Shell);
        explo.Next();
        return !explo.More();
    }

    //=================================================================================================

    private static void MakeList(NCollection_List<Shape> offsetFaces, ShapeImage myInitOffsetFace,
                                 IndexedSet<Shape> myFaces)
    {
        NCollection_List<Shape>.Iterator itLOF = new(myInitOffsetFace.Roots());
        for (; itLOF.More(); itLOF.Next())
        {
            Shape root = itLOF.Value();
            if (!myFaces.Contains(root))
            {
                if (myInitOffsetFace.HasImage(root))
                {
                    NCollection_List<Shape>.Iterator itLS = new(myInitOffsetFace.Image(root));
                    for (; itLS.More(); itLS.Next())
                    {
                        offsetFaces.Append(itLS.Value());
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static void EvalMax(in Shape s, ref double tol)
    {
        Explorer exp = new();
        for (exp.Init(s, ShapeType.Vertex); exp.More(); exp.Next())
        {
            Vertex v = ShapeCasts.Vertex(exp.Current());
            double tolV = Tool.Tolerance(v);
            if (tolV > tol)
            {
                tol = tolV;
            }
        }
    }

    //=================================================================================================

    private void SetFaces()
    {
        for (int ii = 0; ii < _originalFaces.Count; ii++)
        {
            Face face = ShapeCasts.Face(_originalFaces[ii]);
            if (_facePlanfaceMap.TryGetValue(face, out Shape planface))
            {
                face = ShapeCasts.Face(planface);
            }

            _faces.Add(face);
            //-------------
            // MAJ SD.
            //-------------
            _initOffsetFace.SetRoot(face);
            _initOffsetFace.Bind(face, face);
            _imageOffset.SetRoot(face);
        }
    }

    //=================================================================================================

    private void SetFacesWithOffset()
    {
        foreach (KeyValuePair<Shape, Shape> itmap in _facePlanfaceMap)
        {
            Face face = ShapeCasts.Face(itmap.Key);
            Face planface = ShapeCasts.Face(itmap.Value);
            if (_faceOffset.ContainsKey(face))
            {
                double offset = _faceOffset.GetValue(face);
                _faceOffset.Remove(face);
                _faceOffset[planface] = offset;
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Opens the working overlay of a run on the caller's shape (unit 10H5): null when an operation's overlay that
    /// takes writes is open here, which the run then writes into, as OCCT's into the shape it is given. A caller's
    /// unfrozen shape (a shell or compound it built of frozen faces) is worked on with its unfrozen TShapes copied
    /// (<see cref="WorkingOverlay.WithUnfrozenCopied"/>), so that the run never writes into it either; the closing faces
    /// and the faces with their own offsets are mapped onto the copies.
    /// </summary>
    private WorkingOverlay? BeginOverlay()
    {
        if (WorkingOverlay.Writing is not null)
        {
            _overlay = null;
            return null;
        }
        if (_copies is null)
        {
            _shape = WorkingOverlay.WithUnfrozenCopied(_shape, out _copies);
            if (_copies is not null)
            {
                _originals = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
                foreach (KeyValuePair<TShape, TShape> pair in _copies)
                {
                    _originals[pair.Value] = pair.Key;
                }
                IndexedSet<Shape> originalFaces = new(_originalFaces);
                _originalFaces.Clear();
                for (int i = 0; i < originalFaces.Count; ++i)
                {
                    _originalFaces.Add(InternalCopied(originalFaces[i]));
                }
                Dictionary<Shape, double> faceOffset = new(_faceOffset,_faceOffset.Comparer);
                _faceOffset.Clear();
                foreach (KeyValuePair<Shape, double> it in faceOffset)
                {
                    _faceOffset[InternalCopied(it.Key)] = it.Value;
                }
            }
        }
        _overlay = WorkingOverlay.Begin(_shape);
        return _overlay;
    }

    /// <summary>
    /// Ends a run: the result holds new TShapes for the caller's sub-shapes the run changed, the caller's own for the
    /// rest (and for the unfrozen ones copied at the start that it left equal). Finished for a null result too, so that
    /// the history can be read after a run reported done with a null shape (decision 1049).
    /// </summary>
    private void FinishOverlay(WorkingOverlay? overlay)
    {
        if (overlay is not null)
        {
            _offsetShape = WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(_offsetShape), _copies);
        }
    }

    /// <summary>
    /// The shape a run works on for a caller's sub-shape <paramref name="s"/>: itself, or its copy for an unfrozen
    /// sub-shape of an unfrozen input (see <see cref="BeginOverlay"/>).
    /// </summary>
    internal Shape InternalCopied(in Shape s) => WorkingOverlay.CopiedIn(s, _copies);

    // The caller's sub-shape a shape of the run stands for (the inverse of InternalCopied).
    private Shape InternalOriginal(in Shape s)
    {
        if (_originals is null || s.IsNull || !_originals.TryGetValue(s.TShape()!, out TShape? original))
        {
            return s;
        }
        Shape aShape = s;
        aShape.TShape(original);
        return aShape;
    }

    /// <summary>What stands in the result for a shape of the run (see <see cref="WorkingOverlay.Final"/>).</summary>
    internal Shape InternalFinal(in Shape s)
    {
        return _overlay is null || !_overlay.IsFinished || s.IsNull ? s : _overlay.Final(s);
    }

    //=================================================================================================

    /// <summary>Makes the offset shape (a shell, or a solid for a solid input).</summary>
    public void MakeOffsetShape(ProgressRange range = default)
    {
        using WorkingOverlay? overlay = BeginOverlay();
        InternalMakeOffsetShape(range);
        FinishOverlay(overlay);
    }

    private void InternalMakeOffsetShape(ProgressRange range)
    {
        _done = false;
        //

        // check if shape consists of only planar faces
        _isPlanar = IsPlanar();

        SetFaces();
        SetFacesWithOffset();

        BuildFaceComp();

        //------------------------------------------
        // Construction of myShape without caps.
        //------------------------------------------
        if (_faces.Count != 0)
        {
            RemoveCorks(ref _shape, _originalFaces);
            Shape faceComp = _faceComp;
            RemoveCorks(ref faceComp, _faces);
            _faceComp = ShapeCasts.Compound(faceComp);
        }

        using ProgressScope ps = new(range, "Making offset shape", 100);

        NCollection_Array1<double> steps = new(0, PIOperation_Last - 1);
        AnalyzeProgress(100.0, steps);

        if (!CheckInputData(ps.Next(steps[PIOperation_CheckInputData])) || _error != global::Kerf.Engine.Offsets.Error.NoError)
        {
            // There is error in input data.
            // Check Error() method.
            return;
        }
        _error = global::Kerf.Engine.Offsets.Error.NoError;
        Containment side = In;
        if (_offset < 0.0)
        {
            side = Out;
        }

        // ------------
        // Preanalyse.
        // ------------
        EvalMax(_shape, ref _tol);
        // There are possible second variant: analytical continuation of arcsin.
        double tolAngleCoeff = Math.Min(_tol / (Math.Abs(_offset * 0.5) + global::Kerf.Engine.Foundation.Precision.Confusion()), 1.0);
        double tolAngle = 4 * Math.Asin(tolAngleCoeff);
        if ((_join == global::Kerf.Geometry.JoinKind.Intersection) && _inter && _isPlanar)
        {
            _analyse.SetOffsetValue(_offset);
            _analyse.SetFaceOffsetMap(_faceOffset);
        }
        _analyse.Perform(_faceComp, tolAngle, ps.Next(steps[PIOperation_Analyse]));
        Explorer eExp = new(_faceComp, ShapeType.Edge);
        for (; eExp.More(); eExp.Next())
        {
            Edge e = ShapeCasts.Edge(eExp.Current());
            NCollection_List<OffsetInterval> li = _analyse.Type(e);
            if (li.IsEmpty())
            {
                continue;
            }
            if (li.Last().Type() == TypeOfConcavity.Mixed)
            {
                _error = global::Kerf.Engine.Offsets.Error.MixedConnectivity;
                return;
            }
        }
        if (!ps.More())
        {
            _error = global::Kerf.Engine.Offsets.Error.UserBreak;
            return;
        }
        //---------------------------------------------------
        // Construction of Offset from preanalysis.
        //---------------------------------------------------
        //----------------------------
        // MaJ of SD Face - Offset
        //----------------------------
        UpdateFaceOffset();

        if (_join == Arc)
        {
            BuildOffsetByArc(ps.Next(steps[PIOperation_BuildOffsetBy]));
        }
        else if (_join == global::Kerf.Geometry.JoinKind.Intersection)
        {
            BuildOffsetByInter(ps.Next(steps[PIOperation_BuildOffsetBy]));
        }
        if (_error != global::Kerf.Engine.Offsets.Error.NoError)
        {
            return;
        }
        //-----------------
        // Auto unwinding.
        //-----------------
        // if (mySelfInter)  SelfInter(Modif);
        //-----------------
        // Intersection 3d .
        //-----------------
        // OCCT's aPSInter.SetName(...) names the scope "Connect offset faces by arc" or "... by intersection".
        using ProgressScope psInter = new(ps.Next(steps[PIOperation_Intersection]),
                                                   (_join == Arc) ? "Connect offset faces by arc"
                                                                           : "Connect offset faces by intersection",
                                                   100);

        Inter3d inter = new(_asDes, side, _tol);
        Intersection3D(inter, psInter.Next(90));
        if (_error != global::Kerf.Engine.Offsets.Error.NoError)
        {
            return;
        }
        //-----------------
        // Intersection2D
        //-----------------
        IndexedSet<Shape> modif = inter.TouchedFaces();
        IndexedSet<Shape> newEdges = inter.NewEdges();

        if (modif.Count != 0)
        {
            Intersection2D(modif, newEdges, psInter.Next(4));
            if (_error != global::Kerf.Engine.Offsets.Error.NoError)
            {
                return;
            }
        }

        //-------------------------------------------------------
        // Unwinding 2D and reconstruction of modified faces
        //----------------------------------------------------
        MakeLoops(modif, psInter.Next(4));
        if (_error != global::Kerf.Engine.Offsets.Error.NoError)
        {
            return;
        }
        //-----------------------------------------------------
        // Reconstruction of non modified faces sharing
        // reconstructed edges
        //------------------------------------------------------
        if (modif.Count != 0)
        {
            MakeFaces(modif, psInter.Next(2));
            if (_error != global::Kerf.Engine.Offsets.Error.NoError)
            {
                return;
            }
        }

        psInter.Close();

        if (_thickening)
        {
            MakeMissingWalls(ps.Next(steps[PIOperation_MakeMissingWalls]));
            if (_error != global::Kerf.Engine.Offsets.Error.NoError)
            {
                return;
            }
        }

        //-------------------------
        // Construction of shells.
        //-------------------------
        MakeShells(ps.Next(steps[PIOperation_MakeShells]));
        if (_error != global::Kerf.Engine.Offsets.Error.NoError)
        {
            return;
        }
        if (_offsetShape.IsNull)
        {
            // not done
            _done = false;
            return;
        }
        //--------------
        // Unwinding 3D.
        //--------------
        SelectShells();
        //----------------------------------
        // Remove INTERNAL edges if necessary
        //----------------------------------
        if (_removeIntEdges)
        {
            RemoveInternalEdges();
        }
        //----------------------------------
        // Coding of regularities.
        //----------------------------------
        EncodeRegularity();
        //----------------------------------
        // Replace roots in history maps
        //----------------------------------
        ReplaceRoots();
        //----------------------
        // Creation of solids.
        //----------------------
        MakeSolid(ps.Next(steps[PIOperation_MakeSolid]));
        if (_error != global::Kerf.Engine.Offsets.Error.NoError)
        {
            return;
        }
        //-----------------------------
        // MAJ Tolerance edge and Vertex
        // ----------------------------
        if (!_offsetShape.IsNull)
        {
            if (_thickening)
            {
                UpdateTolerance(ref _offsetShape, _faces, _shape);
            }
            else
            {
                Shape dummy = new();
                UpdateTolerance(ref _offsetShape, _faces, dummy);
            }
            BrepLibrary.UpdateTolerances(_offsetShape);
        }

        CorrectConicalFaces();

        // Result solid should be computed in MakeOffset scope.
        if (_thickening && _isPerformSewing)
        {
            Sewing sew = new(_tol);
            sew.Add(_offsetShape);
            sew.Perform(ps.Next(steps[PIOperation_Sewing] / 2.0));
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            _offsetShape = sew.InternalSewedShape();

            // Rebuild solid.
            // Offset shape expected to be really closed after sewing.
            _offsetShape.Closed(true);
            MakeSolid(ps.Next(steps[PIOperation_Sewing] / 2.0));
            if (_error != global::Kerf.Engine.Offsets.Error.NoError)
            {
                return;
            }
        }

        _done = true;
    }

    //=================================================================================================

    /// <summary>
    /// Makes the thick solid: the offset shell of the shape without its closing faces (<see cref="AddFace"/>), closed
    /// by the shape's faces.
    /// </summary>
    public void MakeThickSolid(ProgressRange range = default)
    {
        using WorkingOverlay? overlay = BeginOverlay();
        InternalMakeThickSolid(range);
        FinishOverlay(overlay);
    }

    private void InternalMakeThickSolid(ProgressRange range)
    {
        //--------------------------------------------------------------
        // Construction of shell parallel to shell (initial without cap).
        //--------------------------------------------------------------
        InternalMakeOffsetShape(range);

        if (!_done)
        {
            // Save return code and myDone state.
            return;
        }

        //--------------------------------------------------------------------
        // Construction of a solid with the initial shell, parallel shell
        // limited by caps.
        //--------------------------------------------------------------------
        if (_faces.Count != 0)
        {
            Solid res = new();
            Explorer exp = new();
            BrepBuilder b = new();
            int nbF = _faces.Count;

            b.MakeSolid(ref res);

            Quilt glue = new();
            for (exp.Init(_shape, ShapeType.Face); exp.More(); exp.Next())
            {
                nbF++;
                glue.Add(exp.Current());
            }
            bool yaResult = false;
            if (!_offsetShape.IsNull)
            {
                for (exp.Init(_offsetShape, ShapeType.Face); exp.More(); exp.Next())
                {
                    yaResult = true;
                    glue.Add(exp.Current().Reversed());
                }
                // OCCT_DEBUG: "OffsetShape does not contain a FACES." left out.
            }
            // OCCT_DEBUG: "OffsetShape is null!" left out.

            if (!yaResult)
            {
                _done = false;
                _error = global::Kerf.Engine.Offsets.Error.UnknownError;
                return;
            }

            _offsetShape = glue.InternalShells();
            for (exp.Init(_offsetShape, ShapeType.Shell); exp.More(); exp.Next())
            {
                b.Add(res, exp.Current());
            }
            res.Closed(true);
            _offsetShape = res;

            // Test of Validity of the result of thick Solid
            // more face than the initial solid.
            int nbOF = 0;
            for (exp.Init(_offsetShape, ShapeType.Face); exp.More(); exp.Next())
            {
                nbOF++;
            }
            if (nbOF < nbF)
            {
                _done = false;
                _error = global::Kerf.Engine.Offsets.Error.UnknownError;
                return;
            }
            if (nbOF == nbF)
            {
                _offset = 0;
            }
        }

        if (_offset > 0)
        {
            _offsetShape.Reverse();
        }

        _done = true;
    }

    //=================================================================================================

    /// <summary>True if the last run succeeded.</summary>
    public bool IsDone => _done;

    //=================================================================================================

    /// <summary>The error of the last run.</summary>
    public Error Error() => _error;

    //=================================================================================================

    /// <summary>The result (frozen).</summary>
    public Shape Shape()
    {
        _offsetShape.TShape()?.FreezeGraph();
        return _offsetShape;
    }

    /// <summary>The result, unfrozen, for ported code.</summary>
    internal Shape InternalShape() => _offsetShape;

    /// <summary>The initial shape, as given.</summary>
    public Shape InitShape() => _initialShape;

    //=================================================================================================

    private void MakeOffsetFaces(Dictionary<Shape, Offset> mapSF,
                                 ProgressRange range)
    {
        double curOffset;
        NCollection_List<Shape> lf = new();
        Dictionary<Shape, Shape> shapeTgt = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLF = new();
        //
        bool offsetOutside = (_offset > 0.0);
        //
        BrepLibrary.SortFaces(_faceComp, lf);
        //
        using ProgressScope ps = new(range, "Making offset faces", lf.Extent());
        itLF.Initialize(lf);
        for (; itLF.More(); itLF.Next(), ps.Next())
        {
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Face f = ShapeCasts.Face(itLF.Value());
            curOffset = _faceOffset.ContainsKey(f) ? _faceOffset.GetValue(f) : _offset;
            Offset of = new(f, curOffset, shapeTgt, offsetOutside, _join);
            NCollection_List<Shape> Let = new();
            _analyse.Edges(f, TypeOfConcavity.Tangential, Let);
            NCollection_List<Shape>.Iterator itl = new(Let);
            for (; itl.More(); itl.Next())
            {
                Edge cur = ShapeCasts.Edge(itl.Value());
                if (!shapeTgt.ContainsKey(cur) && !_analyse.HasGenerated(cur))
                {
                    Shape localShape = of.InternalGenerated(cur);
                    Edge ote = ShapeCasts.Edge(localShape);
                    shapeTgt[cur] = of.InternalGenerated(cur);
                    Exploration.Vertices(cur, out Vertex v1, out Vertex v2);
                    Exploration.Vertices(ote, out Vertex ov1, out Vertex ov2);
                    NCollection_List<Shape> le = new();
                    if (!shapeTgt.ContainsKey(v1))
                    {
                        _analyse.Edges(v1, TypeOfConcavity.Tangential, le);
                        NCollection_List<Shape> la = _analyse.Ancestors(v1);
                        if (le.Extent() == la.Extent())
                        {
                            shapeTgt[v1] = ov1;
                        }
                    }
                    if (!shapeTgt.ContainsKey(v2))
                    {
                        le.Clear();
                        _analyse.Edges(v2, TypeOfConcavity.Tangential, le);
                        NCollection_List<Shape> la = _analyse.Ancestors(v2);
                        if (le.Extent() == la.Extent())
                        {
                            shapeTgt[v2] = ov2;
                        }
                    }
                }
            }
            mapSF[f] = of;
        }
        //
        NCollection_List<Shape> newFaces = _analyse.NewFaces();
        for (NCollection_List<Shape>.Iterator it = new(newFaces); it.More(); it.Next())
        {
            Face f = ShapeCasts.Face(it.Value());
            Offset of = new(f, 0.0, shapeTgt, offsetOutside, _join);
            mapSF[f] = of;
        }
    }

    //=================================================================================================

    private void BuildOffsetByInter(ProgressRange range)
    {
        // OCCT_DEBUG: the chronometer is left out.

        using ProgressScope psOuter = new(range, "Connect offset faces by intersection", 100);

        // just for better management and visualization of the progress steps
        // define a nested enum listing all the steps of the current method.
        const int BuildOffsetByInter_MakeOffsetFaces = 0;
        const int BuildOffsetByInter_ConnexIntByInt = 1;
        const int BuildOffsetByInter_ContextIntByInt = 2;
        const int BuildOffsetByInter_IntersectEdges = 3;
        const int BuildOffsetByInter_CompleteEdgesIntersection = 4;
        const int BuildOffsetByInter_BuildFaces = 5;
        const int BuildOffsetByInter_FillHistoryForOffsets = 6;
        const int BuildOffsetByInter_FillHistoryForDeepenings = 7;
        const int BuildOffsetByInter_Last = 8;

        double nbFaces = _faceComp.ChildCount + _analyse.NewFaces().Extent() + _faces.Count;
        double offsetsPart = (_faceComp.ChildCount + _analyse.NewFaces().Extent()) / nbFaces;
        double deepeningsPart = _faces.Count / nbFaces;

        NCollection_Array1<double> steps = new(0, BuildOffsetByInter_Last - 1);
        {
            steps.Init(0);

            bool isInter = _join == global::Kerf.Geometry.JoinKind.Intersection;
            double faceInter = isInter ? 25.0 : 50.0;
            double buildFaces = isInter ? 50.0 : 25.0;
            steps[BuildOffsetByInter_MakeOffsetFaces] = 5.0;
            steps[BuildOffsetByInter_ConnexIntByInt] = faceInter * offsetsPart;
            steps[BuildOffsetByInter_ContextIntByInt] = faceInter * deepeningsPart;
            steps[BuildOffsetByInter_IntersectEdges] = 10.0;
            steps[BuildOffsetByInter_CompleteEdgesIntersection] = 5.0;
            steps[BuildOffsetByInter_BuildFaces] = buildFaces;
            steps[BuildOffsetByInter_FillHistoryForOffsets] = 5.0 * offsetsPart;
            steps[BuildOffsetByInter_FillHistoryForDeepenings] = 5.0 * deepeningsPart;
            NormalizeSteps(100.0, steps);
        }

        Explorer exp = new(), exp2 = new(), expC = new();
        NCollection_List<Shape>.Iterator itLF = new();

        //--------------------------------------------------------
        // Construction of faces parallel to initial faces
        //--------------------------------------------------------
        Dictionary<Shape, Offset> mapSF = new(ShapeHasher.Instance);
        MakeOffsetFaces(mapSF, psOuter.Next(steps[BuildOffsetByInter_MakeOffsetFaces]));
        if (!psOuter.More())
        {
            _error = global::Kerf.Engine.Offsets.Error.UserBreak;
            return;
        }
        //--------------------------------------------------------------------
        // MES   : Map of OffsetShape -> Extended Shapes.
        // Build : Map of Initial SS  -> OffsetShape build by Inter.
        //                               can be an edge or a compound of edges
        //---------------------------------------------------------------------
        Dictionary<Shape, Shape> mes = new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> build = new(ShapeHasher.Instance);
        NCollection_List<Shape> failed = new();
        Containment side = In;
        AsDes asDes = new();

        //-------------------------------------------------------------------
        // Extension of faces and calculation of new edges of intersection.
        //-------------------------------------------------------------------
        bool extentContext = false;
        if (_offset > 0)
        {
            extentContext = true;
        }

        Inter3d inter3 = new(asDes, side, _tol);
        // Intersection between parallel faces
        inter3.ConnexIntByInt(_faceComp, mapSF, _analyse, mes, build, failed,
                              psOuter.Next(steps[BuildOffsetByInter_ConnexIntByInt]), _isPlanar);
        if (!psOuter.More())
        {
            _error = global::Kerf.Engine.Offsets.Error.UserBreak;
            return;
        }
        // Intersection with caps.
        inter3.ContextIntByInt(_faces, extentContext, mapSF, _analyse, mes, build, failed,
                               psOuter.Next(steps[BuildOffsetByInter_ContextIntByInt]), _isPlanar);
        if (!psOuter.More())
        {
            _error = global::Kerf.Engine.Offsets.Error.UserBreak;
            return;
        }

        NCollection_List<Shape> lFaces = new();
        for (exp.Init(_faceComp, ShapeType.Face); exp.More(); exp.Next())
        {
            lFaces.Append(exp.Current());
        }
        for (NCollection_List<Shape>.Iterator it = new(_analyse.NewFaces()); it.More(); it.Next())
        {
            lFaces.Append(it.Value());
        }
        //---------------------------------------------------------------------------------
        // Extension of neighbor edges of new edges and intersection between neighbors.
        //--------------------------------------------------------------------------------
        AsDes asDes2d = new();
        IntersectEdges(lFaces, mapSF, mes, build, ref asDes, ref asDes2d,
                       psOuter.Next(steps[BuildOffsetByInter_IntersectEdges]));
        if (_error != global::Kerf.Engine.Offsets.Error.NoError)
        {
            return;
        }
        //-----------------------------------------------------------
        // Great restriction of new edges and update of AsDes.
        //------------------------------------------ ----------------
        Dictionary<Shape, NCollection_List<Shape>> edgesOrigins =
            new(ShapeHasher.Instance); // offset edge - initial edges
        IndexedSet<Shape> newEdges = new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> eTrimEInf =
            new(ShapeHasher.Instance); // trimmed - not trimmed edges
        //
        // Map of edges obtained after FACE-FACE (offsetted) intersection.
        // Key1 is edge trimmed by intersection points with other edges;
        // Item is not-trimmed edge.
        if (!TrimEdges(_faceComp, _offset, _analyse, mapSF, mes, build, ref asDes, ref asDes2d, newEdges,
                       eTrimEInf, edgesOrigins))
        {
            _error = global::Kerf.Engine.Offsets.Error.CannotTrimEdges;
            return;
        }
        //
        //---------------------------------
        // Intersection 2D on //
        //---------------------------------
        IndexedMap<Shape, NCollection_List<Shape>> dmvv =
            new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> facesOrigins =
            new(ShapeHasher.Instance); // offset face - initial face
        NCollection_List<Shape> lfe = new();
        ShapeImage imoe = new();
        GetEnlargedFaces(lFaces, mapSF, mes, facesOrigins, imoe, lfe);
        //
        NCollection_List<Shape>.Iterator itLFE = new(lfe);
        using ProgressScope ps2d =
            new(psOuter.Next(steps[BuildOffsetByInter_CompleteEdgesIntersection]), null, 2);
        using ProgressScope ps2dOffsets = new(ps2d.Next(2.0 * offsetsPart), null, lfe.Extent());
        for (; itLFE.More(); itLFE.Next())
        {
            if (!ps2dOffsets.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Face nef = ShapeCasts.Face(itLFE.Value());
            double currFaceTol = Tool.Tolerance(nef);
            Inter2d.Compute(asDes, nef, newEdges, currFaceTol, _edgeIntEdges, dmvv,
                                       ps2dOffsets.Next());
        }
        //----------------------------------------------
        // Intersections 2d on caps.
        //----------------------------------------------
        int i;
        using ProgressScope ps2dCaps = new(ps2d.Next(2.0 * deepeningsPart), null, _faces.Count);
        for (i = 1; i <= _faces.Count; i++)
        {
            if (!ps2dCaps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Face cork = ShapeCasts.Face(_faces[i - 1]);
            double currFaceTol = Tool.Tolerance(cork);
            Inter2d.Compute(asDes, cork, newEdges, currFaceTol, _edgeIntEdges, dmvv, ps2dCaps.Next());
        }
        //
        Inter2d.FuseVertices(dmvv, asDes, _imageVV);
        //-------------------------------
        // Unwinding of extended Faces.
        //-------------------------------
        //
        HashSet<Shape> mfDone = new(ShapeHasher.Instance);
        //
        if ((_join == global::Kerf.Geometry.JoinKind.Intersection) && _inter && _isPlanar)
        {
            BuildSplitsOfExtendedFaces(lfe, _analyse, asDes, edgesOrigins, facesOrigins, eTrimEInf, imoe,
                                       psOuter.Next(steps[BuildOffsetByInter_BuildFaces]));
            if (_error != global::Kerf.Engine.Offsets.Error.NoError)
            {
                return;
            }
            //
            NCollection_List<Shape>.Iterator aItLF = new(lfe);
            for (; aItLF.More(); aItLF.Next())
            {
                Shape s = aItLF.Value();
                mfDone.Add(s);
            }
        }
        else
        {
            _makeLoops.Build(lfe, asDes, imoe, _imageVV, psOuter.Next(steps[BuildOffsetByInter_BuildFaces]));
            if (!psOuter.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
        }
        //
        // OCCT_DEBUG: COES is left out.
        //---------------------------
        // MAJ SD. for faces //
        //---------------------------
        using ProgressScope psHist = new(psOuter.Next(steps[BuildOffsetByInter_FillHistoryForOffsets]),
                                                  "Fill history for offset faces", lFaces.Extent());
        for (NCollection_List<Shape>.Iterator it = new(lFaces); it.More(); it.Next(), psHist.Next())
        {
            if (!psHist.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Shape fi = it.Value();
            _initOffsetFace.SetRoot(fi);
            Face of = mapSF.GetValue(fi).InternalFace();
            if (mes.ContainsKey(of))
            {
                of = ShapeCasts.Face(mes.GetValue(of));
                if (imoe.HasImage(of))
                {
                    NCollection_List<Shape> lofe = imoe.Image(of);
                    _initOffsetFace.Bind(fi, lofe);
                    for (itLF.Initialize(lofe); itLF.More(); itLF.Next())
                    {
                        Shape ofe = itLF.Value();
                        _imageOffset.SetRoot(ofe);
                        HashSet<Shape> view = new(ShapeHasher.Instance);
                        for (exp2.Init(ofe.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
                        {
                            Edge coe = ShapeCasts.Edge(exp2.Current());

                            _asDes.Add(ofe, coe);
                            if (view.Add(coe))
                            {
                                if (!_asDes.HasDescendant(coe))
                                {
                                    Exploration.Vertices(coe, out Vertex cv1, out Vertex cv2);
                                    if (!cv1.IsNull)
                                    {
                                        _asDes.Add(coe, cv1.Oriented(Forward));
                                    }
                                    if (!cv2.IsNull)
                                    {
                                        _asDes.Add(coe, cv2.Oriented(global::Kerf.Topology.Orientation.Reversed));
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (mfDone.Contains(of))
                    {
                        continue;
                    }
                    //
                    _initOffsetFace.Bind(fi, of);
                    _imageOffset.SetRoot(of);
                    NCollection_List<Shape> le = asDes.Descendant(of);
                    for (itLF.Initialize(le); itLF.More(); itLF.Next())
                    {
                        Edge oe = ShapeCasts.Edge(itLF.Value());
                        if (imoe.HasImage(oe))
                        {
                            NCollection_List<Shape> loe = imoe.Image(oe);
                            NCollection_List<Shape>.Iterator itLOE = new(loe);
                            for (; itLOE.More(); itLOE.Next())
                            {
                                Shape localShape = itLOE.Value().Oriented(oe.Orientation);
                                Edge coe = ShapeCasts.Edge(localShape);
                                //              const TopoDS_Edge& COE =
                                //              TopoDS::Edge(itLOE.Value().Oriented(OE.Orientation()));
                                _asDes.Add(of, coe);

                                if (!_asDes.HasDescendant(coe))
                                {
                                    Exploration.Vertices(coe, out Vertex cv1, out Vertex cv2);
                                    if (!cv1.IsNull)
                                    {
                                        _asDes.Add(coe, cv1.Oriented(Forward));
                                    }
                                    if (!cv2.IsNull)
                                    {
                                        _asDes.Add(coe, cv2.Oriented(global::Kerf.Topology.Orientation.Reversed));
                                    }
                                }
                            }
                        }
                        else
                        {
                            _asDes.Add(of, oe);

                            NCollection_List<Shape> lv = asDes.Descendant(oe);
                            _asDes.Add(oe, lv);
                        }
                    }
                }
            }
            else
            {
                _initOffsetFace.Bind(fi, of);
                _imageOffset.SetRoot(of);
                HashSet<Shape> view = new(ShapeHasher.Instance);
                for (exp2.Init(of.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
                {

                    Edge coe = ShapeCasts.Edge(exp2.Current());
                    _asDes.Add(of, coe);

                    if (view.Add(exp2.Current()))
                    {
                        if (!_asDes.HasDescendant(coe))
                        {
                            Exploration.Vertices(coe, out Vertex cv1, out Vertex cv2);
                            if (!cv1.IsNull)
                            {
                                _asDes.Add(coe, cv1.Oriented(Forward));
                            }
                            if (!cv2.IsNull)
                            {
                                _asDes.Add(coe, cv2.Oriented(global::Kerf.Topology.Orientation.Reversed));
                            }
                        }
                    }
                }
            }
        }
        //  Modified by skv - Tue Mar 15 16:20:43 2005
        // Add methods for supporting history.
        HashSet<Shape> mapEdges = new(ShapeHasher.Instance);

        for (NCollection_List<Shape>.Iterator it = new(lFaces); it.More(); it.Next())
        {
            Shape faceRef = it.Value();
            exp2.Init(faceRef.Oriented(Forward), ShapeType.Edge);

            for (; exp2.More(); exp2.Next())
            {
                Shape edgeRef = exp2.Current();

                if (mapEdges.Add(edgeRef))
                {
                    _initOffsetEdge.SetRoot(edgeRef);
                    if (build.ContainsKey(edgeRef))
                    {
                        Shape newShape = build.GetValue(edgeRef);

                        if (newShape.Type == ShapeType.Edge)
                        {
                            if (imoe.HasImage(newShape))
                            {
                                NCollection_List<Shape> listNewE = imoe.Image(newShape);

                                _initOffsetEdge.Bind(edgeRef, listNewE);
                            }
                            else
                            {
                                _initOffsetEdge.Bind(edgeRef, newShape);
                            }
                        }
                        else
                        { // aNewShape != TopAbs_EDGE
                            NCollection_List<Shape> listNewEdge = new();

                            for (expC.Init(newShape, ShapeType.Edge); expC.More(); expC.Next())
                            {
                                Shape resEdge = expC.Current();

                                if (imoe.HasImage(resEdge))
                                {
                                    NCollection_List<Shape> listNewE = imoe.Image(resEdge);
                                    NCollection_List<Shape>.Iterator newEIter = new(listNewE);

                                    for (; newEIter.More(); newEIter.Next())
                                    {
                                        listNewEdge.Append(newEIter.Value());
                                    }
                                }
                                else
                                {
                                    listNewEdge.Append(resEdge);
                                }
                            }

                            _initOffsetEdge.Bind(edgeRef, listNewEdge);
                        }
                    }
                    else
                    { // Free boundary.
                        Shape newEdge = mapSF.GetValue(faceRef).InternalGenerated(edgeRef);

                        if (mes.ContainsKey(newEdge))
                        {
                            newEdge = mes.GetValue(newEdge);
                        }

                        if (imoe.HasImage(newEdge))
                        {
                            NCollection_List<Shape> listNewE = imoe.Image(newEdge);

                            _initOffsetEdge.Bind(edgeRef, listNewE);
                        }
                        else
                        {
                            _initOffsetEdge.Bind(edgeRef, newEdge);
                        }
                    }
                }
            }
        }
        //  Modified by skv - Tue Mar 15 16:20:43 2005

        //---------------------------
        // MAJ SD. for caps
        //---------------------------
        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> View;
        using ProgressScope psHist2 = new(psOuter.Next(steps[BuildOffsetByInter_FillHistoryForDeepenings]),
                                                   "Fill history for deepening faces", _faces.Count);
        for (i = 1; i <= _faces.Count; i++, psHist2.Next())
        {
            if (!psHist2.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Shape cork = _faces[i - 1];
            NCollection_List<Shape> le = asDes.Descendant(cork);
            for (itLF.Initialize(le); itLF.More(); itLF.Next())
            {
                Edge oe = ShapeCasts.Edge(itLF.Value());
                if (imoe.HasImage(oe))
                {
                    NCollection_List<Shape> loe = imoe.Image(oe);
                    NCollection_List<Shape>.Iterator itLOE = new(loe);
                    for (; itLOE.More(); itLOE.Next())
                    {
                        Edge coe = ShapeCasts.Edge(itLOE.Value());
                        _asDes.Add(cork, coe.Oriented(oe.Orientation));

                        if (!_asDes.HasDescendant(coe))
                        {
                            Exploration.Vertices(coe, out Vertex cv1, out Vertex cv2);
                            if (!cv1.IsNull)
                            {
                                _asDes.Add(coe, cv1.Oriented(Forward));
                            }
                            if (!cv2.IsNull)
                            {
                                _asDes.Add(coe, cv2.Oriented(global::Kerf.Topology.Orientation.Reversed));
                            }
                        }
                    }
                }
                else
                {
                    _asDes.Add(cork, oe);
                    if (asDes.HasDescendant(oe))
                    {
                        _asDes.Add(oe, asDes.Descendant(oe));
                    }
                }
            }
        }

        // OCCT_DEBUG: DEBVerticesControl and the chronometer are left out.
    }

    //=================================================================================================

    private void ReplaceRoots()
    {
        // Replace the artificial faces and edges in InitOffset maps with the original ones.
        HashSet<Shape> view = new(ShapeHasher.Instance);
        for (Explorer expF = new(_faceComp, ShapeType.Edge); expF.More(); expF.Next())
        {
            Shape f = expF.Current();
            for (Explorer expE = new(f, ShapeType.Edge); expE.More(); expE.Next())
            {
                Shape e = expE.Current();
                if (!view.Add(e))
                {
                    continue;
                }

                Shape fGen = _analyse.Generated(e);
                if (fGen.IsNull)
                {
                    continue;
                }

                _initOffsetFace.ReplaceRoot(fGen, e);

                for (Iterator itV = new(e); itV.More(); itV.Next())
                {
                    Shape v = itV.Value();
                    if (!view.Add(v))
                    {
                        continue;
                    }

                    Shape eGen = _analyse.Generated(v);
                    if (eGen.IsNull)
                    {
                        continue;
                    }

                    _initOffsetEdge.ReplaceRoot(eGen, v);
                }
            }
        }
    }

    //=======================================================================
    // function : BuildFaceComp
    // purpose  : Make a compound containing actual faces (including planar faces instead of their
    // originals)
    //=======================================================================
    private void BuildFaceComp()
    {
        BrepBuilder bb = new();
        bb.MakeCompound(ref _faceComp);
        Explorer explo = new(_shape, ShapeType.Face);
        for (; explo.More(); explo.Next())
        {
            Shape face = explo.Current();
            Orientation or = face.Orientation;
            if (_facePlanfaceMap.TryGetValue(face, out Shape planface))
            {
                face = planface;
            }
            bb.Add(_faceComp, face.Oriented(or));
        }
    }

    //=================================================================================================

    private void BuildOffsetByArc(ProgressRange range)
    {
        // OCCT_DEBUG: the chronometer is left out.

        Explorer exp = new();
        NCollection_List<Shape>.Iterator itLF = new();
        HashSet<Shape> done = new(ShapeHasher.Instance);
        using ProgressScope psOuter = new(range, null, 10);
        //--------------------------------------------------------
        // Construction of faces parallel to initial faces
        //--------------------------------------------------------
        Dictionary<Shape, Offset> mapSF = new(ShapeHasher.Instance);
        MakeOffsetFaces(mapSF, psOuter.Next());
        if (_error != global::Kerf.Engine.Offsets.Error.NoError)
        {
            return;
        }
        //--------------------------------------------------------
        // Construction of tubes on edge.
        //--------------------------------------------------------
        TypeOfConcavity ot = TypeOfConcavity.Convex;
        if (_offset < 0.0)
        {
            ot = TypeOfConcavity.Concave;
        }

        using ProgressScope ps1 = new(psOuter.Next(4), "Constructing tubes on edges", 1, true);
        for (exp.Init(_faceComp, ShapeType.Edge); exp.More(); exp.Next(), ps1.Next())
        {
            if (!ps1.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Edge e = ShapeCasts.Edge(exp.Current());
            if (done.Add(e))
            {
                NCollection_List<Shape> anc = _analyse.Ancestors(e);
                if (anc.Extent() == 2)
                {
                    NCollection_List<OffsetInterval> l = _analyse.Type(e);
                    if (!l.IsEmpty() && l.First().Type() == ot)
                    {
                        double curOffset = _offset;
                        if (_faceOffset.ContainsKey(anc.First()))
                        {
                            curOffset = _faceOffset.GetValue(anc.First());
                        }
                        Shape localShapeGen = mapSF.GetValue(anc.First()).InternalGenerated(e);
                        Edge eOn1 = ShapeCasts.Edge(localShapeGen);
                        localShapeGen = mapSF.GetValue(anc.Last()).InternalGenerated(e);
                        Edge eOn2 = ShapeCasts.Edge(localShapeGen);
                        //          TopoDS_Edge EOn1 = TopoDS::Edge(MapSF(Anc.First()).Generated(E));
                        //          TopoDS_Edge EOn2 = TopoDS::Edge(MapSF(Anc.Last()) .Generated(E));
                        // find if exits tangent edges in the original shape
                        Edge e1f = new(), e1l = new();
                        Exploration.Vertices(e, out Vertex v1f, out Vertex v1l);
                        NCollection_List<Shape> tangE = new();
                        _analyse.TangentEdges(e, v1f, tangE);
                        // find if the pipe on the tangent edges are soon created.
                        NCollection_List<Shape>.Iterator itl = new(tangE);
                        bool find = false;
                        for (; itl.More() && !find; itl.Next())
                        {
                            if (mapSF.ContainsKey(itl.Value()))
                            {
                                Shape localShape = mapSF.GetValue(itl.Value()).InternalGenerated(v1f);
                                e1f = ShapeCasts.Edge(localShape);
                                //              E1f  = TopoDS::Edge(MapSF(itl.Value()).Generated(V1f));
                                find = true;
                            }
                        }
                        tangE.Clear();
                        _analyse.TangentEdges(e, v1l, tangE);
                        // find if the pipe on the tangent edges are soon created.
                        itl.Initialize(tangE);
                        find = false;
                        for (; itl.More() && !find; itl.Next())
                        {
                            if (mapSF.ContainsKey(itl.Value()))
                            {
                                Shape localShape = mapSF.GetValue(itl.Value()).InternalGenerated(v1l);
                                e1l = ShapeCasts.Edge(localShape);
                                //              E1l  = TopoDS::Edge(MapSF(itl.Value()).Generated(V1l));
                                find = true;
                            }
                        }
                        Offset of = new(e, eOn1, eOn2, curOffset, e1f, e1l);
                        mapSF[e] = of;
                    }
                }
                else
                {
                    // ----------------------
                    // free border.
                    // ----------------------
                    Shape localShape = mapSF.GetValue(anc.First()).InternalGenerated(e);
                    Edge eOn1 = ShapeCasts.Edge(localShape);
                    //        TopoDS_Edge EOn1 = TopoDS::Edge(MapSF(Anc.First()).Generated(E));
                    _initOffsetEdge.SetRoot(e); // skv: supporting history.
                    _initOffsetEdge.Bind(e, eOn1);
                }
            }
        }

        //--------------------------------------------------------
        // Construction of spheres on vertex.
        //--------------------------------------------------------
        done.Clear();
        NCollection_List<Shape>.Iterator it = new();
        using ProgressScope ps2 = new(psOuter.Next(4), "Constructing spheres on vertices", 1, true);
        for (exp.Init(_faceComp, ShapeType.Vertex); exp.More(); exp.Next(), ps2.Next())
        {
            if (!ps2.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Vertex v = ShapeCasts.Vertex(exp.Current());
            if (done.Add(v))
            {
                NCollection_List<Shape> la = _analyse.Ancestors(v);
                NCollection_List<Shape> le = new();
                _analyse.Edges(v, ot, le);

                if (le.Extent() >= 3 && le.Extent() == la.Extent())
                {
                    NCollection_List<Shape> loe = new();
                    //--------------------------------------------------------
                    // Return connected edges on tubes.
                    //--------------------------------------------------------
                    for (it.Initialize(le); it.More(); it.Next())
                    {
                        loe.Append(mapSF.GetValue(it.Value()).InternalGenerated(v).Reversed());
                    }
                    //----------------------
                    // construction sphere.
                    //-----------------------
                    NCollection_List<Shape> lla = _analyse.Ancestors(la.First());
                    Shape ff = lla.First();
                    double curOffset = _offset;
                    if (_faceOffset.ContainsKey(ff))
                    {
                        curOffset = _faceOffset.GetValue(ff);
                    }

                    Offset of = new(v, loe, curOffset);
                    mapSF[v] = of;
                }
                //--------------------------------------------------------------
                // Particular processing if V is at least a free border.
                //-------------------------------------------------------------
                NCollection_List<Shape> lbf = new();
                _analyse.Edges(v, TypeOfConcavity.FreeBound, lbf);
                if (!lbf.IsEmpty())
                {
                    bool first = true;
                    for (it.Initialize(le); it.More(); it.Next())
                    {
                        if (first)
                        {
                            _initOffsetEdge.SetRoot(v); // skv: supporting history.
                            _initOffsetEdge.Bind(v, mapSF.GetValue(it.Value()).InternalGenerated(v));
                            first = false;
                        }
                        else
                        {
                            _initOffsetEdge.Add(v, mapSF.GetValue(it.Value()).InternalGenerated(v));
                        }
                    }
                }
            }
        }

        //------------------------------------------------------------
        // Extension of parallel faces to the context.
        // Extended faces are ordered in DS and removed from MapSF.
        //------------------------------------------------------------
        if (_faces.Count != 0)
        {
            ToContext(mapSF);
        }

        //------------------------------------------------------
        // MAJ SD.
        //------------------------------------------------------
        TypeOfConcavity rt = TypeOfConcavity.Concave;
        if (_offset < 0.0)
        {
            rt = TypeOfConcavity.Convex;
        }
        // OCCT.NET: OCCT iterates MapSF by its shapes' addresses; the port in insertion order (decision 1032).
        using ProgressScope ps3 = new(psOuter.Next(), null, mapSF.Count);
        foreach (KeyValuePair<Shape, Offset> It in mapSF)
        {
            if (!ps3.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Shape si = It.Key;
            Offset sf = It.Value;
            if (sf.Status() == OffsetStatus.Reversed || sf.Status() == OffsetStatus.Degenerated)
            {
                //------------------------------------------------
                // Degenerated or returned faces are not stored.
                //------------------------------------------------
                ps3.Next();
                continue;
            }

            Face of = It.Value.InternalFace();
            _initOffsetFace.Bind(si, of);
            _initOffsetFace.SetRoot(si); // Initial<-> Offset
            _imageOffset.SetRoot(of);    // FaceOffset root of images

            if (si.Type == ShapeType.Face)
            {
                for (exp.Init(si.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
                {
                    //--------------------------------------------------------------------
                    // To each face are associatedthe edges that restrict that
                    // The edges that do not generate tubes or are not tangent
                    // to two faces are removed.
                    //--------------------------------------------------------------------
                    Edge e = ShapeCasts.Edge(exp.Current());
                    NCollection_List<OffsetInterval> l = _analyse.Type(e);
                    if (!l.IsEmpty() && l.First().Type() != rt)
                    {
                        Orientation oo = e.Orientation;
                        Shape localShape = It.Value.InternalGenerated(e);
                        Edge oe = ShapeCasts.Edge(localShape);
                        //          TopoDS_Edge        OE  = TopoDS::Edge(It.Value().Generated(E));
                        _asDes.Add(of, oe.Oriented(oo));
                    }
                }
            }
            else
            {
                for (exp.Init(of.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
                {
                    _asDes.Add(of, exp.Current());
                }
            }
            ps3.Next();
        }

        // OCCT_DEBUG: the chronometer is left out.
    }

    //=================================================================================================

    private void SelfInter(HashSet<Shape> modif)
    {
        // OCCT_DEBUG: the chronometer is left out.

        throw new MethodNotImplementedException();
    }

    //=================================================================================================

    private void ToContext(Dictionary<Shape, Offset> mapSF)
    {
        Dictionary<Shape, Shape> created = new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> mef = new(ShapeHasher.Instance);
        IndexedSet<Shape> facesToBuild = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itl = new();
        Explorer exp = new();

        //  TopAbs_State       Side = TopAbs_IN;
        //  if (myOffset < 0.) Side = TopAbs_OUT;

        Containment side = Out;

        // OCCT has a commented-out loop here over the context faces' edges not in the initial shape.

        //--------------------------------------------------------
        // Determine the edges and faces reconstructed by
        // intersection.
        //---------------------------------------------------------
        int j;
        for (j = 1; j <= _faces.Count; j++)
        {
            Face cf = ShapeCasts.Face(_faces[j - 1]);
            for (exp.Init(cf.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                if (_analyse.HasAncestor(e))
                {
                    NCollection_List<Shape> lea = _analyse.Ancestors(e);
                    for (itl.Initialize(lea); itl.More(); itl.Next())
                    {
                        Offset of = mapSF.GetValue(itl.Value());
                        facesToBuild.Add(itl.Value());
                        mef[of.InternalGenerated(e)] = cf;
                    }
                    Vertex[] v = new Vertex[2];
                    Exploration.Vertices(e, out v[0], out v[1]);
                    for (int i = 0; i < 2; i++)
                    {
                        NCollection_List<Shape> lva = _analyse.Ancestors(v[i]);
                        for (itl.Initialize(lva); itl.More(); itl.Next())
                        {
                            Edge ev = ShapeCasts.Edge(itl.Value());
                            if (mapSF.ContainsKey(ev))
                            {
                                Offset of = mapSF.GetValue(ev);
                                facesToBuild.Add(ev);
                                mef[of.InternalGenerated(v[i])] = cf;
                            }
                        }
                    }
                }
            }
        }
        //---------------------------
        // Reconstruction of faces.
        //---------------------------
        Face f, nf = new();
        TypeOfConcavity rt = TypeOfConcavity.Concave;
        if (_offset < 0.0)
        {
            rt = TypeOfConcavity.Convex;
        }
        Shape oe, ne;
        Orientation or;

        for (j = 1; j <= facesToBuild.Count; j++)
        {
            Shape s = facesToBuild[j - 1];
            // OCCT copies the offset (a value type); the port's is only read here, and unbound below.
            Offset bof = mapSF.GetValue(s);
            f = ShapeCasts.Face(bof.InternalFace());
            OffsetTool.ExtentFace(f, created, mef, side, _tol, ref nf);
            mapSF.Remove(s);
            //--------------
            // MAJ SD.
            //--------------
            _initOffsetFace.Bind(s, nf);
            _initOffsetFace.SetRoot(s); // Initial<-> Offset
            _imageOffset.SetRoot(nf);

            if (s.Type == ShapeType.Face)
            {
                for (exp.Init(s.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
                {

                    Edge e = ShapeCasts.Edge(exp.Current());
                    NCollection_List<OffsetInterval> l = _analyse.Type(e);
                    oe = bof.InternalGenerated(e);
                    or = e.Orientation;
                    oe.Orient(or);
                    if (!l.IsEmpty() && l.First().Type() != rt)
                    {
                        if (created.ContainsKey(oe))
                        {
                            ne = created.GetValue(oe);
                            if (ne.Orientation == global::Kerf.Topology.Orientation.Reversed)
                            {
                                ne.Orient(global::Kerf.Engine.Topology.TopAbs.Reverse(or));
                            }
                            else
                            {
                                ne.Orient(or);
                            }
                            _asDes.Add(nf, ne);
                        }
                        else
                        {
                            _asDes.Add(nf, oe);
                        }
                    }
                }
            }
            else
            {
                //------------------
                // Tube
                //---------------------
                for (exp.Init(nf.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
                {
                    _asDes.Add(nf, exp.Current());
                }
            }
            mapSF.Remove(s);
        }

        //------------------
        // MAJ free borders
        //------------------
        // OCCT.NET: OCCT iterates Created by its edges' addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<Shape, Shape> itc in created)
        {
            oe = itc.Key;
            ne = itc.Value;
            if (_initOffsetEdge.IsImage(oe))
            {
                Shape e = _initOffsetEdge.ImageFrom(oe);
                or = _initOffsetEdge.Image(e).First().Orientation;
                if (ne.Orientation == global::Kerf.Topology.Orientation.Reversed)
                {
                    ne.Orient(global::Kerf.Engine.Topology.TopAbs.Reverse(or));
                }
                else
                {
                    ne.Orient(or);
                }
                _initOffsetEdge.Remove(oe);
                _initOffsetEdge.Bind(e, ne);
            }
        }
    }

    //=================================================================================================

    private void UpdateFaceOffset()
    {
        HashSet<Shape> m = new(ShapeHasher.Instance);
        Dictionary<Shape, double> copiedMap = new(_faceOffset,_faceOffset.Comparer);

        TypeOfConcavity rt = TypeOfConcavity.Convex;
        if (_offset < 0.0)
        {
            rt = TypeOfConcavity.Concave;
        }

        // OCCT.NET: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<Shape, double> it in copiedMap)
        {
            Face f = ShapeCasts.Face(it.Key);
            double curOffset = copiedMap.GetValue(f);
            if (!m.Add(f))
            {
                continue;
            }
            Compound co = new();
            BrepBuilder build = new();
            build.MakeCompound(ref co);
            HashSet<Shape> dummy = new(ShapeHasher.Instance);
            build.Add(co, f);
            if (_join == Arc)
            {
                _analyse.AddFaces(f, co, dummy, TypeOfConcavity.Tangential, rt);
            }
            else
            {
                _analyse.AddFaces(f, co, dummy, TypeOfConcavity.Tangential);
            }

            Explorer exp = new(co, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Face ff = ShapeCasts.Face(exp.Current());
                if (!m.Add(ff))
                {
                    continue;
                }
                if (_faceOffset.ContainsKey(ff))
                {
                    _faceOffset.Remove(ff);
                }
                _faceOffset[ff] = curOffset;
            }
        }
    }

    //=================================================================================================

    private void CorrectConicalFaces()
    {
        if (_offsetShape.IsNull)
        {
            return;
        }
        //
        NCollection_Sequence<Shape> cones = new();
        NCollection_Sequence<Shape> circs = new();
        NCollection_Sequence<Shape> seams = new();
        double tolApex = 1.0e-5;

        int i;

        Dictionary<Shape, NCollection_List<Shape>> facesOfCone = new(ShapeHasher.Instance);
        // NCollection_DataMap<TopoDS_Shape, TopoDS_Shape, TopTools_ShapeMapHasher> DegEdges;
        Explorer explo = new(_offsetShape, ShapeType.Face);
        if (_join == Arc)
        {
            for (; explo.More(); explo.Next())
            {
                Face face = ShapeCasts.Face(explo.Current());
                Surface? surf = Tool.Surface(face);
                // if (aSurf->DynamicType() == STANDARD_TYPE(Geom_OffsetSurface))
                // aSurf = (occ::down_cast<Geom_OffsetSurface>(aSurf))->BasisSurface(); //???

                IndexedSet<Shape> emap = new(ShapeHasher.Instance);
                Exploration.MapShapes(face, ShapeType.Edge, emap);
                for (i = 1; i <= emap.Count; i++)
                {
                    Edge edge = ShapeCasts.Edge(emap[i - 1]);
                    // double f, l;
                    // occ::handle<Geom_Curve> theCurve = BRep_Tool::Curve( anEdge, f, l );
                    // occ::handle<BRep_TEdge>& TE = *((occ::handle<BRep_TEdge>*) &anEdge.TShape());
                    if (Tool.Degenerated(edge))
                    {
                        // Check if anEdge is a really degenerated edge or not
                        EdgeCurveAdaptor baCurve = new(edge, face);
                        Point3D pfirst, plast, pmid;
                        pfirst = baCurve.Value(baCurve.FirstParameter());
                        plast = baCurve.Value(baCurve.LastParameter());
                        pmid = baCurve.Value((baCurve.FirstParameter() + baCurve.LastParameter()) / 2.0);
                        if (pfirst.DistanceTo(plast) <= tolApex && pfirst.DistanceTo(pmid) <= tolApex)
                        {
                            continue;
                        }
                        // Cones.Append( aFace );
                        // Circs.Append( anEdge );
                        // TopoDS_Vertex Vdeg = TopExp::FirstVertex( anEdge );
                        Edge orEdge = ShapeCasts.Edge(_initOffsetEdge.Root(edge));
                        Vertex VF = Exploration.FirstVertex(orEdge);
                        if (facesOfCone.ContainsKey(VF))
                        {
                            // add a face to the existing list
                            NCollection_List<Shape> aFaces = facesOfCone.GetValue(VF);
                            aFaces.Append(face);
                            // DegEdges.Bind(aFace, anEdge);
                        }
                        else
                        {
                            // the vertex is not in the map => create a new key and items
                            NCollection_List<Shape> aFaces = new();
                            aFaces.Append(face);
                            facesOfCone[VF] = aFaces;
                            // DegEdges.Bind(aFace, anEdge);
                        }
                    }
                } // for (i = 1; i <= Emap.Extent(); i++)
            } // for (; fexp.More(); fexp.Next())
        } // if (myJoin == GeomAbs_Arc)

        BrepBuilder bb = new();
        Placement L = new();
        bool isModified = false;
        // OCCT.NET: OCCT iterates the apexes by their addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> cone in facesOfCone)
        {
            Sphere theSphere = new();
            SphericalSurface? sphSurf = null;
            Wire sphereWire = new();
            bb.MakeWire(ref sphereWire);
            Vertex apex = ShapeCasts.Vertex(cone.Key);
            NCollection_List<Shape> faces = cone.Value; // FacesOfCone(anApex);
            NCollection_List<Shape>.Iterator itFaces = new(faces);
            bool isFirstFace = true;
            Point3D firstPoint = new();
            Vertex theFirstVertex = new(), curFirstVertex = new();
            for (; itFaces.More(); itFaces.Next())
            {
                Face face = ShapeCasts.Face(itFaces.Value()); // TopoDS::Face(Faces.First());
                Edge degEdge = new();                         // = TopoDS::Edge(DegEdges(aFace));
                for (explo.Init(face, ShapeType.Edge); explo.More(); explo.Next())
                {
                    degEdge = ShapeCasts.Edge(explo.Current());
                    if (Tool.Degenerated(degEdge))
                    {
                        Edge orEdge = ShapeCasts.Edge(_initOffsetEdge.Root(degEdge));
                        Vertex VF = Exploration.FirstVertex(orEdge);
                        if (VF.IsSame(apex))
                        {
                            break;
                        }
                    }
                }
                Shape localDegShape = degEdge.Oriented(Forward);
                Edge curEdge = ShapeCasts.Edge(localDegShape);
                bb.Degenerated(curEdge, false);
                bb.SameRange(curEdge, false);
                bb.SameParameter(curEdge, false);
                GetEdgePoints(curEdge, face, out Point3D fPnt, out Point3D mPnt, out Point3D lPnt);
                Tool.Range(curEdge, out double f, out double l);
                if (isFirstFace)
                {
                    Vector3D vec1 = new(fPnt, mPnt);
                    Vector3D vec2 = new(fPnt, lPnt);
                    Vector3D norm = vec1.Crossed(vec2);
                    Point3D theApex = Tool.Pnt(apex);
                    Vector3D apexToFpnt = new(theApex, fPnt);
                    Vector3D ydir = norm ^ apexToFpnt;
                    Vector3D xdir = ydir ^ norm;
                    // Xdir.Rotate(gp_Ax1(theApex, aNorm), -f);
                    Frame3D ax2 = new(theApex, new Direction3D(norm), new Direction3D(xdir));
                    theSphere = theSphere.WithRadius(_offset);
                    theSphere = theSphere.WithPosition(new CoordinateSystem(ax2) /*gp_Ax3(theApex, gp_Dir(aNorm))*/);
                    sphSurf = new SphericalSurface(theSphere);
                    firstPoint = fPnt;
                    theFirstVertex = new LibraryMakeVertex(fPnt).InternalVertex();
                    curFirstVertex = theFirstVertex;
                }

                Vertex firstVert, endVert;
                Exploration.Vertices(curEdge, out Vertex v1, out Vertex v2);
                firstVert = curFirstVertex;
                if (lPnt.DistanceTo(firstPoint) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    endVert = theFirstVertex;
                }
                else
                {
                    endVert = new LibraryMakeVertex(lPnt).InternalVertex();
                }
                curEdge.Free(true);
                bb.Remove(curEdge, v1);
                bb.Remove(curEdge, v2);
                bb.Add(curEdge, firstVert.Oriented(Forward));
                bb.Add(curEdge, endVert.Oriented(global::Kerf.Topology.Orientation.Reversed));
                // take the curve from sphere an put it to the edge
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theSphere, fPnt, out double uf, out double vf);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theSphere, lPnt, out double ul, out double vl);
                if (Math.Abs(ul) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    ul = 2.0 * M_PI;
                }
                Curve3D curv = sphSurf!.VIso(vf);
                // OCCT has a commented-out rotation of the circle for the faces after the first.
                TrimmedCurve3D trimCurv = new(curv, uf, ul);
                bb.UpdateEdge(curEdge, trimCurv, global::Kerf.Engine.Foundation.Precision.Confusion());
                bb.Range(curEdge, uf, ul, true);
                LineCurve2D theLin2d = new(new Point2D(0.0, vf), Constants.DX2d());
                TrimmedCurve2D theTrimLin2d = new(theLin2d, uf, ul);
                bb.UpdateEdge(curEdge, theTrimLin2d, sphSurf, L, global::Kerf.Engine.Foundation.Precision.Confusion());
                bb.Range(curEdge, sphSurf, L, uf, ul);
                BrepLibrary.SameParameter(curEdge);
                bb.Add(sphereWire, curEdge);
                // Modifying correspondent edges in aFace: substitute vertices common with CurEdge
                EdgeCurve2DAdaptor bAc2d = new(curEdge, face);
                Point2D fPnt2d, lPnt2d;
                fPnt2d = bAc2d.Value(bAc2d.FirstParameter());
                lPnt2d = bAc2d.Value(bAc2d.LastParameter());
                IndexedSet<Shape> emap = new(ShapeHasher.Instance);
                Exploration.MapShapes(face, ShapeType.Edge, emap);
                Edge[] ee = [new(), new()];
                int j = 0, k;
                for (k = 1; k <= emap.Count; k++)
                {
                    Edge edge = ShapeCasts.Edge(emap[k - 1]);
                    if (!Tool.Degenerated(edge))
                    {
                        Exploration.Vertices(edge, out Vertex v1e, out Vertex v2e);
                        if (v1e.IsSame(v1) || v2e.IsSame(v1))
                        {
                            ee[j++] = edge;
                        }
                    }
                }
                for (k = 0; k < j; k++)
                {
                    Shape localShape = ee[k].Oriented(Forward);
                    Edge eforward = ShapeCasts.Edge(localShape);
                    eforward.Free(true);
                    Exploration.Vertices(eforward, out Vertex v1e, out Vertex v2e);
                    EdgeCurve2DAdaptor eEc = new(eforward, face);
                    Point2D p2d1_, p2d2_;
                    p2d1_ = eEc.Value(eEc.FirstParameter());
                    p2d2_ = eEc.Value(eEc.LastParameter());
                    if (v1e.IsSame(v1))
                    {
                        Vertex newV = (p2d1_.DistanceTo(fPnt2d) <= global::Kerf.Engine.Foundation.Precision.Confusion()) ? firstVert : endVert;
                        bb.Remove(eforward, v1e);
                        bb.Add(eforward, newV.Oriented(Forward));
                    }
                    else
                    {
                        Vertex newV = (p2d2_.DistanceTo(fPnt2d) <= global::Kerf.Engine.Foundation.Precision.Confusion()) ? firstVert : endVert;
                        bb.Remove(eforward, v2e);
                        bb.Add(eforward, newV.Oriented(global::Kerf.Topology.Orientation.Reversed));
                    }
                }

                isFirstFace = false;
                curFirstVertex = endVert;
            }
            // Building new spherical face
            double ufirst = RealLast(), ulast = RealFirst();
            Point2D p2d1 = new(), p2d2 = new();
            NCollection_List<Shape> edgesOfWire = new();
            Iterator itw = new(sphereWire);
            for (; itw.More(); itw.Next())
            {
                Edge edge = ShapeCasts.Edge(itw.Value());
                edgesOfWire.Append(edge);
                Curve2D c2d = Tool.CurveOnSurface(edge, sphSurf, L, out double f, out double l)!;
                p2d1 = c2d.Value(f);
                p2d2 = c2d.Value(l);
                if (p2d1.X < ufirst)
                {
                    ufirst = p2d1.X;
                }
                if (p2d1.X > ulast)
                {
                    ulast = p2d1.X;
                }
                if (p2d2.X < ufirst)
                {
                    ufirst = p2d2.X;
                }
                if (p2d2.X > ulast)
                {
                    ulast = p2d2.X;
                }
            }
            NCollection_List<Shape> newEdges = new();
            Edge firstEdge = new();
            NCollection_List<Shape>.Iterator itl = new(edgesOfWire);
            for (; itl.More(); itl.Next())
            {
                firstEdge = ShapeCasts.Edge(itl.Value());
                Curve2D c2d = Tool.CurveOnSurface(firstEdge, sphSurf, L, out double f, out double l)!;
                p2d1 = c2d.Value(f);
                p2d2 = c2d.Value(l);
                if (Math.Abs(p2d1.X - ufirst) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    edgesOfWire.Remove(itl);
                    break;
                }
            }
            newEdges.Append(firstEdge.Oriented(Forward));
            Exploration.Vertices(firstEdge, out Vertex vf1, out Vertex curVertex);
            itl.Initialize(edgesOfWire);
            while (itl.More())
            {
                Edge edge = ShapeCasts.Edge(itl.Value());
                Exploration.Vertices(edge, out Vertex V1_, out Vertex V2_);
                if (V1_.IsSame(curVertex) || V2_.IsSame(curVertex))
                {
                    newEdges.Append(edge.Oriented(Forward));
                    curVertex = (V1_.IsSame(curVertex)) ? V2_ : V1_;
                    edgesOfWire.Remove(itl);
                }
                else
                {
                    itl.Next();
                }
            }

            double vfirst, vlast;
            if (p2d1.Y > 0.0)
            {
                vfirst = p2d1.Y;
                vlast = M_PI / 2.0;
            }
            else
            {
                vfirst = -M_PI / 2.0;
                vlast = p2d1.Y;
            }
            Face newSphericalFace =
                new LibraryMakeFace(sphSurf!, ufirst, ulast, vfirst, vlast, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
            Edge oldEdge = new(), degEdge2 = new();
            for (explo.Init(newSphericalFace, ShapeType.Edge); explo.More(); explo.Next())
            {
                degEdge2 = ShapeCasts.Edge(explo.Current());
                if (Tool.Degenerated(degEdge2))
                {
                    break;
                }
            }
            Vertex degVertex = Exploration.FirstVertex(degEdge2);
            for (explo.Init(newSphericalFace, ShapeType.Edge); explo.More(); explo.Next())
            {
                oldEdge = ShapeCasts.Edge(explo.Current());
                Exploration.Vertices(oldEdge, out Vertex V1_, out Vertex V2_);
                if (!V1_.IsSame(degVertex) && !V2_.IsSame(degVertex))
                {
                    break;
                }
            }
            Exploration.Vertices(oldEdge, out Vertex V1, out Vertex V2);
            NCollection_List<Shape> lv1 = new(), lv2 = new();
            lv1.Append(vf1.Oriented(Forward));
            lv2.Append(curVertex.Oriented(Forward));
            Substitution theSubstitutor = new();
            theSubstitutor.Substitute(V1.Oriented(Forward), lv1);
            if (!V1.IsSame(V2))
            {
                theSubstitutor.Substitute(V2.Oriented(Forward), lv2);
            }
            theSubstitutor.Substitute(oldEdge.Oriented(Forward), newEdges);
            theSubstitutor.Build(newSphericalFace);
            if (theSubstitutor.IsCopied(newSphericalFace))
            {
                NCollection_List<Shape> listSh = theSubstitutor.InternalCopy(newSphericalFace);
                newSphericalFace = ShapeCasts.Face(listSh.First());
            }

            // Adding NewSphericalFace to the shell
            explo.Init(_offsetShape, ShapeType.Shell);
            Shape theShell = explo.Current();
            theShell.Free(true);
            bb.Add(theShell, newSphericalFace);
            isModified = true;
            if (!theShell.IsClosed)
            {
                if (Tool.IsClosed(theShell))
                {
                    theShell.Closed(true);
                }
            }
        }
        //
        if (!isModified)
        {
            return;
        }
        //
        if (_shape.Type == ShapeType.Solid || _thickening)
        {
            // Explo.Init( myOffsetShape, TopAbs_SHELL );

            // if (Explo.More()) {
            //   TopoDS_Shape theShell = Explo.Current();
            //   theShell.Closed( true );
            // }

            int nbShell = 0;
            Compound nc = new();
            Shape s1 = new();
            bb.MakeCompound(ref nc);

            Solid sol = new();
            bb.MakeSolid(ref sol);
            sol.Closed(true);
            for (explo.Init(_offsetShape, ShapeType.Shell); explo.More(); explo.Next())
            {
                Shell sh = ShapeCasts.Shell(explo.Current());
                // if (myThickening && myOffset > 0.)
                //   Sh.Reverse();
                nbShell++;
                if (sh.IsClosed)
                {
                    bb.Add(sol, sh);
                }
                else
                {
                    bb.Add(nc, sh);
                    if (nbShell == 1)
                    {
                        s1 = sh;
                    }
                }
            }
            int nbs = sol.ChildCount;
            bool solIsNull = (nbs == 0);
            // Checking solid
            if (nbs > 1)
            {
                Analyzer check = new(sol, false);
                if (!check.IsValid())
                {
                    NCollection_List<Shape> solList = new();
                    CorrectSolid(ref sol, solList);
                    if (!solList.IsEmpty())
                    {
                        bb.Add(nc, sol);
                        NCollection_List<Shape>.Iterator slIt = new(solList);
                        for (; slIt.More(); slIt.Next())
                        {
                            bb.Add(nc, slIt.Value());
                        }
                        solIsNull = true;
                    }
                }
            }
            //
            bool ncIsNull = (nc.ChildCount == 0);
            if ((!solIsNull) && (!ncIsNull))
            {
                bb.Add(nc, sol);
                _offsetShape = nc;
            }
            else if (solIsNull && (!ncIsNull))
            {
                if (nbShell == 1)
                {
                    _offsetShape = s1;
                }
                else
                {
                    _offsetShape = nc;
                }
            }
            else if ((!solIsNull) && ncIsNull)
            {
                _offsetShape = sol;
            }
            else
            {
                _offsetShape = nc;
            }
        }
    }

    //=================================================================================================

    private void Intersection3D(Inter3d inter, ProgressRange range)
    {
        // OCCT_DEBUG: the chronometer is left out.
        using ProgressScope ps = new(range, null, (_faces.Count != 0 && _join == Arc) ? 2 : 1);

        // In the Complete Intersection mode, implemented currently for planar
        // solids only, there is no need to intersect the faces here.
        // This intersection will be performed in the method BuildShellsCompleteInter
        // where the special treatment is applied to produced faces.
        //
        // Make sure to match the parameters in which the method
        // BuildShellsCompleteInter is called.
        if (_inter && (_join == global::Kerf.Geometry.JoinKind.Intersection) && _isPlanar && !_thickening && _faces.Count == 0
            && IsSolid(_shape))
        {
            return;
        }

        NCollection_List<Shape> offsetFaces = new(); // list of faces // created.
        MakeList(offsetFaces, _initOffsetFace, _faces);

        if (_faces.Count != 0)
        {
            bool inSide = (_offset < 0.0); // Temporary
            // it is necessary to calculate Inside taking account of the concavity or convexity of edges
            // between the cap and the part.

            if (_join == Arc)
            {
                inter.ContextIntByArc(_faces, inSide, _analyse, _initOffsetFace, _initOffsetEdge, ps.Next());
            }
        }
        if (_inter)
        {
            //-------------
            // Complete.
            //-------------
            inter.CompletInt(offsetFaces, _initOffsetFace, ps.Next());
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            IndexedSet<Shape> newEdges = inter.NewEdges();
            if (_join == global::Kerf.Geometry.JoinKind.Intersection)
            {
                OffsetTool.CorrectOrientation(_faceComp, newEdges, _asDes, _initOffsetFace, _offset);
            }
        }
        else
        {
            //--------------------------------
            // Only between neighbor faces.
            //--------------------------------
            inter.ConnexIntByArc(offsetFaces, _faceComp, _analyse, _initOffsetFace, ps.Next());
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
        }
        // OCCT_DEBUG: the chronometer is left out.
    }

    //=================================================================================================

    private void Intersection2D(IndexedSet<Shape> modif, IndexedSet<Shape> newEdges,
                                ProgressRange range)
    {
        // OCCT_DEBUG: the chronometer is left out.
        //--------------------------------------------------------
        // calculate intersections2d on faces concerned by
        // intersection3d
        //---------------------------------------------------------
        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>::Iterator it(Modif);
        //-----------------------------------------------
        // Intersection of edges 2 by 2.
        //-----------------------------------------------
        IndexedMap<Shape, NCollection_List<Shape>> dmvv = new(ShapeHasher.Instance);
        int i;
        using ProgressScope ps = new(range, "Intersection 2D", modif.Count);
        for (i = 1; i <= modif.Count; i++)
        {
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Face f = ShapeCasts.Face(modif[i - 1]);
            Inter2d.Compute(_asDes, f, newEdges, _tol, _edgeIntEdges, dmvv, ps.Next());
        }
        //
        Inter2d.FuseVertices(dmvv, _asDes, _imageVV);
        //
        // OCCT_DEBUG: DEBVerticesControl and the chronometer are left out.
    }

    //=================================================================================================

    private void MakeLoops(IndexedSet<Shape> modif, ProgressRange range)
    {
        // OCCT_DEBUG: the chronometer is left out.
        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>::Iterator    it(Modif);
        NCollection_List<Shape> lf = new(), lc = new();
        //-----------------------------------------
        // unwinding of faces // modified.
        //-----------------------------------------
        int i;
        for (i = 1; i <= modif.Count; i++)
        {
            if (!_faces.Contains(modif[i - 1]))
            {
                lf.Append(modif[i - 1]);
            }
        }
        //
        using ProgressScope ps = new(range, null, lf.Extent() + _faces.Count);
        if ((_join == global::Kerf.Geometry.JoinKind.Intersection) && _inter && _isPlanar)
        {
            BuildSplitsOfTrimmedFaces(lf, _asDes, _imageOffset, ps.Next(lf.Extent()));
        }
        else
        {
            _makeLoops.Build(lf, _asDes, _imageOffset, _imageVV, ps.Next(lf.Extent()));
        }
        if (!ps.More())
        {
            _error = global::Kerf.Engine.Offsets.Error.UserBreak;
            return;
        }

        //-----------------------------------------
        // unwinding of caps.
        //-----------------------------------------
        for (i = 1; i <= _faces.Count; i++)
        {
            lc.Append(_faces[i - 1]);
        }

        bool inSide = true;
        if (_offset > 0)
        {
            inSide = false;
        }
        _makeLoops.BuildOnContext(lc, _analyse, _asDes, _imageOffset, inSide, ps.Next(lc.Extent()));

        // OCCT_DEBUG: the chronometer is left out.
    }

    //=======================================================================
    // function : MakeFaces
    // purpose  : Reconstruction of topologically unchanged faces that
    //           share edges that were reconstructed.
    //=======================================================================
    private void MakeFaces(IndexedSet<Shape> modif, ProgressRange range)
    {
        // OCCT_DEBUG: the chronometer is left out.
        NCollection_List<Shape>.Iterator itr = new();
        NCollection_List<Shape> roots = _initOffsetFace.Roots();
        NCollection_List<Shape> lof = new();
        //----------------------------------
        // Loop on all faces //.
        //----------------------------------
        for (itr.Initialize(roots); itr.More(); itr.Next())
        {
            Face f = ShapeCasts.Face(_initOffsetFace.Image(itr.Value()).First());
            if (!_imageOffset.HasImage(f))
            {
                lof.Append(f);
            }
        }
        //
        using ProgressScope ps = new(range, null, 1);
        if ((_join == global::Kerf.Geometry.JoinKind.Intersection) && _inter && _isPlanar)
        {
            BuildSplitsOfTrimmedFaces(lof, _asDes, _imageOffset, ps.Next());
        }
        else
        {
            _makeLoops.BuildFaces(lof, _asDes, _imageOffset, ps.Next());
        }
        if (!ps.More())
        {
            _error = global::Kerf.Engine.Offsets.Error.UserBreak;
            return;
        }
        // OCCT_DEBUG: the chronometer is left out.
    }

    //=======================================================================
    // function : UpdateInitOffset
    // purpose  : Update and cleaning of myInitOffset
    //=======================================================================
    private static void UpdateInitOffset(ShapeImage myInitOffset, ShapeImage myImageOffset,
                                         in Shape myOffsetShape, ShapeType shapeType) // skv
    {
        ShapeImage niof = new();
        NCollection_List<Shape> roots = myInitOffset.Roots();
        NCollection_List<Shape>.Iterator it = new(roots);
        for (; it.More(); it.Next())
        {
            niof.SetRoot(it.Value());
        }
        for (it.Initialize(roots); it.More(); it.Next())
        {
            Shape si = it.Value();
            NCollection_List<Shape> li = new();
            NCollection_List<Shape> l1 = new();
            myInitOffset.LastImage(si, l1);
            NCollection_List<Shape>.Iterator itL1 = new(l1);
            for (; itL1.More(); itL1.Next())
            {
                Shape o1 = itL1.Value();
                NCollection_List<Shape> l2 = new();
                myImageOffset.LastImage(o1, l2);
                li.Append(l2);
            }
            niof.Bind(si, li);
        }
        //  Modified by skv - Mon Apr  4 18:17:27 2005 Begin
        //  Supporting history.
        //   NIOF.Filter(myOffsetShape,TopAbs_FACE);
        niof.Filter(myOffsetShape, shapeType);
        //  Modified by skv - Mon Apr  4 18:17:27 2005 End
        myInitOffset.Assign(niof);
    }

    //=================================================================================================

    private void MakeMissingWalls(ProgressRange range)
    {
        IndexedMap<Shape, NCollection_List<Shape>> contours =
            new(ShapeHasher.Instance); // Start vertex + list of connected edges (free boundary)
        Dictionary<Shape, Shape> mapEF = new(ShapeHasher.Instance); // Edges of contours: edge + face
        double offsetVal = Math.Abs(_offset);

        FillContours(_faceComp, _analyse, contours, mapEF);

        using ProgressScope ps = new(range, "Making missing walls", contours.Count);
        for (int ic = 1; ic <= contours.Count; ic++, ps.Next())
        {
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Vertex startVertex = ShapeCasts.Vertex(contours.KeyAt(ic - 1));
            Edge startEdge = new();
            NCollection_List<Shape> contour = contours.ValueAt(ic - 1);
            NCollection_List<Shape>.Iterator itl = new(contour);
            bool firstStep = true;
            Edge prevEdge = new();
            Vertex prevVertex = startVertex;
            bool isBuildFromScratch = false; // Problems with edges.
            for (; itl.More(); itl.Next())
            {
                Edge edge = ShapeCasts.Edge(itl.Value());
                Face faceOfEdge = ShapeCasts.Face(mapEF.GetValue(edge));

                // Check for offset existence.
                if (!_initOffsetEdge.HasImage(edge))
                {
                    continue;
                }

                // Check for existence of two different vertices.
                NCollection_List<Shape> loe = new(), loe2 = new();
                _initOffsetEdge.LastImage(edge, loe);
                _imageOffset.LastImage(loe.Last(), loe2);
                Edge oe = ShapeCasts.Edge(loe2.Last());
                Exploration.Vertices(oe, out Vertex v4, out Vertex v3);
                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                Curve3D? c = Tool.Curve(edge, out double aF, out double aL);
                if (v3.IsNull && v4.IsNull)
                {
                    // Initially offset edge is created without vertices.
                    // Then edge is trimmed by intersection line between
                    // two adjacent extended offset faces and get vertices.
                    // When intersection lines are invalid for any reason,
                    // (one reason is mixed connectivity of faces)
                    // algorithm of cutting offset edge by intersection line
                    // can fail and offset edge cannot get vertices.
                    // Following workaround is only to avoid exception if V3 and V4 are Null
                    // Vertex points are invalid.
                    Orientation oeOri = oe.Orientation;
                    oe.Orient(Forward);
                    Curve3D oec = Tool.Curve(oe, out double oef, out double oel)!;
                    BrepBuilder aBB = new();
                    Point3D p1 = oec.Value(aF);
                    Point3D p2 = oec.Value(aL);
                    Vertex oev1 = new(), oev2 = new();
                    double tol = Math.Max(Tool.Tolerance(v1), Tool.Tolerance(v2));
                    aBB.MakeVertex(ref oev1, p1, tol);
                    oev1.Orient(Forward);
                    aBB.MakeVertex(ref oev2, p2, tol);
                    oev2.Orient(global::Kerf.Topology.Orientation.Reversed);
                    aBB.Add(oe, oev1);
                    aBB.Add(oe, oev2);
                    aBB.Range(oe, aF, aL);
                    oe.Orient(oeOri);
                    Exploration.Vertices(oe, out v4, out v3);
                }
                if (c is not null && (!c.IsClosed && !c.IsPeriodic))
                {
                    Point3D pntF = Tool.Pnt(v1);
                    Point3D pntL = Tool.Pnt(v2);
                    double distE = pntF.SquareDistanceTo(pntL);
                    if (distE < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        // Bad case: non closed, but vertexes mapped to same 3d point.
                        continue;
                    }

                    double edgeTol0 = Tool.Tolerance(edge);
                    if (distE < edgeTol0)
                    {
                        // Potential problems not detected via checkshape.
                        Point3D pntOF = Tool.Pnt(v4);
                        Point3D pntOL = Tool.Pnt(v3);
                        if (pntOF.SquareDistanceTo(pntOL) > Constants.Resolution())
                        {
                            // To avoid computation of complex analytical continuation of Sin / ArcSin.
                            double sinValue = Math.Min(2 * edgeTol0 / pntOF.DistanceTo(pntOL), 1.0);
                            double maxAngle = Math.Min(Math.Abs(Math.Asin(sinValue)), M_PI_4); // Maximal angle.
                            double currentAngle = new Vector3D(pntF, pntL).AngleTo(new Vector3D(pntOF, pntOL));
                            if (c is LineCurve3D && Math.Abs(currentAngle) > maxAngle)
                            {
                                // anEdge not collinear to offset edge.
                                isBuildFromScratch = true;
                                _isPerformSewing = true;
                                continue;
                            }
                        }
                    }
                }

                bool toReverse = false;
                if (!v1.IsSame(prevVertex))
                {
                    Vertex vtx = v1;
                    v1 = v2;
                    v2 = vtx;
                    vtx = v3;
                    v3 = v4;
                    v4 = vtx;
                    toReverse = true;
                }

                oe.Orient(global::Kerf.Engine.Topology.TopAbs.Reverse(edge.Orientation));
                Edge E3, E4 = new();
                bool arcOnV2 = ((_join == Arc) && (_initOffsetEdge.HasImage(v2)));
                if (firstStep || isBuildFromScratch)
                {
                    E4 = new LibraryMakeEdge(v1, v4).InternalEdge();
                    if (firstStep)
                    {
                        startEdge = E4;
                    }
                }
                else
                {
                    E4 = prevEdge;
                }
                if (v2.IsSame(startVertex) && !arcOnV2)
                {
                    E3 = startEdge;
                }
                else
                {
                    E3 = new LibraryMakeEdge(v2, v3).InternalEdge();
                }
                E4.Reverse();

                if (isBuildFromScratch)
                {
                    E3.Reverse();
                    E4.Reverse();
                }

                Shape localAnEdge = edge.Oriented(Forward);
                Edge edgeFWD = ShapeCasts.Edge(localAnEdge);
                double parV1 = Tool.Parameter(v1, edgeFWD);
                double parV2 = Tool.Parameter(v2, edgeFWD);
                BrepBuilder bb = new();
                Wire theWire = new();
                bb.MakeWire(ref theWire);
                if (toReverse)
                {
                    bb.Add(theWire, edge.Reversed());
                    bb.Add(theWire, E3.Reversed());
                    bb.Add(theWire, oe.Reversed());
                    bb.Add(theWire, E4.Reversed());
                }
                else
                {
                    bb.Add(theWire, edge);
                    bb.Add(theWire, E3);
                    bb.Add(theWire, oe);
                    bb.Add(theWire, E4);
                }

                BrepLibrary.BuildCurves3d(theWire, _tol);
                theWire.Closed(true);
                Face newFace = new();
                Surface? theSurf = null;
                EdgeCurveAdaptor bAcurve = new(edge);
                EdgeCurveAdaptor bAcurveOE = new(oe);
                double fpar = bAcurve.FirstParameter();
                double lpar = bAcurve.LastParameter();
                Point3D ponE = bAcurve.Value(fpar);
                Point3D ponOE = bAcurveOE.Value(fpar);
                Direction3D offsetDir = new MakeDir(ponE, ponOE).Value();
                LineCurve2D? edgeLine2d = null, oeLine2d = null, line2d = null, line2d2 = null;
                bool isPlanar = false;
                if (bAcurve.GetType() == Circle && bAcurveOE.GetType() == Circle)
                {
                    Circle3D circ = bAcurve.Circle();
                    Circle3D circOE = bAcurveOE.Circle();
                    Line3D axisLine = new(circ.Axis);
                    Direction3D circAxisDir = circ.Axis.Direction;
                    if (circ.Axis.IsParallel(circOE.Axis, global::Kerf.Engine.Foundation.Precision.Confusion())
                        && axisLine.Contains(circOE.Location, global::Kerf.Engine.Foundation.Precision.Confusion()))
                    { // cylinder, plane or cone
                        if (Math.Abs(circ.Radius - circOE.Radius) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        { // case of cylinder
                            theSurf = new MakeCylindricalSurface(circ).Value();
                        }
                        else if (circ.Location.DistanceTo(circOE.Location) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        { // case of plane
                            isPlanar = true;
                            //
                            Point3D ponEL = bAcurve.Value(lpar);
                            if (ponEL.DistanceTo(ponE) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                bool bIsHole;
                                Edge e1, e2;
                                Wire w1 = new(), w2 = new();
                                PlaneSurface pl;
                                FaceClassifier2D clsf = new();
                                //
                                if (circ.Radius > circOE.Radius)
                                {
                                    e1 = edge;
                                    e2 = oe;
                                }
                                else
                                {
                                    e1 = oe;
                                    e2 = edge;
                                }
                                //
                                bb.MakeWire(ref w1);
                                bb.Add(w1, e1);
                                bb.MakeWire(ref w2);
                                bb.Add(w2, e2);
                                //
                                pl = new PlaneSurface(circ.Location, circAxisDir);
                                for (int i = 0; i < 2; ++i)
                                {
                                    ref Wire w = ref (i == 0) ? ref w1 : ref w2;
                                    Edge aE = (i == 0) ? e1 : e2;
                                    //
                                    Face face = new();
                                    bb.MakeFace(ref face, pl, global::Kerf.Engine.Foundation.Precision.Confusion());
                                    bb.Add(face, w);
                                    clsf.Init(face, global::Kerf.Engine.Foundation.Precision.Confusion());
                                    bIsHole = clsf.IsHole;
                                    if ((bIsHole && i == 0) || (!bIsHole && i != 0))
                                    {
                                        w.Nullify();
                                        bb.MakeWire(ref w);
                                        bb.Add(w, aE.Reversed());
                                    }
                                }
                                //
                                bb.MakeFace(ref newFace, pl, global::Kerf.Engine.Foundation.Precision.Confusion());
                                bb.Add(newFace, w1);
                                bb.Add(newFace, w2);
                            }
                        }
                        else // case of cone
                        {
                            Cone theCone =
                                new MakeCone(circ.Location, circOE.Location, circ.Radius, circOE.Radius).Value();
                            CoordinateSystem theAx3 = new(circ.Position);
                            if (circAxisDir * theCone.Axis.Direction < 0.0)
                            {
                                theAx3 = theAx3.ZReversed();
                                circAxisDir = circAxisDir.Reversed();
                            }
                            theCone = theCone.WithPosition(theAx3);
                            theSurf = new ConicalSurface(theCone);
                        }
                        if (!isPlanar)
                        {
                            Placement loc = new();
                            edgeLine2d = new LineCurve2D(new Point2D(0.0, 0.0), new Direction2D(Direction2D.D.X));
                            bb.UpdateEdge(edge, edgeLine2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                            double coeff = (offsetDir * circAxisDir > 0.0) ? 1.0 : -1.0;
                            oeLine2d = new LineCurve2D(new Point2D(0.0, offsetVal * coeff), new Direction2D(Direction2D.D.X));
                            bb.UpdateEdge(oe, oeLine2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                            line2d = new LineCurve2D(new Point2D(parV2, 0.0), new Direction2D(0.0, coeff));
                            line2d2 = new LineCurve2D(new Point2D(parV1, 0.0), new Direction2D(0.0, coeff));
                            if (E3.IsSame(E4))
                            {
                                if (coeff > 0.0)
                                {
                                    bb.UpdateEdge(E3, line2d, line2d2, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                                }
                                else
                                {
                                    bb.UpdateEdge(E3, line2d2, line2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                                    theWire.Nullify();
                                    bb.MakeWire(ref theWire);
                                    bb.Add(theWire, edge.Oriented(global::Kerf.Topology.Orientation.Reversed));
                                    bb.Add(theWire, E4);
                                    bb.Add(theWire, oe.Oriented(Forward));
                                    bb.Add(theWire, E3);
                                    theWire.Closed(true);
                                }
                            }
                            else
                            {
                                bb.SameParameter(E3, false);
                                bb.SameRange(E3, false);
                                bb.SameParameter(E4, false);
                                bb.SameRange(E4, false);
                                bb.UpdateEdge(E3, line2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                                bb.Range(E3, theSurf, loc, 0.0, offsetVal);
                                bb.UpdateEdge(E4, line2d2, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                                bb.Range(E4, theSurf, loc, 0.0, offsetVal);
                            }
                            newFace = new LibraryMakeFace(theSurf!, theWire).InternalFace();
                        }
                    } // cylinder or cone
                } // if both edges are arcs of circles
                if (newFace.IsNull)
                {
                    double edgeTol = Tool.Tolerance(edge);
                    // Tolerances of input shape should not be increased by BRepLib_MakeFace
                    FindSurface findPlane = new(theWire, edgeTol, true); // only plane
                    isPlanar = false;
                    if (findPlane.Found() && findPlane.ToleranceReached() <= edgeTol)
                    {
                        Curve3D? gc = Tool.Curve(edge, out double f, out double l);
                        PlaneSurface pln = (PlaneSurface)findPlane.Surface()!;
                        double maxDist = ComputeMaxDist(pln.Plane, gc, f, l);
                        if (maxDist <= edgeTol)
                        {
                            LibraryMakeFace mf = new(pln.Plane, theWire);
                            if (mf.IsDone)
                            {
                                newFace = mf.InternalFace();
                                Iterator itE = new(theWire);
                                for (; itE.More(); itE.Next())
                                {
                                    Edge e = ShapeCasts.Edge(itE.Value());
                                    if (e.IsSame(edge))
                                    {
                                        continue;
                                    }
                                    gc = Tool.Curve(e, out f, out l);
                                    maxDist = ComputeMaxDist(pln.Plane, gc, f, l);
                                    bb.UpdateEdge(e, maxDist);
                                }
                                isPlanar = true;
                            }
                        }
                    }
                    //
                    if (!isPlanar) // Extrusion (by thrusections)
                    {
                        Curve3D edgeCurve = Tool.Curve(edge, out fpar, out lpar)!;
                        TrimmedCurve3D trEdgeCurve = new(edgeCurve, fpar, lpar);
                        Curve3D offsetCurve = Tool.Curve(oe, out double fparOE0, out double lparOE0)!;
                        TrimmedCurve3D trOffsetCurve = new(offsetCurve, fparOE0, lparOE0);
                        SurfaceGenerator thrusecGenerator = new();
                        thrusecGenerator.AddCurve(trEdgeCurve);
                        thrusecGenerator.AddCurve(trOffsetCurve);
                        thrusecGenerator.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
                        theSurf = thrusecGenerator.Surface()!;
                        theSurf.ParameterBounds(out double uf, out double ul, out double vf, out double vl);
                        Placement loc = new();
                        edgeLine2d = new LineCurve2D(new Point2D(0.0, vf), new Direction2D(Direction2D.D.X));
                        bb.UpdateEdge(edge, edgeLine2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                        oeLine2d = new LineCurve2D(new Point2D(0.0, vl), new Direction2D(Direction2D.D.X));
                        bb.UpdateEdge(oe, oeLine2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                        double uonV1 = (toReverse) ? ul : uf;
                        double uonV2 = (toReverse) ? uf : ul;
                        line2d = new LineCurve2D(new Point2D(uonV2, 0.0), new Direction2D(Direction2D.D.Y));
                        line2d2 = new LineCurve2D(new Point2D(uonV1, 0.0), new Direction2D(Direction2D.D.Y));
                        if (E3.IsSame(E4))
                        {
                            bb.UpdateEdge(E3, line2d, line2d2, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                            Curve3D bSplC34 = theSurf.UIso(uf);
                            bb.UpdateEdge(E3, bSplC34, global::Kerf.Engine.Foundation.Precision.Confusion());
                            bb.Range(E3, vf, vl);
                        }
                        else
                        {
                            bb.SameParameter(E3, false);
                            bb.SameRange(E3, false);
                            bb.SameParameter(E4, false);
                            bb.SameRange(E4, false);
                            bb.UpdateEdge(E3, line2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                            bb.Range(E3, theSurf, loc, vf, vl);
                            bb.UpdateEdge(E4, line2d2, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                            bb.Range(E4, theSurf, loc, vf, vl);
                            Curve3D bSplC3 = theSurf.UIso(uonV2);
                            bb.UpdateEdge(E3, bSplC3, global::Kerf.Engine.Foundation.Precision.Confusion());
                            bb.Range(E3, vf, vl, true); // only for 3d curve
                            Curve3D bSplC4 = theSurf.UIso(uonV1);
                            bb.UpdateEdge(E4, bSplC4, global::Kerf.Engine.Foundation.Precision.Confusion());
                            bb.Range(E4, vf, vl, true); // only for 3d curve
                        }
                        newFace = new LibraryMakeFace(theSurf, theWire).InternalFace();
                    }
                }
                if (!isPlanar)
                {
                    double fparOE = bAcurveOE.FirstParameter();
                    double lparOE = bAcurveOE.LastParameter();
                    Placement loc = new();
                    if (Math.Abs(fpar - fparOE) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        Edge e4 = (toReverse) ? E3 : E4;
                        Point2D fp2d = edgeLine2d!.Value(fpar);
                        Point2D fp2dOE = oeLine2d!.Value(fparOE);
                        line2d2 = new MakeLine2d(fp2d, fp2dOE).Value();
                        Curve3D? curve = null;
                        double firstPar = 0.0, lastPar = fp2d.DistanceTo(fp2dOE);
                        CurveAdaptor2D ac2d = new(line2d2, firstPar, lastPar);
                        SurfaceAdaptor gAsurf = new(theSurf!);
                        CurveAdaptor2D hc2d = ac2d; // OCCT copies the adaptors into handles
                        SurfaceAdaptor hSurf = gAsurf;
                        CurveOnSurface conS = new(hc2d, hSurf);
                        global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(global::Kerf.Engine.Foundation.Precision.Confusion(), conS, firstPar, lastPar, ref curve,
                                             out double max_deviation, out double average_deviation);
                        bb.UpdateEdge(e4, curve, max_deviation);
                        bb.UpdateEdge(e4, line2d2, theSurf, loc, max_deviation);
                        bb.Range(e4, firstPar, lastPar);
                    }
                    if (Math.Abs(lpar - lparOE) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        Edge e3 = (toReverse) ? E4 : E3;
                        Point2D lp2d = edgeLine2d!.Value(lpar);
                        Point2D lp2dOE = oeLine2d!.Value(lparOE);
                        line2d = new MakeLine2d(lp2d, lp2dOE).Value();
                        Curve3D? curve = null;
                        double firstPar = 0.0, lastPar = lp2d.DistanceTo(lp2dOE);
                        CurveAdaptor2D ac2d = new(line2d, firstPar, lastPar);
                        SurfaceAdaptor gAsurf = new(theSurf!);
                        CurveAdaptor2D hc2d = ac2d; // OCCT copies the adaptors into handles
                        SurfaceAdaptor hSurf = gAsurf;
                        CurveOnSurface conS = new(hc2d, hSurf);
                        global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(global::Kerf.Engine.Foundation.Precision.Confusion(), conS, firstPar, lastPar, ref curve,
                                             out double max_deviation, out double average_deviation);
                        bb.UpdateEdge(e3, curve, max_deviation);
                        bb.UpdateEdge(e3, line2d, theSurf, loc, max_deviation);
                        bb.Range(e3, firstPar, lastPar);
                    }
                }

                if (!isPlanar)
                {
                    // For planar faces these operations are useless,
                    // because there are no curves on surface
                    BrepLibrary.SameParameter(newFace);
                    BrepTools.Update(newFace);
                }
                // Check orientation
                Orientation or = OrientationOfEdgeInFace(edge, faceOfEdge);
                Orientation orInNewFace = OrientationOfEdgeInFace(edge, newFace);
                if (orInNewFace != global::Kerf.Engine.Topology.TopAbs.Reverse(or))
                {
                    newFace.Reverse();
                }
                ///////////////////
                _walls.Append(newFace);
                if (arcOnV2)
                {
                    Edge arc = ShapeCasts.Edge(_initOffsetEdge.Image(v2).First());
                    Exploration.Vertices(arc, out Vertex arcV1, out Vertex arcV2);
                    bool arcReverse = false;
                    if (!arcV1.IsSame(v3))
                    {
                        Vertex vtx = arcV1;
                        arcV1 = arcV2;
                        arcV2 = vtx;
                        arcReverse = true;
                    }
                    Edge ea1, ea2;
                    // EA1 = (ToReverse)? E3 : TopoDS::Edge(E3.Reversed());
                    ea1 = E3;
                    ea1.Reverse();
                    if (toReverse)
                    {
                        ea1.Reverse();
                    }
                    //////////////////////////////////////////////////////
                    if (v2.IsSame(startVertex))
                    {
                        ea2 = startEdge;
                    }
                    else
                    {
                        ea2 = new LibraryMakeEdge(v2, arcV2).InternalEdge();
                    }
                    arc.Orient(((arcReverse) ? global::Kerf.Topology.Orientation.Reversed : Forward));
                    if (ea1.Orientation == global::Kerf.Topology.Orientation.Reversed)
                    {
                        arc.Reverse();
                    }
                    ea2.Orient(global::Kerf.Engine.Topology.TopAbs.Reverse(ea1.Orientation));
                    Wire arcWire = new();
                    bb.MakeWire(ref arcWire);
                    bb.Add(arcWire, ea1);
                    bb.Add(arcWire, arc);
                    bb.Add(arcWire, ea2);
                    BrepLibrary.BuildCurves3d(arcWire, _tol);
                    arcWire.Closed(true);
                    Face arcFace = new LibraryMakeFace(arcWire, true).InternalFace();
                    BrepTools.Update(arcFace);
                    _walls.Append(arcFace);
                    Shape localEA2 = ea2.Oriented(Forward);
                    Edge cea2 = ShapeCasts.Edge(localEA2);
                    prevEdge = cea2;
                    prevVertex = v2;
                }
                else
                {
                    if (isBuildFromScratch)
                    {
                        prevEdge = ShapeCasts.Edge(E4);
                        prevVertex = v1;
                        isBuildFromScratch = false;
                    }
                    else
                    {
                        prevEdge = E3;
                        prevVertex = v2;
                    }
                }
                firstStep = false;
            }
        }
    }

    //=================================================================================================

    private void MakeShells(ProgressRange range)
    {
        // OCCT_DEBUG: the chronometer is left out.
        //
        using ProgressScope ps = new(range, "Making shells", 1);
        // Prepare list of splits of the offset faces to make the shells
        NCollection_List<Shape> lsf = new();
        NCollection_List<Shape> r = _imageOffset.Roots();
        NCollection_List<Shape>.Iterator it = new(r);
        //
        for (; it.More(); it.Next())
        {
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Shape f = it.Value();
            if (_thickening)
            { // offsetted faces must change their orientations
                f.Reverse();
            }
            //
            NCollection_List<Shape> image = new();
            _imageOffset.LastImage(f, image);
            NCollection_List<Shape>.Iterator it2 = new(image);
            for (; it2.More(); it2.Next())
            {
                Shape fIm = it2.Value();
                lsf.Append(fIm);
            }
        }
        //
        if (_thickening)
        {
            Explorer explo0 = new(_shape, ShapeType.Face);
            for (; explo0.More(); explo0.Next())
            {
                Shape f = explo0.Current();
                lsf.Append(f);
            }
            //
            it.Initialize(_walls);
            for (; it.More(); it.Next())
            {
                Shape f = it.Value();
                lsf.Append(f);
            }
        }
        //
        if (lsf.IsEmpty())
        {
            return;
        }
        //
        bool bDone = false;
        if ((_join == global::Kerf.Geometry.JoinKind.Intersection) && _inter && !_thickening && _faces.Count == 0 && IsSolid(_shape)
            && _isPlanar)
        {
            //
            Shape shells = new();
            bDone = BuildShellsCompleteInter(lsf, _imageOffset, ref shells, ps.Next());
            if (bDone)
            {
                _offsetShape = shells;
            }
        }
        //
        if (!bDone)
        {
            Quilt glue = new();
            NCollection_List<Shape>.Iterator itLS = new(lsf);
            for (; itLS.More(); itLS.Next())
            {
                glue.Add(itLS.Value());
            }
            _offsetShape = glue.InternalShells();
        }
        //
        // Set correct value for closed flag
        Explorer explo = new(_offsetShape, ShapeType.Shell);
        for (; explo.More(); explo.Next())
        {
            Shape s = explo.Current();
            if (!s.IsClosed)
            {
                if (Tool.IsClosed(s))
                {
                    s.Closed(true);
                }
            }
        }
    }

    //=================================================================================================

    private void MakeSolid(ProgressRange range)
    {
        if (_offsetShape.IsNull)
        {
            return;
        }

        //  Modified by skv - Mon Apr  4 18:17:27 2005 Begin
        //  Supporting history.
        UpdateInitOffset(_initOffsetFace, _imageOffset, _offsetShape, ShapeType.Face);
        UpdateInitOffset(_initOffsetEdge, _imageOffset, _offsetShape, ShapeType.Edge);
        //  Modified by skv - Mon Apr  4 18:17:27 2005 End
        Explorer exp = new();
        BrepBuilder b = new();
        int nbShell = 0;
        Compound nc = new();
        Shape s1 = new();
        b.MakeCompound(ref nc);

        using ProgressScope ps = new(range, "Making solid", 1);

        Solid sol = new();
        b.MakeSolid(ref sol);
        sol.Closed(true);
        bool makeSolid = (_shape.Type == ShapeType.Solid) || _thickening;
        for (exp.Init(_offsetShape, ShapeType.Shell); exp.More(); exp.Next())
        {
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
            Shell sh = ShapeCasts.Shell(exp.Current());
            if (_thickening && _offset > 0.0)
            {
                sh.Reverse();
            }
            nbShell++;
            if (sh.IsClosed && makeSolid)
            {
                b.Add(sol, sh);
            }
            else
            {
                b.Add(nc, sh);
                if (nbShell == 1)
                {
                    s1 = sh;
                }
            }
        }
        int nbs = sol.ChildCount;
        bool solIsNull = (nbs == 0);
        // Checking solid
        if (nbs > 1)
        {
            Analyzer check = new(sol, false);
            if (!check.IsValid())
            {
                NCollection_List<Shape> solList = new();
                CorrectSolid(ref sol, solList);
                if (!solList.IsEmpty())
                {
                    b.Add(nc, sol);
                    NCollection_List<Shape>.Iterator slIt = new(solList);
                    for (; slIt.More(); slIt.Next())
                    {
                        b.Add(nc, slIt.Value());
                    }
                    solIsNull = true;
                }
            }
        }
        bool ncIsNull = (nc.ChildCount == 0);
        if ((!solIsNull) && (!ncIsNull))
        {
            b.Add(nc, sol);
            _offsetShape = nc;
        }
        else if (solIsNull && (!ncIsNull))
        {
            if (nbShell == 1)
            {
                _offsetShape = s1;
            }
            else
            {
                _offsetShape = nc;
            }
        }
        else if ((!solIsNull) && ncIsNull)
        {
            _offsetShape = sol;
        }
        else
        {
            _offsetShape = nc;
        }
    }

    //=================================================================================================

    private void SelectShells()
    {
        HashSet<Shape> freeEdges = new(ShapeHasher.Instance);
        Explorer exp = new(_faceComp, ShapeType.Edge);
        //-------------------------------------------------------------
        // FreeEdges all edges that can have free border in the
        // parallel shell
        // 1 - free borders of myShape .
        //-------------------------------------------------------------
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            NCollection_List<Shape> la = _analyse.Ancestors(e);
            if (la.Extent() < 2)
            {
                if (_analyse.Type(e).First().Type() == TypeOfConcavity.FreeBound)
                {
                    freeEdges.Add(e);
                }
            }
        }
        // myShape has free borders and there are no caps
        // no unwinding 3d.
        if (freeEdges.Count != 0 && _faces.Count == 0)
        {
            return;
        }

        _offsetShape = OffsetTool.Deboucle3D(_offsetShape, freeEdges);
    }

    //=================================================================================================

    /// <summary>
    /// The links between the initial shapes and the offset faces: a copy whose roots are the caller's sub-shapes and
    /// whose images are the result's faces (frozen).
    /// </summary>
    public ShapeImage OffsetFacesFromShapes() => PublishImage(_initOffsetFace);

    /// <summary>The links between the initial shapes and the offset faces, unfrozen, for ported code.</summary>
    internal ShapeImage InternalOffsetFacesFromShapes() => _initOffsetFace;

    //  Modified by skv - Tue Mar 15 16:20:43 2005 Begin

    //=================================================================================================

    /// <summary>The join type.</summary>
    public global::Kerf.Geometry.JoinKind GetJoinType() => _join;

    //=================================================================================================

    /// <summary>
    /// The links between the initial shapes and the offset edges: a copy whose roots are the caller's sub-shapes and
    /// whose images are the result's edges (frozen).
    /// </summary>
    public ShapeImage OffsetEdgesFromShapes() => PublishImage(_initOffsetEdge);

    /// <summary>The links between the initial shapes and the offset edges, unfrozen, for ported code.</summary>
    internal ShapeImage InternalOffsetEdgesFromShapes() => _initOffsetEdge;

    //  Modified by skv - Tue Mar 15 16:20:43 2005 End

    //=================================================================================================

    /// <summary>The closing faces stored by <see cref="AddFace"/>, as the caller gave them.</summary>
    public IndexedSet<Shape> ClosingFaces()
    {
        IndexedSet<Shape> faces = new(ShapeHasher.Instance);
        for (int i = 0; i < _originalFaces.Count; ++i)
        {
            faces.Add(InternalOriginal(_originalFaces[i]));
        }
        return faces;
    }

    /// <summary>The closing faces, for ported code.</summary>
    internal IndexedSet<Shape> InternalClosingFaces() => _originalFaces;

    /// <summary>
    /// A copy of an image whose roots are the caller's sub-shapes and whose images stand for the result's shapes, all
    /// frozen (decision 1030).
    /// </summary>
    private ShapeImage PublishImage(ShapeImage image)
    {
        ShapeImage published = new();
        HashSet<Shape> done = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator itR = new(image.Roots()); itR.More(); itR.Next())
        {
            Shape root = itR.Value();
            Shape pubRoot = InternalOriginal(root);
            pubRoot.TShape()?.FreezeGraph();
            published.SetRoot(pubRoot);
            PublishImages(image, root, pubRoot, published, done);
        }
        return published;
    }

    private void PublishImages(ShapeImage image, in Shape s, in Shape pubS,
                               ShapeImage published, HashSet<Shape> done)
    {
        if (!image.HasImage(s) || !done.Add(s))
        {
            return;
        }
        NCollection_List<Shape> lPub = new();
        NCollection_List<Shape> lIm = image.Image(s);
        for (NCollection_List<Shape>.Iterator it = new(lIm); it.More(); it.Next())
        {
            Shape pub = InternalFinal(it.Value());
            pub.TShape()?.FreezeGraph();
            lPub.Append(pub);
        }
        published.Bind(pubS, lPub);
        NCollection_List<Shape>.Iterator itP = new(lPub);
        for (NCollection_List<Shape>.Iterator it = new(lIm); it.More(); it.Next(), itP.Next())
        {
            if (!it.Value().IsSame(s))
            {
                PublishImages(image, it.Value(), itP.Value(), published, done);
            }
        }
    }

    //=================================================================================================

    private void EncodeRegularity()
    {
        // OCCT_DEBUG: the chronometer is left out.

        if (_offsetShape.IsNull)
        {
            return;
        }
        // find edges G1 in the result
        Explorer exp = new(_offsetShape, ShapeType.Edge);

        BrepBuilder b = new();
        HashSet<Shape> ms = new(ShapeHasher.Instance);

        for (; exp.More(); exp.Next())
        {
            Edge oe = ShapeCasts.Edge(exp.Current());
            BrepLibrary.BuildCurve3d(oe, _tol);
            Edge roe = oe;

            if (!ms.Add(oe))
            {
                continue;
            }

            if (_imageOffset.IsImage(oe))
            {
                roe = ShapeCasts.Edge(_imageOffset.Root(oe));
            }

            NCollection_List<Shape> lofOF = _asDes.Ascendant(roe);

            if (lofOF.Extent() != 2)
            {
                // OCCT_DEBUG_VERB: " Edge shared by N Faces" left out.
                continue;
            }

            Face f1 = ShapeCasts.Face(lofOF.First());
            Face f2 = ShapeCasts.Face(lofOF.Last());

            if (f1.IsNull || f2.IsNull)
            {
                continue;
            }

            Shape root1 = _initOffsetFace.Root(f1);
            Shape root2 = _initOffsetFace.Root(f2);

            ShapeType type1 = root1.Type;
            ShapeType type2 = root2.Type;

            if (f1.IsSame(f2))
            {
                if (Tool.IsClosed(oe, f1))
                {
                    // Temporary Debug for the Bench.
                    // Check with YFR.
                    // In mode intersection, the edges are not coded in myInitOffsetEdge
                    // so, manage case by case
                    // Note DUB; for Hidden parts, it is NECESSARY to code CN
                    // Analytic Surfaces.
                    if (_join == global::Kerf.Geometry.JoinKind.Intersection)
                    {
                        FaceSurfaceAdaptor bs = new(f1, false);
                        SurfaceKind sType = bs.GetType();
                        if (sType == SurfaceKind.Cylinder || sType == SurfaceKind.Cone || sType == SurfaceKind.Sphere
                            || sType == SurfaceKind.Torus)
                        {
                            b.Continuity(oe, f1, f1, CN);
                        }
                        else
                        {
                            // See YFR : MaJ of myInitOffsetFace
                        }
                    }
                    else if (_initOffsetEdge.IsImage(roe))
                    {
                        if (type1 == ShapeType.Face && type2 == ShapeType.Face)
                        {
                            Face fRoot = ShapeCasts.Face(root1);
                            Edge ei = ShapeCasts.Edge(_initOffsetEdge.ImageFrom(roe));
                            Continuity conti = Tool.Continuity(ei, fRoot, fRoot);
                            if (conti == CN)
                            {
                                b.Continuity(oe, f1, f1, CN);
                            }
                            else if (conti > C0)
                            {
                                b.Continuity(oe, f1, f1, G1);
                            }
                        }
                    }
                }
                continue;
            }

            //  code regularities G1 between :
            //    - sphere and tube : one root is a vertex, the other is an edge
            //                        and the vertex is included in the edge
            //    - face and tube   : one root is a face, the other an edge
            //                        and the edge is included in the face
            //    - face and face    : if two root faces are tangent in
            //                        the initial shape, they will be tangent in the offset shape
            //    - tube and tube  : if 2 edges generating tubes are
            //                        tangents, the 2 will be tangent either.
            if (type1 == ShapeType.Edge && type2 == ShapeType.Vertex)
            {
                Exploration.Vertices(ShapeCasts.Edge(root1), out Vertex v1, out Vertex v2);
                if (v1.IsSame(root2) || v2.IsSame(root2))
                {
                    b.Continuity(oe, f1, f2, G1);
                }
            }
            else if (type1 == ShapeType.Vertex && type2 == ShapeType.Edge)
            {
                Exploration.Vertices(ShapeCasts.Edge(root2), out Vertex v1, out Vertex v2);
                if (v1.IsSame(root1) || v2.IsSame(root1))
                {
                    b.Continuity(oe, f1, f2, G1);
                }
            }
            else if (type1 == ShapeType.Face && type2 == ShapeType.Edge)
            {
                Explorer exp2 = new(root1, ShapeType.Edge);
                for (; exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(root2))
                    {
                        b.Continuity(oe, f1, f2, G1);
                        break;
                    }
                }
            }
            else if (type1 == ShapeType.Edge && type2 == ShapeType.Face)
            {
                Explorer exp2 = new(root2, ShapeType.Edge);
                for (; exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(root1))
                    {
                        b.Continuity(oe, f1, f2, G1);
                        break;
                    }
                }
            }
            else if (type1 == ShapeType.Face && type2 == ShapeType.Face)
            {
                //  if two root faces are tangent in
                //  the initial shape, they will be tangent in the offset shape
                NCollection_List<Shape> le = new();
                OffsetTool.FindCommonShapes(root1, root2, ShapeType.Edge, le);
                if (le.Extent() == 1)
                {
                    Edge ed = ShapeCasts.Edge(le.First());
                    if (_analyse.HasAncestor(ed))
                    {
                        NCollection_List<OffsetInterval> li = _analyse.Type(ed);
                        if (li.Extent() == 1 && li.First().Type() == TypeOfConcavity.Tangential)
                        {
                            b.Continuity(oe, f1, f2, G1);
                        }
                    }
                }
            }
            else if (type1 == ShapeType.Edge && type2 == ShapeType.Edge)
            {
                NCollection_List<Shape> lv = new();
                OffsetTool.FindCommonShapes(root1, root2, ShapeType.Vertex, lv);
                if (lv.Extent() == 1)
                {
                    NCollection_List<Shape> lEdTg = new();
                    _analyse.TangentEdges(ShapeCasts.Edge(root1), ShapeCasts.Vertex(lv.First()), lEdTg);
                    NCollection_List<Shape>.Iterator it = new(lEdTg);
                    for (; it.More(); it.Next())
                    {
                        if (it.Value().IsSame(root2))
                        {
                            b.Continuity(oe, f1, f2, G1);
                            break;
                        }
                    }
                }
            }
        }

        // OCCT_DEBUG: the chronometer is left out.
    }

    //=================================================================================================

    private static double ComputeMaxDist(in Plane3D plane, Curve3D? crv, double first, double last)
    {
        double maxDist = 0.0;
        int i, ncontrol = 23;
        double prm, dist2;
        Point3D p;
        for (i = 0; i < ncontrol; i++)
        {
            prm = ((ncontrol - 1 - i) * first + i * last) / (ncontrol - 1);
            p = crv!.Value(prm);
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(p.X) || global::Kerf.Engine.Foundation.Precision.IsInfinite(p.Y) || global::Kerf.Engine.Foundation.Precision.IsInfinite(p.Z))
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            dist2 = plane.SquareDistanceTo(p);
            if (dist2 > maxDist)
            {
                maxDist = dist2;
            }
        }
        return Math.Sqrt(maxDist) * 1.05;
    }

    //=================================================================================================

    // OCCT.NET: Locked(false) acts on the offset's own shapes: the result's new edges and vertices, and, in the
    // thickening mode, the caller's shape in the run's working overlay (unit 10H5).
    private static void UpdateTolerance(ref Shape s, IndexedSet<Shape> faces,
                                        in Shape initShape)
    {
        BrepBuilder b = new();
        HashSet<Shape> view = new(ShapeHasher.Instance);
        Vertex[] v = new Vertex[2];

        // The edges of caps are not modified.
        int j;
        for (j = 1; j <= faces.Count; j++)
        {
            Shape f = faces[j - 1];
            Explorer exp = new();
            for (exp.Init(f, ShapeType.Edge); exp.More(); exp.Next())
            {
                view.Add(exp.Current());
            }
        }

        // The edges of initial shape are  not modified
        HashSet<Shape> mapInitF = new(ShapeHasher.Instance);
        if (!initShape.IsNull)
        {
            Explorer expF0 = new(initShape, ShapeType.Face);
            for (; expF0.More(); expF0.Next())
            {
                mapInitF.Add(expF0.Current());
                Explorer expE = new();
                for (expE.Init(expF0.Current(), ShapeType.Edge); expE.More(); expE.Next())
                {
                    view.Add(expE.Current());
                    Iterator itV = new(expE.Current());
                    for (; itV.More(); itV.Next())
                    {
                        view.Add(itV.Value());
                    }
                }
            }
        }

        double tol;
        Explorer expF = new(s, ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Shape f = expF.Current();
            if (faces.Contains(f) || mapInitF.Contains(f))
            {
                continue;
            }
            FaceSurfaceAdaptor bas = new(ShapeCasts.Face(f), false);
            Explorer exp = new();
            for (exp.Init(f, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                bool isUpdated = false;
                double currTol = Tool.Tolerance(e);
                if (bas.GetType() == Plane)
                {
                    // Edge does not seem to have pcurve on plane,
                    // so EdgeCorrector does not include it in tolerance calculation
                    Curve3D? crv = Tool.Curve(e, out double first, out double last);
                    double maxDist = ComputeMaxDist(bas.Plane(), crv, first, last);
                    if (maxDist > currTol)
                    {
                        b.UpdateEdge(e, maxDist);
                        isUpdated = true;
                    }
                }
                if (view.Add(e))
                {
                    e.Locked(false);
                    EdgeCheck edgeCorrector = new(e);
                    tol = edgeCorrector.Tolerance();
                    if (tol > currTol)
                    {
                        b.UpdateEdge(e, tol);
                        isUpdated = true;
                    }
                }
                if (isUpdated)
                {
                    tol = Tool.Tolerance(e);
                    // Update the vertices.
                    Exploration.Vertices(e, out v[0], out v[1]);

                    for (int i = 0; i <= 1; i++)
                    {
                        v[i].Locked(false);
                        if (view.Add(v[i]))
                        {
                            BrepTVertex tv = (BrepTVertex)v[i].TShape()!;
                            tv.Tolerance(0.0);
                            VertexCheck vertexCorrector = new(v[i]);
                            b.UpdateVertex(v[i], vertexCorrector.Tolerance());
                            // use the occasion to clean the vertices.
                            (tv.ChangePoints()).Clear();
                        }
                        b.UpdateVertex(v[i], tol);
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static void CorrectSolid(ref Solid sol, NCollection_List<Shape> solList)
    {
        BrepBuilder bb = new();
        Shape outerShell = new();
        NCollection_List<double> vols = new();
        double volMax = 0.0, outerVol = 0.0;

        Iterator it = new(sol);
        for (; it.More(); it.Next())
        {
            Shape sh = it.Value();
            Properties vProps = new();
            MassPropertiesBuilder.VolumeProperties(sh, vProps, true);
            if (Math.Abs(vProps.Mass()) > volMax)
            {
                outerVol = vProps.Mass();
                volMax = Math.Abs(outerVol);
                outerShell = sh;
            }
            vols.Append(vProps.Mass());
        }
        //
        if (Math.Abs(outerVol) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return;
        }
        if (outerVol < 0.0)
        {
            outerShell.Reverse();
        }
        Solid newSol = new();
        bb.MakeSolid(ref newSol);
        newSol.Closed(true);
        bb.Add(newSol, outerShell);
        SolidClassifier solClass = new(newSol);
        //
        it.Initialize(sol);
        NCollection_List<double>.Iterator vIt = new(vols);
        for (; it.More(); it.Next(), vIt.Next())
        {
            Shell sh = ShapeCasts.Shell(it.Value());
            if (sh.IsSame(outerShell))
            {
                continue;
            }
            else
            {
                Explorer vExp = new(sh, ShapeType.Vertex);
                Vertex v = ShapeCasts.Vertex(vExp.Current());
                Point3D p = Tool.Pnt(v);
                solClass.Perform(p, Tool.Tolerance(v));
                if (solClass.State() == In)
                {
                    if (vIt.Value() > 0.0)
                    {
                        sh.Reverse();
                    }
                    bb.Add(newSol, sh);
                }
                else
                {
                    if (vIt.Value() < 0.0)
                    {
                        sh.Reverse();
                    }
                    Solid aSol = new();
                    bb.MakeSolid(ref aSol);
                    aSol.Closed(true);
                    bb.Add(aSol, sh);
                    solList.Append(aSol);
                }
            }
        }
        sol = newSol;
    }

    //=======================================================================
    // function : CheckInputData
    // purpose  : Check input data for possibility of offset perform.
    //=======================================================================

    /// <summary>
    /// The pre-analysis: whether the offset can be made (see <see cref="Error"/>). Checks, in order, that some face
    /// has a non-null offset, that the shell is connected, that the surfaces aren't C0, and that their normals exist on
    /// a grid of points.
    /// </summary>
    public bool CheckInputData(ProgressRange range)
    {
        // Set initial error state.
        _error = global::Kerf.Engine.Offsets.Error.NoError;
        Shape tmpShape = new();
        _badShape = tmpShape;
        using ProgressScope ps = new(range, null, 1);
        // Non-null offset.
        if (Math.Abs(_offset) <= _tol)
        {
            bool isFound = false;
            foreach (KeyValuePair<Shape, double> iter in _faceOffset)
            {
                if (Math.Abs(iter.Value) > _tol)
                {
                    isFound = true;
                    break;
                }
            }

            if (!isFound)
            {
                // No face with non-null offset found.
                _error = global::Kerf.Engine.Offsets.Error.NullOffset;
                return false;
            }
        }

        // Connectivity of input shape.
        if (!IsConnectedShell(_faceComp))
        {
            _error = global::Kerf.Engine.Offsets.Error.NotConnectedShell;
            return false;
        }

        // Normals check and continuity check.
        const int pntPerDim = 20; // 21 points on each dimension.
        Explorer expSF = new(_faceComp, ShapeType.Face);
        HashSet<TShape> presenceMap = new(ReferenceEqualityComparer.Instance);
        Point2D pnt2d;
        for (; expSF.More(); expSF.Next())
        {
            if (!ps.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return false;
            }
            Face f = ShapeCasts.Face(expSF.Current());

            if (presenceMap.Contains(f.TShape()!))
            {
                // Not perform computations with partner shapes,
                // since they are contain same geometry.
                continue;
            }
            presenceMap.Add(f.TShape()!);

            Surface surf = Tool.Surface(f, out Placement l)!;
            BrepTools.UVBounds(f, out double umin, out double umax, out double vmin, out double vmax);

            // Continuity check.
            if (surf.Continuity == C0)
            {
                _error = global::Kerf.Engine.Offsets.Error.C0Geometry;
                return false;
            }

            // Get degenerated points, to avoid check them.
            List<Point3D> bad3dPnts = new();
            Explorer expFE = new(f, ShapeType.Edge);
            for (; expFE.More(); expFE.Next())
            {
                Edge e = ShapeCasts.Edge(expFE.Current());
                if (Tool.Degenerated(e))
                {
                    bad3dPnts.Add(Tool.Pnt((Exploration.FirstVertex(e))));
                }
            }

            // Geometry grid check.
            for (int i = 0; i <= pntPerDim; i++)
            {
                double uParam = umin + (umax - umin) * i / pntPerDim;
                for (int j = 0; j <= pntPerDim; j++)
                {
                    double vParam = vmin + (vmax - vmin) * j / pntPerDim;

                    _error = CheckSinglePoint(uParam, vParam, surf, bad3dPnts);
                    if (_error != global::Kerf.Engine.Offsets.Error.NoError)
                    {
                        return false;
                    }
                }
            }

            // Vertex list check.
            Explorer expFV = new(f, ShapeType.Vertex);
            for (; expFV.More(); expFV.Next())
            {
                Vertex v = ShapeCasts.Vertex(expFV.Current());
                pnt2d = Tool.Parameters(v, f);

                _error = CheckSinglePoint(pnt2d.X, pnt2d.Y, surf, bad3dPnts);
                if (_error != global::Kerf.Engine.Offsets.Error.NoError)
                {
                    return false;
                }
            }
        }

        return true;
    }

    //=======================================================================
    // function : GetBadShape
    // purpose  : Get shape where problems detected.
    //=======================================================================

    /// <summary>The shape where the pre-analysis found a problem (frozen).</summary>
    public Shape GetBadShape()
    {
        Shape bad = InternalOriginal(_badShape);
        bad.TShape()?.FreezeGraph();
        return bad;
    }

    //=================================================================================================

    private void RemoveInternalEdges()
    {
        bool bRemoveWire, bRemoveEdge;
        Explorer expF = new(), expW = new(), expE = new();
        IndexedMap<Shape, NCollection_List<Shape>> dmelf = new(ShapeHasher.Instance);
        //
        Exploration.MapShapesAndAncestors(_offsetShape, ShapeType.Edge, ShapeType.Face, dmelf);
        //
        expF.Init(_offsetShape, ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Shape f = expF.Current();
            //
            NCollection_List<Shape> liw = new();
            //
            expW.Init(f, ShapeType.Wire);
            for (; expW.More(); expW.Next())
            {
                Shape w = expW.Current();
                //
                bRemoveWire = true;
                NCollection_List<Shape> lie = new();
                //
                expE.Init(w, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (e.Orientation != Internal)
                    {
                        bRemoveWire = false;
                        continue;
                    }
                    //
                    NCollection_List<Shape> lf = dmelf.GetValue(e);
                    bRemoveEdge = (lf.Extent() == 1);
                    if (bRemoveEdge)
                    {
                        lie.Append(e);
                    }
                    else
                    {
                        bRemoveWire = false;
                    }
                }
                //
                if (bRemoveWire)
                {
                    liw.Append(w);
                }
                else if (lie.Extent() != 0)
                {
                    RemoveShapes(ref w, lie);
                }
            }
            //
            if (liw.Extent() != 0)
            {
                RemoveShapes(ref f, liw);
            }
        }
    }

    //=======================================================================
    // static methods implementation
    //=======================================================================

    //=======================================================================
    // function : checkSinglePoint
    // purpose  : Check single point on surface for bad normals
    //=======================================================================
    private static Error CheckSinglePoint(double uParam, double vParam, Surface surf,
                                                     List<Point3D> badPoints)
    {
        surf.D1(uParam, vParam, out Point3D pnt, out Vector3D d1U, out Vector3D d1V);

        if (d1U.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.SquareConfusion() || d1V.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            bool isKnownBadPnt = false;
            for (int idx = 0; idx <= (badPoints.Count - 1); ++idx)
            {
                if (pnt.SquareDistanceTo(badPoints[idx]) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    isKnownBadPnt = true;
                    break;
                }
            } // for(int anIdx  = theBadPoints.Lower();

            if (!isKnownBadPnt)
            {
                return global::Kerf.Engine.Offsets.Error.BadNormalsOnGeometry;
            }
            else
            {
                return global::Kerf.Engine.Offsets.Error.NoError;
            }
        } //  if (aD1U.SquareMagnitude() < Precision::SquareConfusion() ||

        if (d1U.IsParallel(d1V, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            // Isolines are collinear.
            return global::Kerf.Engine.Offsets.Error.BadNormalsOnGeometry;
        }

        return global::Kerf.Engine.Offsets.Error.NoError;
    }

    //=======================================================================
    // function : RemoveShapes
    // purpose  : Removes the shapes <theLS> from the shape <theS>
    //=======================================================================
    private static void RemoveShapes(ref Shape s, NCollection_List<Shape> ls)
    {
        BrepBuilder bb = new();
        //
        bool bFree = s.Free();
        s.Free(true);
        //
        NCollection_List<Shape>.Iterator it = new(ls);
        for (; it.More(); it.Next())
        {
            Shape si = it.Value();
            bb.Remove(s, si);
        }
        //
        s.Free(bFree);
    }

    //=======================================================================
    // function : UpdateHistory
    // purpose  : Updates the history information
    //=======================================================================
    private static void UpdateHistory(NCollection_List<Shape> lf, BooleanBuilder gf, ShapeImage image)
    {
        NCollection_List<Shape>.Iterator it = new(lf);
        for (; it.More(); it.Next())
        {
            Shape f = it.Value();
            NCollection_List<Shape> lfIm = new(gf.Modified(f));
            if (lfIm.Extent() != 0)
            {
                if (image.HasImage(f))
                {
                    image.Add(f, lfIm);
                }
                else
                {
                    image.Bind(f, lfIm);
                }
            }
        }
    }

    //=================================================================================================

    private void IntersectEdges(NCollection_List<Shape> faces,
                                Dictionary<Shape, Offset> mapSF,
                                Dictionary<Shape, Shape> mes,
                                Dictionary<Shape, Shape> build, ref AsDes asDes,
                                ref AsDes asDes2d, ProgressRange range)
    {
        double tolF;
        IndexedMap<Shape, NCollection_List<Shape>> dmvv = new(ShapeHasher.Instance);
        // intersect edges created from edges
        IndexedSet<Shape> mfv = new(ShapeHasher.Instance);
        using ProgressScope psOuter = new(range, null, 2);
        using ProgressScope ps1 = new(psOuter.Next(), null, faces.Extent());
        for (NCollection_List<Shape>.Iterator it = new(faces); it.More(); it.Next())
        {
            Face f = ShapeCasts.Face(it.Value());
            tolF = Tool.Tolerance(f);
            if (!Inter2d.ConnexIntByInt(f, mapSF.GetValue(f), mes, build, asDes, asDes2d, _offset,
                                                   tolF, _analyse, mfv, _imageVV, _edgeIntEdges, dmvv,
                                                   ps1.Next()))
            {
                _error = global::Kerf.Engine.Offsets.Error.CannotExtentEdge;
                return;
            }
            if (!ps1.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
        }
        // intersect edges created from vertices
        int i, nbF = mfv.Count;
        using ProgressScope ps2 = new(psOuter.Next(), "Intersecting edges created from vertices", nbF);
        for (i = 1; i <= nbF; ++i)
        {
            Face f = ShapeCasts.Face(mfv[i - 1]);
            tolF = Tool.Tolerance(f);
            Inter2d.ConnexIntByIntInVert(f, mapSF.GetValue(f), mes, build, asDes, asDes2d, tolF,
                                                    _analyse, dmvv, ps2.Next());
            if (!ps2.More())
            {
                _error = global::Kerf.Engine.Offsets.Error.UserBreak;
                return;
            }
        }
        //
        // fuse vertices on edges
        if (!Inter2d.FuseVertices(dmvv, asDes2d, _imageVV))
        {
            _error = global::Kerf.Engine.Offsets.Error.CannotFuseVertices;
            return;
        }
    }

    //=================================================================================================

    private static bool TrimEdges(in Shape shape, double offset, Analyse analyse,
                                  Dictionary<Shape, Offset> mapSF,
                                  Dictionary<Shape, Shape> mes,
                                  Dictionary<Shape, Shape> build, ref AsDes asDes,
                                  ref AsDes asDes2d, IndexedSet<Shape> newEdges,
                                  Dictionary<Shape, Shape> eTrimEInf,
                                  Dictionary<Shape, NCollection_List<Shape>> edgesOrigins)
    {
        Explorer exp = new(), exp2 = new(), expC = new();
        Shape NE;
        Face nf;

        NCollection_List<Shape> lFaces = new();
        for (exp.Init(shape, ShapeType.Face); exp.More(); exp.Next())
        {
            lFaces.Append(exp.Current());
        }

        HashSet<Shape> mfGenerated = new(ShapeHasher.Instance);
        IndexedMap<Shape, NCollection_List<Shape>> dmef = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator it = new(analyse.NewFaces()); it.More(); it.Next())
        {
            Shape fg = it.Value();
            lFaces.Append(fg);
            mfGenerated.Add(fg);
            Exploration.MapShapesAndUniqueAncestors(fg, ShapeType.Edge, ShapeType.Face, dmef);
        }

        for (NCollection_List<Shape>.Iterator it = new(lFaces); it.More(); it.Next())
        {
            Face fi = ShapeCasts.Face(it.Value());
            nf = mapSF.GetValue(fi).InternalFace();
            if (mes.ContainsKey(nf))
            {
                nf = ShapeCasts.Face(mes.GetValue(nf));
            }
            //
            HashSet<Shape> view = new(ShapeHasher.Instance);
            IndexedSet<Shape> vEmap = new(ShapeHasher.Instance);
            int i, nb;
            //
            Exploration.MapShapes(fi.Oriented(Forward), ShapeType.Edge, vEmap);
            Exploration.MapShapes(fi.Oriented(Forward), ShapeType.Vertex, vEmap);
            //
            nb = vEmap.Count;
            for (i = 1; i <= nb; ++i)
            {
                Shape s = vEmap[i - 1];
                if (!view.Add(s))
                {
                    continue;
                }
                //
                if (build.ContainsKey(s))
                {
                    NE = build.GetValue(s);
                    // keep connection to original edges
                    expC.Init(NE, ShapeType.Edge);
                    for (; expC.More(); expC.Next())
                    {
                        Edge nec = ShapeCasts.Edge(expC.Current());
                        if (!edgesOrigins.ContainsKey(nec))
                        {
                            edgesOrigins[nec] = new NCollection_List<Shape>();
                        }
                        AppendToList(edgesOrigins.GetValue(nec), s);
                    }
                    // trim edges
                    if (NE.Type == ShapeType.Edge)
                    {
                        if (AddNew(newEdges, NE))
                        {
                            Edge ne = ShapeCasts.Edge(NE);
                            if (!TrimEdge(ref ne, asDes2d, ref asDes, eTrimEInf))
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        //------------------------------------------------------------
                        // The Intersections are on several edges.
                        // The pieces without intersections with neighbors
                        // are removed from AsDes.
                        //------------------------------------------------------------
                        for (expC.Init(NE, ShapeType.Edge); expC.More(); expC.Next())
                        {
                            Edge nec = ShapeCasts.Edge(expC.Current());
                            if (AddNew(newEdges, nec))
                            {
                                if (!asDes2d.Descendant(nec).IsEmpty())
                                {
                                    if (!TrimEdge(ref nec, asDes2d, ref asDes, eTrimEInf))
                                    {
                                        return false;
                                    }
                                }
                                else
                                {
                                    if (asDes.HasAscendant(nec))
                                    {
                                        asDes.Remove(nec);
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (s.Type != ShapeType.Edge)
                    {
                        continue;
                    }
                    if (mfGenerated.Contains(fi) && dmef.GetValue(s).Extent() == 1)
                    {
                        continue;
                    }

                    NE = mapSF.GetValue(fi).InternalGenerated(s);
                    //// modified by jgv, 19.12.03 for OCC4455 ////
                    NE.Orient(s.Orientation);
                    //
                    if (!edgesOrigins.ContainsKey(NE))
                    {
                        edgesOrigins[NE] = new NCollection_List<Shape>();
                    }
                    AppendToList(edgesOrigins.GetValue(NE), s);
                    //
                    if (mes.ContainsKey(NE))
                    {
                        NE = mes.GetValue(NE);
                        NE.Orient(s.Orientation);
                        if (AddNew(newEdges, NE))
                        {
                            Edge ne = ShapeCasts.Edge(NE);
                            if (!TrimEdge(ref ne, asDes2d, ref asDes, eTrimEInf))
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        Edge edge = ShapeCasts.Edge(NE);
                        EdgeCurveAdaptor bac = new(edge);
                        if (bac.GetType() == Line)
                        {
                            Inter2d.ExtentEdge(edge, out Edge newEdge, offset);
                            eTrimEInf[edge] = newEdge;
                        }
                    }
                    asDes.Add(nf, NE);
                }
            }
        }
        return true;
    }

    // OCCT: "if (theNewEdges.Add(NE))" tests the index NCollection_IndexedMap::Add returns, never 0, so an edge already
    // in the map is trimmed again (occt-issues.md).
    private static bool AddNew(IndexedSet<Shape> map, in Shape s)
    {
        return map.Add(s) + 1 != 0;
    }

    //=======================================================================
    // function : TrimEdge
    // purpose  : Trim the edge of the largest of descendants in AsDes2d.
    //           Order in AsDes two vertices that have trimmed the edge.
    //=======================================================================
    private static bool TrimEdge(ref Edge ne, AsDes asDes2d, ref AsDes asDes,
                                 Dictionary<Shape, Shape> eTrimEInf)
    {
        //
        Exploration.Vertices(ne, out Vertex v1, out Vertex v2);
        Tool.Range(ne, out double t1, out double t2);
        //
        AlgoTools.MakeSplitEdge(ne, v1, t1, v2, t2, out Edge sourceEdge);
        //
        //
        double sameParTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        double u = 0.0;
        double uMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        double uMax = -uMin;

        NCollection_List<Shape> le = asDes2d.Descendant(ne);
        //
        bool bTrim = false;
        //
        if (le.Extent() > 1)
        {
            NCollection_List<Shape>.Iterator it = new(le);
            for (; it.More(); it.Next())
            {
                Vertex v = ShapeCasts.Vertex(it.Value());
                if (ne.Orientation == global::Kerf.Topology.Orientation.Reversed)
                {
                    v.Reverse();
                }
                // V.Orientation(TopAbs_INTERNAL);
                if (!FindParameter(v, ne, ref u))
                {
                    Curve3D theCurve = Tool.Curve(ne, out double f, out double l)!;
                    Point3D thePoint = Tool.Pnt(v);
                    ProjectPointOnCurve projector = new(thePoint, theCurve);
                    if (projector.NbPoints() == 0)
                    {
                        return false;
                    }
                    u = projector.LowerDistanceParameter();
                }
                if (u < uMin)
                {
                    uMin = u;
                    v1 = v;
                }
                if (u > uMax)
                {
                    uMax = u;
                    v2 = v;
                }
            }
            //
            if (v1.IsNull || v2.IsNull)
            {
                return false;
            }
            if (!v1.IsSame(v2))
            {
                ne.Free(true);
                BrepBuilder b = new();
                Orientation or = ne.Orientation;
                ne.Orient(Forward);
                Exploration.Vertices(ne, out Vertex vf, out Vertex vl);
                b.Remove(ne, vf);
                b.Remove(ne, vl);
                b.Add(ne, v1.Oriented(Forward));
                b.Add(ne, v2.Oriented(global::Kerf.Topology.Orientation.Reversed));
                b.Range(ne, uMin, uMax);
                ne.Orient(or);
                asDes.Add(ne, v1.Oriented(Forward));
                asDes.Add(ne, v2.Oriented(global::Kerf.Topology.Orientation.Reversed));
                BrepLibrary.SameParameter(ne, sameParTol, true);
                //
                bTrim = true;
            }
        }
        //
        if (!bTrim)
        {
            EdgeCurveAdaptor bac = new(ne);
            if (bac.GetType() == Line)
            {
                if (asDes.HasAscendant(ne))
                {
                    asDes.Remove(ne);
                }
            }
        }
        else
        {
            if (!eTrimEInf.ContainsKey(ne))
            {
                eTrimEInf[ne] = sourceEdge;
            }
        }
        return true;
    }

    //=================================================================================================

    private static void GetEnlargedFaces(NCollection_List<Shape> faces,
                                         Dictionary<Shape, Offset> mapSF,
                                         Dictionary<Shape, Shape> mes,
                                         Dictionary<Shape, Shape> facesOrigins,
                                         ShapeImage image, NCollection_List<Shape> lsf)
    {
        for (NCollection_List<Shape>.Iterator it = new(faces); it.More(); it.Next())
        {
            Shape fi = it.Value();
            Shape ofi = mapSF.GetValue(fi).InternalFace();
            if (mes.ContainsKey(ofi))
            {
                Face localFace = ShapeCasts.Face(mes.GetValue(ofi));
                lsf.Append(localFace);
                image.SetRoot(localFace);
                facesOrigins[localFace] = fi;
            }
        }
    }

    //=======================================================================
    // function : BuildShellsCompleteInter
    // purpose  : Make the shells from list of faces using MakerVolume algorithm.
    //           In case there will be more than just one solid, it will be
    //           rebuilt using only outer faces.
    //=======================================================================
    private static bool BuildShellsCompleteInter(NCollection_List<Shape> lf, ShapeImage image,
                                                 ref Shape shells, ProgressRange range)
    {
        using ProgressScope ps = new(range, null, 5);
        // make solids
        MakerVolume mv1 = new();
        mv1.SetArguments(lf);
        // we need to intersect the faces to process the tangential faces
        mv1.SetIntersect(true);
        mv1.SetAvoidInternalShapes(true);
        mv1.Perform(ps.Next(3));
        //
        bool bDone = !mv1.HasErrors;
        if (!bDone)
        {
            return bDone;
        }
        //
        UpdateHistory(lf, mv1, image);
        //
        Shape result1 = mv1.InternalShape();
        if (result1.Type == ShapeType.Solid)
        {
            // result is the alone solid, nothing to do
            return GetSubShapes(result1, ShapeType.Shell, ref shells);
        }

        // OCCT's incremental allocators are left out.

        // Since the <theImage> object does not support multiple ancestors,
        // prepare local copy of the origins, which will be used to resolve
        // non-manifold solids produced by Maker Volume algorithm by comparison
        // of the normal directions of the split faces with their origins.
        Dictionary<Shape, NCollection_List<Shape>> origins = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLR = new(image.Roots());
        for (; itLR.More(); itLR.Next())
        {
            Shape fr = itLR.Value();

            // Find the last splits of the root face, including the ones
            // created during MakeVolume operation
            NCollection_List<Shape> lfIm = new();
            image.LastImage(fr, lfIm);

            NCollection_List<Shape>.Iterator itLFIm = new(lfIm);
            for (; itLFIm.More(); itLFIm.Next())
            {
                Shape fIm = itLFIm.Value();
                if (!origins.TryGetValue(fIm, out NCollection_List<Shape>? pLFOr))
                {
                    pLFOr = new NCollection_List<Shape>();
                    origins[fIm] = pLFOr;
                }
                pLFOr.Append(fr);
            }
        }

        // It is necessary to rebuild the solids, avoiding internal faces
        // Map faces to solids
        IndexedMap<Shape, NCollection_List<Shape>> dmfs = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(result1, ShapeType.Face, ShapeType.Solid, dmfs);
        //
        int i, nb = dmfs.Count;
        bDone = (nb > 0);
        if (!bDone)
        {
            // unable to build any solid
            return bDone;
        }
        //
        // get faces attached to only one solid
        NCollection_List<Shape> aLF = new();
        for (i = 1; i <= nb; ++i)
        {
            NCollection_List<Shape> ls = dmfs.ValueAt(i - 1);
            if (ls.Extent() == 1)
            {
                Shape f = dmfs.KeyAt(i - 1);
                aLF.Append(f);
            }
        }
        //
        // make solids from the new list
        MakerVolume mv2 = new();
        mv2.SetArguments(aLF);
        // no need to intersect this time
        mv2.SetIntersect(false);
        mv2.SetAvoidInternalShapes(true);
        mv2.Perform(ps.Next());
        bDone = !mv2.HasErrors;
        if (!bDone)
        {
            return bDone;
        }
        //
        Shape result2 = mv2.InternalShape();
        if (result2.Type == ShapeType.Solid)
        {
            return GetSubShapes(result2, ShapeType.Shell, ref shells);
        }
        //
        Explorer exp = new(result2, ShapeType.Face);
        bDone = exp.More();
        if (!bDone)
        {
            return bDone;
        }
        //
        aLF.Clear();
        dmfs.Clear();

        // the result is non-manifold - resolve it comparing normal
        // directions of the offset faces and original faces
        for (; exp.More(); exp.Next())
        {
            Face f = ShapeCasts.Face(exp.Current());
            if (!origins.TryGetValue(f, out NCollection_List<Shape>? pLFOr))
            {
                // Standard_ASSERT_INVOKE: "Origins map does not contain the split face" (debug builds only).
                continue;
            }
            // Check orientation
            NCollection_List<Shape>.Iterator itLOr = new(pLFOr);
            for (; itLOr.More(); itLOr.Next())
            {
                Face fOr = ShapeCasts.Face(itLOr.Value());
                if (OffsetTool.CheckPlanesNormals(f, fOr))
                {
                    aLF.Append(f);
                    break;
                }
            }
        }
        //
        // make solid from most outer faces with correct normal direction
        MakerVolume mv3 = new();
        mv3.SetArguments(aLF);
        mv3.SetIntersect(false);
        mv3.SetAvoidInternalShapes(true);
        mv3.Perform(ps.Next());
        bDone = !mv3.HasErrors;
        if (!bDone)
        {
            return bDone;
        }
        //
        Shape result3 = mv3.InternalShape();
        return GetSubShapes(result3, ShapeType.Shell, ref shells);
    }

    //=================================================================================================

    /// <summary>
    /// The shapes generated from the caller's sub-shape <paramref name="s"/> that are in the result (frozen, with the
    /// result).
    /// </summary>
    public ISchematiqList<Shape> Generated(in Shape s) => PublishList(InternalGenerated(s));

    /// <summary>
    /// The shapes modified from the caller's sub-shape <paramref name="shape"/>: the images of a closing face (frozen,
    /// with the result).
    /// </summary>
    public ISchematiqList<Shape> Modified(in Shape shape) => PublishList(InternalModified(shape));

    private ISchematiqList<Shape> PublishList(NCollection_List<Shape> list)
    {
        if (list.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        _offsetShape.TShape()?.FreezeGraph();
        foreach (Shape s in list)
        {
            s.TShape()?.FreezeGraph();
        }
        return PackedList.Own([.. list]);
    }

    /// <summary>
    /// OCCT's <c>Generated</c>, without freezing: the algorithm's own list (cleared by the next call), for the caller's
    /// sub-shape <paramref name="s0"/>, holding the shapes that stand for the result's (unit 10H5).
    /// </summary>
    internal NCollection_List<Shape> InternalGenerated(in Shape s0)
    {
        Shape theS = InternalCopied(s0);
        _generated.Clear();
        ShapeType type = theS.Type;
        // OCCT's switch falls through from VERTEX to EDGE to FACE.
        if (type == ShapeType.Vertex)
        {
            if (_analyse.HasAncestor(theS))
            {
                HashSet<Shape> mFence = new(ShapeHasher.Instance);
                NCollection_List<Shape> la = _analyse.Ancestors(theS);
                NCollection_List<Shape>.Iterator itLA = new(la);
                for (; _generated.IsEmpty() && itLA.More(); itLA.Next())
                {
                    Shape e = itLA.Value();
                    if (!_initOffsetEdge.HasImage(e))
                    {
                        continue;
                    }
                    NCollection_List<Shape> leIm = new();
                    _initOffsetEdge.LastImage(e, leIm);
                    NCollection_List<Shape>.Iterator itLEIm = new(leIm);
                    for (; _generated.IsEmpty() && itLEIm.More(); itLEIm.Next())
                    {
                        Iterator itV = new(itLEIm.Value());
                        for (; itV.More(); itV.Next())
                        {
                            if (!mFence.Add(itV.Value()))
                            {
                                _generated.Append(itV.Value());
                                break;
                            }
                        }
                    }
                }
            }
        }
        if (type == ShapeType.Vertex || type == ShapeType.Edge)
        {
            if (_initOffsetEdge.HasImage(theS))
            {
                _initOffsetEdge.LastImage(theS, _generated);
            }
        }
        if (type == ShapeType.Vertex || type == ShapeType.Edge || type == ShapeType.Face)
        {
            Shape s = theS;
            if (_facePlanfaceMap.TryGetValue(s, out Shape planface))
            {
                s = ShapeCasts.Face(planface);
            }

            if (!_faces.Contains(s) && _initOffsetFace.HasImage(s))
            {
                _initOffsetFace.LastImage(s, _generated);

                if (_faces.Count != 0)
                {
                    // Reverse generated shapes in case of small solids.
                    // Useful only for faces without influence on others.
                    NCollection_List<Shape>.Iterator it = new(_generated);
                    for (; it.More(); it.Next())
                    {
                        it.ChangeValue().Reverse();
                    }
                }
            }
        }
        else if (type == ShapeType.Solid)
        {
            if (theS.IsSame(_shape))
            {
                _generated.Append(_offsetShape);
            }
        }

        if (_resMap.Count == 0)
        {
            Exploration.MapShapes(_offsetShape, _resMap);
        }

        // Kerf: the run's shapes are replaced by what stands for them in the result (unit 10H5).
        for (NCollection_List<Shape>.Iterator it = new(_generated); it.More(); it.Next())
        {
            it.ChangeValue() = InternalFinal(it.Value());
        }

        for (NCollection_List<Shape>.Iterator it = new(_generated); it.More();)
        {
            if (_resMap.Contains(it.Value()))
            {
                it.Next();
            }
            else
            {
                _generated.Remove(it);
            }
        }

        return _generated;
    }

    //=================================================================================================

    /// <summary>
    /// OCCT's <c>Modified</c>, without freezing: the algorithm's own list (cleared by the next call), for the caller's
    /// sub-shape <paramref name="shape0"/>.
    /// </summary>
    internal NCollection_List<Shape> InternalModified(in Shape shape0)
    {
        Shape theShape = InternalCopied(shape0);
        _generated.Clear();

        if (theShape.Type == ShapeType.Face)
        {
            Shape s = theShape;
            if (_facePlanfaceMap.TryGetValue(s, out Shape planface))
            {
                s = ShapeCasts.Face(planface);
            }

            if (_faces.Contains(s) && _initOffsetFace.HasImage(s))
            {
                _initOffsetFace.LastImage(s, _generated);

                if (_faces.Count != 0)
                {
                    // Reverse generated shapes in case of small solids.
                    // Useful only for faces without influence on others.
                    NCollection_List<Shape>.Iterator it = new(_generated);
                    for (; it.More(); it.Next())
                    {
                        it.ChangeValue().Reverse();
                    }
                }
            }
        }

        // Kerf: the run's shapes are replaced by what stands for them in the result (unit 10H5).
        for (NCollection_List<Shape>.Iterator it = new(_generated); it.More(); it.Next())
        {
            it.ChangeValue() = InternalFinal(it.Value());
        }

        return _generated;
    }

    //=================================================================================================

    /// <summary>True if the caller's sub-shape <paramref name="s"/> has been deleted.</summary>
    public bool IsDeleted(in Shape s)
    {
        if (_resMap.Count == 0)
        {
            Exploration.MapShapes(_offsetShape, _resMap);
        }

        // Kerf: a sub-shape of the caller's that OCCT keeps in its result, changed in place (a thickened shell's edge
        // given a pcurve), is in the result as what stands for it (unit 10H5).
        if (_resMap.Contains(s) || _resMap.Contains(InternalFinal(InternalCopied(s))))
        {
            return false;
        }

        return InternalGenerated(s).IsEmpty() && InternalModified(s).IsEmpty();
    }

    //=================================================================================================

    private static bool GetSubShapes(in Shape shape, ShapeType ssType, ref Shape result)
    {
        Explorer exp = new(shape, ssType);
        if (!exp.More())
        {
            return false;
        }
        //
        Compound aResult = new();
        new BrepBuilder().MakeCompound(ref aResult);
        //
        for (; exp.More(); exp.Next())
        {
            Shape ss = exp.Current();
            new BrepBuilder().Add(aResult, ss);
        }
        result = aResult;
        return true;
    }

    //=================================================================================================

    /// <summary>Analyzes the progress steps of the whole operation.</summary>
    private void AnalyzeProgress(double whole, NCollection_Array1<double> steps)
    {
        steps.Init(0.0);

        // Set, approximately, the proportions for each operation.
        // It is not a problem that the sum of the set values will not
        // be equal to 100%, as the values will be normalized.
        // The main point is to make the proportions valid relatively each other.

        // Proportions will be different for different connection types
        bool isArc = (_join == Arc);
        bool isPlanarIntCase = _inter && !isArc && _isPlanar && !_thickening && _faces.Count == 0 && IsSolid(_shape);

        steps[PIOperation_CheckInputData] = 1.0;
        steps[PIOperation_Analyse] = 2.0;
        steps[PIOperation_BuildOffsetBy] = isPlanarIntCase ? 70.0 : (isArc ? 20.0 : 50.0);
        steps[PIOperation_Intersection] = isPlanarIntCase ? 0.0 : (isArc ? 50.0 : 20.0);
        if (_thickening)
        {
            steps[PIOperation_MakeMissingWalls] = 5.0;
        }
        steps[PIOperation_MakeShells] = isPlanarIntCase ? 25.0 : 5.0;
        steps[PIOperation_MakeSolid] = 5.0;
        if (_isPerformSewing && _thickening)
        {
            steps[PIOperation_Sewing] = 10.0;
        }

        NormalizeSteps(whole, steps);
    }

    //=======================================================================
    // function : IsPlanar
    // purpose  : Checks if all the faces of the shape are planes
    //=======================================================================
    private bool IsPlanar()
    {
        bool isNonPlanarFound = false;
        BrepBuilder bb = new();

        Explorer exp = new(_shape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Face f = ShapeCasts.Face(exp.Current());
            FaceSurfaceAdaptor bas = new(f, false);
            if (bas.GetType() == Plane)
            {
                continue;
            }

            if (_isLinearizationAllowed)
            {
                // define the toleance
                double tolForFace = Tool.Tolerance(f);

                // try to linearize
                Surface surf = Tool.Surface(f)!;
                IsPlanarSurface planarityChecker = new(surf, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (planarityChecker.IsPlanar)
                {
                    Plane3D pln = planarityChecker.Plan();
                    double um, vm;
                    surf.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
                    bool isInf1 = global::Kerf.Engine.Foundation.Precision.IsInfinite(u1), isInf2 = global::Kerf.Engine.Foundation.Precision.IsInfinite(u2);
                    if (!isInf1 && !isInf2)
                    {
                        um = (u1 + u2) / 2.0;
                    }
                    else if (isInf1 && !isInf2)
                    {
                        um = u2 - 1.0;
                    }
                    else if (!isInf1 && isInf2)
                    {
                        um = u1 + 1.0;
                    }
                    else // isInf1 && isInf2
                    {
                        um = 0.0;
                    }
                    isInf1 = global::Kerf.Engine.Foundation.Precision.IsInfinite(v1);
                    isInf2 = global::Kerf.Engine.Foundation.Precision.IsInfinite(v2);
                    if (!isInf1 && !isInf2)
                    {
                        vm = (v1 + v2) / 2.0;
                    }
                    else if (isInf1 && !isInf2)
                    {
                        vm = v2 - 1.0;
                    }
                    else if (!isInf1 && isInf2)
                    {
                        vm = v1 + 1.0;
                    }
                    else // isInf1 && isInf2
                    {
                        vm = 0.0;
                    }
                    bas.D1(um, vm, out Point3D p, out Vector3D d1, out Vector3D d2);
                    Vector3D norm = d1.Crossed(d2);
                    Direction3D plnNorm = pln.Position.Direction;
                    if (norm.Dot(new Vector3D(plnNorm)) < 0.0)
                    {
                        plnNorm = plnNorm.Reversed();
                        Axis3D ax = new(pln.Position.Location, plnNorm);
                        pln = pln.WithAxis(ax);
                    }
                    PlaneSurface plane = new(pln);
                    Face planarFace = new();
                    bb.MakeFace(ref planarFace, plane, tolForFace);
                    Face faceForward = f;
                    faceForward.Orient(Forward);
                    Iterator itFace = new(faceForward);
                    for (; itFace.More(); itFace.Next())
                    {
                        Shape wire = itFace.Value();
                        bb.Add(planarFace, wire);
                    }
                    RemoveSeamAndDegeneratedEdges(planarFace, faceForward);
                    _facePlanfaceMap[f] = planarFace;
                    if (_faces.Contains(f))
                    {
                        _faces.Remove(f);
                        _faces.Add(planarFace);
                    }
                }
                else
                {
                    isNonPlanarFound = true;
                }
            }
            else
            {
                isNonPlanarFound = true;
            }
        }

        return (!isNonPlanarFound);
    }

    //=======================================================================
    // function : RemoveSeamAndDegeneratedEdges
    // purpose  : Removes useless seam and degenerated edges from a face that becomes planar
    //=======================================================================
    private static void RemoveSeamAndDegeneratedEdges(in Face face, in Face oldFace)
    {
        Face aFace = face;
        aFace.Orient(Forward);

        bool isDegOrSeamFound = false;
        NCollection_Sequence<Shape> eseq = new();
        Explorer explo = new(aFace, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Edge edge = ShapeCasts.Edge(explo.Current());
            if (Tool.Degenerated(edge) || BrepTools.IsReallyClosed(edge, oldFace))
            {
                isDegOrSeamFound = true;
            }
            else
            {
                eseq.Append(edge);
            }
        }

        if (!isDegOrSeamFound)
        {
            return;
        }

        // Reconstruct wires
        BrepBuilder bb = new();
        NCollection_List<Shape> wlist = new();
        Iterator itFace = new(aFace);
        for (; itFace.More(); itFace.Next())
        {
            wlist.Append(itFace.Value());
        }

        aFace.Free(true);
        NCollection_List<Shape>.Iterator itl = new(wlist);
        for (; itl.More(); itl.Next())
        {
            bb.Remove(aFace, itl.Value());
        }

        while (!eseq.IsEmpty())
        {
            Wire newWire = new();
            bb.MakeWire(ref newWire);
            Edge curEdge = ShapeCasts.Edge(eseq[1]);
            bb.Add(newWire, curEdge);
            eseq.Remove(1);
            Exploration.Vertices(curEdge, out Vertex firstVertex, out Vertex curVertex, true); // with orientation
            while (!curVertex.IsSame(firstVertex))
            {
                Vertex v1 = new(), v2 = new();
                int ind;
                for (ind = 1; ind <= eseq.Length(); ind++)
                {
                    curEdge = ShapeCasts.Edge(eseq[ind]);
                    Exploration.Vertices(curEdge, out v1, out v2, true); // with orientation
                    if (v1.IsSame(curVertex))
                    {
                        break;
                    }
                }
                if (ind > eseq.Length())
                { // error occurred: wire is not closed
                    break;
                }

                bb.Add(newWire, curEdge);
                eseq.Remove(ind);
                curVertex = v2;
            }

            bb.Add(aFace, newWire);
        }
    }

    //=================================================================================================

    private static bool IsSolid(in Shape s)
    {
        Explorer exp = new(s, ShapeType.Solid);
        return exp.More();
    }

    //=======================================================================
    // function : AppendToList
    // purpose  : Add to a list only unique elements
    //=======================================================================
    private static void AppendToList(NCollection_List<Shape> list, in Shape shape)
    {
        NCollection_List<Shape>.Iterator it = new(list);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            if (s.IsSame(shape))
            {
                return;
            }
        }
        list.Append(shape);
    }

    //=================================================================================================

    /// <summary>
    /// The analysis of the run (OCCT's <c>GetAnalyse</c>): internal, since its tangent treatment holds the
    /// offset's working shapes (decision 1007).
    /// </summary>
    internal Analyse GetAnalyse() => _analyse;
}
