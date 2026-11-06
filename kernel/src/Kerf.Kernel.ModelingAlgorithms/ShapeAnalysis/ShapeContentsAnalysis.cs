// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_ShapeContents.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_ShapeContents.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Counts the sub-shapes of a shape and its geometric peculiarities (offset, indirect, trimmed, big B-spline
/// geometry, seams, missing pcurves, free and shared sub-shapes), and collects the faces or edges concerned when the
/// corresponding mode is set. Only reads the shape.
/// </summary>
/// <remarks>
/// A working object (decision 21): the modes are OCCT's modifiable flags (<c>ref bool</c>), and the sequences
/// (<see cref="BigSplineSec"/>, ...) are the object's own, cleared by the next <see cref="Perform"/>.
/// </remarks>
internal sealed class ShapeContentsAnalysis
{
    private int _nbSolids;
    private int _nbShells;
    private int _nbFaces;
    private int _nbWires;
    private int _nbEdges;
    private int _nbVertices;
    private int _nbSolidsWithVoids;
    private int _nbBigSplines;
    private int _nbC0Surfaces;
    private int _nbC0Curves;
    private int _nbOffsetSurf;
    private int _nbIndirectSurf;
    private int _nbOffsetCurves;
    private int _nbTrimmedCurve2d;
    private int _nbTrimmedCurve3d;
    private int _nbBSplibeSurf;
    private int _nbBezierSurf;
    private int _nbTrimSurf;
    private int _nbWireWitnSeam;
    private int _nbWireWithSevSeams;
    private int _nbFaceWithSevWires;
    private int _nbNoPCurve;
    private int _nbFreeFaces;
    private int _nbFreeWires;
    private int _nbFreeEdges;
    private int _nbSharedSolids;
    private int _nbSharedShells;
    private int _nbSharedFaces;
    private int _nbSharedWires;
    private int _nbSharedFreeWires;
    private int _nbSharedFreeEdges;
    private int _nbSharedEdges;
    private int _nbSharedVertices;
    private bool _bigSplineMode;
    private bool _indirectMode;
    private bool _offsetSurfaceMode;
    private bool _trimmed3dMode;
    private bool _offsetCurveMode;
    private bool _trimmed2dMode;
    private readonly NCollection_HSequence<Shape> _bigSplineSec;
    private readonly NCollection_HSequence<Shape> _indirectSec;
    private readonly NCollection_HSequence<Shape> _offsetSurfaceSec;
    private readonly NCollection_HSequence<Shape> _trimmed3dSec;
    private readonly NCollection_HSequence<Shape> _offsetCurveSec;
    private readonly NCollection_HSequence<Shape> _trimmed2dSec;

    /// <summary>Initializes the fields and clears the flags.</summary>
    public ShapeContentsAnalysis()
    {
        _bigSplineSec = new NCollection_HSequence<Shape>();
        _indirectSec = new NCollection_HSequence<Shape>();
        _offsetSurfaceSec = new NCollection_HSequence<Shape>();
        _trimmed3dSec = new NCollection_HSequence<Shape>();
        _offsetCurveSec = new NCollection_HSequence<Shape>();
        _trimmed2dSec = new NCollection_HSequence<Shape>();
        ClearFlags();
    }

    /// <summary>Clears all accumulated statistics.</summary>
    public void Clear()
    {
        _nbSolids = 0;
        _nbShells = 0;
        _nbFaces = 0;
        _nbWires = 0;
        _nbEdges = 0;
        _nbVertices = 0;
        _nbSolidsWithVoids = 0;
        _nbBigSplines = 0;
        _nbC0Surfaces = 0;
        _nbC0Curves = 0;
        _nbOffsetSurf = 0;
        _nbIndirectSurf = 0;
        _nbOffsetCurves = 0;
        _nbTrimmedCurve2d = 0;
        _nbTrimmedCurve3d = 0;
        _nbBSplibeSurf = 0;
        _nbBezierSurf = 0;
        _nbTrimSurf = 0;
        _nbWireWitnSeam = 0;
        _nbWireWithSevSeams = 0;
        _nbFaceWithSevWires = 0;
        _nbNoPCurve = 0;
        _nbFreeFaces = 0;
        _nbFreeWires = 0;
        _nbFreeEdges = 0;

        _nbSharedSolids = 0;
        _nbSharedShells = 0;
        _nbSharedFaces = 0;
        _nbSharedWires = 0;
        _nbSharedFreeWires = 0;
        _nbSharedFreeEdges = 0;
        _nbSharedEdges = 0;
        _nbSharedVertices = 0;

        _bigSplineSec.Clear();
        _indirectSec.Clear();
        _offsetSurfaceSec.Clear();
        _trimmed3dSec.Clear();
        _offsetCurveSec.Clear();
        _trimmed2dSec.Clear();
    }

    /// <summary>Clears all flags.</summary>
    public void ClearFlags()
    {
        _bigSplineMode = false;
        _indirectMode = false;
        _offsetSurfaceMode = false;
        _trimmed3dMode = false;
        _offsetCurveMode = false;
        _trimmed2dMode = false;
    }

    /// <summary>Counts the sub-shapes of the shape and stores sub-shapes according to the flags.</summary>
    public void Perform(in Shape shape)
    {
        Clear();
        //  On y va
        Explorer exp = new();
        var mapsh = new HashSet<Shape>(ShapeHasher.Instance);
        //  On note pour les SOLIDES : ceux qui ont des trous (plus d un SHELL)

        for (exp.Init(shape, ShapeType.Solid); exp.More(); exp.Next())
        {
            Solid sol = ShapeCasts.Solid(exp.Current());
            sol.Location(new Placement());
            mapsh.Add(sol);
            int nbs = 0;
            for (var shel = new Explorer(sol, ShapeType.Shell); shel.More(); shel.Next())
            {
                nbs++;
            }
            if (nbs > 1)
            {
                _nbSolidsWithVoids++;
            }
            _nbSolids++;
        }
        _nbSharedSolids = mapsh.Count;

        //  Pour les SHELLS, on compte les faces dans les SHELLS
        //  Ensuite une soustraction, et on a les faces libres
        mapsh.Clear();
        int nbfaceshell = 0;
        for (exp.Init(shape, ShapeType.Shell); exp.More(); exp.Next())
        {
            _nbShells++;
            Shell she = ShapeCasts.Shell(exp.Current());
            she.Location(new Placement());
            mapsh.Add(she);
            for (var shel = new Explorer(she, ShapeType.Face); shel.More(); shel.Next())
            {
                nbfaceshell++;
            }
        }
        _nbSharedShells = mapsh.Count;
        //  On note pour les FACES pas mal de choses (surface, topologie)
        //  * Surface BSpline > 8192 poles
        //  * Surface BSpline "OnlyC0" (not yet impl)
        //  * Surface Offset
        //  * Surface Elementaire INDIRECTE
        //  * Presence de COUTURES; en particulier WIRE A PLUS D UNE COUTURE
        //  * Edge : OffsetCurve

        mapsh.Clear();
        for (exp.Init(shape, ShapeType.Face); exp.More(); exp.Next())
        {
            Face face = ShapeCasts.Face(exp.Current());
            _nbFaces++;
            Surface? surf = Tool.Surface(face, out Placement _);
            face.Location(new Placement());
            mapsh.Add(face);
            if (surf is TrimmedSurface trsu)
            {
                _nbTrimSurf++;
                surf = trsu.BasisSurface;
            }
            // #10 rln 27/02/98 BUC50003 entity 56
            // C0 if at least in one direction (U or V)
            if (surf is not null && !(surf.IsCNu(1) && surf.IsCNv(1)))
            {
                _nbC0Surfaces++;
            }

            if (surf is BSplineSurface bsps)
            {
                _nbBSplibeSurf++;
                if (bsps.UPoleCount * bsps.VPoleCount > 8192)
                {
                    _nbBigSplines++;
                    if (_bigSplineMode)
                    {
                        _bigSplineSec.Append(face);
                    }
                }
            }
            if (surf is ElementarySurface els)
            {
                if (!els.Position.IsDirect)
                {
                    _nbIndirectSurf++;
                    if (_indirectMode)
                    {
                        _indirectSec.Append(face);
                    }
                }
            }
            // OCCT dereferences a null surface here (a face without one); the port throws NullReferenceException.
            if (surf!.IsKind(typeof(OffsetSurface)))
            {
                _nbOffsetSurf++;
                if (_offsetSurfaceMode)
                {
                    _offsetSurfaceSec.Append(face);
                }
            }
            else if (surf.IsKind(typeof(BezierSurface)))
            {
                _nbBezierSurf++;
            }

            int maxseam = 0, nbwires = 0;
            for (var wires = new Explorer(face, ShapeType.Wire); wires.More(); wires.Next())
            {
                Wire wire = ShapeCasts.Wire(wires.Current());
                int nbseam = 0;
                nbwires++;
                for (var edg = new Explorer(wire, ShapeType.Edge); edg.More(); edg.Next())
                {
                    Edge edge = ShapeCasts.Edge(edg.Current());
                    if (Tool.IsClosed(edge, face))
                    {
                        nbseam++;
                    }
                    Curve3D? c3d = Tool.Curve(edge, out double first, out double last);
                    if (c3d is not null)
                    {
                        if (c3d.IsKind(typeof(TrimmedCurve3D)))
                        {
                            _nbTrimmedCurve3d++;
                            if (_trimmed3dMode)
                            {
                                _trimmed3dSec.Append(face);
                            }
                        }
                    }
                    Curve2D? c2d = Tool.CurveOnSurface(edge, face, out first, out last);
                    if (c2d is null)
                    {
                        _nbNoPCurve++;
                    }
                    else if (c2d.IsKind(typeof(OffsetCurve2D)))
                    {
                        _nbOffsetCurves++;
                        if (_offsetCurveMode)
                        {
                            _offsetCurveSec.Append(face);
                        }
                    }
                    else if (c2d.IsKind(typeof(TrimmedCurve2D)))
                    {
                        _nbTrimmedCurve2d++;
                        if (_trimmed2dMode)
                        {
                            _trimmed2dSec.Append(face);
                        }
                    }
                }
                if (nbseam > maxseam)
                {
                    maxseam = nbseam;
                }
            }
            if (maxseam == 1)
            {
                _nbWireWitnSeam++;
            }
            else if (maxseam > 1)
            {
                _nbWireWithSevSeams++;
            }
            if (nbwires > 1)
            {
                _nbFaceWithSevWires++;
            }
        }
        _nbSharedFaces = mapsh.Count;

        mapsh.Clear();
        for (exp.Init(shape, ShapeType.Wire); exp.More(); exp.Next())
        {
            Wire wire = ShapeCasts.Wire(exp.Current());
            wire.Location(new Placement());
            mapsh.Add(wire);
            _nbWires++;
        }
        _nbSharedWires = mapsh.Count;

        //  Ne pas oublier les FACES :
        _nbFreeFaces = _nbFaces - nbfaceshell;

        mapsh.Clear();
        for (exp.Init(shape, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            edge.Location(new Placement());
            mapsh.Add(edge);
            _nbEdges++;
            Curve3D? c3d = Tool.Curve(edge, out Placement _, out double _, out double _);
            if (c3d is not null && c3d.IsKind(typeof(OffsetCurve3D)))
            {
                _nbOffsetCurves++;
                if (_offsetCurveMode)
                {
                    _offsetCurveSec.Append(edge);
                }
            }
            if (c3d is not null && !c3d.IsCN(1))
            {
                _nbC0Curves++;
            }
        }
        _nbSharedEdges = mapsh.Count;

        mapsh.Clear();
        for (exp.Init(shape, ShapeType.Vertex); exp.More(); exp.Next())
        {
            Vertex vert = ShapeCasts.Vertex(exp.Current());
            vert.Location(new Placement());
            _nbVertices++;
            mapsh.Add(vert);
        }
        _nbSharedVertices = mapsh.Count;

        mapsh.Clear();
        for (exp.Init(shape, ShapeType.Edge, ShapeType.Face); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            edge.Location(new Placement());
            _nbFreeEdges++;
            mapsh.Add(edge);
        }
        _nbSharedFreeEdges = mapsh.Count;

        mapsh.Clear();
        for (exp.Init(shape, ShapeType.Wire, ShapeType.Face); exp.More(); exp.Next())
        {
            Wire wire = ShapeCasts.Wire(exp.Current());
            wire.Location(new Placement());
            _nbFreeWires++;
            mapsh.Add(wire);
        }
        _nbSharedFreeWires = mapsh.Count;
    }

    /// <summary>The flag to store the faces with edges whose 3D curves have more than 8192 poles (modifiable).</summary>
    public ref bool ModifyBigSplineMode() => ref _bigSplineMode;

    /// <summary>The flag to store the faces on indirect surfaces (modifiable).</summary>
    public ref bool ModifyIndirectMode() => ref _indirectMode;

    /// <summary>The flag to store the faces on offset surfaces (modifiable).</summary>
    public ref bool ModifyOffsetSurfaceMode() => ref _offsetSurfaceMode;

    /// <summary>The flag to store the faces with trimmed 3D curves (modifiable).</summary>
    public ref bool ModifyTrimmed3dMode() => ref _trimmed3dMode;

    /// <summary>The flag to store the faces with offset pcurves and the edges with offset 3D curves (modifiable).</summary>
    public ref bool ModifyOffsetCurveMode() => ref _offsetCurveMode;

    /// <summary>The flag to store the faces with trimmed pcurves (modifiable).</summary>
    public ref bool ModifyTrimmed2dMode() => ref _trimmed2dMode;
    public int SolidCount => _nbSolids;
    public int ShellCount => _nbShells;
    public int FaceCount => _nbFaces;
    public int WireCount => _nbWires;
    public int EdgeCount => _nbEdges;
    public int VertexCount => _nbVertices;
    public int SolidsWithVoidCount => _nbSolidsWithVoids;
    public int BigSplineCount => _nbBigSplines;
    public int C0SurfaceCount => _nbC0Surfaces;
    public int C0CurveCount => _nbC0Curves;
    public int OffsetSurfCount => _nbOffsetSurf;
    public int IndirectSurfCount => _nbIndirectSurf;
    public int OffsetCurveCount => _nbOffsetCurves;
    public int TrimmedCurve2dCount => _nbTrimmedCurve2d;
    public int TrimmedCurve3dCount => _nbTrimmedCurve3d;
    public int BSplibeSurfCount => _nbBSplibeSurf;
    public int BezierSurfCount => _nbBezierSurf;
    public int TrimSurfCount => _nbTrimSurf;
    public int WireWitnSeamCount => _nbWireWitnSeam;
    public int WireWithSevSeamCount => _nbWireWithSevSeams;
    public int FaceWithSevWireCount => _nbFaceWithSevWires;
    public int NoPCurveCount => _nbNoPCurve;
    public int FreeFaceCount => _nbFreeFaces;
    public int FreeWireCount => _nbFreeWires;
    public int FreeEdgeCount => _nbFreeEdges;
    public int SharedSolidCount => _nbSharedSolids;
    public int SharedShellCount => _nbSharedShells;
    public int SharedFaceCount => _nbSharedFaces;
    public int SharedWireCount => _nbSharedWires;
    public int SharedFreeWireCount => _nbSharedFreeWires;
    public int SharedFreeEdgeCount => _nbSharedFreeEdges;
    public int SharedEdgeCount => _nbSharedEdges;
    public int SharedVertexCount => _nbSharedVertices;
    public NCollection_HSequence<Shape> BigSplineSec() => _bigSplineSec;
    public NCollection_HSequence<Shape> IndirectSec() => _indirectSec;
    public NCollection_HSequence<Shape> OffsetSurfaceSec() => _offsetSurfaceSec;
    public NCollection_HSequence<Shape> Trimmed3dSec() => _trimmed3dSec;
    public NCollection_HSequence<Shape> OffsetCurveSec() => _offsetCurveSec;
    public NCollection_HSequence<Shape> Trimmed2dSec() => _trimmed2dSec;
}
