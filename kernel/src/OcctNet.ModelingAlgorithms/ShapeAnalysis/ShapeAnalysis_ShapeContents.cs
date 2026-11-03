// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_ShapeContents.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_ShapeContents.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Counts the sub-shapes of a shape and its geometric peculiarities (offset, indirect, trimmed, big B-spline
/// geometry, seams, missing pcurves, free and shared sub-shapes), and collects the faces or edges concerned when the
/// corresponding mode is set. Only reads the shape.
/// </summary>
/// <remarks>
/// A working object (decision 21): the modes are OCCT's modifiable flags (<c>ref bool</c>), and the sequences
/// (<see cref="BigSplineSec"/>, ...) are the object's own, cleared by the next <see cref="Perform"/>.
/// </remarks>
public sealed class ShapeAnalysis_ShapeContents
{
    private int myNbSolids;
    private int myNbShells;
    private int myNbFaces;
    private int myNbWires;
    private int myNbEdges;
    private int myNbVertices;
    private int myNbSolidsWithVoids;
    private int myNbBigSplines;
    private int myNbC0Surfaces;
    private int myNbC0Curves;
    private int myNbOffsetSurf;
    private int myNbIndirectSurf;
    private int myNbOffsetCurves;
    private int myNbTrimmedCurve2d;
    private int myNbTrimmedCurve3d;
    private int myNbBSplibeSurf;
    private int myNbBezierSurf;
    private int myNbTrimSurf;
    private int myNbWireWitnSeam;
    private int myNbWireWithSevSeams;
    private int myNbFaceWithSevWires;
    private int myNbNoPCurve;
    private int myNbFreeFaces;
    private int myNbFreeWires;
    private int myNbFreeEdges;
    private int myNbSharedSolids;
    private int myNbSharedShells;
    private int myNbSharedFaces;
    private int myNbSharedWires;
    private int myNbSharedFreeWires;
    private int myNbSharedFreeEdges;
    private int myNbSharedEdges;
    private int myNbSharedVertices;
    private bool myBigSplineMode;
    private bool myIndirectMode;
    private bool myOffsetSurfaceMode;
    private bool myTrimmed3dMode;
    private bool myOffsetCurveMode;
    private bool myTrimmed2dMode;
    private readonly NCollection_HSequence<Shape> myBigSplineSec;
    private readonly NCollection_HSequence<Shape> myIndirectSec;
    private readonly NCollection_HSequence<Shape> myOffsetSurfaceSec;
    private readonly NCollection_HSequence<Shape> myTrimmed3dSec;
    private readonly NCollection_HSequence<Shape> myOffsetCurveSec;
    private readonly NCollection_HSequence<Shape> myTrimmed2dSec;

    /// <summary>Initializes the fields and clears the flags.</summary>
    public ShapeAnalysis_ShapeContents()
    {
        myBigSplineSec = new NCollection_HSequence<Shape>();
        myIndirectSec = new NCollection_HSequence<Shape>();
        myOffsetSurfaceSec = new NCollection_HSequence<Shape>();
        myTrimmed3dSec = new NCollection_HSequence<Shape>();
        myOffsetCurveSec = new NCollection_HSequence<Shape>();
        myTrimmed2dSec = new NCollection_HSequence<Shape>();
        ClearFlags();
    }

    /// <summary>Clears all accumulated statistics.</summary>
    public void Clear()
    {
        myNbSolids = 0;
        myNbShells = 0;
        myNbFaces = 0;
        myNbWires = 0;
        myNbEdges = 0;
        myNbVertices = 0;
        myNbSolidsWithVoids = 0;
        myNbBigSplines = 0;
        myNbC0Surfaces = 0;
        myNbC0Curves = 0;
        myNbOffsetSurf = 0;
        myNbIndirectSurf = 0;
        myNbOffsetCurves = 0;
        myNbTrimmedCurve2d = 0;
        myNbTrimmedCurve3d = 0;
        myNbBSplibeSurf = 0;
        myNbBezierSurf = 0;
        myNbTrimSurf = 0;
        myNbWireWitnSeam = 0;
        myNbWireWithSevSeams = 0;
        myNbFaceWithSevWires = 0;
        myNbNoPCurve = 0;
        myNbFreeFaces = 0;
        myNbFreeWires = 0;
        myNbFreeEdges = 0;

        myNbSharedSolids = 0;
        myNbSharedShells = 0;
        myNbSharedFaces = 0;
        myNbSharedWires = 0;
        myNbSharedFreeWires = 0;
        myNbSharedFreeEdges = 0;
        myNbSharedEdges = 0;
        myNbSharedVertices = 0;

        myBigSplineSec.Clear();
        myIndirectSec.Clear();
        myOffsetSurfaceSec.Clear();
        myTrimmed3dSec.Clear();
        myOffsetCurveSec.Clear();
        myTrimmed2dSec.Clear();
    }

    /// <summary>Clears all flags.</summary>
    public void ClearFlags()
    {
        myBigSplineMode = false;
        myIndirectMode = false;
        myOffsetSurfaceMode = false;
        myTrimmed3dMode = false;
        myOffsetCurveMode = false;
        myTrimmed2dMode = false;
    }

    /// <summary>Counts the sub-shapes of the shape and stores sub-shapes according to the flags.</summary>
    public void Perform(in Shape Shape)
    {
        Clear();
        //  On y va
        Explorer exp = new();
        var mapsh = new NCollection_Map<Shape>(ShapeHasher.Instance);
        //  On note pour les SOLIDES : ceux qui ont des trous (plus d un SHELL)

        for (exp.Init(Shape, ShapeType.Solid); exp.More(); exp.Next())
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
                myNbSolidsWithVoids++;
            }
            myNbSolids++;
        }
        myNbSharedSolids = mapsh.Extent();

        //  Pour les SHELLS, on compte les faces dans les SHELLS
        //  Ensuite une soustraction, et on a les faces libres
        mapsh.Clear();
        int nbfaceshell = 0;
        for (exp.Init(Shape, ShapeType.Shell); exp.More(); exp.Next())
        {
            myNbShells++;
            Shell she = ShapeCasts.Shell(exp.Current());
            she.Location(new Placement());
            mapsh.Add(she);
            for (var shel = new Explorer(she, ShapeType.Face); shel.More(); shel.Next())
            {
                nbfaceshell++;
            }
        }
        myNbSharedShells = mapsh.Extent();
        //  On note pour les FACES pas mal de choses (surface, topologie)
        //  * Surface BSpline > 8192 poles
        //  * Surface BSpline "OnlyC0" (not yet impl)
        //  * Surface Offset
        //  * Surface Elementaire INDIRECTE
        //  * Presence de COUTURES; en particulier WIRE A PLUS D UNE COUTURE
        //  * Edge : OffsetCurve

        mapsh.Clear();
        for (exp.Init(Shape, ShapeType.Face); exp.More(); exp.Next())
        {
            Face face = ShapeCasts.Face(exp.Current());
            myNbFaces++;
            Surface? surf = Tool.Surface(face, out Placement _);
            face.Location(new Placement());
            mapsh.Add(face);
            if (surf is TrimmedSurface trsu)
            {
                myNbTrimSurf++;
                surf = trsu.BasisSurface();
            }
            // #10 rln 27/02/98 BUC50003 entity 56
            // C0 if at least in one direction (U or V)
            if (surf is not null && !(surf.IsCNu(1) && surf.IsCNv(1)))
            {
                myNbC0Surfaces++;
            }

            if (surf is BSplineSurface bsps)
            {
                myNbBSplibeSurf++;
                if (bsps.UPoleCount * bsps.VPoleCount > 8192)
                {
                    myNbBigSplines++;
                    if (myBigSplineMode)
                    {
                        myBigSplineSec.Append(face);
                    }
                }
            }
            if (surf is ElementarySurface els)
            {
                if (!els.Position().Direct())
                {
                    myNbIndirectSurf++;
                    if (myIndirectMode)
                    {
                        myIndirectSec.Append(face);
                    }
                }
            }
            // OCCT dereferences a null surface here (a face without one); the port throws NullReferenceException.
            if (surf!.IsKind(typeof(OffsetSurface)))
            {
                myNbOffsetSurf++;
                if (myOffsetSurfaceMode)
                {
                    myOffsetSurfaceSec.Append(face);
                }
            }
            else if (surf.IsKind(typeof(BezierSurface)))
            {
                myNbBezierSurf++;
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
                            myNbTrimmedCurve3d++;
                            if (myTrimmed3dMode)
                            {
                                myTrimmed3dSec.Append(face);
                            }
                        }
                    }
                    Curve2D? c2d = Tool.CurveOnSurface(edge, face, out first, out last);
                    if (c2d is null)
                    {
                        myNbNoPCurve++;
                    }
                    else if (c2d.IsKind(typeof(OffsetCurve2D)))
                    {
                        myNbOffsetCurves++;
                        if (myOffsetCurveMode)
                        {
                            myOffsetCurveSec.Append(face);
                        }
                    }
                    else if (c2d.IsKind(typeof(TrimmedCurve2D)))
                    {
                        myNbTrimmedCurve2d++;
                        if (myTrimmed2dMode)
                        {
                            myTrimmed2dSec.Append(face);
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
                myNbWireWitnSeam++;
            }
            else if (maxseam > 1)
            {
                myNbWireWithSevSeams++;
            }
            if (nbwires > 1)
            {
                myNbFaceWithSevWires++;
            }
        }
        myNbSharedFaces = mapsh.Extent();

        mapsh.Clear();
        for (exp.Init(Shape, ShapeType.Wire); exp.More(); exp.Next())
        {
            Wire wire = ShapeCasts.Wire(exp.Current());
            wire.Location(new Placement());
            mapsh.Add(wire);
            myNbWires++;
        }
        myNbSharedWires = mapsh.Extent();

        //  Ne pas oublier les FACES :
        myNbFreeFaces = myNbFaces - nbfaceshell;

        mapsh.Clear();
        for (exp.Init(Shape, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            edge.Location(new Placement());
            mapsh.Add(edge);
            myNbEdges++;
            Curve3D? c3d = Tool.Curve(edge, out Placement _, out double _, out double _);
            if (c3d is not null && c3d.IsKind(typeof(OffsetCurve3D)))
            {
                myNbOffsetCurves++;
                if (myOffsetCurveMode)
                {
                    myOffsetCurveSec.Append(edge);
                }
            }
            if (c3d is not null && !c3d.IsCN(1))
            {
                myNbC0Curves++;
            }
        }
        myNbSharedEdges = mapsh.Extent();

        mapsh.Clear();
        for (exp.Init(Shape, ShapeType.Vertex); exp.More(); exp.Next())
        {
            Vertex vert = ShapeCasts.Vertex(exp.Current());
            vert.Location(new Placement());
            myNbVertices++;
            mapsh.Add(vert);
        }
        myNbSharedVertices = mapsh.Extent();

        mapsh.Clear();
        for (exp.Init(Shape, ShapeType.Edge, ShapeType.Face); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            edge.Location(new Placement());
            myNbFreeEdges++;
            mapsh.Add(edge);
        }
        myNbSharedFreeEdges = mapsh.Extent();

        mapsh.Clear();
        for (exp.Init(Shape, ShapeType.Wire, ShapeType.Face); exp.More(); exp.Next())
        {
            Wire wire = ShapeCasts.Wire(exp.Current());
            wire.Location(new Placement());
            myNbFreeWires++;
            mapsh.Add(wire);
        }
        myNbSharedFreeWires = mapsh.Extent();
    }

    /// <summary>The flag to store the faces with edges whose 3D curves have more than 8192 poles (modifiable).</summary>
    public ref bool ModifyBigSplineMode() => ref myBigSplineMode;

    /// <summary>The flag to store the faces on indirect surfaces (modifiable).</summary>
    public ref bool ModifyIndirectMode() => ref myIndirectMode;

    /// <summary>The flag to store the faces on offset surfaces (modifiable).</summary>
    public ref bool ModifyOffsetSurfaceMode() => ref myOffsetSurfaceMode;

    /// <summary>The flag to store the faces with trimmed 3D curves (modifiable).</summary>
    public ref bool ModifyTrimmed3dMode() => ref myTrimmed3dMode;

    /// <summary>The flag to store the faces with offset pcurves and the edges with offset 3D curves (modifiable).</summary>
    public ref bool ModifyOffsetCurveMode() => ref myOffsetCurveMode;

    /// <summary>The flag to store the faces with trimmed pcurves (modifiable).</summary>
    public ref bool ModifyTrimmed2dMode() => ref myTrimmed2dMode;

    /// <summary>OCCT's misspelt, deprecated form of <see cref="ModifyOffsetSurfaceMode"/>.</summary>
    [Obsolete("ModifyOffsetSurfaceMode() should be used instead")]
    public ref bool ModifyOffestSurfaceMode() => ref myOffsetSurfaceMode;

    public int NbSolids() => myNbSolids;
    public int NbShells() => myNbShells;
    public int NbFaces() => myNbFaces;
    public int NbWires() => myNbWires;
    public int NbEdges() => myNbEdges;
    public int NbVertices() => myNbVertices;
    public int NbSolidsWithVoids() => myNbSolidsWithVoids;
    public int NbBigSplines() => myNbBigSplines;
    public int NbC0Surfaces() => myNbC0Surfaces;
    public int NbC0Curves() => myNbC0Curves;
    public int NbOffsetSurf() => myNbOffsetSurf;
    public int NbIndirectSurf() => myNbIndirectSurf;
    public int NbOffsetCurves() => myNbOffsetCurves;
    public int NbTrimmedCurve2d() => myNbTrimmedCurve2d;
    public int NbTrimmedCurve3d() => myNbTrimmedCurve3d;
    public int NbBSplibeSurf() => myNbBSplibeSurf;
    public int NbBezierSurf() => myNbBezierSurf;
    public int NbTrimSurf() => myNbTrimSurf;
    public int NbWireWitnSeam() => myNbWireWitnSeam;
    public int NbWireWithSevSeams() => myNbWireWithSevSeams;
    public int NbFaceWithSevWires() => myNbFaceWithSevWires;
    public int NbNoPCurve() => myNbNoPCurve;
    public int NbFreeFaces() => myNbFreeFaces;
    public int NbFreeWires() => myNbFreeWires;
    public int NbFreeEdges() => myNbFreeEdges;
    public int NbSharedSolids() => myNbSharedSolids;
    public int NbSharedShells() => myNbSharedShells;
    public int NbSharedFaces() => myNbSharedFaces;
    public int NbSharedWires() => myNbSharedWires;
    public int NbSharedFreeWires() => myNbSharedFreeWires;
    public int NbSharedFreeEdges() => myNbSharedFreeEdges;
    public int NbSharedEdges() => myNbSharedEdges;
    public int NbSharedVertices() => myNbSharedVertices;
    public NCollection_HSequence<Shape> BigSplineSec() => myBigSplineSec;
    public NCollection_HSequence<Shape> IndirectSec() => myIndirectSec;
    public NCollection_HSequence<Shape> OffsetSurfaceSec() => myOffsetSurfaceSec;
    public NCollection_HSequence<Shape> Trimmed3dSec() => myTrimmed3dSec;
    public NCollection_HSequence<Shape> OffsetCurveSec() => myOffsetCurveSec;
    public NCollection_HSequence<Shape> Trimmed2dSec() => myTrimmed2dSec;
}
