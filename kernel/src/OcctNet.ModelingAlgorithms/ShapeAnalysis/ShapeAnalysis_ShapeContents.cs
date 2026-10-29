// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_ShapeContents.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_ShapeContents.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

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
    private readonly NCollection_HSequence<TopoDS_Shape> myBigSplineSec;
    private readonly NCollection_HSequence<TopoDS_Shape> myIndirectSec;
    private readonly NCollection_HSequence<TopoDS_Shape> myOffsetSurfaceSec;
    private readonly NCollection_HSequence<TopoDS_Shape> myTrimmed3dSec;
    private readonly NCollection_HSequence<TopoDS_Shape> myOffsetCurveSec;
    private readonly NCollection_HSequence<TopoDS_Shape> myTrimmed2dSec;

    /// <summary>Initializes the fields and clears the flags.</summary>
    public ShapeAnalysis_ShapeContents()
    {
        myBigSplineSec = new NCollection_HSequence<TopoDS_Shape>();
        myIndirectSec = new NCollection_HSequence<TopoDS_Shape>();
        myOffsetSurfaceSec = new NCollection_HSequence<TopoDS_Shape>();
        myTrimmed3dSec = new NCollection_HSequence<TopoDS_Shape>();
        myOffsetCurveSec = new NCollection_HSequence<TopoDS_Shape>();
        myTrimmed2dSec = new NCollection_HSequence<TopoDS_Shape>();
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
    public void Perform(in TopoDS_Shape Shape)
    {
        Clear();
        //  On y va
        TopExp_Explorer exp = new();
        var mapsh = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        //  On note pour les SOLIDES : ceux qui ont des trous (plus d un SHELL)

        for (exp.Init(Shape, TopAbs_SOLID); exp.More(); exp.Next())
        {
            TopoDS_Solid sol = TopoDS.Solid(exp.Current());
            sol.Location(new TopLoc_Location());
            mapsh.Add(sol);
            int nbs = 0;
            for (var shel = new TopExp_Explorer(sol, TopAbs_SHELL); shel.More(); shel.Next())
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
        for (exp.Init(Shape, TopAbs_SHELL); exp.More(); exp.Next())
        {
            myNbShells++;
            TopoDS_Shell she = TopoDS.Shell(exp.Current());
            she.Location(new TopLoc_Location());
            mapsh.Add(she);
            for (var shel = new TopExp_Explorer(she, TopAbs_FACE); shel.More(); shel.Next())
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
        for (exp.Init(Shape, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face face = TopoDS.Face(exp.Current());
            myNbFaces++;
            Geom_Surface? surf = BRep_Tool.Surface(face, out TopLoc_Location _);
            face.Location(new TopLoc_Location());
            mapsh.Add(face);
            if (surf is Geom_RectangularTrimmedSurface trsu)
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

            if (surf is Geom_BSplineSurface bsps)
            {
                myNbBSplibeSurf++;
                if (bsps.NbUPoles() * bsps.NbVPoles() > 8192)
                {
                    myNbBigSplines++;
                    if (myBigSplineMode)
                    {
                        myBigSplineSec.Append(face);
                    }
                }
            }
            if (surf is Geom_ElementarySurface els)
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
            if (surf!.IsKind(typeof(Geom_OffsetSurface)))
            {
                myNbOffsetSurf++;
                if (myOffsetSurfaceMode)
                {
                    myOffsetSurfaceSec.Append(face);
                }
            }
            else if (surf.IsKind(typeof(Geom_BezierSurface)))
            {
                myNbBezierSurf++;
            }

            int maxseam = 0, nbwires = 0;
            for (var wires = new TopExp_Explorer(face, TopAbs_WIRE); wires.More(); wires.Next())
            {
                TopoDS_Wire wire = TopoDS.Wire(wires.Current());
                int nbseam = 0;
                nbwires++;
                for (var edg = new TopExp_Explorer(wire, TopAbs_EDGE); edg.More(); edg.Next())
                {
                    TopoDS_Edge edge = TopoDS.Edge(edg.Current());
                    if (BRep_Tool.IsClosed(edge, face))
                    {
                        nbseam++;
                    }
                    Geom_Curve? c3d = BRep_Tool.Curve(edge, out double first, out double last);
                    if (c3d is not null)
                    {
                        if (c3d.IsKind(typeof(Geom_TrimmedCurve)))
                        {
                            myNbTrimmedCurve3d++;
                            if (myTrimmed3dMode)
                            {
                                myTrimmed3dSec.Append(face);
                            }
                        }
                    }
                    Geom2d_Curve? c2d = BRep_Tool.CurveOnSurface(edge, face, out first, out last);
                    if (c2d is null)
                    {
                        myNbNoPCurve++;
                    }
                    else if (c2d.IsKind(typeof(Geom2d_OffsetCurve)))
                    {
                        myNbOffsetCurves++;
                        if (myOffsetCurveMode)
                        {
                            myOffsetCurveSec.Append(face);
                        }
                    }
                    else if (c2d.IsKind(typeof(Geom2d_TrimmedCurve)))
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
        for (exp.Init(Shape, TopAbs_WIRE); exp.More(); exp.Next())
        {
            TopoDS_Wire wire = TopoDS.Wire(exp.Current());
            wire.Location(new TopLoc_Location());
            mapsh.Add(wire);
            myNbWires++;
        }
        myNbSharedWires = mapsh.Extent();

        //  Ne pas oublier les FACES :
        myNbFreeFaces = myNbFaces - nbfaceshell;

        mapsh.Clear();
        for (exp.Init(Shape, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge edge = TopoDS.Edge(exp.Current());
            edge.Location(new TopLoc_Location());
            mapsh.Add(edge);
            myNbEdges++;
            Geom_Curve? c3d = BRep_Tool.Curve(edge, out TopLoc_Location _, out double _, out double _);
            if (c3d is not null && c3d.IsKind(typeof(Geom_OffsetCurve)))
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
        for (exp.Init(Shape, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            TopoDS_Vertex vert = TopoDS.Vertex(exp.Current());
            vert.Location(new TopLoc_Location());
            myNbVertices++;
            mapsh.Add(vert);
        }
        myNbSharedVertices = mapsh.Extent();

        mapsh.Clear();
        for (exp.Init(Shape, TopAbs_EDGE, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Edge edge = TopoDS.Edge(exp.Current());
            edge.Location(new TopLoc_Location());
            myNbFreeEdges++;
            mapsh.Add(edge);
        }
        myNbSharedFreeEdges = mapsh.Extent();

        mapsh.Clear();
        for (exp.Init(Shape, TopAbs_WIRE, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Wire wire = TopoDS.Wire(exp.Current());
            wire.Location(new TopLoc_Location());
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
    public NCollection_HSequence<TopoDS_Shape> BigSplineSec() => myBigSplineSec;
    public NCollection_HSequence<TopoDS_Shape> IndirectSec() => myIndirectSec;
    public NCollection_HSequence<TopoDS_Shape> OffsetSurfaceSec() => myOffsetSurfaceSec;
    public NCollection_HSequence<TopoDS_Shape> Trimmed3dSec() => myTrimmed3dSec;
    public NCollection_HSequence<TopoDS_Shape> OffsetCurveSec() => myOffsetCurveSec;
    public NCollection_HSequence<TopoDS_Shape> Trimmed2dSec() => myTrimmed2dSec;
}
