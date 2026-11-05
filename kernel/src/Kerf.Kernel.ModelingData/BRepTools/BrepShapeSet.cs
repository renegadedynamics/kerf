// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_ShapeSet.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_ShapeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Globalization;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// A set of BRep shapes, written and read in OCCT's text format (a <c>.brep</c> file): the shape set of
/// <see cref="ShapeSet"/> with the geometry of vertices, edges and faces (curves, 2D curves and surfaces
/// through the GeomTools sets), their polygons and triangulations.
/// </summary>
/// <remarks>
/// <para>
/// Triangulations and polygons are written when the set is made with triangles (the default), and always for a
/// face that has a triangulation and no surface. Normals are written (format version 3) when the set is made with
/// normals, and always for such a face.
/// </para>
/// <para>
/// Reading builds the shapes with <c>BRep_Builder</c>, which freezes the geometry, triangulations and polygons it
/// attaches; the set freezes the shapes once they are all read (<see cref="ShapeSet"/>).
/// </para>
/// </remarks>
internal class BrepShapeSet : ShapeSet
{
    private readonly BrepBuilder _builder;
    private readonly SurfaceSet _surfaces = new();
    private readonly CurveSet _curves = new();
    private readonly Curve2dSet _curves2d = new();
    private readonly IndexedSet<object> _polygons2D = new(ReferenceEqualityComparer.Instance);
    private readonly IndexedSet<object> _polygons3D = new(ReferenceEqualityComparer.Instance);
    // with the flag that says whether to write the triangulation's normals
    private readonly IndexedMap<Triangulation, bool> _triangulations = new(ReferenceEqualityComparer.Instance);
    private readonly IndexedSet<object> _nodes = new(ReferenceEqualityComparer.Instance);
    private bool _withTriangles;
    private bool _withNormals;

    /// <summary>Builds an empty set, which writes triangulation data if asked, and normals if asked.</summary>
    public BrepShapeSet(bool withTriangles = true, bool withNormals = false)
    {
        _builder = new BrepBuilder();
        _withTriangles = withTriangles;
        _withNormals = withNormals;
    }

    /// <summary>Builds an empty set reading with the builder.</summary>
    internal BrepShapeSet(BrepBuilder builder, bool withTriangles = true, bool withNormals = false)
    {
        _builder = builder;
        _withTriangles = withTriangles;
        _withNormals = withNormals;
    }

    /// <summary>True if shapes are written with their triangulations and polygons.</summary>
    public bool IsWithTriangles => _withTriangles;

    /// <summary>True if triangulations are written with their normals.</summary>
    public bool IsWithNormals => _withNormals;

    /// <summary>
    /// Sets whether shapes are written with their triangulations and polygons (always, for a face that has a
    /// triangulation and no surface).
    /// </summary>
    public void SetWithTriangles(bool withTriangles)
    {
        _withTriangles = withTriangles;
    }

    /// <summary>
    /// Sets whether triangulations are written with their normals (always, for a face that has a triangulation and no
    /// surface).
    /// </summary>
    public void SetWithNormals(bool withNormals)
    {
        _withNormals = withNormals;
    }

    /// <summary>Clears the content of the set.</summary>
    public override void Clear()
    {
        _surfaces.Clear();
        _curves.Clear();
        _curves2d.Clear();
        _polygons3D.Clear();
        _polygons2D.Clear();
        _nodes.Clear();
        _triangulations.Clear();
        base.Clear();
    }

    private static void Out(TextWriter os, int value) => StreamUtilities.Write(os, value);

    private static void Out(TextWriter os, double value) => StreamUtilities.Write(os, value);

    // OS << std::setw(theWidth) << theValue
    private static void Setw(TextWriter os, int width, int value)
        => os.Write(StreamUtilities.Setw(width, value.ToString(CultureInfo.InvariantCulture)));

    private static void Setw(TextWriter os, int width, double value)
        => os.Write(StreamUtilities.Setw(width, StreamUtilities.FormatReal(value, StreamUtilities.Precision(os))));

    /// <summary>Stores the geometry of the shape: its curves, surfaces, locations, polygons and triangulations.</summary>
    public override void AddGeometry(in Shape s)
    {
        // Add the geometry

        if (s.Type == ShapeType.Vertex)
        {
            BrepTVertex tv = (BrepTVertex)s.TShape()!;
            NCollection_List<PointRepresentation>.Iterator itrp = new(tv.Points());

            while (itrp.More())
            {
                PointRepresentation pr = itrp.Value();

                if (pr.IsPointOnCurve())
                {
                    _curves.Add(pr.Curve());
                }

                else if (pr.IsPointOnCurveOnSurface())
                {
                    _curves2d.Add(pr.PCurve());
                    _surfaces.Add(pr.Surface());
                }

                else if (pr.IsPointOnSurface())
                {
                    _surfaces.Add(pr.Surface());
                }

                ChangeLocations().Add(pr.Location());
                itrp.Next();
            }
        }
        else if (s.Type == ShapeType.Edge)
        {
            // Add the curve geometry
            BrepTEdge te = (BrepTEdge)s.TShape()!;
            NCollection_List<CurveRepresentation>.Iterator itrc = new(te.Curves());

            while (itrc.More())
            {
                CurveRepresentation cr = itrc.Value();
                if (cr.IsCurve3D)
                {
                    if (cr.Curve3D() is not null)
                    {
                        _curves.Add(cr.Curve3D());
                        ChangeLocations().Add(cr.Location());
                    }
                }
                else if (cr.IsCurveOnSurface())
                {
                    _surfaces.Add(cr.Surface());
                    _curves2d.Add(cr.PCurve());
                    ChangeLocations().Add(cr.Location());
                    if (cr.IsCurveOnClosedSurface)
                    {
                        _curves2d.Add(cr.PCurve2());
                    }
                }
                else if (cr.IsRegularity())
                {
                    _surfaces.Add(cr.Surface());
                    ChangeLocations().Add(cr.Location());
                    _surfaces.Add(cr.Surface2());
                    ChangeLocations().Add(cr.Location2());
                }
                else if (_withTriangles)
                {
                    // for XML Persistence
                    if (cr.IsPolygon3D)
                    {
                        if (cr.Polygon3D() is not null)
                        {
                            _polygons3D.Add(cr.Polygon3D()!);
                            ChangeLocations().Add(cr.Location());
                        }
                    }
                    else if (cr.IsPolygonOnTriangulation())
                    {
                        // NCollection_IndexedDataMap::Add() function use is correct because
                        // Bin(Brep)Tools_ShapeSet::AddGeometry() is called from Bin(Brep)Tools_ShapeSet::Add()
                        // that processes shapes recursively from complex to elementary ones.
                        // As a result, the TopAbs_FACE's will be processed earlier than the TopAbs_EDGE's.
                        // Kerf: OCCT adds a null triangulation as a key, and crashes writing it
                        _triangulations.Add(NotNull(cr.Triangulation()), false); // edge triangulation does not need normals
                        _nodes.Add(NotNull(cr.PolygonOnTriangulation()));
                        ChangeLocations().Add(cr.Location());
                        if (cr.IsPolygonOnClosedTriangulation)
                        {
                            _nodes.Add(NotNull(cr.PolygonOnTriangulation2()));
                        }
                    }
                    else if (cr.IsPolygonOnSurface())
                    {
                        _surfaces.Add(cr.Surface());
                        // the 2D polygons aren't written; OCCT's map holds a null one as a key like any other
                        _polygons2D.Add((object?)cr.Polygon() ?? CurveSet.NullKey);
                        ChangeLocations().Add(cr.Location());
                        if (cr.IsPolygonOnClosedSurface)
                        {
                            _polygons2D.Add((object?)cr.Polygon2() ?? CurveSet.NullKey);
                        }
                    }
                }

                itrc.Next();
            }
        }

        else if (s.Type == ShapeType.Face)
        {
            // Add the surface geometry
            bool needNormals = _withNormals;
            BrepTFace tf = (BrepTFace)s.TShape()!;
            if (tf.Surface() is not null)
            {
                _surfaces.Add(tf.Surface());
            }
            else
            {
                needNormals = true;
            }

            if (_withTriangles || tf.Surface() is null)
            {
                // for XML Persistence
                Triangulation? tr = tf.Triangulation();
                if (tr is not null)
                {
                    _triangulations.Add(tr, needNormals);
                }
            }

            ChangeLocations().Add(tf.Location());
        }
    }

    // OCCT keeps null handles in its maps of polygons and triangulations, and crashes writing them; the port can't
    // write them either
    private static T NotNull<T>(T? theObject) where T : class
        => theObject ?? throw new NullObjectException("BRepTools_ShapeSet: a null polygon or triangulation");

    /// <summary>Dumps the geometry of the set in a readable form.</summary>
    public override void DumpGeometry(TextWriter os)
    {
        _curves2d.Dump(os);
        _curves.Dump(os);
        DumpPolygon3D(os);
        DumpPolygonOnTriangulation(os);
        _surfaces.Dump(os);
        DumpTriangulation(os);
    }

    /// <summary>
    /// Writes the geometry of the set: 2D curves, curves, 3D polygons, polygons on triangulations, surfaces and
    /// triangulations.
    /// </summary>
    public override void WriteGeometry(TextWriter os, ProgressRange progress = default)
    {
        // Make nested progress scope for processing geometry
        using var ps = new ProgressScope(progress, "Geometry", 100);

        _curves2d.Write(os, ps.Next(20));
        if (ps.UserBreak())
        {
            return;
        }

        _curves.Write(os, ps.Next(20));
        if (ps.UserBreak())
        {
            return;
        }

        WritePolygon3D(os, true, ps.Next(10));
        if (ps.UserBreak())
        {
            return;
        }

        WritePolygonOnTriangulation(os, true, ps.Next(10));
        if (ps.UserBreak())
        {
            return;
        }

        _surfaces.Write(os, ps.Next(20));
        if (ps.UserBreak())
        {
            return;
        }

        WriteTriangulation(os, true, ps.Next(20));
    }

    /// <summary>Reads the geometry of the set, written by <see cref="WriteGeometry(TextWriter, ProgressRange)"/>.</summary>
    public override void ReadGeometry(TextReader IS, ProgressRange progress = default)
    {
        // Make nested progress scope for processing geometry
        using var ps = new ProgressScope(progress, "Geometry", 100);

        _curves2d.Read(IS, ps.Next(20));
        if (ps.UserBreak())
        {
            return;
        }

        _curves.Read(IS, ps.Next(20));
        if (ps.UserBreak())
        {
            return;
        }

        ReadPolygon3D(IS, ps.Next(15));
        if (ps.UserBreak())
        {
            return;
        }

        ReadPolygonOnTriangulation(IS, ps.Next(15));
        if (ps.UserBreak())
        {
            return;
        }

        _surfaces.Read(IS, ps.Next(15));
        if (ps.UserBreak())
        {
            return;
        }

        ReadTriangulation(IS, ps.Next(15));
    }

    private static void PrintRegularity(Continuity c, TextWriter os)
    {
        switch (c)
        {
            case C0:
                os.Write("C0");
                break;

            case G1:
                os.Write("G1");
                break;

            case C1:
                os.Write("C1");
                break;

            case G2:
                os.Write("G2");
                break;

            case C2:
                os.Write("C2");
                break;

            case C3:
                os.Write("C3");
                break;

            case CN:
                os.Write("CN");
                break;
        }
    }

    /// <summary>Dumps the geometry of the shape in a readable form.</summary>
    public override void DumpGeometry(in Shape s, TextWriter os)
    {
        // Dump the geometry

        if (s.Type == ShapeType.Vertex)
        {
            // Dump the point geometry
            Vertex v = ShapeCasts.Vertex(s);
            os.Write("    Tolerance : ");
            Out(os, Tool.Tolerance(v));
            os.Write("\n");
            Point3D p = Tool.Pnt(v);
            os.Write("    - Point 3D : ");
            Out(os, p.X);
            os.Write(", ");
            Out(os, p.Y);
            os.Write(", ");
            Out(os, p.Z);
            os.Write("\n");

            BrepTVertex tv = (BrepTVertex)s.TShape()!;
            NCollection_List<PointRepresentation>.Iterator itrp = new(tv.Points());

            while (itrp.More())
            {
                PointRepresentation pr = itrp.Value();

                os.Write("    - Parameter : ");
                Out(os, pr.Parameter());
                if (pr.IsPointOnCurve())
                {
                    os.Write(" on curve ");
                    Out(os, _curves.Index(pr.Curve()));
                }

                else if (pr.IsPointOnCurveOnSurface())
                {
                    os.Write(" on pcurve ");
                    Out(os, _curves2d.Index(pr.PCurve()));
                    os.Write(" on surface ");
                    Out(os, _surfaces.Index(pr.Surface()));
                }

                else if (pr.IsPointOnSurface())
                {
                    os.Write(", ");
                    Out(os, pr.Parameter2());
                    os.Write(" on surface ");
                    Out(os, _surfaces.Index(pr.Surface()));
                }

                if (!pr.Location().IsIdentity)
                {
                    os.Write(" location ");
                    Out(os, Locations().Index(pr.Location()));
                }

                os.Write("\n");

                itrp.Next();
            }
        }

        else if (s.Type == ShapeType.Edge)
        {
            BrepTEdge te = (BrepTEdge)s.TShape()!;
            Point2D pf, pl;

            // Dump the curve geometry
            os.Write("    Tolerance : ");
            Out(os, te.Tolerance());
            os.Write("\n");
            if (te.SameParameter())
            {
                os.Write("     same parametrisation of curves\n");
            }

            if (te.SameRange())
            {
                os.Write("     same range on curves\n");
            }

            if (te.Degenerated())
            {
                os.Write("     degenerated\n");
            }

            double first, last;
            NCollection_List<CurveRepresentation>.Iterator itrc = new(te.Curves());
            while (itrc.More())
            {
                CurveRepresentation cr = itrc.Value();
                if (cr.IsCurve3D)
                {
                    GCurve gc = (GCurve)itrc.Value();
                    gc.Range(out first, out last);
                    if (cr.Curve3D() is not null)
                    {
                        os.Write("    - Curve 3D : ");
                        Out(os, _curves.Index(cr.Curve3D()));
                        if (!cr.Location().IsIdentity)
                        {
                            os.Write(" location ");
                            Out(os, Locations().Index(cr.Location()));
                        }

                        os.Write(", range : ");
                        Out(os, first);
                        os.Write(" ");
                        Out(os, last);
                        os.Write("\n");
                    }
                }
                else if (cr.IsCurveOnSurface())
                {
                    GCurve gc = (GCurve)itrc.Value();
                    gc.Range(out first, out last);
                    os.Write("    - PCurve : ");
                    Out(os, _curves2d.Index(cr.PCurve()));
                    if (cr.IsCurveOnClosedSurface)
                    {
                        os.Write(", ");
                        Out(os, _curves2d.Index(cr.PCurve2()));
                        os.Write(" (");
                        PrintRegularity(cr.Continuity(), os);
                        os.Write(")");
                    }

                    os.Write(" on surface ");
                    Out(os, _surfaces.Index(cr.Surface()));
                    if (!cr.Location().IsIdentity)
                    {
                        os.Write(" location ");
                        Out(os, Locations().Index(cr.Location()));
                    }

                    os.Write(", range : ");
                    Out(os, first);
                    os.Write(" ");
                    Out(os, last);
                    os.Write("\n");

                    CurveOnSurfaceRepresentation cos = (CurveOnSurfaceRepresentation)cr;
                    cos.UVPoints(out pf, out pl);
                    os.Write("  UV Points : ");
                    Out(os, pf.X);
                    os.Write(", ");
                    Out(os, pf.Y);
                    os.Write(" ");
                    Out(os, pl.X);
                    os.Write(", ");
                    Out(os, pl.Y);
                    os.Write("\n");
                    if (cr.IsCurveOnClosedSurface)
                    {
                        CurveOnClosedSurface cocs = (CurveOnClosedSurface)cr;
                        cocs.UVPoints2(out pf, out pl);
                        os.Write("  UV Points : ");
                        Out(os, pf.X);
                        os.Write(", ");
                        Out(os, pf.Y);
                        os.Write(" ");
                        Out(os, pl.X);
                        os.Write(", ");
                        Out(os, pl.Y);
                        os.Write("\n");
                    }
                }
                else if (cr.IsRegularity())
                {
                    os.Write("    - Regularity ");
                    PrintRegularity(cr.Continuity(), os);
                    os.Write("   on surfaces : ");
                    Out(os, _surfaces.Index(cr.Surface()));
                    if (!cr.Location().IsIdentity)
                    {
                        os.Write(" location ");
                        Out(os, Locations().Index(cr.Location()));
                    }

                    os.Write(", ");
                    Out(os, _surfaces.Index(cr.Surface2()));
                    if (!cr.Location2().IsIdentity)
                    {
                        os.Write(" location ");
                        Out(os, Locations().Index(cr.Location2()));
                    }

                    os.Write("\n");
                }
                else if (cr.IsPolygon3D)
                {
                    Polygon3DRepresentation gc = (Polygon3DRepresentation)itrc.Value();
                    if (gc.Polygon3D() is not null)
                    {
                        os.Write("    - Polygon 3D : ");
                        Out(os, _polygons3D.IndexOf(cr.Polygon3D()!) + 1);
                        if (!cr.Location().IsIdentity)
                        {
                            os.Write(" location ");
                            Out(os, Locations().Index(cr.Location()));
                        }
                    }
                }
                else if (cr.IsPolygonOnTriangulation())
                {
                    os.Write("    - PolygonOnTriangulation ");
                    Out(os, FindIndex(_nodes, cr.PolygonOnTriangulation()));
                    if (cr.IsPolygonOnClosedTriangulation)
                    {
                        os.Write(" ");
                        Out(os, FindIndex(_nodes, cr.PolygonOnTriangulation2()));
                    }

                    os.Write(" on triangulation ");
                    Out(os, cr.Triangulation() is { } tr ? (_triangulations.IndexOf(tr) + 1) : 0);
                    if (!cr.Location().IsIdentity)
                    {
                        os.Write(" location ");
                        Out(os, Locations().Index(cr.Location()));
                    }

                    os.Write("\n");
                }

                itrc.Next();
            }
        }

        else if (s.Type == ShapeType.Face)
        {
            Face f = ShapeCasts.Face(s);
            if (Tool.NaturalRestriction(f))
            {
                os.Write("NaturalRestriction\n");
            }

            // Dump the surface geometry
            BrepTFace tf = (BrepTFace)s.TShape()!;
            if (tf.Surface() is not null)
            {
                os.Write("    Tolerance : ");
                Out(os, tf.Tolerance());
                os.Write("\n");
                os.Write("    - Surface : ");
                Out(os, _surfaces.Index(tf.Surface()));
                if (!s.Placement.IsIdentity)
                {
                    os.Write(" location ");
                    Out(os, Locations().Index(s.Placement));
                }

                os.Write("\n");
            }

            if (tf.Triangulation() is not null)
            {
                // Dump the triangulation
                os.Write("    - Triangulation : ");
                Out(os, _triangulations.IndexOf(tf.Triangulation()!) + 1);
                if (!s.Placement.IsIdentity)
                {
                    os.Write(" location ");
                    Out(os, Locations().Index(tf.Location()));
                }

                os.Write("\n");
            }
        }

        os.Write("\n");
    }

    // FindIndex of a handle that may be null (OCCT's maps hold the null handle as a key like any other)
    private static int FindIndex(IndexedSet<object> map, object? key)
        => key is null ? 0 : (map.IndexOf(key) + 1);

    /// <summary>Writes the geometry of the shape, with the indices of its curves, surfaces, polygons and locations.</summary>
    public override void WriteGeometry(in Shape s, TextWriter os)
    {
        // Write the geometry

        if (s.Type == ShapeType.Vertex)
        {
            // Write the point geometry
            Vertex v = ShapeCasts.Vertex(s);
            Out(os, Tool.Tolerance(v));
            os.Write("\n");
            Point3D p = Tool.Pnt(v);
            Out(os, p.X);
            os.Write(" ");
            Out(os, p.Y);
            os.Write(" ");
            Out(os, p.Z);
            os.Write("\n");

            BrepTVertex tv = (BrepTVertex)s.TShape()!;
            NCollection_List<PointRepresentation>.Iterator itrp = new(tv.Points());

            while (itrp.More())
            {
                PointRepresentation pr = itrp.Value();

                Out(os, pr.Parameter());
                if (pr.IsPointOnCurve())
                {
                    os.Write(" 1 ");
                    Out(os, _curves.Index(pr.Curve()));
                }

                else if (pr.IsPointOnCurveOnSurface())
                {
                    os.Write(" 2 ");
                    Out(os, _curves2d.Index(pr.PCurve()));
                    os.Write(" ");
                    Out(os, _surfaces.Index(pr.Surface()));
                }

                else if (pr.IsPointOnSurface())
                {
                    os.Write(" 3 ");
                    Out(os, pr.Parameter2());
                    os.Write(" ");
                    Out(os, _surfaces.Index(pr.Surface()));
                }

                os.Write(" ");
                Out(os, Locations().Index(pr.Location()));
                os.Write("\n");

                itrp.Next();
            }

            os.Write("0 0\n"); // end representations
        }

        else if (s.Type == ShapeType.Edge)
        {
            // Write the curve geometry

            BrepTEdge te = (BrepTEdge)s.TShape()!;

            os.Write(" ");
            Out(os, te.Tolerance());
            os.Write(" ");
            os.Write(te.SameParameter() ? "1" : "0");
            os.Write(" ");
            os.Write(te.SameRange() ? "1" : "0");
            os.Write(" ");
            os.Write(te.Degenerated() ? "1" : "0");
            os.Write("\n");

            double first, last;
            NCollection_List<CurveRepresentation>.Iterator itrc = new(te.Curves());
            while (itrc.More())
            {
                CurveRepresentation cr = itrc.Value();
                if (cr.IsCurve3D)
                {
                    if (cr.Curve3D() is not null)
                    {
                        GCurve gc = (GCurve)itrc.Value();
                        gc.Range(out first, out last);
                        os.Write("1 "); // -1- Curve 3D
                        os.Write(" ");
                        Out(os, _curves.Index(cr.Curve3D()));
                        os.Write(" ");
                        Out(os, Locations().Index(cr.Location()));
                        os.Write(" ");
                        Out(os, first);
                        os.Write(" ");
                        Out(os, last);
                        os.Write("\n");
                    }
                }
                else if (cr.IsCurveOnSurface())
                {
                    GCurve gc = (GCurve)itrc.Value();
                    gc.Range(out first, out last);
                    if (!cr.IsCurveOnClosedSurface)
                    {
                        os.Write("2 "); // -2- Curve on surf
                    }
                    else
                    {
                        os.Write("3 "); // -3- Curve on closed surf
                    }

                    os.Write(" ");
                    Out(os, _curves2d.Index(cr.PCurve()));
                    if (cr.IsCurveOnClosedSurface)
                    {
                        os.Write(" ");
                        Out(os, _curves2d.Index(cr.PCurve2()));
                        PrintRegularity(cr.Continuity(), os);
                    }

                    os.Write(" ");
                    Out(os, _surfaces.Index(cr.Surface()));
                    os.Write(" ");
                    Out(os, Locations().Index(cr.Location()));
                    os.Write(" ");
                    Out(os, first);
                    os.Write(" ");
                    Out(os, last);
                    os.Write("\n");

                    // Write UV Points // for XML Persistence higher performance
                    if (FormatNb() == (int)FormatVersion.FormatVersion_VERSION_2)
                    {
                        Point2D pf, pl;
                        if (cr.IsCurveOnClosedSurface)
                        {
                            CurveOnClosedSurface cocs = (CurveOnClosedSurface)cr;
                            cocs.UVPoints2(out pf, out pl);
                        }
                        else
                        {
                            CurveOnSurfaceRepresentation cos = (CurveOnSurfaceRepresentation)cr;
                            cos.UVPoints(out pf, out pl);
                        }

                        Out(os, pf.X);
                        os.Write(" ");
                        Out(os, pf.Y);
                        os.Write(" ");
                        Out(os, pl.X);
                        os.Write(" ");
                        Out(os, pl.Y);
                        os.Write("\n");
                    }
                }
                else if (cr.IsRegularity())
                {
                    os.Write("4 "); // -4- Regularity
                    PrintRegularity(cr.Continuity(), os);
                    os.Write(" ");
                    Out(os, _surfaces.Index(cr.Surface()));
                    os.Write(" ");
                    Out(os, Locations().Index(cr.Location()));
                    os.Write(" ");
                    Out(os, _surfaces.Index(cr.Surface2()));
                    os.Write(" ");
                    Out(os, Locations().Index(cr.Location2()));
                    os.Write("\n");
                }

                else if (_withTriangles)
                {
                    // for XML Persistence
                    if (cr.IsPolygon3D)
                    {
                        Polygon3DRepresentation gc = (Polygon3DRepresentation)itrc.Value();
                        if (gc.Polygon3D() is not null)
                        {
                            os.Write("5 "); // -5- Polygon3D
                            os.Write(" ");
                            Out(os, _polygons3D.IndexOf(cr.Polygon3D()!) + 1);
                            os.Write(" ");
                            Out(os, Locations().Index(cr.Location()));
                            os.Write("\n");
                        }
                    }
                    else if (cr.IsPolygonOnTriangulation())
                    {
                        PolygonOnTriangulationRepresentation pt = (PolygonOnTriangulationRepresentation)itrc.Value();
                        if (!cr.IsPolygonOnClosedTriangulation)
                        {
                            os.Write("6 "); // -6- Polygon on triangulation
                        }
                        else
                        {
                            os.Write("7 "); // -7- Polygon on closed triangulation
                        }

                        os.Write(" ");
                        Out(os, FindIndex(_nodes, pt.PolygonOnTriangulation()));
                        if (cr.IsPolygonOnClosedTriangulation)
                        {
                            os.Write(" ");
                            Out(os, FindIndex(_nodes, pt.PolygonOnTriangulation2()));
                        }

                        os.Write(" ");
                        Out(os, pt.Triangulation() is { } tr ? (_triangulations.IndexOf(tr) + 1) : 0);
                        os.Write(" ");
                        Out(os, Locations().Index(cr.Location()));
                        os.Write("\n");
                    }
                }

                itrc.Next();
            }

            os.Write("0\n"); // end of the list of representations
        }

        else if (s.Type == ShapeType.Face)
        {
            BrepTFace tf = (BrepTFace)s.TShape()!;
            Face f = ShapeCasts.Face(s);

            if (tf.Surface() is not null)
            {
                os.Write(Tool.NaturalRestriction(f) ? "1" : "0");
                os.Write(" ");
                // Write the surface geometry
                os.Write(" ");
                Out(os, tf.Tolerance());
                os.Write(" ");
                Out(os, _surfaces.Index(tf.Surface()));
                os.Write(" ");
                Out(os, Locations().Index(tf.Location()));
                os.Write("\n");
            }
            else // For correct reading of null face
            {
                Out(os, 0);
                os.Write(" ");
                os.Write(" ");
                Out(os, tf.Tolerance());
                os.Write(" ");
                Out(os, 0);
                os.Write(" ");
                Out(os, 0);
                os.Write("\n");
            }

            if (_withTriangles || tf.Surface() is null)
            {
                // for XML Persistence
                if (tf.Triangulation() is not null)
                {
                    Out(os, 2);
                    os.Write(" ");
                    // Write the triangulation
                    os.Write(" ");
                    Out(os, _triangulations.IndexOf(tf.Triangulation()!) + 1);
                }
            }
        }
    }

    private static Continuity ReadRegularity(TextReader IS)
    {
        char[] buffer = new char[255];
        StreamUtilities.ReadWord(IS, buffer);
        switch (buffer[0])
        {
            case 'C':
                switch (buffer[1])
                {
                    case '0':
                        return C0;

                    case '1':
                        return C1;

                    case '2':
                        return C2;

                    case '3':
                        return C3;

                    case 'N':
                        return CN;
                }

                break;

            case 'G':
                switch (buffer[1])
                {
                    case '1':
                        return G1;

                    case '2':
                        return G2;
                }

                break;
        }

        return C0;
    }

    /// <summary>Reads the geometry of a shape of type T and creates the shape in S, unfrozen.</summary>
    /// <exception cref="StreamFormatErrorException">The input can't be read safely.</exception>
    internal override void ReadGeometry(ShapeType T, TextReader IS, ref Shape S)
    {
        // Read the geometry

        // OCCT leaves most of them uninitialised
        int val = 0, c = 0, pc = 0, pc2 = 0, s = 0, s2 = 0, l = 0, l2 = 0, t = 0, pt = 0, pt2 = 0;
        double tol, x, y, z, first, last, p1, p2 = 0;
        double pfX, pfY, plX, plY;
        Point2D pf = new(), pl = new();
        bool closed;
        Continuity reg = C0;
        switch (T)
        {
            //---------
            // vertex
            //---------

            case ShapeType.Vertex:
            {
                Vertex v = new();

                // Read the point geometry
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out tol);
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out z);
                _builder.MakeVertex(ref v, new Point3D(x, y, z), tol);
                S = v;
                BrepTVertex tv = (BrepTVertex)v.TShape()!;

                NCollection_List<PointRepresentation> lpr = tv.ChangePoints();
                Placement L = new();

                do
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out p1);
                    StreamUtilities.Read(IS, ref val);
                    // Kerf: untrusted input: OCCT loops for ever on a stream that has failed
                    StreamUtilities.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadGeometry");

                    PointRepresentation? pr = null;
                    switch (val)
                    {
                        case 1:
                        {
                            StreamUtilities.Read(IS, ref c);

                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:09 2002 Begin
                            if (_curves.Curve(c) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:13 2002 End

                            PointOnCurve poc = new(p1, _curves.Curve(c), L);
                            pr = poc;
                        }
                            break;

                        case 2:
                        {
                            StreamUtilities.Read(IS, ref pc);
                            StreamUtilities.Read(IS, ref s);

                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:09 2002 Begin
                            if (_curves2d.Curve2d(pc) is null || _surfaces.Surface(s) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:13 2002 End

                            PointOnCurveOnSurface poc = new(p1, _curves2d.Curve2d(pc), _surfaces.Surface(s), L);
                            pr = poc;
                        }
                            break;

                        case 3:
                        {
                            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out p2);
                            StreamUtilities.Read(IS, ref s);

                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:09 2002 Begin
                            if (_surfaces.Surface(s) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:13 2002 End

                            PointOnSurface poc = new(p1, p2, _surfaces.Surface(s), L);
                            pr = poc;
                        }
                            break;
                    }

                    if (val > 0)
                    {
                        StreamUtilities.Read(IS, ref l);
                        if (pr is not null)
                        {
                            pr.Location(Locations().Location(l));
                            lpr.Append(pr);
                        }
                    }
                } while (val > 0);
            }
                break;

            //---------
            // edge
            //---------

            case ShapeType.Edge:

                // Create an edge
            {
                Edge e = new();

                _builder.MakeEdge(ref e);
                S = e;

                // Read the curve geometry
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out tol);
                StreamUtilities.Read(IS, ref val);
                _builder.SameParameter(e, (val == 1));
                StreamUtilities.Read(IS, ref val);
                _builder.SameRange(e, (val == 1));
                StreamUtilities.Read(IS, ref val);
                _builder.Degenerated(e, (val == 1));

                do
                {
                    StreamUtilities.Read(IS, ref val);
                    // Kerf: untrusted input: OCCT loops for ever on a stream that has failed
                    StreamUtilities.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadGeometry");
                    switch (val)
                    {
                        case 1: // -1- Curve 3D
                            StreamUtilities.Read(IS, ref c);
                            StreamUtilities.Read(IS, ref l);
                            if (_curves.Curve(c) is not null)
                            {
                                _builder.UpdateEdge(e, _curves.Curve(c), Locations().Location(l), tol);
                            }

                            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out first);
                            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out last);
                            if (_curves.Curve(c) is not null)
                            {
                                bool only3d = true;
                                _builder.Range(e, first, last, only3d);
                            }

                            break;

                        case 2: // -2- Curve on surf
                        case 3: // -3- Curve on closed surf
                            closed = (val == 3);
                            StreamUtilities.Read(IS, ref pc);
                            if (closed)
                            {
                                StreamUtilities.Read(IS, ref pc2);
                                reg = ReadRegularity(IS);
                            }

                            // surface, location
                            StreamUtilities.Read(IS, ref s);
                            StreamUtilities.Read(IS, ref l);

                            // range
                            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out first);
                            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out last);

                            // read UV Points // for XML Persistence higher performance
                            if (FormatNb() == (int)FormatVersion.FormatVersion_VERSION_2)
                            {
                                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out pfX);
                                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out pfY);
                                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out plX);
                                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out plY);
                                pf = new Point2D(pfX, pfY);
                                pl = new Point2D(plX, plY);
                            }

                            //  Modified by Sergey KHROMOV - Wed Apr 24 12:11:16 2002 Begin
                            if (_curves2d.Curve2d(pc) is null || (closed && _curves2d.Curve2d(pc2) is null)
                                || _surfaces.Surface(s) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 12:11:17 2002 End

                            if (closed)
                            {
                                if (FormatNb() == (int)FormatVersion.FormatVersion_VERSION_2)
                                {
                                    _builder.UpdateEdge(e,
                                                         _curves2d.Curve2d(pc),
                                                         _curves2d.Curve2d(pc2),
                                                         _surfaces.Surface(s),
                                                         Locations().Location(l),
                                                         tol,
                                                         pf,
                                                         pl);
                                }
                                else
                                {
                                    _builder.UpdateEdge(e,
                                                         _curves2d.Curve2d(pc),
                                                         _curves2d.Curve2d(pc2),
                                                         _surfaces.Surface(s),
                                                         Locations().Location(l),
                                                         tol);
                                }

                                _builder.Continuity(e,
                                                     _surfaces.Surface(s),
                                                     _surfaces.Surface(s),
                                                     Locations().Location(l),
                                                     Locations().Location(l),
                                                     reg);
                            }
                            else
                            {
                                if (FormatNb() == (int)FormatVersion.FormatVersion_VERSION_2)
                                {
                                    _builder.UpdateEdge(e,
                                                         _curves2d.Curve2d(pc),
                                                         _surfaces.Surface(s),
                                                         Locations().Location(l),
                                                         tol,
                                                         pf,
                                                         pl);
                                }
                                else
                                {
                                    _builder.UpdateEdge(e,
                                                         _curves2d.Curve2d(pc),
                                                         _surfaces.Surface(s),
                                                         Locations().Location(l),
                                                         tol);
                                }
                            }

                            _builder.Range(e, _surfaces.Surface(s), Locations().Location(l), first, last);
                            break;

                        case 4: // -4- Regularity
                            reg = ReadRegularity(IS);
                            StreamUtilities.Read(IS, ref s);
                            StreamUtilities.Read(IS, ref l);
                            StreamUtilities.Read(IS, ref s2);
                            StreamUtilities.Read(IS, ref l2);
                            //  Modified by Sergey KHROMOV - Wed Apr 24 12:39:13 2002 Begin
                            if (_surfaces.Surface(s) is null || _surfaces.Surface(s2) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 12:39:14 2002 End
                            _builder.Continuity(e,
                                                 _surfaces.Surface(s),
                                                 _surfaces.Surface(s2),
                                                 Locations().Location(l),
                                                 Locations().Location(l2),
                                                 reg);
                            break;

                        case 5: // -5- Polygon3D
                            StreamUtilities.Read(IS, ref c);
                            StreamUtilities.Read(IS, ref l);
                            // szy-02.05.2004
                            // myBuilder.UpdateEdge(E,occ::down_cast<Poly_Polygon3D>(myPolygons3D(c)));
                            if (c > 0 && c <= _polygons3D.Count)
                            {
                                _builder.UpdateEdge(e, _polygons3D[c - 1] as Polygon3D, Locations().Location(l));
                            }

                            break;

                        case 6:
                        case 7:
                            closed = (val == 7);
                            StreamUtilities.Read(IS, ref pt);
                            if (closed)
                            {
                                StreamUtilities.Read(IS, ref pt2);
                            }

                            StreamUtilities.Read(IS, ref t);
                            StreamUtilities.Read(IS, ref l);
                            if (closed)
                            {
                                if (t > 0 && t <= _triangulations.Count && pt > 0 && pt <= _nodes.Count
                                    && pt2 > 0 && pt2 <= _nodes.Count)
                                {
                                    _builder.UpdateEdge(e,
                                                         _nodes[pt - 1] as PolygonOnTriangulation,
                                                         _nodes[pt2 - 1] as PolygonOnTriangulation,
                                                         _triangulations.KeyAt(t - 1),
                                                         Locations().Location(l));
                                }
                            }
                            else
                            {
                                if (t > 0 && t <= _triangulations.Count && pt > 0 && pt <= _nodes.Count)
                                {
                                    _builder.UpdateEdge(e,
                                                         _nodes[pt - 1] as PolygonOnTriangulation,
                                                         _triangulations.KeyAt(t - 1),
                                                         Locations().Location(l));
                                }
                            }
                            // range

                            break;
                    }
                } while (val > 0);
            }
                break;

            //---------
            // wire
            //---------

            case ShapeType.Wire:
            {
                Wire w = new();
                _builder.MakeWire(ref w);
                S = w;
            }
                break;

            //---------
            // face
            //---------

            case ShapeType.Face:
            {
                // create a face :
                Face f = new();
                //    std::streampos pos;
                _builder.MakeFace(ref f);
                S = f;

                StreamUtilities.Read(IS, ref val); // natural restriction
                if (val == 0 || val == 1)
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out tol);
                    StreamUtilities.Read(IS, ref s);
                    StreamUtilities.Read(IS, ref l);
                    //  Modified by Sergey KHROMOV - Wed Apr 24 12:39:13 2002 Begin
                    if (_surfaces.Surface(s) is not null)
                    {
                        //  Modified by Sergey KHROMOV - Wed Apr 24 12:39:14 2002 End
                        _builder.UpdateFace(f, _surfaces.Surface(s), Locations().Location(l), tol);
                        _builder.NaturalRestriction(f, (val == 1));
                    }
                    //      pos = IS.tellg();
                    //      IS >> val;
                }
                else if (val == 2)
                {
                    // only triangulation
                    StreamUtilities.Read(IS, ref s);
                    // Kerf: untrusted input: OCCT reads outside its map
                    if (s < 1 || s > _triangulations.Count)
                    {
                        throw new StreamFormatErrorException("BRepTools_ShapeSet::ReadGeometry: no triangulation "
                                                            + s.ToString(CultureInfo.InvariantCulture));
                    }

                    _builder.UpdateFace(f, _triangulations.KeyAt(s - 1));
                }
                //    else pos = IS.tellg();

                // BUC60769

                if (val == 2)
                {
                    break;
                }

                char[] @string = new char[260];
                StreamUtilities.GetLine(IS, @string, 256, '\n');
                StreamUtilities.GetLine(IS, @string, 256, '\n');

                if (@string[0] == '2')
                {
                    // cas triangulation
                    s = StreamUtilities.Atoi(@string.AsSpan(2));
                    if (s > 0 && s <= _triangulations.Count)
                    {
                        _builder.UpdateFace(f, _triangulations.KeyAt(s - 1));
                    }
                }
                //    else IS.seekg(pos);
            }
                break;

            //---------
            // shell
            //---------

            case ShapeType.Shell:
            {
                Shell sh = new();
                _builder.MakeShell(ref sh);
                S = sh;
            }
                break;

            //---------
            // solid
            //---------

            case ShapeType.Solid:
            {
                Solid so = new();
                _builder.MakeSolid(ref so);
                S = so;
            }
                break;

            //---------
            // compsolid
            //---------

            case ShapeType.CompSolid:
            {
                CompSolid cs = new();
                _builder.MakeCompSolid(ref cs);
                S = cs;
            }
                break;

            //---------
            // compound
            //---------

            case ShapeType.Compound:
            {
                Compound co = new();
                _builder.MakeCompound(ref co);
                S = co;
            }
                break;

            default:
                break;
        }
    }

    /// <summary>Adds S2 to S1, with the set's builder.</summary>
    internal override void AddShapes(ref Shape s1, in Shape s2)
    {
        _builder.Add(s1, s2);
    }

    /// <summary>Completes a face read from a version 1 file: <c>BRepTools.Update</c> computes its UV points.</summary>
    internal override void Check(ShapeType t, ref Shape s)
    {
        if (t == ShapeType.Face)
        {
            Face f = ShapeCasts.Face(s);
            BrepTools.Update(f);
        }
    }

    /// <summary>
    /// Writes the polygons on triangulations: compact, in the format <see cref="ReadPolygonOnTriangulation"/> reads,
    /// or in a readable form.
    /// </summary>
    public void WritePolygonOnTriangulation(TextWriter os, bool compact = true, ProgressRange progress = default)
    {
        int i, j, nbpOntri = _nodes.Count;

        using var ps = new ProgressScope(progress, "Polygons On Triangulation", nbpOntri);
        if (compact)
        {
            os.Write("PolygonOnTriangulations ");
            Out(os, nbpOntri);
            os.Write("\n");
        }
        else
        {
            os.Write(" -------\n");
            os.Write("Dump of ");
            Out(os, nbpOntri);
            os.Write(" PolygonOnTriangulations\n");
            os.Write(" -------\n");
        }

        PolygonOnTriangulation poly;
        NCollection_HArray1<double>? param;
        for (i = 1; i <= nbpOntri && ps.More(); i++, ps.Next())
        {
            poly = (PolygonOnTriangulation)_nodes[i - 1];
            NCollection_Array1<int> nodes = poly.Nodes();
            if (!compact)
            {
                os.Write("  ");
                Out(os, i);
                os.Write(" : PolygonOnTriangulation with ");
                Out(os, nodes.Length());
                os.Write(" Nodes\n");
            }
            else
            {
                Out(os, nodes.Length());
                os.Write(" ");
            }

            if (!compact)
            {
                os.Write("  ");
            }

            for (j = 1; j <= nodes.Length(); j++)
            {
                Out(os, nodes.Value(j));
                os.Write(" ");
            }

            os.Write("\n");

            // writing parameters:
            param = poly.Parameters();
            if (compact)
            {
                os.Write("p ");
            }

            // write the deflection
            if (!compact)
            {
                os.Write("  Deflection : ");
            }

            Out(os, poly.Deflection());
            os.Write(" ");
            if (!compact)
            {
                os.Write("\n");
            }

            if (param is not null)
            {
                if (!compact)
                {
                    os.Write("  Parameters :");
                }
                else
                {
                    os.Write("1 ");
                }

                if (!compact)
                {
                    os.Write("  ");
                }

                for (j = 1; j <= param.Length(); j++)
                {
                    Out(os, param.Value(j));
                    os.Write(" ");
                }

                os.Write("\n");
            }
            else
            {
                os.Write("0 \n");
            }
        }
    }

    /// <summary>Dumps the polygons on triangulations in a readable form.</summary>
    public void DumpPolygonOnTriangulation(TextWriter os)
    {
        WritePolygonOnTriangulation(os, false);
    }

    /// <summary>Reads the polygons on triangulations, written by <see cref="WritePolygonOnTriangulation"/>.</summary>
    /// <exception cref="StreamFormatErrorException">The input can't be read safely.</exception>
    public void ReadPolygonOnTriangulation(TextReader IS, ProgressRange progress = default)
    {
        char[] buffer = new char[255];
        StreamUtilities.ReadWord(IS, buffer);
        if (!StreamUtilities.CString(buffer).Contains("PolygonOnTriangulations", StringComparison.Ordinal))
        {
            return;
        }

        int i, j, val = 0, nbpol = 0, nbnodes = 0;
        int hasparameters = 0;
        double par;
        PolygonOnTriangulation poly;
        StreamUtilities.Read(IS, ref nbpol);
        // Kerf: untrusted input: a polygon takes at least 9 characters ("0 \np 0 0 \n")
        StreamUtilities.CheckCount(IS, Math.Max(nbpol, 0), 8, "polygons on triangulations");
        // OCC19559
        using var ps = new ProgressScope(progress, "Polygons On Triangulation", nbpol);
        for (i = 1; i <= nbpol && ps.More(); i++, ps.Next())
        {
            StreamUtilities.Read(IS, ref nbnodes);
            // Kerf: untrusted input: OCCT allocates for any count, and again for each polygon on a failed stream
            StreamUtilities.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadPolygonOnTriangulation");
            StreamUtilities.CheckCount(IS, nbnodes, 2, "polygon nodes");
            NCollection_Array1<int> nodes = new(1, nbnodes);
            for (j = 1; j <= nbnodes; j++)
            {
                StreamUtilities.Read(IS, ref val);
                nodes[j] = val;
            }

            StreamUtilities.ReadWord(IS, buffer);
            //      if (!strcasecmp(buffer, "p")) {
            double def;
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out def);
            StreamUtilities.Read(IS, ref hasparameters);
            if (hasparameters != 0)
            {
                NCollection_Array1<double> param1 = new(1, nbnodes);
                for (j = 1; j <= nbnodes; j++)
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out par);
                    param1[j] = par;
                }

                poly = new PolygonOnTriangulation(nodes, param1);
            }
            else
            {
                poly = new PolygonOnTriangulation(nodes);
            }

            poly.Deflection(def);
            //      }
            //      else {
            //      IS.seekg(ppp);
            //      Poly = new Poly_PolygonOnTriangulation(Nodes);
            //      }
            _nodes.Add(poly);
        }
        //  }
        //  else IS.seekg(pos);
    }

    /// <summary>Writes the 3D polygons: compact, in the format <see cref="ReadPolygon3D"/> reads, or in a readable form.</summary>
    public void WritePolygon3D(TextWriter os, bool compact = true, ProgressRange progress = default)
    {
        int i, j, nbpol = _polygons3D.Count;

        using var ps = new ProgressScope(progress, "3D Polygons", nbpol);

        if (compact)
        {
            os.Write("Polygon3D ");
            Out(os, nbpol);
            os.Write("\n");
        }
        else
        {
            os.Write(" -------\n");
            os.Write("Dump of ");
            Out(os, nbpol);
            os.Write(" Polygon3Ds\n");
            os.Write(" -------\n");
        }

        Polygon3D p;
        for (i = 1; i <= nbpol && ps.More(); i++, ps.Next())
        {
            p = (Polygon3D)_polygons3D[i - 1];
            if (compact)
            {
                Out(os, p.NodeCount);
                os.Write(" ");
                os.Write(p.HasParameters ? "1" : "0");
                os.Write("\n");
            }
            else
            {
                os.Write("  ");
                Out(os, i);
                os.Write(" : Polygon3D with ");
                Out(os, p.NodeCount);
                os.Write(" Nodes\n");
                os.Write(p.HasParameters ? "with" : "without");
                os.Write(" parameters\n");
            }

            // write the deflection
            if (!compact)
            {
                os.Write("Deflection : ");
            }

            Out(os, p.Deflection());
            os.Write("\n");

            // write the nodes
            if (!compact)
            {
                os.Write("\nNodes :\n");
            }

            int i1, nbNodes = p.NodeCount;
            NCollection_Array1<Point3D> nodes = p.Nodes();
            for (j = 1; j <= nbNodes; j++)
            {
                if (!compact)
                {
                    Setw(os, 10, j);
                    os.Write(" : ");
                }

                if (!compact)
                {
                    Setw(os, 17, nodes[j].X);
                }
                else
                {
                    Out(os, nodes[j].X);
                }

                os.Write(" ");
                if (!compact)
                {
                    Setw(os, 17, nodes[j].Y);
                }
                else
                {
                    Out(os, nodes[j].Y);
                }

                os.Write(" ");
                if (!compact)
                {
                    Setw(os, 17, nodes[j].Z);
                }
                else
                {
                    Out(os, nodes[j].Z);
                }

                if (!compact)
                {
                    os.Write("\n");
                }
                else
                {
                    os.Write(" ");
                }
            }

            os.Write("\n");

            if (p.HasParameters)
            {
                if (!compact)
                {
                    os.Write("\nParameters :\n");
                }

                NCollection_Array1<double> param = p.Parameters();
                for (i1 = 1; i1 <= nbNodes; i1++)
                {
                    Out(os, param[i1]);
                    os.Write(" ");
                }

                os.Write("\n");
            }
        }
    }

    /// <summary>Dumps the 3D polygons in a readable form.</summary>
    public void DumpPolygon3D(TextWriter os)
    {
        WritePolygon3D(os, false);
    }

    /// <summary>Reads the 3D polygons, written by <see cref="WritePolygon3D"/>.</summary>
    /// <exception cref="StreamFormatErrorException">The input can't be read safely.</exception>
    public void ReadPolygon3D(TextReader IS, ProgressRange progress = default)
    {
        char[] buffer = new char[255];
        //  int i, j, p, val, nbpol, nbnodes, hasparameters;
        int i, j, p, nbpol = 0, nbnodes = 0, hasparameters = 0;
        double d, x, y, z;

        StreamUtilities.ReadWord(IS, buffer);
        if (!StreamUtilities.CString(buffer).Contains("Polygon3D", StringComparison.Ordinal))
        {
            return;
        }

        Polygon3D P;
        StreamUtilities.Read(IS, ref nbpol);
        // Kerf: untrusted input: a polygon takes at least 6 characters ("0 0\n0\n\n")
        StreamUtilities.CheckCount(IS, Math.Max(nbpol, 0), 6, "3D polygons");
        // OCC19559
        using var ps = new ProgressScope(progress, "3D Polygons", nbpol);
        for (i = 1; i <= nbpol && ps.More(); i++, ps.Next())
        {
            StreamUtilities.Read(IS, ref nbnodes);
            StreamUtilities.Read(IS, ref hasparameters);
            // Kerf: untrusted input: OCCT allocates for any count, and again for each polygon on a failed stream
            StreamUtilities.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadPolygon3D");
            StreamUtilities.CheckCount(IS, nbnodes, 6, "polygon nodes");
            NCollection_Array1<Point3D> nodes = new(1, nbnodes);
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out d);
            for (j = 1; j <= nbnodes; j++)
            {
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out z);
                nodes[j] = nodes[j].WithCoord(x, y, z);
            }

            if (hasparameters != 0)
            {
                NCollection_Array1<double> param = new(1, nbnodes);
                for (p = 1; p <= nbnodes; p++)
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out param[p]);
                }

                P = new Polygon3D(nodes, param);
            }
            else
            {
                P = new Polygon3D(nodes);
            }

            P.Deflection(d);
            _polygons3D.Add(P);
        }
    }

    /// <summary>
    /// Writes the triangulations: compact, in the format <see cref="ReadTriangulation"/> reads, or in a readable form.
    /// </summary>
    public void WriteTriangulation(TextWriter os, bool compact = true, ProgressRange progress = default)
    {
        int i, j, nbNodes, nbtri = _triangulations.Count;
        int nbTriangles = 0, n1, n2, n3;

        using var ps = new ProgressScope(progress, "Triangulations", nbtri);

        if (compact)
        {
            os.Write("Triangulations ");
            Out(os, nbtri);
            os.Write("\n");
        }
        else
        {
            os.Write(" -------\n");
            os.Write("Dump of ");
            Out(os, nbtri);
            os.Write(" Triangulations\n");
            os.Write(" -------\n");
        }

        Triangulation t;
        for (i = 1; i <= nbtri && ps.More(); i++, ps.Next())
        {
            t = _triangulations.KeyAt(i - 1);
            bool toWriteNormals = _triangulations.ValueAt(i - 1);
            if (compact)
            {
                Out(os, t.NodeCount);
                os.Write(" ");
                Out(os, t.TriangleCount);
                os.Write(" ");
                os.Write(t.HasUVNodes ? "1" : "0");
                os.Write(" ");
                if (FormatNb() >= (int)FormatVersion.FormatVersion_VERSION_3)
                {
                    os.Write(t.HasNormals && toWriteNormals ? "1" : "0");
                    os.Write(" ");
                }
            }
            else
            {
                os.Write("  ");
                Out(os, i);
                os.Write(" : Triangulation with ");
                Out(os, t.NodeCount);
                os.Write(" Nodes and ");
                Out(os, t.TriangleCount);
                os.Write(" Triangles\n");
                os.Write("      ");
                os.Write(t.HasUVNodes ? "with" : "without");
                os.Write(" UV nodes\n");
                if (FormatNb() >= (int)FormatVersion.FormatVersion_VERSION_3)
                {
                    os.Write("      ");
                    os.Write(t.HasNormals && toWriteNormals ? "with" : "without");
                    os.Write(" normals\n");
                }
            }

            // write the deflection

            if (!compact)
            {
                os.Write("  Deflection : ");
            }

            Out(os, t.Deflection());
            os.Write("\n");

            // write the 3d nodes

            if (!compact)
            {
                os.Write("\n3D Nodes :\n");
            }

            nbNodes = t.NodeCount;
            for (j = 1; j <= nbNodes; j++)
            {
                Point3D node = t.Node(j);
                if (!compact)
                {
                    Setw(os, 10, j);
                    os.Write(" : ");
                }

                if (!compact)
                {
                    Setw(os, 17, node.X);
                }
                else
                {
                    Out(os, node.X);
                }

                os.Write(" ");
                if (!compact)
                {
                    Setw(os, 17, node.Y);
                }
                else
                {
                    Out(os, node.Y);
                }

                os.Write(" ");
                if (!compact)
                {
                    Setw(os, 17, node.Z);
                }
                else
                {
                    Out(os, node.Z);
                }

                if (!compact)
                {
                    os.Write("\n");
                }
                else
                {
                    os.Write(" ");
                }
            }

            if (t.HasUVNodes)
            {
                if (!compact)
                {
                    os.Write("\nUV Nodes :\n");
                }

                for (j = 1; j <= nbNodes; j++)
                {
                    Point2D node2d = t.UVNode(j);
                    if (!compact)
                    {
                        Setw(os, 10, j);
                        os.Write(" : ");
                    }

                    if (!compact)
                    {
                        Setw(os, 17, node2d.X);
                    }
                    else
                    {
                        Out(os, node2d.X);
                    }

                    os.Write(" ");
                    if (!compact)
                    {
                        Setw(os, 17, node2d.Y);
                    }
                    else
                    {
                        Out(os, node2d.Y);
                    }

                    if (!compact)
                    {
                        os.Write("\n");
                    }
                    else
                    {
                        os.Write(" ");
                    }
                }
            }

            if (!compact)
            {
                os.Write("\nTriangles :\n");
            }

            nbTriangles = t.TriangleCount;
            for (j = 1; j <= nbTriangles; j++)
            {
                if (!compact)
                {
                    Setw(os, 10, j);
                    os.Write(" : ");
                }

                t.Triangle(j).Get(out n1, out n2, out n3);
                if (!compact)
                {
                    Setw(os, 10, n1);
                }
                else
                {
                    Out(os, n1);
                }

                os.Write(" ");
                if (!compact)
                {
                    Setw(os, 10, n2);
                }
                else
                {
                    Out(os, n2);
                }

                os.Write(" ");
                if (!compact)
                {
                    Setw(os, 10, n3);
                }
                else
                {
                    Out(os, n3);
                }

                if (!compact)
                {
                    os.Write("\n");
                }
                else
                {
                    os.Write(" ");
                }
            }

            if (FormatNb() >= (int)FormatVersion.FormatVersion_VERSION_3)
            {
                if (t.HasNormals && toWriteNormals)
                {
                    if (!compact)
                    {
                        os.Write("\nNormals :\n");
                    }

                    for (j = 1; j <= nbNodes; j++)
                    {
                        // std::setw(17) applies to the first component only
                        bool isSetw = false;
                        if (!compact)
                        {
                            Setw(os, 10, j);
                            os.Write(" : ");
                            isSetw = true;
                        }

                        NCollection_Vec3<float> norm;
                        for (int k = 0; k < 3; ++k)
                        {
                            t.Normal(j, out norm);
                            if (isSetw)
                            {
                                Setw(os, 17, norm[k]);
                                isSetw = false;
                            }
                            else
                            {
                                Out(os, norm[k]);
                            }

                            os.Write(!compact ? "\n" : " ");
                        }
                    }
                }
            }

            os.Write("\n");
        }
    }

    /// <summary>Dumps the triangulations in a readable form.</summary>
    public void DumpTriangulation(TextWriter os)
    {
        WriteTriangulation(os, false);
    }

    /// <summary>Reads the triangulations, written by <see cref="WriteTriangulation"/>.</summary>
    /// <exception cref="StreamFormatErrorException">The input can't be read safely.</exception>
    public void ReadTriangulation(TextReader IS, ProgressRange progress = default)
    {
        char[] buffer = new char[255];
        int i, j, nbtri = 0;
        double d, x, y, z;
        int nbNodes = 0, nbTriangles = 0;
        bool hasUV = false;
        bool hasNormals = false;

        Triangulation t;

        StreamUtilities.ReadWord(IS, buffer);
        if (!StreamUtilities.CString(buffer).Contains("Triangulations", StringComparison.Ordinal))
        {
            return;
        }

        StreamUtilities.Read(IS, ref nbtri);
        // Kerf: untrusted input: a triangulation takes at least 10 characters ("0 0 0 0 0\n\n")
        StreamUtilities.CheckCount(IS, Math.Max(nbtri, 0), 10, "triangulations");
        // OCC19559
        using var ps = new ProgressScope(progress, "Triangulations", nbtri);
        for (i = 1; i <= nbtri && ps.More(); i++, ps.Next())
        {
            StreamUtilities.Read(IS, ref nbNodes);
            StreamUtilities.Read(IS, ref nbTriangles);
            StreamUtilities.Read(IS, ref hasUV);
            if (FormatNb() >= (int)FormatVersion.FormatVersion_VERSION_3)
            {
                StreamUtilities.Read(IS, ref hasNormals);
            }

            // Kerf: untrusted input: OCCT allocates for any count, and again for each triangulation on a failed
            // stream; a node takes at least 6 characters, a triangle 6
            StreamUtilities.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadTriangulation");
            StreamUtilities.CheckCount(IS, nbNodes, hasUV ? 10 : 6, "triangulation nodes");
            StreamUtilities.CheckCount(IS, nbTriangles, 6, "triangles");
            StreamUtilities.CheckCount(IS, (long)nbNodes * (hasUV ? 10 : 6) + (long)nbTriangles * 6, 1, "triangulation data");
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out d);

            t = new Triangulation(nbNodes, nbTriangles, hasUV, hasNormals);

            for (j = 1; j <= nbNodes; j++)
            {
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out z);
                t.SetNode(j, new Point3D(x, y, z));
            }

            if (hasUV)
            {
                for (j = 1; j <= nbNodes; j++)
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
                    t.SetUVNode(j, new Point2D(x, y));
                }
            }

            // read the triangles
            // OCCT's n1, n2 and n3 are uninitialised, and left so when the stream has failed
            int n1 = 0, n2 = 0, n3 = 0;
            for (j = 1; j <= nbTriangles; j++)
            {
                StreamUtilities.Read(IS, ref n1);
                StreamUtilities.Read(IS, ref n2);
                StreamUtilities.Read(IS, ref n3);
                t.SetTriangle(j, new Triangle1(n1, n2, n3));
            }

            if (hasNormals)
            {
                double normX, normY, normZ;
                for (j = 1; j <= nbNodes; j++)
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out normX);
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out normY);
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out normZ);
                    t.SetNormal(j, new NCollection_Vec3<float>((float)normX, (float)normY, (float)normZ));
                }
            }

            t.Deflection(d);
            _triangulations.Add(t, hasNormals);
        }
    }
}
