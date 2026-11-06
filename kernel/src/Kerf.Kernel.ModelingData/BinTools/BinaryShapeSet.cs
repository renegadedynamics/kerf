// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// A set of shapes with their geometry, written and read in OCCT's table-based binary BRep format (versions 1 to 4):
/// a header line, the locations, the 2D curves, curves, 3D polygons, polygons on triangulations, surfaces and
/// triangulations, then the TShapes, sub-shapes first, each referring to its sub-shapes by index.
/// </summary>
/// <remarks>
/// <para>
/// <see cref="Read(Stream, ProgressRange)"/> freezes the shapes it creates once it has read them all (or has
/// failed), so every shape a set hands out after a read is frozen, as phase3-design.md, section 4.2, requires. The
/// writers write the seven flags OCCT writes (Free to Convex: the Locked bit isn't part of this format).
/// </para>
/// <para>
/// Reading a vertex in versions 1, 2 and 4 steps back in the stream, as OCCT does, so the stream must be seekable;
/// <see cref="BinTools.Read(out Shape, Stream, ProgressRange)"/> buffers a stream that isn't.
/// </para>
/// </remarks>
internal class BinaryShapeSet : ShapeSetBase
{
    // index and its shape (started from 1)
    private readonly IndexedSet<Shape> _shapes = new(ShapeHasher.Instance);
    private readonly BinaryLocationSet _locations = new();
    private readonly BrepBuilder _builder = new();
    private readonly BinarySurfaceSet _surfaces = new();
    private readonly BinaryCurveSet _curves = new();
    private readonly BinaryCurve2dSet _curves2d = new();
    private readonly IndexedSet<Polygon2D> _polygons2D = new(ReferenceEqualityComparer.Instance);
    private readonly IndexedSet<Polygon3D> _polygons3D = new(ReferenceEqualityComparer.Instance);

    // with a flag telling whether the triangulation's normals are written
    private readonly IndexedMap<Triangulation, bool> _triangulations = new(ReferenceEqualityComparer.Instance);
    private readonly IndexedSet<PolygonOnTriangulation> _nodes = new(ReferenceEqualityComparer.Instance);

    // Kerf: untrusted input: the nesting depth of each shape read, by index - 1
    private readonly List<int> _depths = [];

    /// <summary>An empty set, in the current format version, without triangulations.</summary>
    public BinaryShapeSet()
    {
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
        _shapes.Clear();
        _locations.Clear();
        _depths.Clear();
    }

    /// <summary>
    /// Adds the shape, its sub-shapes (first) and their geometry to the set, and returns its index; 0 for a null shape.
    /// </summary>
    public int Add(in Shape shape)
    {
        if (shape.IsNull)
        {
            return 0;
        }

        // Kerf: a shape nested too deeply for the stack throws InsufficientExecutionStackException
        System.Runtime.CompilerServices.RuntimeHelpers.EnsureSufficientExecutionStack();

        _locations.Add(shape.Placement);
        Shape s2 = shape;
        s2.Location(new Placement());
        int index = _shapes.IndexOf(s2) + 1;
        if (index == 0)
        {
            AddShape(s2);
            for (var its = new Iterator(s2, false, false); its.More(); its.Next())
            {
                Add(its.Value());
            }

            index = _shapes.Add(s2) + 1;
        }

        return index;
    }

    /// <summary>The shape of index I.</summary>
    /// <exception cref="OutOfRangeException">There is no shape of index I.</exception>
    public Shape Shape(int indx)
    {
        return _shapes[indx - 1];
    }

    /// <summary>The index of the shape (its TShape and location), 0 if it isn't in the set.</summary>
    public int Index(in Shape shape)
    {
        return _shapes.IndexOf(shape) + 1;
    }

    /// <summary>The locations of the set.</summary>
    public BinaryLocationSet Locations()
    {
        return _locations;
    }

    /// <summary>The locations of the set, to change.</summary>
    public BinaryLocationSet ChangeLocations()
    {
        return _locations;
    }

    /// <summary>The number of shapes.</summary>
    public int ShapeCount
    {
        get
        {
            return _shapes.Count;
        }
    }

    /// <summary>Adds the geometry of the shape (not its sub-shapes) to the set.</summary>
    public virtual void AddShape(in Shape s)
    {
        // Add the geometry
        if (s.Type == ShapeType.Vertex)
        {
            BrepTVertex tv = (BrepTVertex)s.TShape()!;
            foreach (PointRepresentation pr in tv.Points())
            {
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
            }
        }
        else if (s.Type == ShapeType.Edge)
        {
            // Add the curve geometry
            BrepTEdge te = (BrepTEdge)s.TShape()!;
            foreach (CurveRepresentation cr in te.Curves())
            {
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
                else if (IsWithTriangles)
                {
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
                        // Add() is right: the shapes are added from complex to elementary ones, so a face's
                        // triangulation is added (with its normals flag) before its edges'.
                        if (cr.Triangulation() is null)
                        {
                            // OCCT adds the null handle and dereferences it in WriteTriangulation, which throws (a caught
                            // access violation); the port's maps can't hold null, so it throws here
                            throw new FailureException("BinTools_ShapeSet: a polygon on a null triangulation");
                        }

                        _triangulations.Add(cr.Triangulation()!, false); // edge triangulation does not need normals
                        _nodes.Add(cr.PolygonOnTriangulation()!);
                        ChangeLocations().Add(cr.Location());
                        if (cr.IsPolygonOnClosedTriangulation)
                        {
                            _nodes.Add(cr.PolygonOnTriangulation2()!);
                        }
                    }
                    else if (cr.IsPolygonOnSurface())
                    {
                        _surfaces.Add(cr.Surface());
                        // Kerf: the 2D polygons are never written; the port's map can't hold a null one
                        if (cr.Polygon() is not null)
                        {
                            _polygons2D.Add(cr.Polygon()!);
                        }

                        ChangeLocations().Add(cr.Location());
                        if (cr.IsPolygonOnClosedSurface)
                        {
                            if (cr.Polygon2() is not null)
                            {
                                _polygons2D.Add(cr.Polygon2()!);
                            }
                        }
                    }
                }
            }
        }
        else if (s.Type == ShapeType.Face)
        {
            // Add the surface geometry
            bool needNormals = IsWithNormals;
            BrepTFace tf = (BrepTFace)s.TShape()!;
            if (tf.Surface() is not null)
            {
                _surfaces.Add(tf.Surface());
            }
            else
            {
                needNormals = true;
            }

            if (IsWithTriangles || tf.Surface() is null)
            {
                Triangulation? tr = tf.Triangulation();
                if (tr is not null)
                {
                    _triangulations.Add(tr, needNormals);
                }
            }

            ChangeLocations().Add(tf.Location());
        }
    }

    /// <summary>Writes the geometry of the set: 2D curves, curves, 3D polygons, polygons on triangulations, surfaces, triangulations.</summary>
    public virtual void WriteGeometry(Stream os, ProgressRange range = default)
    {
        using var ps = new ProgressScope(range, "Writing geometry", 6);
        _curves2d.Write(os, ps.Next());
        if (!ps.More())
        {
            return;
        }

        _curves.Write(os, ps.Next());
        if (!ps.More())
        {
            return;
        }

        WritePolygon3D(os, ps.Next());
        if (!ps.More())
        {
            return;
        }

        WritePolygonOnTriangulation(os, ps.Next());
        if (!ps.More())
        {
            return;
        }

        _surfaces.Write(os, ps.Next());
        if (!ps.More())
        {
            return;
        }

        WriteTriangulation(os, ps.Next());
    }

    /// <summary>
    /// Writes the set: the header line, the locations, the geometry, then the shapes, sub-shapes first, each with its
    /// type, geometry, flags and sub-shapes.
    /// </summary>
    public override void Write(Stream os, ProgressRange range = default)
    {
        // write the copyright
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, THE_ASCII_VERSIONS[FormatNb()]);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");

        //-----------------------------------------
        // write the locations
        //-----------------------------------------

        _locations.Write(os);

        //-----------------------------------------
        // write the geometry
        //-----------------------------------------

        using var ps = new ProgressScope(range, "Writing geometry", 2);

        WriteGeometry(os, ps.Next());
        if (!ps.More())
        {
            return;
        }

        //-----------------------------------------
        // write the shapes
        //-----------------------------------------

        int i, nbShapes = _shapes.Count;
        using var pSinner = new ProgressScope(ps.Next(), "Writing shapes", nbShapes);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\nTShapes ");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, nbShapes);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");

        // subshapes are written first
        for (i = 1; i <= nbShapes && pSinner.More(); i++, pSinner.Next())
        {
            Shape s = _shapes[i - 1];

            // Type
            os.WriteByte((byte)s.Type);

            // Geometry
            WriteShape(s, os);

            // Flags (OCCT writes no Locked bit in this format)
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, s.Free());
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, s.Modified());
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, s.Checked());
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, s.OrientableFlag);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, s.ClosedFlag);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, s.InfiniteFlag);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, s.ConvexFlag);

            // sub-shapes
            for (var its = new Iterator(s, false, false); its.More(); its.Next())
            {
                Write(its.Value(), os);
            }

            Write(new Shape(), os); // Null shape to end the list
        }
    }

    /// <summary>
    /// Clears the set and reads one written by <see cref="Write(Stream, ProgressRange)"/>. The shapes are frozen.
    /// If the stream has no header line of a known version, writes a message to the console and returns, as OCCT does.
    /// </summary>
    /// <exception cref="FailureException">The stream isn't a binary shape set OCCT can read; for input that can't be
    /// read safely, a <see cref="StreamFormatErrorException"/>.</exception>
    public override void Read(Stream IS, ProgressRange range = default)
    {
        try
        {
            ReadSet(IS, range);
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsSignal(exception))
        {
            // Kerf: untrusted input: a malformed stream raised an exception that OCCT doesn't catch here
            throw new StreamFormatErrorException("BinTools_ShapeSet::Read: " + Refusal.Unwrapped(exception).Message, exception);
        }
        finally
        {
            // design 4.2: the shapes freeze once the whole set is read; on a failure too, since the set hands them out
            for (int i = 0; i < _shapes.Count; i++)
            {
                _shapes[i].TShape()!.FreezeGraph();
            }
        }
    }

    private void ReadSet(Stream IS, ProgressRange range)
    {
        Clear();

        // Check the version
        bool isSetFormat = false;
        bool isFail;
        bool isEof = false;
        do
        {
            isFail = !global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetLine(IS, out string vers, ref isEof);

            // OCCT compares C strings
            int nul = vers.IndexOf('\0', StringComparison.Ordinal);
            if (nul >= 0)
            {
                vers = vers[..nul];
            }

            // BUC60769 PTV 18.10.2000: remove possible '\r' at the end of the line
            int lv = vers.Length;
            if (lv > 0)
            {
                for (lv--; lv > 0 && (vers[lv] == '\r' || vers[lv] == '\n'); lv--)
                {
                    vers = vers[..lv];
                }
            }

            for (int i = (int)BinaryFormatVersionBounds.BinTools_FormatVersion_LOWER;
                 i <= (int)BinaryFormatVersionBounds.BinTools_FormatVersion_UPPER;
                 ++i)
            {
                if (vers == THE_ASCII_VERSIONS[i])
                {
                    SetFormatNb(i);
                    isSetFormat = true;
                    break;
                }
            }

            if (isSetFormat)
            {
                break;
            }
        } while (!isFail);

        if (isFail)
        {
            Console.Out.Write("BinTools_ShapeSet::Read: File was not written with this version of the topology" + "\n");
            return;
        }

        //-----------------------------------------
        // read the locations
        //-----------------------------------------

        _locations.Read(IS);

        //-----------------------------------------
        // read the geometry
        //-----------------------------------------

        using var pSouter = new ProgressScope(range, "Reading", 2);
        ReadGeometry(IS, pSouter.Next());
        if (!pSouter.More())
        {
            return;
        }

        //-----------------------------------------
        // read the shapes
        //-----------------------------------------

        string? buffer = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadWord(IS);
        if (buffer is null || buffer != "TShapes")
        {
            throw new FailureException("BinTools_ShapeSet::Read: Not a TShape table" + "\n");
        }

        int nbShapes = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedInteger(IS);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // remove lf
        // Kerf: untrusted input: a shape takes at least 9 bytes (its type, 7 flags and the end of its sub-shapes)
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbShapes, 9, "shapes");
        using var pSinner = new ProgressScope(pSouter.Next(), "Reading Shapes", nbShapes);
        for (int i = 1; i <= nbShapes && pSinner.More(); i++, pSinner.Next())
        {
            Shape s = new();
            ShapeType t = (ShapeType)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS);
            ReadShape(t, IS, ref s);
            ReadFlagsAndSubs(ref s, t, IS, nbShapes);
            _shapes.Add(s);

            // Kerf: untrusted input: cap the nesting, which recursive writers and explorers follow
            int depth = 1;
            for (var its = new Iterator(s, false, false); its.More(); its.Next())
            {
                int sub = _shapes.IndexOf(its.Value().Located(new Placement())) + 1;
                depth = Math.Max(depth, _depths[sub - 1] + 1);
            }

            if (depth > global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxShapeNesting)
            {
                throw new StreamFormatErrorException("BinTools_ShapeSet::Read: shapes nested too deeply");
            }

            _depths.Add(depth);
        }
    }

    /// <summary>Writes a reference to the shape: its orientation, the index of its TShape and of its location.</summary>
    public override void Write(in Shape s, Stream os)
    {
        if (s.IsNull)
        {
            os.WriteByte((byte)'*');
        }
        else
        {
            // {TopAbs_FORWARD, TopAbs_REVERSED, TopAbs_INTERNAL, TopAbs_EXTERNAL}
            os.WriteByte((byte)s.Orientation);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _shapes.Count - (_shapes.IndexOf(s.Located(new Placement())) + 1) + 1);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(s.Placement));
        }
    }

    // Reads the flags and the sub-shapes of a shape just created by ReadShape
    internal virtual void ReadFlagsAndSubs(ref Shape s, ShapeType t, Stream IS, int nbShapes)
    {
        // Set the flags
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool free);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool mod);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool aChecked);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool orient);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool closed);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool inf);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool conv);

        // sub-shapes
        Shape ss = new();
        do
        {
            ReadSubs(ref ss, IS, nbShapes);
            if (!ss.IsNull)
            {
                AddShapes(ref s, ss);
            }
        } while (!ss.IsNull);

        s.Free(free);
        s.Modified(mod);
        if (FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_1)
        {
            aChecked = false; // force check at reading
        }

        s.Checked(aChecked);
        s.Orientable(orient);
        s.Closed(closed);
        s.Infinite(inf);
        s.Convex(conv);

        // check
        if (FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_1)
        {
            if (t == ShapeType.Face)
            {
                Face f = ShapeCasts.Face(s);
                BrepTools.Update(f);
            }
        }
    }

    /// <summary>
    /// Reads a reference to a shape written by <see cref="Write(in Shape, Stream)"/>: one of the
    /// <paramref name="nbshapes"/> shapes of the set, or a null shape for the end of a list.
    /// </summary>
    public virtual void ReadSubs(ref Shape s, Stream IS, int nbshapes)
    {
        int aChar = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedChar(IS);
        if (aChar == '*')
        {
            s = new Shape();
        }
        else
        {
            Orientation orient;
            // Kerf: untrusted input: OCCT casts any character
            orient = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckOrientation(aChar);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int indx);
            long shapeIndex = (long)nbshapes - indx + 1;
            // Kerf: untrusted input: OCCT reads outside its map
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(shapeIndex is >= int.MinValue and <= int.MaxValue ? (int)shapeIndex : 0,
                                1,
                                _shapes.Count,
                                "shape");
            s = Shape((int)shapeIndex);
            s.Orient(orient);

            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int l);
            // Kerf: untrusted input
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l, 0, _locations.LocationCount, "location");
            s.Location(_locations.Location(l), false);
        }
    }

    /// <summary>Reads the geometry of the set, as <see cref="WriteGeometry"/> writes it.</summary>
    public virtual void ReadGeometry(Stream IS, ProgressRange range = default)
    {
        using var ps = new ProgressScope(range, "Reading geometry", 6);
        _curves2d.Read(IS, ps.Next());
        if (!ps.More())
        {
            return;
        }

        _curves.Read(IS, ps.Next());
        if (!ps.More())
        {
            return;
        }

        ReadPolygon3D(IS, ps.Next());
        if (!ps.More())
        {
            return;
        }

        ReadPolygonOnTriangulation(IS, ps.Next());
        if (!ps.More())
        {
            return;
        }

        _surfaces.Read(IS, ps.Next());
        if (!ps.More())
        {
            return;
        }

        ReadTriangulation(IS, ps.Next());
    }

    /// <summary>Writes the geometry of the shape (not its sub-shapes).</summary>
    public virtual void WriteShape(in Shape s, Stream os)
    {
        // Write the geometry
        try
        {
            if (s.Type == ShapeType.Vertex)
            {
                // Write the point geometry
                Vertex v = ShapeCasts.Vertex(s);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, Tool.Tolerance(v));
                Point3D p = Tool.Pnt(v);
                Write(os, p);

                BrepTVertex tv = (BrepTVertex)s.TShape()!;
                foreach (PointRepresentation pr in tv.Points())
                {
                    if (pr.IsPointOnCurve())
                    {
                        os.WriteByte(1); // 1
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pr.Parameter());
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _curves.Index(pr.Curve()));
                    }
                    else if (pr.IsPointOnCurveOnSurface())
                    {
                        os.WriteByte(2); // 2
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pr.Parameter());
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _curves2d.Index(pr.PCurve()));
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _surfaces.Index(pr.Surface()));
                    }
                    else if (pr.IsPointOnSurface())
                    {
                        // the second parameter first, as OCCT writes it: the readers take it so (OCCT's took it for the
                        // first, swapping the parameters on each round trip; Kerf decision 1500)
                        os.WriteByte(3); // 3
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pr.Parameter2());
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pr.Parameter());
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _surfaces.Index(pr.Surface()));
                    }

                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(pr.Location()));
                }

                os.WriteByte(0);
            }
            else if (s.Type == ShapeType.Edge)
            {
                // Write the curve geometry
                BrepTEdge te = (BrepTEdge)s.TShape()!;

                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, te.Tolerance());

                bool val = te.SameParameter();
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, val);
                val = te.SameRange();
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, val);
                val = te.Degenerated();
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, val);

                double first, last;
                foreach (CurveRepresentation cr in te.Curves())
                {
                    if (cr.IsCurve3D)
                    {
                        if (cr.Curve3D() is not null)
                        {
                            GCurve gc = (GCurve)cr;
                            gc.Range(out first, out last);
                            os.WriteByte(1); // CURVE_3D;
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _curves.Index(cr.Curve3D()));
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(cr.Location()));
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, first);
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, last);
                        }
                    }
                    else if (cr.IsCurveOnSurface())
                    {
                        GCurve gc = (GCurve)cr;
                        gc.Range(out first, out last);
                        if (!cr.IsCurveOnClosedSurface)
                        {
                            // -2- Curve on surf
                            os.WriteByte(2);
                        }
                        else
                        {
                            // -3- Curve on closed surf
                            os.WriteByte(3);
                        }

                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _curves2d.Index(cr.PCurve()));
                        if (cr.IsCurveOnClosedSurface)
                        {
                            //+ int|char
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _curves2d.Index(cr.PCurve2()));
                            os.WriteByte((byte)cr.Continuity());
                        }

                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _surfaces.Index(cr.Surface()));
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(cr.Location()));
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, first);
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, last);

                        // Write UV Points for higher performance
                        if (FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_2
                            || FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_3)
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

                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pf.X);
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pf.Y);
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pl.X);
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pl.Y);
                        }
                    }
                    else if (cr.IsRegularity())
                    {
                        // -4- Regularity
                        os.WriteByte(4);
                        os.WriteByte((byte)cr.Continuity());
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _surfaces.Index(cr.Surface()));
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(cr.Location()));
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _surfaces.Index(cr.Surface2()));
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(cr.Location2()));
                    }
                    else if (IsWithTriangles)
                    {
                        if (cr.IsPolygon3D)
                        {
                            Polygon3DRepresentation gc = (Polygon3DRepresentation)cr;
                            if (gc.Polygon3D() is not null)
                            {
                                // -5- Polygon3D
                                os.WriteByte(5);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _polygons3D.IndexOf(cr.Polygon3D()!) + 1);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(cr.Location()));
                            }
                        }
                        else if (cr.IsPolygonOnTriangulation())
                        {
                            PolygonOnTriangulationRepresentation pt = (PolygonOnTriangulationRepresentation)cr;
                            if (!cr.IsPolygonOnClosedTriangulation)
                            {
                                // -6- Polygon on triangulation
                                os.WriteByte(6);
                            }
                            else
                            {
                                // -7- Polygon on closed triangulation
                                os.WriteByte(7);
                            }

                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _nodes.IndexOf(pt.PolygonOnTriangulation()!) + 1);

                            if (cr.IsPolygonOnClosedTriangulation)
                            {
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _nodes.IndexOf(pt.PolygonOnTriangulation2()!) + 1);
                            }

                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _triangulations.IndexOf(pt.Triangulation()!) + 1);
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(cr.Location()));
                        }
                    }
                }

                os.WriteByte(0); // end of the list of representations
            }
            else if (s.Type == ShapeType.Face)
            {
                BrepTFace tf = (BrepTFace)s.TShape()!;
                Face f = ShapeCasts.Face(s);

                // Write the surface geometry
                bool natRes = Tool.NaturalRestriction(f);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, natRes);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, tf.Tolerance());
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, tf.Surface() is not null ? _surfaces.Index(tf.Surface()) : 0);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, Locations().Index(tf.Location()));

                if (IsWithTriangles || tf.Surface() is null)
                {
                    if (tf.Triangulation() is not null)
                    {
                        os.WriteByte(2);
                        // Write the triangulation
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, _triangulations.IndexOf(tf.Triangulation()!) + 1);
                    }
                    else
                    {
                        os.WriteByte(1);
                    }
                }
                else
                {
                    os.WriteByte(0); // without triangulation
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeSet::WriteGeometry(S,OS)", exception);
        }
    }

    // Reads the geometry of a shape of type T into a new shape S (not frozen: the set freezes it after reading)
    internal virtual void ReadShape(ShapeType T, Stream IS, ref Shape S)
    {
        // Read the geometry
        int val, c, pc, pc2 = 0, s, s2, l, l2, t, pt, pt2 = 0;
        double tol, x, y, z, first, last, p1 = 0.0, p2;
        double pfX, pfY, plX, plY;
        Point2D pf = new(), pl = new();
        bool closed, bval;
        Continuity reg = Continuity.C0;
        try
        {
            switch (T)
            {
                //---------
                // vertex
                //---------
                case ShapeType.Vertex:
                {
                    Vertex v = new();
                    // Read the point geometry
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out tol);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out x);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out y);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out z);
                    Point3D pnt = new(x, y, z);
                    _builder.MakeVertex(ref v, pnt, tol);
                    S = v;
                    BrepTVertex tv = (BrepTVertex)v.TShape()!;
                    NCollection_List<PointRepresentation> lpr = tv.ChangePoints();
                    Placement L = new();

                    do
                    {
                        if (FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_3)
                        {
                            val = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // case {0|1|2|3}
                            if (val > 0 && val <= 3)
                            {
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out p1);
                            }
                        }
                        else
                        {
                            // OCCT (versions 1, 2 and 4): reads the parameter before the type, as older files store
                            // them, and steps back when the byte after it isn't a type. A version 4 file stores the
                            // type first, so a parameter whose last byte is 1, 2 or 3 (a positive real below about
                            // 1e-303) is misread (occt-issues.md).
                            if (!IS.CanSeek)
                            {
                                throw new ProgramErrorException("BinTools_ShapeSet: reading a vertex needs a seekable stream");
                            }

                            long pos = IS.Position;
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out p1);
                            val = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // case {0|1|2|3}
                            if (val != 1 && val != 2 && val != 3)
                            {
                                IS.Seek(pos, SeekOrigin.Begin);
                                val = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS);
                                if (val > 0 && val <= 3)
                                {
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out p1);
                                }
                            }
                        }

                        PointRepresentation? pr = null;
                        switch (val)
                        {
                            case 0:
                                break;

                            case 1:
                            {
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out c);
                                // Kerf: untrusted input
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(c, 0, _curves.CurveCount, "curve");
                                if (_curves.Curve(c) is null)
                                {
                                    break;
                                }

                                PointOnCurve poc = new(p1, _curves.Curve(c), L);
                                pr = poc;
                            }
                                break;

                            case 2:
                            {
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out pc);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out s);
                                // Kerf: untrusted input
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(pc, 1, _curves2d.CurveCount, "2D curve");
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(s, 1, _surfaces.SurfaceCount, "surface");
                                if (_curves2d.Curve2d(pc) is null || _surfaces.Surface(s) is null)
                                {
                                    break;
                                }

                                PointOnCurveOnSurface poc = new(p1, _curves2d.Curve2d(pc), _surfaces.Surface(s), L);
                                pr = poc;
                            }
                                break;

                            case 3:
                            {
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out p2);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out s);
                                // Kerf: untrusted input
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(s, 1, _surfaces.SurfaceCount, "surface");
                                if (_surfaces.Surface(s) is null)
                                {
                                    break;
                                }

                                // the second parameter is written first (see WriteShape; OCCT read it as the first)
                                PointOnSurface poc = new(p2, p1, _surfaces.Surface(s), L);
                                pr = poc;
                            }
                                break;

                            default:
                            {
                                throw new FailureException("BinTools_SurfaceSet::ReadGeometry: UnExpected BRep_PointRepresentation = "
                                                           + val + "\n");
                            }
                        }

                        if (val > 0)
                        {
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l); // Locations index
                            // Kerf: untrusted input
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l, 0, _locations.LocationCount, "location");

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
                {
                    // Create an edge
                    Edge e = new();
                    _builder.MakeEdge(ref e);
                    S = e;

                    // Read the curve geometry
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out tol);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bval);
                    _builder.SameParameter(e, bval);

                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bval);
                    _builder.SameRange(e, bval);

                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bval);
                    _builder.Degenerated(e, bval);

                    do
                    {
                        val = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); //{0|1|2|3|4|5|6|7}
                        // -0- no representation
                        // -1- Curve 3D
                        // -2- Curve on surf
                        // -3- Curve on closed surf
                        // -4- Regularity
                        // -5- Polygon3D
                        // -6- Polygon on triangulation
                        // -7- Polygon on closed triangulation

                        switch (val)
                        {
                            case 0:
                                break;

                            case 1: // -1- Curve 3D
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out c);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l);
                                // Kerf: untrusted input
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(c, 0, _curves.CurveCount, "curve");
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l, 0, _locations.LocationCount, "location");
                                if (_curves.Curve(c) is not null)
                                {
                                    _builder.UpdateEdge(e, _curves.Curve(c), Locations().Location(l), tol);
                                }

                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out first);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out last);
                                if (_curves.Curve(c) is not null)
                                {
                                    bool only3d = true;
                                    _builder.Range(e, first, last, only3d);
                                }

                                break;

                            case 2: // -2- Curve on surf
                            case 3: // -3- Curve on closed surf
                                closed = (val == 3);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out pc);
                                if (closed)
                                {
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out pc2);
                                    reg = (Continuity)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS);
                                }

                                // surface, location
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out s);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l);

                                // range
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out first);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out last);

                                // read UV Points // for XML Persistence higher performance
                                if (FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_2
                                    || FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_3)
                                {
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out pfX);
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out pfY);
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out plX);
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out plY);
                                    pf = new Point2D(pfX, pfY);
                                    pl = new Point2D(plX, plY);
                                }

                                // Kerf: untrusted input
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(pc, 1, _curves2d.CurveCount, "2D curve");
                                if (closed)
                                {
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(pc2, 1, _curves2d.CurveCount, "2D curve");
                                }

                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(s, 1, _surfaces.SurfaceCount, "surface");
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l, 0, _locations.LocationCount, "location");

                                if (_curves2d.Curve2d(pc) is null || (closed && _curves2d.Curve2d(pc2) is null)
                                    || _surfaces.Surface(s) is null)
                                {
                                    break;
                                }

                                if (closed)
                                {
                                    if (FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_2
                                        || FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_3)
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
                                    if (FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_2
                                        || FormatNb() == (int)BinaryFormatVersion.FormatVersion_VERSION_3)
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
                                reg = (Continuity)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out s);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out s2);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l2);
                                // Kerf: untrusted input
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(s, 1, _surfaces.SurfaceCount, "surface");
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l, 0, _locations.LocationCount, "location");
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(s2, 1, _surfaces.SurfaceCount, "surface");
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l2, 0, _locations.LocationCount, "location");
                                if (_surfaces.Surface(s) is null || _surfaces.Surface(s2) is null)
                                {
                                    break;
                                }

                                _builder.Continuity(e,
                                                     _surfaces.Surface(s),
                                                     _surfaces.Surface(s2),
                                                     Locations().Location(l),
                                                     Locations().Location(l2),
                                                     reg);
                                break;

                            case 5: // -5- Polygon3D
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out c);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l);
                                // Kerf: untrusted input
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(c, 1, _polygons3D.Count, "3D polygon");
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l, 0, _locations.LocationCount, "location");
                                //??? Bug?  myBuilder.UpdateEdge(E,myPolygons3D(c));
                                _builder.UpdateEdge(e, _polygons3D[c - 1], Locations().Location(l));
                                break;

                            case 6: // -6- Polygon on triangulation
                            case 7: // -7- Polygon on closed triangulation
                                closed = (val == 7);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out pt);
                                if (closed)
                                {
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out pt2);
                                }

                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out t);
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l);
                                // Kerf: untrusted input
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(pt, 1, _nodes.Count, "polygon on triangulation");
                                if (closed)
                                {
                                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(pt2, 1, _nodes.Count, "polygon on triangulation");
                                }

                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(t, 1, _triangulations.Count, "triangulation");
                                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l, 0, _locations.LocationCount, "location");
                                if (closed)
                                {
                                    _builder.UpdateEdge(e,
                                                         _nodes[pt - 1],
                                                         _nodes[pt2 - 1],
                                                         _triangulations.KeyAt(t - 1),
                                                         Locations().Location(l));
                                }
                                else
                                {
                                    _builder.UpdateEdge(e,
                                                         _nodes[pt - 1],
                                                         _triangulations.KeyAt(t - 1),
                                                         Locations().Location(l));
                                }

                                // range
                                break;

                            default:
                            {
                                throw new FailureException("Unexpected Curve Representation =" + val + "\n");
                            }
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
                    _builder.MakeFace(ref f);
                    S = f;
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bval); // NaturalRestriction flag
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out tol);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out s); // surface indx
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out l); // location indx
                    // Kerf: untrusted input
                    if (s > 0)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(s, 1, _surfaces.SurfaceCount, "surface");
                    }

                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(l, 0, _locations.LocationCount, "location");
                    _builder.UpdateFace(f, s > 0 ? _surfaces.Surface(s) : null, Locations().Location(l), tol);
                    _builder.NaturalRestriction(f, bval);

                    byte aByte = (byte)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS);
                    // triangulation case
                    if (aByte == 2)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out s);
                        // Kerf: untrusted input
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckIndex(s, 1, _triangulations.Count, "triangulation");
                        _builder.UpdateFace(f, _triangulations.KeyAt(s - 1));
                    }
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
                {
                    throw new FailureException("Unexpected topology type = " + (int)T + "\n");
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeSet::ReadGeometry(S,OS)", exception);
        }
    }

    // Inserts S2 in S1 (the builder's Add)
    internal virtual void AddShapes(ref Shape s1, in Shape s2)
    {
        _builder.Add(s1, s2);
    }

    /// <summary>Writes the polygons on triangulations ("PolygonOnTriangulations", the count, then each polygon).</summary>
    public void WritePolygonOnTriangulation(Stream os, ProgressRange range = default)
    {
        int nbPol = _nodes.Count;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "PolygonOnTriangulations ");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, nbPol);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");
        try
        {
            using var ps = new ProgressScope(range, "Writing polygons on triangulation", nbPol);
            for (int polIter = 1; polIter <= nbPol && ps.More(); ++polIter, ps.Next())
            {
                PolygonOnTriangulation poly = _nodes[polIter - 1];
                NCollection_Array1<int> nodes = poly.Nodes();
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, nodes.Length());
                for (int nodeIter = 1; nodeIter <= nodes.Length(); ++nodeIter)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, nodes.Value(nodeIter));
                }

                // write the deflection
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, poly.Deflection());

                // writing parameters
                if (poly.Parameters() is NCollection_HArray1<double> param)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, true);
                    for (int nodeIter = 1; nodeIter <= param.Length(); ++nodeIter)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, param.Value(nodeIter));
                    }
                }
                else
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, false);
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeSet::WritePolygonOnTriangulation(..)", exception);
        }
    }

    /// <summary>Reads the polygons on triangulations written by <see cref="WritePolygonOnTriangulation"/>.</summary>
    public void ReadPolygonOnTriangulation(Stream IS, ProgressRange range = default)
    {
        string? header = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadWord(IS);
        if (header is null || !header.Contains("PolygonOnTriangulations", StringComparison.Ordinal))
        {
            throw new FailureException("BinTools_ShapeSet::ReadPolygonOnTriangulation: Not a PolygonOnTriangulation section");
        }

        int nbPol = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedInteger(IS);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // remove LF
        // Kerf: untrusted input: a polygon takes at least 13 bytes
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbPol, 13, "polygons on triangulation");
        try
        {
            using var ps = new ProgressScope(range, "Reading Polygones on triangulation", nbPol);
            for (int polIter = 1; polIter <= nbPol && ps.More(); ++polIter, ps.Next())
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int nbNodes);
                // Kerf: untrusted input
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbNodes, 4, "nodes");
                PolygonOnTriangulation poly = new(nbNodes, false);
                for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int node);
                    poly.SetNode(nodeIter, node);
                }

                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double defl);
                poly.Deflection(defl);

                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool hasParameters);
                if (hasParameters)
                {
                    // Kerf: untrusted input
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbNodes, 8, "parameters");
                    NCollection_HArray1<double> aParams = new(1, nbNodes);
                    for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out aParams.ChangeValue(nodeIter));
                    }

                    poly.SetParameters(aParams);
                }

                _nodes.Add(poly);
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeSet::ReadPolygonOnTriangulation(..)", exception);
        }
    }

    /// <summary>Writes the 3D polygons ("Polygon3D", the count, then each polygon).</summary>
    public void WritePolygon3D(Stream os, ProgressRange range = default)
    {
        int nbPol = _polygons3D.Count;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "Polygon3D ");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, nbPol);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");
        try
        {
            using var ps = new ProgressScope(range, "Writing polygons 3D", nbPol);
            for (int polIter = 1; polIter <= nbPol && ps.More(); ++polIter, ps.Next())
            {
                Polygon3D poly = _polygons3D[polIter - 1];
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, poly.NodeCount);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, poly.HasParameters);

                // write the deflection
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, poly.Deflection());

                // write the nodes
                int nbNodes = poly.NodeCount;
                NCollection_Array1<Point3D> nodes = poly.Nodes();
                for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                {
                    Point3D pnt = nodes.Value(nodeIter);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pnt.X);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pnt.Y);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pnt.Z);
                }

                if (poly.HasParameters)
                {
                    NCollection_Array1<double> param = poly.Parameters();
                    for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, param.Value(nodeIter));
                    }
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeSet::WritePolygon3D(..)", exception);
        }
    }

    /// <summary>Reads the 3D polygons written by <see cref="WritePolygon3D"/>.</summary>
    public void ReadPolygon3D(Stream IS, ProgressRange range = default)
    {
        string? header = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadWord(IS);
        if (header is null || !header.Contains("Polygon3D", StringComparison.Ordinal))
        {
            throw new FailureException("BinTools_ShapeSet::ReadPolygon3D: Not a Polygon3D section");
        }

        int nbPol = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedInteger(IS);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // remove LF
        // Kerf: untrusted input: a polygon takes at least 13 bytes
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbPol, 13, "3D polygons");
        try
        {
            using var ps = new ProgressScope(range, "Reading polygones 3D", nbPol);
            for (int polIter = 1; polIter <= nbPol && ps.More(); ++polIter, ps.Next())
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int nbNodes);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool hasParameters);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double defl);
                // Kerf: untrusted input
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbNodes, hasParameters ? 32 : 24, "nodes");
                Polygon3D poly = new(nbNodes, hasParameters);
                poly.Deflection(defl);

                NCollection_Array1<Point3D> nodes = poly.ChangeNodes();
                for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double x);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double y);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double z);
                    nodes[nodeIter] = new Point3D(x, y, z);
                }

                if (hasParameters)
                {
                    NCollection_Array1<double> param = poly.ChangeParameters();
                    for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out param[nodeIter]);
                    }
                }

                _polygons3D.Add(poly);
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeSet::ReadPolygon3D(..)", exception);
        }
    }

    /// <summary>Writes the triangulations ("Triangulations", the count, then each triangulation).</summary>
    public void WriteTriangulation(Stream os, ProgressRange range = default)
    {
        int nbTriangulations = _triangulations.Count;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "Triangulations ");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, nbTriangulations);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");

        try
        {
            using var ps = new ProgressScope(range, "Writing triangulation", nbTriangulations);
            for (int triangulationIter = 1; triangulationIter <= nbTriangulations && ps.More();
                 ++triangulationIter, ps.Next())
            {
                Triangulation triangulation = _triangulations.KeyAt(triangulationIter - 1);
                bool needToWriteNormals = _triangulations.ValueAt(triangulationIter - 1);
                int nbNodes = triangulation.NodeCount;
                int nbTriangles = triangulation.TriangleCount;
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, nbNodes);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, nbTriangles);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, triangulation.HasUVNodes);
                if (FormatNb() >= (int)BinaryFormatVersion.FormatVersion_VERSION_4)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutBool(os, triangulation.HasNormals && needToWriteNormals);
                }

                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, triangulation.Deflection());

                // write the 3d nodes
                for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                {
                    Point3D pnt = triangulation.Node(nodeIter);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pnt.X);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pnt.Y);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, pnt.Z);
                }

                if (triangulation.HasUVNodes)
                {
                    for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                    {
                        Point2D uv = triangulation.UVNode(nodeIter);
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, uv.X);
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, uv.Y);
                    }
                }

                for (int triIter = 1; triIter <= nbTriangles; ++triIter)
                {
                    Triangle1 tri = triangulation.Triangle(triIter);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, tri.Value(1));
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, tri.Value(2));
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutInteger(os, tri.Value(3));
                }

                // write the normals
                if (FormatNb() >= (int)BinaryFormatVersion.FormatVersion_VERSION_4)
                {
                    if (triangulation.HasNormals && needToWriteNormals)
                    {
                        for (int normalIter = 1; normalIter <= nbNodes; ++normalIter)
                        {
                            triangulation.Normal(normalIter, out NCollection_Vec3<float> normal);
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutShortReal(os, normal.X());
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutShortReal(os, normal.Y());
                            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutShortReal(os, normal.Z());
                        }
                    }
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeSet::WriteTriangulation(..)", exception);
        }
    }

    /// <summary>Reads the triangulations written by <see cref="WriteTriangulation"/>.</summary>
    public void ReadTriangulation(Stream IS, ProgressRange range = default)
    {
        string? header = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadWord(IS);
        if (header is null || !header.Contains("Triangulations", StringComparison.Ordinal))
        {
            throw new FailureException("BinTools_ShapeSet::Triangulation: Not a Triangulation section");
        }

        int nbTriangulations = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedInteger(IS);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // remove LF
        // Kerf: untrusted input: a triangulation takes at least 17 bytes
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbTriangulations, 17, "triangulations");

        try
        {
            using var ps = new ProgressScope(range, "Reading triangulation", nbTriangulations);
            for (int triangulationIter = 1; triangulationIter <= nbTriangulations && ps.More();
                 ++triangulationIter, ps.Next())
            {
                bool hasNormals = false;
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int nbNodes);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int nbTriangles);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool hasUV);
                if (FormatNb() >= (int)BinaryFormatVersion.FormatVersion_VERSION_4)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out hasNormals);
                }

                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double defl); // deflection
                // Kerf: untrusted input
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbNodes, 24 + (hasUV ? 16 : 0) + (hasNormals ? 12 : 0), "nodes");
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbTriangles, 12, "triangles");
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS,
                                    nbNodes * (24L + (hasUV ? 16 : 0) + (hasNormals ? 12 : 0)) + nbTriangles * 12L,
                                    1,
                                    "bytes of a triangulation");
                Triangulation triangulation = new(nbNodes, nbTriangles, hasUV, hasNormals);
                triangulation.Deflection(defl);

                for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double x);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double y);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double z);
                    triangulation.SetNode(nodeIter, new Point3D(x, y, z));
                }

                if (hasUV)
                {
                    for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double u);
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v);
                        triangulation.SetUVNode(nodeIter, new Point2D(u, v));
                    }
                }

                // read the triangles
                for (int triIter = 1; triIter <= nbTriangles; ++triIter)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int triNode0);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int triNode1);
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out int triNode2);
                    triangulation.SetTriangle(triIter, new Triangle1(triNode0, triNode1, triNode2));
                }

                if (hasNormals)
                {
                    for (int normalIter = 1; normalIter <= nbNodes; ++normalIter)
                    {
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetShortReal(IS, out float x);
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetShortReal(IS, out float y);
                        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetShortReal(IS, out float z);
                        triangulation.SetNormal(normalIter, new NCollection_Vec3<float>(x, y, z));
                    }
                }

                _triangulations.Add(triangulation, hasNormals);
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeSet::ReadTriangulation(..)", exception);
        }
    }
}
