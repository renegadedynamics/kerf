// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeWriter.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeWriter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// Writes shapes in OCCT's position-based binary format, which <see cref="ShapeReader"/> reads: each shape,
/// location, curve, surface, polygon and triangulation is written once, where it is first met, and referred to after
/// that by its offset back from the reference. OCCT uses it for shapes in binary documents. The writer remembers what
/// it has written across calls of <see cref="Write(in Shape, Stream)"/> until <see cref="Clear"/>.
/// </summary>
/// <remarks>The format has no version and no Locked bit: a shape's flags are Free to Convex, in one byte.</remarks>
public class ShapeWriter : ShapeSetBase
{
    // positions of the objects already stored
    private readonly Dictionary<Shape, ulong> _shapePos = new(ShapeHasher.Instance);
    private readonly Dictionary<Placement, ulong> _locationPos = new();
    private readonly Dictionary<Curve3D, ulong> _curvePos = new(ReferenceEqualityComparer.Instance);
    private readonly Dictionary<Curve2D, ulong> _curve2dPos = new(ReferenceEqualityComparer.Instance);
    private readonly Dictionary<Surface, ulong> _surfacePos = new(ReferenceEqualityComparer.Instance);
    private readonly Dictionary<Polygon3D, ulong> _polygon3dPos = new(ReferenceEqualityComparer.Instance);
    private readonly Dictionary<PolygonOnTriangulation, ulong> _polygonPos = new(ReferenceEqualityComparer.Instance);
    private readonly Dictionary<Triangulation, ulong> _triangulationPos = new(ReferenceEqualityComparer.Instance);

    /// <summary>A writer that has written nothing yet.</summary>
    public ShapeWriter()
    {
    }

    /// <summary>Forgets what has been written.</summary>
    public override void Clear()
    {
        base.Clear();
        _shapePos.Clear();
        _locationPos.Clear();
        _curvePos.Clear();
        _curve2dPos.Clear();
        _surfacePos.Clear();
        _polygon3dPos.Clear();
        _polygonPos.Clear();
        _triangulationPos.Clear();
    }

    /// <summary>Writes the shape, or a reference to it if it has been written already.</summary>
    public override void Write(in Shape shape, Stream stream)
    {
        var oStream = new OStream(stream);
        WriteShape(oStream, shape);
    }

    /// <summary>Writes the location, or a reference to it if it has been written already.</summary>
    public virtual void WriteLocation(OStream stream, in Placement location)
    {
        if (location.IsIdentity)
        {
            stream = stream << ObjectType.ObjectType_EmptyLocation;
            return;
        }

        if (_locationPos.TryGetValue(location, out ulong loc))
        {
            stream.WriteReference(loc);
            return;
        }

        ulong newLoc = stream.Position();
        try
        {
            Placement l2 = location.NextLocation();
            bool isSimple = l2.IsIdentity;
            int power = location.FirstPower();
            Placement l1 = new(location.FirstDatum());
            bool elementary = (isSimple && power == 1);
            if (elementary)
            {
                stream = stream << ObjectType.ObjectType_SimpleLocation << location.Transformation();
            }
            else
            {
                stream = stream << ObjectType.ObjectType_Location;
                WriteLocation(stream, l1);
                stream = stream << power;
                while (!l2.IsIdentity)
                {
                    l1 = new Placement(l2.FirstDatum());
                    power = l2.FirstPower();
                    l2 = l2.NextLocation();
                    WriteLocation(stream, l1);
                    stream = stream << power;
                }

                stream = stream << ObjectType.ObjectType_LocationEnd;
            }

            _locationPos[location] = newLoc;
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeWriter::WriteLocation", exception);
        }
    }

    // Writes the shape (type, orientation, location, geometry, flags, sub-shapes), or a reference to it
    private void WriteShape(OStream stream, in Shape shape)
    {
        // OCCT.NET: a shape nested too deeply for the stack throws InsufficientExecutionStackException
        System.Runtime.CompilerServices.RuntimeHelpers.EnsureSufficientExecutionStack();

        if (shape.IsNull)
        {
            stream = stream << ObjectType.ObjectType_EmptyShape;
            return;
        }

        Shape aShape = shape.Located(new Placement());
        if (_shapePos.TryGetValue(aShape, out ulong existing))
        {
            // shape is already there, so, write reference to it
            stream.WriteReference(existing);
            WriteLocation(stream, shape.Location());
            stream = stream << (byte)shape.Orientation();
            return;
        }

        ulong newPos = stream.Position();
        _shapePos[aShape] = newPos;
        stream.WriteShape(aShape.ShapeType(), aShape.Orientation());
        WriteLocation(stream, shape.Location());

        // OCCT.NET: the geometry is written by a separate method (a smaller frame for the recursion)
        WriteShapeGeometry(stream, aShape);

        stream.PutBools(aShape.Free(),
                           aShape.Modified(),
                           aShape.Checked(),
                           aShape.Orientable(),
                           aShape.Closed(),
                           aShape.Infinite(),
                           aShape.Convex());

        // process sub-shapes
        for (var sub = new Iterator(aShape, false, false); sub.More(); sub.Next())
        {
            WriteShape(stream, sub.Value());
        }

        stream = stream << ObjectType.ObjectType_EndShape;
    }

    // The geometry of a shape, apart from WriteShape so that its recursion keeps a small stack frame
    private void WriteShapeGeometry(OStream stream, in Shape aShape)
    {
        try
        {
            switch (aShape.ShapeType())
            {
                case ShapeType.Vertex:
                {
                    Vertex v = ShapeCasts.Vertex(aShape);
                    stream = stream << Tool.Tolerance(v);
                    Point3D p = Tool.Pnt(v);
                    stream = stream << p;
                    BrepTVertex tv = (BrepTVertex)aShape.TShape()!;
                    foreach (PointRepresentation pr in tv.Points())
                    {
                        if (pr.IsPointOnCurve())
                        {
                            stream = stream << (byte)1; // 1
                            stream = stream << pr.Parameter();
                            WriteCurve(stream, pr.Curve());
                        }
                        else if (pr.IsPointOnCurveOnSurface())
                        {
                            stream = stream << (byte)2; // 2
                            stream = stream << pr.Parameter();
                            WriteCurve(stream, pr.PCurve());
                            WriteSurface(stream, pr.Surface());
                        }
                        else if (pr.IsPointOnSurface())
                        {
                            // the second parameter first, as OCCT writes it (the readers take it so; Kerf decision 1500)
                            stream = stream << (byte)3; // 3
                            stream = stream << pr.Parameter2() << pr.Parameter();
                            WriteSurface(stream, pr.Surface());
                        }

                        WriteLocation(stream, pr.Location());
                    }

                    stream = stream << (byte)0;
                }
                    break;

                case ShapeType.Edge:
                {
                    BrepTEdge te = (BrepTEdge)aShape.TShape()!;
                    stream = stream << te.Tolerance();
                    stream.PutBools(te.SameParameter(), te.SameRange(), te.Degenerated());
                    double first, last;
                    foreach (CurveRepresentation cr in te.Curves())
                    {
                        if (cr.IsCurve3D)
                        {
                            if (cr.Curve3D() is not null)
                            {
                                GCurve aGC = (GCurve)cr;
                                aGC.Range(out first, out last);
                                stream = stream << (byte)1; // -1- CURVE_3D;
                                WriteCurve(stream, cr.Curve3D());
                                WriteLocation(stream, cr.Location());
                                stream = stream << first << last;
                            }
                        }
                        else if (cr.IsCurveOnSurface())
                        {
                            GCurve GC = (GCurve)cr;
                            GC.Range(out first, out last);
                            if (!cr.IsCurveOnClosedSurface)
                            {
                                stream = stream << (byte)2; // -2- Curve on surf
                            }
                            else
                            {
                                stream = stream << (byte)3; // -3- Curve on closed surf
                            }

                            WriteCurve(stream, cr.PCurve());
                            if (cr.IsCurveOnClosedSurface)
                            {
                                //+ int|char
                                WriteCurve(stream, cr.PCurve2());
                                stream = stream << (byte)cr.Continuity();
                            }

                            WriteSurface(stream, cr.Surface());
                            WriteLocation(stream, cr.Location());
                            stream = stream << first << last;
                        }
                        else if (cr.IsRegularity())
                        {
                            stream = stream << (byte)4; // -4- Regularity
                            stream = stream << (byte)cr.Continuity();
                            WriteSurface(stream, cr.Surface());
                            WriteLocation(stream, cr.Location());
                            WriteSurface(stream, cr.Surface2());
                            WriteLocation(stream, cr.Location2());
                        }
                        else if (IsWithTriangles)
                        {
                            if (cr.IsPolygon3D)
                            {
                                Polygon3DRepresentation gc = (Polygon3DRepresentation)cr;
                                if (gc.Polygon3D() is not null)
                                {
                                    stream = stream << (byte)5; // -5- Polygon3D
                                    WritePolygon(stream, cr.Polygon3D());
                                    WriteLocation(stream, cr.Location());
                                }
                            }
                            else if (cr.IsPolygonOnTriangulation())
                            {
                                PolygonOnTriangulationRepresentation pt = (PolygonOnTriangulationRepresentation)cr;
                                if (!cr.IsPolygonOnClosedTriangulation)
                                {
                                    stream = stream << (byte)6; // -6- Polygon on triangulation
                                }
                                else
                                {
                                    stream = stream << (byte)7; // -7- Polygon on closed triangulation
                                }

                                WritePolygon(stream, pt.PolygonOnTriangulation());
                                if (cr.IsPolygonOnClosedTriangulation)
                                {
                                    WritePolygon(stream, pt.PolygonOnTriangulation2());
                                }

                                // edge triangulation does not need normals
                                WriteTriangulation(stream, pt.Triangulation(), false);
                                WriteLocation(stream, cr.Location());
                            }
                        }
                    }

                    stream = stream << (byte)0;
                }
                    break;

                case ShapeType.Face:
                {
                    BrepTFace tf = (BrepTFace)aShape.TShape()!;
                    Face f = ShapeCasts.Face(aShape);

                    // Write the surface geometry
                    stream = stream << Tool.NaturalRestriction(f) << tf.Tolerance();
                    WriteSurface(stream, tf.Surface());
                    WriteLocation(stream, tf.Location());

                    if (IsWithTriangles || tf.Surface() is null)
                    {
                        if (tf.Triangulation() is not null)
                        {
                            stream = stream << (byte)2;
                            WriteTriangulation(stream,
                                               tf.Triangulation(),
                                               tf.Triangulation()!.HasNormals
                                                   && (IsWithNormals || tf.Surface() is null));
                        }
                        else
                        {
                            stream = stream << (byte)1;
                        }
                    }
                    else
                    {
                        stream = stream << (byte)0; // without triangulation
                    }
                }
                    break;

                default:
                {
                }
                    break;
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeWriter::WriteShape", exception);
        }
    }

    private void WriteCurve(OStream stream, Curve3D? curve)
    {
        if (curve is null)
        {
            stream = stream << ObjectType.ObjectType_EmptyCurve;
            return;
        }

        if (_curvePos.TryGetValue(curve, out ulong aCurve))
        {
            stream.WriteReference(aCurve);
            return;
        }

        _curvePos[curve] = stream.Position();
        stream = stream << ObjectType.ObjectType_Curve;
        BinaryCurveSet.WriteCurve(curve, stream);
    }

    private void WriteCurve(OStream stream, Curve2D? curve)
    {
        if (curve is null)
        {
            stream = stream << ObjectType.ObjectType_EmptyCurve2d;
            return;
        }

        if (_curve2dPos.TryGetValue(curve, out ulong aCurve))
        {
            stream.WriteReference(aCurve);
            return;
        }

        _curve2dPos[curve] = stream.Position();
        stream = stream << ObjectType.ObjectType_Curve2d;
        BinaryCurve2dSet.WriteCurve2d(curve, stream);
    }

    private void WriteSurface(OStream stream, Surface? surface)
    {
        if (surface is null)
        {
            stream = stream << ObjectType.ObjectType_EmptySurface;
            return;
        }

        if (_surfacePos.TryGetValue(surface, out ulong aSurface))
        {
            stream.WriteReference(aSurface);
            return;
        }

        _surfacePos[surface] = stream.Position();
        stream = stream << ObjectType.ObjectType_Surface;
        BinarySurfaceSet.WriteSurface(surface, stream);
    }

    private void WritePolygon(OStream stream, Polygon3D? polygon)
    {
        if (polygon is null)
        {
            stream = stream << ObjectType.ObjectType_EmptyPolygon3d;
            return;
        }

        if (_polygon3dPos.TryGetValue(polygon, out ulong aPolygon))
        {
            stream.WriteReference(aPolygon);
            return;
        }

        _polygon3dPos[polygon] = stream.Position();
        stream = stream << ObjectType.ObjectType_Polygon3d;

        int nbNodes = polygon.NodeCount;
        stream = stream << nbNodes << polygon.HasParameters << polygon.Deflection();
        NCollection_Array1<Point3D> nodes = polygon.Nodes();
        for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
        {
            stream = stream << nodes.Value(nodeIter);
        }

        if (polygon.HasParameters)
        {
            NCollection_Array1<double> param = polygon.Parameters();
            for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
            {
                stream = stream << param.Value(nodeIter);
            }
        }
    }

    private void WritePolygon(OStream stream, PolygonOnTriangulation? polygon)
    {
        if (polygon is null)
        {
            stream = stream << ObjectType.ObjectType_EmptyPolygonOnTriangulation;
            return;
        }

        if (_polygonPos.TryGetValue(polygon, out ulong aPolygon))
        {
            stream.WriteReference(aPolygon);
            return;
        }

        _polygonPos[polygon] = stream.Position();
        stream = stream << ObjectType.ObjectType_PolygonOnTriangulation;

        NCollection_Array1<int> nodes = polygon.Nodes();
        stream = stream << nodes.Length();
        for (int nodeIter = 1; nodeIter <= nodes.Length(); ++nodeIter)
        {
            stream = stream << nodes.Value(nodeIter);
        }

        stream = stream << polygon.Deflection();
        if (polygon.Parameters() is NCollection_HArray1<double> param)
        {
            stream = stream << true;
            for (int nodeIter = 1; nodeIter <= param.Length(); ++nodeIter)
            {
                stream = stream << param.Value(nodeIter);
            }
        }
        else
        {
            stream = stream << false;
        }
    }

    private void WriteTriangulation(OStream stream, Triangulation? triangulation, bool needToWriteNormals)
    {
        if (triangulation is null)
        {
            stream = stream << ObjectType.ObjectType_EmptyTriangulation;
            return;
        }

        if (_triangulationPos.TryGetValue(triangulation, out ulong aTriangulation))
        {
            stream.WriteReference(aTriangulation);
            return;
        }

        _triangulationPos[triangulation] = stream.Position();
        stream = stream << ObjectType.ObjectType_Triangulation;

        int nbNodes = triangulation.NodeCount;
        int nbTriangles = triangulation.TriangleCount;
        stream = stream << nbNodes << nbTriangles << triangulation.HasUVNodes;
        stream = stream << needToWriteNormals << triangulation.Deflection();

        // write the 3d nodes
        for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
        {
            stream = stream << triangulation.Node(nodeIter);
        }

        if (triangulation.HasUVNodes)
        {
            for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
            {
                stream = stream << triangulation.UVNode(nodeIter);
            }
        }

        for (int triIter = 1; triIter <= nbTriangles; ++triIter)
        {
            stream = stream << triangulation.Triangle(triIter);
        }

        if (needToWriteNormals)
        {
            for (int normalIter = 1; normalIter <= nbNodes; ++normalIter)
            {
                triangulation.Normal(normalIter, out NCollection_Vec3<float> normal);
                stream = stream << normal;
            }
        }
    }
}
