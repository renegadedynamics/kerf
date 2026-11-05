// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeReader.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeReader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// Reads shapes written by <see cref="ShapeWriter"/>: objects are read where they are first met, and
/// references to them are followed by seeking back in the stream, which must therefore be seekable. The reader
/// remembers what it has read across calls of <see cref="Read(Stream, ref Shape)"/> until <see cref="Clear"/>.
/// </summary>
/// <remarks>
/// The shapes read are frozen. On untrusted input the reader also checks that references point inside the stream and
/// don't form cycles (OCCT recurses forever), and caps the nesting of shapes, locations and references at
/// <see cref="BinTools.MaxShapeNesting"/> and of geometry at <see cref="BinTools.MaxGeometryNesting"/>; failures
/// throw <see cref="StreamFormatErrorException"/>.
/// </remarks>
internal class ShapeReader : ShapeSetBase
{
    private static readonly BrepBuilder ABuilder = new();

    // positions of the objects already read
    private readonly Dictionary<ulong, Shape> _shapePos = new();
    private readonly Dictionary<ulong, Placement> _locationPos = new();
    private readonly Dictionary<ulong, Curve3D?> _curvePos = new();
    private readonly Dictionary<ulong, Curve2D?> _curve2dPos = new();
    private readonly Dictionary<ulong, Surface?> _surfacePos = new();
    private readonly Dictionary<ulong, Polygon3D> _polygon3dPos = new();
    private readonly Dictionary<ulong, PolygonOnTriangulation> _polygonPos = new();
    private readonly Dictionary<ulong, Triangulation> _triangulationPos = new();

    // Kerf: untrusted input: the positions of the shapes and locations being read, and the depth of the recursion
    private readonly HashSet<ulong> _shapesInProgress = [];
    private readonly HashSet<ulong> _locationsInProgress = [];
    private int _depth;

    /// <summary>A reader that has read nothing yet.</summary>
    public ShapeReader()
    {
    }

    /// <summary>Forgets what has been read.</summary>
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
        _shapesInProgress.Clear();
        _locationsInProgress.Clear();
        _depth = 0;
    }

    /// <summary>Reads a shape from the stream, which must be seekable. The shape is frozen.</summary>
    /// <exception cref="FailureException">The stream doesn't hold a shape OCCT can read; for input that can't be read
    /// safely, a <see cref="StreamFormatErrorException"/>.</exception>
    public override void Read(Stream stream, ref Shape shape)
    {
        var aStream = new IStream(stream);
        try
        {
            shape = ReadShape(aStream);
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsSignal(exception))
        {
            // Kerf: untrusted input: a malformed stream raised an exception that OCCT doesn't catch here
            throw new StreamFormatErrorException("BinTools_ShapeReader::Read: " + Refusal.Unwrapped(exception).Message, exception);
        }
        finally
        {
            _shapesInProgress.Clear();
            _locationsInProgress.Clear();
            _depth = 0;
        }

        // design 4.2: a shape read is frozen
        if (!shape.IsNull)
        {
            shape.TShape()!.FreezeGraph();
        }
    }

    // Kerf: untrusted input: bounds the recursion through nested shapes, locations and references
    private void Enter()
    {
        if (!System.Runtime.CompilerServices.RuntimeHelpers.TryEnsureSufficientExecutionStack())
        {
            throw new StreamFormatErrorException("BinTools_ShapeReader: objects nested too deeply for the stack");
        }

        if (++_depth > global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxShapeNesting)
        {
            throw new StreamFormatErrorException("BinTools_ShapeReader: objects nested too deeply");
        }
    }

    private Shape ReadShape(IStream stream)
    {
        Enter();
        try
        {
            return ReadShapeAt(stream);
        }
        finally
        {
            _depth--;
        }
    }

    private Shape ReadShapeAt(IStream stream)
    {
        Shape result = new();
        ulong position = stream.Position();
        ObjectType type = stream.ReadType();
        if (type == ObjectType.ObjectType_EmptyShape
            || type == ObjectType.ObjectType_EndShape)
        {
            return result;
        }

        if (stream.IsReference)
        {
            ulong aRef = stream.ReadReference();
            if (_shapePos.TryGetValue(aRef, out Shape found))
            {
                // the shape is already retrieved, just add location
                result = found;
            }
            else
            {
                if (_shapesInProgress.Contains(aRef))
                {
                    // Kerf: untrusted input: OCCT reads the shape again, forever
                    throw new StreamFormatErrorException("BinTools_ShapeReader: a shape contains itself");
                }

                ulong current = stream.Position();
                stream.GoTo(aRef); // go to the referenced position
                result = ReadShape(stream);
                stream.GoTo(current); // returns to the current position
            }

            result.Location(ReadLocation(stream), false);
            // Kerf: untrusted input: OCCT casts any byte
            result.Orient(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckOrientation(stream.ReadByte()));
            return result;
        }

        // read the shape
        ShapeType shapeType = stream.ShapeType();
        Orientation shapeOrientation = stream.ShapeOrientation();
        Placement shapeLocation = ReadLocation(stream);
        _shapesInProgress.Add(position);
        // Kerf: the geometry is read by a separate method (a smaller frame for the recursion)
        result = ReadShapeGeometry(stream, shapeType);

        // read flags and subs
        stream.ReadBools(out bool free,
                            out bool mod,
                            out bool aChecked,
                            out bool orient,
                            out bool closedFlag,
                            out bool inf,
                            out bool conv);

        // sub-shapes
        for (Shape sub = ReadShape(stream); !sub.IsNull; sub = ReadShape(stream))
        {
            ABuilder.Add(result, sub);
        }

        result.Free(free);
        result.Modified(mod);
        result.Checked(aChecked);
        result.Orientable(orient);
        result.Closed(closedFlag);
        result.Infinite(inf);
        result.Convex(conv);
        _shapePos[position] = result;
        _shapesInProgress.Remove(position);
        result.Orient(shapeOrientation);
        result.Location(shapeLocation, false);
        return result;
    }

    // The geometry of a shape, apart from ReadShapeAt so that its recursion keeps a small stack frame
    private Shape ReadShapeGeometry(IStream stream, ShapeType aShapeType)
    {
        Shape result = new();
        double tol;
        try
        {
            switch (aShapeType)
            {
                case ShapeType.Vertex:
                {
                    Vertex v = new();
                    // Read the point geometry
                    tol = stream.ReadReal();
                    Point3D pnt = stream.ReadPnt();
                    ABuilder.MakeVertex(ref v, pnt, tol);
                    result = v;
                    BrepTVertex tv = (BrepTVertex)v.TShape()!;
                    NCollection_List<PointRepresentation> lpr = tv.ChangePoints();
                    Placement emptyLoc = new();
                    while (stream)
                    {
                        byte prsType = stream.ReadByte();
                        if (prsType == 0)
                        {
                            // end of the cycle
                            break;
                        }

                        double param = stream.ReadReal();
                        PointRepresentation? pr = null;
                        switch (prsType)
                        {
                            case 1:
                            {
                                Curve3D? curve = ReadCurve(stream);
                                if (curve is not null)
                                {
                                    pr = new PointOnCurve(param, curve, emptyLoc);
                                }
                            }
                                break;

                            case 2:
                            {
                                Curve2D? curve2d = ReadCurve2d(stream);
                                Surface? surface = ReadSurface(stream);
                                // OCCT: tests that the surface is null, not that it isn't, so a point on a curve on a
                                // surface is read only when its surface is missing (occt-issues.md)
                                if (curve2d is not null && surface is null)
                                {
                                    pr = new PointOnCurveOnSurface(param, curve2d, surface, emptyLoc);
                                }
                            }
                                break;

                            case 3:
                            {
                                double param2 = stream.ReadReal();
                                Surface? surface = ReadSurface(stream);
                                if (surface is not null)
                                {
                                    // the second parameter is written first (see BinTools_ShapeWriter; OCCT read it as the
                                    // first)
                                    pr = new PointOnSurface(param2, param, surface, emptyLoc);
                                }
                            }
                                break;

                            default:
                            {
                                throw new FailureException("BinTools_ShapeReader::Read: UnExpected BRep_PointRepresentation = "
                                                           + prsType + "\n");
                            }
                        }

                        Placement prLoc = ReadLocation(stream);
                        if (pr is not null)
                        {
                            pr.Location(prLoc);
                            lpr.Append(pr);
                        }
                    }
                }
                    break;

                case ShapeType.Edge:
                {
                    Edge e = new();
                    ABuilder.MakeEdge(ref e);
                    result = e;

                    // Read the curve geometry
                    tol = stream.ReadReal();
                    stream.ReadBools(out bool sameParameter, out bool sameRange, out bool degenerated);
                    ABuilder.SameParameter(e, sameParameter);
                    ABuilder.SameRange(e, sameRange);
                    ABuilder.Degenerated(e, degenerated);
                    double first, last;
                    while (stream)
                    {
                        byte prsType = stream.ReadByte(); //{0|1|2|3|4|5|6|7}
                        if (prsType == 0)
                        {
                            break;
                        }

                        switch (prsType)
                        {
                            case 1: // -1- Curve 3D
                            {
                                Curve3D? curve = ReadCurve(stream);
                                Placement loc = ReadLocation(stream);
                                first = stream.ReadReal();
                                last = stream.ReadReal();
                                if (curve is not null)
                                {
                                    ABuilder.UpdateEdge(e, curve, loc, tol);
                                    ABuilder.Range(e, first, last, true);
                                }
                            }
                                break;

                            case 2: // -2- Curve on surf
                            case 3: // -3- Curve on closed surf
                            {
                                bool closed = (prsType == 3);
                                Curve2D? curve2d_2 = null, curve2d_1 = ReadCurve2d(stream);
                                Continuity reg = Continuity.C0;
                                if (closed)
                                {
                                    curve2d_2 = ReadCurve2d(stream);
                                    reg = (Continuity)stream.ReadByte();
                                }

                                Surface? surface = ReadSurface(stream);
                                Placement loc = ReadLocation(stream);

                                // range
                                first = stream.ReadReal();
                                last = stream.ReadReal();
                                if (curve2d_1 is not null && (!closed || curve2d_2 is not null) && surface is not null)
                                {
                                    if (closed)
                                    {
                                        ABuilder.UpdateEdge(e, curve2d_1, curve2d_2, surface, loc, tol);
                                        ABuilder.Continuity(e, surface, surface, loc, loc, reg);
                                    }
                                    else
                                    {
                                        ABuilder.UpdateEdge(e, curve2d_1, surface, loc, tol);
                                    }

                                    ABuilder.Range(e, surface, loc, first, last);
                                }
                            }
                                break;

                            case 4: // -4- Regularity
                            {
                                Continuity reg = (Continuity)stream.ReadByte();
                                Surface? surface1 = ReadSurface(stream);
                                Placement loc1 = ReadLocation(stream);
                                Surface? surface2 = ReadSurface(stream);
                                Placement loc2 = ReadLocation(stream);
                                if (surface1 is not null && surface2 is not null)
                                {
                                    ABuilder.Continuity(e, surface1, surface2, loc1, loc2, reg);
                                }
                            }
                                break;

                            case 5: // -5- Polygon3D
                            {
                                Polygon3D? polygon = ReadPolygon3d(stream);
                                Placement loc = ReadLocation(stream);
                                ABuilder.UpdateEdge(e, polygon, loc);
                            }
                                break;

                            case 6: // -6- Polygon on triangulation
                            case 7: // -7- Polygon on closed triangulation
                            {
                                bool closed = (prsType == 7);
                                PolygonOnTriangulation? poly2 = null, poly1 = ReadPolygon(stream);
                                if (closed)
                                {
                                    poly2 = ReadPolygon(stream);
                                }

                                Triangulation? triangulation = ReadTriangulation(stream);
                                Placement loc = ReadLocation(stream);
                                if (closed)
                                {
                                    ABuilder.UpdateEdge(e, poly1, poly2, triangulation, loc);
                                }
                                else
                                {
                                    ABuilder.UpdateEdge(e, poly1, triangulation, loc);
                                }

                                // range
                            }
                                break;

                            default:
                            {
                                throw new FailureException("Unexpected Curve Representation =" + prsType + "\n");
                            }
                        }
                    }
                }
                    break;

                case ShapeType.Wire:
                {
                    Wire w = new();
                    ABuilder.MakeWire(ref w);
                    result = w;
                }
                    break;

                case ShapeType.Face:
                {
                    Face f = new();
                    ABuilder.MakeFace(ref f);
                    result = f;
                    bool natRes = stream.ReadBool();
                    tol = stream.ReadReal();
                    Surface? surface = ReadSurface(stream);
                    Placement loc = ReadLocation(stream);
                    ABuilder.UpdateFace(f, surface, loc, tol);
                    ABuilder.NaturalRestriction(f, natRes);
                    if (stream.ReadByte() == 2)
                    {
                        // triangulation
                        ABuilder.UpdateFace(f, ReadTriangulation(stream));
                    }
                }
                    break;

                case ShapeType.Shell:
                {
                    Shell sh = new();
                    ABuilder.MakeShell(ref sh);
                    result = sh;
                }
                    break;

                case ShapeType.Solid:
                {
                    Solid so = new();
                    ABuilder.MakeSolid(ref so);
                    result = so;
                }
                    break;

                case ShapeType.CompSolid:
                {
                    CompSolid cs = new();
                    ABuilder.MakeCompSolid(ref cs);
                    result = cs;
                }
                    break;

                case ShapeType.Compound:
                {
                    Compound c = new();
                    ABuilder.MakeCompound(ref c);
                    result = c;
                }
                    break;

                default:
                {
                    throw new FailureException("Unexpected topology type = " + (int)aShapeType + "\n");
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_ShapeReader::Read", exception);
        }

        return result;
    }

    /// <summary>Reads a location written by <see cref="ShapeWriter.WriteLocation"/>, or a reference to one.</summary>
    public Placement ReadLocation(IStream stream)
    {
        Enter();
        try
        {
            return ReadLocationAt(stream);
        }
        finally
        {
            _depth--;
        }
    }

    private Placement ReadLocationAt(IStream stream)
    {
        ulong position = stream.Position();
        ObjectType type = stream.ReadType();
        if (type == ObjectType.ObjectType_EmptyLocation
            || type == ObjectType.ObjectType_LocationEnd)
        {
            return new Placement();
        }

        if (stream.IsReference)
        {
            ulong aRef = stream.ReadReference();
            if (_locationPos.TryGetValue(aRef, out Placement found))
            {
                // the location is already retrieved
                return found;
            }

            if (_locationsInProgress.Contains(aRef))
            {
                // Kerf: untrusted input: OCCT reads the location again, forever
                throw new StreamFormatErrorException("BinTools_ShapeReader: a location contains itself");
            }

            ulong current = stream.Position();
            stream.GoTo(aRef); // go to the referenced position
            Placement result = ReadLocation(stream);
            stream.GoTo(current); // returns to the current position
            return result;
        }

        // read the location directly from the stream
        Placement loc = new();
        if (type == ObjectType.ObjectType_SimpleLocation)
        {
            Transform3D trsf = stream.ReadTrsf();
            loc = new Placement(trsf);
        }
        else if (type == ObjectType.ObjectType_Location)
        {
            _locationsInProgress.Add(position);
            for (Placement nextLoc = ReadLocation(stream); !nextLoc.IsIdentity;
                 nextLoc = ReadLocation(stream))
            {
                // Kerf: untrusted input: bounds the recursion of Powered and Multiplied
                loc = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PoweredTimes(nextLoc, stream.ReadInteger(), loc);
            }

            _locationsInProgress.Remove(position);
        }

        _locationPos[position] = loc;
        return loc;
    }

    private Curve3D? ReadCurve(IStream stream)
    {
        Enter();
        try
        {
            Curve3D? result = null;
            ulong position = stream.Position();
            stream.ReadType();
            if (stream.IsReference)
            {
                // get by reference
                ulong aRef = stream.ReadReference();
                if (_curvePos.TryGetValue(aRef, out Curve3D? found))
                {
                    // the curve is already retrieved
                    return found;
                }

                ulong current = stream.Position();
                stream.GoTo(aRef); // go to the referenced position
                result = ReadCurve(stream);
                stream.GoTo(current); // returns to the current position
            }
            else if (stream.LastType() == ObjectType.ObjectType_Curve)
            {
                // read from the stream
                BinaryCurveSet.ReadCurve(stream.Stream(), out result);
                stream.UpdatePosition();
                _curvePos[position] = result;
            }

            return result;
        }
        finally
        {
            _depth--;
        }
    }

    private Curve2D? ReadCurve2d(IStream stream)
    {
        Enter();
        try
        {
            Curve2D? result = null;
            ulong position = stream.Position();
            stream.ReadType();
            if (stream.IsReference)
            {
                // get by reference
                ulong aRef = stream.ReadReference();
                if (_curve2dPos.TryGetValue(aRef, out Curve2D? found))
                {
                    // the curve is already retrieved
                    return found;
                }

                ulong current = stream.Position();
                stream.GoTo(aRef); // go to the referenced position
                result = ReadCurve2d(stream);
                stream.GoTo(current); // returns to the current position
            }
            else if (stream.LastType() == ObjectType.ObjectType_Curve2d)
            {
                // read from the stream
                BinaryCurve2dSet.ReadCurve2d(stream.Stream(), out result);
                stream.UpdatePosition();
                _curve2dPos[position] = result;
            }

            return result;
        }
        finally
        {
            _depth--;
        }
    }

    private Surface? ReadSurface(IStream stream)
    {
        Enter();
        try
        {
            Surface? result = null;
            ulong position = stream.Position();
            stream.ReadType();
            if (stream.IsReference)
            {
                // get by reference
                ulong aRef = stream.ReadReference();
                if (_surfacePos.TryGetValue(aRef, out Surface? found))
                {
                    // the surface is already retrieved
                    return found;
                }

                ulong current = stream.Position();
                stream.GoTo(aRef); // go to the referenced position
                result = ReadSurface(stream);
                stream.GoTo(current); // returns to the current position
            }
            else if (stream.LastType() == ObjectType.ObjectType_Surface)
            {
                // read from the stream
                BinarySurfaceSet.ReadSurface(stream.Stream(), out result);
                stream.UpdatePosition();
                _surfacePos[position] = result;
            }

            return result;
        }
        finally
        {
            _depth--;
        }
    }

    private Polygon3D? ReadPolygon3d(IStream stream)
    {
        Enter();
        try
        {
            Polygon3D? result = null;
            ulong position = stream.Position();
            stream.ReadType();
            if (stream.IsReference)
            {
                // get by reference
                ulong aRef = stream.ReadReference();
                if (_polygon3dPos.TryGetValue(aRef, out Polygon3D? found))
                {
                    // the polygon is already retrieved
                    return found;
                }

                ulong current = stream.Position();
                stream.GoTo(aRef); // go to the referenced position
                result = ReadPolygon3d(stream);
                stream.GoTo(current); // returns to the current position
            }
            else if (stream.LastType() == ObjectType.ObjectType_Polygon3d)
            {
                // read from the stream
                int nbNodes = stream.ReadInteger();
                bool hasParameters = stream.ReadBool();
                // Kerf: untrusted input
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(stream.Stream(), nbNodes, hasParameters ? 32 : 24, "nodes");
                result = new Polygon3D(nbNodes, hasParameters);
                result.Deflection(stream.ReadReal());
                NCollection_Array1<Point3D> nodes = result.ChangeNodes();
                for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                {
                    nodes[nodeIter] = stream.ReadPnt();
                }

                if (hasParameters)
                {
                    NCollection_Array1<double> param = result.ChangeParameters();
                    for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                    {
                        param[nodeIter] = stream.ReadReal();
                    }
                }

                _polygon3dPos[position] = result;
            }

            return result;
        }
        finally
        {
            _depth--;
        }
    }

    private PolygonOnTriangulation? ReadPolygon(IStream stream)
    {
        Enter();
        try
        {
            PolygonOnTriangulation? result = null;
            ulong position = stream.Position();
            stream.ReadType();
            if (stream.IsReference)
            {
                // get by reference
                ulong aRef = stream.ReadReference();
                if (_polygonPos.TryGetValue(aRef, out PolygonOnTriangulation? found))
                {
                    // the polygon is already retrieved
                    return found;
                }

                ulong current = stream.Position();
                stream.GoTo(aRef); // go to the referenced position
                result = ReadPolygon(stream);
                stream.GoTo(current); // returns to the current position
            }
            else if (stream.LastType() == ObjectType.ObjectType_PolygonOnTriangulation)
            {
                // read from the stream
                int nbNodes = stream.ReadInteger();
                // Kerf: untrusted input
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(stream.Stream(), nbNodes, 4, "nodes");
                result = new PolygonOnTriangulation(nbNodes, false);
                for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                {
                    result.SetNode(nodeIter, stream.ReadInteger());
                }

                result.Deflection(stream.ReadReal());
                if (stream.ReadBool())
                {
                    // Kerf: untrusted input
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(stream.Stream(), nbNodes, 8, "parameters");
                    NCollection_HArray1<double> aParams = new(1, nbNodes);
                    for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                    {
                        aParams.ChangeValue(nodeIter) = stream.ReadReal();
                    }

                    result.SetParameters(aParams);
                }

                _polygonPos[position] = result;
            }

            return result;
        }
        finally
        {
            _depth--;
        }
    }

    private Triangulation? ReadTriangulation(IStream stream)
    {
        Enter();
        try
        {
            Triangulation? result = null;
            ulong position = stream.Position();
            ObjectType type = stream.ReadType();
            if (stream.IsReference)
            {
                // get by reference
                ulong aRef = stream.ReadReference();
                if (_triangulationPos.TryGetValue(aRef, out Triangulation? found))
                {
                    // the triangulation is already retrieved
                    return found;
                }

                ulong current = stream.Position();
                stream.GoTo(aRef); // go to the referenced position
                result = ReadTriangulation(stream);
                stream.GoTo(current); // returns to the current position
            }
            else if (type == ObjectType.ObjectType_Triangulation)
            {
                // read from the stream
                int nbNodes = stream.ReadInteger();
                int nbTriangles = stream.ReadInteger();
                bool hasUV = stream.ReadBool();
                bool hasNormals = stream.ReadBool();
                // Kerf: untrusted input
                int nodeBytes = 24 + (hasUV ? 16 : 0) + (hasNormals ? 12 : 0);
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(stream.Stream(), nbNodes, nodeBytes, "nodes");
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(stream.Stream(), nbTriangles, 12, "triangles");
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(stream.Stream(),
                                    (long)nbNodes * nodeBytes + nbTriangles * 12L,
                                    1,
                                    "bytes of a triangulation");
                result = new Triangulation(nbNodes, nbTriangles, hasUV, hasNormals);
                result.Deflection(stream.ReadReal());
                for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                {
                    result.SetNode(nodeIter, stream.ReadPnt());
                }

                if (hasUV)
                {
                    for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                    {
                        double u = stream.ReadReal();
                        double v = stream.ReadReal();
                        result.SetUVNode(nodeIter, new Point2D(u, v));
                    }
                }

                // read the triangles
                for (int triIter = 1; triIter <= nbTriangles; ++triIter)
                {
                    int n1 = stream.ReadInteger();
                    int n2 = stream.ReadInteger();
                    int n3 = stream.ReadInteger();
                    result.SetTriangle(triIter, new Triangle1(n1, n2, n3));
                }

                if (hasNormals)
                {
                    for (int normalIter = 1; normalIter <= nbNodes; ++normalIter)
                    {
                        float x = stream.ReadShortReal();
                        float y = stream.ReadShortReal();
                        float z = stream.ReadShortReal();
                        result.SetNormal(normalIter, new NCollection_Vec3<float>(x, y, z));
                    }
                }

                _triangulationPos[position] = result;
            }

            return result;
        }
        finally
        {
            _depth--;
        }
    }
}
