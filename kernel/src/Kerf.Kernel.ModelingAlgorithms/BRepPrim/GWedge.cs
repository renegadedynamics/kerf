// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_GWedge.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_GWedge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Construction.Primitives.PrimitiveDirection;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>
/// A wedge: a box, or a box whose YMax face is a smaller rectangle (Z2Min..Z2Max, X2Min..X2Max), possibly a segment
/// or a point. Each face of the wedge can be removed (Open) to make it infinite in that direction. Building an element
/// builds all its sub-elements, so building the shell builds everything.
/// </summary>
/// <remarks>Internal (decision 465): its shapes are unfrozen; BRepPrimAPI_MakeBox and MakeWedge publish them.</remarks>
internal class GWedge
{
    private const int Nbfaces = 6;
    private const int Nbwires = 6;
    private const int Nbedges = 12;
    private const int Nbvertices = 8;

    private static readonly int[] Num = [0, 1, 2, 3, 4, 5];
    private static readonly int[] Val = [0, 4, 0, 2, 0, 1];
    private static readonly int[,] Tab = { { -1, -1, 0, 1, 8, 9 },
                                           { -1, -1, 2, 3, 10, 11 },
                                           { 0, 2, -1, -1, 4, 5 },
                                           { 1, 3, -1, -1, 6, 7 },
                                           { 8, 10, 4, 6, -1, -1 },
                                           { 9, 11, 5, 7, -1, -1 } };

    // when giving a direction return the range of the face
    private static int BRepPrim_Wedge_NumDir1(PrimitiveDirection d1)
    {
        return Num[(int)d1];
    }

    // when giving two directions return the range of the edge
    private static int BRepPrim_Wedge_NumDir2(PrimitiveDirection d1, PrimitiveDirection d2)
    {
        int i1 = BRepPrim_Wedge_NumDir1(d1);
        int i2 = BRepPrim_Wedge_NumDir1(d2);
        if (i1 / 2 == i2 / 2)
        {
            throw new DomainErrorException();
        }
        return Tab[i1, i2];
    }

    // when giving three directions return the range of the vertex
    private static int BRepPrim_Wedge_NumDir3(PrimitiveDirection d1, PrimitiveDirection d2, PrimitiveDirection d3)
    {
        int i1 = BRepPrim_Wedge_NumDir1(d1);
        int i2 = BRepPrim_Wedge_NumDir1(d2);
        int i3 = BRepPrim_Wedge_NumDir1(d3);
        if ((i1 / 2 == i2 / 2) || (i2 / 2 == i3 / 2) || (i3 / 2 == i1 / 2))
        {
            throw new DomainErrorException();
        }
        return Val[i1] + Val[i2] + Val[i3];
    }

    // raise Standard_DomainError if something was built
    private static void BRepPrim_Wedge_Check(bool[] v, bool[] e, bool[] w, bool[] f)
    {
        int i;
        for (i = 0; i < Nbvertices; i++)
        {
            if (v[i])
            {
                throw new DomainErrorException();
            }
        }
        for (i = 0; i < Nbedges; i++)
        {
            if (e[i])
            {
                throw new DomainErrorException();
            }
        }
        for (i = 0; i < Nbwires; i++)
        {
            if (w[i])
            {
                throw new DomainErrorException();
            }
        }
        for (i = 0; i < Nbfaces; i++)
        {
            if (f[i])
            {
                throw new DomainErrorException();
            }
        }
    }

    private static void BRepPrim_Wedge_Init(out bool s, bool[] v, bool[] e, bool[] w, bool[] f)
    {
        int i;
        s = false;
        for (i = 0; i < Nbvertices; i++)
        {
            v[i] = false;
        }
        for (i = 0; i < Nbedges; i++)
        {
            e[i] = false;
        }
        for (i = 0; i < Nbwires; i++)
        {
            w[i] = false;
        }
        for (i = 0; i < Nbfaces; i++)
        {
            f[i] = false;
        }
    }

    private readonly PrimitiveBuilder _builder;
    private readonly Frame3D _axes;
    private readonly double _xMin;
    private readonly double _xMax;
    private readonly double _yMin;
    private readonly double _yMax;
    private readonly double _zMin;
    private readonly double _zMax;
    private readonly double _z2Min;
    private readonly double _z2Max;
    private readonly double _x2Min;
    private readonly double _x2Max;
    private Shell _shell = new();
    private bool _shellBuilt;
    private readonly Vertex[] _vertices = NewArray<Vertex>(Nbvertices);
    private readonly bool[] _verticesBuilt = new bool[Nbvertices];
    private readonly Edge[] _edges = NewArray<Edge>(Nbedges);
    private readonly bool[] _edgesBuilt = new bool[Nbedges];
    private readonly Wire[] _wires = NewArray<Wire>(Nbwires);
    private readonly bool[] _wiresBuilt = new bool[Nbwires];
    private readonly Face[] _faces = NewArray<Face>(Nbfaces);
    private readonly bool[] _facesBuilt = new bool[Nbfaces];
    private readonly bool[] _infinite = new bool[Nbfaces];

    // Arrays of null shapes, as C++ default-constructs its arrays of shapes
    internal static T[] NewArray<T>(int size) where T : struct
    {
        var array = new T[size];
        for (int i = 0; i < size; i++)
        {
            array[i] = new T();
        }
        return array;
    }

    public GWedge()
    {
        _builder = new PrimitiveBuilder();
        _axes = new Frame3D();
        _xMin = 0;
        _xMax = 0;
        _yMin = 0;
        _yMax = 0;
        _zMin = 0;
        _zMax = 0;
        _z2Min = 0;
        _z2Max = 0;
        _x2Min = 0;
        _x2Max = 0;
        for (int i = 0; i < Nbfaces; i++)
        {
            _infinite[i] = false;
        }

        BRepPrim_Wedge_Init(out _shellBuilt, _verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
    }

    /// <summary>A box: XMin, YMin, ZMin at 0, XMax, YMax, ZMax at dx, dy, dz, Z2 = Z and X2 = X.</summary>
    public GWedge(PrimitiveBuilder b, in Frame3D axes, double dx, double dy, double dz)
    {
        _builder = b;
        _axes = axes;
        _xMin = 0;
        _xMax = dx;
        _yMin = 0;
        _yMax = dy;
        _zMin = 0;
        _zMax = dz;
        _z2Min = 0;
        _z2Max = dz;
        _x2Min = 0;
        _x2Max = dx;
        for (int i = 0; i < Nbfaces; i++)
        {
            _infinite[i] = false;
        }

        BRepPrim_Wedge_Init(out _shellBuilt, _verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
    }

    /// <summary>A STEP right angular wedge: as the box, with X2Min = 0 and X2Max = ltx.</summary>
    public GWedge(PrimitiveBuilder b, in Frame3D axes, double dx, double dy, double dz, double ltx)
    {
        _builder = b;
        _axes = axes;
        _xMin = 0;
        _xMax = dx;
        _yMin = 0;
        _yMax = dy;
        _zMin = 0;
        _zMax = dz;
        _z2Min = 0;
        _z2Max = dz;
        _x2Min = 0;
        _x2Max = ltx;
        for (int i = 0; i < Nbfaces; i++)
        {
            _infinite[i] = false;
        }

        BRepPrim_Wedge_Init(out _shellBuilt, _verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
    }

    /// <summary>Builds a wedge by giving all the fields.</summary>
    public GWedge(PrimitiveBuilder b,
                           in Frame3D axes,
                           double xmin,
                           double ymin,
                           double zmin,
                           double z2min,
                           double x2min,
                           double xmax,
                           double ymax,
                           double zmax,
                           double z2max,
                           double x2max)
    {
        _builder = b;
        _axes = axes;
        _xMin = xmin;
        _xMax = xmax;
        _yMin = ymin;
        _yMax = ymax;
        _zMin = zmin;
        _zMax = zmax;
        _z2Min = z2min;
        _z2Max = z2max;
        _x2Min = x2min;
        _x2Max = x2max;
        for (int i = 0; i < Nbfaces; i++)
        {
            _infinite[i] = false;
        }

        BRepPrim_Wedge_Init(out _shellBuilt, _verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
    }

    public Frame3D Axes()
    {
        return _axes;
    }

    public double GetXMin()
    {
        return _xMin;
    }

    public double GetYMin()
    {
        return _yMin;
    }

    public double GetZMin()
    {
        return _zMin;
    }

    public double GetZ2Min()
    {
        return _z2Min;
    }

    public double GetX2Min()
    {
        return _x2Min;
    }

    public double GetXMax()
    {
        return _xMax;
    }

    public double GetYMax()
    {
        return _yMax;
    }

    public double GetZMax()
    {
        return _zMax;
    }

    public double GetZ2Max()
    {
        return _z2Max;
    }

    public double GetX2Max()
    {
        return _x2Max;
    }

    /// <summary>Opens the wedge in the direction (removes the face). Throws if anything was built.</summary>
    public void Open(PrimitiveDirection d1)
    {
        BRepPrim_Wedge_Check(_verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
        _infinite[BRepPrim_Wedge_NumDir1(d1)] = true;
    }

    /// <summary>Closes the wedge in the direction. Throws if anything was built.</summary>
    public void Close(PrimitiveDirection d1)
    {
        BRepPrim_Wedge_Check(_verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
        _infinite[BRepPrim_Wedge_NumDir1(d1)] = false;
    }

    /// <summary>True if it is open in the given direction.</summary>
    public bool IsInfinite(PrimitiveDirection d1)
    {
        return _infinite[BRepPrim_Wedge_NumDir1(d1)];
    }

    /// <summary>The shell of the wedge (unfrozen).</summary>
    public Shell Shell()
    {
        if (IsDegeneratedShape)
        {
            throw new DomainErrorException();
        }

        if (!_shellBuilt)
        {
            _builder.MakeShell(ref _shell);

            if (HasFace(PrimitiveDirection.BRepPrim_XMin))
            {
                _builder.AddShellFace(_shell, Face(PrimitiveDirection.BRepPrim_XMin));
            }
            if (HasFace(PrimitiveDirection.BRepPrim_XMax))
            {
                _builder.AddShellFace(_shell, Face(PrimitiveDirection.BRepPrim_XMax));
            }
            if (HasFace(PrimitiveDirection.BRepPrim_YMin))
            {
                _builder.AddShellFace(_shell, Face(PrimitiveDirection.BRepPrim_YMin));
            }
            if (HasFace(PrimitiveDirection.BRepPrim_YMax))
            {
                _builder.AddShellFace(_shell, Face(PrimitiveDirection.BRepPrim_YMax));
            }
            if (HasFace(PrimitiveDirection.BRepPrim_ZMin))
            {
                _builder.AddShellFace(_shell, Face(PrimitiveDirection.BRepPrim_ZMin));
            }
            if (HasFace(PrimitiveDirection.BRepPrim_ZMax))
            {
                _builder.AddShellFace(_shell, Face(PrimitiveDirection.BRepPrim_ZMax));
            }

            _shell.Closed(Tool.IsClosed(_shell));
            _builder.CompleteShell(_shell);
            _shellBuilt = true;
        }
        return _shell;
    }

    /// <summary>True if the face exists in the direction.</summary>
    public bool HasFace(PrimitiveDirection d1)
    {
        bool state = !_infinite[BRepPrim_Wedge_NumDir1(d1)];
        if (d1 == PrimitiveDirection.BRepPrim_YMax)
        {
            state = state && (_z2Max != _z2Min) && (_x2Max != _x2Min);
        }
        return state;
    }

    /// <summary>The plane of the face in the direction.</summary>
    public Plane3D Plane(PrimitiveDirection d1)
    {
        int i = BRepPrim_Wedge_NumDir1(d1);

        Direction3D d = new();
        Vector3D vx = new(_axes.XDirection);
        Vector3D vy = new(_axes.YDirection);
        Vector3D vz = new(_axes.Direction);

        switch (i / 2)
        {
            case 0:
                d = _axes.XDirection;
                break;

            case 1:
                d = _axes.YDirection;
                break;

            case 2:
                d = _axes.Direction;
                break;
        }
        double x = 0.0, y = 0.0, z = 0.0;

        switch (i)
        {
            case 0:
                // XMin
                x = _xMin;
                y = _yMin;
                z = _zMin;
                if (_x2Min != _xMin)
                {
                    d = new Direction3D((_yMax - _yMin) * vx + (_xMin - _x2Min) * vy);
                }
                break;

            case 1:
                // XMax
                x = _xMax;
                y = _yMin;
                z = _zMin;
                if (_x2Max != _xMax)
                {
                    d = new Direction3D((_yMax - _yMin) * vx + (_xMax - _x2Max) * vy);
                }
                break;

            case 2:
                // YMin
                x = _xMin;
                y = _yMin;
                z = _zMin;
                break;

            case 3:
                // YMax
                x = _xMin;
                y = _yMax;
                z = _zMin;
                break;

            case 4:
                // ZMin
                x = _xMin;
                y = _yMin;
                z = _zMin;
                if (_z2Min != _zMin)
                {
                    d = new Direction3D((_yMax - _yMin) * vz + (_zMin - _z2Min) * vy);
                }
                break;

            case 5:
                // ZMax
                x = _xMin;
                y = _yMin;
                z = _zMax;
                if (_z2Max != _zMax)
                {
                    d = new Direction3D((_yMax - _yMin) * vz + (_zMax - _z2Max) * vy);
                }
                break;
        }

        Point3D p = _axes.Location;
        p = p.Translated(x * new Vector3D(_axes.XDirection));
        p = p.Translated(y * new Vector3D(_axes.YDirection));
        p = p.Translated(z * new Vector3D(_axes.Direction));
        Plane3D plane = new(p, d);
        return plane;
    }

    /// <summary>The face in the direction (unfrozen), built with its wire and pcurves.</summary>
    public Face Face(PrimitiveDirection d1)
    {
        int i = BRepPrim_Wedge_NumDir1(d1);

        if (!_facesBuilt[i])
        {
            Plane3D p = Plane(d1);
            _builder.MakeFace(ref _faces[i], p);
            if (HasWire(d1))
            {
                _builder.AddFaceWire(_faces[i], Wire(d1));
            }
            if (i % 2 == 0)
            {
                _builder.ReverseFace(ref _faces[i]);
            }

            // pcurves

            PrimitiveDirection dd1 = PrimitiveDirection.BRepPrim_ZMin, dd2 = PrimitiveDirection.BRepPrim_YMax, dd3 = PrimitiveDirection.BRepPrim_ZMax, dd4 = PrimitiveDirection.BRepPrim_YMin;

            switch (i / 2)
            {
                case 0:
                    // XMin XMax
                    dd1 = PrimitiveDirection.BRepPrim_ZMin;
                    dd2 = PrimitiveDirection.BRepPrim_YMax;
                    dd3 = PrimitiveDirection.BRepPrim_ZMax;
                    dd4 = PrimitiveDirection.BRepPrim_YMin;
                    break;

                case 1:
                    // YMin YMax
                    dd1 = PrimitiveDirection.BRepPrim_XMin;
                    dd2 = PrimitiveDirection.BRepPrim_ZMax;
                    dd3 = PrimitiveDirection.BRepPrim_XMax;
                    dd4 = PrimitiveDirection.BRepPrim_ZMin;
                    break;

                case 2:
                    // ZMin ZMax
                    dd1 = PrimitiveDirection.BRepPrim_YMin;
                    dd2 = PrimitiveDirection.BRepPrim_XMax;
                    dd3 = PrimitiveDirection.BRepPrim_YMax;
                    dd4 = PrimitiveDirection.BRepPrim_XMin;
                    break;
            }

            Line3D l;
            Direction3D dx = p.XAxis.Direction;
            Direction3D dy = p.YAxis.Direction;
            double u, v, du, dv;
            if (HasEdge(d1, dd4))
            {
                l = Line(d1, dd4);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(p, l.Location, out u, out v);
                du = l.Direction * dx;
                dv = l.Direction * dy;
                _builder.SetPCurve(_edges[BRepPrim_Wedge_NumDir2(d1, dd4)],
                                    _faces[i],
                                    new Line2D(new Point2D(u, v), new Direction2D(du, dv)));
            }
            if (HasEdge(d1, dd3))
            {
                l = Line(d1, dd3);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(p, l.Location, out u, out v);
                du = l.Direction * dx;
                dv = l.Direction * dy;
                _builder.SetPCurve(_edges[BRepPrim_Wedge_NumDir2(d1, dd3)],
                                    _faces[i],
                                    new Line2D(new Point2D(u, v), new Direction2D(du, dv)));
            }

            if (HasEdge(d1, dd2))
            {
                l = Line(d1, dd2);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(p, l.Location, out u, out v);
                du = l.Direction * dx;
                dv = l.Direction * dy;
                _builder.SetPCurve(_edges[BRepPrim_Wedge_NumDir2(d1, dd2)],
                                    _faces[i],
                                    new Line2D(new Point2D(u, v), new Direction2D(du, dv)));
            }

            if (HasEdge(d1, dd1))
            {
                l = Line(d1, dd1);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(p, l.Location, out u, out v);
                du = l.Direction * dx;
                dv = l.Direction * dy;
                _builder.SetPCurve(_edges[BRepPrim_Wedge_NumDir2(d1, dd1)],
                                    _faces[i],
                                    new Line2D(new Point2D(u, v), new Direction2D(du, dv)));
            }

            _builder.CompleteFace(_faces[i]);
            _facesBuilt[i] = true;
        }

        return _faces[i];
    }

    /// <summary>True if the face in the direction has a wire.</summary>
    public bool HasWire(PrimitiveDirection d1)
    {
        int i = BRepPrim_Wedge_NumDir1(d1);

        if (_infinite[i])
        {
            return false;
        }
        PrimitiveDirection dd1 = PrimitiveDirection.BRepPrim_XMin, dd2 = PrimitiveDirection.BRepPrim_YMax, dd3 = PrimitiveDirection.BRepPrim_XMax, dd4 = PrimitiveDirection.BRepPrim_ZMin;

        switch (i / 2)
        {
            case 0:
                // XMin XMax
                dd1 = PrimitiveDirection.BRepPrim_ZMin;
                dd2 = PrimitiveDirection.BRepPrim_YMax;
                dd3 = PrimitiveDirection.BRepPrim_ZMax;
                dd4 = PrimitiveDirection.BRepPrim_YMin;
                break;

            case 1:
                // YMin YMax
                dd1 = PrimitiveDirection.BRepPrim_XMin;
                dd2 = PrimitiveDirection.BRepPrim_ZMax;
                dd3 = PrimitiveDirection.BRepPrim_XMax;
                dd4 = PrimitiveDirection.BRepPrim_ZMin;
                break;

            case 2:
                // ZMin ZMax
                dd1 = PrimitiveDirection.BRepPrim_YMin;
                dd2 = PrimitiveDirection.BRepPrim_XMax;
                dd3 = PrimitiveDirection.BRepPrim_YMax;
                dd4 = PrimitiveDirection.BRepPrim_XMin;
                break;
            default:
                break;
        }

        return HasEdge(d1, dd1) || HasEdge(d1, dd2) || HasEdge(d1, dd3) || HasEdge(d1, dd4);
    }

    /// <summary>The wire of the face in the direction (unfrozen).</summary>
    public Wire Wire(PrimitiveDirection d1)
    {
        int i = BRepPrim_Wedge_NumDir1(d1);

        PrimitiveDirection dd1 = PrimitiveDirection.BRepPrim_XMin, dd2 = PrimitiveDirection.BRepPrim_YMax, dd3 = PrimitiveDirection.BRepPrim_XMax, dd4 = PrimitiveDirection.BRepPrim_ZMin;

        if (!_wiresBuilt[i])
        {
            switch (i / 2)
            {
                case 0:
                    // XMin XMax
                    dd1 = PrimitiveDirection.BRepPrim_ZMin;
                    dd2 = PrimitiveDirection.BRepPrim_YMax;
                    dd3 = PrimitiveDirection.BRepPrim_ZMax;
                    dd4 = PrimitiveDirection.BRepPrim_YMin;
                    break;

                case 1:
                    // YMin YMax
                    dd1 = PrimitiveDirection.BRepPrim_XMin;
                    dd2 = PrimitiveDirection.BRepPrim_ZMax;
                    dd3 = PrimitiveDirection.BRepPrim_XMax;
                    dd4 = PrimitiveDirection.BRepPrim_ZMin;
                    break;

                case 2:
                    // ZMin ZMax
                    dd1 = PrimitiveDirection.BRepPrim_YMin;
                    dd2 = PrimitiveDirection.BRepPrim_XMax;
                    dd3 = PrimitiveDirection.BRepPrim_YMax;
                    dd4 = PrimitiveDirection.BRepPrim_XMin;
                    break;
                default:
                    break;
            }

            _builder.MakeWire(ref _wires[i]);

            if (HasEdge(d1, dd4))
            {
                _builder.AddWireEdge(_wires[i], Edge(d1, dd4), false);
            }
            if (HasEdge(d1, dd3))
            {
                _builder.AddWireEdge(_wires[i], Edge(d1, dd3), false);
            }
            if (HasEdge(d1, dd2))
            {
                _builder.AddWireEdge(_wires[i], Edge(d1, dd2), true);
            }
            if (HasEdge(d1, dd1))
            {
                _builder.AddWireEdge(_wires[i], Edge(d1, dd1), true);
            }

            _builder.CompleteWire(_wires[i]);
            _wiresBuilt[i] = true;
        }

        return _wires[i];
    }

    /// <summary>True if the edge between the two directions exists.</summary>
    public bool HasEdge(PrimitiveDirection d1, PrimitiveDirection d2)
    {
        bool state = !(_infinite[BRepPrim_Wedge_NumDir1(d1)] || _infinite[BRepPrim_Wedge_NumDir1(d2)]);
        int i = BRepPrim_Wedge_NumDir2(d1, d2);
        if (i == 6 || i == 7)
        {
            state = state && (_x2Max != _x2Min);
        }
        else if (i == 1 || i == 3)
        {
            state = state && (_z2Max != _z2Min);
        }
        return state;
    }

    /// <summary>The line of the edge between the two directions.</summary>
    public Line3D Line(PrimitiveDirection d1, PrimitiveDirection d2)
    {
        if (!HasEdge(d1, d2))
        {
            throw new DomainErrorException();
        }

        int i = BRepPrim_Wedge_NumDir2(d1, d2);

        double x = 0.0, y = 0.0, z = 0.0;

        Direction3D d = new();
        Vector3D vx = new(_axes.XDirection);
        Vector3D vy = new(_axes.YDirection);
        Vector3D vz = new(_axes.Direction);

        switch (i / 4)
        {
            case 0:
                d = _axes.Direction;
                break;

            case 1:
                d = _axes.XDirection;
                break;

            case 2:
                d = _axes.YDirection;
                break;
        }

        switch (i)
        {
            case 0:
                // XMin YMin
                x = _xMin;
                y = _yMin;
                z = _zMin;
                break;

            case 1:
                // XMin YMax
                x = _x2Min;
                y = _yMax;
                z = _z2Min;
                break;

            case 2:
                // XMax YMin
                x = _xMax;
                y = _yMin;
                z = _zMin;
                break;

            case 3:
                // XMax YMax
                x = _x2Max;
                y = _yMax;
                z = _z2Min;
                break;

            case 4:
                // YMin ZMin
                x = _xMin;
                y = _yMin;
                z = _zMin;
                break;

            case 5:
                // YMin ZMax
                x = _xMin;
                y = _yMin;
                z = _zMax;
                break;

            case 6:
                // YMax ZMin
                x = _x2Min;
                y = _yMax;
                z = _z2Min;
                break;

            case 7:
                // YMax ZMax
                x = _x2Min;
                y = _yMax;
                z = _z2Max;
                break;

            case 8:
                // ZMin XMin
                x = _xMin;
                y = _yMin;
                z = _zMin;
                if ((_xMin != _x2Min) || (_zMin != _z2Min))
                {
                    d = new Direction3D((_x2Min - _xMin) * vx + (_yMax - _yMin) * vy + (_z2Min - _zMin) * vz);
                }
                break;

            case 9:
                // ZMax XMin
                x = _xMin;
                y = _yMin;
                z = _zMax;
                if ((_xMin != _x2Min) || (_zMax != _z2Max))
                {
                    d = new Direction3D((_x2Min - _xMin) * vx + (_yMax - _yMin) * vy + (_z2Max - _zMax) * vz);
                }
                break;

            case 10:
                // ZMin XMax
                x = _xMax;
                y = _yMin;
                z = _zMin;
                if ((_xMax != _x2Max) || (_zMin != _z2Min))
                {
                    d = new Direction3D((_x2Max - _xMax) * vx + (_yMax - _yMin) * vy + (_z2Min - _zMin) * vz);
                }
                break;

            case 11:
                // ZMax XMax
                x = _xMax;
                y = _yMin;
                z = _zMax;
                if ((_xMax != _x2Max) || (_zMax != _z2Max))
                {
                    d = new Direction3D((_x2Max - _xMax) * vx + (_yMax - _yMin) * vy + (_z2Max - _zMax) * vz);
                }
                break;
        }

        Point3D p = _axes.Location;
        p = p.Translated(x * new Vector3D(_axes.XDirection));
        p = p.Translated(y * new Vector3D(_axes.YDirection));
        p = p.Translated(z * new Vector3D(_axes.Direction));
        return new Line3D(new Axis3D(p, d));
    }

    /// <summary>The edge between the two directions (unfrozen).</summary>
    public Edge Edge(PrimitiveDirection d1, PrimitiveDirection d2)
    {
        if (!HasEdge(d1, d2))
        {
            throw new DomainErrorException();
        }

        int i = BRepPrim_Wedge_NumDir2(d1, d2);

        if (!_edgesBuilt[i])
        {
            PrimitiveDirection dd1 = PrimitiveDirection.BRepPrim_XMin, dd2 = PrimitiveDirection.BRepPrim_XMax;

            switch (i / 4)
            {
                case 0:
                    dd1 = PrimitiveDirection.BRepPrim_ZMin;
                    dd2 = PrimitiveDirection.BRepPrim_ZMax;
                    break;

                case 1:
                    dd1 = PrimitiveDirection.BRepPrim_XMin;
                    dd2 = PrimitiveDirection.BRepPrim_XMax;
                    break;

                case 2:
                    dd1 = PrimitiveDirection.BRepPrim_YMin;
                    dd2 = PrimitiveDirection.BRepPrim_YMax;
                    break;

                default:
                    break;
            }

            Line3D l = Line(d1, d2);
            _builder.MakeEdge(ref _edges[i], l);

            if (HasVertex(d1, d2, dd2))
            {
                _builder.AddEdgeVertex(_edges[i],
                                        Vertex(d1, d2, dd2),
                                        global::Kerf.Engine.Geometry.ElCLib.Parameter(l, Point(d1, d2, dd2)),
                                        false);
            }
            if (HasVertex(d1, d2, dd1))
            {
                _builder.AddEdgeVertex(_edges[i],
                                        Vertex(d1, d2, dd1),
                                        global::Kerf.Engine.Geometry.ElCLib.Parameter(l, Point(d1, d2, dd1)),
                                        true);
            }

            if (_z2Max == _z2Min)
            {
                if (i == 6)
                {
                    _edges[7] = _edges[6];
                    _edgesBuilt[7] = true;
                }
                else if (i == 7)
                {
                    _edges[6] = _edges[7];
                    _edgesBuilt[6] = true;
                }
            }
            if (_x2Max == _x2Min)
            {
                if (i == 1)
                {
                    _edges[3] = _edges[1];
                    _edgesBuilt[3] = true;
                }
                else if (i == 3)
                {
                    _edges[1] = _edges[3];
                    _edgesBuilt[1] = true;
                }
            }

            _builder.CompleteEdge(_edges[i]);
            _edgesBuilt[i] = true;
        }

        return _edges[i];
    }

    /// <summary>True if the vertex at the three directions exists.</summary>
    public bool HasVertex(PrimitiveDirection d1, PrimitiveDirection d2, PrimitiveDirection d3)
    {
        return !(_infinite[BRepPrim_Wedge_NumDir1(d1)] || _infinite[BRepPrim_Wedge_NumDir1(d2)]
                 || _infinite[BRepPrim_Wedge_NumDir1(d3)]);
    }

    /// <summary>The point of the vertex at the three directions.</summary>
    public Point3D Point(PrimitiveDirection d1, PrimitiveDirection d2, PrimitiveDirection d3)
    {
        if (!HasVertex(d1, d2, d3))
        {
            throw new DomainErrorException();
        }

        int i = BRepPrim_Wedge_NumDir3(d1, d2, d3);

        double x = 0.0, y = 0.0, z = 0.0;

        switch (i)
        {
            case 0:
                x = _xMin;
                y = _yMin;
                z = _zMin;
                break;

            case 1:
                x = _xMin;
                y = _yMin;
                z = _zMax;
                break;

            case 2:
                x = _x2Min;
                y = _yMax;
                z = _z2Min;
                break;

            case 3:
                x = _x2Min;
                y = _yMax;
                z = _z2Max;
                break;

            case 4:
                x = _xMax;
                y = _yMin;
                z = _zMin;
                break;

            case 5:
                x = _xMax;
                y = _yMin;
                z = _zMax;
                break;

            case 6:
                x = _x2Max;
                y = _yMax;
                z = _z2Min;
                break;

            case 7:
                x = _x2Max;
                y = _yMax;
                z = _z2Max;
                break;
        }

        Point3D p = _axes.Location;
        p = p.Translated(x * new Vector3D(_axes.XDirection));
        p = p.Translated(y * new Vector3D(_axes.YDirection));
        p = p.Translated(z * new Vector3D(_axes.Direction));
        return p;
    }

    /// <summary>The vertex at the three directions (unfrozen).</summary>
    public Vertex Vertex(PrimitiveDirection d1, PrimitiveDirection d2, PrimitiveDirection d3)
    {
        if (!HasVertex(d1, d2, d3))
        {
            throw new DomainErrorException();
        }

        int i = BRepPrim_Wedge_NumDir3(d1, d2, d3);

        if (!_verticesBuilt[i])
        {
            _builder.MakeVertex(ref _vertices[i], Point(d1, d2, d3));

            // OCCT: the vertices shared by a degenerated top are copied in pairs, a partner not built yet as a null vertex
            // marked built, which a later request for it then returns (occt-issues.md).
            if (_z2Max == _z2Min)
            {
                if (i == 2 || i == 6)
                {
                    _vertices[3] = _vertices[2];
                    _vertices[7] = _vertices[6];
                    _verticesBuilt[3] = true;
                    _verticesBuilt[7] = true;
                }
                else if (i == 3 || i == 7)
                {
                    _vertices[2] = _vertices[3];
                    _vertices[6] = _vertices[7];
                    _verticesBuilt[2] = true;
                    _verticesBuilt[6] = true;
                }
            }
            if (_x2Max == _x2Min)
            {
                if (i == 2 || i == 3)
                {
                    _vertices[6] = _vertices[2];
                    _vertices[7] = _vertices[3];
                    _verticesBuilt[6] = true;
                    _verticesBuilt[7] = true;
                }
                else if (i == 6 || i == 7)
                {
                    _vertices[2] = _vertices[6];
                    _vertices[3] = _vertices[7];
                    _verticesBuilt[2] = true;
                    _verticesBuilt[3] = true;
                }
            }

            _verticesBuilt[i] = true;
        }

        return _vertices[i];
    }

    /// <summary>True if the wedge is degenerated (a null dimension, or a negative top).</summary>
    public bool IsDegeneratedShape
    {
        get
        {
            return (_xMax - _xMin <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (_yMax - _yMin <= global::Kerf.Engine.Foundation.Precision.Confusion())
                   || (_zMax - _zMin <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (_z2Max - _z2Min < 0) || (_x2Max - _x2Min < 0);
        }
    }
}
