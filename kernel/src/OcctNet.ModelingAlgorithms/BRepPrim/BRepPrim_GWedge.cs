// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_GWedge.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_GWedge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.BRepPrim_Direction;

namespace Occt;

/// <summary>
/// A wedge: a box, or a box whose YMax face is a smaller rectangle (Z2Min..Z2Max, X2Min..X2Max), possibly a segment
/// or a point. Each face of the wedge can be removed (Open) to make it infinite in that direction. Building an element
/// builds all its sub-elements, so building the shell builds everything.
/// </summary>
/// <remarks>Internal (decision 465): its shapes are unfrozen; BRepPrimAPI_MakeBox and MakeWedge publish them.</remarks>
internal class BRepPrim_GWedge
{
    private const int NBFACES = 6;
    private const int NBWIRES = 6;
    private const int NBEDGES = 12;
    private const int NBVERTICES = 8;

    private static readonly int[] num = [0, 1, 2, 3, 4, 5];
    private static readonly int[] val = [0, 4, 0, 2, 0, 1];
    private static readonly int[,] tab = { { -1, -1, 0, 1, 8, 9 },
                                           { -1, -1, 2, 3, 10, 11 },
                                           { 0, 2, -1, -1, 4, 5 },
                                           { 1, 3, -1, -1, 6, 7 },
                                           { 8, 10, 4, 6, -1, -1 },
                                           { 9, 11, 5, 7, -1, -1 } };

    // when giving a direction return the range of the face
    private static int BRepPrim_Wedge_NumDir1(BRepPrim_Direction d1)
    {
        return num[(int)d1];
    }

    // when giving two directions return the range of the edge
    private static int BRepPrim_Wedge_NumDir2(BRepPrim_Direction d1, BRepPrim_Direction d2)
    {
        int i1 = BRepPrim_Wedge_NumDir1(d1);
        int i2 = BRepPrim_Wedge_NumDir1(d2);
        if (i1 / 2 == i2 / 2)
        {
            throw new DomainErrorException();
        }
        return tab[i1, i2];
    }

    // when giving three directions return the range of the vertex
    private static int BRepPrim_Wedge_NumDir3(BRepPrim_Direction d1, BRepPrim_Direction d2, BRepPrim_Direction d3)
    {
        int i1 = BRepPrim_Wedge_NumDir1(d1);
        int i2 = BRepPrim_Wedge_NumDir1(d2);
        int i3 = BRepPrim_Wedge_NumDir1(d3);
        if ((i1 / 2 == i2 / 2) || (i2 / 2 == i3 / 2) || (i3 / 2 == i1 / 2))
        {
            throw new DomainErrorException();
        }
        return val[i1] + val[i2] + val[i3];
    }

    // raise Standard_DomainError if something was built
    private static void BRepPrim_Wedge_Check(bool[] V, bool[] E, bool[] W, bool[] F)
    {
        int i;
        for (i = 0; i < NBVERTICES; i++)
        {
            if (V[i])
            {
                throw new DomainErrorException();
            }
        }
        for (i = 0; i < NBEDGES; i++)
        {
            if (E[i])
            {
                throw new DomainErrorException();
            }
        }
        for (i = 0; i < NBWIRES; i++)
        {
            if (W[i])
            {
                throw new DomainErrorException();
            }
        }
        for (i = 0; i < NBFACES; i++)
        {
            if (F[i])
            {
                throw new DomainErrorException();
            }
        }
    }

    private static void BRepPrim_Wedge_Init(out bool S, bool[] V, bool[] E, bool[] W, bool[] F)
    {
        int i;
        S = false;
        for (i = 0; i < NBVERTICES; i++)
        {
            V[i] = false;
        }
        for (i = 0; i < NBEDGES; i++)
        {
            E[i] = false;
        }
        for (i = 0; i < NBWIRES; i++)
        {
            W[i] = false;
        }
        for (i = 0; i < NBFACES; i++)
        {
            F[i] = false;
        }
    }

    private readonly BRepPrim_Builder myBuilder;
    private readonly Frame3D myAxes;
    private readonly double XMin;
    private readonly double XMax;
    private readonly double YMin;
    private readonly double YMax;
    private readonly double ZMin;
    private readonly double ZMax;
    private readonly double Z2Min;
    private readonly double Z2Max;
    private readonly double X2Min;
    private readonly double X2Max;
    private TopoDS_Shell myShell = new();
    private bool ShellBuilt;
    private readonly TopoDS_Vertex[] myVertices = NewArray<TopoDS_Vertex>(NBVERTICES);
    private readonly bool[] VerticesBuilt = new bool[NBVERTICES];
    private readonly TopoDS_Edge[] myEdges = NewArray<TopoDS_Edge>(NBEDGES);
    private readonly bool[] EdgesBuilt = new bool[NBEDGES];
    private readonly TopoDS_Wire[] myWires = NewArray<TopoDS_Wire>(NBWIRES);
    private readonly bool[] WiresBuilt = new bool[NBWIRES];
    private readonly TopoDS_Face[] myFaces = NewArray<TopoDS_Face>(NBFACES);
    private readonly bool[] FacesBuilt = new bool[NBFACES];
    private readonly bool[] myInfinite = new bool[NBFACES];

    // Arrays of null shapes, as C++ default-constructs its arrays of shapes
    internal static T[] NewArray<T>(int theSize) where T : struct
    {
        var anArray = new T[theSize];
        for (int i = 0; i < theSize; i++)
        {
            anArray[i] = new T();
        }
        return anArray;
    }

    public BRepPrim_GWedge()
    {
        myBuilder = new BRepPrim_Builder();
        myAxes = new Frame3D();
        XMin = 0;
        XMax = 0;
        YMin = 0;
        YMax = 0;
        ZMin = 0;
        ZMax = 0;
        Z2Min = 0;
        Z2Max = 0;
        X2Min = 0;
        X2Max = 0;
        for (int i = 0; i < NBFACES; i++)
        {
            myInfinite[i] = false;
        }

        BRepPrim_Wedge_Init(out ShellBuilt, VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
    }

    /// <summary>A box: XMin, YMin, ZMin at 0, XMax, YMax, ZMax at dx, dy, dz, Z2 = Z and X2 = X.</summary>
    public BRepPrim_GWedge(BRepPrim_Builder B, in Frame3D Axes, double dx, double dy, double dz)
    {
        myBuilder = B;
        myAxes = Axes;
        XMin = 0;
        XMax = dx;
        YMin = 0;
        YMax = dy;
        ZMin = 0;
        ZMax = dz;
        Z2Min = 0;
        Z2Max = dz;
        X2Min = 0;
        X2Max = dx;
        for (int i = 0; i < NBFACES; i++)
        {
            myInfinite[i] = false;
        }

        BRepPrim_Wedge_Init(out ShellBuilt, VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
    }

    /// <summary>A STEP right angular wedge: as the box, with X2Min = 0 and X2Max = ltx.</summary>
    public BRepPrim_GWedge(BRepPrim_Builder B, in Frame3D Axes, double dx, double dy, double dz, double ltx)
    {
        myBuilder = B;
        myAxes = Axes;
        XMin = 0;
        XMax = dx;
        YMin = 0;
        YMax = dy;
        ZMin = 0;
        ZMax = dz;
        Z2Min = 0;
        Z2Max = dz;
        X2Min = 0;
        X2Max = ltx;
        for (int i = 0; i < NBFACES; i++)
        {
            myInfinite[i] = false;
        }

        BRepPrim_Wedge_Init(out ShellBuilt, VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
    }

    /// <summary>Builds a wedge by giving all the fields.</summary>
    public BRepPrim_GWedge(BRepPrim_Builder B,
                           in Frame3D Axes,
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
        myBuilder = B;
        myAxes = Axes;
        XMin = xmin;
        XMax = xmax;
        YMin = ymin;
        YMax = ymax;
        ZMin = zmin;
        ZMax = zmax;
        Z2Min = z2min;
        Z2Max = z2max;
        X2Min = x2min;
        X2Max = x2max;
        for (int i = 0; i < NBFACES; i++)
        {
            myInfinite[i] = false;
        }

        BRepPrim_Wedge_Init(out ShellBuilt, VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
    }

    public Frame3D Axes()
    {
        return myAxes;
    }

    public double GetXMin()
    {
        return XMin;
    }

    public double GetYMin()
    {
        return YMin;
    }

    public double GetZMin()
    {
        return ZMin;
    }

    public double GetZ2Min()
    {
        return Z2Min;
    }

    public double GetX2Min()
    {
        return X2Min;
    }

    public double GetXMax()
    {
        return XMax;
    }

    public double GetYMax()
    {
        return YMax;
    }

    public double GetZMax()
    {
        return ZMax;
    }

    public double GetZ2Max()
    {
        return Z2Max;
    }

    public double GetX2Max()
    {
        return X2Max;
    }

    /// <summary>Opens the wedge in the direction (removes the face). Throws if anything was built.</summary>
    public void Open(BRepPrim_Direction d1)
    {
        BRepPrim_Wedge_Check(VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
        myInfinite[BRepPrim_Wedge_NumDir1(d1)] = true;
    }

    /// <summary>Closes the wedge in the direction. Throws if anything was built.</summary>
    public void Close(BRepPrim_Direction d1)
    {
        BRepPrim_Wedge_Check(VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
        myInfinite[BRepPrim_Wedge_NumDir1(d1)] = false;
    }

    /// <summary>True if it is open in the given direction.</summary>
    public bool IsInfinite(BRepPrim_Direction d1)
    {
        return myInfinite[BRepPrim_Wedge_NumDir1(d1)];
    }

    /// <summary>The shell of the wedge (unfrozen).</summary>
    public TopoDS_Shell Shell()
    {
        if (IsDegeneratedShape())
        {
            throw new DomainErrorException();
        }

        if (!ShellBuilt)
        {
            myBuilder.MakeShell(ref myShell);

            if (HasFace(BRepPrim_XMin))
            {
                myBuilder.AddShellFace(myShell, Face(BRepPrim_XMin));
            }
            if (HasFace(BRepPrim_XMax))
            {
                myBuilder.AddShellFace(myShell, Face(BRepPrim_XMax));
            }
            if (HasFace(BRepPrim_YMin))
            {
                myBuilder.AddShellFace(myShell, Face(BRepPrim_YMin));
            }
            if (HasFace(BRepPrim_YMax))
            {
                myBuilder.AddShellFace(myShell, Face(BRepPrim_YMax));
            }
            if (HasFace(BRepPrim_ZMin))
            {
                myBuilder.AddShellFace(myShell, Face(BRepPrim_ZMin));
            }
            if (HasFace(BRepPrim_ZMax))
            {
                myBuilder.AddShellFace(myShell, Face(BRepPrim_ZMax));
            }

            myShell.Closed(BRep_Tool.IsClosed(myShell));
            myBuilder.CompleteShell(myShell);
            ShellBuilt = true;
        }
        return myShell;
    }

    /// <summary>True if the face exists in the direction.</summary>
    public bool HasFace(BRepPrim_Direction d1)
    {
        bool state = !myInfinite[BRepPrim_Wedge_NumDir1(d1)];
        if (d1 == BRepPrim_YMax)
        {
            state = state && (Z2Max != Z2Min) && (X2Max != X2Min);
        }
        return state;
    }

    /// <summary>The plane of the face in the direction.</summary>
    public Plane3D Plane(BRepPrim_Direction d1)
    {
        int i = BRepPrim_Wedge_NumDir1(d1);

        Direction3D D = new();
        Vector3D VX = new(myAxes.XDirection());
        Vector3D VY = new(myAxes.YDirection());
        Vector3D VZ = new(myAxes.Direction());

        switch (i / 2)
        {
            case 0:
                D = myAxes.XDirection();
                break;

            case 1:
                D = myAxes.YDirection();
                break;

            case 2:
                D = myAxes.Direction();
                break;
        }
        double X = 0.0, Y = 0.0, Z = 0.0;

        switch (i)
        {
            case 0:
                // XMin
                X = XMin;
                Y = YMin;
                Z = ZMin;
                if (X2Min != XMin)
                {
                    D = new Direction3D((YMax - YMin) * VX + (XMin - X2Min) * VY);
                }
                break;

            case 1:
                // XMax
                X = XMax;
                Y = YMin;
                Z = ZMin;
                if (X2Max != XMax)
                {
                    D = new Direction3D((YMax - YMin) * VX + (XMax - X2Max) * VY);
                }
                break;

            case 2:
                // YMin
                X = XMin;
                Y = YMin;
                Z = ZMin;
                break;

            case 3:
                // YMax
                X = XMin;
                Y = YMax;
                Z = ZMin;
                break;

            case 4:
                // ZMin
                X = XMin;
                Y = YMin;
                Z = ZMin;
                if (Z2Min != ZMin)
                {
                    D = new Direction3D((YMax - YMin) * VZ + (ZMin - Z2Min) * VY);
                }
                break;

            case 5:
                // ZMax
                X = XMin;
                Y = YMin;
                Z = ZMax;
                if (Z2Max != ZMax)
                {
                    D = new Direction3D((YMax - YMin) * VZ + (ZMax - Z2Max) * VY);
                }
                break;
        }

        Point3D P = myAxes.Location();
        P.Translate(X * new Vector3D(myAxes.XDirection()));
        P.Translate(Y * new Vector3D(myAxes.YDirection()));
        P.Translate(Z * new Vector3D(myAxes.Direction()));
        Plane3D plane = new(P, D);
        return plane;
    }

    /// <summary>The face in the direction (unfrozen), built with its wire and pcurves.</summary>
    public TopoDS_Face Face(BRepPrim_Direction d1)
    {
        int i = BRepPrim_Wedge_NumDir1(d1);

        if (!FacesBuilt[i])
        {
            Plane3D P = Plane(d1);
            myBuilder.MakeFace(ref myFaces[i], P);
            if (HasWire(d1))
            {
                myBuilder.AddFaceWire(myFaces[i], Wire(d1));
            }
            if (i % 2 == 0)
            {
                myBuilder.ReverseFace(ref myFaces[i]);
            }

            // pcurves

            BRepPrim_Direction dd1 = BRepPrim_ZMin, dd2 = BRepPrim_YMax, dd3 = BRepPrim_ZMax, dd4 = BRepPrim_YMin;

            switch (i / 2)
            {
                case 0:
                    // XMin XMax
                    dd1 = BRepPrim_ZMin;
                    dd2 = BRepPrim_YMax;
                    dd3 = BRepPrim_ZMax;
                    dd4 = BRepPrim_YMin;
                    break;

                case 1:
                    // YMin YMax
                    dd1 = BRepPrim_XMin;
                    dd2 = BRepPrim_ZMax;
                    dd3 = BRepPrim_XMax;
                    dd4 = BRepPrim_ZMin;
                    break;

                case 2:
                    // ZMin ZMax
                    dd1 = BRepPrim_YMin;
                    dd2 = BRepPrim_XMax;
                    dd3 = BRepPrim_YMax;
                    dd4 = BRepPrim_XMin;
                    break;
            }

            Line3D L;
            Direction3D DX = P.XAxis().Direction();
            Direction3D DY = P.YAxis().Direction();
            double U, V, DU, DV;
            if (HasEdge(d1, dd4))
            {
                L = Line(d1, dd4);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(P, L.Location(), out U, out V);
                DU = L.Direction() * DX;
                DV = L.Direction() * DY;
                myBuilder.SetPCurve(myEdges[BRepPrim_Wedge_NumDir2(d1, dd4)],
                                    myFaces[i],
                                    new Line2D(new Point2D(U, V), new Direction2D(DU, DV)));
            }
            if (HasEdge(d1, dd3))
            {
                L = Line(d1, dd3);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(P, L.Location(), out U, out V);
                DU = L.Direction() * DX;
                DV = L.Direction() * DY;
                myBuilder.SetPCurve(myEdges[BRepPrim_Wedge_NumDir2(d1, dd3)],
                                    myFaces[i],
                                    new Line2D(new Point2D(U, V), new Direction2D(DU, DV)));
            }

            if (HasEdge(d1, dd2))
            {
                L = Line(d1, dd2);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(P, L.Location(), out U, out V);
                DU = L.Direction() * DX;
                DV = L.Direction() * DY;
                myBuilder.SetPCurve(myEdges[BRepPrim_Wedge_NumDir2(d1, dd2)],
                                    myFaces[i],
                                    new Line2D(new Point2D(U, V), new Direction2D(DU, DV)));
            }

            if (HasEdge(d1, dd1))
            {
                L = Line(d1, dd1);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(P, L.Location(), out U, out V);
                DU = L.Direction() * DX;
                DV = L.Direction() * DY;
                myBuilder.SetPCurve(myEdges[BRepPrim_Wedge_NumDir2(d1, dd1)],
                                    myFaces[i],
                                    new Line2D(new Point2D(U, V), new Direction2D(DU, DV)));
            }

            myBuilder.CompleteFace(myFaces[i]);
            FacesBuilt[i] = true;
        }

        return myFaces[i];
    }

    /// <summary>True if the face in the direction has a wire.</summary>
    public bool HasWire(BRepPrim_Direction d1)
    {
        int i = BRepPrim_Wedge_NumDir1(d1);

        if (myInfinite[i])
        {
            return false;
        }
        BRepPrim_Direction dd1 = BRepPrim_XMin, dd2 = BRepPrim_YMax, dd3 = BRepPrim_XMax, dd4 = BRepPrim_ZMin;

        switch (i / 2)
        {
            case 0:
                // XMin XMax
                dd1 = BRepPrim_ZMin;
                dd2 = BRepPrim_YMax;
                dd3 = BRepPrim_ZMax;
                dd4 = BRepPrim_YMin;
                break;

            case 1:
                // YMin YMax
                dd1 = BRepPrim_XMin;
                dd2 = BRepPrim_ZMax;
                dd3 = BRepPrim_XMax;
                dd4 = BRepPrim_ZMin;
                break;

            case 2:
                // ZMin ZMax
                dd1 = BRepPrim_YMin;
                dd2 = BRepPrim_XMax;
                dd3 = BRepPrim_YMax;
                dd4 = BRepPrim_XMin;
                break;
            default:
                break;
        }

        return HasEdge(d1, dd1) || HasEdge(d1, dd2) || HasEdge(d1, dd3) || HasEdge(d1, dd4);
    }

    /// <summary>The wire of the face in the direction (unfrozen).</summary>
    public TopoDS_Wire Wire(BRepPrim_Direction d1)
    {
        int i = BRepPrim_Wedge_NumDir1(d1);

        BRepPrim_Direction dd1 = BRepPrim_XMin, dd2 = BRepPrim_YMax, dd3 = BRepPrim_XMax, dd4 = BRepPrim_ZMin;

        if (!WiresBuilt[i])
        {
            switch (i / 2)
            {
                case 0:
                    // XMin XMax
                    dd1 = BRepPrim_ZMin;
                    dd2 = BRepPrim_YMax;
                    dd3 = BRepPrim_ZMax;
                    dd4 = BRepPrim_YMin;
                    break;

                case 1:
                    // YMin YMax
                    dd1 = BRepPrim_XMin;
                    dd2 = BRepPrim_ZMax;
                    dd3 = BRepPrim_XMax;
                    dd4 = BRepPrim_ZMin;
                    break;

                case 2:
                    // ZMin ZMax
                    dd1 = BRepPrim_YMin;
                    dd2 = BRepPrim_XMax;
                    dd3 = BRepPrim_YMax;
                    dd4 = BRepPrim_XMin;
                    break;
                default:
                    break;
            }

            myBuilder.MakeWire(ref myWires[i]);

            if (HasEdge(d1, dd4))
            {
                myBuilder.AddWireEdge(myWires[i], Edge(d1, dd4), false);
            }
            if (HasEdge(d1, dd3))
            {
                myBuilder.AddWireEdge(myWires[i], Edge(d1, dd3), false);
            }
            if (HasEdge(d1, dd2))
            {
                myBuilder.AddWireEdge(myWires[i], Edge(d1, dd2), true);
            }
            if (HasEdge(d1, dd1))
            {
                myBuilder.AddWireEdge(myWires[i], Edge(d1, dd1), true);
            }

            myBuilder.CompleteWire(myWires[i]);
            WiresBuilt[i] = true;
        }

        return myWires[i];
    }

    /// <summary>True if the edge between the two directions exists.</summary>
    public bool HasEdge(BRepPrim_Direction d1, BRepPrim_Direction d2)
    {
        bool state = !(myInfinite[BRepPrim_Wedge_NumDir1(d1)] || myInfinite[BRepPrim_Wedge_NumDir1(d2)]);
        int i = BRepPrim_Wedge_NumDir2(d1, d2);
        if (i == 6 || i == 7)
        {
            state = state && (X2Max != X2Min);
        }
        else if (i == 1 || i == 3)
        {
            state = state && (Z2Max != Z2Min);
        }
        return state;
    }

    /// <summary>The line of the edge between the two directions.</summary>
    public Line3D Line(BRepPrim_Direction d1, BRepPrim_Direction d2)
    {
        if (!HasEdge(d1, d2))
        {
            throw new DomainErrorException();
        }

        int i = BRepPrim_Wedge_NumDir2(d1, d2);

        double X = 0.0, Y = 0.0, Z = 0.0;

        Direction3D D = new();
        Vector3D VX = new(myAxes.XDirection());
        Vector3D VY = new(myAxes.YDirection());
        Vector3D VZ = new(myAxes.Direction());

        switch (i / 4)
        {
            case 0:
                D = myAxes.Direction();
                break;

            case 1:
                D = myAxes.XDirection();
                break;

            case 2:
                D = myAxes.YDirection();
                break;
        }

        switch (i)
        {
            case 0:
                // XMin YMin
                X = XMin;
                Y = YMin;
                Z = ZMin;
                break;

            case 1:
                // XMin YMax
                X = X2Min;
                Y = YMax;
                Z = Z2Min;
                break;

            case 2:
                // XMax YMin
                X = XMax;
                Y = YMin;
                Z = ZMin;
                break;

            case 3:
                // XMax YMax
                X = X2Max;
                Y = YMax;
                Z = Z2Min;
                break;

            case 4:
                // YMin ZMin
                X = XMin;
                Y = YMin;
                Z = ZMin;
                break;

            case 5:
                // YMin ZMax
                X = XMin;
                Y = YMin;
                Z = ZMax;
                break;

            case 6:
                // YMax ZMin
                X = X2Min;
                Y = YMax;
                Z = Z2Min;
                break;

            case 7:
                // YMax ZMax
                X = X2Min;
                Y = YMax;
                Z = Z2Max;
                break;

            case 8:
                // ZMin XMin
                X = XMin;
                Y = YMin;
                Z = ZMin;
                if ((XMin != X2Min) || (ZMin != Z2Min))
                {
                    D = new Direction3D((X2Min - XMin) * VX + (YMax - YMin) * VY + (Z2Min - ZMin) * VZ);
                }
                break;

            case 9:
                // ZMax XMin
                X = XMin;
                Y = YMin;
                Z = ZMax;
                if ((XMin != X2Min) || (ZMax != Z2Max))
                {
                    D = new Direction3D((X2Min - XMin) * VX + (YMax - YMin) * VY + (Z2Max - ZMax) * VZ);
                }
                break;

            case 10:
                // ZMin XMax
                X = XMax;
                Y = YMin;
                Z = ZMin;
                if ((XMax != X2Max) || (ZMin != Z2Min))
                {
                    D = new Direction3D((X2Max - XMax) * VX + (YMax - YMin) * VY + (Z2Min - ZMin) * VZ);
                }
                break;

            case 11:
                // ZMax XMax
                X = XMax;
                Y = YMin;
                Z = ZMax;
                if ((XMax != X2Max) || (ZMax != Z2Max))
                {
                    D = new Direction3D((X2Max - XMax) * VX + (YMax - YMin) * VY + (Z2Max - ZMax) * VZ);
                }
                break;
        }

        Point3D P = myAxes.Location();
        P.Translate(X * new Vector3D(myAxes.XDirection()));
        P.Translate(Y * new Vector3D(myAxes.YDirection()));
        P.Translate(Z * new Vector3D(myAxes.Direction()));
        return new Line3D(new Axis3D(P, D));
    }

    /// <summary>The edge between the two directions (unfrozen).</summary>
    public TopoDS_Edge Edge(BRepPrim_Direction d1, BRepPrim_Direction d2)
    {
        if (!HasEdge(d1, d2))
        {
            throw new DomainErrorException();
        }

        int i = BRepPrim_Wedge_NumDir2(d1, d2);

        if (!EdgesBuilt[i])
        {
            BRepPrim_Direction dd1 = BRepPrim_XMin, dd2 = BRepPrim_XMax;

            switch (i / 4)
            {
                case 0:
                    dd1 = BRepPrim_ZMin;
                    dd2 = BRepPrim_ZMax;
                    break;

                case 1:
                    dd1 = BRepPrim_XMin;
                    dd2 = BRepPrim_XMax;
                    break;

                case 2:
                    dd1 = BRepPrim_YMin;
                    dd2 = BRepPrim_YMax;
                    break;

                default:
                    break;
            }

            Line3D L = Line(d1, d2);
            myBuilder.MakeEdge(ref myEdges[i], L);

            if (HasVertex(d1, d2, dd2))
            {
                myBuilder.AddEdgeVertex(myEdges[i],
                                        Vertex(d1, d2, dd2),
                                        global::Kerf.Engine.Geometry.ElCLib.Parameter(L, Point(d1, d2, dd2)),
                                        false);
            }
            if (HasVertex(d1, d2, dd1))
            {
                myBuilder.AddEdgeVertex(myEdges[i],
                                        Vertex(d1, d2, dd1),
                                        global::Kerf.Engine.Geometry.ElCLib.Parameter(L, Point(d1, d2, dd1)),
                                        true);
            }

            if (Z2Max == Z2Min)
            {
                if (i == 6)
                {
                    myEdges[7] = myEdges[6];
                    EdgesBuilt[7] = true;
                }
                else if (i == 7)
                {
                    myEdges[6] = myEdges[7];
                    EdgesBuilt[6] = true;
                }
            }
            if (X2Max == X2Min)
            {
                if (i == 1)
                {
                    myEdges[3] = myEdges[1];
                    EdgesBuilt[3] = true;
                }
                else if (i == 3)
                {
                    myEdges[1] = myEdges[3];
                    EdgesBuilt[1] = true;
                }
            }

            myBuilder.CompleteEdge(myEdges[i]);
            EdgesBuilt[i] = true;
        }

        return myEdges[i];
    }

    /// <summary>True if the vertex at the three directions exists.</summary>
    public bool HasVertex(BRepPrim_Direction d1, BRepPrim_Direction d2, BRepPrim_Direction d3)
    {
        return !(myInfinite[BRepPrim_Wedge_NumDir1(d1)] || myInfinite[BRepPrim_Wedge_NumDir1(d2)]
                 || myInfinite[BRepPrim_Wedge_NumDir1(d3)]);
    }

    /// <summary>The point of the vertex at the three directions.</summary>
    public Point3D Point(BRepPrim_Direction d1, BRepPrim_Direction d2, BRepPrim_Direction d3)
    {
        if (!HasVertex(d1, d2, d3))
        {
            throw new DomainErrorException();
        }

        int i = BRepPrim_Wedge_NumDir3(d1, d2, d3);

        double X = 0.0, Y = 0.0, Z = 0.0;

        switch (i)
        {
            case 0:
                X = XMin;
                Y = YMin;
                Z = ZMin;
                break;

            case 1:
                X = XMin;
                Y = YMin;
                Z = ZMax;
                break;

            case 2:
                X = X2Min;
                Y = YMax;
                Z = Z2Min;
                break;

            case 3:
                X = X2Min;
                Y = YMax;
                Z = Z2Max;
                break;

            case 4:
                X = XMax;
                Y = YMin;
                Z = ZMin;
                break;

            case 5:
                X = XMax;
                Y = YMin;
                Z = ZMax;
                break;

            case 6:
                X = X2Max;
                Y = YMax;
                Z = Z2Min;
                break;

            case 7:
                X = X2Max;
                Y = YMax;
                Z = Z2Max;
                break;
        }

        Point3D P = myAxes.Location();
        P.Translate(X * new Vector3D(myAxes.XDirection()));
        P.Translate(Y * new Vector3D(myAxes.YDirection()));
        P.Translate(Z * new Vector3D(myAxes.Direction()));
        return P;
    }

    /// <summary>The vertex at the three directions (unfrozen).</summary>
    public TopoDS_Vertex Vertex(BRepPrim_Direction d1, BRepPrim_Direction d2, BRepPrim_Direction d3)
    {
        if (!HasVertex(d1, d2, d3))
        {
            throw new DomainErrorException();
        }

        int i = BRepPrim_Wedge_NumDir3(d1, d2, d3);

        if (!VerticesBuilt[i])
        {
            myBuilder.MakeVertex(ref myVertices[i], Point(d1, d2, d3));

            // OCCT: the vertices shared by a degenerated top are copied in pairs, a partner not built yet as a null vertex
            // marked built, which a later request for it then returns (occt-issues.md).
            if (Z2Max == Z2Min)
            {
                if (i == 2 || i == 6)
                {
                    myVertices[3] = myVertices[2];
                    myVertices[7] = myVertices[6];
                    VerticesBuilt[3] = true;
                    VerticesBuilt[7] = true;
                }
                else if (i == 3 || i == 7)
                {
                    myVertices[2] = myVertices[3];
                    myVertices[6] = myVertices[7];
                    VerticesBuilt[2] = true;
                    VerticesBuilt[6] = true;
                }
            }
            if (X2Max == X2Min)
            {
                if (i == 2 || i == 3)
                {
                    myVertices[6] = myVertices[2];
                    myVertices[7] = myVertices[3];
                    VerticesBuilt[6] = true;
                    VerticesBuilt[7] = true;
                }
                else if (i == 6 || i == 7)
                {
                    myVertices[2] = myVertices[6];
                    myVertices[3] = myVertices[7];
                    VerticesBuilt[2] = true;
                    VerticesBuilt[3] = true;
                }
            }

            VerticesBuilt[i] = true;
        }

        return myVertices[i];
    }

    /// <summary>True if the wedge is degenerated (a null dimension, or a negative top).</summary>
    public bool IsDegeneratedShape()
    {
        return (XMax - XMin <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (YMax - YMin <= global::Kerf.Engine.Foundation.Precision.Confusion())
               || (ZMax - ZMin <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (Z2Max - Z2Min < 0) || (X2Max - X2Min < 0);
    }
}
