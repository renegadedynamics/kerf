// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_OneAxis.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_OneAxis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// Algorithm to build primitives with one axis of revolution: a coordinate system whose Z axis is the axis of
/// revolution, an angle around it (2π: no planar side faces; U ranges from 0 to the angle), a parameter range VMin,
/// VMax on the meridian, and the meridian, described by deferred methods. There are at most 5 faces: the lateral face,
/// the top and bottom faces, and the start and end faces. Building an element builds all its sub-elements, so building
/// the shell builds everything.
/// </summary>
/// <remarks>Internal (decision 465): its shapes are unfrozen; the BRepPrimAPI makers publish them.</remarks>
internal abstract class BRepPrim_OneAxis
{
    // Multiplier for Precision::Confusion() to separate overlapping pcurves when heights at VMin and VMax are equal.
    private const double THE_PCURVE_OFFSET_FACTOR = 10.0;

    // Vertex array indices.
    private const int VAXISTOP = 0;
    private const int VAXISBOT = 1;
    private const int VTOPSTART = 2;
    private const int VTOPEND = 3;
    private const int VBOTSTART = 4;
    private const int VBOTEND = 5;
    private const int NBVERTICES = 6;

    // Edge array indices.
    private const int EAXIS = 0;
    private const int ESTART = 1;
    private const int EEND = 2;
    private const int ETOPSTART = 3;
    private const int ETOPEND = 4;
    private const int EBOTSTART = 5;
    private const int EBOTEND = 6;
    private const int ETOP = 7;
    private const int EBOTTOM = 8;
    private const int NBEDGES = 9;

    // Wire array indices. Index 4 is intentionally skipped to preserve historical array sizing.
    private const int WLATERAL = 0;
    private const int WLATERALSTART = WLATERAL;
    private const int WLATERALEND = 1;
    private const int WTOP = 2;
    private const int WBOTTOM = 3;
    private const int WSTART = 5;
    private const int WAXISSTART = 6;
    private const int WEND = 7;
    private const int WAXISEND = 8;
    private const int NBWIRES = 9;

    // Face array indices.
    private const int FLATERAL = 0;
    private const int FTOP = 1;
    private const int FBOTTOM = 2;
    private const int FSTART = 3;
    private const int FEND = 4;
    private const int NBFACES = 5;

    private static void BRepPrim_OneAxis_Check(bool[] V, bool[] E, bool[] W, bool[] F)
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

    protected readonly BRepPrim_Builder myBuilder;

    private Frame3D myAxes;
    private double myAngle;
    private double myVMin;
    private double myVMax;
    private double myMeridianOffset;
    private Shell myShell = new();
    private bool ShellBuilt;
    private readonly Vertex[] myVertices = BRepPrim_GWedge.NewArray<Vertex>(NBVERTICES);
    private readonly bool[] VerticesBuilt = new bool[NBVERTICES];
    private readonly Edge[] myEdges = BRepPrim_GWedge.NewArray<Edge>(NBEDGES);
    private readonly bool[] EdgesBuilt = new bool[NBEDGES];
    private readonly Wire[] myWires = BRepPrim_GWedge.NewArray<Wire>(NBWIRES);
    private readonly bool[] WiresBuilt = new bool[NBWIRES];
    private readonly Face[] myFaces = BRepPrim_GWedge.NewArray<Face>(NBFACES);
    private readonly bool[] FacesBuilt = new bool[NBFACES];

    /// <summary>Creates a OneAxis algorithm. <paramref name="B"/> is used to build the Topology. The angle defaults to 2π.</summary>
    protected BRepPrim_OneAxis(BRepPrim_Builder B, in Frame3D A, double VMin, double VMax)
    {
        myBuilder = B;
        myAxes = A;
        myAngle = 2 * M_PI;
        myVMin = VMin;
        myVMax = VMax;
        myMeridianOffset = 0;

        // init Built flags
        int i;
        ShellBuilt = false;
        for (i = 0; i < NBVERTICES; i++)
        {
            VerticesBuilt[i] = false;
        }
        for (i = 0; i < NBEDGES; i++)
        {
            EdgesBuilt[i] = false;
        }
        for (i = 0; i < NBWIRES; i++)
        {
            WiresBuilt[i] = false;
        }
        for (i = 0; i < NBFACES; i++)
        {
            FacesBuilt[i] = false;
        }
    }

    /// <summary>
    /// The MeridianOffset is added to the parameters on the meridian curve and to the V values of the pcurves (the
    /// sphere's meridian edge has another range than VMin, VMax).
    /// </summary>
    public void SetMeridianOffset(double O = 0)
    {
        myMeridianOffset = O;
    }

    public Frame3D Axes()
    {
        return myAxes;
    }

    public void Axes(in Frame3D A)
    {
        BRepPrim_OneAxis_Check(VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
        myAxes = A;
    }

    public double Angle()
    {
        return myAngle;
    }

    public void Angle(double A)
    {
        BRepPrim_OneAxis_Check(VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
        myAngle = A;
    }

    public double VMin()
    {
        return myVMin;
    }

    public void VMin(double V)
    {
        BRepPrim_OneAxis_Check(VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
        myVMin = V;
    }

    public double VMax()
    {
        return myVMax;
    }

    public void VMax(double V)
    {
        BRepPrim_OneAxis_Check(VerticesBuilt, EdgesBuilt, WiresBuilt, FacesBuilt);
        myVMax = V;
    }

    /// <summary>A face with no edges on the lateral surface, normals pointing outward; U is the angle, V the meridian's parameter.</summary>
    public abstract Face MakeEmptyLateralFace();

    /// <summary>An edge with a 3D curve made from the meridian rotated by <paramref name="Ang"/> (0 or the angle) around the axis.</summary>
    public abstract Edge MakeEmptyMeridianEdge(double Ang);

    /// <summary>Sets the pcurve of the edge on the face to the 2D representation of the meridian.</summary>
    public abstract void SetMeridianPCurve(in Edge E, in Face F);

    /// <summary>The meridian point at parameter <paramref name="V"/> in the plane XZ.</summary>
    public abstract Point2D MeridianValue(double V);

    /// <summary>True if the point of parameter <paramref name="V"/> on the meridian is on the axis.</summary>
    public virtual bool MeridianOnAxis(double V)
    {
        return Math.Abs(MeridianValue(V).X()) < global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>True if the meridian is closed.</summary>
    public virtual bool MeridianClosed()
    {
        if (VMaxInfinite())
        {
            return false;
        }
        if (VMinInfinite())
        {
            return false;
        }
        return MeridianValue(myVMin).IsEqual(MeridianValue(myVMax), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>True if VMax is infinite.</summary>
    public virtual bool VMaxInfinite()
    {
        return global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(myVMax);
    }

    /// <summary>True if VMin is infinite.</summary>
    public virtual bool VMinInfinite()
    {
        return global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(myVMin);
    }

    // True if the geometric height ordering is inverted (MeridianValue(VMax).Y() < MeridianValue(VMin).Y()): VMin is
    // then the geometric top. This can occur with a torus on a V range like (90deg, 270deg).
    private bool isHeightInverted()
    {
        if (VMaxInfinite() || VMinInfinite())
        {
            return false;
        }
        double yMax = MeridianValue(myVMax).Y();
        double yMin = MeridianValue(myVMin).Y();
        // Use tolerance to avoid floating-point issues (e.g., for closed meridian where yMax ~ yMin)
        return yMax < yMin - global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    // True if the heights at VMin and VMax are equal (within tolerance): the AxisEdge would be degenerate.
    private bool areHeightsEqual()
    {
        if (VMaxInfinite() || VMinInfinite())
        {
            return false;
        }
        double yMax = MeridianValue(myVMax).Y();
        double yMin = MeridianValue(myVMin).Y();
        return Math.Abs(yMax - yMin) < global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>True if there is a top face: neither VMaxInfinite, MeridianClosed nor MeridianOnAxis(VMax).</summary>
    public virtual bool HasTop()
    {
        if (VMaxInfinite())
        {
            return false;
        }
        if (MeridianClosed())
        {
            return false;
        }
        if (MeridianOnAxis(myVMax))
        {
            return false;
        }
        return true;
    }

    /// <summary>True if there is a bottom face: neither VMinInfinite, MeridianClosed nor MeridianOnAxis(VMin).</summary>
    public virtual bool HasBottom()
    {
        if (VMinInfinite())
        {
            return false;
        }
        if (MeridianClosed())
        {
            return false;
        }
        if (MeridianOnAxis(myVMin))
        {
            return false;
        }
        return true;
    }

    /// <summary>True if there are start and end faces: 2π - Angle > Precision::Angular().</summary>
    public virtual bool HasSides()
    {
        return 2 * M_PI - myAngle > global::Kerf.Engine.Foundation.Precision.Angular();
    }

    /// <summary>The shell containing all the faces of the primitive (unfrozen).</summary>
    public Shell Shell()
    {
        if (!ShellBuilt)
        {
            myBuilder.MakeShell(ref myShell);

            myBuilder.AddShellFace(myShell, LateralFace());
            if (HasTop())
            {
                myBuilder.AddShellFace(myShell, TopFace());
            }
            if (HasBottom())
            {
                myBuilder.AddShellFace(myShell, BottomFace());
            }
            if (HasSides())
            {
                myBuilder.AddShellFace(myShell, StartFace());
                myBuilder.AddShellFace(myShell, EndFace());
            }

            myShell.Closed(Tool.IsClosed(myShell));
            myBuilder.CompleteShell(myShell);
            ShellBuilt = true;
        }
        return myShell;
    }

    /// <summary>The lateral face, oriented toward the outside of the primitive.</summary>
    public Face LateralFace()
    {
        // do it if not done
        if (!FacesBuilt[FLATERAL])
        {
            // build an empty lateral face
            myFaces[FLATERAL] = MakeEmptyLateralFace();

            // add the wires
            if (VMaxInfinite() && VMinInfinite())
            {
                myBuilder.AddFaceWire(myFaces[FLATERAL], LateralStartWire());
                myBuilder.AddFaceWire(myFaces[FLATERAL], LateralEndWire());
            }
            else
            {
                myBuilder.AddFaceWire(myFaces[FLATERAL], LateralWire());
            }

            // put the parametric curves
            if (MeridianClosed())
            {
                // closed edge
                myBuilder.SetPCurve(myEdges[ETOP],
                                    myFaces[FLATERAL],
                                    new Line2D(new Point2D(0, myVMin), new Direction2D(Direction2D.D.X)),
                                    new Line2D(new Point2D(0, myVMax), new Direction2D(Direction2D.D.X)));
            }
            else
            {
                if (!VMaxInfinite())
                {
                    myBuilder.SetPCurve(myEdges[ETOP],
                                        myFaces[FLATERAL],
                                        new Line2D(new Point2D(0, myVMax), new Direction2D(Direction2D.D.X)));
                    if (!HasSides() || MeridianOnAxis(myVMax))
                    {
                        // closed edge set parameters
                        myBuilder.SetParameters(myEdges[ETOP], TopEndVertex(), 0.0, myAngle);
                    }
                }
                if (!VMinInfinite())
                {
                    myBuilder.SetPCurve(myEdges[EBOTTOM],
                                        myFaces[FLATERAL],
                                        new Line2D(new Point2D(0, myVMin), new Direction2D(Direction2D.D.X)));
                    if (!HasSides() || MeridianOnAxis(myVMin))
                    {
                        // closed edge set parameters
                        myBuilder.SetParameters(myEdges[EBOTTOM], BottomEndVertex(), 0.0, myAngle);
                    }
                }
            }
            if (HasSides())
            {
                myBuilder.SetPCurve(myEdges[ESTART],
                                    myFaces[FLATERAL],
                                    new Line2D(new Point2D(0, -myMeridianOffset), new Direction2D(Direction2D.D.Y)));

                myBuilder.SetPCurve(myEdges[EEND],
                                    myFaces[FLATERAL],
                                    new Line2D(new Point2D(myAngle, -myMeridianOffset), new Direction2D(Direction2D.D.Y)));
            }
            else
            {
                // closed edge
                myBuilder.SetPCurve(myEdges[ESTART],
                                    myFaces[FLATERAL],
                                    new Line2D(new Point2D(myAngle, -myMeridianOffset), new Direction2D(Direction2D.D.Y)),
                                    new Line2D(new Point2D(0, -myMeridianOffset), new Direction2D(Direction2D.D.Y)));
            }
            myBuilder.CompleteFace(myFaces[FLATERAL]);
            FacesBuilt[FLATERAL] = true;
        }
        return myFaces[FLATERAL];
    }

    /// <summary>The top planar face, oriented toward +Z (outside).</summary>
    public Face TopFace()
    {
        // do it if not done
        if (!FacesBuilt[FTOP])
        {
            Standard_DomainError_Raise_if(!HasTop(), "BRepPrim_OneAxis::TopFace:No top face");

            // make the empty face by translating the axes
            double z = MeridianValue(myVMax).Y();
            Vector3D V = new(myAxes.Direction());
            V.Multiply(z);
            myBuilder.MakeFace(ref myFaces[FTOP], new Plane3D(new CoordinateSystem(myAxes.Translated(V))));

            myBuilder.AddFaceWire(myFaces[FTOP], TopWire());

            // put the parametric curves
            myBuilder.SetPCurve(myEdges[ETOP],
                                myFaces[FTOP],
                                new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)),
                                              MeridianValue(myVMax).X()));
            if (HasSides())
            {
                myBuilder.SetPCurve(myEdges[ETOPSTART],
                                    myFaces[FTOP],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)));
                myBuilder.SetPCurve(myEdges[ETOPEND],
                                    myFaces[FTOP],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Math.Cos(myAngle), Math.Sin(myAngle))));
            }
            myBuilder.CompleteFace(myFaces[FTOP]);
            FacesBuilt[FTOP] = true;
        }
        return myFaces[FTOP];
    }

    /// <summary>The bottom planar face, oriented toward -Z (outside).</summary>
    public Face BottomFace()
    {
        // do it if not done
        if (!FacesBuilt[FBOTTOM])
        {
            Standard_DomainError_Raise_if(!HasBottom(), "BRepPrim_OneAxis::BottomFace:No bottom face");

            // make the empty face by translating the axes
            double z = MeridianValue(myVMin).Y();
            Vector3D V = new(myAxes.Direction());
            V.Multiply(z);
            Frame3D axes = myAxes.Translated(V);
            myBuilder.MakeFace(ref myFaces[FBOTTOM], new Plane3D(new CoordinateSystem(axes)));
            myBuilder.ReverseFace(ref myFaces[FBOTTOM]);
            myBuilder.AddFaceWire(myFaces[FBOTTOM], BottomWire());

            // put the parametric curves
            myBuilder.SetPCurve(myEdges[EBOTTOM],
                                myFaces[FBOTTOM],
                                new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)),
                                              MeridianValue(myVMin).X()));
            if (HasSides())
            {
                myBuilder.SetPCurve(myEdges[EBOTSTART],
                                    myFaces[FBOTTOM],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)));
                myBuilder.SetPCurve(myEdges[EBOTEND],
                                    myFaces[FBOTTOM],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Math.Cos(myAngle), Math.Sin(myAngle))));
            }
            myBuilder.CompleteFace(myFaces[FBOTTOM]);
            FacesBuilt[FBOTTOM] = true;
        }
        return myFaces[FBOTTOM];
    }

    /// <summary>The face starting the slice, oriented toward the exterior.</summary>
    public Face StartFace()
    {
        // do it if not done
        if (!FacesBuilt[FSTART])
        {
            Standard_DomainError_Raise_if(!HasSides(), "BRepPrim_OneAxes::StartFace:No side faces");

            // build the empty face, perpendicular to myTool.Axes()
            Frame3D axes = new(myAxes.Location(), myAxes.YDirection().Reversed(), myAxes.XDirection());
            myBuilder.MakeFace(ref myFaces[FSTART], new Plane3D(new CoordinateSystem(axes)));

            // When heights are inverted, the wire traversal direction changes due to the AxisEdge running in the
            // opposite direction. Reverse the face to compensate. For equal heights with open meridian (not closed),
            // AxisEdge is skipped. For closed meridian, areHeightsEqual() may be true but the wire structure doesn't
            // change, so the equal heights reversal doesn't apply.
            if (isHeightInverted() || (areHeightsEqual() && !MeridianClosed()))
            {
                myBuilder.ReverseFace(ref myFaces[FSTART]);
            }

            if (VMaxInfinite() && VMinInfinite())
            {
                myBuilder.AddFaceWire(myFaces[FSTART], AxisStartWire());
            }

            myBuilder.AddFaceWire(myFaces[FSTART], StartWire());

            // parametric curves
            SetMeridianPCurve(myEdges[ESTART], myFaces[FSTART]);
            // Skip AxisEdge pcurve when heights are equal since the edge is not in the wire
            if (EdgesBuilt[EAXIS] && !areHeightsEqual())
            {
                myBuilder.SetPCurve(myEdges[EAXIS],
                                    myFaces[FSTART],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Direction2D.D.Y)));
            }

            // When heights are equal, the pcurves for TopStart and BotStart edges would overlap at Y = height. Use
            // small offsets to separate them.
            double aPCurveOffset = areHeightsEqual() ? global::Kerf.Engine.Foundation.Precision.Confusion() * THE_PCURVE_OFFSET_FACTOR : 0.0;
            if (EdgesBuilt[ETOPSTART])
            {
                myBuilder.SetPCurve(myEdges[ETOPSTART],
                                    myFaces[FSTART],
                                    new Line2D(new Point2D(0, MeridianValue(myVMax).Y() + aPCurveOffset), new Direction2D(Direction2D.D.X)));
            }
            if (EdgesBuilt[EBOTSTART])
            {
                myBuilder.SetPCurve(myEdges[EBOTSTART],
                                    myFaces[FSTART],
                                    new Line2D(new Point2D(0, MeridianValue(myVMin).Y() - aPCurveOffset), new Direction2D(Direction2D.D.X)));
            }
            myBuilder.CompleteFace(myFaces[FSTART]);
            FacesBuilt[FSTART] = true;
        }
        return myFaces[FSTART];
    }

    /// <summary>The face ending the slice, oriented toward the exterior.</summary>
    public Face EndFace()
    {
        // do it if not done
        if (!FacesBuilt[FEND])
        {
            Standard_DomainError_Raise_if(!HasSides(), "BRepPrim_OneAxes::EndFace:No side faces");

            // build the empty face, perpendicular to myTool.Axes()
            Frame3D axes = new(myAxes.Location(), myAxes.YDirection().Reversed(), myAxes.XDirection());
            axes.Rotate(myAxes.Axis(), myAngle);
            myBuilder.MakeFace(ref myFaces[FEND], new Plane3D(new CoordinateSystem(axes)));

            // When heights are inverted, the wire traversal direction changes due to the AxisEdge running in the
            // opposite direction. Don't reverse the face in this case. For equal heights with open meridian (not
            // closed), AxisEdge is skipped. For closed meridian, the equal heights reversal doesn't apply.
            if (!isHeightInverted() && (!areHeightsEqual() || MeridianClosed()))
            {
                myBuilder.ReverseFace(ref myFaces[FEND]);
            }

            if (VMaxInfinite() && VMinInfinite())
            {
                myBuilder.AddFaceWire(myFaces[FEND], AxisEndWire());
            }
            myBuilder.AddFaceWire(myFaces[FEND], EndWire());

            // parametric curves
            SetMeridianPCurve(myEdges[EEND], myFaces[FEND]);
            // Skip AxisEdge pcurve when heights are equal since the edge is not in the wire
            if (EdgesBuilt[EAXIS] && !areHeightsEqual())
            {
                myBuilder.SetPCurve(myEdges[EAXIS],
                                    myFaces[FEND],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Direction2D.D.Y)));
            }

            // When heights are equal, the pcurves for TopEnd and BotEnd edges would overlap at Y = height. Use small
            // offsets to separate them.
            double aPCurveOffset = areHeightsEqual() ? global::Kerf.Engine.Foundation.Precision.Confusion() * THE_PCURVE_OFFSET_FACTOR : 0.0;
            if (EdgesBuilt[ETOPEND])
            {
                myBuilder.SetPCurve(myEdges[ETOPEND],
                                    myFaces[FEND],
                                    new Line2D(new Point2D(0, MeridianValue(myVMax).Y() + aPCurveOffset), new Direction2D(Direction2D.D.X)));
            }
            if (EdgesBuilt[EBOTEND])
            {
                myBuilder.SetPCurve(myEdges[EBOTEND],
                                    myFaces[FEND],
                                    new Line2D(new Point2D(0, MeridianValue(myVMin).Y() - aPCurveOffset), new Direction2D(Direction2D.D.X)));
            }
            myBuilder.CompleteFace(myFaces[FEND]);
            FacesBuilt[FEND] = true;
        }
        return myFaces[FEND];
    }

    /// <summary>The wire in the lateral face.</summary>
    public Wire LateralWire()
    {
        // do it if not done
        if (!WiresBuilt[WLATERAL])
        {
            myBuilder.MakeWire(ref myWires[WLATERAL]);

            if (!VMaxInfinite())
            {
                myBuilder.AddWireEdge(myWires[WLATERAL], TopEdge(), false);
            }
            myBuilder.AddWireEdge(myWires[WLATERAL], EndEdge(), true);
            if (!VMinInfinite())
            {
                myBuilder.AddWireEdge(myWires[WLATERAL], BottomEdge(), true);
            }
            myBuilder.AddWireEdge(myWires[WLATERAL], StartEdge(), false);

            myBuilder.CompleteWire(myWires[WLATERAL]);
            WiresBuilt[WLATERAL] = true;
        }
        return myWires[WLATERAL];
    }

    /// <summary>The wire in the lateral face with the start edge.</summary>
    public Wire LateralStartWire()
    {
        // do it if not done
        if (!WiresBuilt[WLATERALSTART])
        {
            myBuilder.MakeWire(ref myWires[WLATERALSTART]);

            myBuilder.AddWireEdge(myWires[WLATERALSTART], StartEdge(), false);

            myBuilder.CompleteWire(myWires[WLATERALSTART]);
            WiresBuilt[WLATERALSTART] = true;
        }
        return myWires[WLATERALSTART];
    }

    /// <summary>The wire in the lateral face with the end edge.</summary>
    public Wire LateralEndWire()
    {
        // do it if not done
        if (!WiresBuilt[WLATERALEND])
        {
            myBuilder.MakeWire(ref myWires[WLATERALEND]);

            myBuilder.AddWireEdge(myWires[WLATERALEND], EndEdge(), true);

            myBuilder.CompleteWire(myWires[WLATERALEND]);
            WiresBuilt[WLATERALEND] = true;
        }
        return myWires[WLATERALEND];
    }

    /// <summary>The wire in the top face.</summary>
    public Wire TopWire()
    {
        // do it if not done
        if (!WiresBuilt[WTOP])
        {
            Standard_DomainError_Raise_if(!HasTop(), "BRepPrim_OneAxis::TopWire: no top");

            myBuilder.MakeWire(ref myWires[WTOP]);

            myBuilder.AddWireEdge(myWires[WTOP], TopEdge(), true);
            if (HasSides())
            {
                myBuilder.AddWireEdge(myWires[WTOP], StartTopEdge(), true);
                myBuilder.AddWireEdge(myWires[WTOP], EndTopEdge(), false);
            }
            myBuilder.CompleteWire(myWires[WTOP]);
            WiresBuilt[WTOP] = true;
        }
        return myWires[WTOP];
    }

    /// <summary>The wire in the bottom face.</summary>
    public Wire BottomWire()
    {
        // do it if not done
        if (!WiresBuilt[WBOTTOM])
        {
            Standard_DomainError_Raise_if(!HasBottom(), "BRepPrim_OneAxis::BottomWire: no bottom");

            myBuilder.MakeWire(ref myWires[WBOTTOM]);

            myBuilder.AddWireEdge(myWires[WBOTTOM], BottomEdge(), false);
            if (HasSides())
            {
                myBuilder.AddWireEdge(myWires[WBOTTOM], EndBottomEdge(), true);
                myBuilder.AddWireEdge(myWires[WBOTTOM], StartBottomEdge(), false);
            }

            myBuilder.CompleteWire(myWires[WBOTTOM]);
            WiresBuilt[WBOTTOM] = true;
        }
        return myWires[WBOTTOM];
    }

    /// <summary>The wire in the start face.</summary>
    public Wire StartWire()
    {
        // do it if not done
        if (!WiresBuilt[WSTART])
        {
            Standard_DomainError_Raise_if(!HasSides(), "BRepPrim_OneAxes::StartWire:no sides");

            myBuilder.MakeWire(ref myWires[WSTART]);

            // When heights are inverted, the AxisEdge direction is flipped: flip its reversed flag to keep the wire
            // connected. When heights are equal, skip the degenerate AxisEdge: the wire still closes because
            // AxisTopVertex and AxisBottomVertex are the same vertex.
            bool isInverted = isHeightInverted();
            bool heightsEqual = areHeightsEqual();

            if (HasBottom())
            {
                myBuilder.AddWireEdge(myWires[WSTART], StartBottomEdge(), true);
            }

            if (!MeridianClosed() && !heightsEqual)
            {
                if (!VMaxInfinite() || !VMinInfinite())
                {
                    myBuilder.AddWireEdge(myWires[WSTART], AxisEdge(), isInverted);
                }
            }

            if (HasTop())
            {
                myBuilder.AddWireEdge(myWires[WSTART], StartTopEdge(), false);
            }
            myBuilder.AddWireEdge(myWires[WSTART], StartEdge(), true);

            myBuilder.CompleteWire(myWires[WSTART]);
            WiresBuilt[WSTART] = true;
        }
        return myWires[WSTART];
    }

    /// <summary>The wire in the start face with the AxisEdge.</summary>
    public Wire AxisStartWire()
    {
        // do it if not done
        if (!WiresBuilt[WAXISSTART])
        {
            Standard_DomainError_Raise_if(!HasSides(), "BRepPrim_OneAxes::AxisStartWire:no sides");

            Standard_DomainError_Raise_if(!VMaxInfinite() || !VMinInfinite(),
                                          "BRepPrim_OneAxes::AxisStartWire:not infinite");

            Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxes::AxisStartWire:meridian closed");

            myBuilder.MakeWire(ref myWires[WAXISSTART]);

            myBuilder.AddWireEdge(myWires[WAXISSTART], AxisEdge(), false);

            myBuilder.CompleteWire(myWires[WAXISSTART]);
            WiresBuilt[WAXISSTART] = true;
        }
        return myWires[WAXISSTART];
    }

    /// <summary>The wire in the end face.</summary>
    public Wire EndWire()
    {
        // do it if not done
        if (!WiresBuilt[WEND])
        {
            Standard_DomainError_Raise_if(!HasSides(), "BRepPrim_OneAxes::EndWire:no sides");

            myBuilder.MakeWire(ref myWires[WEND]);

            // When heights are inverted, the AxisEdge direction is flipped: flip its reversed flag to keep the wire
            // connected. When heights are equal, skip the degenerate AxisEdge: the wire still closes because
            // AxisTopVertex and AxisBottomVertex are the same vertex.
            bool isInverted = isHeightInverted();
            bool heightsEqual = areHeightsEqual();

            if (HasTop())
            {
                myBuilder.AddWireEdge(myWires[WEND], EndTopEdge(), true);
            }
            if (!MeridianClosed() && !heightsEqual)
            {
                if (!VMaxInfinite() || !VMinInfinite())
                {
                    myBuilder.AddWireEdge(myWires[WEND], AxisEdge(), !isInverted);
                }
            }
            if (HasBottom())
            {
                myBuilder.AddWireEdge(myWires[WEND], EndBottomEdge(), false);
            }
            myBuilder.AddWireEdge(myWires[WEND], EndEdge(), false);

            myBuilder.CompleteWire(myWires[WEND]);
            WiresBuilt[WEND] = true;
        }
        return myWires[WEND];
    }

    /// <summary>The wire in the end face with the AxisEdge.</summary>
    public Wire AxisEndWire()
    {
        // do it if not done
        if (!WiresBuilt[WAXISEND])
        {
            Standard_DomainError_Raise_if(!HasSides(), "BRepPrim_OneAxes::AxisEndWire:no sides");

            Standard_DomainError_Raise_if(!VMaxInfinite() || !VMinInfinite(),
                                          "BRepPrim_OneAxes::AxisEndWire:not infinite");

            Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxes::AxisEndWire:meridian closed");

            myBuilder.MakeWire(ref myWires[WAXISEND]);

            myBuilder.AddWireEdge(myWires[WAXISEND], AxisEdge(), true);

            myBuilder.CompleteWire(myWires[WAXISEND]);
            WiresBuilt[WAXISEND] = true;
        }
        return myWires[WAXISEND];
    }

    /// <summary>The edge along the axis, oriented on +Z.</summary>
    public Edge AxisEdge()
    {
        // do it if not done
        if (!EdgesBuilt[EAXIS])
        {
            Standard_DomainError_Raise_if(!HasSides(), "BRepPrim_OneAxis::AxisEdge:no sides");
            Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxis::AxisEdge:closed");

            if (areHeightsEqual())
            {
                // When heights are equal, the AxisEdge is degenerate (zero length): a degenerate edge at the shared
                // vertex.
                double aHeight = MeridianValue(myVMax).Y();
                myBuilder.MakeDegeneratedEdge(ref myEdges[EAXIS]);
                myBuilder.AddEdgeVertex(myEdges[EAXIS], AxisTopVertex(), aHeight, aHeight);
            }
            else
            {
                // build the empty edge.
                myBuilder.MakeEdge(ref myEdges[EAXIS], new Line3D(myAxes.Axis()));

                // When heights are inverted (VMax is geometrically below VMin), swap the first flags so the edge's
                // parameter range is increasing: the edge always runs from the geometrically lower vertex up.
                bool isInverted = isHeightInverted();
                double yMax = MeridianValue(myVMax).Y();
                double yMin = MeridianValue(myVMin).Y();

                if (!VMaxInfinite())
                {
                    myBuilder.AddEdgeVertex(myEdges[EAXIS], AxisTopVertex(), yMax, isInverted);
                }
                if (!VMinInfinite())
                {
                    myBuilder.AddEdgeVertex(myEdges[EAXIS], AxisBottomVertex(), yMin, !isInverted);
                }
            }

            myBuilder.CompleteEdge(myEdges[EAXIS]);
            EdgesBuilt[EAXIS] = true;
        }
        return myEdges[EAXIS];
    }

    /// <summary>The edge at angle 0.</summary>
    public Edge StartEdge()
    {
        // do it if not done
        if (!EdgesBuilt[ESTART])
        {
            // is it shared with the EndEdge
            if (!HasSides() && EdgesBuilt[EEND])
            {
                myEdges[ESTART] = myEdges[EEND];
            }
            else
            {
                // build the empty Edge
                myEdges[ESTART] = MakeEmptyMeridianEdge(0.0);

                if (MeridianClosed())
                {
                    // Closed edge
                    myBuilder.AddEdgeVertex(myEdges[ESTART],
                                            TopStartVertex(),
                                            myVMin + myMeridianOffset,
                                            myVMax + myMeridianOffset);
                }
                else
                {
                    if (!VMaxInfinite())
                    {
                        myBuilder.AddEdgeVertex(myEdges[ESTART], TopStartVertex(), myVMax + myMeridianOffset, false);
                    }
                    if (!VMinInfinite())
                    {
                        myBuilder.AddEdgeVertex(myEdges[ESTART], BottomStartVertex(), myVMin + myMeridianOffset, true);
                    }
                }
            }

            myBuilder.CompleteEdge(myEdges[ESTART]);
            EdgesBuilt[ESTART] = true;
        }
        return myEdges[ESTART];
    }

    /// <summary>The edge at angle Angle. Without sides, the start and end edges are the same edge.</summary>
    public Edge EndEdge()
    {
        // do it if not done
        if (!EdgesBuilt[EEND])
        {
            // is it shared with the start edge
            if (!HasSides() && EdgesBuilt[ESTART])
            {
                myEdges[EEND] = myEdges[ESTART];
            }
            else
            {
                // build the empty Edge
                myEdges[EEND] = MakeEmptyMeridianEdge(myAngle);

                if (MeridianClosed())
                {
                    // Closed edge
                    myBuilder.AddEdgeVertex(myEdges[EEND],
                                            TopEndVertex(),
                                            myVMin + myMeridianOffset,
                                            myVMax + myMeridianOffset);
                }
                else
                {
                    if (!VMaxInfinite())
                    {
                        myBuilder.AddEdgeVertex(myEdges[EEND], TopEndVertex(), myVMax + myMeridianOffset, false);
                    }
                    if (!VMinInfinite())
                    {
                        myBuilder.AddEdgeVertex(myEdges[EEND], BottomEndVertex(), myVMin + myMeridianOffset, true);
                    }
                }
            }

            myBuilder.CompleteEdge(myEdges[EEND]);
            EdgesBuilt[EEND] = true;
        }
        return myEdges[EEND];
    }

    /// <summary>The linear edge between the start face and the top face.</summary>
    public Edge StartTopEdge()
    {
        // do it if not done
        if (!EdgesBuilt[ETOPSTART])
        {
            Standard_DomainError_Raise_if(!HasTop() || !HasSides(), "BRepPrim_OneAxis::StartTopEdge:no sides or no top");

            // build the empty Edge
            Vector3D V = new(myAxes.Direction());
            V.Multiply(MeridianValue(myVMax).Y());
            Point3D P = myAxes.Location().Translated(V);
            myBuilder.MakeEdge(ref myEdges[ETOPSTART], new Line3D(P, myAxes.XDirection()));

            myBuilder.AddEdgeVertex(myEdges[ETOPSTART], AxisTopVertex(), 0.0, true);
            myBuilder.AddEdgeVertex(myEdges[ETOPSTART], TopStartVertex(), MeridianValue(myVMax).X(), false);

            myBuilder.CompleteEdge(myEdges[ETOPSTART]);
            EdgesBuilt[ETOPSTART] = true;
        }
        return myEdges[ETOPSTART];
    }

    /// <summary>The linear edge between the start face and the bottom face.</summary>
    public Edge StartBottomEdge()
    {
        // do it if not done
        if (!EdgesBuilt[EBOTSTART])
        {
            Standard_DomainError_Raise_if(!HasBottom() || !HasSides(), "BRepPrim_OneAxis::StartBottomEdge:no sides or no top");

            // build the empty Edge
            Vector3D V = new(myAxes.Direction());
            V.Multiply(MeridianValue(myVMin).Y());
            Point3D P = myAxes.Location().Translated(V);
            myBuilder.MakeEdge(ref myEdges[EBOTSTART], new Line3D(P, myAxes.XDirection()));

            myBuilder.AddEdgeVertex(myEdges[EBOTSTART], BottomStartVertex(), MeridianValue(myVMin).X(), false);
            myBuilder.AddEdgeVertex(myEdges[EBOTSTART], AxisBottomVertex(), 0.0, true);

            myBuilder.CompleteEdge(myEdges[EBOTSTART]);
            EdgesBuilt[EBOTSTART] = true;
        }
        return myEdges[EBOTSTART];
    }

    /// <summary>The linear edge between the end face and the top face.</summary>
    public Edge EndTopEdge()
    {
        // do it if not done
        if (!EdgesBuilt[ETOPEND])
        {
            Standard_DomainError_Raise_if(!HasTop() || !HasSides(), "BRepPrim_OneAxis::EndTopEdge:no sides or no top");

            // build the empty Edge
            Vector3D V = new(myAxes.Direction());
            V.Multiply(MeridianValue(myVMax).Y());
            Point3D P = myAxes.Location().Translated(V);
            Line3D L = new(P, myAxes.XDirection());
            L.Rotate(myAxes.Axis(), myAngle);
            myBuilder.MakeEdge(ref myEdges[ETOPEND], L);

            myBuilder.AddEdgeVertex(myEdges[ETOPEND], AxisTopVertex(), 0.0, true);
            myBuilder.AddEdgeVertex(myEdges[ETOPEND], TopEndVertex(), MeridianValue(myVMax).X(), false);

            myBuilder.CompleteEdge(myEdges[ETOPEND]);
            EdgesBuilt[ETOPEND] = true;
        }
        return myEdges[ETOPEND];
    }

    /// <summary>The linear edge between the end face and the bottom face.</summary>
    public Edge EndBottomEdge()
    {
        // do it if not done
        if (!EdgesBuilt[EBOTEND])
        {
            Standard_DomainError_Raise_if(!HasBottom() || !HasSides(), "BRepPrim_OneAxis::EndBottomEdge:no sides or no bottom");

            // build the empty Edge
            Vector3D V = new(myAxes.Direction());
            V.Multiply(MeridianValue(myVMin).Y());
            Point3D P = myAxes.Location().Translated(V);
            Line3D L = new(P, myAxes.XDirection());
            L.Rotate(myAxes.Axis(), myAngle);
            myBuilder.MakeEdge(ref myEdges[EBOTEND], L);

            myBuilder.AddEdgeVertex(myEdges[EBOTEND], AxisBottomVertex(), 0.0, true);
            myBuilder.AddEdgeVertex(myEdges[EBOTEND], BottomEndVertex(), MeridianValue(myVMin).X(), false);

            myBuilder.CompleteEdge(myEdges[EBOTEND]);
            EdgesBuilt[EBOTEND] = true;
        }
        return myEdges[EBOTEND];
    }

    /// <summary>The edge at VMax. If the meridian is closed, the top and bottom edges are the same edge.</summary>
    public Edge TopEdge()
    {
        // do it if not done
        if (!EdgesBuilt[ETOP])
        {
            // Test if shared with bottom edge
            if (MeridianClosed() && EdgesBuilt[EBOTTOM])
            {
                myEdges[ETOP] = myEdges[EBOTTOM];
            }
            else
            {
                // build the empty Edge
                if (!MeridianOnAxis(myVMax))
                {
                    Point2D mp = MeridianValue(myVMax);
                    Vector3D V = new(myAxes.Direction());
                    V.Multiply(mp.Y());
                    Point3D P = myAxes.Location().Translated(V);
                    Circle3D C = new(new Frame3D(P, myAxes.Direction(), myAxes.XDirection()), mp.X());
                    myBuilder.MakeEdge(ref myEdges[ETOP], C);
                }
                else
                {
                    myBuilder.MakeDegeneratedEdge(ref myEdges[ETOP]);
                }

                if (!HasSides())
                {
                    // closed edge
                    myBuilder.AddEdgeVertex(myEdges[ETOP], TopEndVertex(), 0.0, myAngle);
                }
                else
                {
                    myBuilder.AddEdgeVertex(myEdges[ETOP], TopEndVertex(), myAngle, false);
                    myBuilder.AddEdgeVertex(myEdges[ETOP], TopStartVertex(), 0.0, true);
                }
            }

            myBuilder.CompleteEdge(myEdges[ETOP]);
            EdgesBuilt[ETOP] = true;
        }
        return myEdges[ETOP];
    }

    /// <summary>The edge at VMin. If the meridian is closed, the top and bottom edges are the same edge.</summary>
    public Edge BottomEdge()
    {
        // do it if not done
        if (!EdgesBuilt[EBOTTOM])
        {
            // Test if shared with top edge
            if (MeridianClosed() && EdgesBuilt[ETOP])
            {
                myEdges[EBOTTOM] = myEdges[ETOP];
            }
            else
            {
                // build the empty Edge
                if (!MeridianOnAxis(myVMin))
                {
                    Point2D mp = MeridianValue(myVMin);
                    Vector3D V = new(myAxes.Direction());
                    V.Multiply(mp.Y());
                    Point3D P = myAxes.Location().Translated(V);
                    Circle3D C = new(new Frame3D(P, myAxes.Direction(), myAxes.XDirection()), mp.X());
                    myBuilder.MakeEdge(ref myEdges[EBOTTOM], C);
                }
                else
                {
                    myBuilder.MakeDegeneratedEdge(ref myEdges[EBOTTOM]);
                }

                if (!HasSides())
                {
                    // closed edge
                    myBuilder.AddEdgeVertex(myEdges[EBOTTOM], BottomEndVertex(), 0.0, myAngle);
                }
                else
                {
                    myBuilder.AddEdgeVertex(myEdges[EBOTTOM], BottomEndVertex(), myAngle, false);
                    myBuilder.AddEdgeVertex(myEdges[EBOTTOM], BottomStartVertex(), 0.0, true);
                }
            }

            myBuilder.CompleteEdge(myEdges[EBOTTOM]);
            EdgesBuilt[EBOTTOM] = true;
        }
        return myEdges[EBOTTOM];
    }

    /// <summary>The vertex at the top altitude on the axis.</summary>
    public Vertex AxisTopVertex()
    {
        // do it if not done
        if (!VerticesBuilt[VAXISTOP])
        {
            // deduct from others
            if (MeridianOnAxis(myVMax) && VerticesBuilt[VTOPSTART])
            {
                myVertices[VAXISTOP] = myVertices[VTOPSTART];
            }
            else if (MeridianOnAxis(myVMax) && VerticesBuilt[VTOPEND])
            {
                myVertices[VAXISTOP] = myVertices[VTOPEND];
            }
            // Share with AxisBottomVertex if heights are equal (handles case when bottom is built first)
            else if (VerticesBuilt[VAXISBOT] && areHeightsEqual())
            {
                myVertices[VAXISTOP] = myVertices[VAXISBOT];
            }
            else
            {
                Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxis::AxisTopVertex");
                Standard_DomainError_Raise_if(VMaxInfinite(), "BRepPrim_OneAxis::AxisTopVertex");

                Vector3D V = new(myAxes.Direction());
                V.Multiply(MeridianValue(myVMax).Y());
                Point3D P = myAxes.Location().Translated(V);
                myBuilder.MakeVertex(ref myVertices[VAXISTOP], P);
            }

            VerticesBuilt[VAXISTOP] = true;
        }
        return myVertices[VAXISTOP];
    }

    /// <summary>The vertex at the bottom altitude on the axis.</summary>
    public Vertex AxisBottomVertex()
    {
        // do it if not done
        if (!VerticesBuilt[VAXISBOT])
        {
            // deduct from others
            if (MeridianOnAxis(myVMin) && VerticesBuilt[VBOTSTART])
            {
                myVertices[VAXISBOT] = myVertices[VBOTSTART];
            }
            else if (MeridianOnAxis(myVMin) && VerticesBuilt[VBOTEND])
            {
                myVertices[VAXISBOT] = myVertices[VBOTEND];
            }
            // Share with AxisTopVertex if heights are equal (within tolerance)
            else if (VerticesBuilt[VAXISTOP] && areHeightsEqual())
            {
                myVertices[VAXISBOT] = myVertices[VAXISTOP];
            }
            else
            {
                Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxis::AxisBottomVertex");
                Standard_DomainError_Raise_if(VMinInfinite(), "BRepPrim_OneAxis::AxisBottomVertex");

                Vector3D V = new(myAxes.Direction());
                V.Multiply(MeridianValue(myVMin).Y());
                Point3D P = myAxes.Location().Translated(V);
                myBuilder.MakeVertex(ref myVertices[VAXISBOT], P);
            }

            VerticesBuilt[VAXISBOT] = true;
        }
        return myVertices[VAXISBOT];
    }

    /// <summary>The vertex (0, VMax).</summary>
    public Vertex TopStartVertex()
    {
        // do it if not done
        if (!VerticesBuilt[VTOPSTART])
        {
            // deduct from others
            if (MeridianOnAxis(myVMax) && VerticesBuilt[VAXISTOP])
            {
                myVertices[VTOPSTART] = myVertices[VAXISTOP];
            }
            else if ((MeridianOnAxis(myVMax) || !HasSides()) && VerticesBuilt[VTOPEND])
            {
                myVertices[VTOPSTART] = myVertices[VTOPEND];
            }
            else if (MeridianClosed() && VerticesBuilt[VBOTSTART])
            {
                myVertices[VTOPSTART] = myVertices[VBOTSTART];
            }
            else if ((MeridianClosed() && !HasSides()) && VerticesBuilt[VBOTEND])
            {
                myVertices[VTOPSTART] = myVertices[VBOTEND];
            }
            else
            {
                Point2D mp = MeridianValue(myVMax);
                Vector3D V = new(myAxes.Direction());
                V.Multiply(mp.Y());
                Point3D P = myAxes.Location().Translated(V);
                V = new Vector3D(myAxes.XDirection());
                V.Multiply(mp.X());
                P.Translate(V);
                myBuilder.MakeVertex(ref myVertices[VTOPSTART], P);
            }

            VerticesBuilt[VTOPSTART] = true;
        }
        return myVertices[VTOPSTART];
    }

    /// <summary>The vertex (Angle, VMax).</summary>
    public Vertex TopEndVertex()
    {
        // do it if not done
        if (!VerticesBuilt[VTOPEND])
        {
            // deduct from others
            if (MeridianOnAxis(myVMax) && VerticesBuilt[VAXISTOP])
            {
                myVertices[VTOPEND] = myVertices[VAXISTOP];
            }
            else if ((MeridianOnAxis(myVMax) || !HasSides()) && VerticesBuilt[VTOPSTART])
            {
                myVertices[VTOPEND] = myVertices[VTOPSTART];
            }
            else if (MeridianClosed() && VerticesBuilt[VBOTEND])
            {
                myVertices[VTOPEND] = myVertices[VBOTEND];
            }
            else if ((MeridianClosed() && !HasSides()) && VerticesBuilt[VBOTSTART])
            {
                myVertices[VTOPEND] = myVertices[VBOTSTART];
            }
            else
            {
                Point2D mp = MeridianValue(myVMax);
                Vector3D V = new(myAxes.Direction());
                V.Multiply(mp.Y());
                Point3D P = myAxes.Location().Translated(V);
                V = new Vector3D(myAxes.XDirection());
                V.Multiply(mp.X());
                P.Translate(V);
                P.Rotate(myAxes.Axis(), myAngle);
                myBuilder.MakeVertex(ref myVertices[VTOPEND], P);
            }

            VerticesBuilt[VTOPEND] = true;
        }
        return myVertices[VTOPEND];
    }

    /// <summary>The vertex (0, VMin).</summary>
    public Vertex BottomStartVertex()
    {
        // do it if not done
        if (!VerticesBuilt[VBOTSTART])
        {
            // deduct from others
            if (MeridianOnAxis(myVMin) && VerticesBuilt[VAXISBOT])
            {
                myVertices[VBOTSTART] = myVertices[VAXISBOT];
            }
            else if ((MeridianOnAxis(myVMin) || !HasSides()) && VerticesBuilt[VBOTEND])
            {
                myVertices[VBOTSTART] = myVertices[VBOTEND];
            }
            else if (MeridianClosed() && VerticesBuilt[VTOPSTART])
            {
                myVertices[VBOTSTART] = myVertices[VTOPSTART];
            }
            else if ((MeridianClosed() && !HasSides()) && VerticesBuilt[VTOPEND])
            {
                myVertices[VBOTSTART] = myVertices[VTOPEND];
            }
            else
            {
                Point2D mp = MeridianValue(myVMin);
                Vector3D V = new(myAxes.Direction());
                V.Multiply(mp.Y());
                Point3D P = myAxes.Location().Translated(V);
                V = new Vector3D(myAxes.XDirection());
                V.Multiply(mp.X());
                P.Translate(V);
                myBuilder.MakeVertex(ref myVertices[VBOTSTART], P);
            }

            VerticesBuilt[VBOTSTART] = true;
        }
        return myVertices[VBOTSTART];
    }

    /// <summary>The vertex (Angle, VMin).</summary>
    public Vertex BottomEndVertex()
    {
        // do it if not done
        if (!VerticesBuilt[VBOTEND])
        {
            // deduct from others
            if (MeridianOnAxis(myVMin) && VerticesBuilt[VAXISBOT])
            {
                myVertices[VBOTEND] = myVertices[VAXISBOT];
            }
            else if ((MeridianOnAxis(myVMin) || !HasSides()) && VerticesBuilt[VBOTSTART])
            {
                myVertices[VBOTEND] = myVertices[VBOTSTART];
            }
            else if (MeridianClosed() && VerticesBuilt[VTOPEND])
            {
                myVertices[VBOTEND] = myVertices[VTOPEND];
            }
            else if (MeridianClosed() && !HasSides() && VerticesBuilt[VTOPSTART])
            {
                myVertices[VBOTEND] = myVertices[VTOPSTART];
            }
            else
            {
                Point2D mp = MeridianValue(myVMin);
                Vector3D V = new(myAxes.Direction());
                V.Multiply(mp.Y());
                Point3D P = myAxes.Location().Translated(V);
                V = new Vector3D(myAxes.XDirection());
                V.Multiply(mp.X());
                P.Translate(V);
                P.Rotate(myAxes.Axis(), myAngle);
                myBuilder.MakeVertex(ref myVertices[VBOTEND], P);
            }

            VerticesBuilt[VBOTEND] = true;
        }
        return myVertices[VBOTEND];
    }
}
