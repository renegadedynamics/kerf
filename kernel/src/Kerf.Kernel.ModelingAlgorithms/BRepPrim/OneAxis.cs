// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_OneAxis.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_OneAxis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>
/// Algorithm to build primitives with one axis of revolution: a coordinate system whose Z axis is the axis of
/// revolution, an angle around it (2π: no planar side faces; U ranges from 0 to the angle), a parameter range VMin,
/// VMax on the meridian, and the meridian, described by deferred methods. There are at most 5 faces: the lateral face,
/// the top and bottom faces, and the start and end faces. Building an element builds all its sub-elements, so building
/// the shell builds everything.
/// </summary>
/// <remarks>Internal (decision 465): its shapes are unfrozen; the BRepPrimAPI makers publish them.</remarks>
internal abstract class OneAxis
{
    // Multiplier for Precision::Confusion() to separate overlapping pcurves when heights at VMin and VMax are equal.
    private const double THE_PCURVE_OFFSET_FACTOR = 10.0;

    // Vertex array indices.
    private const int Vaxistop = 0;
    private const int Vaxisbot = 1;
    private const int Vtopstart = 2;
    private const int Vtopend = 3;
    private const int Vbotstart = 4;
    private const int Vbotend = 5;
    private const int Nbvertices = 6;

    // Edge array indices.
    private const int Eaxis = 0;
    private const int Estart = 1;
    private const int Eend = 2;
    private const int Etopstart = 3;
    private const int Etopend = 4;
    private const int Ebotstart = 5;
    private const int Ebotend = 6;
    private const int Etop = 7;
    private const int Ebottom = 8;
    private const int Nbedges = 9;

    // Wire array indices. Index 4 is intentionally skipped to preserve historical array sizing.
    private const int Wlateral = 0;
    private const int Wlateralstart = Wlateral;
    private const int Wlateralend = 1;
    private const int Wtop = 2;
    private const int Wbottom = 3;
    private const int Wstart = 5;
    private const int Waxisstart = 6;
    private const int Wend = 7;
    private const int Waxisend = 8;
    private const int Nbwires = 9;

    // Face array indices.
    private const int Flateral = 0;
    private const int Ftop = 1;
    private const int Fbottom = 2;
    private const int Fstart = 3;
    private const int Fend = 4;
    private const int Nbfaces = 5;

    private static void BRepPrim_OneAxis_Check(bool[] v, bool[] e, bool[] w, bool[] f)
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

    protected readonly PrimitiveBuilder _builder;

    private Frame3D _axes;
    private double _angle;
    private double _vMin;
    private double _vMax;
    private double _meridianOffset;
    private Shell _shell = new();
    private bool _shellBuilt;
    private readonly Vertex[] _vertices = GWedge.NewArray<Vertex>(Nbvertices);
    private readonly bool[] _verticesBuilt = new bool[Nbvertices];
    private readonly Edge[] _edges = GWedge.NewArray<Edge>(Nbedges);
    private readonly bool[] _edgesBuilt = new bool[Nbedges];
    private readonly Wire[] _wires = GWedge.NewArray<Wire>(Nbwires);
    private readonly bool[] _wiresBuilt = new bool[Nbwires];
    private readonly Face[] _faces = GWedge.NewArray<Face>(Nbfaces);
    private readonly bool[] _facesBuilt = new bool[Nbfaces];

    /// <summary>Creates a OneAxis algorithm. <paramref name="b"/> is used to build the Topology. The angle defaults to 2π.</summary>
    protected OneAxis(PrimitiveBuilder b, in Frame3D a, double vMin, double vMax)
    {
        _builder = b;
        _axes = a;
        _angle = 2 * M_PI;
        _vMin = vMin;
        _vMax = vMax;
        _meridianOffset = 0;

        // init Built flags
        int i;
        _shellBuilt = false;
        for (i = 0; i < Nbvertices; i++)
        {
            _verticesBuilt[i] = false;
        }
        for (i = 0; i < Nbedges; i++)
        {
            _edgesBuilt[i] = false;
        }
        for (i = 0; i < Nbwires; i++)
        {
            _wiresBuilt[i] = false;
        }
        for (i = 0; i < Nbfaces; i++)
        {
            _facesBuilt[i] = false;
        }
    }

    /// <summary>
    /// The MeridianOffset is added to the parameters on the meridian curve and to the V values of the pcurves (the
    /// sphere's meridian edge has another range than VMin, VMax).
    /// </summary>
    public void SetMeridianOffset(double o = 0)
    {
        _meridianOffset = o;
    }

    public Frame3D Axes()
    {
        return _axes;
    }

    public void Axes(in Frame3D a)
    {
        BRepPrim_OneAxis_Check(_verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
        _axes = a;
    }

    public double Angle()
    {
        return _angle;
    }

    public void Angle(double a)
    {
        BRepPrim_OneAxis_Check(_verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
        _angle = a;
    }

    public double VMin()
    {
        return _vMin;
    }

    public void VMin(double v)
    {
        BRepPrim_OneAxis_Check(_verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
        _vMin = v;
    }

    public double VMax()
    {
        return _vMax;
    }

    public void VMax(double v)
    {
        BRepPrim_OneAxis_Check(_verticesBuilt, _edgesBuilt, _wiresBuilt, _facesBuilt);
        _vMax = v;
    }

    /// <summary>A face with no edges on the lateral surface, normals pointing outward; U is the angle, V the meridian's parameter.</summary>
    public abstract Face MakeEmptyLateralFace();

    /// <summary>An edge with a 3D curve made from the meridian rotated by <paramref name="ang"/> (0 or the angle) around the axis.</summary>
    public abstract Edge MakeEmptyMeridianEdge(double ang);

    /// <summary>Sets the pcurve of the edge on the face to the 2D representation of the meridian.</summary>
    public abstract void SetMeridianPCurve(in Edge e, in Face f);

    /// <summary>The meridian point at parameter <paramref name="v"/> in the plane XZ.</summary>
    public abstract Point2D MeridianValue(double v);

    /// <summary>True if the point of parameter <paramref name="v"/> on the meridian is on the axis.</summary>
    public virtual bool MeridianOnAxis(double v)
    {
        return Math.Abs(MeridianValue(v).X) < global::Kerf.Engine.Foundation.Precision.Confusion();
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
        return MeridianValue(_vMin).IsEqual(MeridianValue(_vMax), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>True if VMax is infinite.</summary>
    public virtual bool VMaxInfinite()
    {
        return global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_vMax);
    }

    /// <summary>True if VMin is infinite.</summary>
    public virtual bool VMinInfinite()
    {
        return global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_vMin);
    }

    // True if the geometric height ordering is inverted (MeridianValue(VMax).Y() < MeridianValue(VMin).Y()): VMin is
    // then the geometric top. This can occur with a torus on a V range like (90deg, 270deg).
    private bool IsHeightInverted()
    {
        if (VMaxInfinite() || VMinInfinite())
        {
            return false;
        }
        double yMax = MeridianValue(_vMax).Y;
        double yMin = MeridianValue(_vMin).Y;
        // Use tolerance to avoid floating-point issues (e.g., for closed meridian where yMax ~ yMin)
        return yMax < yMin - global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    // True if the heights at VMin and VMax are equal (within tolerance): the AxisEdge would be degenerate.
    private bool AreHeightsEqual()
    {
        if (VMaxInfinite() || VMinInfinite())
        {
            return false;
        }
        double yMax = MeridianValue(_vMax).Y;
        double yMin = MeridianValue(_vMin).Y;
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
        if (MeridianOnAxis(_vMax))
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
        if (MeridianOnAxis(_vMin))
        {
            return false;
        }
        return true;
    }

    /// <summary>True if there are start and end faces: 2π - Angle > Precision::Angular().</summary>
    public virtual bool HasSides
    {
        get
        {
            return 2 * M_PI - _angle > global::Kerf.Engine.Foundation.Precision.Angular();
        }
    }

    /// <summary>The shell containing all the faces of the primitive (unfrozen).</summary>
    public Shell Shell()
    {
        if (!_shellBuilt)
        {
            _builder.MakeShell(ref _shell);

            _builder.AddShellFace(_shell, LateralFace());
            if (HasTop())
            {
                _builder.AddShellFace(_shell, TopFace());
            }
            if (HasBottom())
            {
                _builder.AddShellFace(_shell, BottomFace());
            }
            if (HasSides)
            {
                _builder.AddShellFace(_shell, StartFace());
                _builder.AddShellFace(_shell, EndFace());
            }

            _shell.Closed(Tool.IsClosed(_shell));
            _builder.CompleteShell(_shell);
            _shellBuilt = true;
        }
        return _shell;
    }

    /// <summary>The lateral face, oriented toward the outside of the primitive.</summary>
    public Face LateralFace()
    {
        // do it if not done
        if (!_facesBuilt[Flateral])
        {
            // build an empty lateral face
            _faces[Flateral] = MakeEmptyLateralFace();

            // add the wires
            if (VMaxInfinite() && VMinInfinite())
            {
                _builder.AddFaceWire(_faces[Flateral], LateralStartWire());
                _builder.AddFaceWire(_faces[Flateral], LateralEndWire());
            }
            else
            {
                _builder.AddFaceWire(_faces[Flateral], LateralWire());
            }

            // put the parametric curves
            if (MeridianClosed())
            {
                // closed edge
                _builder.SetPCurve(_edges[Etop],
                                    _faces[Flateral],
                                    new Line2D(new Point2D(0, _vMin), new Direction2D(Direction2D.D.X)),
                                    new Line2D(new Point2D(0, _vMax), new Direction2D(Direction2D.D.X)));
            }
            else
            {
                if (!VMaxInfinite())
                {
                    _builder.SetPCurve(_edges[Etop],
                                        _faces[Flateral],
                                        new Line2D(new Point2D(0, _vMax), new Direction2D(Direction2D.D.X)));
                    if (!HasSides || MeridianOnAxis(_vMax))
                    {
                        // closed edge set parameters
                        _builder.SetParameters(_edges[Etop], TopEndVertex(), 0.0, _angle);
                    }
                }
                if (!VMinInfinite())
                {
                    _builder.SetPCurve(_edges[Ebottom],
                                        _faces[Flateral],
                                        new Line2D(new Point2D(0, _vMin), new Direction2D(Direction2D.D.X)));
                    if (!HasSides || MeridianOnAxis(_vMin))
                    {
                        // closed edge set parameters
                        _builder.SetParameters(_edges[Ebottom], BottomEndVertex(), 0.0, _angle);
                    }
                }
            }
            if (HasSides)
            {
                _builder.SetPCurve(_edges[Estart],
                                    _faces[Flateral],
                                    new Line2D(new Point2D(0, -_meridianOffset), new Direction2D(Direction2D.D.Y)));

                _builder.SetPCurve(_edges[Eend],
                                    _faces[Flateral],
                                    new Line2D(new Point2D(_angle, -_meridianOffset), new Direction2D(Direction2D.D.Y)));
            }
            else
            {
                // closed edge
                _builder.SetPCurve(_edges[Estart],
                                    _faces[Flateral],
                                    new Line2D(new Point2D(_angle, -_meridianOffset), new Direction2D(Direction2D.D.Y)),
                                    new Line2D(new Point2D(0, -_meridianOffset), new Direction2D(Direction2D.D.Y)));
            }
            _builder.CompleteFace(_faces[Flateral]);
            _facesBuilt[Flateral] = true;
        }
        return _faces[Flateral];
    }

    /// <summary>The top planar face, oriented toward +Z (outside).</summary>
    public Face TopFace()
    {
        // do it if not done
        if (!_facesBuilt[Ftop])
        {
            Standard_DomainError_Raise_if(!HasTop(), "BRepPrim_OneAxis::TopFace:No top face");

            // make the empty face by translating the axes
            double z = MeridianValue(_vMax).Y;
            Vector3D v = new(_axes.Direction);
            v = v.Multiplied(z);
            _builder.MakeFace(ref _faces[Ftop], new Plane3D(new CoordinateSystem(_axes.Translated(v))));

            _builder.AddFaceWire(_faces[Ftop], TopWire());

            // put the parametric curves
            _builder.SetPCurve(_edges[Etop],
                                _faces[Ftop],
                                new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)),
                                              MeridianValue(_vMax).X));
            if (HasSides)
            {
                _builder.SetPCurve(_edges[Etopstart],
                                    _faces[Ftop],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)));
                _builder.SetPCurve(_edges[Etopend],
                                    _faces[Ftop],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Math.Cos(_angle), Math.Sin(_angle))));
            }
            _builder.CompleteFace(_faces[Ftop]);
            _facesBuilt[Ftop] = true;
        }
        return _faces[Ftop];
    }

    /// <summary>The bottom planar face, oriented toward -Z (outside).</summary>
    public Face BottomFace()
    {
        // do it if not done
        if (!_facesBuilt[Fbottom])
        {
            Standard_DomainError_Raise_if(!HasBottom(), "BRepPrim_OneAxis::BottomFace:No bottom face");

            // make the empty face by translating the axes
            double z = MeridianValue(_vMin).Y;
            Vector3D v = new(_axes.Direction);
            v = v.Multiplied(z);
            Frame3D axes = _axes.Translated(v);
            _builder.MakeFace(ref _faces[Fbottom], new Plane3D(new CoordinateSystem(axes)));
            _builder.ReverseFace(ref _faces[Fbottom]);
            _builder.AddFaceWire(_faces[Fbottom], BottomWire());

            // put the parametric curves
            _builder.SetPCurve(_edges[Ebottom],
                                _faces[Fbottom],
                                new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)),
                                              MeridianValue(_vMin).X));
            if (HasSides)
            {
                _builder.SetPCurve(_edges[Ebotstart],
                                    _faces[Fbottom],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)));
                _builder.SetPCurve(_edges[Ebotend],
                                    _faces[Fbottom],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Math.Cos(_angle), Math.Sin(_angle))));
            }
            _builder.CompleteFace(_faces[Fbottom]);
            _facesBuilt[Fbottom] = true;
        }
        return _faces[Fbottom];
    }

    /// <summary>The face starting the slice, oriented toward the exterior.</summary>
    public Face StartFace()
    {
        // do it if not done
        if (!_facesBuilt[Fstart])
        {
            Standard_DomainError_Raise_if(!HasSides, "BRepPrim_OneAxes::StartFace:No side faces");

            // build the empty face, perpendicular to myTool.Axes()
            Frame3D axes = new(_axes.Location, _axes.YDirection.Reversed(), _axes.XDirection);
            _builder.MakeFace(ref _faces[Fstart], new Plane3D(new CoordinateSystem(axes)));

            // When heights are inverted, the wire traversal direction changes due to the AxisEdge running in the
            // opposite direction. Reverse the face to compensate. For equal heights with open meridian (not closed),
            // AxisEdge is skipped. For closed meridian, areHeightsEqual() may be true but the wire structure doesn't
            // change, so the equal heights reversal doesn't apply.
            if (IsHeightInverted() || (AreHeightsEqual() && !MeridianClosed()))
            {
                _builder.ReverseFace(ref _faces[Fstart]);
            }

            if (VMaxInfinite() && VMinInfinite())
            {
                _builder.AddFaceWire(_faces[Fstart], AxisStartWire());
            }

            _builder.AddFaceWire(_faces[Fstart], StartWire());

            // parametric curves
            SetMeridianPCurve(_edges[Estart], _faces[Fstart]);
            // Skip AxisEdge pcurve when heights are equal since the edge is not in the wire
            if (_edgesBuilt[Eaxis] && !AreHeightsEqual())
            {
                _builder.SetPCurve(_edges[Eaxis],
                                    _faces[Fstart],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Direction2D.D.Y)));
            }

            // When heights are equal, the pcurves for TopStart and BotStart edges would overlap at Y = height. Use
            // small offsets to separate them.
            double pCurveOffset = AreHeightsEqual() ? global::Kerf.Engine.Foundation.Precision.Confusion() * THE_PCURVE_OFFSET_FACTOR : 0.0;
            if (_edgesBuilt[Etopstart])
            {
                _builder.SetPCurve(_edges[Etopstart],
                                    _faces[Fstart],
                                    new Line2D(new Point2D(0, MeridianValue(_vMax).Y + pCurveOffset), new Direction2D(Direction2D.D.X)));
            }
            if (_edgesBuilt[Ebotstart])
            {
                _builder.SetPCurve(_edges[Ebotstart],
                                    _faces[Fstart],
                                    new Line2D(new Point2D(0, MeridianValue(_vMin).Y - pCurveOffset), new Direction2D(Direction2D.D.X)));
            }
            _builder.CompleteFace(_faces[Fstart]);
            _facesBuilt[Fstart] = true;
        }
        return _faces[Fstart];
    }

    /// <summary>The face ending the slice, oriented toward the exterior.</summary>
    public Face EndFace()
    {
        // do it if not done
        if (!_facesBuilt[Fend])
        {
            Standard_DomainError_Raise_if(!HasSides, "BRepPrim_OneAxes::EndFace:No side faces");

            // build the empty face, perpendicular to myTool.Axes()
            Frame3D axes = new(_axes.Location, _axes.YDirection.Reversed(), _axes.XDirection);
            axes = axes.Rotated(_axes.Axis, _angle);
            _builder.MakeFace(ref _faces[Fend], new Plane3D(new CoordinateSystem(axes)));

            // When heights are inverted, the wire traversal direction changes due to the AxisEdge running in the
            // opposite direction. Don't reverse the face in this case. For equal heights with open meridian (not
            // closed), AxisEdge is skipped. For closed meridian, the equal heights reversal doesn't apply.
            if (!IsHeightInverted() && (!AreHeightsEqual() || MeridianClosed()))
            {
                _builder.ReverseFace(ref _faces[Fend]);
            }

            if (VMaxInfinite() && VMinInfinite())
            {
                _builder.AddFaceWire(_faces[Fend], AxisEndWire());
            }
            _builder.AddFaceWire(_faces[Fend], EndWire());

            // parametric curves
            SetMeridianPCurve(_edges[Eend], _faces[Fend]);
            // Skip AxisEdge pcurve when heights are equal since the edge is not in the wire
            if (_edgesBuilt[Eaxis] && !AreHeightsEqual())
            {
                _builder.SetPCurve(_edges[Eaxis],
                                    _faces[Fend],
                                    new Line2D(new Point2D(0, 0), new Direction2D(Direction2D.D.Y)));
            }

            // When heights are equal, the pcurves for TopEnd and BotEnd edges would overlap at Y = height. Use small
            // offsets to separate them.
            double pCurveOffset = AreHeightsEqual() ? global::Kerf.Engine.Foundation.Precision.Confusion() * THE_PCURVE_OFFSET_FACTOR : 0.0;
            if (_edgesBuilt[Etopend])
            {
                _builder.SetPCurve(_edges[Etopend],
                                    _faces[Fend],
                                    new Line2D(new Point2D(0, MeridianValue(_vMax).Y + pCurveOffset), new Direction2D(Direction2D.D.X)));
            }
            if (_edgesBuilt[Ebotend])
            {
                _builder.SetPCurve(_edges[Ebotend],
                                    _faces[Fend],
                                    new Line2D(new Point2D(0, MeridianValue(_vMin).Y - pCurveOffset), new Direction2D(Direction2D.D.X)));
            }
            _builder.CompleteFace(_faces[Fend]);
            _facesBuilt[Fend] = true;
        }
        return _faces[Fend];
    }

    /// <summary>The wire in the lateral face.</summary>
    public Wire LateralWire()
    {
        // do it if not done
        if (!_wiresBuilt[Wlateral])
        {
            _builder.MakeWire(ref _wires[Wlateral]);

            if (!VMaxInfinite())
            {
                _builder.AddWireEdge(_wires[Wlateral], TopEdge(), false);
            }
            _builder.AddWireEdge(_wires[Wlateral], EndEdge(), true);
            if (!VMinInfinite())
            {
                _builder.AddWireEdge(_wires[Wlateral], BottomEdge(), true);
            }
            _builder.AddWireEdge(_wires[Wlateral], StartEdge(), false);

            _builder.CompleteWire(_wires[Wlateral]);
            _wiresBuilt[Wlateral] = true;
        }
        return _wires[Wlateral];
    }

    /// <summary>The wire in the lateral face with the start edge.</summary>
    public Wire LateralStartWire()
    {
        // do it if not done
        if (!_wiresBuilt[Wlateralstart])
        {
            _builder.MakeWire(ref _wires[Wlateralstart]);

            _builder.AddWireEdge(_wires[Wlateralstart], StartEdge(), false);

            _builder.CompleteWire(_wires[Wlateralstart]);
            _wiresBuilt[Wlateralstart] = true;
        }
        return _wires[Wlateralstart];
    }

    /// <summary>The wire in the lateral face with the end edge.</summary>
    public Wire LateralEndWire()
    {
        // do it if not done
        if (!_wiresBuilt[Wlateralend])
        {
            _builder.MakeWire(ref _wires[Wlateralend]);

            _builder.AddWireEdge(_wires[Wlateralend], EndEdge(), true);

            _builder.CompleteWire(_wires[Wlateralend]);
            _wiresBuilt[Wlateralend] = true;
        }
        return _wires[Wlateralend];
    }

    /// <summary>The wire in the top face.</summary>
    public Wire TopWire()
    {
        // do it if not done
        if (!_wiresBuilt[Wtop])
        {
            Standard_DomainError_Raise_if(!HasTop(), "BRepPrim_OneAxis::TopWire: no top");

            _builder.MakeWire(ref _wires[Wtop]);

            _builder.AddWireEdge(_wires[Wtop], TopEdge(), true);
            if (HasSides)
            {
                _builder.AddWireEdge(_wires[Wtop], StartTopEdge(), true);
                _builder.AddWireEdge(_wires[Wtop], EndTopEdge(), false);
            }
            _builder.CompleteWire(_wires[Wtop]);
            _wiresBuilt[Wtop] = true;
        }
        return _wires[Wtop];
    }

    /// <summary>The wire in the bottom face.</summary>
    public Wire BottomWire()
    {
        // do it if not done
        if (!_wiresBuilt[Wbottom])
        {
            Standard_DomainError_Raise_if(!HasBottom(), "BRepPrim_OneAxis::BottomWire: no bottom");

            _builder.MakeWire(ref _wires[Wbottom]);

            _builder.AddWireEdge(_wires[Wbottom], BottomEdge(), false);
            if (HasSides)
            {
                _builder.AddWireEdge(_wires[Wbottom], EndBottomEdge(), true);
                _builder.AddWireEdge(_wires[Wbottom], StartBottomEdge(), false);
            }

            _builder.CompleteWire(_wires[Wbottom]);
            _wiresBuilt[Wbottom] = true;
        }
        return _wires[Wbottom];
    }

    /// <summary>The wire in the start face.</summary>
    public Wire StartWire()
    {
        // do it if not done
        if (!_wiresBuilt[Wstart])
        {
            Standard_DomainError_Raise_if(!HasSides, "BRepPrim_OneAxes::StartWire:no sides");

            _builder.MakeWire(ref _wires[Wstart]);

            // When heights are inverted, the AxisEdge direction is flipped: flip its reversed flag to keep the wire
            // connected. When heights are equal, skip the degenerate AxisEdge: the wire still closes because
            // AxisTopVertex and AxisBottomVertex are the same vertex.
            bool isInverted = IsHeightInverted();
            bool heightsEqual = AreHeightsEqual();

            if (HasBottom())
            {
                _builder.AddWireEdge(_wires[Wstart], StartBottomEdge(), true);
            }

            if (!MeridianClosed() && !heightsEqual)
            {
                if (!VMaxInfinite() || !VMinInfinite())
                {
                    _builder.AddWireEdge(_wires[Wstart], AxisEdge(), isInverted);
                }
            }

            if (HasTop())
            {
                _builder.AddWireEdge(_wires[Wstart], StartTopEdge(), false);
            }
            _builder.AddWireEdge(_wires[Wstart], StartEdge(), true);

            _builder.CompleteWire(_wires[Wstart]);
            _wiresBuilt[Wstart] = true;
        }
        return _wires[Wstart];
    }

    /// <summary>The wire in the start face with the AxisEdge.</summary>
    public Wire AxisStartWire()
    {
        // do it if not done
        if (!_wiresBuilt[Waxisstart])
        {
            Standard_DomainError_Raise_if(!HasSides, "BRepPrim_OneAxes::AxisStartWire:no sides");

            Standard_DomainError_Raise_if(!VMaxInfinite() || !VMinInfinite(),
                                          "BRepPrim_OneAxes::AxisStartWire:not infinite");

            Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxes::AxisStartWire:meridian closed");

            _builder.MakeWire(ref _wires[Waxisstart]);

            _builder.AddWireEdge(_wires[Waxisstart], AxisEdge(), false);

            _builder.CompleteWire(_wires[Waxisstart]);
            _wiresBuilt[Waxisstart] = true;
        }
        return _wires[Waxisstart];
    }

    /// <summary>The wire in the end face.</summary>
    public Wire EndWire()
    {
        // do it if not done
        if (!_wiresBuilt[Wend])
        {
            Standard_DomainError_Raise_if(!HasSides, "BRepPrim_OneAxes::EndWire:no sides");

            _builder.MakeWire(ref _wires[Wend]);

            // When heights are inverted, the AxisEdge direction is flipped: flip its reversed flag to keep the wire
            // connected. When heights are equal, skip the degenerate AxisEdge: the wire still closes because
            // AxisTopVertex and AxisBottomVertex are the same vertex.
            bool isInverted = IsHeightInverted();
            bool heightsEqual = AreHeightsEqual();

            if (HasTop())
            {
                _builder.AddWireEdge(_wires[Wend], EndTopEdge(), true);
            }
            if (!MeridianClosed() && !heightsEqual)
            {
                if (!VMaxInfinite() || !VMinInfinite())
                {
                    _builder.AddWireEdge(_wires[Wend], AxisEdge(), !isInverted);
                }
            }
            if (HasBottom())
            {
                _builder.AddWireEdge(_wires[Wend], EndBottomEdge(), false);
            }
            _builder.AddWireEdge(_wires[Wend], EndEdge(), false);

            _builder.CompleteWire(_wires[Wend]);
            _wiresBuilt[Wend] = true;
        }
        return _wires[Wend];
    }

    /// <summary>The wire in the end face with the AxisEdge.</summary>
    public Wire AxisEndWire()
    {
        // do it if not done
        if (!_wiresBuilt[Waxisend])
        {
            Standard_DomainError_Raise_if(!HasSides, "BRepPrim_OneAxes::AxisEndWire:no sides");

            Standard_DomainError_Raise_if(!VMaxInfinite() || !VMinInfinite(),
                                          "BRepPrim_OneAxes::AxisEndWire:not infinite");

            Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxes::AxisEndWire:meridian closed");

            _builder.MakeWire(ref _wires[Waxisend]);

            _builder.AddWireEdge(_wires[Waxisend], AxisEdge(), true);

            _builder.CompleteWire(_wires[Waxisend]);
            _wiresBuilt[Waxisend] = true;
        }
        return _wires[Waxisend];
    }

    /// <summary>The edge along the axis, oriented on +Z.</summary>
    public Edge AxisEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Eaxis])
        {
            Standard_DomainError_Raise_if(!HasSides, "BRepPrim_OneAxis::AxisEdge:no sides");
            Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxis::AxisEdge:closed");

            if (AreHeightsEqual())
            {
                // When heights are equal, the AxisEdge is degenerate (zero length): a degenerate edge at the shared
                // vertex.
                double height = MeridianValue(_vMax).Y;
                _builder.MakeDegeneratedEdge(ref _edges[Eaxis]);
                _builder.AddEdgeVertex(_edges[Eaxis], AxisTopVertex(), height, height);
            }
            else
            {
                // build the empty edge.
                _builder.MakeEdge(ref _edges[Eaxis], new Line3D(_axes.Axis));

                // When heights are inverted (VMax is geometrically below VMin), swap the first flags so the edge's
                // parameter range is increasing: the edge always runs from the geometrically lower vertex up.
                bool isInverted = IsHeightInverted();
                double yMax = MeridianValue(_vMax).Y;
                double yMin = MeridianValue(_vMin).Y;

                if (!VMaxInfinite())
                {
                    _builder.AddEdgeVertex(_edges[Eaxis], AxisTopVertex(), yMax, isInverted);
                }
                if (!VMinInfinite())
                {
                    _builder.AddEdgeVertex(_edges[Eaxis], AxisBottomVertex(), yMin, !isInverted);
                }
            }

            _builder.CompleteEdge(_edges[Eaxis]);
            _edgesBuilt[Eaxis] = true;
        }
        return _edges[Eaxis];
    }

    /// <summary>The edge at angle 0.</summary>
    public Edge StartEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Estart])
        {
            // is it shared with the EndEdge
            if (!HasSides && _edgesBuilt[Eend])
            {
                _edges[Estart] = _edges[Eend];
            }
            else
            {
                // build the empty Edge
                _edges[Estart] = MakeEmptyMeridianEdge(0.0);

                if (MeridianClosed())
                {
                    // Closed edge
                    _builder.AddEdgeVertex(_edges[Estart],
                                            TopStartVertex(),
                                            _vMin + _meridianOffset,
                                            _vMax + _meridianOffset);
                }
                else
                {
                    if (!VMaxInfinite())
                    {
                        _builder.AddEdgeVertex(_edges[Estart], TopStartVertex(), _vMax + _meridianOffset, false);
                    }
                    if (!VMinInfinite())
                    {
                        _builder.AddEdgeVertex(_edges[Estart], BottomStartVertex(), _vMin + _meridianOffset, true);
                    }
                }
            }

            _builder.CompleteEdge(_edges[Estart]);
            _edgesBuilt[Estart] = true;
        }
        return _edges[Estart];
    }

    /// <summary>The edge at angle Angle. Without sides, the start and end edges are the same edge.</summary>
    public Edge EndEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Eend])
        {
            // is it shared with the start edge
            if (!HasSides && _edgesBuilt[Estart])
            {
                _edges[Eend] = _edges[Estart];
            }
            else
            {
                // build the empty Edge
                _edges[Eend] = MakeEmptyMeridianEdge(_angle);

                if (MeridianClosed())
                {
                    // Closed edge
                    _builder.AddEdgeVertex(_edges[Eend],
                                            TopEndVertex(),
                                            _vMin + _meridianOffset,
                                            _vMax + _meridianOffset);
                }
                else
                {
                    if (!VMaxInfinite())
                    {
                        _builder.AddEdgeVertex(_edges[Eend], TopEndVertex(), _vMax + _meridianOffset, false);
                    }
                    if (!VMinInfinite())
                    {
                        _builder.AddEdgeVertex(_edges[Eend], BottomEndVertex(), _vMin + _meridianOffset, true);
                    }
                }
            }

            _builder.CompleteEdge(_edges[Eend]);
            _edgesBuilt[Eend] = true;
        }
        return _edges[Eend];
    }

    /// <summary>The linear edge between the start face and the top face.</summary>
    public Edge StartTopEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Etopstart])
        {
            Standard_DomainError_Raise_if(!HasTop() || !HasSides, "BRepPrim_OneAxis::StartTopEdge:no sides or no top");

            // build the empty Edge
            Vector3D v = new(_axes.Direction);
            v = v.Multiplied(MeridianValue(_vMax).Y);
            Point3D p = _axes.Location.Translated(v);
            _builder.MakeEdge(ref _edges[Etopstart], new Line3D(p, _axes.XDirection));

            _builder.AddEdgeVertex(_edges[Etopstart], AxisTopVertex(), 0.0, true);
            _builder.AddEdgeVertex(_edges[Etopstart], TopStartVertex(), MeridianValue(_vMax).X, false);

            _builder.CompleteEdge(_edges[Etopstart]);
            _edgesBuilt[Etopstart] = true;
        }
        return _edges[Etopstart];
    }

    /// <summary>The linear edge between the start face and the bottom face.</summary>
    public Edge StartBottomEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Ebotstart])
        {
            Standard_DomainError_Raise_if(!HasBottom() || !HasSides, "BRepPrim_OneAxis::StartBottomEdge:no sides or no top");

            // build the empty Edge
            Vector3D v = new(_axes.Direction);
            v = v.Multiplied(MeridianValue(_vMin).Y);
            Point3D p = _axes.Location.Translated(v);
            _builder.MakeEdge(ref _edges[Ebotstart], new Line3D(p, _axes.XDirection));

            _builder.AddEdgeVertex(_edges[Ebotstart], BottomStartVertex(), MeridianValue(_vMin).X, false);
            _builder.AddEdgeVertex(_edges[Ebotstart], AxisBottomVertex(), 0.0, true);

            _builder.CompleteEdge(_edges[Ebotstart]);
            _edgesBuilt[Ebotstart] = true;
        }
        return _edges[Ebotstart];
    }

    /// <summary>The linear edge between the end face and the top face.</summary>
    public Edge EndTopEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Etopend])
        {
            Standard_DomainError_Raise_if(!HasTop() || !HasSides, "BRepPrim_OneAxis::EndTopEdge:no sides or no top");

            // build the empty Edge
            Vector3D v = new(_axes.Direction);
            v = v.Multiplied(MeridianValue(_vMax).Y);
            Point3D p = _axes.Location.Translated(v);
            Line3D l = new(p, _axes.XDirection);
            l = l.Rotated(_axes.Axis, _angle);
            _builder.MakeEdge(ref _edges[Etopend], l);

            _builder.AddEdgeVertex(_edges[Etopend], AxisTopVertex(), 0.0, true);
            _builder.AddEdgeVertex(_edges[Etopend], TopEndVertex(), MeridianValue(_vMax).X, false);

            _builder.CompleteEdge(_edges[Etopend]);
            _edgesBuilt[Etopend] = true;
        }
        return _edges[Etopend];
    }

    /// <summary>The linear edge between the end face and the bottom face.</summary>
    public Edge EndBottomEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Ebotend])
        {
            Standard_DomainError_Raise_if(!HasBottom() || !HasSides, "BRepPrim_OneAxis::EndBottomEdge:no sides or no bottom");

            // build the empty Edge
            Vector3D v = new(_axes.Direction);
            v = v.Multiplied(MeridianValue(_vMin).Y);
            Point3D p = _axes.Location.Translated(v);
            Line3D l = new(p, _axes.XDirection);
            l = l.Rotated(_axes.Axis, _angle);
            _builder.MakeEdge(ref _edges[Ebotend], l);

            _builder.AddEdgeVertex(_edges[Ebotend], AxisBottomVertex(), 0.0, true);
            _builder.AddEdgeVertex(_edges[Ebotend], BottomEndVertex(), MeridianValue(_vMin).X, false);

            _builder.CompleteEdge(_edges[Ebotend]);
            _edgesBuilt[Ebotend] = true;
        }
        return _edges[Ebotend];
    }

    /// <summary>The edge at VMax. If the meridian is closed, the top and bottom edges are the same edge.</summary>
    public Edge TopEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Etop])
        {
            // Test if shared with bottom edge
            if (MeridianClosed() && _edgesBuilt[Ebottom])
            {
                _edges[Etop] = _edges[Ebottom];
            }
            else
            {
                // build the empty Edge
                if (!MeridianOnAxis(_vMax))
                {
                    Point2D mp = MeridianValue(_vMax);
                    Vector3D v = new(_axes.Direction);
                    v = v.Multiplied(mp.Y);
                    Point3D p = _axes.Location.Translated(v);
                    Circle3D c = new(new Frame3D(p, _axes.Direction, _axes.XDirection), mp.X);
                    _builder.MakeEdge(ref _edges[Etop], c);
                }
                else
                {
                    _builder.MakeDegeneratedEdge(ref _edges[Etop]);
                }

                if (!HasSides)
                {
                    // closed edge
                    _builder.AddEdgeVertex(_edges[Etop], TopEndVertex(), 0.0, _angle);
                }
                else
                {
                    _builder.AddEdgeVertex(_edges[Etop], TopEndVertex(), _angle, false);
                    _builder.AddEdgeVertex(_edges[Etop], TopStartVertex(), 0.0, true);
                }
            }

            _builder.CompleteEdge(_edges[Etop]);
            _edgesBuilt[Etop] = true;
        }
        return _edges[Etop];
    }

    /// <summary>The edge at VMin. If the meridian is closed, the top and bottom edges are the same edge.</summary>
    public Edge BottomEdge()
    {
        // do it if not done
        if (!_edgesBuilt[Ebottom])
        {
            // Test if shared with top edge
            if (MeridianClosed() && _edgesBuilt[Etop])
            {
                _edges[Ebottom] = _edges[Etop];
            }
            else
            {
                // build the empty Edge
                if (!MeridianOnAxis(_vMin))
                {
                    Point2D mp = MeridianValue(_vMin);
                    Vector3D v = new(_axes.Direction);
                    v = v.Multiplied(mp.Y);
                    Point3D p = _axes.Location.Translated(v);
                    Circle3D c = new(new Frame3D(p, _axes.Direction, _axes.XDirection), mp.X);
                    _builder.MakeEdge(ref _edges[Ebottom], c);
                }
                else
                {
                    _builder.MakeDegeneratedEdge(ref _edges[Ebottom]);
                }

                if (!HasSides)
                {
                    // closed edge
                    _builder.AddEdgeVertex(_edges[Ebottom], BottomEndVertex(), 0.0, _angle);
                }
                else
                {
                    _builder.AddEdgeVertex(_edges[Ebottom], BottomEndVertex(), _angle, false);
                    _builder.AddEdgeVertex(_edges[Ebottom], BottomStartVertex(), 0.0, true);
                }
            }

            _builder.CompleteEdge(_edges[Ebottom]);
            _edgesBuilt[Ebottom] = true;
        }
        return _edges[Ebottom];
    }

    /// <summary>The vertex at the top altitude on the axis.</summary>
    public Vertex AxisTopVertex()
    {
        // do it if not done
        if (!_verticesBuilt[Vaxistop])
        {
            // deduct from others
            if (MeridianOnAxis(_vMax) && _verticesBuilt[Vtopstart])
            {
                _vertices[Vaxistop] = _vertices[Vtopstart];
            }
            else if (MeridianOnAxis(_vMax) && _verticesBuilt[Vtopend])
            {
                _vertices[Vaxistop] = _vertices[Vtopend];
            }
            // Share with AxisBottomVertex if heights are equal (handles case when bottom is built first)
            else if (_verticesBuilt[Vaxisbot] && AreHeightsEqual())
            {
                _vertices[Vaxistop] = _vertices[Vaxisbot];
            }
            else
            {
                Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxis::AxisTopVertex");
                Standard_DomainError_Raise_if(VMaxInfinite(), "BRepPrim_OneAxis::AxisTopVertex");

                Vector3D v = new(_axes.Direction);
                v = v.Multiplied(MeridianValue(_vMax).Y);
                Point3D p = _axes.Location.Translated(v);
                _builder.MakeVertex(ref _vertices[Vaxistop], p);
            }

            _verticesBuilt[Vaxistop] = true;
        }
        return _vertices[Vaxistop];
    }

    /// <summary>The vertex at the bottom altitude on the axis.</summary>
    public Vertex AxisBottomVertex()
    {
        // do it if not done
        if (!_verticesBuilt[Vaxisbot])
        {
            // deduct from others
            if (MeridianOnAxis(_vMin) && _verticesBuilt[Vbotstart])
            {
                _vertices[Vaxisbot] = _vertices[Vbotstart];
            }
            else if (MeridianOnAxis(_vMin) && _verticesBuilt[Vbotend])
            {
                _vertices[Vaxisbot] = _vertices[Vbotend];
            }
            // Share with AxisTopVertex if heights are equal (within tolerance)
            else if (_verticesBuilt[Vaxistop] && AreHeightsEqual())
            {
                _vertices[Vaxisbot] = _vertices[Vaxistop];
            }
            else
            {
                Standard_DomainError_Raise_if(MeridianClosed(), "BRepPrim_OneAxis::AxisBottomVertex");
                Standard_DomainError_Raise_if(VMinInfinite(), "BRepPrim_OneAxis::AxisBottomVertex");

                Vector3D v = new(_axes.Direction);
                v = v.Multiplied(MeridianValue(_vMin).Y);
                Point3D p = _axes.Location.Translated(v);
                _builder.MakeVertex(ref _vertices[Vaxisbot], p);
            }

            _verticesBuilt[Vaxisbot] = true;
        }
        return _vertices[Vaxisbot];
    }

    /// <summary>The vertex (0, VMax).</summary>
    public Vertex TopStartVertex()
    {
        // do it if not done
        if (!_verticesBuilt[Vtopstart])
        {
            // deduct from others
            if (MeridianOnAxis(_vMax) && _verticesBuilt[Vaxistop])
            {
                _vertices[Vtopstart] = _vertices[Vaxistop];
            }
            else if ((MeridianOnAxis(_vMax) || !HasSides) && _verticesBuilt[Vtopend])
            {
                _vertices[Vtopstart] = _vertices[Vtopend];
            }
            else if (MeridianClosed() && _verticesBuilt[Vbotstart])
            {
                _vertices[Vtopstart] = _vertices[Vbotstart];
            }
            else if ((MeridianClosed() && !HasSides) && _verticesBuilt[Vbotend])
            {
                _vertices[Vtopstart] = _vertices[Vbotend];
            }
            else
            {
                Point2D mp = MeridianValue(_vMax);
                Vector3D v = new(_axes.Direction);
                v = v.Multiplied(mp.Y);
                Point3D p = _axes.Location.Translated(v);
                v = new Vector3D(_axes.XDirection);
                v = v.Multiplied(mp.X);
                p = p.Translated(v);
                _builder.MakeVertex(ref _vertices[Vtopstart], p);
            }

            _verticesBuilt[Vtopstart] = true;
        }
        return _vertices[Vtopstart];
    }

    /// <summary>The vertex (Angle, VMax).</summary>
    public Vertex TopEndVertex()
    {
        // do it if not done
        if (!_verticesBuilt[Vtopend])
        {
            // deduct from others
            if (MeridianOnAxis(_vMax) && _verticesBuilt[Vaxistop])
            {
                _vertices[Vtopend] = _vertices[Vaxistop];
            }
            else if ((MeridianOnAxis(_vMax) || !HasSides) && _verticesBuilt[Vtopstart])
            {
                _vertices[Vtopend] = _vertices[Vtopstart];
            }
            else if (MeridianClosed() && _verticesBuilt[Vbotend])
            {
                _vertices[Vtopend] = _vertices[Vbotend];
            }
            else if ((MeridianClosed() && !HasSides) && _verticesBuilt[Vbotstart])
            {
                _vertices[Vtopend] = _vertices[Vbotstart];
            }
            else
            {
                Point2D mp = MeridianValue(_vMax);
                Vector3D v = new(_axes.Direction);
                v = v.Multiplied(mp.Y);
                Point3D p = _axes.Location.Translated(v);
                v = new Vector3D(_axes.XDirection);
                v = v.Multiplied(mp.X);
                p = p.Translated(v);
                p = p.Rotated(_axes.Axis, _angle);
                _builder.MakeVertex(ref _vertices[Vtopend], p);
            }

            _verticesBuilt[Vtopend] = true;
        }
        return _vertices[Vtopend];
    }

    /// <summary>The vertex (0, VMin).</summary>
    public Vertex BottomStartVertex()
    {
        // do it if not done
        if (!_verticesBuilt[Vbotstart])
        {
            // deduct from others
            if (MeridianOnAxis(_vMin) && _verticesBuilt[Vaxisbot])
            {
                _vertices[Vbotstart] = _vertices[Vaxisbot];
            }
            else if ((MeridianOnAxis(_vMin) || !HasSides) && _verticesBuilt[Vbotend])
            {
                _vertices[Vbotstart] = _vertices[Vbotend];
            }
            else if (MeridianClosed() && _verticesBuilt[Vtopstart])
            {
                _vertices[Vbotstart] = _vertices[Vtopstart];
            }
            else if ((MeridianClosed() && !HasSides) && _verticesBuilt[Vtopend])
            {
                _vertices[Vbotstart] = _vertices[Vtopend];
            }
            else
            {
                Point2D mp = MeridianValue(_vMin);
                Vector3D v = new(_axes.Direction);
                v = v.Multiplied(mp.Y);
                Point3D p = _axes.Location.Translated(v);
                v = new Vector3D(_axes.XDirection);
                v = v.Multiplied(mp.X);
                p = p.Translated(v);
                _builder.MakeVertex(ref _vertices[Vbotstart], p);
            }

            _verticesBuilt[Vbotstart] = true;
        }
        return _vertices[Vbotstart];
    }

    /// <summary>The vertex (Angle, VMin).</summary>
    public Vertex BottomEndVertex()
    {
        // do it if not done
        if (!_verticesBuilt[Vbotend])
        {
            // deduct from others
            if (MeridianOnAxis(_vMin) && _verticesBuilt[Vaxisbot])
            {
                _vertices[Vbotend] = _vertices[Vaxisbot];
            }
            else if ((MeridianOnAxis(_vMin) || !HasSides) && _verticesBuilt[Vbotstart])
            {
                _vertices[Vbotend] = _vertices[Vbotstart];
            }
            else if (MeridianClosed() && _verticesBuilt[Vtopend])
            {
                _vertices[Vbotend] = _vertices[Vtopend];
            }
            else if (MeridianClosed() && !HasSides && _verticesBuilt[Vtopstart])
            {
                _vertices[Vbotend] = _vertices[Vtopstart];
            }
            else
            {
                Point2D mp = MeridianValue(_vMin);
                Vector3D v = new(_axes.Direction);
                v = v.Multiplied(mp.Y);
                Point3D p = _axes.Location.Translated(v);
                v = new Vector3D(_axes.XDirection);
                v = v.Multiplied(mp.X);
                p = p.Translated(v);
                p = p.Rotated(_axes.Axis, _angle);
                _builder.MakeVertex(ref _vertices[Vbotend], p);
            }

            _verticesBuilt[Vbotend] = true;
        }
        return _vertices[Vbotend];
    }
}
