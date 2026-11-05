// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Api.FaceError;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Offsets.Api;

/// <summary>
/// Offsets the wires of a planar face, or planar wires, in their plane (the 2D wire offset): one
/// <see cref="OffsetWire"/> per domain the closed wires bound.
/// </summary>
/// <remarks>
/// <para>A positive offset goes outside the closed wires, a negative one inside. The join type is GeomAbs_Arc (arcs
/// of circles around salient vertices) or GeomAbs_Intersection (the parallels prolonged to their intersection). With
/// IsOpenResult an open wire gives an open parallel on each side instead of a closed one around it.</para>
/// <para>A maker (decisions 414 and 454): <see cref="MakeShape.Shape"/> and
/// <see cref="MakeShape.Generated"/> hand out frozen shapes; <see cref="ConvertFace"/> returns a frozen
/// face. The algorithm never writes into the caller's face or wires.</para>
/// </remarks>
internal class MakeWireOffset : MakeShape
{
    private bool _isInitialized;
    private bool _lastIsLeft;
    private JoinKind _join;
    private bool _isOpenResult;
    private bool _isToApprox;
    private Face _face = new();
    private readonly NCollection_List<Shape> _wires = new();
    private readonly NCollection_List<OffsetWire> _left = new();
    private readonly NCollection_List<OffsetWire> _right = new();

    private static bool NeedsConvertion(in Wire wire)
    {
        Iterator iter = new(wire);
        for (; iter.More(); iter.Next())
        {
            Edge edge = ShapeCasts.Edge(iter.Value());
            EdgeCurveAdaptor bAcurve = new(edge);
            CurveKind type = bAcurve.GetType();
            if (type != Line && type != Circle)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Converts each wire of the face into a contour of arcs of circles and segments only (with new 3D curves); wires
    /// already made of lines and circles are kept. The result is frozen.
    /// </summary>
    public static Face ConvertFace(in Face face, double angleTolerance)
    {
        Face newFace = InternalConvertFace(face, angleTolerance);
        newFace.TShape()?.FreezeGraph();
        return newFace;
    }

    /// <summary><see cref="ConvertFace"/> without freezing, for <see cref="Perform"/>.</summary>
    internal static Face InternalConvertFace(in Face face, double angleTolerance)
    {
        Orientation or = face.Orientation;
        Face aFace = face;
        aFace.Orient(Forward);
        Face newFace = ShapeCasts.Face(aFace.EmptyCopied());
        BrepBuilder bb = new();
        Iterator iter = new(aFace);
        for (; iter.More(); iter.Next())
        {
            Wire wire = ShapeCasts.Wire(iter.Value());
            if (NeedsConvertion(wire))
            {
                Orientation orOfWire = wire.Orientation;
                wire.Orient(Forward);
                // Kerf: the converted wire is new; BuildCurves3d writes into it (decision 915)
                wire = global::Kerf.Engine.Booleans.Algo.BRepAlgo.InternalConvertWire(wire, angleTolerance, aFace);
                BrepLibrary.BuildCurves3d(wire);
                wire.Orient(orOfWire);
            }
            bb.Add(newFace, wire);
        }
        newFace.Orient(or);
        return newFace;
    }

    /// <summary>An empty offset.</summary>
    public MakeWireOffset()
    {
        _isInitialized = false;
        _join = Arc;
        _isOpenResult = false;
        _isToApprox = false;
    }

    /// <summary>The offset of the wires of the planar face <paramref name="spine"/>.</summary>
    public MakeWireOffset(in Face spine, JoinKind Join = Arc, bool isOpenResult = false)
    {
        Init(spine, Join, isOpenResult);
    }

    /// <summary>
    /// Initializes the algorithm to construct parallels to the wires of <paramref name="spine"/>. Join defines the
    /// parallels around the salient vertices: arcs of circles (GeomAbs_Arc) or the adjacent parallels prolonged to
    /// their intersection (GeomAbs_Intersection).
    /// </summary>
    public void Init(in Face spine, JoinKind Join = Arc, bool isOpenResult = false)
    {
        _face = spine;
        _isInitialized = true;
        _join = Join;
        _isOpenResult = isOpenResult;
        _isToApprox = false;
        Explorer exp = new();
        for (exp.Init(_face, ShapeType.Wire); exp.More(); exp.Next())
        {
            _wires.Append(exp.Current());
        }
    }

    /// <summary>The offset of the planar wire <paramref name="spine"/>.</summary>
    public MakeWireOffset(in Wire spine, JoinKind Join = Arc, bool isOpenResult = false)
    {
        _wires.Append(spine);
        _isInitialized = true;
        _join = Join;
        _isOpenResult = isOpenResult;
        _isToApprox = false;
    }

    /// <summary>Sets the join type and the open-result flag.</summary>
    public void Init(JoinKind Join = Arc, bool isOpenResult = false)
    {
        _join = Join;
        _isOpenResult = isOpenResult;
    }

    /// <summary>
    /// Sets the approximation flag: the input contours are first converted into contours of 2D arcs of circles and
    /// segments.
    /// </summary>
    public void SetApprox(bool toApprox)
    {
        _isToApprox = toApprox;
    }

    /// <summary>Adds a wire to offset.</summary>
    public void AddWire(in Wire spine)
    {
        _isInitialized = true;
        _wires.Append(spine);
    }

    private static void BuildDomains(ref Face myFace,
                                     NCollection_List<Shape> workWires,
                                     NCollection_List<OffsetWire> myAlgos,
                                     JoinKind myJoin,
                                     bool myIsOpenResult,
                                     bool isPositive,
                                     ref bool isWasReversed)
    {
        FaceRestrictor fr = new();
        Vertex vf, vl;
        NCollection_List<Shape> low = new();
        BrepBuilder b = new();

        if (myFace.IsNull)
        {
            myFace = new MakeFace(ShapeCasts.Wire(workWires.First()), true).Face();
            if (myFace.IsNull)
            {
                throw new NotDoneException("BRepOffsetAPI_MakeOffset : the wire is not planar");
            }
        }
        //  Modified by Sergey KHROMOV - Thu Apr 26 16:04:43 2001 Begin
        Explorer anExp = new(myFace, ShapeType.Wire);
        Shape wire1 = workWires.First();
        Shape wire2;
        if (anExp.More())
        {
            wire2 = anExp.Current();
            if ((wire1.Orientation == wire2.Orientation && isPositive)
                || (wire1.Orientation == global::Kerf.Engine.Topology.TopAbs.Complement(wire2.Orientation) && !isPositive))
            {
                NCollection_List<Shape> lWires = new();
                NCollection_List<Shape>.Iterator itl1 = new();
                for (itl1.Initialize(workWires); itl1.More(); itl1.Next())
                {
                    Shape w = itl1.Value();
                    lWires.Append(w.Reversed());
                }
                isWasReversed = true;
                // the caller's myWires, reversed (OCCT assigns the list through the reference)
                workWires.Assign(lWires);
            }
        }
        //  Modified by Sergey KHROMOV - Thu Apr 26 16:04:44 2001 End

        fr.Init(myFace, true);
        //====================================================
        // Construction of faces limited by closed wires.
        //====================================================
        NCollection_List<Shape>.Iterator itl = new(workWires);
        for (; itl.More(); itl.Next())
        {
            Wire w = ShapeCasts.Wire(itl.Value());
            if (w.ClosedFlag)
            {
                fr.Add(w);
                continue;
            }
            Exploration.Vertices(w, out vf, out vl);
            if (vf.IsSame(vl))
            {
                fr.Add(w);
            }
            else
            {
                low.Append(w);
            }
        }
        // Kerf: the restrictor copies frozen wires at the entry and gives the caller's back; its faces stay
        // unfrozen, as the open wires are added to them (decision 914)
        fr.Perform();
        if (!fr.IsDone)
        {
            throw new NotDoneException("BRepOffsetAPI_MakeOffset : Build Domains");
        }
        NCollection_List<Shape> faces = new();
        for (; fr.More(); fr.Next())
        {
            faces.Append(fr.InternalCurrent());
        }

        //===========================================
        // No closed wire => only one domain
        //===========================================
        if (faces.IsEmpty())
        {
            Shape localShape = myFace.EmptyCopied();
            Face f = ShapeCasts.Face(localShape);
            NCollection_List<Shape>.Iterator itW = new(low);
            for (; itW.More(); itW.Next())
            {
                b.Add(f, itW.Value());
            }
            OffsetWire algo = new(f, myJoin, myIsOpenResult);
            myAlgos.Append(algo);
            return;
        }

        //====================================================
        // Classification of open wires.
        //====================================================
        NCollection_List<Shape>.Iterator itF = new();
        for (itF.Initialize(faces); itF.More(); itF.Next())
        {
            Face f = ShapeCasts.Face(itF.Value());
            FaceSurfaceAdaptor s = new(f, false);
            double tol = Tool.Tolerance(f);

            FClass2d cl = new(f, global::Kerf.Engine.Foundation.Precision.Confusion());

            NCollection_List<Shape>.Iterator itW = new(low);
            while (itW.More())
            {
                Wire w = ShapeCasts.Wire(itW.Value());
                //=======================================================
                // Choice of a point on the wire. + projection on the face.
                //=======================================================
                Explorer exp = new(w, ShapeType.Vertex);
                Vertex v = ShapeCasts.Vertex(exp.Current());
                Point2D pv = new();
                Point3D p3d = Tool.Pnt(v);
                ExtPS extPS = new(p3d, s, tol, tol, ExtFlag_MIN);
                double dist2Min = global::Kerf.Engine.Foundation.Precision.Infinite();
                // OCCT declares Found as a double
                bool found = false;
                for (int ie = 1; ie <= extPS.ExtCount; ie++)
                {
                    double x, y;
                    if (extPS.SquareDistance(ie) < dist2Min)
                    {
                        dist2Min = extPS.SquareDistance(ie);
                        found = true;
                        extPS.Point(ie).Parameter(out x, out y);
                        pv = pv.WithCoord(x, y);
                    }
                }
                if (found && (cl.Perform(pv) == In))
                {
                    // The face that contains a wire is found and it is removed from the list
                    b.Add(f, w);
                    low.Remove(itW);
                }
                else
                {
                    itW.Next();
                }
            }
        }

        //========================================
        // Creation of algorithms on each domain.
        //========================================
        for (itF.Initialize(faces); itF.More(); itF.Next())
        {
            OffsetWire algo = new(ShapeCasts.Face(itF.Value()), myJoin, myIsOpenResult);
            myAlgos.Append(algo);
        }
    }

    /// <summary>
    /// Computes a parallel to the spine at the distance <paramref name="offset"/> (outside the closed wires if
    /// positive), at the altitude <paramref name="alt"/> along the normal of the spine's plane. Failures leave the
    /// maker not done.
    /// </summary>
    /// <exception cref="NotDoneException">The maker has neither a face nor a wire.</exception>
    public void Perform(double offset, double alt = 0.0)
    {
        StdFail_NotDone_Raise_if(!_isInitialized, "BRepOffsetAPI_MakeOffset : Perform without Init");

        try
        {
            if (_isToApprox)
            {
                double tol = 0.1;
                if (_face.IsNull)
                {
                    Face face = new();
                    bool onlyPlane = true;
                    NCollection_List<Shape>.Iterator itl = new(_wires);
                    for (; itl.More(); itl.Next())
                    {
                        MakeFace faceMaker = new(ShapeCasts.Wire(itl.Value()), onlyPlane);
                        if (faceMaker.Error() == FaceError.FaceDone)
                        {
                            face = faceMaker.Face();
                            break;
                        }
                    }
                    for (itl.Initialize(_wires); itl.More(); itl.Next())
                    {
                        Wire wire = ShapeCasts.Wire(itl.Value());
                        if (NeedsConvertion(wire))
                        {
                            Wire newWire = global::Kerf.Engine.Booleans.Algo.BRepAlgo.InternalConvertWire(wire, tol, face);
                            BrepLibrary.BuildCurves3d(newWire);
                            newWire.Orient(wire.Orientation);
                            itl.ChangeValue() = newWire;
                        }
                    }
                }
                else
                {
                    _face = InternalConvertFace(_face, tol);
                    BrepLibrary.BuildCurves3d(_face);
                    _wires.Clear();
                    Iterator iter = new(_face);
                    for (; iter.More(); iter.Next())
                    {
                        _wires.Append(iter.Value());
                    }
                }
            }

            int i = 1;
            NCollection_List<OffsetWire>.Iterator itOW = new();
            Compound res = new();
            BrepBuilder b = new();
            b.MakeCompound(ref res);
            _lastIsLeft = (offset <= 0);

            bool isWasReversed = false;
            if (offset <= 0.0)
            {
                if (_left.IsEmpty())
                {
                    //  Modified by Sergey KHROMOV - Fri Apr 27 14:35:26 2001 Begin
                    BuildDomains(ref _face, _wires, _left, _join, _isOpenResult, false, ref isWasReversed);
                    //  Modified by Sergey KHROMOV - Fri Apr 27 14:35:26 2001 End
                }

                for (itOW.Initialize(_left); itOW.More(); itOW.Next())
                {
                    OffsetWire algo = itOW.Value();
                    algo.Perform(Math.Abs(offset), alt);
                    if (algo.IsDone && !algo.InternalShape().IsNull)
                    {
                        b.Add(res, isWasReversed ? algo.InternalShape().Reversed() : algo.InternalShape());
                        if (i == 1)
                        {
                            _shape = isWasReversed ? algo.InternalShape().Reversed() : algo.InternalShape();
                        }

                        i++;
                    }
                }
            }
            else
            {
                if (_right.IsEmpty())
                {
                    //  Modified by Sergey KHROMOV - Fri Apr 27 14:35:28 2001 Begin
                    BuildDomains(ref _face, _wires, _right, _join, _isOpenResult, true, ref isWasReversed);
                    //  Modified by Sergey KHROMOV - Fri Apr 27 14:35:35 2001 End
                }

                for (itOW.Initialize(_right); itOW.More(); itOW.Next())
                {
                    OffsetWire algo = itOW.Value();
                    algo.Perform(offset, alt);

                    if (algo.IsDone && !algo.InternalShape().IsNull)
                    {
                        b.Add(res, isWasReversed ? algo.InternalShape().Reversed() : algo.InternalShape());
                        if (i == 1)
                        {
                            _shape = isWasReversed ? algo.InternalShape().Reversed() : algo.InternalShape();
                        }

                        i++;
                    }
                }
            }

            if (i > 2)
            {
                _shape = res;
            }

            if (_shape.IsNull)
            {
                NotDone();
            }
            else
            {
                Done();
            }
        }
        catch (Exception exception) when ((Refusal.IsFailure(exception) || exception is NullReferenceException)
                                            && exception is not ImmutableObjectException)
        {
            // OCCT catches Standard_Failure; with the signals Draw and build123d.NET set, an access violation (the
            // medial axis's null bisector, occt-issues.md) arrives as one: the port's NullReferenceException
            // (decision 917)
            NotDone();
            _shape.Nullify();
        }
    }

    /// <summary>Does nothing: the shape is built by <see cref="Perform"/>.</summary>
    public override void Build(ProgressRange range = default)
    {
        Done();
    }

    private protected override void BeforePublishingLists()
    {
        _shape.TShape()?.FreezeGraph();
    }

    /// <summary>The shapes of the last parallel generated by the edge or vertex <paramref name="s"/> of a spine.</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        NCollection_List<OffsetWire>.Iterator itOW = new();
        NCollection_List<OffsetWire> algos;
        algos = _left;
        if (!_lastIsLeft)
        {
            algos = _right;
        }
        for (itOW.Initialize(algos); itOW.More(); itOW.Next())
        {
            OffsetWire ow = itOW.Value();
            NCollection_List<Shape> l;
            l = new NCollection_List<Shape>(ow.InternalGeneratedShapes(s.Oriented(Forward)));
            _generated.Append(l);
            l = new NCollection_List<Shape>(ow.InternalGeneratedShapes(s.Oriented(Reversed)));
            _generated.Append(l);
        }
        return _generated;
    }
}
