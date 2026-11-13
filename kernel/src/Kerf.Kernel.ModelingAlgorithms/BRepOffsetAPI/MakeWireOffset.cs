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
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Topology.Orientation;
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

        RefusedBecause = null;
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

            // Kerf: each loop of a closed parallel lies on the side of the spine it was asked for, or the offset is no
            // success (decisions 1742, 35202)
            if (!_shape.IsNull && !_isOpenResult && WrongSide(_wires, _face, _shape, offset) is { } wrong)
            {
                _shape.Nullify();
                RefusedBecause = (OffsetWire.WrongSideCode, $"The offset in the plane is on the wrong side of the wire, at {OffsetWire.Where(wrong)}.");
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
            // Kerf: why a parallel was refused, if one was (decision 2422)
            foreach (OffsetWire algo in offset <= 0.0 ? _left : _right)
            {
                if (algo.RefusedBecause is { } why)
                {
                    RefusedBecause = why;
                    break;
                }
            }
            // OCCT catches Standard_Failure; with the signals Draw and Kerf.Modeling set, an access violation (the
            // medial axis's null bisector, occt-issues.md) arrives as one: the port's NullReferenceException
            // (decision 917)
            NotDone();
            _shape.Nullify();
        }
    }

    /// <summary>
    /// Kerf: why the last <see cref="Perform"/> refused a parallel it made, not at its distance from the spine or
    /// crossing itself, as a diagnostic's code and message; null if it didn't (decisions 2279, 2422).
    /// </summary>
    internal (string Code, string Message)? RefusedBecause { get; private set; }

    /// <summary>
    /// Kerf (decision 35202): a point of the first loop of the parallel that lies on the wrong side of the spine, or null
    /// if every loop is on the side asked for: inside the region the closed wires bound for an offset inwards, outside it
    /// for one outwards, as MakeOffset's sign has it where no wire lies inside another. A loop is at the offset from the
    /// spine everywhere (decisions 2279, 2281) and doesn't cross the spine, so the middle of its first edge decides its
    /// side, and the spine's edges as polylines within a quarter of the offset of their curves can't misplace a point that
    /// far from them. An even-odd count of the polylines' crossings by each of two rays from the point, in the spine's
    /// plane, a loop wrong only if both say so: independent of the medial axis, of the face made on the spine and of the
    /// wires' orientations. Not judged where a wire of the spine lies inside another (a hole, whose parallel outwards is
    /// its own outside, into the face), where a wire is open (a vertex not met by an even number of its edges' ends), nor
    /// below <c>Precision::Confusion</c>.
    /// </summary>
    internal static Point3D? WrongSide(NCollection_List<Shape> spineWires, in Face face, in Shape parallel, double offset)
    {
        double deflection = 0.25 * Math.Abs(offset);
        if (face.IsNull || Tool.Surface(face) is not PlaneSurface plane
            || !(deflection > global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            return null;
        }
        Plane3D pln = plane.Plane;
        List<(double U, double V)[]> polylines = [];
        List<List<(double U, double V)[]>> byWire = [];
        foreach (Shape spineWire in spineWires)
        {
            List<(double U, double V)[]> ofWire = [];
            byWire.Add(ofWire);
            Dictionary<Shape, int> ends = new(ShapeHasher.Instance);
            for (Iterator it = new(spineWire); it.More(); it.Next())
            {
                Edge edge = ShapeCasts.Edge(it.Value());
                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                foreach (Vertex v in new[] { v1, v2 })
                {
                    if (v.IsNull)
                    {
                        return null;
                    }
                    ends[v.Oriented(Forward)] = ends.GetValueOrDefault(v.Oriented(Forward)) + 1;
                }
                if (!Tool.IsGeometric(edge))
                {
                    continue;
                }
                TangentialDeflection points = new(new EdgeCurveAdaptor(edge), 0.5, deflection);
                var polyline = new (double U, double V)[points.PointCount];
                for (int k = 1; k <= points.PointCount; k++)
                {
                    ElSLib.Parameters(pln, points.Value(k), out double u, out double v);
                    polyline[k - 1] = (u, v);
                }
                polylines.Add(polyline);
                ofWire.Add(polyline);
            }
            if (ends.Count == 0 || ends.Values.Any(n => n % 2 != 0))
            {
                return null;
            }
        }
        // A wire inside another (a hole) is offset to its own outside when outwards, into the face, so the region of the
        // wires is no measure there: such spines aren't judged
        for (int i = 0; i < byWire.Count; i++)
        {
            for (int j = 0; j < byWire.Count; j++)
            {
                if (i != j && byWire[i].Count > 0 && byWire[i][0].Length > 0
                    && IsInside(byWire[j], byWire[i][0][0].U, byWire[i][0][0].V, 0.3711))
                {
                    return null;
                }
            }
        }
        for (Explorer exp = new(parallel, ShapeType.Wire); exp.More(); exp.Next())
        {
            Explorer edges = new(exp.Current(), ShapeType.Edge);
            for (; edges.More() && !Tool.IsGeometric(ShapeCasts.Edge(edges.Current())); edges.Next())
            {
            }
            if (!edges.More())
            {
                continue;
            }
            Edge edge = ShapeCasts.Edge(edges.Current());
            Tool.Range(edge, out double first, out double last);
            Point3D middle = new EdgeCurveAdaptor(edge).Value(0.5 * (first + last));
            ElSLib.Parameters(pln, middle, out double x, out double y);
            // Two rays, in directions no wire of a drawing is likely to run along, so that one through a vertex, where
            // the polylines of two edges may miss each other by a rounding, doesn't misplace the point: wrong only if
            // both say so
            if (IsInside(polylines, x, y, 0.3711) == (offset <= 0.0) || IsInside(polylines, x, y, 1.9137) == (offset <= 0.0))
            {
                continue;
            }
            return middle;
        }
        return null;
    }

    // Kerf: whether the point is inside the polylines, by the even-odd count of their crossings by a ray from it at the
    // angle given
    private static bool IsInside(List<(double U, double V)[]> polylines, double x, double y, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        bool inside = false;
        foreach ((double U, double V)[] polyline in polylines)
        {
            for (int k = 0; k + 1 < polyline.Length; k++)
            {
                // In a frame whose first axis is the ray
                double au = (polyline[k].U - x) * c + (polyline[k].V - y) * s, av = (polyline[k].V - y) * c - (polyline[k].U - x) * s;
                double bu = (polyline[k + 1].U - x) * c + (polyline[k + 1].V - y) * s, bv = (polyline[k + 1].V - y) * c - (polyline[k + 1].U - x) * s;
                if ((av > 0.0) != (bv > 0.0) && au - av * (bu - au) / (bv - av) > 0.0)
                {
                    inside = !inside;
                }
            }
        }
        return inside;
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
