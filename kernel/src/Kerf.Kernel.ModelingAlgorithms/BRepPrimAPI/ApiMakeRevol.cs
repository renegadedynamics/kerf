// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevol.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Sweeping;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds the revolved topology of a shape around an axis, by an angle or a whole turn: vertex to edge, edge to face,
/// wire to shell, face to solid, shell to compsolid. A degenerated edge shared by several faces is copied for each.
/// </summary>
/// <remarks>
/// <para>Without copy, OCCT shares the generating shape's sub-shapes in the result and writes into them. Kerf sweeps
/// the caller's shape itself in a working overlay (<see cref="MakeSweep"/>): the result holds its own sub-shapes
/// wherever the sweep left them unchanged, and the history answers for the caller's sub-shapes directly.</para>
/// <para>If an edge of the shape crosses the axis, the result is null (<see cref="MakeShape.Shape"/>
/// returns a null shape, as OCCT does).</para>
/// </remarks>
public class ApiMakeRevol : MakeSweep
{
    private readonly Revol _revol;
    private readonly NCollection_List<Shape> _degenerated = new();
    private History? _hist;
    private bool _isBuild;

    // perform checks on the argument
    private static Shape CheckCore(in Shape s)
    {
        BrepLibrary.BuildCurves3d(s);
        return s;
    }

    /// <summary>
    /// The revolution of <paramref name="s"/> around <paramref name="a"/> by the angle <paramref name="d"/> (radians,
    /// reduced to [0, 2π]). With <paramref name="copy"/>, S's sub-shapes are copied.
    /// </summary>
    public ApiMakeRevol(in Shape s, in Axis3D a, double d, bool copy = false)
    {
        using WorkingOverlay? overlay = BeginGenerating(s);
        Shape aS = s;
        _revol = new Revol(CheckCore(aS), a, d, copy);
        _isBuild = false;
        if (!CheckValidity(CheckCore(aS), a))
        {
            _shape.Nullify();
            _isBuild = true;
        }
        else
        {
            Build();
        }
    }

    /// <summary>The whole revolution of <paramref name="s"/> around <paramref name="a"/>.</summary>
    public ApiMakeRevol(in Shape s, in Axis3D a, bool copy = false)
    {
        using WorkingOverlay? overlay = BeginGenerating(s);
        Shape aS = s;
        _revol = new Revol(CheckCore(aS), a, 2.0 * Math.PI, copy);
        _isBuild = false;
        if (!CheckValidity(CheckCore(aS), a))
        {
            _shape.Nullify();
            _isBuild = true;
        }
        else
        {
            Build();
        }
    }

    /// <summary>The internal sweeping algorithm (keyed by the generating shape's own sub-shapes).</summary>
    internal Revol Revol() => _revol;

    /// <summary>Builds the resulting shape (redefined from MakeShape).</summary>
    public override void Build(ProgressRange range = default)
    {
        if (_isBuild)
        {
            return;
        }
        _shape = _revol.Shape();
        BrepLibrary.UpdateInnerTolerances(_shape);

        Done();
        _isBuild = true;

        _hist = null;
        _degenerated.Clear();
        var degE = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var bb = new BrepBuilder();

        var exp = new Explorer(_shape, ShapeType.Edge);
        // Problem is that some degenerated edges can be shared by different faces.
        // It is not valid for correct shape.
        // To solve problem it is possible to copy shared degenerated edge for each face, which has it,
        // and replace shared edge by its copy
        for (; exp.More(); exp.Next())
        {
            Shape edge = exp.Current();
            var tEdge = (BrepTEdge)edge.TShape()!;
            if (tEdge.Degenerated())
            {
                if (degE.TryGetValue(edge, out NCollection_List<Shape>? l))
                {
                    // Make the copy if degenerated edge occurs more then once
                    Shape copyE = edge.EmptyCopied();
                    copyE.Orient(Forward);
                    var vIter = new Iterator(edge.Oriented(Forward), false);
                    for (; vIter.More(); vIter.Next())
                    {
                        bb.Add(copyE, vIter.Value());
                    }
                    copyE.Orient(edge.Orientation);
                    l.Append(copyE);
                    _degenerated.Append(copyE);
                }
                else
                {
                    l = degE.SetValueRef(edge, new NCollection_List<Shape>());
                    l.Append(edge);
                    _degenerated.Append(edge);
                }
            }
        }
        if (!_degenerated.IsEmpty())
        {
            var subs = new ReShape();
            var degF = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            bool isReplaced = false;
            exp.Init(_shape, ShapeType.Face);
            // Replace degenerated edge by its copies for different faces
            // First, for each face list of d.e. is created
            for (; exp.More(); exp.Next())
            {
                Shape f = exp.Current();
                var expE = new Explorer(f, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (Tool.Degenerated(ShapeCasts.Edge(e)))
                    {
                        if (!degF.TryGetValue(f, out NCollection_List<Shape>? l))
                        {
                            l = degF.SetValueRef(f, new NCollection_List<Shape>());
                        }
                        l.Append(e);
                    }
                }
            }
            //
            // Second, replace edges by copies using ReShape
            var subsF = new ReShape();
            // OCCT iterates its map in the hash order of the faces' TShape addresses; the port's map iterates in the
            // order the faces were bound (decision 18). Which face keeps the original degenerated edge may differ.
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> fIter in degF)
            {
                subs.Clear();
                isReplaced = false;
                Shape f = fIter.Key;
                NCollection_List<Shape> del = fIter.Value;
                var eIter = new NCollection_List<Shape>.Iterator(del);
                for (; eIter.More(); eIter.Next())
                {
                    Shape e = eIter.Value();
                    if (degE.ContainsKey(e))
                    {
                        NCollection_List<Shape> cel = degE.GetValue(e);
                        var it = new NCollection_List<Shape>.Iterator(cel);
                        for (; it.More(); it.Next())
                        {
                            if (it.Value().IsEqual(e))
                            {
                                // First occurrence of initial deg. edge is not replaced
                                cel.Remove(it);
                                break;
                            }
                            if (it.Value().Orientation == e.Orientation)
                            {
                                // All other occurrences of anE are replaced by any copy
                                // with suitable orientation
                                isReplaced = true;
                                subs.Replace(e, it.Value());
                                cel.Remove(it);
                                break;
                            }
                        }
                    }
                }
                if (isReplaced)
                {
                    Shape nf = subs.InternalApply(f);
                    subsF.Replace(f, nf);
                    if (_hist is null)
                    {
                        _hist = subs.InternalHistory();
                    }
                    else
                    {
                        _hist.Merge(subs.InternalHistory());
                    }
                    _shape = subsF.InternalApply(_shape);
                    _hist.Merge(subsF.InternalHistory());
                    // Pair aF->aNF is in history after first replacing of edge by aNF = aSubs.Apply(aF)
                    // After merging history for replacing faces, modified list for aF contains two exemplar of
                    // aNF So, using ReplaceModified clears modified list for aF and leaves only one exemplar of
                    // aNF
                    _hist.ReplaceModified(f, nf);
                    subsF.Clear();
                }
            }
        }

        FinishGenerating(ref _shape);
    }

    // used in CheckValidity to find out is there
    // intersection between curve and axe of revolution
    private static bool IsIntersect(CurveAdaptorBase c, in Axis3D axe)
    {
        var axis = new Line3D(axe);
        // Quick test for circle
        if (c.GetType() == Circle)
        {
            Circle3D circ = c.Circle();
            Point3D centr = circ.Location;
            double r2 = circ.Radius;
            r2 -= global::Kerf.Engine.Foundation.Precision.Confusion();
            r2 *= r2;
            if (axis.SquareDistanceTo(centr) > r2)
            {
                return false;
            }
        }
        var l = new LineCurve3D(axis);
        var lin = new CurveAdaptor(l);
        double parTol = c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        double parF = c.FirstParameter() + parTol, parL = c.LastParameter() - parTol;

        var extr = new ExtCC(c, lin);
        extr.Perform();
        if (extr.IsDone && extr.ExtCount > 0)
        {
            for (int i = 1; i <= extr.ExtCount; i++)
            {
                if (extr.SquareDistance(i) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    continue;
                }
                extr.Points(i, out POnCurv p1, out POnCurv _);
                if ((parF < p1.Parameter()) && (p1.Parameter() < parL))
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>False if an edge of <paramref name="shape"/> crosses the axis (edges lying on it are valid).</summary>
    protected bool CheckValidity(in Shape shape, in Axis3D a)
    {
        var exp = new Explorer(shape, ShapeType.Edge);
        bool isValid = true;
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            if (Tool.Degenerated(e))
            {
                continue;
            }

            Curve3D c = Tool.Curve(e, out Placement l, out double first, out double last)!;
            Transform3D tr = l.Transformation;
            c = (Curve3D)c.Copy();
            c = new TrimmedCurve3D(c, first, last);
            c.Transform(tr);

            var hc = new CurveAdaptor();
            hc.Load(c, first, last);

            // Checking coincidence axe of revolution and basis curve
            // This code is taken directly from GeomAdaptor_SurfaceOfRevolution
            int ratio = 1;
            double dist;
            Point3D pp;
            do
            {
                pp = hc.Value(first + (last - first) / ratio);
                dist = new Line3D(a).DistanceTo(pp);
                ratio++;
            } while (dist < global::Kerf.Engine.Foundation.Precision.Confusion() && ratio < 100);
            //
            if (ratio >= 100) // edge coincides with axes
            {
                isValid = true; // Such edges are allowed by revol algo and treated
                                // by special way, so they must be considered as valid
            }
            else
            {
                isValid = !IsIntersect(hc, a);
            }
            if (!isValid)
            {
                break;
            }
        }
        //
        return isValid;
    }

    internal override Shape InternalFirstShape() => FromSwept(_revol.FirstShape());

    internal override Shape InternalLastShape() => FromSwept(_revol.LastShape());

    /// <summary>The list of shapes generated from the shape <paramref name="s"/>.</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        using WorkingOverlay query = BeginQuery(); // Kerf: may build on demand
        Shape ss = s;

        if (!_revol.IsUsed(ss))
        {
            return _generated;
        }

        Shape gs = _revol.Shape(ss);
        if (!gs.IsNull)
        {
            if (History.IsSupportedType(gs))
            {
                if (gs.Type == ShapeType.Edge)
                {
                    bool isDeg = Tool.Degenerated(ShapeCasts.Edge(gs));
                    if (isDeg)
                    {
                        var it = new NCollection_List<Shape>.Iterator(_degenerated);
                        for (; it.More(); it.Next())
                        {
                            if (gs.IsSame(it.Value()))
                            {
                                _generated.Append(FromSwept(gs));
                                if (_hist is not null)
                                {
                                    foreach (Shape m in _hist.Modified(gs))
                                    {
                                        _generated.Append(FromSwept(m));
                                    }
                                    return _generated;
                                }
                            }
                        }
                        return _generated;
                    }
                }
                //
                if (_hist is null)
                {
                    _generated.Append(FromSwept(gs));
                    return _generated;
                }
                //
                ISchematiqList<Shape> modified = _hist.Modified(gs);
                if (modified.Count == 0)
                {
                    _generated.Append(FromSwept(gs));
                    return _generated;
                }
                //
                foreach (Shape m in modified)
                {
                    _generated.Append(FromSwept(m));
                }
            }
        }
        return _generated;
    }

    /// <summary>True if the shape <paramref name="s"/> has been deleted.</summary>
    public override bool IsDeleted(in Shape s)
    {
        return !_revol.IsUsed(s);
    }

    /// <summary>The shape at the beginning of the revolution generated with <paramref name="shape"/>, a sub-shape of the generating shape (frozen).</summary>
    public Shape FirstShape(in Shape shape) => Published(InternalFirstShape(shape));

    /// <summary><see cref="FirstShape(in Shape)"/> without freezing.</summary>
    internal Shape InternalFirstShape(in Shape shape) => FromSwept(_revol.FirstShape(shape));

    /// <summary>The shape at the end of the revolution generated with <paramref name="shape"/>, a sub-shape of the generating shape (frozen).</summary>
    public Shape LastShape(in Shape shape) => Published(InternalLastShape(shape));

    /// <summary><see cref="LastShape(in Shape)"/> without freezing.</summary>
    internal Shape InternalLastShape(in Shape shape) => FromSwept(_revol.LastShape(shape));

    /// <summary>True if the result has degenerated edges.</summary>
    public bool HasDegenerated => !_degenerated.IsEmpty();

    /// <summary>The degenerated edges of the result (frozen).</summary>
    public ISchematiqList<Shape> Degenerated()
    {
        // Kerf: a degenerated edge of the generating shape, as it stands in the result
        var degenerated = new NCollection_List<Shape>();
        for (var it = new NCollection_List<Shape>.Iterator(_degenerated); it.More(); it.Next())
        {
            degenerated.Append(FromSwept(it.Value()));
        }
        return PublishedList(degenerated);
    }

    /// <summary>The list of degenerated edges, unfrozen (OCCT's const reference).</summary>
    internal NCollection_List<Shape> InternalDegenerated() => _degenerated;
}
