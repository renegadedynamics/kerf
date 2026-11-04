using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

namespace Build123d.Tests;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// The engine's own views of shapes, for the tests that look past the public API (the modelling layer itself is on the
/// kernel's public API alone since unit 10M1, which moved these here): a copy of a shape carrying OCCT's
/// triangulation, and the native shim's estimate of a shape's memory.
/// </summary>
internal static class EngineShapes
{
    /// <summary>A copy of the shape carrying OCCT's triangulation (<c>BRepMesh_IncrementalMesh</c>), as the modelling
    /// layer's <c>MeshedCopy</c> made one before unit 10M1: meshing never writes into the public API's shapes.</summary>
    public static global::Kerf.Topology.Shape MeshedCopy(this Shape shape, double tolerance, double angularTolerance, bool relative = true)
    {
        if (shape.IsEmpty)
            throw new InvalidOperationException("Cannot mesh an empty shape");
        return new IncrementalMesh(shape.TopoShape, tolerance, relative, angularTolerance, true).Shape();
    }

    /// <summary>
    /// A rough estimate of the memory a shape holds, in bytes: its sub-shapes, curves, surfaces and triangulations,
    /// each counted once however often it's shared (the native shim's estimate, for the same shapes).
    /// </summary>
    public static long EstimateMemory(global::Kerf.Topology.Shape shape)
    {
        try
        {
            return new MemoryEstimate().Of(shape);
        }
        catch (FailureException)
        {
            return 0;
        }
    }

    /// <summary>The shim's MemoryEstimate, over OCCT.NET's representations.</summary>
    private sealed class MemoryEstimate
    {
        private readonly HashSet<object> _seen = new(ReferenceEqualityComparer.Instance);
        private long _bytes;

        public long Of(global::Kerf.Topology.Shape shape)
        {
            if (!shape.IsNull)
                Visit(shape);
            return _bytes;
        }

        private bool First(object? o) => o is not null && _seen.Add(o);

        private void Visit(global::Kerf.Topology.Shape shape)
        {
            var tshape = shape.TShape();
            if (!First(tshape))
                return;
            _bytes += 96;
            if (tshape is BrepTEdge tedge)
                foreach (var rep in tedge.CurveRepresentations())
                {
                    _bytes += 64;
                    if (rep.IsCurve3D)
                        Curve(rep.Curve3D());
                    else if (rep.IsCurveOnSurface())
                    {
                        Curve2d(rep.PCurve());
                        if (rep.IsCurveOnClosedSurface)
                            Curve2d(rep.PCurve2());
                    }
                    else if (rep.IsPolygon3D && rep.Polygon3D() is { } p && First(p))
                        _bytes += 32L * p.NodeCount;
                    else if (rep.IsPolygonOnTriangulation() && rep.PolygonOnTriangulation() is { } pt && First(pt))
                        _bytes += 12L * pt.NodeCount;
                }
            else if (tshape is BrepTFace tface)
            {
                Surface(tface.Surface());
                foreach (var tri in tface.TriangulationList())
                    if (First(tri))
                        _bytes += (long)tri.NodeCount * (24 + (tri.HasUVNodes ? 16 : 0) + (tri.HasNormals ? 12 : 0)) + 12L * tri.TriangleCount;
            }
            for (var it = new Iterator(shape, false, false); it.More(); it.Next())
                Visit(it.Value());
        }

        private void Curve(Curve3D? c)
        {
            if (!First(c))
                return;
            _bytes += 80;
            switch (c)
            {
                case BSplineCurve3D b: _bytes += 32L * b.PoleCount + 12L * b.KnotCount; break;
                case BezierCurve3D z: _bytes += 32L * z.PoleCount; break;
                case TrimmedCurve3D t: Curve(t.BasisCurve); break;
                case OffsetCurve3D o: Curve(o.BasisCurve); break;
            }
        }

        private void Curve2d(Curve2D? c)
        {
            if (!First(c))
                return;
            _bytes += 64;
            switch (c)
            {
                case BSplineCurve2D b: _bytes += 24L * b.PoleCount + 12L * b.KnotCount; break;
                case BezierCurve2D z: _bytes += 24L * z.PoleCount; break;
                case TrimmedCurve2D t: Curve2d(t.BasisCurve); break;
                case OffsetCurve2D o: Curve2d(o.BasisCurve); break;
            }
        }

        private void Surface(Surface? s)
        {
            if (!First(s))
                return;
            _bytes += 120;
            switch (s)
            {
                case BSplineSurface b: _bytes += 32L * b.UPoleCount * b.VPoleCount + 12L * (b.UKnotCount + b.VKnotCount); break;
                case BezierSurface z: _bytes += 32L * z.UPoleCount * z.VPoleCount; break;
                case TrimmedSurface t: Surface(t.BasisSurface); break;
                case OffsetSurface o: Surface(o.BasisSurface); break;
                case SweptSurface w: Curve(w.BasisCurve); break;
            }
        }
    }
}
