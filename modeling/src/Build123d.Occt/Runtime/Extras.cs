using System.Runtime.InteropServices;

namespace Build123d.Occt;

public partial class TopoDS_Shape
{
    /// <summary>
    /// <c>std::hash&lt;TopoDS_Shape&gt;</c>: equal for shapes that are <c>IsSame</c> (the same TShape at the
    /// same Location), whatever their orientation.
    /// </summary>
    public ulong StdHash()
    {
        var h = Extras.N.b3d_TopoDS_Shape_hash(PtrAs_TopoDS_Shape());
        GC.KeepAlive(this);
        return h;
    }
}

/// <summary>Hand-written shim functions (native/shim/src/b3d_extras.cpp) for what OcctGen can't bind.</summary>
public static partial class Extras
{
    /// <summary>
    /// Whether the edge's own 3D curve lies in a plane normal to <paramref name="normal"/>
    /// (<c>ShapeAnalysis_Curve::IsPlanar</c> on the curve in <c>BRep_TEdge::Curves()</c>), or null if it has none.
    /// </summary>
    public static bool? EdgeCurve3DIsPlanar(TopoDS_Edge edge, gp_XYZ normal, double tolerance)
    {
        ArgumentNullException.ThrowIfNull(edge);
        ArgumentNullException.ThrowIfNull(normal);
        var r = N.b3d_edge_curve3d_is_planar(edge.PtrAs_TopoDS_Edge(), normal.PtrAs_gp_XYZ(), tolerance, out var err);
        GC.KeepAlive(edge);
        GC.KeepAlive(normal);
        OcctException.ThrowIfError(err);
        return r < 0 ? null : r == 1;
    }

    /// <summary>
    /// A copy of a history with its maps copied as they are. <c>Merge</c> into an empty history isn't a copy: it drops
    /// the removal of a shape that also generated or was modified.
    /// </summary>
    public static BRepTools_History CopyHistory(BRepTools_History history)
    {
        ArgumentNullException.ThrowIfNull(history);
        var r = N.b3d_BRepTools_History_copy(history.PtrAs_BRepTools_History(), out var err);
        GC.KeepAlive(history);
        OcctException.ThrowIfError(err);
        return new BRepTools_History(r);
    }

    internal static partial class N
    {
        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_BRepTools_History_copy(IntPtr history, out IntPtr err);

        [LibraryImport("b3docct")]
        internal static partial ulong b3d_TopoDS_Shape_hash(IntPtr shape);

        [LibraryImport("b3docct")]
        internal static partial int b3d_edge_curve3d_is_planar(IntPtr edge, IntPtr normal, double tolerance, out IntPtr err);
    }
}
