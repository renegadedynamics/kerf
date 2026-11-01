namespace Build123d.Occt;

public partial class BRepLib
{
    // OCCT edits the shape in place; OCCT.NET's functional twin returns the edited copy, which replaces the wrapper's shape

    public static bool BuildCurves3d(TopoDS_Shape S) => Build(S, s => (global::Occt.BRepLib.Curves3dBuilt(s, out var done), done));

    public static bool BuildCurves3d(TopoDS_Shape S, double Tolerance) =>
        Build(S, s => (global::Occt.BRepLib.Curves3dBuilt(s, Tolerance, out var done), done));

    public static bool BuildCurves3d(TopoDS_Shape S, double Tolerance, GeomAbs_Shape Continuity) =>
        Build(S, s => (global::Occt.BRepLib.Curves3dBuilt(s, Tolerance, out var done, (global::Occt.GeomAbs_Shape)Continuity), done));

    public static bool BuildCurves3d(TopoDS_Shape S, double Tolerance, GeomAbs_Shape Continuity, int MaxDegree) =>
        Build(S, s => (global::Occt.BRepLib.Curves3dBuilt(s, Tolerance, out var done, (global::Occt.GeomAbs_Shape)Continuity, MaxDegree), done));

    public static bool BuildCurves3d(TopoDS_Shape S, double Tolerance, GeomAbs_Shape Continuity, int MaxDegree, int MaxSegment) =>
        Build(S, s => (global::Occt.BRepLib.Curves3dBuilt(s, Tolerance, out var done, (global::Occt.GeomAbs_Shape)Continuity, MaxDegree, MaxSegment), done));

    private static bool Build(TopoDS_Shape S, Func<global::Occt.TopoDS_Shape, (global::Occt.TopoDS_Shape Shape, bool Done)> twin)
    {
        ArgumentNullException.ThrowIfNull(S);
        try
        {
            var (shape, done) = twin(S.V);
            S.V = shape;
            return done;
        }
        catch (global::Occt.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }
}
