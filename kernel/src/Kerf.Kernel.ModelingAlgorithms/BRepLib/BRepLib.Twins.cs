// OCCT.NET: the functional twins of BRepLib's in-place statics (docs/phase3-design.md, section 4.6). Not a port of
// an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;

namespace Kerf.Engine.Construction.Library;

public static partial class BrepLibrary
{
    // The twin of an in-place static: the operation on an exact copy for write, whose sub-shapes it left unchanged are
    // the input's own again, frozen. The history records the input's sub-shapes that the result replaces.
    private static Shape OnCopy(in Shape shape, Action<Shape> operation, out History history)
    {
        if (shape.IsNull)
        {
            history = new History();
            return shape;
        }

        Shape copy = BrepTools.ExactCopyForWrite(shape, out _);
        operation(copy);
        Shape result = BrepTools.KeepUnchanged(shape, copy, out history);
        result.TShape()!.FreezeGraph();
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="SameParameter(in Shape, double, bool)"/>: the shape with new pcurves
    /// for its edges that weren't SameParameter (all of them if <paramref name="forced"/>), and its tolerances updated.
    /// (OCCT's non-destructive form, <see cref="SameParameter(in Shape, ReShape, double, bool)"/>, is
    /// public too; its results can differ from the in-place static's, which this twin reproduces.)
    /// </summary>
    public static Shape SameParametered(in Shape s, double tolerance = 1.0e-5, bool forced = false)
        => SameParametered(s, out _, tolerance, forced);

    /// <summary>The functional twin of SameParameter, with the history from the shape to the result.</summary>
    public static Shape SameParametered(in Shape s, out History history, double tolerance = 1.0e-5,
                                               bool forced = false)
        => OnCopy(s, aCopy => SameParameter(aCopy, tolerance, forced), out history);

    /// <summary>
    /// The functional twin of <see cref="UpdateTolerances(in Shape, bool)"/>: the shape with the tolerances of its
    /// faces, edges and vertices raised to cover their connected shapes. (OCCT's non-destructive form,
    /// <see cref="UpdateTolerances(in Shape, ReShape, bool)"/>, is public too.)
    /// </summary>
    public static Shape TolerancesUpdated(in Shape s, bool verifyFaceTolerance = false)
        => TolerancesUpdated(s, out _, verifyFaceTolerance);

    /// <summary>The functional twin of UpdateTolerances, with the history from the shape to the result.</summary>
    public static Shape TolerancesUpdated(in Shape s, out History history, bool verifyFaceTolerance = false)
        => OnCopy(s, aCopy => UpdateTolerances(aCopy, verifyFaceTolerance), out history);

    /// <summary>
    /// The functional twin of <see cref="UpdateInnerTolerances"/>: the shape whose edges' and vertices' tolerances
    /// cover the deviations of their curves and pcurves, inner points included.
    /// </summary>
    public static Shape InnerTolerancesUpdated(in Shape s) => InnerTolerancesUpdated(s, out _);

    /// <summary>The functional twin of UpdateInnerTolerances, with the history from the shape to the result.</summary>
    public static Shape InnerTolerancesUpdated(in Shape s, out History history)
        => OnCopy(s, aCopy => UpdateInnerTolerances(aCopy), out history);

    /// <summary>
    /// The functional twin of <see cref="BuildCurves3d(in Shape, double, Continuity, int, int)"/>: the shape
    /// whose edges without a 3D curve have one, computed from their pcurves. <paramref name="isDone"/> is
    /// BuildCurves3d's result (false if one of the computations failed).
    /// </summary>
    public static Shape Curves3dBuilt(in Shape s,
                                             double tolerance,
                                             out bool isDone,
                                             Continuity continuity = Continuity.C1,
                                             int maxDegree = 14,
                                             int maxSegment = 0)
        => Curves3dBuilt(s, tolerance, out isDone, out _, continuity, maxDegree, maxSegment);

    /// <summary>The functional twin of BuildCurves3d, with the history from the shape to the result.</summary>
    public static Shape Curves3dBuilt(in Shape s,
                                             double tolerance,
                                             out bool theIsDone,
                                             out History history,
                                             Continuity continuity = Continuity.C1,
                                             int maxDegree = 14,
                                             int maxSegment = 0)
    {
        bool isDone = true;
        Shape result = OnCopy(s, aCopy => isDone = BuildCurves3d(aCopy, tolerance, continuity, maxDegree, maxSegment),
                                      out history);
        theIsDone = isDone;
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="BuildCurves3d(in Shape)"/> (tolerance 1e-5, C1, degree 14).
    /// </summary>
    public static Shape Curves3dBuilt(in Shape s, out bool theIsDone)
    {
        bool isDone = true;
        Shape result = OnCopy(s, aCopy => isDone = BuildCurves3d(aCopy), out _);
        theIsDone = isDone;
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="EncodeRegularity(in Shape, double)"/>: the shape whose edges between
    /// two faces (or twice on one face) carry the continuity across them, where it wasn't coded before.
    /// </summary>
    public static Shape RegularityEncoded(in Shape s, double tolAng = 1.0e-10) => RegularityEncoded(s, out _, tolAng);

    /// <summary>The functional twin of EncodeRegularity, with the history from the shape to the result.</summary>
    public static Shape RegularityEncoded(in Shape s, out History history, double tolAng = 1.0e-10)
        => OnCopy(s, aCopy => EncodeRegularity(aCopy, tolAng), out history);

    /// <summary>
    /// The functional twin of <see cref="EncodeRegularity(in Shape, NCollection_List{Shape}, double)"/>:
    /// the regularity of the listed edges of the shape only.
    /// </summary>
    public static Shape RegularityEncoded(in Shape s, IEnumerable<Shape> le, double tolAng = 1.0e-10)
        => RegularityEncoded(s, le, out _, tolAng);

    /// <summary>The functional twin of EncodeRegularity on listed edges, with the history from the shape to the result.</summary>
    public static Shape RegularityEncoded(in Shape s, IEnumerable<Shape> le, out History history,
                                                 double tolAng = 1.0e-10)
    {
        if (s.IsNull)
        {
            history = new History();
            return s;
        }

        Shape copy = BrepTools.ExactCopyForWrite(s, out History copyHistory);
        // the listed edges, as the copy's
        var list = new NCollection_List<Shape>();
        foreach (Shape e in le)
        {
            ISchematiqList<Shape> modified = copyHistory.Modified(e);
            list.Append(modified.Count > 0 ? modified[0].Oriented(e.Orientation()) : e);
        }
        EncodeRegularity(copy, list, tolAng);
        Shape result = BrepTools.KeepUnchanged(s, copy, out history);
        result.TShape()!.FreezeGraph();
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="OrientClosedSolid"/>: the solid oriented to hold matter inside its shells
    /// (forward or reversed: OrientClosedSolid only reverses the value). <paramref name="isOriented"/> is false if
    /// the solid is open or incoherent, and the solid is then returned as it is.
    /// </summary>
    public static Solid ClosedSolidOriented(in Solid solid, out bool isOriented)
    {
        Solid aSolid = solid;
        isOriented = OrientClosedSolid(ref aSolid);
        return isOriented ? aSolid : solid;
    }

    /// <summary>
    /// The functional twin of <see cref="UpdateDeflection"/>: the shape whose faces' triangulations (copies, with their
    /// edges' polygons) carry the maximal deviation of the mesh from the surface as their deflection.
    /// </summary>
    public static Shape DeflectionUpdated(in Shape s) => DeflectionUpdated(s, out _);

    /// <summary>The functional twin of UpdateDeflection, with the history from the shape to the result.</summary>
    public static Shape DeflectionUpdated(in Shape s, out History history)
        => OnCopy(s, aCopy => UpdateDeflection(aCopy), out history);

    /// <summary>
    /// The functional twin of <see cref="EnsureNormalConsistency"/>: the shape whose faces' triangulations (copies) have
    /// normals that agree across smooth edges. <paramref name="isCorrected"/> is EnsureNormalConsistency's result.
    /// </summary>
    public static Shape NormalConsistencyEnsured(in Shape s,
                                                        out bool isCorrected,
                                                        double angTol = 0.001,
                                                        bool forceComputeNormals = false)
        => NormalConsistencyEnsured(s, out isCorrected, out _, angTol, forceComputeNormals);

    /// <summary>The functional twin of EnsureNormalConsistency, with the history from the shape to the result.</summary>
    public static Shape NormalConsistencyEnsured(in Shape s,
                                                        out bool theIsCorrected,
                                                        out History history,
                                                        double angTol = 0.001,
                                                        bool forceComputeNormals = false)
    {
        bool isCorrected = false;
        Shape result = OnCopy(s, aCopy => isCorrected = EnsureNormalConsistency(aCopy, angTol, forceComputeNormals),
                                      out history);
        theIsCorrected = isCorrected;
        return result;
    }
}
