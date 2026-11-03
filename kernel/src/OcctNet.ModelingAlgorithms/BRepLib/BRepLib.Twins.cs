// OCCT.NET: the functional twins of BRepLib's in-place statics (docs/phase3-design.md, section 4.6). Not a port of
// an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;

namespace Occt;

public static partial class BRepLib
{
    // The twin of an in-place static: the operation on an exact copy for write, whose sub-shapes it left unchanged are
    // the input's own again, frozen. The history records the input's sub-shapes that the result replaces.
    private static Shape onCopy(in Shape theShape, Action<Shape> theOperation, out History theHistory)
    {
        if (theShape.IsNull)
        {
            theHistory = new History();
            return theShape;
        }

        Shape aCopy = BrepTools.ExactCopyForWrite(theShape, out _);
        theOperation(aCopy);
        Shape aResult = BrepTools.KeepUnchanged(theShape, aCopy, out theHistory);
        aResult.TShape()!.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="SameParameter(in Shape, double, bool)"/>: the shape with new pcurves
    /// for its edges that weren't SameParameter (all of them if <paramref name="forced"/>), and its tolerances updated.
    /// (OCCT's non-destructive form, <see cref="SameParameter(in Shape, ReShape, double, bool)"/>, is
    /// public too; its results can differ from the in-place static's, which this twin reproduces.)
    /// </summary>
    public static Shape SameParametered(in Shape S, double Tolerance = 1.0e-5, bool forced = false)
        => SameParametered(S, out _, Tolerance, forced);

    /// <summary>The functional twin of SameParameter, with the history from the shape to the result.</summary>
    public static Shape SameParametered(in Shape S, out History theHistory, double Tolerance = 1.0e-5,
                                               bool forced = false)
        => onCopy(S, aCopy => SameParameter(aCopy, Tolerance, forced), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="UpdateTolerances(in Shape, bool)"/>: the shape with the tolerances of its
    /// faces, edges and vertices raised to cover their connected shapes. (OCCT's non-destructive form,
    /// <see cref="UpdateTolerances(in Shape, ReShape, bool)"/>, is public too.)
    /// </summary>
    public static Shape TolerancesUpdated(in Shape S, bool verifyFaceTolerance = false)
        => TolerancesUpdated(S, out _, verifyFaceTolerance);

    /// <summary>The functional twin of UpdateTolerances, with the history from the shape to the result.</summary>
    public static Shape TolerancesUpdated(in Shape S, out History theHistory, bool verifyFaceTolerance = false)
        => onCopy(S, aCopy => UpdateTolerances(aCopy, verifyFaceTolerance), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="UpdateInnerTolerances"/>: the shape whose edges' and vertices' tolerances
    /// cover the deviations of their curves and pcurves, inner points included.
    /// </summary>
    public static Shape InnerTolerancesUpdated(in Shape S) => InnerTolerancesUpdated(S, out _);

    /// <summary>The functional twin of UpdateInnerTolerances, with the history from the shape to the result.</summary>
    public static Shape InnerTolerancesUpdated(in Shape S, out History theHistory)
        => onCopy(S, aCopy => UpdateInnerTolerances(aCopy), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="BuildCurves3d(in Shape, double, Continuity, int, int)"/>: the shape
    /// whose edges without a 3D curve have one, computed from their pcurves. <paramref name="theIsDone"/> is
    /// BuildCurves3d's result (false if one of the computations failed).
    /// </summary>
    public static Shape Curves3dBuilt(in Shape S,
                                             double Tolerance,
                                             out bool theIsDone,
                                             Continuity Continuity = Continuity.C1,
                                             int MaxDegree = 14,
                                             int MaxSegment = 0)
        => Curves3dBuilt(S, Tolerance, out theIsDone, out _, Continuity, MaxDegree, MaxSegment);

    /// <summary>The functional twin of BuildCurves3d, with the history from the shape to the result.</summary>
    public static Shape Curves3dBuilt(in Shape S,
                                             double Tolerance,
                                             out bool theIsDone,
                                             out History theHistory,
                                             Continuity Continuity = Continuity.C1,
                                             int MaxDegree = 14,
                                             int MaxSegment = 0)
    {
        bool isDone = true;
        Shape aResult = onCopy(S, aCopy => isDone = BuildCurves3d(aCopy, Tolerance, Continuity, MaxDegree, MaxSegment),
                                      out theHistory);
        theIsDone = isDone;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="BuildCurves3d(in Shape)"/> (tolerance 1e-5, C1, degree 14).
    /// </summary>
    public static Shape Curves3dBuilt(in Shape S, out bool theIsDone)
    {
        bool isDone = true;
        Shape aResult = onCopy(S, aCopy => isDone = BuildCurves3d(aCopy), out _);
        theIsDone = isDone;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="EncodeRegularity(in Shape, double)"/>: the shape whose edges between
    /// two faces (or twice on one face) carry the continuity across them, where it wasn't coded before.
    /// </summary>
    public static Shape RegularityEncoded(in Shape S, double TolAng = 1.0e-10) => RegularityEncoded(S, out _, TolAng);

    /// <summary>The functional twin of EncodeRegularity, with the history from the shape to the result.</summary>
    public static Shape RegularityEncoded(in Shape S, out History theHistory, double TolAng = 1.0e-10)
        => onCopy(S, aCopy => EncodeRegularity(aCopy, TolAng), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="EncodeRegularity(in Shape, NCollection_List{Shape}, double)"/>:
    /// the regularity of the listed edges of the shape only.
    /// </summary>
    public static Shape RegularityEncoded(in Shape S, IEnumerable<Shape> LE, double TolAng = 1.0e-10)
        => RegularityEncoded(S, LE, out _, TolAng);

    /// <summary>The functional twin of EncodeRegularity on listed edges, with the history from the shape to the result.</summary>
    public static Shape RegularityEncoded(in Shape S, IEnumerable<Shape> LE, out History theHistory,
                                                 double TolAng = 1.0e-10)
    {
        if (S.IsNull)
        {
            theHistory = new History();
            return S;
        }

        Shape aCopy = BrepTools.ExactCopyForWrite(S, out History aCopyHistory);
        // the listed edges, as the copy's
        var aList = new NCollection_List<Shape>();
        foreach (Shape anE in LE)
        {
            ISchematiqList<Shape> aModified = aCopyHistory.Modified(anE);
            aList.Append(aModified.Count > 0 ? aModified[0].Oriented(anE.Orientation()) : anE);
        }
        EncodeRegularity(aCopy, aList, TolAng);
        Shape aResult = BrepTools.KeepUnchanged(S, aCopy, out theHistory);
        aResult.TShape()!.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="OrientClosedSolid"/>: the solid oriented to hold matter inside its shells
    /// (forward or reversed: OrientClosedSolid only reverses the value). <paramref name="theIsOriented"/> is false if
    /// the solid is open or incoherent, and the solid is then returned as it is.
    /// </summary>
    public static Solid ClosedSolidOriented(in Solid solid, out bool theIsOriented)
    {
        Solid aSolid = solid;
        theIsOriented = OrientClosedSolid(ref aSolid);
        return theIsOriented ? aSolid : solid;
    }

    /// <summary>
    /// The functional twin of <see cref="UpdateDeflection"/>: the shape whose faces' triangulations (copies, with their
    /// edges' polygons) carry the maximal deviation of the mesh from the surface as their deflection.
    /// </summary>
    public static Shape DeflectionUpdated(in Shape S) => DeflectionUpdated(S, out _);

    /// <summary>The functional twin of UpdateDeflection, with the history from the shape to the result.</summary>
    public static Shape DeflectionUpdated(in Shape S, out History theHistory)
        => onCopy(S, aCopy => UpdateDeflection(aCopy), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="EnsureNormalConsistency"/>: the shape whose faces' triangulations (copies) have
    /// normals that agree across smooth edges. <paramref name="theIsCorrected"/> is EnsureNormalConsistency's result.
    /// </summary>
    public static Shape NormalConsistencyEnsured(in Shape S,
                                                        out bool theIsCorrected,
                                                        double theAngTol = 0.001,
                                                        bool theForceComputeNormals = false)
        => NormalConsistencyEnsured(S, out theIsCorrected, out _, theAngTol, theForceComputeNormals);

    /// <summary>The functional twin of EnsureNormalConsistency, with the history from the shape to the result.</summary>
    public static Shape NormalConsistencyEnsured(in Shape S,
                                                        out bool theIsCorrected,
                                                        out History theHistory,
                                                        double theAngTol = 0.001,
                                                        bool theForceComputeNormals = false)
    {
        bool isCorrected = false;
        Shape aResult = onCopy(S, aCopy => isCorrected = EnsureNormalConsistency(aCopy, theAngTol, theForceComputeNormals),
                                      out theHistory);
        theIsCorrected = isCorrected;
        return aResult;
    }
}
