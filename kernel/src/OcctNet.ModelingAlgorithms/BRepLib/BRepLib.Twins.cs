// OCCT.NET: the functional twins of BRepLib's in-place statics (docs/phase3-design.md, section 4.6). Not a port of
// an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;

namespace Occt;

public static partial class BRepLib
{
    // The twin of an in-place static: the operation on an exact copy for write, whose sub-shapes it left unchanged are
    // the input's own again, frozen. The history records the input's sub-shapes that the result replaces.
    private static TopoDS_Shape onCopy(in TopoDS_Shape theShape, Action<TopoDS_Shape> theOperation, out BRepTools_History theHistory)
    {
        if (theShape.IsNull())
        {
            theHistory = new BRepTools_History();
            return theShape;
        }

        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(theShape, out _);
        theOperation(aCopy);
        TopoDS_Shape aResult = BRepTools.KeepUnchanged(theShape, aCopy, out theHistory);
        aResult.TShape()!.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="SameParameter(in TopoDS_Shape, double, bool)"/>: the shape with new pcurves
    /// for its edges that weren't SameParameter (all of them if <paramref name="forced"/>), and its tolerances updated.
    /// (OCCT's non-destructive form, <see cref="SameParameter(in TopoDS_Shape, BRepTools_ReShape, double, bool)"/>, is
    /// public too; its results can differ from the in-place static's, which this twin reproduces.)
    /// </summary>
    public static TopoDS_Shape SameParametered(in TopoDS_Shape S, double Tolerance = 1.0e-5, bool forced = false)
        => SameParametered(S, out _, Tolerance, forced);

    /// <summary>The functional twin of SameParameter, with the history from the shape to the result.</summary>
    public static TopoDS_Shape SameParametered(in TopoDS_Shape S, out BRepTools_History theHistory, double Tolerance = 1.0e-5,
                                               bool forced = false)
        => onCopy(S, aCopy => SameParameter(aCopy, Tolerance, forced), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="UpdateTolerances(in TopoDS_Shape, bool)"/>: the shape with the tolerances of its
    /// faces, edges and vertices raised to cover their connected shapes. (OCCT's non-destructive form,
    /// <see cref="UpdateTolerances(in TopoDS_Shape, BRepTools_ReShape, bool)"/>, is public too.)
    /// </summary>
    public static TopoDS_Shape TolerancesUpdated(in TopoDS_Shape S, bool verifyFaceTolerance = false)
        => TolerancesUpdated(S, out _, verifyFaceTolerance);

    /// <summary>The functional twin of UpdateTolerances, with the history from the shape to the result.</summary>
    public static TopoDS_Shape TolerancesUpdated(in TopoDS_Shape S, out BRepTools_History theHistory, bool verifyFaceTolerance = false)
        => onCopy(S, aCopy => UpdateTolerances(aCopy, verifyFaceTolerance), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="UpdateInnerTolerances"/>: the shape whose edges' and vertices' tolerances
    /// cover the deviations of their curves and pcurves, inner points included.
    /// </summary>
    public static TopoDS_Shape InnerTolerancesUpdated(in TopoDS_Shape S) => InnerTolerancesUpdated(S, out _);

    /// <summary>The functional twin of UpdateInnerTolerances, with the history from the shape to the result.</summary>
    public static TopoDS_Shape InnerTolerancesUpdated(in TopoDS_Shape S, out BRepTools_History theHistory)
        => onCopy(S, aCopy => UpdateInnerTolerances(aCopy), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="BuildCurves3d(in TopoDS_Shape, double, GeomAbs_Shape, int, int)"/>: the shape
    /// whose edges without a 3D curve have one, computed from their pcurves. <paramref name="theIsDone"/> is
    /// BuildCurves3d's result (false if one of the computations failed).
    /// </summary>
    public static TopoDS_Shape Curves3dBuilt(in TopoDS_Shape S,
                                             double Tolerance,
                                             out bool theIsDone,
                                             GeomAbs_Shape Continuity = GeomAbs_Shape.GeomAbs_C1,
                                             int MaxDegree = 14,
                                             int MaxSegment = 0)
        => Curves3dBuilt(S, Tolerance, out theIsDone, out _, Continuity, MaxDegree, MaxSegment);

    /// <summary>The functional twin of BuildCurves3d, with the history from the shape to the result.</summary>
    public static TopoDS_Shape Curves3dBuilt(in TopoDS_Shape S,
                                             double Tolerance,
                                             out bool theIsDone,
                                             out BRepTools_History theHistory,
                                             GeomAbs_Shape Continuity = GeomAbs_Shape.GeomAbs_C1,
                                             int MaxDegree = 14,
                                             int MaxSegment = 0)
    {
        bool isDone = true;
        TopoDS_Shape aResult = onCopy(S, aCopy => isDone = BuildCurves3d(aCopy, Tolerance, Continuity, MaxDegree, MaxSegment),
                                      out theHistory);
        theIsDone = isDone;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="BuildCurves3d(in TopoDS_Shape)"/> (tolerance 1e-5, C1, degree 14).
    /// </summary>
    public static TopoDS_Shape Curves3dBuilt(in TopoDS_Shape S, out bool theIsDone)
    {
        bool isDone = true;
        TopoDS_Shape aResult = onCopy(S, aCopy => isDone = BuildCurves3d(aCopy), out _);
        theIsDone = isDone;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="EncodeRegularity(in TopoDS_Shape, double)"/>: the shape whose edges between
    /// two faces (or twice on one face) carry the continuity across them, where it wasn't coded before.
    /// </summary>
    public static TopoDS_Shape RegularityEncoded(in TopoDS_Shape S, double TolAng = 1.0e-10) => RegularityEncoded(S, out _, TolAng);

    /// <summary>The functional twin of EncodeRegularity, with the history from the shape to the result.</summary>
    public static TopoDS_Shape RegularityEncoded(in TopoDS_Shape S, out BRepTools_History theHistory, double TolAng = 1.0e-10)
        => onCopy(S, aCopy => EncodeRegularity(aCopy, TolAng), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="EncodeRegularity(in TopoDS_Shape, NCollection_List{TopoDS_Shape}, double)"/>:
    /// the regularity of the listed edges of the shape only.
    /// </summary>
    public static TopoDS_Shape RegularityEncoded(in TopoDS_Shape S, IEnumerable<TopoDS_Shape> LE, double TolAng = 1.0e-10)
        => RegularityEncoded(S, LE, out _, TolAng);

    /// <summary>The functional twin of EncodeRegularity on listed edges, with the history from the shape to the result.</summary>
    public static TopoDS_Shape RegularityEncoded(in TopoDS_Shape S, IEnumerable<TopoDS_Shape> LE, out BRepTools_History theHistory,
                                                 double TolAng = 1.0e-10)
    {
        if (S.IsNull())
        {
            theHistory = new BRepTools_History();
            return S;
        }

        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(S, out BRepTools_History aCopyHistory);
        // the listed edges, as the copy's
        var aList = new NCollection_List<TopoDS_Shape>();
        foreach (TopoDS_Shape anE in LE)
        {
            ISchematiqList<TopoDS_Shape> aModified = aCopyHistory.Modified(anE);
            aList.Append(aModified.Count > 0 ? aModified[0].Oriented(anE.Orientation()) : anE);
        }
        EncodeRegularity(aCopy, aList, TolAng);
        TopoDS_Shape aResult = BRepTools.KeepUnchanged(S, aCopy, out theHistory);
        aResult.TShape()!.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="OrientClosedSolid"/>: the solid oriented to hold matter inside its shells
    /// (forward or reversed: OrientClosedSolid only reverses the value). <paramref name="theIsOriented"/> is false if
    /// the solid is open or incoherent, and the solid is then returned as it is.
    /// </summary>
    public static TopoDS_Solid ClosedSolidOriented(in TopoDS_Solid solid, out bool theIsOriented)
    {
        TopoDS_Solid aSolid = solid;
        theIsOriented = OrientClosedSolid(ref aSolid);
        return theIsOriented ? aSolid : solid;
    }

    /// <summary>
    /// The functional twin of <see cref="UpdateDeflection"/>: the shape whose faces' triangulations (copies, with their
    /// edges' polygons) carry the maximal deviation of the mesh from the surface as their deflection.
    /// </summary>
    public static TopoDS_Shape DeflectionUpdated(in TopoDS_Shape S) => DeflectionUpdated(S, out _);

    /// <summary>The functional twin of UpdateDeflection, with the history from the shape to the result.</summary>
    public static TopoDS_Shape DeflectionUpdated(in TopoDS_Shape S, out BRepTools_History theHistory)
        => onCopy(S, aCopy => UpdateDeflection(aCopy), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="EnsureNormalConsistency"/>: the shape whose faces' triangulations (copies) have
    /// normals that agree across smooth edges. <paramref name="theIsCorrected"/> is EnsureNormalConsistency's result.
    /// </summary>
    public static TopoDS_Shape NormalConsistencyEnsured(in TopoDS_Shape S,
                                                        out bool theIsCorrected,
                                                        double theAngTol = 0.001,
                                                        bool theForceComputeNormals = false)
        => NormalConsistencyEnsured(S, out theIsCorrected, out _, theAngTol, theForceComputeNormals);

    /// <summary>The functional twin of EnsureNormalConsistency, with the history from the shape to the result.</summary>
    public static TopoDS_Shape NormalConsistencyEnsured(in TopoDS_Shape S,
                                                        out bool theIsCorrected,
                                                        out BRepTools_History theHistory,
                                                        double theAngTol = 0.001,
                                                        bool theForceComputeNormals = false)
    {
        bool isCorrected = false;
        TopoDS_Shape aResult = onCopy(S, aCopy => isCorrected = EnsureNormalConsistency(aCopy, theAngTol, theForceComputeNormals),
                                      out theHistory);
        theIsCorrected = isCorrected;
        return aResult;
    }
}
