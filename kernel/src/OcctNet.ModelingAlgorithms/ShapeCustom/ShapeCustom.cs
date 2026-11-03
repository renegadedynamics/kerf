// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Converts the geometry of shapes: B-spline and Bezier surfaces to analytical ones, indirect elementary surfaces
/// (left-handed frames) to direct ones, elementary surfaces to surfaces of revolution, extrusions, revolutions,
/// offsets and planes to B-splines, the degree and number of spans of B-splines, and scales shapes. Each function
/// applies a <see cref="Modification"/> with <see cref="Modifier"/>.
/// </summary>
/// <remarks>
/// <para>
/// The caller's shape is never written into (decision 973): the modifier rebuilds every sub-shape whose geometry
/// changes, and gives back the caller's own sub-shapes elsewhere. The functions are publication points: they freeze
/// the shape they return. Ported code that goes on building on the result calls <see cref="InternalApplyModifier"/>,
/// which doesn't freeze, and freezes at its own publication point.
/// </para>
/// </remarks>
public static class ShapeCustom
{
    // Updates the reshaper by the info of the given shape.
    private static bool UpdateHistoryShape(in Shape theShape,
                                           Modifier theModifier,
                                           ShapeBuild_ReShape theReShape)
    {
        Shape aResult;
        try
        {
            // OCCT.NET: the internal accessor, so the shapes recorded aren't frozen before the whole result is built
            // (decision 973)
            aResult = theModifier.InternalModifiedShape(theShape);
        }
        catch (NoSuchObjectException)
        {
            // the sub shape isn't in the map
            aResult = new Shape();
        }

        if (!aResult.IsNull && !theShape.IsSame(aResult))
        {
            theReShape.Replace(theShape, aResult);
            return true;
        }

        return false;
    }

    // Recursively updates the reshaper to add information of all sub-shapes.
    private static void UpdateHistory(in Shape theShape,
                                      Modifier theModifier,
                                      ShapeBuild_ReShape theReShape)
    {
        for (var theIterator = new Iterator(theShape, false); theIterator.More(); theIterator.Next())
        {
            Shape aCurrent = theIterator.Value();
            if (UpdateHistoryShape(aCurrent, theModifier, theReShape))
            {
                UpdateHistory(aCurrent, theModifier, theReShape);
            }
        }
    }

    // Recursively updates the reshaper to add information of all sub-shapes.
    private static void UpdateShapeBuild(in Shape theShape,
                                         Modifier theModifier,
                                         ShapeBuild_ReShape theReShape)
    {
        UpdateHistoryShape(theShape, theModifier, theReShape);
        UpdateHistory(theShape, theModifier, theReShape);
    }

    /// <summary>
    /// Applies the modification to the shape with the modifier <paramref name="MD"/>, sharing the results of
    /// compounds' children that are the same shape (assemblies) through <paramref name="context"/>, and records the
    /// replacements in <paramref name="aReShape"/> if given. The result is frozen (a publication point).
    /// </summary>
    /// <remarks>
    /// The modifier keeps its maps from one call to the next, as OCCT's: a modifier reused for a shape that shares
    /// sub-shapes with an earlier call's result would add pcurves to that result's edges, which are frozen, and throws
    /// <see cref="LockedShapeException"/> (decision 973). Use a new modifier for each call.
    /// </remarks>
    public static Shape ApplyModifier(in Shape S,
                                             Modification M,
                                             NCollection_DataMap<Shape, Shape> context,
                                             Modifier MD,
                                             ProgressRange theProgress = default,
                                             ShapeBuild_ReShape? aReShape = null)
    {
        Shape aResult = InternalApplyModifier(S, M, context, MD, theProgress, aReShape);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// <see cref="ApplyModifier"/> without freezing the result, for ported code that goes on building on it (decision
    /// 8101). The shapes it returns, binds in the context and records in the reshaper are frozen once the caller
    /// freezes the result.
    /// </summary>
    internal static Shape InternalApplyModifier(in Shape S,
                                                       Modification M,
                                                       NCollection_DataMap<Shape, Shape> context,
                                                       Modifier MD,
                                                       ProgressRange theProgress = default,
                                                       ShapeBuild_ReShape? aReShape = null)
    {
        // protect against INTERNAL/EXTERNAL shapes
        Shape SF = S.Oriented(Forward);

        // Process COMPOUNDs separately in order to handle sharing in assemblies
        if (SF.ShapeType() == ShapeType.Compound)
        {
            bool locModified = false;
            Compound C = new();
            BrepBuilder B = new BrepBuilder();
            B.MakeCompound(ref C);

            SF.Location(new Placement());
            int aShapeCount = SF.ChildCount;
            using var aPS = new ProgressScope(theProgress, "Applying Modifier For Solids", aShapeCount);
            for (var it = new Iterator(SF); it.More() && aPS.More(); it.Next())
            {
                ProgressRange aRange = aPS.Next();

                Shape shape = it.Value();
                Shape aShapeNoLoc = it.Value();
                aShapeNoLoc.Location(new Placement());

                Shape res;
                if (context.Find(aShapeNoLoc, out res))
                {
                    res.Orientation(shape.Orientation());
                    res.Location(shape.Location(), false);
                }
                else
                {
                    res = InternalApplyModifier(shape, M, context, MD, aRange, aReShape);
                }

                locModified |= !res.IsSame(shape);

                B.Add(C, res);
            }

            if (!aPS.More())
            {
                // Was cancelled
                return S;
            }

            if (!locModified)
            {
                return S;
            }

            SF.Orientation(S.Orientation());
            C.Orientation(S.Orientation());

            context.Bind(SF, C);

            C.Location(S.Location(), false);

            aReShape?.Replace(S, C);

            return C;
        }

        using var aPS2 = new ProgressScope(theProgress, "Modify the Shape", 1);
        // Modify the shape
        MD.Init(SF);
        MD.Perform(M, aPS2.Next());

        if (!aPS2.More() || !MD.IsDone)
        {
            return S;
        }

        // OCCT.NET: the internal accessor; the result is frozen at the publication point (decision 973)
        Shape aResult = MD.InternalModifiedShape(SF);
        aResult.Orientation(S.Orientation());

        if (!SF.IsSame(aResult))
        {
            context.Bind(S.Located(new Placement()), aResult.Located(new Placement()));
        }

        if (aReShape is not null)
        {
            UpdateShapeBuild(SF, MD, aReShape);
        }

        return aResult;
    }

    /// <summary>A new shape without indirect surfaces (frozen).</summary>
    public static Shape DirectFaces(in Shape S)
    {
        // Create a modification description
        ShapeCustom_DirectModification DM = new ShapeCustom_DirectModification();
        var context = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Modifier MD = new Modifier();
        return ApplyModifier(S, DM, context, MD);
    }

    /// <summary>A new shape, the original scaled about the origin (frozen).</summary>
    public static Shape ScaleShape(in Shape S, double scale)
    {
        // Create a modification description
        Transform3D T = new Transform3D();
        T.SetScale(new Point3D(0, 0, 0), scale);
        ShapeCustom_TrsfModification TM = new ShapeCustom_TrsfModification(T);
        var context = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Modifier MD = new Modifier();
        return ShapeCustom.ApplyModifier(S, TM, context, MD);
    }

    /// <summary>
    /// A new shape (frozen) whose B-spline and Bezier surfaces, curves and pcurves, and those based on them, are
    /// approximated with a degree up to <paramref name="MaxDegree"/> or a number of spans up to
    /// <paramref name="MaxNbSegment"/>, as the priority <paramref name="Degree"/> says. <paramref name="Rational"/>
    /// converts rational ones into polynomial B-splines; <paramref name="aParameters"/> says which geometry to convert.
    /// </summary>
    public static Shape BSplineRestriction(in Shape S,
                                                  double Tol3d,
                                                  double Tol2d,
                                                  int MaxDegree,
                                                  int MaxNbSegment,
                                                  Continuity Continuity3d,
                                                  Continuity Continuity2d,
                                                  bool Degree,
                                                  bool Rational,
                                                  ShapeCustom_RestrictionParameters aParameters)
    {
        // Create a modification description

        ShapeCustom_BSplineRestriction BSR = new ShapeCustom_BSplineRestriction();
        BSR.SetTol3d(Tol3d);
        BSR.SetTol2d(Tol2d);
        BSR.SetMaxDegree(MaxDegree);
        BSR.SetMaxNbSegments(MaxNbSegment);
        BSR.SetContinuity3d(Continuity3d);
        BSR.SetContinuity2d(Continuity2d);
        BSR.SetPriority(Degree);
        BSR.SetConvRational(Rational);
        BSR.SetRestrictionParameters(aParameters);
        // Modify the shape
        var context = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Modifier MD = new Modifier();
        return ShapeCustom.ApplyModifier(S, BSR, context, MD);
    }

    /// <summary>A new shape (frozen) with all elementary periodic surfaces converted to Geom_SurfaceOfRevolution.</summary>
    public static Shape ConvertToRevolution(in Shape S)
    {
        // Create a modification description
        ShapeCustom_ConvertToRevolution CRev = new ShapeCustom_ConvertToRevolution();
        var context = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Modifier MD = new Modifier();
        return ShapeCustom.ApplyModifier(S, CRev, context, MD);
    }

    /// <summary>
    /// A new shape (frozen) with all surfaces of revolution and linear extrusion converted to elementary periodic
    /// surfaces where they are such.
    /// </summary>
    public static Shape SweptToElementary(in Shape S)
    {
        // Create a modification description
        ShapeCustom_SweptToElementary SE = new ShapeCustom_SweptToElementary();
        var context = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Modifier MD = new Modifier();
        return ShapeCustom.ApplyModifier(S, SE, context, MD);
    }

    /// <summary>
    /// A new shape (frozen) with its surfaces of linear extrusion, revolution, offset surfaces and planes converted to
    /// Geom_BSplineSurface (with the same parametrisation), as the flags say.
    /// </summary>
    public static Shape ConvertToBSpline(in Shape S,
                                                bool extrMode,
                                                bool revolMode,
                                                bool offsetMode,
                                                bool planeMode = false)
    {
        // Create a modification description
        ShapeCustom_ConvertToBSpline BSRev = new ShapeCustom_ConvertToBSpline();
        BSRev.SetExtrusionMode(extrMode);
        BSRev.SetRevolutionMode(revolMode);
        BSRev.SetOffsetMode(offsetMode);
        BSRev.SetPlaneMode(planeMode);
        var context = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Modifier MD = new Modifier();
        return ShapeCustom.ApplyModifier(S, BSRev, context, MD);
    }
}
