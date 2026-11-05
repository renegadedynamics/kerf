// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Custom;

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
internal static class Customize
{
    // Updates the reshaper by the info of the given shape.
    private static bool UpdateHistoryShape(in Shape shape,
                                           Modifier modifier,
                                           ReShapeBuilder reShape)
    {
        Shape result;
        try
        {
            // OCCT.NET: the internal accessor, so the shapes recorded aren't frozen before the whole result is built
            // (decision 973)
            result = modifier.InternalModifiedShape(shape);
        }
        catch (Exception caught) when (Refusal.Of(caught) is NoSuchObjectException)
        {
            // the sub shape isn't in the map
            result = new Shape();
        }

        if (!result.IsNull && !shape.IsSame(result))
        {
            reShape.Replace(shape, result);
            return true;
        }

        return false;
    }

    // Recursively updates the reshaper to add information of all sub-shapes.
    private static void UpdateHistory(in Shape shape,
                                      Modifier modifier,
                                      ReShapeBuilder reShape)
    {
        for (var theIterator = new Iterator(shape, false); theIterator.More(); theIterator.Next())
        {
            Shape current = theIterator.Value();
            if (UpdateHistoryShape(current, modifier, reShape))
            {
                UpdateHistory(current, modifier, reShape);
            }
        }
    }

    // Recursively updates the reshaper to add information of all sub-shapes.
    private static void UpdateShapeBuild(in Shape shape,
                                         Modifier modifier,
                                         ReShapeBuilder reShape)
    {
        UpdateHistoryShape(shape, modifier, reShape);
        UpdateHistory(shape, modifier, reShape);
    }

    /// <summary>
    /// Applies the modification to the shape with the modifier <paramref name="md"/>, sharing the results of
    /// compounds' children that are the same shape (assemblies) through <paramref name="context"/>, and records the
    /// replacements in <paramref name="aReShape"/> if given. The result is frozen (a publication point).
    /// </summary>
    /// <remarks>
    /// The modifier keeps its maps from one call to the next, as OCCT's: a modifier reused for a shape that shares
    /// sub-shapes with an earlier call's result would add pcurves to that result's edges, which are frozen, and throws
    /// <see cref="ImmutableObjectException"/> (decision 973). Use a new modifier for each call.
    /// </remarks>
    public static Shape ApplyModifier(in Shape s,
                                             Modification m,
                                             Dictionary<Shape, Shape> context,
                                             Modifier md,
                                             ProgressRange progress = default,
                                             ReShapeBuilder? aReShape = null)
    {
        Shape result = InternalApplyModifier(s, m, context, md, progress, aReShape);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary>
    /// <see cref="ApplyModifier"/> without freezing the result, for ported code that goes on building on it (decision
    /// 8101). The shapes it returns, binds in the context and records in the reshaper are frozen once the caller
    /// freezes the result.
    /// </summary>
    internal static Shape InternalApplyModifier(in Shape s,
                                                       Modification m,
                                                       Dictionary<Shape, Shape> context,
                                                       Modifier md,
                                                       ProgressRange progress = default,
                                                       ReShapeBuilder? aReShape = null)
    {
        // protect against INTERNAL/EXTERNAL shapes
        Shape sf = s.Oriented(Forward);

        // Process COMPOUNDs separately in order to handle sharing in assemblies
        if (sf.Type == ShapeType.Compound)
        {
            bool locModified = false;
            Compound c = new();
            BrepBuilder b = new BrepBuilder();
            b.MakeCompound(ref c);

            sf.Location(new Placement());
            int shapeCount = sf.ChildCount;
            using var ps = new ProgressScope(progress, "Applying Modifier For Solids", shapeCount);
            for (var it = new Iterator(sf); it.More() && ps.More(); it.Next())
            {
                ProgressRange range = ps.Next();

                Shape shape = it.Value();
                Shape shapeNoLoc = it.Value();
                shapeNoLoc.Location(new Placement());

                Shape res;
                if (context.TryGetValue(shapeNoLoc, out res))
                {
                    res.Orient(shape.Orientation);
                    res.Location(shape.Placement, false);
                }
                else
                {
                    res = InternalApplyModifier(shape, m, context, md, range, aReShape);
                }

                locModified |= !res.IsSame(shape);

                b.Add(c, res);
            }

            if (!ps.More())
            {
                // Was cancelled
                return s;
            }

            if (!locModified)
            {
                return s;
            }

            sf.Orient(s.Orientation);
            c.Orient(s.Orientation);

            context[sf] = c;

            c.Location(s.Placement, false);

            aReShape?.Replace(s, c);

            return c;
        }

        using var ps2 = new ProgressScope(progress, "Modify the Shape", 1);
        // Modify the shape
        md.Init(sf);
        md.Perform(m, ps2.Next());

        if (!ps2.More() || !md.IsDone)
        {
            return s;
        }

        // OCCT.NET: the internal accessor; the result is frozen at the publication point (decision 973)
        Shape result = md.InternalModifiedShape(sf);
        result.Orient(s.Orientation);

        if (!sf.IsSame(result))
        {
            context[s.Located(new Placement())] = result.Located(new Placement());
        }

        if (aReShape is not null)
        {
            UpdateShapeBuild(sf, md, aReShape);
        }

        return result;
    }

    /// <summary>A new shape without indirect surfaces (frozen).</summary>
    public static Shape DirectFaces(in Shape s)
    {
        // Create a modification description
        DirectModification dm = new DirectModification();
        var context = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Modifier md = new Modifier();
        return ApplyModifier(s, dm, context, md);
    }

    /// <summary>A new shape, the original scaled about the origin (frozen).</summary>
    public static Shape ScaleShape(in Shape s, double scale)
    {
        // Create a modification description
        Transform3D t = new Transform3D().WithScale(new Point3D(0, 0, 0), scale);
        CustomTrsfModification tm = new CustomTrsfModification(t);
        var context = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Modifier md = new Modifier();
        return Customize.ApplyModifier(s, tm, context, md);
    }

    /// <summary>
    /// A new shape (frozen) whose B-spline and Bezier surfaces, curves and pcurves, and those based on them, are
    /// approximated with a degree up to <paramref name="maxDegree"/> or a number of spans up to
    /// <paramref name="maxNbSegment"/>, as the priority <paramref name="degree"/> says. <paramref name="rational"/>
    /// converts rational ones into polynomial B-splines; <paramref name="aParameters"/> says which geometry to convert.
    /// </summary>
    public static Shape BSplineRestriction(in Shape s,
                                                  double tol3d,
                                                  double tol2d,
                                                  int maxDegree,
                                                  int maxNbSegment,
                                                  Continuity continuity3d,
                                                  Continuity continuity2d,
                                                  bool degree,
                                                  bool rational,
                                                  RestrictionParameters aParameters)
    {
        // Create a modification description

        BSplineRestriction bsr = new BSplineRestriction();
        bsr.SetTol3d(tol3d);
        bsr.SetTol2d(tol2d);
        bsr.SetMaxDegree(maxDegree);
        bsr.SetMaxNbSegments(maxNbSegment);
        bsr.SetContinuity3d(continuity3d);
        bsr.SetContinuity2d(continuity2d);
        bsr.SetPriority(degree);
        bsr.SetConvRational(rational);
        bsr.SetRestrictionParameters(aParameters);
        // Modify the shape
        var context = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Modifier md = new Modifier();
        return Customize.ApplyModifier(s, bsr, context, md);
    }

    /// <summary>A new shape (frozen) with all elementary periodic surfaces converted to Geom_SurfaceOfRevolution.</summary>
    public static Shape ConvertToRevolution(in Shape s)
    {
        // Create a modification description
        ConvertToRevolution cRev = new ConvertToRevolution();
        var context = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Modifier md = new Modifier();
        return Customize.ApplyModifier(s, cRev, context, md);
    }

    /// <summary>
    /// A new shape (frozen) with all surfaces of revolution and linear extrusion converted to elementary periodic
    /// surfaces where they are such.
    /// </summary>
    public static Shape SweptToElementary(in Shape s)
    {
        // Create a modification description
        SweptToElementary se = new SweptToElementary();
        var context = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Modifier md = new Modifier();
        return Customize.ApplyModifier(s, se, context, md);
    }

    /// <summary>
    /// A new shape (frozen) with its surfaces of linear extrusion, revolution, offset surfaces and planes converted to
    /// Geom_BSplineSurface (with the same parametrisation), as the flags say.
    /// </summary>
    public static Shape ConvertToBSpline(in Shape s,
                                                bool extrMode,
                                                bool revolMode,
                                                bool offsetMode,
                                                bool planeMode = false)
    {
        // Create a modification description
        ConvertToBSpline bsRev = new ConvertToBSpline();
        bsRev.SetExtrusionMode(extrMode);
        bsRev.SetRevolutionMode(revolMode);
        bsRev.SetOffsetMode(offsetMode);
        bsRev.SetPlaneMode(planeMode);
        var context = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Modifier md = new Modifier();
        return Customize.ApplyModifier(s, bsRev, context, md);
    }
}
