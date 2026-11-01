// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Converts the geometry of shapes: B-spline and Bezier surfaces to analytical ones, indirect elementary surfaces
/// (left-handed frames) to direct ones, elementary surfaces to surfaces of revolution, extrusions, revolutions,
/// offsets and planes to B-splines, the degree and number of spans of B-splines, and scales shapes. Each function
/// applies a <see cref="BRepTools_Modification"/> with <see cref="BRepTools_Modifier"/>.
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
    private static bool UpdateHistoryShape(in TopoDS_Shape theShape,
                                           BRepTools_Modifier theModifier,
                                           ShapeBuild_ReShape theReShape)
    {
        TopoDS_Shape aResult;
        try
        {
            // OCCT.NET: the internal accessor, so the shapes recorded aren't frozen before the whole result is built
            // (decision 973)
            aResult = theModifier.InternalModifiedShape(theShape);
        }
        catch (Standard_NoSuchObject)
        {
            // the sub shape isn't in the map
            aResult = new TopoDS_Shape();
        }

        if (!aResult.IsNull() && !theShape.IsSame(aResult))
        {
            theReShape.Replace(theShape, aResult);
            return true;
        }

        return false;
    }

    // Recursively updates the reshaper to add information of all sub-shapes.
    private static void UpdateHistory(in TopoDS_Shape theShape,
                                      BRepTools_Modifier theModifier,
                                      ShapeBuild_ReShape theReShape)
    {
        for (var theIterator = new TopoDS_Iterator(theShape, false); theIterator.More(); theIterator.Next())
        {
            TopoDS_Shape aCurrent = theIterator.Value();
            if (UpdateHistoryShape(aCurrent, theModifier, theReShape))
            {
                UpdateHistory(aCurrent, theModifier, theReShape);
            }
        }
    }

    // Recursively updates the reshaper to add information of all sub-shapes.
    private static void UpdateShapeBuild(in TopoDS_Shape theShape,
                                         BRepTools_Modifier theModifier,
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
    /// <see cref="TopoDS_LockedShape"/> (decision 973). Use a new modifier for each call.
    /// </remarks>
    public static TopoDS_Shape ApplyModifier(in TopoDS_Shape S,
                                             BRepTools_Modification M,
                                             NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> context,
                                             BRepTools_Modifier MD,
                                             Message_ProgressRange theProgress = default,
                                             ShapeBuild_ReShape? aReShape = null)
    {
        TopoDS_Shape aResult = InternalApplyModifier(S, M, context, MD, theProgress, aReShape);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// <see cref="ApplyModifier"/> without freezing the result, for ported code that goes on building on it (decision
    /// 8101). The shapes it returns, binds in the context and records in the reshaper are frozen once the caller
    /// freezes the result.
    /// </summary>
    internal static TopoDS_Shape InternalApplyModifier(in TopoDS_Shape S,
                                                       BRepTools_Modification M,
                                                       NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> context,
                                                       BRepTools_Modifier MD,
                                                       Message_ProgressRange theProgress = default,
                                                       ShapeBuild_ReShape? aReShape = null)
    {
        // protect against INTERNAL/EXTERNAL shapes
        TopoDS_Shape SF = S.Oriented(TopAbs_FORWARD);

        // Process COMPOUNDs separately in order to handle sharing in assemblies
        if (SF.ShapeType() == TopAbs_COMPOUND)
        {
            bool locModified = false;
            TopoDS_Compound C = new();
            BRep_Builder B = new BRep_Builder();
            B.MakeCompound(ref C);

            SF.Location(new TopLoc_Location());
            int aShapeCount = SF.NbChildren();
            using var aPS = new Message_ProgressScope(theProgress, "Applying Modifier For Solids", aShapeCount);
            for (var it = new TopoDS_Iterator(SF); it.More() && aPS.More(); it.Next())
            {
                Message_ProgressRange aRange = aPS.Next();

                TopoDS_Shape shape = it.Value();
                TopoDS_Shape aShapeNoLoc = it.Value();
                aShapeNoLoc.Location(new TopLoc_Location());

                TopoDS_Shape res;
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

        using var aPS2 = new Message_ProgressScope(theProgress, "Modify the Shape", 1);
        // Modify the shape
        MD.Init(SF);
        MD.Perform(M, aPS2.Next());

        if (!aPS2.More() || !MD.IsDone())
        {
            return S;
        }

        // OCCT.NET: the internal accessor; the result is frozen at the publication point (decision 973)
        TopoDS_Shape aResult = MD.InternalModifiedShape(SF);
        aResult.Orientation(S.Orientation());

        if (!SF.IsSame(aResult))
        {
            context.Bind(S.Located(new TopLoc_Location()), aResult.Located(new TopLoc_Location()));
        }

        if (aReShape is not null)
        {
            UpdateShapeBuild(SF, MD, aReShape);
        }

        return aResult;
    }

    /// <summary>A new shape without indirect surfaces (frozen).</summary>
    public static TopoDS_Shape DirectFaces(in TopoDS_Shape S)
    {
        // Create a modification description
        ShapeCustom_DirectModification DM = new ShapeCustom_DirectModification();
        var context = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRepTools_Modifier MD = new BRepTools_Modifier();
        return ApplyModifier(S, DM, context, MD);
    }

    /// <summary>A new shape, the original scaled about the origin (frozen).</summary>
    public static TopoDS_Shape ScaleShape(in TopoDS_Shape S, double scale)
    {
        // Create a modification description
        gp_Trsf T = new gp_Trsf();
        T.SetScale(new gp_Pnt(0, 0, 0), scale);
        ShapeCustom_TrsfModification TM = new ShapeCustom_TrsfModification(T);
        var context = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRepTools_Modifier MD = new BRepTools_Modifier();
        return ShapeCustom.ApplyModifier(S, TM, context, MD);
    }

    /// <summary>
    /// A new shape (frozen) whose B-spline and Bezier surfaces, curves and pcurves, and those based on them, are
    /// approximated with a degree up to <paramref name="MaxDegree"/> or a number of spans up to
    /// <paramref name="MaxNbSegment"/>, as the priority <paramref name="Degree"/> says. <paramref name="Rational"/>
    /// converts rational ones into polynomial B-splines; <paramref name="aParameters"/> says which geometry to convert.
    /// </summary>
    public static TopoDS_Shape BSplineRestriction(in TopoDS_Shape S,
                                                  double Tol3d,
                                                  double Tol2d,
                                                  int MaxDegree,
                                                  int MaxNbSegment,
                                                  GeomAbs_Shape Continuity3d,
                                                  GeomAbs_Shape Continuity2d,
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
        var context = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRepTools_Modifier MD = new BRepTools_Modifier();
        return ShapeCustom.ApplyModifier(S, BSR, context, MD);
    }

    /// <summary>A new shape (frozen) with all elementary periodic surfaces converted to Geom_SurfaceOfRevolution.</summary>
    public static TopoDS_Shape ConvertToRevolution(in TopoDS_Shape S)
    {
        // Create a modification description
        ShapeCustom_ConvertToRevolution CRev = new ShapeCustom_ConvertToRevolution();
        var context = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRepTools_Modifier MD = new BRepTools_Modifier();
        return ShapeCustom.ApplyModifier(S, CRev, context, MD);
    }

    /// <summary>
    /// A new shape (frozen) with all surfaces of revolution and linear extrusion converted to elementary periodic
    /// surfaces where they are such.
    /// </summary>
    public static TopoDS_Shape SweptToElementary(in TopoDS_Shape S)
    {
        // Create a modification description
        ShapeCustom_SweptToElementary SE = new ShapeCustom_SweptToElementary();
        var context = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRepTools_Modifier MD = new BRepTools_Modifier();
        return ShapeCustom.ApplyModifier(S, SE, context, MD);
    }

    /// <summary>
    /// A new shape (frozen) with its surfaces of linear extrusion, revolution, offset surfaces and planes converted to
    /// Geom_BSplineSurface (with the same parametrisation), as the flags say.
    /// </summary>
    public static TopoDS_Shape ConvertToBSpline(in TopoDS_Shape S,
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
        var context = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRepTools_Modifier MD = new BRepTools_Modifier();
        return ShapeCustom.ApplyModifier(S, BSRev, context, MD);
    }
}
