// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_CartesianPointReducer.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_CartesianPointReducer.cxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_DirectionReducer.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_DirectionReducer.cxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_VectorReducer.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_VectorReducer.cxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_LineReducer.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_LineReducer.cxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_PlaneReducer.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_PlaneReducer.cxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_CircleReducer.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_CircleReducer.cxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_Axis2Placement3dReducer.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_Axis2Placement3dReducer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Tidy;

/// <summary>What the reducers' replacers share, as OCCT repeats it in each.</summary>
internal static class Replace
{
    public static bool InArray<TItem>(NCollection_HArray1<TItem> items, Transient oldEntity, TItem newEntity)
        where TItem : class
    {
        bool isReplaced = false;
        for (int i = items.Lower(); i <= items.Upper(); ++i)
        {
            if (ReferenceEquals(items.Value(i), oldEntity))
            {
                items.SetValue(i, newEntity);
                isReplaced = true;
            }
        }
        return isReplaced;
    }

    public static bool InArray<TItem>(NCollection_HArray2<TItem> items, Transient oldEntity, TItem newEntity)
        where TItem : class
    {
        bool isReplaced = false;
        for (int i = items.LowerRow(); i <= items.UpperRow(); ++i)
        {
            for (int j = items.LowerCol(); j <= items.UpperCol(); ++j)
            {
                if (ReferenceEquals(items.Value(i, j), oldEntity))
                {
                    items.SetValue(i, j, newEntity);
                    isReplaced = true;
                }
            }
        }
        return isReplaced;
    }

    public static bool InLayerAssignment(PresentationLayerAssignment sharing, RepresentationItem oldEntity, RepresentationItem newEntity)
    {
        NCollection_HArray1<LayeredItem> items = sharing.AssignedItems();
        bool isReplaced = false;
        for (int i = items.Lower(); i <= items.Upper(); ++i)
        {
            LayeredItem item = items.Value(i);
            if (ReferenceEquals(item.RepresentationItem(), oldEntity))
            {
                item.SetValue(newEntity);
                isReplaced = true;
            }
        }
        return isReplaced;
    }

    public static bool InStyledItem(StyledItem sharing, RepresentationItem oldEntity, RepresentationItem newEntity)
    {
        if (ReferenceEquals(sharing.Item(), oldEntity))
        {
            sharing.SetItem(newEntity);
            return true;
        }
        return false;
    }
}

/// <summary>Merges Cartesian points.</summary>
internal sealed class CartesianPointReducer : EntityReducer<StepCartesianPoint>
{
    public CartesianPointReducer(ExchangeWorkSession workSession) : base(workSession, CartesianPointHasher.Instance)
    {
        RegisterReplacer(typeof(StepAxis1Placement), static (o, n, s) => ReplaceLocation((StepAxis1Placement)s, o, n));
        RegisterReplacer(typeof(StepAxis2Placement3d), static (o, n, s) => ReplaceLocation((StepAxis2Placement3d)s, o, n));
        RegisterReplacer(typeof(StepVertexPoint), ReplaceVertexPoint);
        RegisterReplacer(typeof(StepGeometricCurveSet), ReplaceGeometricCurveSet);
        RegisterReplacer(typeof(PresentationLayerAssignment), static (o, n, s) => Replace.InLayerAssignment((PresentationLayerAssignment)s, o, n));
        RegisterReplacer(typeof(StyledItem), static (o, n, s) => Replace.InStyledItem((StyledItem)s, o, n));
        RegisterReplacer(typeof(StepBSplineCurveWithKnots), static (o, n, s) => Replace.InArray(((StepBSplineCurve)s).ControlPointsList(), o, n));
        RegisterReplacer(typeof(StepLine), ReplaceLine);
        RegisterReplacer(typeof(StepBSplineSurfaceWithKnots), static (o, n, s) => Replace.InArray(((StepBSplineSurface)s).ControlPointsList(), o, n));
        RegisterReplacer(typeof(StepBSplineCurveWithKnotsAndRationalBSplineCurve), static (o, n, s) => Replace.InArray(((StepBSplineCurve)s).ControlPointsList(), o, n));
        RegisterReplacer(typeof(StepBSplineSurfaceWithKnotsAndRationalBSplineSurface), static (o, n, s) => Replace.InArray(((StepBSplineSurface)s).ControlPointsList(), o, n));
        RegisterReplacer(typeof(StepRepresentation), static (o, n, s) => Replace.InArray<RepresentationItem>(((StepRepresentation)s).Items(), o, n));
    }

    private static bool ReplaceLocation(StepPlacement sharing, StepCartesianPoint oldEntity, StepCartesianPoint newEntity)
    {
        if (ReferenceEquals(sharing.Location(), oldEntity))
        {
            sharing.SetLocation(newEntity);
            return true;
        }
        return false;
    }

    private static bool ReplaceVertexPoint(StepCartesianPoint oldEntity, StepCartesianPoint newEntity, Transient sharing)
    {
        var vertex = (StepVertexPoint)sharing;
        if (ReferenceEquals(vertex.VertexGeometry(), oldEntity))
        {
            vertex.SetVertexGeometry(newEntity);
            return true;
        }
        return false;
    }

    private static bool ReplaceGeometricCurveSet(StepCartesianPoint oldEntity, StepCartesianPoint newEntity, Transient sharing)
    {
        NCollection_HArray1<StepGeometricSetSelect> elements = ((StepGeometricSet)sharing).Elements();
        bool isReplaced = false;
        for (int i = elements.Lower(); i <= elements.Upper(); ++i)
        {
            StepGeometricSetSelect element = elements.Value(i);
            if (ReferenceEquals(element.Point(), oldEntity))
            {
                element.SetValue(newEntity);
                isReplaced = true;
            }
        }
        return isReplaced;
    }

    private static bool ReplaceLine(StepCartesianPoint oldEntity, StepCartesianPoint newEntity, Transient sharing)
    {
        var line = (StepLine)sharing;
        if (ReferenceEquals(line.Pnt(), oldEntity))
        {
            line.SetPnt(newEntity);
            return true;
        }
        return false;
    }
}

/// <summary>Merges directions.</summary>
internal sealed class DirectionReducer : EntityReducer<StepDirection>
{
    public DirectionReducer(ExchangeWorkSession workSession) : base(workSession, DirectionHasher.Instance)
    {
        RegisterReplacer(typeof(StepAxis1Placement), ReplaceAxis1Placement);
        RegisterReplacer(typeof(StepAxis2Placement3d), ReplaceAxis2Placement3d);
        RegisterReplacer(typeof(StepVector), ReplaceVector);
    }

    private static bool ReplaceAxis1Placement(StepDirection oldEntity, StepDirection newEntity, Transient sharing)
    {
        var placement = (StepAxis1Placement)sharing;
        if (ReferenceEquals(placement.Axis(), oldEntity))
        {
            placement.SetAxis(newEntity);
            return true;
        }
        return false;
    }

    // OCCT's returns after the axis: a placement whose axis and reference direction are the same entity keeps the
    // second, so the duplicate stays referred to, and AddWithRefs brings it back. Replacing both is what's meant.
    private static bool ReplaceAxis2Placement3d(StepDirection oldEntity, StepDirection newEntity, Transient sharing)
    {
        var placement = (StepAxis2Placement3d)sharing;
        bool isReplaced = false;
        if (ReferenceEquals(placement.Axis(), oldEntity))
        {
            placement.SetAxis(newEntity);
            isReplaced = true;
        }
        if (ReferenceEquals(placement.RefDirection(), oldEntity))
        {
            placement.SetRefDirection(newEntity);
            isReplaced = true;
        }
        return isReplaced;
    }

    private static bool ReplaceVector(StepDirection oldEntity, StepDirection newEntity, Transient sharing)
    {
        var vector = (StepVector)sharing;
        if (ReferenceEquals(vector.Orientation(), oldEntity))
        {
            vector.SetOrientation(newEntity);
            return true;
        }
        return false;
    }
}

/// <summary>Merges vectors.</summary>
internal sealed class VectorReducer : EntityReducer<StepVector>
{
    public VectorReducer(ExchangeWorkSession workSession) : base(workSession, VectorHasher.Instance)
        => RegisterReplacer(typeof(StepLine), ReplaceLine);

    private static bool ReplaceLine(StepVector oldEntity, StepVector newEntity, Transient sharing)
    {
        var line = (StepLine)sharing;
        if (ReferenceEquals(line.Dir(), oldEntity))
        {
            line.SetDir(newEntity);
            return true;
        }
        return false;
    }
}

/// <summary>What the reducers of curves share: the entities that hold a curve.</summary>
internal static class ReplaceCurve
{
    public static bool InEdgeCurve(StepCurve oldEntity, StepCurve newEntity, Transient sharing)
    {
        var edge = (StepEdgeCurve)sharing;
        if (ReferenceEquals(edge.EdgeGeometry(), oldEntity))
        {
            edge.SetEdgeGeometry(newEntity);
            return true;
        }
        return false;
    }

    public static bool InSurfaceCurve(StepCurve oldEntity, StepCurve newEntity, Transient sharing)
    {
        var curve = (StepSurfaceCurve)sharing;
        if (ReferenceEquals(curve.Curve3d(), oldEntity))
        {
            curve.SetCurve3d(newEntity);
            return true;
        }
        return false;
    }
}

/// <summary>Merges lines.</summary>
internal sealed class LineReducer : EntityReducer<StepLine>
{
    public LineReducer(ExchangeWorkSession workSession) : base(workSession, LineHasher.Instance)
    {
        RegisterReplacer(typeof(StepEdgeCurve), ReplaceCurve.InEdgeCurve);
        RegisterReplacer(typeof(StepTrimmedCurve), ReplaceTrimmedCurve);
        RegisterReplacer(typeof(StepSurfaceCurve), ReplaceCurve.InSurfaceCurve);
        RegisterReplacer(typeof(DefinitionalRepresentation), static (o, n, s) => Replace.InArray<RepresentationItem>(((StepRepresentation)s).Items(), o, n));
        RegisterReplacer(typeof(StepSeamCurve), ReplaceCurve.InSurfaceCurve);
    }

    private static bool ReplaceTrimmedCurve(StepLine oldEntity, StepLine newEntity, Transient sharing)
    {
        var curve = (StepTrimmedCurve)sharing;
        if (ReferenceEquals(curve.BasisCurve(), oldEntity))
        {
            curve.SetBasisCurve(newEntity);
            return true;
        }
        return false;
    }
}

/// <summary>Merges planes.</summary>
internal sealed class PlaneReducer : EntityReducer<StepPlane>
{
    public PlaneReducer(ExchangeWorkSession workSession) : base(workSession, PlaneHasher.Instance)
    {
        RegisterReplacer(typeof(StepAdvancedFace), ReplaceAdvancedFace);
        RegisterReplacer(typeof(StepPcurve), ReplacePcurve);
    }

    private static bool ReplaceAdvancedFace(StepPlane oldEntity, StepPlane newEntity, Transient sharing)
    {
        var face = (StepAdvancedFace)sharing;
        if (ReferenceEquals(face.FaceGeometry(), oldEntity))
        {
            face.SetFaceGeometry(newEntity);
            return true;
        }
        return false;
    }

    private static bool ReplacePcurve(StepPlane oldEntity, StepPlane newEntity, Transient sharing)
    {
        var curve = (StepPcurve)sharing;
        if (ReferenceEquals(curve.BasisSurface(), oldEntity))
        {
            curve.SetBasisSurface(newEntity);
            return true;
        }
        return false;
    }
}

/// <summary>Merges circles.</summary>
internal sealed class CircleReducer : EntityReducer<StepCircle>
{
    public CircleReducer(ExchangeWorkSession workSession) : base(workSession, CircleHasher.Instance)
    {
        RegisterReplacer(typeof(StepEdgeCurve), ReplaceCurve.InEdgeCurve);
        RegisterReplacer(typeof(StepSurfaceCurve), ReplaceCurve.InSurfaceCurve);
        RegisterReplacer(typeof(StepSeamCurve), ReplaceCurve.InSurfaceCurve);
    }
}

/// <summary>Merges placements in 3D.</summary>
internal sealed class Axis2Placement3dReducer : EntityReducer<StepAxis2Placement3d>
{
    public Axis2Placement3dReducer(ExchangeWorkSession workSession) : base(workSession, Axis2Placement3dHasher.Instance)
    {
        RegisterReplacer(typeof(StepPlane), ReplaceElementarySurface);
        RegisterReplacer(typeof(ItemDefinedTransformation), ReplaceItemDefinedTransformation);
        RegisterReplacer(typeof(StepCylindricalSurface), ReplaceElementarySurface);
        RegisterReplacer(typeof(StepShapeRepresentation), ReplaceInItems);
        RegisterReplacer(typeof(ConstructiveGeometryRepresentation), ReplaceInItems);
        RegisterReplacer(typeof(StepCircle), ReplaceConic);
        RegisterReplacer(typeof(PresentationLayerAssignment), static (o, n, s) => Replace.InLayerAssignment((PresentationLayerAssignment)s, o, n));
        RegisterReplacer(typeof(StyledItem), static (o, n, s) => Replace.InStyledItem((StyledItem)s, o, n));
        RegisterReplacer(typeof(StepEllipse), ReplaceConic);
        RegisterReplacer(typeof(StepConicalSurface), ReplaceElementarySurface);
        RegisterReplacer(typeof(StepToroidalSurface), ReplaceElementarySurface);
        RegisterReplacer(typeof(StepAdvancedBrepShapeRepresentation), ReplaceInItems);
        RegisterReplacer(typeof(StepSphericalSurface), ReplaceElementarySurface);
    }

    private static bool ReplaceElementarySurface(StepAxis2Placement3d oldEntity, StepAxis2Placement3d newEntity, Transient sharing)
    {
        var surface = (StepElementarySurface)sharing;
        if (ReferenceEquals(surface.Position(), oldEntity))
        {
            surface.SetPosition(newEntity);
            return true;
        }
        return false;
    }

    private static bool ReplaceItemDefinedTransformation(StepAxis2Placement3d oldEntity, StepAxis2Placement3d newEntity, Transient sharing)
    {
        var transformation = (ItemDefinedTransformation)sharing;
        bool isReplaced = false;
        if (ReferenceEquals(transformation.TransformItem1(), oldEntity))
        {
            transformation.SetTransformItem1(newEntity);
            isReplaced = true;
        }
        if (ReferenceEquals(transformation.TransformItem2(), oldEntity))
        {
            transformation.SetTransformItem2(newEntity);
            isReplaced = true;
        }
        return isReplaced;
    }

    private static bool ReplaceInItems(StepAxis2Placement3d oldEntity, StepAxis2Placement3d newEntity, Transient sharing)
        => Replace.InArray<RepresentationItem>(((StepRepresentation)sharing).Items(), oldEntity, newEntity);

    // OCCT's sets the value on a copy of the conic's position, so the conic keeps its placement, and the duplicate comes
    // back with it. Here the position is the conic's own, so the conic takes the new one (occt-issues.md, unit 11N).
    private static bool ReplaceConic(StepAxis2Placement3d oldEntity, StepAxis2Placement3d newEntity, Transient sharing)
    {
        StepAxis2Placement position = ((StepConic)sharing).Position();
        if (ReferenceEquals(position.Axis2Placement3d(), oldEntity))
        {
            position.SetValue(newEntity);
            return true;
        }
        return false;
    }
}
