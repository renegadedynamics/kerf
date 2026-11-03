// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeEvolved.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeEvolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Geometry.JoinKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds evolved shapes: the unlooped sweep of a profile (a wire) along a planar spine (a face or a wire).
/// </summary>
/// <remarks>
/// <para>
/// The profile is defined in a coordinate system R placed at each point of the spine: R is the global system if
/// theIsAxeProf is true; otherwise its origin is the point of the spine closest to the profile, its X axis the
/// spine's tangent there and its Z axis the normal to the spine's plane (<see cref="Fill.Axe"/>). Only the
/// GeomAbs_Arc join is implemented (as in OCCT). With theIsSolid the result is completed to a solid or a compound of
/// solids. With theIsProfOnSpine the profile must touch the spine.
/// </para>
/// <para>
/// The volume mode (theIsVolume) builds the evolved volume with <see cref="AdvancedEvolved"/> (the sweep
/// with its self-intersections removed by the Boolean operations); the spine of a face is its first wire.
/// </para>
/// <para>
/// A maker (decisions 414 and 454): <see cref="MakeShape.Shape"/>, <see cref="Top"/>,
/// <see cref="Bottom"/> and <see cref="GeneratedShapes"/> hand out frozen shapes.
/// </para>
/// </remarks>
public class MakeEvolved : MakeShape
{
    private readonly Evolved _evolved = new();
    private readonly AdvancedEvolved _volume = new();
    private readonly bool _isVolume;

    /// <summary>An empty maker.</summary>
    public MakeEvolved()
    {
    }

    /// <summary>
    /// Builds the evolved shape of <paramref name="profile"/> swept along <paramref name="spine"/> (a wire or a
    /// face).
    /// </summary>
    /// <exception cref="TypeMismatchException">The spine is neither a wire nor a face.</exception>
    /// <exception cref="MethodNotImplementedException">A join other than GeomAbs_Arc (not in the volume mode).</exception>
    public MakeEvolved(in Shape spine,
                                     in Wire profile,
                                     global::Kerf.Geometry.JoinKind joinType = Arc,
                                     bool isAxeProf = true,
                                     bool isSolid = false,
                                     bool isProfOnSpine = false,
                                     double tol = 0.0000001,
                                     bool isVolume = false,
                                     bool runInParallel = false)
    {
        _isVolume = isVolume;
        if (spine.ShapeType() != ShapeType.Wire && spine.ShapeType() != ShapeType.Face)
        {
            throw new TypeMismatchException("BRepOffsetAPI_MakeEvolved: face or wire is expected as a spine");
        }
        if (isVolume)
        {
            _volume.SetParallelMode(runInParallel);
            Wire aSpine;
            if (spine.ShapeType() == ShapeType.Wire)
            {
                aSpine = ShapeCasts.Wire(spine);
            }
            else
            {
                aSpine = ShapeCasts.Wire(new Iterator(spine).Value());
            }
            _volume.Perform(aSpine, profile, tol, isSolid);
            if (!_volume.IsDone())
            {
                return;
            }
        }
        else
        {
            CoordinateSystem axis = new(Constants.Origin(), Constants.DZ(), Constants.DX());

            if (!isAxeProf)
            {
                Fill.Axe(spine, profile, out axis, out bool pos, Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion()));
                if (isProfOnSpine && !pos)
                {
                    return;
                }
            }
            if (spine.ShapeType() == ShapeType.Wire)
            {
                _evolved.Perform(ShapeCasts.Wire(spine), profile, axis, joinType, isSolid);
            }
            else
            {
                _evolved.Perform(ShapeCasts.Face(spine), profile, axis, joinType, isSolid);
            }
        }

        Build();
    }

    /// <summary>The underlying algorithm.</summary>
    /// <exception cref="TypeMismatchException">In volume mode.</exception>
    public Evolved Evolved()
    {
        if (_isVolume)
        {
            throw new TypeMismatchException("BRepOffsetAPI_MakeEvolved: myEvolved is accessed while in volume mode");
        }
        return _evolved;
    }

    /// <summary>Takes the result of the algorithm.</summary>
    public override void Build(ProgressRange range = default)
    {
        if (_evolved.IsDone)
        {
            _shape = _evolved.InternalShape();
        }
        else if (_volume.IsDone())
        {
            _shape = _volume.InternalShape();
        }

        Done();
    }

    /// <summary>
    /// The shapes generated from a sub-shape <paramref name="spineShape"/> of the spine and a sub-shape
    /// <paramref name="profShape"/> of the profile (frozen, with the result).
    /// </summary>
    public ISchematiqList<Shape> GeneratedShapes(in Shape spineShape, in Shape profShape)
    {
        if (!_evolved.IsDone)
        {
            return PackedList<Shape>.Empty;
        }

        return _evolved.GeneratedShapes(spineShape, profShape);
    }

    /// <summary>The top face if the result was built solid (frozen, with the result).</summary>
    public Shape Top() => _evolved.Top();

    /// <summary>The bottom face if the result was built solid (frozen, with the result).</summary>
    public Shape Bottom() => _evolved.Bottom();
}
