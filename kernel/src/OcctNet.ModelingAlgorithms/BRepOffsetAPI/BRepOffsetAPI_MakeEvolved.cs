// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeEvolved.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeEvolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Schematiq.Collections;
using static Kerf.Geometry.JoinKind;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds evolved shapes: the unlooped sweep of a profile (a wire) along a planar spine (a face or a wire).
/// </summary>
/// <remarks>
/// <para>
/// The profile is defined in a coordinate system R placed at each point of the spine: R is the global system if
/// theIsAxeProf is true; otherwise its origin is the point of the spine closest to the profile, its X axis the
/// spine's tangent there and its Z axis the normal to the spine's plane (<see cref="BRepFill.Axe"/>). Only the
/// GeomAbs_Arc join is implemented (as in OCCT). With theIsSolid the result is completed to a solid or a compound of
/// solids. With theIsProfOnSpine the profile must touch the spine.
/// </para>
/// <para>
/// The volume mode (theIsVolume) builds the evolved volume with <see cref="BRepFill_AdvancedEvolved"/> (the sweep
/// with its self-intersections removed by the Boolean operations); the spine of a face is its first wire.
/// </para>
/// <para>
/// A maker (decisions 414 and 454): <see cref="BRepBuilderAPI_MakeShape.Shape"/>, <see cref="Top"/>,
/// <see cref="Bottom"/> and <see cref="GeneratedShapes"/> hand out frozen shapes.
/// </para>
/// </remarks>
public class BRepOffsetAPI_MakeEvolved : BRepBuilderAPI_MakeShape
{
    private readonly BRepFill_Evolved myEvolved = new();
    private readonly BRepFill_AdvancedEvolved myVolume = new();
    private readonly bool myIsVolume;

    /// <summary>An empty maker.</summary>
    public BRepOffsetAPI_MakeEvolved()
    {
    }

    /// <summary>
    /// Builds the evolved shape of <paramref name="theProfile"/> swept along <paramref name="theSpine"/> (a wire or a
    /// face).
    /// </summary>
    /// <exception cref="TypeMismatchException">The spine is neither a wire nor a face.</exception>
    /// <exception cref="MethodNotImplementedException">A join other than GeomAbs_Arc (not in the volume mode).</exception>
    public BRepOffsetAPI_MakeEvolved(in TopoDS_Shape theSpine,
                                     in TopoDS_Wire theProfile,
                                     global::Kerf.Geometry.JoinKind theJoinType = Arc,
                                     bool theIsAxeProf = true,
                                     bool theIsSolid = false,
                                     bool theIsProfOnSpine = false,
                                     double theTol = 0.0000001,
                                     bool theIsVolume = false,
                                     bool theRunInParallel = false)
    {
        myIsVolume = theIsVolume;
        if (theSpine.ShapeType() != TopAbs_WIRE && theSpine.ShapeType() != TopAbs_FACE)
        {
            throw new TypeMismatchException("BRepOffsetAPI_MakeEvolved: face or wire is expected as a spine");
        }
        if (theIsVolume)
        {
            myVolume.SetParallelMode(theRunInParallel);
            TopoDS_Wire aSpine;
            if (theSpine.ShapeType() == TopAbs_WIRE)
            {
                aSpine = TopoDS.Wire(theSpine);
            }
            else
            {
                aSpine = TopoDS.Wire(new TopoDS_Iterator(theSpine).Value());
            }
            myVolume.Perform(aSpine, theProfile, theTol, theIsSolid);
            if (!myVolume.IsDone())
            {
                return;
            }
        }
        else
        {
            CoordinateSystem Axis = new(Constants.Origin(), Constants.DZ(), Constants.DX());

            if (!theIsAxeProf)
            {
                BRepFill.Axe(theSpine, theProfile, out Axis, out bool POS, Max(theTol, global::Kerf.Engine.Foundation.Precision.Confusion()));
                if (theIsProfOnSpine && !POS)
                {
                    return;
                }
            }
            if (theSpine.ShapeType() == TopAbs_WIRE)
            {
                myEvolved.Perform(TopoDS.Wire(theSpine), theProfile, Axis, theJoinType, theIsSolid);
            }
            else
            {
                myEvolved.Perform(TopoDS.Face(theSpine), theProfile, Axis, theJoinType, theIsSolid);
            }
        }

        Build();
    }

    /// <summary>The underlying algorithm.</summary>
    /// <exception cref="TypeMismatchException">In volume mode.</exception>
    public BRepFill_Evolved Evolved()
    {
        if (myIsVolume)
        {
            throw new TypeMismatchException("BRepOffsetAPI_MakeEvolved: myEvolved is accessed while in volume mode");
        }
        return myEvolved;
    }

    /// <summary>Takes the result of the algorithm.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        if (myEvolved.IsDone())
        {
            myShape = myEvolved.InternalShape();
        }
        else if (myVolume.IsDone())
        {
            myShape = myVolume.InternalShape();
        }

        Done();
    }

    /// <summary>
    /// The shapes generated from a sub-shape <paramref name="SpineShape"/> of the spine and a sub-shape
    /// <paramref name="ProfShape"/> of the profile (frozen, with the result).
    /// </summary>
    public ISchematiqList<TopoDS_Shape> GeneratedShapes(in TopoDS_Shape SpineShape, in TopoDS_Shape ProfShape)
    {
        if (!myEvolved.IsDone())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }

        return myEvolved.GeneratedShapes(SpineShape, ProfShape);
    }

    /// <summary>The top face if the result was built solid (frozen, with the result).</summary>
    public TopoDS_Shape Top() => myEvolved.Top();

    /// <summary>The bottom face if the result was built solid (frozen, with the result).</summary>
    public TopoDS_Shape Bottom() => myEvolved.Bottom();
}
