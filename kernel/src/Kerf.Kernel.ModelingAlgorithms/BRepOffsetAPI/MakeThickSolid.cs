// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeThickSolid.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeThickSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Offsets.OffsetMode;
using static Kerf.Geometry.JoinKind;

namespace Kerf.Engine.Offsets.Api;

/// <summary>
/// Builds a thick solid (a hollowed solid, or a thickened shell): <see cref="MakeThickSolidByJoin"/> removes the closing
/// faces from the shape and offsets the rest, closing the gap with the shape itself; <see cref="MakeThickSolidBySimple"/>
/// offsets a shell with the simple algorithm and closes it with walls.
/// </summary>
/// <remarks>
/// <para>
/// A positive offset thickens outward, a negative one inward. The closing faces must belong to the shape. The surfaces
/// must be at least C1, and the offset small enough to avoid self-intersections.
/// </para>
/// <para>
/// A maker (decision 414), as <see cref="MakeOffsetShape"/>: the caller's shape is never written (decisions 1005
/// and 16201), and the result holds the caller's faces where the algorithm left them unchanged.
/// </para>
/// </remarks>
public class MakeThickSolid : MakeOffsetShape
{
    //=================================================================================================

    /// <summary>An empty algorithm.</summary>
    public MakeThickSolid()
    {
        // Build only solids.
        _simpleOffsetShape.SetBuildSolidFlag(true);
    }

    //=================================================================================================

    /// <summary>
    /// Makes a thick solid from <paramref name="s"/>: the faces <paramref name="closingFaces"/> are removed and the rest
    /// offset by <paramref name="offset"/>, the gap closed by the shape's own faces. The other arguments are those of
    /// <see cref="MakeOffsetShape.PerformByJoin"/>.
    /// </summary>
    public void MakeThickSolidByJoin(in Shape s, NCollection_List<Shape> closingFaces, double offset,
                                     double tol, OffsetMode mode = OffsetMode.Skin, bool intersection = false,
                                     bool selfInter = false, JoinKind Join = Arc,
                                     bool removeIntEdges = false, ProgressRange range = default)
    {
        NotDone();
        _lastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_JOIN;

        _offsetShape.Initialize(s, offset, tol, mode, intersection, selfInter, Join, false, removeIntEdges);
        NCollection_List<Shape>.Iterator it = new(closingFaces);
        for (; it.More(); it.Next())
        {
            _offsetShape.AddFace(ShapeCasts.Face(it.Value()));
        }

        _offsetShape.MakeThickSolid(range);
        if (!_offsetShape.IsDone)
        {
            return;
        }

        _shape = _offsetShape.InternalShape();
        Done();
    }

    //=================================================================================================

    /// <summary>Makes a thick solid with the simple algorithm (the shell offset, closed by walls).</summary>
    public void MakeThickSolidBySimple(in Shape s, double offsetValue)
    {
        NotDone();
        _lastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_SIMPLE;

        _simpleOffsetShape.Initialize(s, offsetValue);
        _simpleOffsetShape.Perform();

        if (!_simpleOffsetShape.IsDone)
        {
            return;
        }

        _shape = _simpleOffsetShape.InternalGetResultShape();
        Done();
    }

    //=================================================================================================

    /// <summary>Does nothing.</summary>
    public override void Build(ProgressRange range = default) { }

    //=================================================================================================

    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        _generated.Clear();

        // the face the run worked on: the caller's, or its copy in a caller's unfrozen shape (unit 10H5)
        Shape aF = _offsetShape.InternalCopied(f);
        if (_lastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_JOIN && _offsetShape.InternalOffsetFacesFromShapes().HasImage(aF))
        {
            if (_offsetShape.InternalClosingFaces().Contains(aF))
            {
                _offsetShape.InternalOffsetFacesFromShapes().LastImage(aF, _generated);

                // Reverse generated shapes in case of small solids.
                // Useful only for faces without influence on others.
                NCollection_List<Shape>.Iterator it = new(_generated);
                for (; it.More(); it.Next())
                {
                    it.ChangeValue().Reverse();
                    it.ChangeValue() = _offsetShape.InternalFinal(it.Value());
                }
            }
        }
        else if (_lastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_SIMPLE)
        {
            Shape modShape = _simpleOffsetShape.InternalModified(f);
            if (!modShape.IsNull)
            {
                _generated.Append(modShape);
            }
        }

        return _generated;
    }
}
