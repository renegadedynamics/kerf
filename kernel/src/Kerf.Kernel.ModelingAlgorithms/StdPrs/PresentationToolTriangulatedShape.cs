// Port of: src/Visualization/TKV3d/StdPrs/StdPrs_ToolTriangulatedShape.hxx
// Port of: src/Visualization/TKV3d/StdPrs/StdPrs_ToolTriangulatedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.HiddenLines.Presentation;

/// <summary>
/// Meshing a shape for display as OCCT's presentations do: the deflection from a <see cref="Drawer"/> (relative
/// to the shape's bounding box by default) and the mesh factory's algorithm, run only when the shape's mesh is
/// missing or coarser than that deflection.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's <c>Tessellate</c> writes the meshes into the shape. The port's meshing algorithm meshes a shape holding
/// frozen TShapes on a copy (decision 489), so
/// <see cref="Tessellate(in Shape, Drawer, out Shape, out History)"/> also returns the
/// meshed shape and the history from the input's sub-shapes to it.
/// </para>
/// <para>
/// Not ported (unit 4L2 ports what meshing for display needs): <c>IsClosed</c>, which the shaded presentation's
/// back-face culling reads, and <c>ClearOnOwnDeflectionChange</c>, which cleans the shape in place when an object's own
/// deviation changes.
/// </para>
/// </remarks>
internal static class PresentationToolTriangulatedShape
{
    /// <summary>True if every face of the shape has a triangulation.</summary>
    public static bool IsTriangulated(in Shape shape)
    {
        for (var faceIter = new Explorer(shape, ShapeType.Face); faceIter.More(); faceIter.Next())
        {
            Face face = ShapeCasts.Face(faceIter.Current());
            if (Tool.Triangulation(face, out _) is null)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// The absolute deflection to mesh the shape with: the drawer's maximal chordial deviation, or with a relative
    /// type of deflection the deviation coefficient applied to the shape's bounding box (without triangulations), which
    /// is then stored into the drawer as its maximal chordial deviation.
    /// </summary>
    public static double GetDeflection(in Shape shape, Drawer drawer)
    {
        if (drawer.TypeOfDeflection() != TypeOfDeflection.TodRelative)
        {
            return drawer.MaximalChordialDeviation();
        }

        var bndBox = new Box();
        Bounds.Add(shape, ref bndBox, false);
        if (bndBox.IsVoid)
        {
            return drawer.MaximalChordialDeviation();
        }
        else if (bndBox.IsOpen)
        {
            if (!bndBox.HasFinitePart)
            {
                return drawer.MaximalChordialDeviation();
            }

            bndBox = bndBox.FinitePart();
        }

        // store computed relative deflection of shape as absolute deviation coefficient in case relative type to use
        // it later on for sub-shapes
        double deflection = global::Kerf.Engine.HiddenLines.Presentation.Prs3d.GetDeflection(bndBox, drawer.DeviationCoefficient(), drawer.MaximalChordialDeviation());
        drawer.SetMaximalChordialDeviation(deflection);
        return deflection;
    }

    /// <summary>
    /// True if the shape is meshed finely enough for the drawer: every face triangulated within
    /// <see cref="GetDeflection"/>, with polygons on its edges, and every free edge's polygon within it.
    /// </summary>
    public static bool IsTessellated(in Shape shape, Drawer drawer)
        => BrepTools.Triangulation(shape, GetDeflection(shape, drawer), true);

    /// <summary>
    /// Meshes the shape with the drawer's deflection and deviation angle, through
    /// <see cref="DiscretFactory"/>'s default algorithm, unless it is meshed finely enough already. True if it
    /// was meshed. An unfrozen shape is meshed in place, as in OCCT; a shape holding frozen TShapes is meshed on a
    /// copy, which only the other overload returns.
    /// </summary>
    public static bool Tessellate(in Shape shape, Drawer drawer)
        => Tessellate(shape, drawer, out _, out _);

    /// <summary>
    /// <see cref="Tessellate(in Shape, Drawer)"/>, with the meshed shape (the input itself when it wasn't
    /// meshed, or was meshed in place) and the history from the input's sub-shapes to the meshed shape's (empty when the
    /// algorithm isn't a <see cref="IncrementalMesh"/>, or meshed in place).
    /// </summary>
    public static bool Tessellate(in Shape shape, Drawer drawer, out Shape meshed,
                                  out History history)
    {
        meshed = shape;
        history = new History();
        bool wasRecomputed = false;

        // Check if it is possible to avoid unnecessary recomputation of shape triangulation
        if (IsTessellated(shape, drawer))
        {
            return wasRecomputed;
        }

        double deflection = GetDeflection(shape, drawer);

        // retrieve meshing tool from Factory
        DiscretRoot? meshAlgo = DiscretFactory.Get().Discret(shape, deflection, drawer.DeviationAngle());
        if (meshAlgo is not null)
        {
            meshAlgo.Perform();
            wasRecomputed = true;

            // Kerf: the meshed copy (decision 489)
            meshed = meshAlgo.Shape();
            if (meshAlgo is IncrementalMesh incMesh)
            {
                history = incMesh.History();
            }
        }

        return wasRecomputed;
    }
}
