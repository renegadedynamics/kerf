// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_Parameters.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Kerf.Engine.Meshing.Tools;

/// <summary>The meshing parameters.</summary>
/// <remarks>
/// <para>
/// OCCT's struct with OCCT's public fields (decision 489): a caller writes
/// <c>new IMeshTools_Parameters { Deflection = 0.1, Angle = 0.5, InParallel = true }</c>. It is a value, so changing a
/// copy changes nothing shared, and it is equatable field by field (doubles compared exactly, as <c>double.Equals</c>
/// does), so <c>(shape, parameters)</c> can key a cache of meshes.
/// </para>
/// <para>
/// <c>new IMeshTools_Parameters()</c> has OCCT's defaults. <c>default(IMeshTools_Parameters)</c> doesn't: every field
/// is zero or false and the algorithm is Watson. Ported code writes <c>new IMeshTools_Parameters()</c>.
/// </para>
/// </remarks>
internal struct ToolParameters : IEquatable<ToolParameters>
{
    /// <summary>OCCT's defaults.</summary>
    public ToolParameters()
    {
        MeshAlgo = MeshAlgoType.MeshAlgoType_DEFAULT;
        Angle = 0.5;
        Deflection = 0.001;
        AngleInterior = -1.0;
        DeflectionInterior = -1.0;
        MinSize = -1.0;
        InParallel = false;
        Relative = false;
        InternalVerticesMode = true;
        ControlSurfaceDeflection = true;
        EnableControlSurfaceDeflectionAllSurfaces = false;
        CleanModel = true;
        AdjustMinSize = false;
        ForceFaceDeflection = false;
        AllowQualityDecrease = false;
    }

    /// <summary>The factor of the deflection giving the default <see cref="MinSize"/> (the minimum mesh edge length).</summary>
    public static double RelMinSize() => 0.1;

    /// <summary>The 2D Delaunay triangulation algorithm factory to use.</summary>
    public MeshAlgoType MeshAlgo;

    /// <summary>The angular deflection used to tessellate the boundary edges.</summary>
    public double Angle;

    /// <summary>The linear deflection used to tessellate the boundary edges.</summary>
    public double Deflection;

    /// <summary>The angular deflection used to tessellate the face interior.</summary>
    public double AngleInterior;

    /// <summary>The linear deflection used to tessellate the face interior.</summary>
    public double DeflectionInterior;

    /// <summary>
    /// The minimum size of the triangles' edges, which prevents sinking into amplification on distorted curves and
    /// surfaces.
    /// </summary>
    public double MinSize;

    /// <summary>Switches multi-threaded computation on or off.</summary>
    public bool InParallel;

    /// <summary>
    /// Switches the relative computation of the edges' deflection on or off: each edge is then polygonized with
    /// <see cref="Deflection"/> times its size, and each face with the maximum deflection of its edges.
    /// </summary>
    public bool Relative;

    /// <summary>Whether the face's internal vertices are taken into account in the triangulation.</summary>
    public bool InternalVerticesMode;

    /// <summary>Whether the deviation of the triangulation from the interior of the face is checked.</summary>
    public bool ControlSurfaceDeflection;

    /// <summary>Enables the check of <see cref="ControlSurfaceDeflection"/> for all surfaces, analytical ones included.</summary>
    public bool EnableControlSurfaceDeflectionAllSurfaces;

    /// <summary>Cleans the temporary data model when the algorithm finishes.</summary>
    public bool CleanModel;

    /// <summary>Enables the local adjustment of the minimum size by the edge size. Off by default.</summary>
    public bool AdjustMinSize;

    /// <summary>Enables the use of the shape's tolerances in computing the face deflection. Off by default.</summary>
    public bool ForceFaceDeflection;

    /// <summary>Allows the quality of the generated mesh to decrease below the existing one's.</summary>
    public bool AllowQualityDecrease;

    /// <summary>Kerf: stops a public mesh too large to be meant (decision 1856); null for the engine's own.</summary>
    public MeshBudget? Budget;

    // Kerf additions: equality (translation-rules section 3)

    public readonly bool Equals(ToolParameters other) =>
        MeshAlgo == other.MeshAlgo
        && Angle.Equals(other.Angle)
        && Deflection.Equals(other.Deflection)
        && AngleInterior.Equals(other.AngleInterior)
        && DeflectionInterior.Equals(other.DeflectionInterior)
        && MinSize.Equals(other.MinSize)
        && InParallel == other.InParallel
        && Relative == other.Relative
        && InternalVerticesMode == other.InternalVerticesMode
        && ControlSurfaceDeflection == other.ControlSurfaceDeflection
        && EnableControlSurfaceDeflectionAllSurfaces == other.EnableControlSurfaceDeflectionAllSurfaces
        && CleanModel == other.CleanModel
        && AdjustMinSize == other.AdjustMinSize
        && ForceFaceDeflection == other.ForceFaceDeflection
        && AllowQualityDecrease == other.AllowQualityDecrease;

    public override readonly bool Equals(object? obj) => obj is ToolParameters other && Equals(other);

    public override readonly int GetHashCode()
    {
        var hash = new HashCode();
        hash.Add(MeshAlgo);
        hash.Add(Angle);
        hash.Add(Deflection);
        hash.Add(AngleInterior);
        hash.Add(DeflectionInterior);
        hash.Add(MinSize);
        int flags = (InParallel ? 1 : 0) | (Relative ? 2 : 0) | (InternalVerticesMode ? 4 : 0)
                     | (ControlSurfaceDeflection ? 8 : 0) | (EnableControlSurfaceDeflectionAllSurfaces ? 16 : 0)
                     | (CleanModel ? 32 : 0) | (AdjustMinSize ? 64 : 0) | (ForceFaceDeflection ? 128 : 0)
                     | (AllowQualityDecrease ? 256 : 0);
        hash.Add(flags);
        return hash.ToHashCode();
    }

    public static bool operator ==(in ToolParameters left, in ToolParameters right) => left.Equals(right);

    public static bool operator !=(in ToolParameters left, in ToolParameters right) => !left.Equals(right);

    public override readonly string ToString() =>
        $"IMeshTools_Parameters(MeshAlgo={MeshAlgo}, Angle={F(Angle)}, Deflection={F(Deflection)}, "
        + $"AngleInterior={F(AngleInterior)}, DeflectionInterior={F(DeflectionInterior)}, "
        + $"MinSize={F(MinSize)}, InParallel={InParallel}, Relative={Relative}, "
        + $"InternalVerticesMode={InternalVerticesMode}, ControlSurfaceDeflection={ControlSurfaceDeflection}, "
        + $"EnableControlSurfaceDeflectionAllSurfaces={EnableControlSurfaceDeflectionAllSurfaces}, "
        + $"CleanModel={CleanModel}, AdjustMinSize={AdjustMinSize}, ForceFaceDeflection={ForceFaceDeflection}, "
        + $"AllowQualityDecrease={AllowQualityDecrease})";

    private static string F(double value) => value.ToString("R", CultureInfo.InvariantCulture);
}
