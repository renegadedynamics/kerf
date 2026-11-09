// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Meshing;

/// <summary>
/// For a torus, whose grid alone OCCT trusts: inserts nodes only where the mesh is further from the surface than
/// the deflection asked for, so a grid that meets it stays as it is (decision 1929). OCCT #1444's torus is 1.43 out for 0.45
/// asked. At a relative deflection the limit is the face's own, which relative mode works out from the face's and its
/// edges' sizes (decision 2056).
/// </summary>
internal sealed class DelaunayDeflectionGuardMeshAlgo<RangeSplitter> : DelaunayDeflectionControlMeshAlgo<RangeSplitter>
    where RangeSplitter : DefaultRangeSplitter, new()
{
    /// <inheritdoc/>
    protected override double ControlledDeflection() => Deflection.SurfaceLimit(GetDFace(), GetParameters());

    /// <inheritdoc/>
    protected override bool ClearsFrontier => true;

    /// <inheritdoc/>
    protected override bool ReportsExceeded => true;
}
