// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_Operator.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_Operator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Healing.Process;

/// <summary>
/// The abstract operator that the performer (<see cref="ShapeProcess"/>) runs on a context.
/// </summary>
internal abstract class Operator : Transient
{
    /// <summary>Performs the operation on the context; true if done.</summary>
    public abstract bool Perform(ProcessContext context, ProgressRange progress = default);
}
