// Unit 6F merge shim: lets 6A's stub BOP and Section (ported by another 6F fork) compile against the ported builder.
// Delete this file when the BOP/Section/Splitter port is merged: nothing calls BuildStubResult any more.

namespace Occt;

public partial class BOPAlgo_Builder
{
    /// <summary>Unused: 6A's stub hook, overridden by the stub BOP and Section until their port is merged.</summary>
    protected virtual void BuildStubResult()
    {
    }
}
