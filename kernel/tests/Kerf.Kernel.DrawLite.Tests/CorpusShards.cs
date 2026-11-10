// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

// The corpus tests' shard classes (decision 2082): one per shard of each plan, so that xUnit runs the shards side by
// side. A plan's shard count is the first argument of its CorpusPlan; changing it means adding or removing lines here,
// and CorpusShardsTests fails until each plan has one class per shard, no more and no fewer. About 30 s of case time
// a shard (in a run of DrawLite alone) keeps them balanced.

public sealed class DrawBooleanCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard0>(theOutput) { }
public sealed class DrawBooleanCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard1>(theOutput) { }
public sealed class DrawBooleanCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard2>(theOutput) { }
public sealed class DrawBooleanCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard3>(theOutput) { }
public sealed class DrawBooleanCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard4>(theOutput) { }
public sealed class DrawBooleanCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard5>(theOutput) { }
public sealed class DrawBooleanCorpusShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard6>(theOutput) { }
public sealed class DrawBooleanCorpusShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard7>(theOutput) { }
public sealed class DrawBooleanCorpusShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard8>(theOutput) { }
public sealed class DrawBooleanCorpusShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard9>(theOutput) { }
public sealed class DrawBooleanCorpusShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard10>(theOutput) { }
public sealed class DrawBooleanCorpusShard11Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard11>(theOutput) { }
public sealed class DrawBooleanCorpusShard12Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard12>(theOutput) { }
public sealed class DrawBooleanCorpusShard13Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard13>(theOutput) { }
public sealed class DrawBooleanCorpusShard14Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard14>(theOutput) { }
public sealed class DrawBooleanCorpusShard15Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard15>(theOutput) { }
public sealed class DrawBooleanCorpusShard16Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard16>(theOutput) { }
public sealed class DrawBooleanCorpusShard17Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard17>(theOutput) { }
public sealed class DrawBooleanCorpusShard18Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard18>(theOutput) { }
public sealed class DrawBooleanCorpusShard19Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard19>(theOutput) { }
public sealed class DrawBooleanCorpusShard20Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard20>(theOutput) { }
public sealed class DrawBooleanCorpusShard21Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard21>(theOutput) { }
public sealed class DrawBooleanCorpusShard22Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard22>(theOutput) { }
public sealed class DrawBooleanCorpusShard23Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBooleanCorpusTests, Shard23>(theOutput) { }

public sealed class DrawBopCheckCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard0>(theOutput) { }
public sealed class DrawBopCheckCommandsShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard1>(theOutput) { }
public sealed class DrawBopCheckCommandsShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard2>(theOutput) { }
public sealed class DrawBopCheckCommandsShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard3>(theOutput) { }
public sealed class DrawBopCheckCommandsShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard4>(theOutput) { }
public sealed class DrawBopCheckCommandsShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard5>(theOutput) { }
public sealed class DrawBopCheckCommandsShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard6>(theOutput) { }
public sealed class DrawBopCheckCommandsShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard7>(theOutput) { }
public sealed class DrawBopCheckCommandsShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard8>(theOutput) { }
public sealed class DrawBopCheckCommandsShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard9>(theOutput) { }
public sealed class DrawBopCheckCommandsShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard10>(theOutput) { }
public sealed class DrawBopCheckCommandsShard11Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard11>(theOutput) { }
public sealed class DrawBopCheckCommandsShard12Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawBopCheckCommandsTests, Shard12>(theOutput) { }

public sealed class DrawCheapGapsCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard0>(theOutput) { }
public sealed class DrawCheapGapsCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard1>(theOutput) { }
public sealed class DrawCheapGapsCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard2>(theOutput) { }
public sealed class DrawCheapGapsCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard3>(theOutput) { }
public sealed class DrawCheapGapsCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard4>(theOutput) { }
public sealed class DrawCheapGapsCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard5>(theOutput) { }
public sealed class DrawCheapGapsCorpusShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard6>(theOutput) { }
public sealed class DrawCheapGapsCorpusShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard7>(theOutput) { }
public sealed class DrawCheapGapsCorpusShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawCheapGapsCorpusTests, Shard8>(theOutput) { }

public sealed class DrawConstraintCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstraintCorpusTests, Shard0>(theOutput) { }

public sealed class DrawConstructionCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard0>(theOutput) { }
public sealed class DrawConstructionCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard1>(theOutput) { }
public sealed class DrawConstructionCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard2>(theOutput) { }
public sealed class DrawConstructionCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard3>(theOutput) { }
public sealed class DrawConstructionCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard4>(theOutput) { }
public sealed class DrawConstructionCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard5>(theOutput) { }
public sealed class DrawConstructionCorpusShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard6>(theOutput) { }
public sealed class DrawConstructionCorpusShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard7>(theOutput) { }
public sealed class DrawConstructionCorpusShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard8>(theOutput) { }
public sealed class DrawConstructionCorpusShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard9>(theOutput) { }
public sealed class DrawConstructionCorpusShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard10>(theOutput) { }
public sealed class DrawConstructionCorpusShard11Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard11>(theOutput) { }
public sealed class DrawConstructionCorpusShard12Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard12>(theOutput) { }
public sealed class DrawConstructionCorpusShard13Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard13>(theOutput) { }
public sealed class DrawConstructionCorpusShard14Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawConstructionCorpusTests, Shard14>(theOutput) { }

public sealed class DrawDataExchangeCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawDataExchangeCorpusTests, Shard0>(theOutput) { }

public sealed class DrawEvolvedCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawEvolvedCommandsTests, Shard0>(theOutput) { }
public sealed class DrawEvolvedCommandsShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawEvolvedCommandsTests, Shard1>(theOutput) { }

public sealed class DrawFeatureCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFeatureCorpusTests, Shard0>(theOutput) { }
public sealed class DrawFeatureCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFeatureCorpusTests, Shard1>(theOutput) { }
public sealed class DrawFeatureCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFeatureCorpusTests, Shard2>(theOutput) { }

public sealed class DrawFillet2dCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFillet2dCorpusTests, Shard0>(theOutput) { }

public sealed class DrawFilletCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard0>(theOutput) { }
public sealed class DrawFilletCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard1>(theOutput) { }
public sealed class DrawFilletCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard2>(theOutput) { }
public sealed class DrawFilletCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard3>(theOutput) { }
public sealed class DrawFilletCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard4>(theOutput) { }
public sealed class DrawFilletCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard5>(theOutput) { }
public sealed class DrawFilletCorpusShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard6>(theOutput) { }
public sealed class DrawFilletCorpusShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard7>(theOutput) { }
public sealed class DrawFilletCorpusShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard8>(theOutput) { }
public sealed class DrawFilletCorpusShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard9>(theOutput) { }
public sealed class DrawFilletCorpusShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFilletCorpusTests, Shard10>(theOutput) { }

public sealed class DrawFillingCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawFillingCorpusTests, Shard0>(theOutput) { }

public sealed class DrawHealCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard0>(theOutput) { }
public sealed class DrawHealCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard1>(theOutput) { }
public sealed class DrawHealCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard2>(theOutput) { }
public sealed class DrawHealCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard3>(theOutput) { }
public sealed class DrawHealCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard4>(theOutput) { }
public sealed class DrawHealCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard5>(theOutput) { }
public sealed class DrawHealCorpusShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard6>(theOutput) { }
public sealed class DrawHealCorpusShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard7>(theOutput) { }
public sealed class DrawHealCorpusShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard8>(theOutput) { }
public sealed class DrawHealCorpusShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard9>(theOutput) { }
public sealed class DrawHealCorpusShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealCorpusTests, Shard10>(theOutput) { }

public sealed class DrawHealDivideCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard0>(theOutput) { }
public sealed class DrawHealDivideCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard1>(theOutput) { }
public sealed class DrawHealDivideCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard2>(theOutput) { }
public sealed class DrawHealDivideCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard3>(theOutput) { }
public sealed class DrawHealDivideCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard4>(theOutput) { }
public sealed class DrawHealDivideCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard5>(theOutput) { }
public sealed class DrawHealDivideCorpusShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard6>(theOutput) { }
public sealed class DrawHealDivideCorpusShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard7>(theOutput) { }
public sealed class DrawHealDivideCorpusShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard8>(theOutput) { }
public sealed class DrawHealDivideCorpusShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard9>(theOutput) { }
public sealed class DrawHealDivideCorpusShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard10>(theOutput) { }
public sealed class DrawHealDivideCorpusShard11Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard11>(theOutput) { }
public sealed class DrawHealDivideCorpusShard12Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHealDivideCorpusTests, Shard12>(theOutput) { }

public sealed class DrawHelixCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHelixCorpusTests, Shard0>(theOutput) { }
public sealed class DrawHelixCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHelixCorpusTests, Shard1>(theOutput) { }
public sealed class DrawHelixCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHelixCorpusTests, Shard2>(theOutput) { }

public sealed class DrawHlrCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHlrCorpusTests, Shard0>(theOutput) { }
public sealed class DrawHlrCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHlrCorpusTests, Shard1>(theOutput) { }
public sealed class DrawHlrCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHlrCorpusTests, Shard2>(theOutput) { }
public sealed class DrawHlrCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawHlrCorpusTests, Shard3>(theOutput) { }

public sealed class DrawIntersectCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawIntersectCorpusTests, Shard0>(theOutput) { }
public sealed class DrawIntersectCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawIntersectCorpusTests, Shard1>(theOutput) { }
public sealed class DrawIntersectCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawIntersectCorpusTests, Shard2>(theOutput) { }

public sealed class DrawMatCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawMatCommandsTests, Shard0>(theOutput) { }

public sealed class DrawOffsetCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard0>(theOutput) { }
public sealed class DrawOffsetCommandsShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard1>(theOutput) { }
public sealed class DrawOffsetCommandsShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard2>(theOutput) { }
public sealed class DrawOffsetCommandsShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard3>(theOutput) { }
public sealed class DrawOffsetCommandsShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard4>(theOutput) { }
public sealed class DrawOffsetCommandsShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard5>(theOutput) { }
public sealed class DrawOffsetCommandsShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard6>(theOutput) { }
public sealed class DrawOffsetCommandsShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard7>(theOutput) { }
public sealed class DrawOffsetCommandsShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard8>(theOutput) { }
public sealed class DrawOffsetCommandsShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard9>(theOutput) { }
public sealed class DrawOffsetCommandsShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard10>(theOutput) { }
public sealed class DrawOffsetCommandsShard11Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard11>(theOutput) { }
public sealed class DrawOffsetCommandsShard12Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard12>(theOutput) { }
public sealed class DrawOffsetCommandsShard13Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetCommandsTests, Shard13>(theOutput) { }

public sealed class DrawOffsetDraftCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard0>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard1>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard2>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard3>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard4>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard5>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard6>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard7>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard8>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard9>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard10>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard11Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard11>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard12Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard12>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard13Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard13>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard14Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard14>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard15Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard15>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard16Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard16>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard17Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard17>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard18Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard18>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard19Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard19>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard20Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard20>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard21Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard21>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard22Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard22>(theOutput) { }
public sealed class DrawOffsetDraftCorpusShard23Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawOffsetDraftCorpusTests, Shard23>(theOutput) { }

public sealed class DrawPeriodicityCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawPeriodicityCommandsTests, Shard0>(theOutput) { }
public sealed class DrawPeriodicityCommandsShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawPeriodicityCommandsTests, Shard1>(theOutput) { }

public sealed class DrawPipeCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawPipeCommandsTests, Shard0>(theOutput) { }

public sealed class DrawPipeMakerCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawPipeMakerCommandsTests, Shard0>(theOutput) { }
public sealed class DrawPipeMakerCommandsShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawPipeMakerCommandsTests, Shard1>(theOutput) { }
public sealed class DrawPipeMakerCommandsShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawPipeMakerCommandsTests, Shard2>(theOutput) { }
public sealed class DrawPipeMakerCommandsShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawPipeMakerCommandsTests, Shard3>(theOutput) { }

public sealed class DrawRemoveFeaturesCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawRemoveFeaturesCommandsTests, Shard0>(theOutput) { }

public sealed class DrawShapeUpgradeCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawShapeUpgradeCommandsTests, Shard0>(theOutput) { }
public sealed class DrawShapeUpgradeCommandsShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawShapeUpgradeCommandsTests, Shard1>(theOutput) { }
public sealed class DrawShapeUpgradeCommandsShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawShapeUpgradeCommandsTests, Shard2>(theOutput) { }
public sealed class DrawShapeUpgradeCommandsShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawShapeUpgradeCommandsTests, Shard3>(theOutput) { }
public sealed class DrawShapeUpgradeCommandsShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawShapeUpgradeCommandsTests, Shard4>(theOutput) { }
public sealed class DrawShapeUpgradeCommandsShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawShapeUpgradeCommandsTests, Shard5>(theOutput) { }
public sealed class DrawShapeUpgradeCommandsShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawShapeUpgradeCommandsTests, Shard6>(theOutput) { }

public sealed class DrawStepCafCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard0>(theOutput) { }
public sealed class DrawStepCafCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard1>(theOutput) { }
public sealed class DrawStepCafCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard2>(theOutput) { }
public sealed class DrawStepCafCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard3>(theOutput) { }
public sealed class DrawStepCafCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard4>(theOutput) { }
public sealed class DrawStepCafCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard5>(theOutput) { }
public sealed class DrawStepCafCorpusShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard6>(theOutput) { }
public sealed class DrawStepCafCorpusShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard7>(theOutput) { }
public sealed class DrawStepCafCorpusShard8Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard8>(theOutput) { }
public sealed class DrawStepCafCorpusShard9Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard9>(theOutput) { }
public sealed class DrawStepCafCorpusShard10Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard10>(theOutput) { }
public sealed class DrawStepCafCorpusShard11Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard11>(theOutput) { }
public sealed class DrawStepCafCorpusShard12Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard12>(theOutput) { }
public sealed class DrawStepCafCorpusShard13Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard13>(theOutput) { }
public sealed class DrawStepCafCorpusShard14Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard14>(theOutput) { }
public sealed class DrawStepCafCorpusShard15Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCafCorpusTests, Shard15>(theOutput) { }

public sealed class DrawStepCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCorpusTests, Shard0>(theOutput) { }
public sealed class DrawStepCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCorpusTests, Shard1>(theOutput) { }
public sealed class DrawStepCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCorpusTests, Shard2>(theOutput) { }
public sealed class DrawStepCorpusShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCorpusTests, Shard3>(theOutput) { }
public sealed class DrawStepCorpusShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCorpusTests, Shard4>(theOutput) { }
public sealed class DrawStepCorpusShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStepCorpusTests, Shard5>(theOutput) { }

public sealed class DrawStlCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawStlCorpusTests, Shard0>(theOutput) { }

public sealed class DrawSurfaceFromCurvesCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawSurfaceFromCurvesCorpusTests, Shard0>(theOutput) { }

public sealed class DrawVolumeCellsCommandsShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawVolumeCellsCommandsTests, Shard0>(theOutput) { }
public sealed class DrawVolumeCellsCommandsShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawVolumeCellsCommandsTests, Shard1>(theOutput) { }
public sealed class DrawVolumeCellsCommandsShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawVolumeCellsCommandsTests, Shard2>(theOutput) { }
public sealed class DrawVolumeCellsCommandsShard3Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawVolumeCellsCommandsTests, Shard3>(theOutput) { }
public sealed class DrawVolumeCellsCommandsShard4Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawVolumeCellsCommandsTests, Shard4>(theOutput) { }
public sealed class DrawVolumeCellsCommandsShard5Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawVolumeCellsCommandsTests, Shard5>(theOutput) { }
public sealed class DrawVolumeCellsCommandsShard6Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawVolumeCellsCommandsTests, Shard6>(theOutput) { }
public sealed class DrawVolumeCellsCommandsShard7Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawVolumeCellsCommandsTests, Shard7>(theOutput) { }

public sealed class DrawXdeCorpusShard0Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawXdeCorpusTests, Shard0>(theOutput) { }
public sealed class DrawXdeCorpusShard1Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawXdeCorpusTests, Shard1>(theOutput) { }
public sealed class DrawXdeCorpusShard2Tests(ITestOutputHelper theOutput) : CorpusShardTests<DrawXdeCorpusTests, Shard2>(theOutput) { }

// The shards' indices
public sealed class Shard0 : IShardIndex { public static int Index => 0; }
public sealed class Shard1 : IShardIndex { public static int Index => 1; }
public sealed class Shard2 : IShardIndex { public static int Index => 2; }
public sealed class Shard3 : IShardIndex { public static int Index => 3; }
public sealed class Shard4 : IShardIndex { public static int Index => 4; }
public sealed class Shard5 : IShardIndex { public static int Index => 5; }
public sealed class Shard6 : IShardIndex { public static int Index => 6; }
public sealed class Shard7 : IShardIndex { public static int Index => 7; }
public sealed class Shard8 : IShardIndex { public static int Index => 8; }
public sealed class Shard9 : IShardIndex { public static int Index => 9; }
public sealed class Shard10 : IShardIndex { public static int Index => 10; }
public sealed class Shard11 : IShardIndex { public static int Index => 11; }
public sealed class Shard12 : IShardIndex { public static int Index => 12; }
public sealed class Shard13 : IShardIndex { public static int Index => 13; }
public sealed class Shard14 : IShardIndex { public static int Index => 14; }
public sealed class Shard15 : IShardIndex { public static int Index => 15; }
public sealed class Shard16 : IShardIndex { public static int Index => 16; }
public sealed class Shard17 : IShardIndex { public static int Index => 17; }
public sealed class Shard18 : IShardIndex { public static int Index => 18; }
public sealed class Shard19 : IShardIndex { public static int Index => 19; }
public sealed class Shard20 : IShardIndex { public static int Index => 20; }
public sealed class Shard21 : IShardIndex { public static int Index => 21; }
public sealed class Shard22 : IShardIndex { public static int Index => 22; }
public sealed class Shard23 : IShardIndex { public static int Index => 23; }
