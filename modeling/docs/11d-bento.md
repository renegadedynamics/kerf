# Unit 11D: runtime settings for Bento's branch

Unit 11D measured what limits parallel recalculation (kernel `docs/performance.md`, "Unit 11D"; decisions 18801 to 18806). Most of it is two
runtime settings, which belong to the host. No Kerf API changes, and shapes, volumes and face counts are unchanged.

## What Bento has to change

In `Bento.App/Bento.App.csproj`, beside the collector's settings it already has:

```xml
<TieredPGO>false</TieredPGO>
```

and in an `ItemGroup`:

```xml
<RuntimeHostConfigurationOption Include="System.Runtime.TieredCompilation.CallCountingDelayMs" Value="0" />
```

**Keep** `<ServerGarbageCollection>true</ServerGarbageCollection>` and `<ConcurrentGarbageCollection>true</ConcurrentGarbageCollection>`. Bento's choice of the
server collector, with DATAS left on, is the right one. Don't set `GCHeapCount`, `GCgen0size` or `GCConserveMemory`, and don't turn DATAS off.

Bento's test projects don't need these for correctness. A project that times parallel work should use the same settings.

## What to expect

Measured on Bento's eleven examples, 24 copies on 24 logical processors (12 cores), on the benchmark rather than in Bento:

| | Server collector alone (Bento today) | With both settings |
|---|---:|---:|
| A round, ms | 1,250 | 620 |
| Recalculation, ms, passes 2 to 9 | 370 | 180 |
| Opening, ms | 730 | 615 |
| Peak working set in the round, MB | 320 to 420 | 320 to 410 |

## Check these in Bento

- **Bento's own start-up.** `CallCountingDelayMs=0` starts counting calls at once, in every assembly. Kerf's are ReadyToRun, and gain. Avalonia and Bento's own may not be, and the benchmark has no UI, so compare the time to the first window with and without it. If it's worse, drop that one line and keep `TieredPGO`: the round is then about 650 ms and recalculation about 260 ms.
- **Very long sessions.** Without PGO, code that has fully settled is about 6% slower (a recalculation 164 ms against 154, after more than a hundred passes), and a minute of Booleans 11%. PGO would pay for its slow start after about two minutes of continuous recalculation. That's the right trade for a spreadsheet, but it is one.
