// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Process;
using Kerf.Engine.Meshing;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Collections.Immutable;
using System.Text;

namespace Kerf.Kernel.DataExchange.Tests.Concurrency;

/// <summary>
/// Independent operations on many threads at once (unit 10I, plan 2.7): each gives exactly its serial result (BRep text
/// identical), and a setting one operation changes in its <see cref="OperationContext"/> stays in it.
/// </summary>
public class ConcurrencyTests
{
    // How many times each operation runs in the parallel rounds: enough to keep every processor busy
    private static int Copies => Math.Max(4, Environment.ProcessorCount / 2);

    private static string BRepText(in Shape shape)
    {
        using var writer = new StringWriter();
        BrepTools.Write(shape, writer);
        return writer.ToString();
    }

    private static Shape Box() => new MakeBox(10.0, 20.0, 30.0).Shape();

    private static Shape Cylinder() => new ApiMakeCylinder(4.0, 40.0).Shape();

    private static List<Shape> SubShapes(in Shape shape, ShapeType type)
    {
        var shapes = new List<Shape>();
        for (Explorer explorer = new(shape, type); explorer.More(); explorer.Next())
        {
            shapes.Add(explorer.Current());
        }
        return shapes;
    }

    private static Shape Fused() => new Fuse(Box(), Cylinder()).Shape();

    private static string StepText(in Shape shape)
    {
        var writer = new Writer();
        Assert.Equal(ReturnStatus.RetDone, writer.Transfer(shape, ModelType.AsIs));
        // a fixed time stamp, for repeatable text
        new MakeHeader(writer.Model()).SetTimeStamp(new TCollection_HAsciiString("2026-01-01T00:00:00"));
        using var stream = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, writer.WriteStream(stream));
        return Encoding.UTF8.GetString(stream.ToArray());
    }

    private static readonly Lazy<string> FusedStep = new(() => StepText(Fused()));

    // The representative operations, each giving its result as text
    private static readonly (string Name, Func<string> Run)[] Operations =
    [
        ("fuse", () => BRepText(Fused())),
        ("cut", () => BRepText(new Cut(Box(), Cylinder()).Shape())),
        ("fillet", () =>
        {
            Shape box = Box();
            var fillet = new MakeFillet(box);
            foreach (Shape edge in SubShapes(box, ShapeType.Edge).Take(4))
            {
                fillet.Add(2.0, ShapeCasts.Edge(edge));
            }
            return BRepText(fillet.Shape());
        }),
        ("thick solid", () =>
        {
            Shape box = Box();
            var closing = new NCollection_List<Shape>();
            closing.Append(SubShapes(box, ShapeType.Face)[5]);
            var thick = new MakeThickSolid();
            thick.MakeThickSolidByJoin(box, closing, -1.0, 1.0e-3, global::Kerf.Offsets.OffsetMode.Skin, false, false, JoinKind.Arc);
            return BRepText(thick.Shape());
        }),
        ("healing", () =>
        {
            var fixer = new ShapeFixer(Fused());
            fixer.Perform();
            return BRepText(fixer.Shape());
        }),
        ("shape processing", () => ProcessText(Cylinder(), overrides: null)),
        ("STEP write", () => StepText(Fused())),
        ("STEP read", () =>
        {
            var reader = new StepReader();
            Assert.Equal(ReturnStatus.RetDone,
                         reader.ReadStream("fused.stp", new MemoryStream(Encoding.UTF8.GetBytes(FusedStep.Value))));
            reader.TransferRoots();
            return BRepText(reader.OneShape());
        }),
        ("meshing", () => BRepText(new IncrementalMesh(Fused(), 0.5).Shape())),
    ];

    // STEP's shape processing ("ToSTEP", OCCT's resource file compiled in), with the flow's resource overrides if any
    private static string ProcessText(in Shape shape, ImmutableDictionary<string, string>? overrides)
    {
        using OperationContext.Scope scope = overrides is null
            ? OperationContext.Begin(OperationContext.Current)
            : OperationContext.Begin(ProcessContext.ResourceOverridesSetting, overrides);
        OperLibrary.Init();
        var context = new ShapeContext(shape, "STEP");
        ShapeProcess.Perform(context, "ToSTEP");
        return BRepText(context.Result());
    }

    // Splits the cylinder's faces at 90 degrees, where STEP's own sequence changes nothing
    private static readonly ImmutableDictionary<string, string> SplitAngleOverrides =
        ImmutableDictionary.CreateRange(StringComparer.Ordinal, new Dictionary<string, string>
        {
            ["ToSTEP.exec.op"] = "SplitAngle",
            ["ToSTEP.SplitAngle.Angle"] = "1.5707963267948966",
        });

    [Fact]
    public void Independent_operations_in_parallel_give_their_serial_results()
    {
        string[] serial = [.. Operations.Select(operation => operation.Run())];
        int count = Operations.Length;
        var results = new string[Copies * count];
        Parallel.For(0, results.Length, k => results[k] = Operations[k % count].Run());
        for (int k = 0; k < results.Length; k++)
        {
            Assert.True(serial[k % count] == results[k], $"{Operations[k % count].Name}, run {k / count}");
        }
    }

    [Fact]
    public void An_operation_with_its_own_resources_runs_beside_others_with_the_defaults()
    {
        string plain = ProcessText(Cylinder(), overrides: null);
        string split = ProcessText(Cylinder(), SplitAngleOverrides);
        Assert.NotEqual(plain, split); // the overrides change the result
        Assert.Equal(plain, ProcessText(Cylinder(), overrides: null)); // and stay in their scope

        var results = new string[Copies * 2];
        Parallel.For(0, results.Length, k => results[k] = ProcessText(Cylinder(), k % 2 == 0 ? null : SplitAngleOverrides));
        for (int k = 0; k < results.Length; k++)
        {
            Assert.True((k % 2 == 0 ? plain : split) == results[k], $"run {k}");
        }
    }

    [Fact]
    public async Task Settings_stay_in_their_flow_of_execution()
    {
        double defaultEpsilon = Kerf.Engine.Foundation.Color.Epsilon();
        bool defaultParallel = Options.GetParallelMode();

        // A setting made in a task ends with it
        await Task.Run(() =>
        {
            Kerf.Engine.Foundation.Color.SetEpsilon(0.5);
            Options.SetParallelMode(!defaultParallel);
            Assert.Equal(0.5, Kerf.Engine.Foundation.Color.Epsilon());
        });
        Assert.Equal(defaultEpsilon, Kerf.Engine.Foundation.Color.Epsilon());
        Assert.Equal(defaultParallel, Options.GetParallelMode());

        // Scopes on many threads at once each see their own, and the operations they start inherit it
        var failures = new System.Collections.Concurrent.ConcurrentBag<string>();
        Parallel.For(0, Copies * 4, k =>
        {
            bool parallel = k % 2 == 0;
            using OperationContext.Scope scope = OperationContext.Begin(
                OperationContext.Current.With(Options.ParallelModeSetting, parallel)
                                        .With(Kerf.Engine.Foundation.Color.EpsilonSetting, 0.001 * (k + 1)));
            var fuse = new Fuse(Box(), Cylinder());
            // the loops an operation starts see its settings
            var inLoop = new bool[2];
            Parallel.For(0, inLoop.Length, i => inLoop[i] = Options.GetParallelMode());
            if (fuse.RunParallel() != parallel || inLoop[0] != parallel || inLoop[1] != parallel
                || Kerf.Engine.Foundation.Color.Epsilon() != 0.001 * (k + 1))
            {
                failures.Add($"run {k}");
            }
        });
        Assert.Empty(failures);
        Assert.Equal(defaultParallel, Options.GetParallelMode());
    }
}
