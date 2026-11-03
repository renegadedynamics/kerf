// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using Kerf.Kernel.DataExchange.Tests.StepGen;
using Xunit.Abstractions;

namespace Kerf.Kernel.DataExchange.Tests.STEPControl;

/// <summary>STEPControl's reader and writer, unit by unit (the comparison with native OCCT is in the oracle tests).</summary>
public class STEPControlTests(ITestOutputHelper theOutput)
{
    private static string BRepText(in Shape theShape)
    {
        using var aWriter = new StringWriter();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static string DataStep(string theName) =>
        Path.Combine(StepRoundTripTests.Metadata("OcctSourceDir"), "data", "step", theName);

    [Fact]
    public void ReadsAFileAndPublishesFrozenShapes()
    {
        string aPath = DataStep("screw.step");
        if (!File.Exists(aPath))
        {
            return;
        }
        var aReader = new StepReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadFile(aPath));
        int aNbRoots = aReader.NbRootsForTransfer();
        Assert.True(aNbRoots > 0);
        int aNbTransferred = aReader.TransferRoots();
        Assert.True(aNbTransferred > 0);
        Shape aShape = aReader.OneShape();
        Assert.False(aShape.IsNull);
        Assert.True(aShape.TShape()!.IsFrozen);
        theOutput.WriteLine($"{aNbRoots} roots, {aReader.ShapeCount} shapes, {BRepText(aShape).Length} chars of BRep");
    }

    [Fact]
    public void ReadsAStreamWithLimitsAndCancellation()
    {
        string aPath = DataStep("screw.step");
        if (!File.Exists(aPath))
        {
            return;
        }
        byte[] aBytes = File.ReadAllBytes(aPath);

        // a stream reads as the file does
        var aFileReader = new StepReader();
        aFileReader.ReadFile(aPath);
        aFileReader.TransferRoots();
        var aStreamReader = new StepReader();
        Assert.Equal(ReturnStatus.RetDone, aStreamReader.ReadStream("screw.step", new MemoryStream(aBytes)));
        aStreamReader.TransferRoots();
        Assert.Equal(BRepText(aFileReader.OneShape()), BRepText(aStreamReader.OneShape()));

        // a limit refuses the file
        var aLimited = new StepReader();
        Assert.Equal(ReturnStatus.RetFail,
                     aLimited.ReadStream("screw.step", new MemoryStream(aBytes), default, new ParserReadLimits { MaxRecords = 10 }));

        // a cancelled read stops
        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        var anIndicator = new ProgressIndicator(aSource.Token);
        var aCancelled = new StepReader();
        Assert.NotEqual(ReturnStatus.RetDone,
                        aCancelled.ReadStream("screw.step", new MemoryStream(aBytes), anIndicator.Start()));
    }

    private static Shape Box()
    {
        return new MakeBox(10.0, 20.0, 30.0).Shape();
    }

    private static string WriteToString(Writer theWriter)
    {
        // a fixed header, for repeatable text
        var aHeader = new MakeHeader(theWriter.Model());
        aHeader.SetTimeStamp(new TCollection_HAsciiString("2026-01-01T00:00:00"));
        using var aStream = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, theWriter.WriteStream(aStream));
        return Encoding.UTF8.GetString(aStream.ToArray());
    }

    [Fact]
    public void WritesAndReadsBackABox()
    {
        Shape aBox = Box();
        string aBefore = BRepText(aBox);
        var aWriter = new Writer();
        Assert.Equal(ReturnStatus.RetDone, aWriter.Transfer(aBox, ModelType.AsIs));
        string aText = WriteToString(aWriter);
        Assert.Contains("MANIFOLD_SOLID_BREP", aText);
        Assert.Equal(aBefore, BRepText(aBox)); // the writer never changes its input

        // the same shape written again in a new writer gives the same text
        var aWriter2 = new Writer();
        aWriter2.Transfer(aBox, ModelType.AsIs);
        Assert.Equal(aText, WriteToString(aWriter2));

        var aReader = new StepReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadStream("box.stp", new MemoryStream(Encoding.UTF8.GetBytes(aText))));
        Assert.Equal(1, aReader.TransferRoots());
        Shape aRead = aReader.OneShape();
        Assert.Equal(ShapeType.Solid, aRead.Type);
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aRead, aProps);
        Assert.Equal(6000.0, aProps.Mass(), 6);
    }

    [Fact]
    public void WritesAnAssemblyOfLocatedInstances()
    {
        Shape aBox = Box();
        var aBuilder = new BrepBuilder();
        var aCompound = new Compound();
        aBuilder.MakeCompound(ref aCompound);
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(100.0, 0.0, 0.0));
        aBuilder.Add(aCompound, aBox);
        aBuilder.Add(aCompound, aBox.Moved(new Placement(aTrsf)));

        var aWriter = new Writer();
        Assert.Equal(ReturnStatus.RetDone, aWriter.Transfer(aCompound, ModelType.AsIs));
        string aText = WriteToString(aWriter);
        Assert.Contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE('1'", aText);
        Assert.Contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE('2'", aText);

        var aReader = new StepReader();
        aReader.ReadStream("asm.stp", new MemoryStream(Encoding.UTF8.GetBytes(aText)));
        aReader.TransferRoots();
        Shape aRead = aReader.OneShape();
        Assert.Equal(ShapeType.Compound, aRead.Type);
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aRead, aProps);
        Assert.Equal(12000.0, aProps.Mass(), 6);
    }

    [Fact]
    public void ParametersComeFromTheSessionsStatics()
    {
        var aWriter = new Writer();
        aWriter.WS().Statics().SetCVal("write.step.schema", "AP203");
        aWriter.Transfer(Box(), ModelType.ManifoldSolidBrep);
        string aText = WriteToString(aWriter);
        Assert.Contains("CONFIG_CONTROL_DESIGN", aText);
        Assert.Contains("CC_DESIGN_APPROVAL", aText);

        // another writer isn't affected
        var aWriter2 = new Writer();
        aWriter2.Transfer(Box(), ModelType.ManifoldSolidBrep);
        Assert.Contains("AUTOMOTIVE_DESIGN", WriteToString(aWriter2));
    }
}
