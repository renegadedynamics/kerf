// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Exchange.Step.Parameters.WriteMode_StepSchema;

namespace Kerf.Kernel.DataExchange.Tests.STEPControl;

/// <summary>
/// A part's product data as STEP's schemas want it (unit 12Q, decision 1753): one chain from the shape's definition to
/// its product, the product named as asked, and each schema's own context and category; and what's set is what's read.
/// </summary>
public class STEPConstructPartLawsTests
{
    private static Part Made(Kerf.Engine.Exchange.Step.Parameters.WriteMode_StepSchema theSchema, out StepShapeRepresentation theShape, out ApplicationContext theContext)
    {
        var aModel = new StepModel();
        aModel.InternalParameters.WriteSchema = theSchema;
        theContext = new ApplicationContext();
        theContext.Init(new TCollection_HAsciiString("core data for automotive mechanical design processes"));
        theShape = new StepShapeRepresentation();
        var aPart = new Part();
        aPart.MakeSDR(theShape, new TCollection_HAsciiString("Body"), theContext, aModel);
        return aPart;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void A_part_is_one_chain_from_its_shape_to_its_product(int theSchema)
    {
        Part aPart = Made((Kerf.Engine.Exchange.Step.Parameters.WriteMode_StepSchema)theSchema, out StepShapeRepresentation aShape, out ApplicationContext aContext);
        Assert.True(aPart.IsDone);
        Assert.Same(aShape, aPart.SRValue());
        // the shape's definition is the product definition shape, defining the product definition, of the formation, of
        // the product: each the next one's
        Assert.Same(aPart.PDS(), aPart.SDRValue().Definition().PropertyDefinition());
        Assert.Same(aPart.PD(), aPart.PDS().Definition().ProductDefinition());
        Assert.Same(aPart.PDF(), aPart.PD().Formation());
        Assert.Same(aPart.Product(), aPart.PDF().OfProduct());
        Assert.Same(aContext, aPart.AC());
        Assert.Same(aContext, aPart.PC().FrameOfReference());
        Assert.Same(aContext, aPart.PDC().FrameOfReference());
        Assert.Equal("Body", aPart.Pid().ToCString());
        Assert.Equal("Body", aPart.Pname().ToCString());
        Assert.Equal("", aPart.Pdescription().ToCString());
        Assert.Equal("design", aPart.PD().Id().ToCString());
        Assert.Equal("", aPart.PDdescription().ToCString());
        Assert.Equal("", aPart.PDFid().ToCString());
        Assert.Equal("", aPart.PDFdescription().ToCString());
        Assert.Equal("", aPart.PDSname().ToCString());
        Assert.Equal("", aPart.PDSdescription().ToCString());
        Assert.Equal("mechanical", aPart.PCdisciplineType().ToCString());
        Assert.Equal("", aPart.PCname().ToCString());
        Assert.Equal("design", aPart.PDCstage().ToCString());
        Assert.Equal("core data for automotive mechanical design processes", aPart.ACapplication().ToCString());
        Assert.Same(aPart.Product(), aPart.PRPC().Products().Value(1));
    }

    [Fact]
    public void Each_schema_has_its_own_context_and_category()
    {
        // AP214 (CD): a mechanical context, a part definition, a product type "part"
        Part a214 = Made(WriteMode_StepSchema_AP214CD, out _, out _);
        Assert.IsType<MechanicalContext>(a214.PC());
        Assert.Equal("part definition", a214.PDCname().ToCString());
        Assert.IsType<ProductType>(a214.PRPC());
        Assert.Equal("part", a214.PRPCname().ToCString());
        // AP203: a design context with no name, a formation with its source not known, a category "detail"
        Part a203 = Made(WriteMode_StepSchema_AP203, out _, out _);
        Assert.IsType<MechanicalContext>(a203.PC());
        Assert.IsType<DesignContext>(a203.PDC());
        Assert.Equal("", a203.PDCname().ToCString());
        Assert.Equal(Source.SNotKnown, Assert.IsType<ProductDefinitionFormationWithSpecifiedSource>(a203.PDF()).MakeOrBuy());
        Assert.Equal("detail", a203.PRPCname().ToCString());
        // AP242: a plain product context and category
        Part a242 = Made(WriteMode_StepSchema_AP242DIS, out _, out _);
        Assert.Equal(typeof(ProductContext), a242.PC().GetType());
        Assert.Equal(typeof(ProductRelatedProductCategory), a242.PRPC().GetType());
        Assert.Equal("part", a242.PRPCname().ToCString());
        Assert.Null(a242.PRPCdescription());
    }

    [Fact]
    public void What_is_set_is_what_is_read_and_a_read_definition_is_its_own()
    {
        Part aPart = Made(WriteMode_StepSchema_AP214IS, out _, out _);
        aPart.SetPid(new TCollection_HAsciiString("B-1"));
        aPart.SetPname(new TCollection_HAsciiString("Bracket"));
        aPart.SetPdescription(new TCollection_HAsciiString("left"));
        aPart.SetPDFid(new TCollection_HAsciiString("A"));
        aPart.SetPDFdescription(new TCollection_HAsciiString("first issue"));
        aPart.SetPDdescription(new TCollection_HAsciiString("as designed"));
        aPart.SetPDSname(new TCollection_HAsciiString("shape"));
        aPart.SetPDSdescription(new TCollection_HAsciiString("its shape"));
        aPart.SetPCname(new TCollection_HAsciiString("context"));
        aPart.SetPCdisciplineType(new TCollection_HAsciiString("electrical"));
        aPart.SetACapplication(new TCollection_HAsciiString("Kerf"));
        aPart.SetPDCname(new TCollection_HAsciiString("detail definition"));
        aPart.SetPDCstage(new TCollection_HAsciiString("manufacturing"));
        aPart.SetPRPCname(new TCollection_HAsciiString("assembly"));
        aPart.SetPRPCdescription(new TCollection_HAsciiString("a group"));
        Assert.Equal(("B-1", "Bracket", "left"), (aPart.Pid().ToCString(), aPart.Pname().ToCString(), aPart.Pdescription().ToCString()));
        Assert.Equal(("A", "first issue", "as designed"), (aPart.PDFid().ToCString(), aPart.PDFdescription().ToCString(), aPart.PDdescription().ToCString()));
        Assert.Equal(("shape", "its shape"), (aPart.PDSname().ToCString(), aPart.PDSdescription().ToCString()));
        Assert.Equal(("context", "electrical", "Kerf"), (aPart.PCname().ToCString(), aPart.PCdisciplineType().ToCString(), aPart.ACapplication().ToCString()));
        Assert.Equal(("detail definition", "manufacturing"), (aPart.PDCname().ToCString(), aPart.PDCstage().ToCString()));
        Assert.Equal(("assembly", "a group"), (aPart.PRPCname().ToCString(), aPart.PRPCdescription().ToCString()));

        // a part read from a definition is that definition, and done only with one
        var aRead = new Part();
        Assert.False(aRead.IsDone);
        aRead.ReadSDR(aPart.SDRValue());
        Assert.True(aRead.IsDone);
        Assert.Equal("Bracket", aRead.Pname().ToCString());
        Assert.Same(aPart.SRValue(), aRead.SRValue());
        aRead.ReadSDR(null!);
        Assert.False(aRead.IsDone);
        Assert.Null(aRead.SRValue());
    }

    [Fact]
    public void A_shape_written_is_found_as_its_entity_and_an_entity_read_as_its_shape()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        var aWriter = new Writer();
        Assert.Equal(ReturnStatus.RetDone, aWriter.Transfer(aBox, ModelType.AsIs));
        var aFinder = aWriter.WS()!.TransferWriter()!.FinderProcess()!;
        Assert.IsType<StepManifoldSolidBrep>(StepConstruct.FindEntity(aFinder, aBox));
        Shape aFace = new Explorer(aBox, ShapeType.Face).Current();
        Assert.IsType<StepAdvancedFace>(StepConstruct.FindEntity(aFinder, aFace));
        // moved, the box isn't what was written; asked with its placement, it's found unmoved, and the placement says so
        Shape aMoved = aBox.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(1, 0, 0))));
        Assert.Null(StepConstruct.FindEntity(aFinder, aMoved));
        Placement aLoc = new();
        Assert.IsType<StepManifoldSolidBrep>(StepConstruct.FindEntity(aFinder, aMoved, ref aLoc));
        Assert.True(aLoc.IsIdentity);

        using var aStream = new MemoryStream();
        aWriter.WriteStream(aStream);
        var aReader = new StepReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadStream("box.stp", new MemoryStream(aStream.ToArray())));
        aReader.TransferRoots();
        var aProcess = aReader.WS()!.TransferReader()!.TransientProcess()!;
        var aModel = aReader.WS()!.Model()!;
        StepManifoldSolidBrep? aBrep = null;
        for (int i = 1; i <= aModel.EntityCount && aBrep is null; i++)
        {
            aBrep = aModel.Value(i) as StepManifoldSolidBrep;
        }
        Shape aRead = StepConstruct.FindShape(aProcess, aBrep!);
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aRead, aProps);
        Assert.Equal(6000, aProps.Mass(), 6);
        Assert.True(StepConstruct.FindShape(aProcess, new StepManifoldSolidBrep()).IsNull);
    }

    [Theory]
    [InlineData(1, "INCH")]
    [InlineData(2, ".MILLI.")]
    [InlineData(4, "FOOT")]
    [InlineData(5, "MILE")]
    [InlineData(6, "SI_UNIT($,.METRE.)")]
    [InlineData(7, ".KILO.")]
    [InlineData(8, "MIL")]
    [InlineData(9, ".MICRO.")]
    [InlineData(10, ".CENTI.")]
    [InlineData(11, "MICROINCH")]
    public void A_box_written_in_any_unit_of_the_writer_reads_back_its_size(int theUnit, string theMarker)
    {
        // the writer converts millimetres to the file's unit and the reader back: the box is 6000 again
        var aWriter = new Writer();
        aWriter.WS()!.Statics().SetIVal("write.step.unit", theUnit);
        Assert.Equal(ReturnStatus.RetDone, aWriter.Transfer(new MakeBox(10, 20, 30).Shape(), ModelType.AsIs));
        using var aStream = new MemoryStream();
        aWriter.WriteStream(aStream);
        string aText = Encoding.ASCII.GetString(aStream.ToArray());
        Assert.Contains(theMarker, aText);
        var aReader = new StepReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadStream("unit.stp", new MemoryStream(aStream.ToArray())));
        aReader.TransferRoots();
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aReader.OneShape(), aProps);
        Assert.Equal(6000, aProps.Mass(), 6000 * 1e-9);
    }

    [Fact]
    public void Each_si_prefix_is_its_power_of_ten()
    {
        (SiPrefix Prefix, int Power)[] aPrefixes =
        [
            (SiPrefix.SpExa, 18), (SiPrefix.SpPeta, 15), (SiPrefix.SpTera, 12), (SiPrefix.SpGiga, 9), (SiPrefix.SpMega, 6),
            (SiPrefix.SpKilo, 3), (SiPrefix.SpHecto, 2), (SiPrefix.SpDeca, 1), (SiPrefix.SpDeci, -1), (SiPrefix.SpCenti, -2),
            (SiPrefix.SpMilli, -3), (SiPrefix.SpMicro, -6), (SiPrefix.SpNano, -9), (SiPrefix.SpPico, -12), (SiPrefix.SpFemto, -15),
            (SiPrefix.SpAtto, -18),
        ];
        foreach ((SiPrefix aPrefix, int aPower) in aPrefixes)
        {
            Assert.Equal(Math.Pow(10, aPower), UnitContext.ConvertSiPrefix(aPrefix), Math.Pow(10, aPower) * 1e-12);
        }
    }

    [Theory]
    [InlineData(1, "automotive_design", 1997, "committee draft", "core data for automotive mechanical design processes")]
    [InlineData(2, "automotive_design", 1998, "draft international standard", "core data for automotive mechanical design processes")]
    [InlineData(3, "config_control_design", 1994, "international standard", "configuration controlled 3D designs of mechanical parts and assemblies")]
    [InlineData(4, "automotive_design", 2000, "international standard", "core data for automotive mechanical design processes")]
    [InlineData(5, "ap242_managed_model_based_3d_engineering", 2013, "international standard", "Managed model based 3d engineering")]
    public void Each_schema_has_its_protocol(int theSchema, string theName, int theYear, string theStatus, string theApplication)
    {
        var aModel = new StepModel();
        aModel.InternalParameters.WriteSchema = (Kerf.Engine.Exchange.Step.Parameters.WriteMode_StepSchema)theSchema;
        var aTool = new ContextTool(aModel);
        // with none in the model, there's none until it's added, and the defaults stand
        Assert.Null(aTool.GetAPD());
        Assert.Equal(1998, aTool.GetACyear());
        Assert.Equal("", aTool.GetACschemaName().ToCString());
        Assert.Equal("", aTool.GetACname().ToCString());
        aTool.AddAPD();
        Assert.Equal(theName, aTool.GetACschemaName().ToCString());
        Assert.Equal(theYear, aTool.GetACyear());
        Assert.Equal(theStatus, aTool.GetACstatus().ToCString());
        Assert.Equal(theApplication, aTool.GetACname().ToCString());
        Assert.Equal(theSchema == 3, aTool.IsAP203);
        Assert.Equal(theName == "automotive_design", aTool.IsAP214);
        Assert.Equal(theSchema == 5, aTool.IsAP242);
        aTool.SetACname(new TCollection_HAsciiString("Kerf"));
        aTool.SetACstatus(new TCollection_HAsciiString("draft"));
        aTool.SetACschemaName(new TCollection_HAsciiString("my_schema"));
        aTool.SetACyear(2026);
        Assert.Equal(("Kerf", "draft", "my_schema", 2026), (aTool.GetACname().ToCString(), aTool.GetACstatus().ToCString(), aTool.GetACschemaName().ToCString(), aTool.GetACyear()));
        Assert.False(aTool.IsAP203 || aTool.IsAP214 || aTool.IsAP242);
        // the default axis is the world's
        StepAxis2Placement3d anAxis = aTool.GetDefaultAxis();
        Assert.Equal(0, anAxis.Location().CoordinatesValue(1));
        Assert.Same(anAxis, aTool.GetDefaultAxis());
    }

    [Fact]
    public void An_assembly_position_is_a_level_and_an_index_on_it()
    {
        var aTool = new ContextTool(new StepModel());
        Assert.Equal((0, 0), (aTool.Level(), aTool.Index()));
        aTool.NextIndex();
        Assert.Equal(0, aTool.Index());
        aTool.NextLevel();
        aTool.NextIndex();
        aTool.NextIndex();
        Assert.Equal((1, 3), (aTool.Level(), aTool.Index()));
        aTool.NextLevel();
        aTool.SetIndex(7);
        aTool.PrevIndex();
        Assert.Equal((2, 6), (aTool.Level(), aTool.Index()));
        aTool.PrevLevel();
        Assert.Equal((1, 3), (aTool.Level(), aTool.Index()));
        aTool.SetLevel(4);
        Assert.Equal((4, 1), (aTool.Level(), aTool.Index()));
        aTool.SetLevel(1);
        Assert.Equal((1, 3), (aTool.Level(), aTool.Index()));
        aTool.SetLevel(0);
        aTool.PrevLevel();
        Assert.Equal((0, 0), (aTool.Level(), aTool.Index()));
    }
}
