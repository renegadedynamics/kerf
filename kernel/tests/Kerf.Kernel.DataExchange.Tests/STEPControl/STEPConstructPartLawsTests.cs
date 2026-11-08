// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
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
}
