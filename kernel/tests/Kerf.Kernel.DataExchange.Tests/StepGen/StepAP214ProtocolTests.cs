// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Ap214.ReadWrite;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;

namespace Kerf.Kernel.DataExchange.Tests.StepGen;

/// <summary>
/// StepAP214's protocol and its two modules (unit 8C2): the protocol's case numbers, the general module's empty
/// entities and the read-write module's type names agree with each other, as OCCT's dispatch tables do.
/// </summary>
public sealed class StepAP214ProtocolTests
{
    /// <summary>Every entity class of the generated packages with the protocol's case number for it.</summary>
    private static List<(Type Type, int Case)> Cases()
    {
        var aProtocol = global::Kerf.Engine.Exchange.Step.Ap214.StepAP214.Protocol();
        return typeof(global::Kerf.Engine.Exchange.Step.Ap214.Ap214Protocol).Assembly.GetTypes()
            .Where(t => Kerf.Engine.Foundation.OcctTypeNames.Of(t).StartsWith("Step", StringComparison.Ordinal) && typeof(Transient).IsAssignableFrom(t))
            .OrderBy(t => Kerf.Engine.Foundation.OcctTypeNames.Of(t), StringComparer.Ordinal)
            .Select(t => (t, aProtocol.TypeNumber(t)))
            .Where(x => x.Item2 > 0)
            .ToList();
    }

    [Fact]
    public void ProtocolIsOnePerProcess()
    {
        Assert.Same(global::Kerf.Engine.Exchange.Step.Ap214.StepAP214.Protocol(), global::Kerf.Engine.Exchange.Step.Ap214.StepAP214.Protocol());
        Assert.Equal(1, global::Kerf.Engine.Exchange.Step.Ap214.StepAP214.Protocol().ResourceCount);
        Assert.IsType<HeaderProtocol>(global::Kerf.Engine.Exchange.Step.Ap214.StepAP214.Protocol().Resource(1));
        // a second protocol shares the class map (OCCT's file static)
        Assert.Equal(59, new global::Kerf.Engine.Exchange.Step.Ap214.Ap214Protocol().TypeNumber(typeof(StepCartesianPoint)));
    }

    [Fact]
    public void SchemaNameFollowsTheModelsParameters()
    {
        var aProtocol = global::Kerf.Engine.Exchange.Step.Ap214.StepAP214.Protocol();
        Assert.Equal("AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }", aProtocol.SchemaName(null!));
        var aModel = new StepModel();
        aModel.InternalParameters.WriteSchema = Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203;
        Assert.Equal("CONFIG_CONTROL_DESIGN", aProtocol.SchemaName(aModel));
        aModel.InternalParameters.WriteSchema = Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP242DIS;
        Assert.Equal("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF {1 0 10303 442 1 1 4 }", aProtocol.SchemaName(aModel));
    }

    /// <summary>
    /// Where OCCT's own tables disagree (occt-issues.md, unit 8C2), by case: reproduced, so the test lists them.
    /// </summary>
    private static readonly Dictionary<int, string> OcctDisagrees = new()
    {
        // ComplexType gives the generic components (GEOMETRIC_TOLERANCE...), CaseStep wants a specific tolerance kind
        [694] = "components", [695] = "components", [696] = "components", [697] = "components", [705] = "components", [706] = "components",
        [800] = "components",
        // NewVoid creates a StepVisual_AnnotationOccurrence for DRAUGHTING_ANNOTATION_OCCURRENCE
        [106] = "NewVoid",
        // (825, 826 and 708, which OCCT couldn't name or read, are named since Kerf decision 16821)
    };

    [Fact]
    public void CasesAgreeAcrossTheProtocolAndModules()
    {
        var aGeneral = new Ap214GeneralModule();
        var aReadWrite = new Ap214ReadWriteModule();
        var aCases = Cases();
        Assert.Equal(713, aCases.Count); // StepAP214_Protocol binds 713 classes
        var aFailures = new List<string>();
        foreach (var (aType, aCase) in aCases)
        {
            if (OcctDisagrees.ContainsKey(aCase))
            {
                continue;
            }

            // the general module creates the protocol's class (or, for a few cases OCCT shares, one of its kind)
            if (!aGeneral.NewVoid(aCase, out var anEnt) || anEnt is null)
            {
                aFailures.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(aType)} ({aCase}): NewVoid creates nothing");
                continue;
            }

            if (anEnt.GetType() != aType)
            {
                aFailures.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(aType)} ({aCase}): NewVoid creates {Kerf.Engine.Foundation.OcctTypeNames.Of(anEnt.GetType())}");
            }

            // a simple type's name gives the case back
            if (!aReadWrite.IsComplex(aCase))
            {
                string aName = aReadWrite.StepType(aCase);
                if (aName.Length == 0 || aName == "?")
                {
                    aFailures.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(aType)} ({aCase}): no STEP type");
                }
                else if (aReadWrite.CaseStep(aName) != aCase)
                {
                    aFailures.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(aType)} ({aCase}): {aName} is case {aReadWrite.CaseStep(aName)}");
                }
            }
            else
            {
                // a complex type's components give the case back
                var aTypes = new NCollection_Sequence<string>();
                Assert.True(aReadWrite.ComplexType(aCase, aTypes), $"{Kerf.Engine.Foundation.OcctTypeNames.Of(aType)} ({aCase})");
                if (aReadWrite.CaseStep(aTypes) != aCase)
                {
                    aFailures.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(aType)} ({aCase}): its components are case {aReadWrite.CaseStep(aTypes)}");
                }
            }
        }

        Assert.True(aFailures.Count == 0, string.Join("\n", aFailures));

        // the disagreements are OCCT's
        Assert.True(aGeneral.NewVoid(106, out var aDraughting));
        Assert.IsType<AnnotationOccurrence>(aDraughting);
        // OCCT's 825, 826 and 708, named and read since Kerf decision 16821
        Assert.Equal("SURFACE_STYLE_REFLECTANCE_AMBIENT_DIFFUSE", aReadWrite.StepType(825));
        Assert.Equal(825, aReadWrite.CaseStep("SURFACE_STYLE_REFLECTANCE_AMBIENT_DIFFUSE"));
        Assert.Equal(826, aReadWrite.CaseStep(aReadWrite.StepType(826)));
        Assert.Equal(708, aReadWrite.CaseStep("TESSELLATED_ITEM"));
        var aTolerance = new NCollection_Sequence<string>();
        aReadWrite.ComplexType(694, aTolerance);
        Assert.Equal(0, aReadWrite.CaseStep(aTolerance));
    }

    [Fact]
    public void IsComplexListsEveryComplexCase()
    {
        // every case ComplexType gives components for is complex, 800 and 802 too (OCCT's IsComplex missed them, so
        // the writer wrote them as a record of type "?" followed by the components: occt-issues.md,
        // RWStepAP214_ReadWriteModule::IsComplex; Kerf decision 16819)
        var aReadWrite = new Ap214ReadWriteModule();
        for (int aCase = 1; aCase <= 900; aCase++)
        {
            Assert.Equal(aReadWrite.ComplexType(aCase, new NCollection_Sequence<string>()), aReadWrite.IsComplex(aCase));
        }
    }

    [Fact]
    public void ShortNamesAndCartesianPoint()
    {
        var aReadWrite = new Ap214ReadWriteModule();
        Assert.Equal(59, aReadWrite.CaseStep("CARTESIAN_POINT"));
        Assert.Equal(0, aReadWrite.CaseStep("NOT_A_STEP_TYPE"));
        // a short name (AP203's list) gives the long name's case
        Assert.Equal(aReadWrite.CaseStep("REPRESENTATION_ITEM"), aReadWrite.CaseStep("RPRITM"));
        // a complex type in any order, with short names converted
        var aTypes = new NCollection_Sequence<string>();
        aReadWrite.ComplexType(319, aTypes);
        var aReversed = new NCollection_Sequence<string>();
        for (int i = aTypes.Length(); i >= 1; i--)
        {
            aReversed.Append(aTypes.Value(i));
        }

        Assert.Equal(319, aReadWrite.CaseStep(aReversed));
    }

    [Fact]
    public void InitRegistersTheModulesOnce()
    {
        Ap214ReadWrite.Init();
        Ap214ReadWrite.Init();
        var aProtocol = global::Kerf.Engine.Exchange.Step.Ap214.StepAP214.Protocol();
        var aLib = new WriterLib(aProtocol);
        Assert.True(aLib.Select(new StepCartesianPoint(), out var aModule, out int aCase));
        Assert.IsType<Ap214ReadWriteModule>(aModule);
        Assert.Equal(59, aCase);
    }
}
