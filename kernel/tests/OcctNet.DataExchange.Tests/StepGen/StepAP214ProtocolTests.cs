// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DataExchange.Tests.StepGen;

/// <summary>
/// StepAP214's protocol and its two modules (unit 8C2): the protocol's case numbers, the general module's empty
/// entities and the read-write module's type names agree with each other, as OCCT's dispatch tables do.
/// </summary>
public sealed class StepAP214ProtocolTests
{
    /// <summary>Every entity class of the generated packages with the protocol's case number for it.</summary>
    private static List<(Type Type, int Case)> Cases()
    {
        var aProtocol = StepAP214.Protocol();
        return typeof(StepAP214_Protocol).Assembly.GetTypes()
            .Where(t => t.Namespace == "Occt" && t.Name.StartsWith("Step", StringComparison.Ordinal) && typeof(Standard_Transient).IsAssignableFrom(t))
            .OrderBy(t => t.Name, StringComparer.Ordinal)
            .Select(t => (t, aProtocol.TypeNumber(t)))
            .Where(x => x.Item2 > 0)
            .ToList();
    }

    [Fact]
    public void ProtocolIsOnePerProcess()
    {
        Assert.Same(StepAP214.Protocol(), StepAP214.Protocol());
        Assert.Equal(1, StepAP214.Protocol().NbResources());
        Assert.IsType<HeaderSection_Protocol>(StepAP214.Protocol().Resource(1));
        // a second protocol shares the class map (OCCT's file static)
        Assert.Equal(59, new StepAP214_Protocol().TypeNumber(typeof(StepGeom_CartesianPoint)));
    }

    [Fact]
    public void SchemaNameFollowsTheModelsParameters()
    {
        var aProtocol = StepAP214.Protocol();
        Assert.Equal("AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }", aProtocol.SchemaName(null!));
        var aModel = new StepData_StepModel();
        aModel.InternalParameters.WriteSchema = DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203;
        Assert.Equal("CONFIG_CONTROL_DESIGN", aProtocol.SchemaName(aModel));
        aModel.InternalParameters.WriteSchema = DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP242DIS;
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
        // no type name: SURFACE_STYLE_REFLECTANCE_AMBIENT_DIFFUSE(_SPECULAR) can be neither read nor written
        [825] = "name", [826] = "name",
        // TESSELLATED_ITEM is named by StepType but not in the type map: it can be written, not read
        [708] = "name",
    };

    [Fact]
    public void CasesAgreeAcrossTheProtocolAndModules()
    {
        var aGeneral = new RWStepAP214_GeneralModule();
        var aReadWrite = new RWStepAP214_ReadWriteModule();
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
                aFailures.Add($"{aType.Name} ({aCase}): NewVoid creates nothing");
                continue;
            }

            if (anEnt.GetType() != aType)
            {
                aFailures.Add($"{aType.Name} ({aCase}): NewVoid creates {anEnt.GetType().Name}");
            }

            // a simple type's name gives the case back
            if (!aReadWrite.IsComplex(aCase) && aCase is not (800 or 802))
            {
                string aName = aReadWrite.StepType(aCase);
                if (aName.Length == 0 || aName == "?")
                {
                    aFailures.Add($"{aType.Name} ({aCase}): no STEP type");
                }
                else if (aReadWrite.CaseStep(aName) != aCase)
                {
                    aFailures.Add($"{aType.Name} ({aCase}): {aName} is case {aReadWrite.CaseStep(aName)}");
                }
            }
            else
            {
                // a complex type's components give the case back
                var aTypes = new NCollection_Sequence<string>();
                Assert.True(aReadWrite.ComplexType(aCase, aTypes), $"{aType.Name} ({aCase})");
                if (aReadWrite.CaseStep(aTypes) != aCase)
                {
                    aFailures.Add($"{aType.Name} ({aCase}): its components are case {aReadWrite.CaseStep(aTypes)}");
                }
            }
        }

        Assert.True(aFailures.Count == 0, string.Join("\n", aFailures));

        // the disagreements are OCCT's
        Assert.True(aGeneral.NewVoid(106, out var aDraughting));
        Assert.IsType<StepVisual_AnnotationOccurrence>(aDraughting);
        Assert.Equal("?", aReadWrite.StepType(825));
        Assert.Equal(0, aReadWrite.CaseStep("TESSELLATED_ITEM"));
        var aTolerance = new NCollection_Sequence<string>();
        aReadWrite.ComplexType(694, aTolerance);
        Assert.Equal(0, aReadWrite.CaseStep(aTolerance));
    }

    [Fact]
    public void IsComplexMissesTwoComplexCasesAsOcctDoes()
    {
        // OCCT's IsComplex lists every case of ComplexType but 800 and 802 (occt-issues.md, unit 8C2): the writer
        // writes them as a simple record of type "?" followed by the components
        var aReadWrite = new RWStepAP214_ReadWriteModule();
        foreach (int aCase in new[] { 800, 802 })
        {
            Assert.False(aReadWrite.IsComplex(aCase));
            Assert.True(aReadWrite.ComplexType(aCase, new NCollection_Sequence<string>()));
            Assert.Equal("?", aReadWrite.StepType(aCase));
        }

        Assert.True(aReadWrite.IsComplex(319));
    }

    [Fact]
    public void ShortNamesAndCartesianPoint()
    {
        var aReadWrite = new RWStepAP214_ReadWriteModule();
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
        RWStepAP214.Init();
        RWStepAP214.Init();
        var aProtocol = StepAP214.Protocol();
        var aLib = new StepData_WriterLib(aProtocol);
        Assert.True(aLib.Select(new StepGeom_CartesianPoint(), out var aModule, out int aCase));
        Assert.IsType<RWStepAP214_ReadWriteModule>(aModule);
        Assert.Equal(59, aCase);
    }
}
