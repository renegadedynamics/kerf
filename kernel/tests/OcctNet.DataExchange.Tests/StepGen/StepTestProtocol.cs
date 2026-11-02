// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using System.Text.RegularExpressions;

namespace OcctNet.DataExchange.Tests.StepGen;

/// <summary>
/// A test protocol for the generated packages (unit 8C1), standing in for StepAP214_Protocol and its modules (unit
/// 8C2): the STEP type names, case numbers, entity classes and RW tools are read from OCCT's own
/// RWStepAP214_ReadWriteModule.cxx, and the modules dispatch to the generated RW tools by reflection. Only the
/// classes of the packages given are recognised; the other records are read as unknown entities.
/// </summary>
internal sealed class StepTestTable
{
    public readonly Dictionary<string, int> CaseOfType = new(StringComparer.Ordinal);
    public readonly Dictionary<int, string> TypeOfCase = [];
    public readonly Dictionary<string, int> CaseOfComplex = new(StringComparer.Ordinal);
    public readonly Dictionary<int, List<string>> ComplexOfCase = [];
    public readonly Dictionary<int, Type> EntityOfCase = [];
    public readonly Dictionary<Type, int> CaseOfEntity = [];
    public readonly Dictionary<int, object> ToolOfCase = [];
    public readonly Dictionary<int, (MethodInfo? Read, MethodInfo? Write, MethodInfo? Share, MethodInfo? Check)> Methods = [];

    public static StepTestTable? Load(string theOcctSource, IReadOnlyCollection<string> thePackages)
    {
        string aPath = Path.Combine(theOcctSource, "src", "DataExchange", "TKDESTEP", "RWStepAP214", "RWStepAP214_ReadWriteModule.cxx");
        if (!File.Exists(aPath))
        {
            return null;
        }

        string aText = File.ReadAllText(aPath);
        var aReco = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (Match m in Regex.Matches(aText, @"static constexpr std::string_view (\w+)\(\s*""([^""]*)""\s*\)"))
        {
            aReco[m.Groups[1].Value] = m.Groups[2].Value;
        }

        var aTable = new StepTestTable();
        var aNames = new Dictionary<int, string>();
        foreach (Match m in Regex.Matches(aText, @"theTypenums\.Bind\((\w+),\s*(\d+)\)"))
        {
            aNames[int.Parse(m.Groups[2].Value)] = aReco[m.Groups[1].Value];
        }

        // StepType's switch names the cases the map doesn't (components of complex types)
        foreach (Match m in Regex.Matches(aText, @"case (\d+):\s*return (Reco_\w+);"))
        {
            aNames.TryAdd(int.Parse(m.Groups[1].Value), aReco[m.Groups[2].Value]);
        }

        var aShort = new List<(string, int)>();
        foreach (Match m in Regex.Matches(aText, @"theTypeshor\.Bind\((\w+),\s*(\d+)\)"))
        {
            aShort.Add((aReco[m.Groups[1].Value], int.Parse(m.Groups[2].Value)));
        }

        // StepAP214_Protocol's map of classes to cases (one case per class; the writer's type)
        string aProtocolPath = Path.Combine(theOcctSource, "src", "DataExchange", "TKDESTEP", "StepAP214", "StepAP214_Protocol.cxx");
        var aClassOfCase = new Dictionary<int, string>();
        foreach (Match m in Regex.Matches(File.ReadAllText(aProtocolPath), @"^\s*types\.Bind\(STANDARD_TYPE\((\w+)\),\s*(\d+)\);", RegexOptions.Multiline))
        {
            aClassOfCase[int.Parse(m.Groups[2].Value)] = m.Groups[1].Value;
        }

        // ReadStep's cases: the RW tool (its entity may be a base of the protocol's class)
        int aRead = aText.IndexOf("::ReadStep(", StringComparison.Ordinal);
        int aWrite = aText.IndexOf("::WriteStep(", aRead, StringComparison.Ordinal);
        string aReadText = aText[aRead..aWrite];
        var anAssembly = typeof(StepData_SelectType).Assembly;
        foreach (Match m in Regex.Matches(aReadText, @"case (\d+):\s*(?://[^\n]*\s*)?\{\s*(?://[^\n]*\s*)?DeclareAndCast\((\w+),\s*\w+,\s*\w+\);\s*(RWStep\w+)\s+tool;"))
        {
            int aCase = int.Parse(m.Groups[1].Value);
            string anEntity = aClassOfCase.GetValueOrDefault(aCase, m.Groups[2].Value), aTool = m.Groups[3].Value;
            if (!thePackages.Contains(anEntity[..anEntity.IndexOf('_')]))
            {
                continue;
            }

            var anEntityType = anAssembly.GetType("Occt." + anEntity);
            var aToolType = anAssembly.GetType("Occt." + aTool);
            if (anEntityType is null || aToolType is null || aTable.EntityOfCase.ContainsKey(aCase))
            {
                continue;
            }

            aTable.EntityOfCase[aCase] = anEntityType;
            if (aClassOfCase.ContainsKey(aCase))
            {
                aTable.CaseOfEntity[anEntityType] = aCase;
            }
            aTable.ToolOfCase[aCase] = Activator.CreateInstance(aToolType)!;
            aTable.Methods[aCase] = (aToolType.GetMethod("ReadStep"), aToolType.GetMethod("WriteStep"), aToolType.GetMethod("Share"), aToolType.GetMethod("Check"));
        }

        foreach (var (aCase, aName) in aNames)
        {
            aTable.TypeOfCase[aCase] = aName;
            if (aTable.EntityOfCase.ContainsKey(aCase))
            {
                aTable.CaseOfType[aName] = aCase;
            }
        }

        foreach (var (aName, aCase) in aShort)
        {
            if (aTable.EntityOfCase.ContainsKey(aCase))
            {
                aTable.CaseOfType.TryAdd(aName, aCase);
            }
        }

        // complex types: ComplexType's lists, matched sorted (as CaseStep sorts the record's types)
        int aComplex = aText.IndexOf("::ComplexType(", StringComparison.Ordinal);
        int aComplexEnd = aText.IndexOf("return true;", aComplex, StringComparison.Ordinal);
        foreach (Match m in Regex.Matches(aText[aComplex..aComplexEnd], @"case (\d+):((?:\s*types\.Append\(TCollection_AsciiString\(StepType\(\d+\)\)\);)+)"))
        {
            int aCase = int.Parse(m.Groups[1].Value);
            var aNums = Regex.Matches(m.Groups[2].Value, @"StepType\((\d+)\)").Select(x => int.Parse(x.Groups[1].Value)).ToList();
            if (aNums.Any(n => !aNames.ContainsKey(n)))
            {
                continue;
            }

            var aParts = aNums.Select(n => aNames[n]).ToList();
            aTable.ComplexOfCase[aCase] = aParts;
            if (aTable.EntityOfCase.ContainsKey(aCase))
            {
                aTable.CaseOfComplex[string.Join(",", aParts.Order(StringComparer.Ordinal))] = aCase;
            }
        }

        return aTable;
    }
}

internal sealed class StepTestReadWriteModule(StepTestTable theTable) : StepData_ReadWriteModule
{
    public override int CaseStep(string atype) => theTable.CaseOfType.GetValueOrDefault(atype);

    public override int CaseStep(NCollection_Sequence<string> types)
    {
        var aNames = new List<string>();
        for (int i = 1; i <= types.Length(); i++)
        {
            string aName = types.Value(i);
            // short names to long ones, as RWStepAP214_ReadWriteModule::CaseStep does
            if (theTable.CaseOfType.TryGetValue(aName, out int aShort) && theTable.TypeOfCase.TryGetValue(aShort, out string? aLong))
            {
                aName = aLong;
            }

            aNames.Add(aName);
        }

        return theTable.CaseOfComplex.GetValueOrDefault(string.Join(",", aNames.Order(StringComparer.Ordinal)));
    }

    public override bool IsComplex(int CN) => theTable.ComplexOfCase.ContainsKey(CN);

    public override string StepType(int CN) => theTable.ComplexOfCase.ContainsKey(CN) ? "" : theTable.TypeOfCase.GetValueOrDefault(CN, "");

    public override bool ComplexType(int CN, NCollection_Sequence<string> types)
    {
        if (!theTable.ComplexOfCase.TryGetValue(CN, out var aParts))
        {
            return false;
        }

        foreach (string p in aParts)
        {
            types.Append(p);
        }

        return true;
    }

    public override void ReadStep(int CN, StepData_StepReaderData data, int num, Interface_Check ach, Standard_Transient? ent)
    {
        if (theTable.Methods.TryGetValue(CN, out var m) && m.Read is not null)
        {
            Invoke(m.Read, theTable.ToolOfCase[CN], [data, num, ach, ent]);
        }
    }

    public override void WriteStep(int CN, StepData_StepWriter SW, Standard_Transient? ent)
    {
        if (theTable.Methods.TryGetValue(CN, out var m) && m.Write is not null)
        {
            Invoke(m.Write, theTable.ToolOfCase[CN], [SW, ent]);
        }
    }

    internal static void Invoke(MethodInfo theMethod, object theTool, object?[] theArgs)
    {
        try
        {
            theMethod.Invoke(theTool, theArgs);
        }
        catch (TargetInvocationException ex) when (ex.InnerException is not null)
        {
            System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(ex.InnerException).Throw();
        }
    }
}

internal sealed class StepTestGeneralModule(StepTestTable theTable) : StepData_GeneralModule
{
    public override void FillSharedCase(int casenum, Standard_Transient? ent, Interface_EntityIterator iter)
    {
        if (theTable.Methods.TryGetValue(casenum, out var m) && m.Share is not null)
        {
            StepTestReadWriteModule.Invoke(m.Share, theTable.ToolOfCase[casenum], [ent, iter]);
        }
    }

    public override void CheckCase(int casenum, Standard_Transient? ent, Interface_ShareTool shares, ref Interface_Check? ach)
    {
        if (theTable.Methods.TryGetValue(casenum, out var m) && m.Check is not null)
        {
            StepTestReadWriteModule.Invoke(m.Check, theTable.ToolOfCase[casenum], [ent, shares, ach]);
        }
    }

    public override bool NewVoid(int CN, out Standard_Transient? entto)
    {
        entto = theTable.EntityOfCase.TryGetValue(CN, out var t) ? (Standard_Transient?)Activator.CreateInstance(t) : null;
        return entto is not null;
    }

    public override void CopyCase(int casenum, Standard_Transient? entfrom, Standard_Transient? entto, Interface_CopyTool TC)
    {
    }
}

internal sealed class StepTestProtocol(StepTestTable theTable) : StepData_Protocol
{
    public StepTestTable Table => theTable;

    public override int TypeNumber(Type? atype) => atype is not null && theTable.CaseOfEntity.TryGetValue(atype, out int n) ? n : 0;

    public override string SchemaName(Interface_InterfaceModel? theModel) => "AUTOMOTIVE_DESIGN";
}
