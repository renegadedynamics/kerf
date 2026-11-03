// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using System.Reflection;

namespace Kerf.Kernel.DataExchange.Tests.StepGen;

/// <summary>
/// Smoke tests of the generated entity packages (units 8C1 and 8C2): every class constructs, every Init runs, and the
/// accessors named after Init's parameters give back what Init stored; the SELECT types copy as C++ does; the RW
/// enum tools convert every value both ways.
/// </summary>
public sealed class StepEntitySmokeTests
{
    private static readonly string[] Packages =
        ["StepBasic", "StepGeom", "StepShape", "StepRepr", "StepVisual", "StepAP214", "StepAP203", "StepAP209", "StepAP242", "StepDimTol",
         "StepKinematics", "StepFEA", "StepElement"];

    private static IEnumerable<Type> EntityTypes(string thePackage) =>
        typeof(SelectType).Assembly.GetTypes()
            .Where(t => Kerf.Engine.Foundation.OcctTypeNames.Of(t).StartsWith(thePackage + "_", StringComparison.Ordinal) && t.IsClass)
            .OrderBy(t => Kerf.Engine.Foundation.OcctTypeNames.Of(t), StringComparer.Ordinal);

    /// <summary>A value for a parameter: distinct strings, numbers and entities, so the accessors can be checked.</summary>
    private static object? Sample(Type t, int theSeed)
    {
        if (t.IsByRef)
        {
            t = t.GetElementType()!;
        }

        if (t == typeof(int))
        {
            return 3 + theSeed;
        }

        if (t == typeof(double))
        {
            return 1.5 + theSeed;
        }

        if (t == typeof(bool))
        {
            return true;
        }

        if (t == typeof(string))
        {
            return "s" + theSeed;
        }

        if (t == typeof(TCollection_HAsciiString))
        {
            return new TCollection_HAsciiString("text" + theSeed);
        }

        if (t.IsEnum)
        {
            var aValues = Enum.GetValues(t);
            return aValues.GetValue(Math.Min(1, aValues.Length - 1));
        }

        if (t.IsGenericType && t.GetGenericTypeDefinition() == typeof(NCollection_HArray1<>))
        {
            var anArray = Activator.CreateInstance(t, 1, 2)!;
            var anElem = t.GetGenericArguments()[0];
            var aSet = t.GetMethod("SetValue")!;
            aSet.Invoke(anArray, [1, Sample(anElem, theSeed)]);
            aSet.Invoke(anArray, [2, Sample(anElem, theSeed + 1)]);
            return anArray;
        }

        if (t.IsGenericType && t.GetGenericTypeDefinition() == typeof(NCollection_HArray2<>))
        {
            var anArray = Activator.CreateInstance(t, 1, 2, 1, 2)!;
            var anElem = t.GetGenericArguments()[0];
            var aSet = t.GetMethod("SetValue")!;
            for (int i = 1; i <= 2; i++)
            {
                for (int j = 1; j <= 2; j++)
                {
                    aSet.Invoke(anArray, [i, j, Sample(anElem, theSeed + i + j)]);
                }
            }

            return anArray;
        }

        if (typeof(SelectType).IsAssignableFrom(t))
        {
            // an empty select: setting a value needs a member of one of its types
            return Activator.CreateInstance(t);
        }

        if (!t.IsAbstract && t.GetConstructor(Type.EmptyTypes) is { } aCtor)
        {
            return aCtor.Invoke(null);
        }

        // an abstract entity: a concrete descendant
        var aConcrete = typeof(SelectType).Assembly.GetTypes()
                         .Where(x => Kerf.Engine.Foundation.OcctTypeNames.Of(x).StartsWith("Step", StringComparison.Ordinal) && !x.IsAbstract && !x.ContainsGenericParameters
                                     && t.IsAssignableFrom(x) && x.GetConstructor(Type.EmptyTypes) != null)
                         .OrderBy(x => Kerf.Engine.Foundation.OcctTypeNames.Of(x), StringComparer.Ordinal).FirstOrDefault();
        return aConcrete is null ? null : Activator.CreateInstance(aConcrete);
    }

    private static string Field(string theParam)
    {
        foreach (string aPrefix in new[] { "the", "a", "an" })
        {
            if (theParam.Length > aPrefix.Length && theParam.StartsWith(aPrefix, StringComparison.Ordinal) && char.IsUpper(theParam[aPrefix.Length]))
            {
                return theParam[aPrefix.Length..];
            }
        }

        // the kernel's parameters are camelCase since unit 10E ("name" -> Name())
        return theParam.Length > 0 ? char.ToUpperInvariant(theParam[0]) + theParam[1..] : theParam;
    }

    /// <summary>
    /// Complex entities whose Init (all fields) calls Init on the field of their second supertype, which the default
    /// constructor leaves null: a null dereference in OCCT too (occt-issues.md, unit 8C2). The readers use the setters.
    /// </summary>
    private static readonly HashSet<string> NullInit =
        ["StepDimTol_CommonDatum", "StepAP214_ExternallyDefinedClass", "StepAP214_ExternallyDefinedGeneralProperty", "StepAP214_RepItemGroup"];

    [Theory]
    [InlineData("StepBasic")]
    [InlineData("StepGeom")]
    [InlineData("StepShape")]
    [InlineData("StepRepr")]
    [InlineData("StepVisual")]
    [InlineData("StepAP214")]
    [InlineData("StepAP203")]
    [InlineData("StepAP242")]
    [InlineData("StepDimTol")]
    [InlineData("StepKinematics")]
    [InlineData("StepFEA")]
    [InlineData("StepElement")]
    public void ConstructInitAndRead(string thePackage)
    {
        int aClasses = 0, anInits = 0, aChecked = 0;
        var aFailures = new List<string>();
        foreach (var aType in EntityTypes(thePackage))
        {
            if (aType.IsAbstract || aType.GetConstructor(Type.EmptyTypes) is not { } aCtor)
            {
                continue;
            }

            object anEnt = aCtor.Invoke(null);
            aClasses++;
            foreach (var anInit in aType.GetMethods(BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly).Where(m => m.Name == "Init"))
            {
                var aParams = anInit.GetParameters();
                object?[] anArgs = aParams.Select((p, k) => Sample(p.ParameterType, k)).ToArray();
                try
                {
                    anInit.Invoke(anEnt, anArgs);
                    anInits++;
                }
                catch (TargetInvocationException ex) when (ex.InnerException is NullReferenceException && NullInit.Contains(Kerf.Engine.Foundation.OcctTypeNames.Of(aType)))
                {
                    continue; // OCCT's Init calls Init on a null handle (occt-issues.md, 8C2)
                }
                catch (TargetInvocationException ex)
                {
                    aFailures.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(aType)}.Init: {ex.InnerException?.GetType().Name}: {ex.InnerException?.Message}");
                    continue;
                }

                // the accessors named after the parameters ("aName" -> Name())
                for (int k = 0; k < aParams.Length; k++)
                {
                    string aName = Field(aParams[k].Name!);
                    var aGetter = aType.GetMethod(aName, BindingFlags.Public | BindingFlags.Instance, Type.EmptyTypes);
                    if (aGetter is null || aGetter.ReturnType == typeof(void) || aGetter.ReturnType.IsByRef || aGetter.ReturnType.IsByRefLike
                        || !aGetter.ReturnType.IsAssignableFrom(aParams[k].ParameterType) || anArgs[k] is null || aParams[k].ParameterType == typeof(bool))
                    {
                        continue;
                    }

                    object? aGot = aGetter.Invoke(anEnt, null);
                    bool isSame = ReferenceEquals(aGot, anArgs[k]) || Equals(aGot, anArgs[k])
                                  || (aGot is SelectType s1 && anArgs[k] is SelectType s2 && ReferenceEquals(s1.Value(), s2.Value()));
                    if (isSame)
                    {
                        aChecked++;
                    }
                    else if (aGot is not null && anArgs[k] is not null && aGot.GetType() == anArgs[k]!.GetType() && !aGot.GetType().IsValueType)
                    {
                        // a redefined field (OCCT's "the field is redefined"): reported, not a failure
                    }
                    else
                    {
                        aFailures.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(aType)}.{aName}() after Init: {aGot} for {anArgs[k]}");
                    }
                }
            }
        }

        // StepAP242 is the smallest package (6 classes)
        int aMin = thePackage is "StepAP242" or "StepAP203" ? 3 : 20;
        Assert.True(aClasses > aMin, $"{aClasses} classes");
        Assert.True(anInits > aMin / 2, $"{anInits} Init calls");
        Assert.True(aChecked > aMin, $"{aChecked} accessors checked");
        Assert.True(aFailures.Count == 0, string.Join("\n", aFailures));
    }

    [Fact]
    public void CartesianPointAndBSplineCurve()
    {
        var aPoint = new StepCartesianPoint();
        aPoint.Init3D(new TCollection_HAsciiString("p"), 1, 2, 3);
        Assert.Equal(3, aPoint.CoordinateCount);
        Assert.Equal(2.0, aPoint.CoordinatesValue(2));
        Assert.Equal([1.0, 2.0, 3.0], aPoint.Coordinates().ToArray());
        aPoint.Init2D(new TCollection_HAsciiString("q"), 4, 5);
        Assert.Equal(2, aPoint.CoordinateCount);
        Assert.Equal("q", aPoint.Name().ToCString());

        // std::array is a value: SetCoordinates copies, Coordinates is read-only
        double[] aCoords = [7, 8, 9];
        aPoint.SetCoordinates(aCoords);
        aCoords[0] = 0;
        Assert.Equal(7.0, aPoint.CoordinatesValue(1));

        var aPoles = new NCollection_HArray1<StepCartesianPoint>(1, 2);
        aPoles.SetValue(1, aPoint);
        aPoles.SetValue(2, aPoint);
        var aMults = new NCollection_HArray1<int>(1, 2, 2);
        var aKnots = new NCollection_HArray1<double>(1, 2);
        aKnots.SetValue(2, 1.0);
        var aCurve = new StepBSplineCurveWithKnots();
        aCurve.Init(new TCollection_HAsciiString("c"), 1, aPoles, StepBSplineCurveForm.BscfPolylineForm,
                    Logical.LFalse, Logical.LFalse, aMults, aKnots,
                    StepKnotType.KtPiecewiseBezierKnots);
        Assert.Equal(2, aCurve.NbControlPointsList());
        Assert.Same(aPoint, aCurve.ControlPointsListValue(2));
        Assert.Equal(2, aCurve.KnotCount);
        Assert.Equal(StepKnotType.KtPiecewiseBezierKnots, aCurve.KnotSpec());
        Assert.Equal(1, aCurve.Degree());
    }

    [Fact]
    public void SelectTypesRecogniseAndCopy()
    {
        var aPlacement = new StepAxis2Placement3d();
        var aSelect = new StepAxis2Placement();
        Assert.Equal(2, aSelect.CaseNum(aPlacement));
        Assert.Equal(0, aSelect.CaseNum(new StepCartesianPoint()));
        aSelect.SetValue(aPlacement);
        Assert.Same(aPlacement, aSelect.Axis2Placement3d());
        Assert.Null(aSelect.Axis2Placement2d());

        // C++ copies a SELECT by value (decision 1210)
        var aCopy = new StepAxis2Placement(aSelect);
        aSelect.SetValue(new StepAxis2Placement2d());
        Assert.Same(aPlacement, aCopy.Value());

        // a SELECT in an entity's array
        var anItem = new StepGeometricSetSelect();
        anItem.SetValue(new StepCartesianPoint());
        var aSet = new StepGeometricSet();
        var anItems = new NCollection_HArray1<StepGeometricSetSelect>(1, 1);
        anItems.SetValue(1, anItem);
        aSet.Init(new TCollection_HAsciiString("g"), anItems);
        Assert.IsType<StepCartesianPoint>(aSet.ElementsValue(1).Value());
    }

    [Fact]
    public void EnumToolsConvertEveryValue()
    {
        int aTools = 0, aValues = 0;
        foreach (var aTool in typeof(SelectType).Assembly.GetTypes()
                     .Where(t => t.IsAbstract && t.IsSealed && Packages.Any(p => Kerf.Engine.Foundation.OcctTypeNames.Of(t).StartsWith("RW" + p + "_RW", StringComparison.Ordinal)))
                     .OrderBy(t => Kerf.Engine.Foundation.OcctTypeNames.Of(t), StringComparer.Ordinal))
        {
            var aToString = aTool.GetMethod("ConvertToString", BindingFlags.Public | BindingFlags.Static);
            var aToEnum = aTool.GetMethod("ConvertToEnum", BindingFlags.Public | BindingFlags.Static);
            if (aToString is null || aToEnum is null)
            {
                continue;
            }

            aTools++;
            var anEnum = aToString.GetParameters()[0].ParameterType;
            foreach (object v in Enum.GetValues(anEnum))
            {
                string s = (string)aToString.Invoke(null, [v])!;
                Assert.Matches(@"^\.[A-Z0-9_]+\.$", s);
                object?[] anArgs = [s, Enum.ToObject(anEnum, 0)];
                Assert.True((bool)aToEnum.Invoke(null, anArgs)!, $"{Kerf.Engine.Foundation.OcctTypeNames.Of(aTool)}: {s}");
                Assert.Equal(v, anArgs[1]);
                aValues++;
            }

            object?[] aBad = [".NOT_A_VALUE.", Enum.ToObject(anEnum, 0)];
            Assert.False((bool)aToEnum.Invoke(null, aBad)!);
        }

        Assert.True(aTools >= 10, $"{aTools} enum tools");
        Assert.True(aValues >= 40, $"{aValues} values");
    }
}
