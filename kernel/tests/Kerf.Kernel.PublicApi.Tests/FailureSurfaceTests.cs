// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit;

namespace Kerf.Tests;

/// <summary>
/// The public failure surface (unit 10P2): an expected failure is a result, misuse is .NET's exception, and no exception
/// of the engine's reaches a caller, who couldn't name it.
/// </summary>
public class FailureSurfaceTests
{
    private static readonly Assembly[] KernelAssemblies =
    [
        typeof(Point3D).Assembly, typeof(Shape).Assembly, typeof(Primitives).Assembly, typeof(Kerf.Exchange.Step).Assembly,
    ];

    private static readonly Type FailureType =
        typeof(Point3D).Assembly.GetType("Kerf.Engine.Foundation.FailureException", throwOnError: true)!;

    private static IEnumerable<Type> PublicTypes => KernelAssemblies.SelectMany(a => a.GetTypes()).Where(t =>
        t.IsVisible && t.Namespace is { } ns && (ns == "Kerf" || ns.StartsWith("Kerf.", StringComparison.Ordinal))
        && !ns.StartsWith("Kerf.Engine", StringComparison.Ordinal) && !ns.StartsWith("Kerf.Kernel", StringComparison.Ordinal));

    [Fact]
    public void A_zero_direction_is_an_argument_exception()
    {
        var e = Assert.Throws<ArgumentException>(() => new Direction3D(0, 0, 0));
        Assert.DoesNotContain("gp_", e.Message);
        Assert.Throws<ArgumentException>(() => new Direction2D(0, 0));
    }

    [Fact]
    public void Each_kind_of_misuse_is_the_exception_dotnet_would_throw()
    {
        Assert.Throws<ArgumentException>(() => new Circle3D(Frame3D.World, -1));
        Assert.Throws<InvalidOperationException>(() => new Vector3D(0, 0, 0).Normalized());
        Assert.Throws<InvalidOperationException>(() => default(Shape).Type);
        Assert.Throws<ArgumentException>(() => new TrimmedCurve3D(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), double.NaN, 1));
    }

    /// <summary>Every guard in a public type's own code goes through Refusal: its failure never leaves as itself.</summary>
    [Fact]
    public void No_public_type_throws_a_failure_of_the_engine_itself()
    {
        var found = new List<string>();
        var ops = typeof(OpCodes).GetFields(BindingFlags.Public | BindingFlags.Static).Select(f => (OpCode)f.GetValue(null)!).ToDictionary(o => o.Value);
        foreach (Type type in PublicTypes)
        {
            foreach (MethodBase method in type.GetMethods(All).Cast<MethodBase>().Concat(type.GetConstructors(All)))
            {
                if (method.GetMethodBody()?.GetILAsByteArray() is not { } il)
                    continue;
                Type[]? typeArgs = type.IsGenericType ? type.GetGenericArguments() : null;
                Type[]? methodArgs = method is MethodInfo { IsGenericMethod: true } generic ? generic.GetGenericArguments() : null;
                int pendingFailure = 0;
                for (int at = 0; at < il.Length;)
                {
                    short code = il[at++];
                    if (code == 0xFE)
                        code = (short)(0xFE00 | il[at++]);
                    OpCode op = ops[code];
                    if (op.OperandType == OperandType.InlineMethod)
                    {
                        MethodBase called = method.Module.ResolveMethod(BitConverter.ToInt32(il, at), typeArgs, methodArgs)!;
                        if (op == OpCodes.Newobj && FailureType.IsAssignableFrom(called.DeclaringType))
                            pendingFailure++;
                        else if (called.DeclaringType?.Name == "Refusal" && pendingFailure > 0)
                            pendingFailure--;
                        else if (called.Name.EndsWith("_Raise_if", StringComparison.Ordinal))
                            found.Add($"{type.FullName}.{method.Name} calls {called.Name}");
                    }
                    if (op == OpCodes.Throw && pendingFailure > 0)
                    {
                        found.Add($"{type.FullName}.{method.Name} throws a failure of the engine");
                        pendingFailure = 0;
                    }
                    at += op.OperandType switch
                    {
                        OperandType.InlineNone => 0,
                        OperandType.ShortInlineBrTarget or OperandType.ShortInlineI or OperandType.ShortInlineVar => 1,
                        OperandType.InlineVar => 2,
                        OperandType.InlineI8 or OperandType.InlineR => 8,
                        OperandType.InlineSwitch => 4 + (4 * BitConverter.ToInt32(il, at)),
                        _ => 4,
                    };
                }
            }
        }
        Assert.True(found.Count == 0, string.Join("\n", found));
    }

    /// <summary>
    /// Calls every public member of the public types on sample objects, with awkward arguments one at a time (zeros,
    /// negatives, NaN, far past any index, nulls, default values): whatever it throws must be a type a caller can name.
    /// </summary>
    [Fact]
    public void No_public_member_lets_an_exception_of_the_engine_escape()
    {
        var samples = new Samples();
        var escapes = new ConcurrentDictionary<string, string>();
        foreach (Type type in PublicTypes)
        {
            if (!type.IsGenericTypeDefinition && !Skipped(type))
                samples.Exercise(type, escapes);
        }
        Assert.True(escapes.IsEmpty, $"{escapes.Count} escapes:\n" + string.Join("\n", escapes.OrderBy(e => e.Key).Select(e => $"{e.Key}: {e.Value}")));
    }

    private const BindingFlags All = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;

    // Writing to the machine or running for long on nonsense isn't what this test is about: it isn't asking about the
    // operations' numerics, which run on EngineOperation.Run
    private static bool Skipped(Type type) => type.Namespace is "Kerf.Analyzers" || typeof(Delegate).IsAssignableFrom(type) || type.Name.StartsWith('<');

    private sealed class Samples
    {
        private readonly Dictionary<Type, List<object?>> _byType = [];

        public Samples()
        {
            var points = new[] { new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 1) };
            Solid box = Solid.Box(1, 2, 3).Value.Shape;
            var circle = new CircleCurve3D(Frame3D.World, 2);
            var plane = new PlaneSurface(new Plane3D(new CoordinateSystem(Frame3D.World)));
            var bezier2D = new BezierCurve2D([new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)]);
            var bezierSurface = new BezierSurface(new Grid<Point3D>(new Point3D[] { new(0, 0, 0), new(0, 1, 0), new(1, 0, 0), new(1, 1, 0) }, 2, 2));
            var seeds = new object?[]
            {
                new Point3D(1, 2, 3), new Vector3D(1, 2, 3), new Direction3D(0, 0, 1), new Point2D(1, 2), new Vector2D(1, 2), new Direction2D(1, 0),
                Axis3D.ZAxis, Frame3D.World, Transform3D.Identity, Transform2D.Identity,
                new Circle3D(Frame3D.World, 2), new Ellipse3D(Frame3D.World, 3, 2), new Hyperbola3D(Frame3D.World, 3, 2), new Parabola3D(Frame3D.World, 2),
                new Plane3D(new CoordinateSystem(Frame3D.World)), new Cylinder3D(new CoordinateSystem(Frame3D.World), 2), new Sphere3D(new CoordinateSystem(Frame3D.World), 2), new Torus3D(new CoordinateSystem(Frame3D.World), 3, 1), new Cone3D(new CoordinateSystem(Frame3D.World), 0.3, 2),
                new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), new CircleCurve3D(Frame3D.World, 2),
                BSplineCurve3D.Interpolate(points).Value,
                new PlaneSurface(new Plane3D(new CoordinateSystem(Frame3D.World))), new CylindricalSurface(new CoordinateSystem(Frame3D.World), 2),
                box, (Shape)box, box.AllFaces()[0], box.AllEdges()[0], box.AllVertices()[0], box.AllWires()[0], box.AllShells()[0],
                points, new[] { box.AllEdges()[0] }, new[] { box.AllFaces()[0] }, new[] { 1.0, 2.0 }, new[] { 1, 2 },
                new EllipseCurve3D(Frame3D.World, 3, 2), new HyperbolaCurve3D(Frame3D.World, 3, 2), new ParabolaCurve3D(Frame3D.World, 2),
                new BezierCurve3D(points), new TrimmedCurve3D(circle, 0, 1), new OffsetCurve3D(circle, 1, new Direction3D(0, 0, 1)),
                new LineCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0))), new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2), new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3, 2), new HyperbolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3, 2), new ParabolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2),
                bezier2D, new BSplineCurve2D([new Point2D(0, 0), new Point2D(1, 1)], [0, 1], [2, 2], 1), new TrimmedCurve2D(bezier2D, 0.1, 0.9), new OffsetCurve2D(bezier2D, 0.1),
                new ConicalSurface(new CoordinateSystem(Frame3D.World), 0.3, 2), new SphericalSurface(new CoordinateSystem(Frame3D.World), 2), new ToroidalSurface(new CoordinateSystem(Frame3D.World), 3, 1), bezierSurface,
                new ExtrusionSurface(circle, new Direction3D(0, 0, 1)), new RevolutionSurface(new LineCurve3D(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), Axis3D.ZAxis),
                new OffsetSurface(plane, 1), new TrimmedSurface(plane, 0, 1, 0, 1),
                Kerf.Meshing.Mesher.Mesh(box).Value, Kerf.Exchange.ModelNode.Part(box),
                "text", 1.0, 1, true,
            };
            foreach (object? seed in seeds)
                Add(seed);
            foreach (Type type in PublicTypes.Where(t => !t.IsAbstract && !t.ContainsGenericParameters && t.GetConstructor(Type.EmptyTypes) is not null))
                Add(Activator.CreateInstance(type));
            Harvest();
            Harvest();
        }

        private void Add(object? value)
        {
            if (value is null)
                return;
            if (!_byType.TryGetValue(value.GetType(), out var list))
                _byType[value.GetType()] = list = [];
            if (list.Count < 2)
                list.Add(value);
        }

        // What the seeds' properties and the methods without arguments give: the evaluators, boxes, meshes, histories
        private void Harvest()
        {
            foreach (var pair in _byType.ToArray())
            {
                foreach (object? receiver in pair.Value.ToArray())
                {
                    foreach (MethodInfo method in pair.Key.GetMethods(BindingFlags.Public | BindingFlags.Instance))
                    {
                        if (method.GetParameters().Length != 0 || method.ReturnType == typeof(void) || method.IsGenericMethodDefinition
                            || method.ReturnType.IsByRefLike || method.DeclaringType == typeof(object))
                            continue;
                        try
                        {
                            Add(method.Invoke(receiver, null));
                        }
                        catch (TargetInvocationException)
                        {
                        }
                    }
                }
            }
        }

        private IEnumerable<object?> Good(Type type)
        {
            if (type == typeof(CancellationToken))
                return [default(CancellationToken)];
            if (_byType.TryGetValue(type, out var exact))
                return exact;
            foreach (var pair in _byType)
            {
                if (type.IsAssignableFrom(pair.Key))
                    return pair.Value;
            }
            return [type.IsValueType ? Activator.CreateInstance(type) : null];
        }

        private static IEnumerable<object?> Awkward(Type type)
        {
            if (type == typeof(double))
                return [0.0, -1.0, double.NaN, double.PositiveInfinity, 1e-300];
            if (type == typeof(int))
                return [0, -1, 1000];
            if (type == typeof(CancellationToken))
                return [];
            if (type.IsEnum)
                return [Enum.ToObject(type, 99)];
            return type.IsValueType ? [Activator.CreateInstance(type)] : [null];
        }

        public void Exercise(Type type, ConcurrentDictionary<string, string> escapes)
        {
            var receivers = new List<object?>();
            if (!type.IsAbstract || !type.IsSealed)
            {
                receivers.AddRange(Good(type).Where(type.IsInstanceOfType));
                if (type.IsValueType)
                    receivers.Add(Activator.CreateInstance(type));
            }
            foreach (ConstructorInfo constructor in type.GetConstructors(BindingFlags.Public | BindingFlags.Instance))
                Call(constructor, null, escapes);
            foreach (MethodInfo declared in type.GetMethods(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly))
            {
                if (Closed(declared) is not { } method)
                    continue;
                if (method.IsStatic)
                {
                    Call(method, null, escapes);
                    continue;
                }
                foreach (object? receiver in receivers)
                    Call(method, receiver, escapes);
            }
        }

        // A generic operation (extension<TShape>) on solids, or left out if a solid won't do
        private static MethodInfo? Closed(MethodInfo method)
        {
            if (!method.IsGenericMethodDefinition)
                return method;
            try
            {
                return method.MakeGenericMethod(method.GetGenericArguments().Select(_ => typeof(Solid)).ToArray());
            }
            catch (ArgumentException)
            {
                return null;
            }
        }

        private void Call(MethodBase method, object? receiver, ConcurrentDictionary<string, string> escapes)
        {
            ParameterInfo[] parameters = method.GetParameters();
            if (parameters.Any(p => p.ParameterType.IsByRefLike || p.ParameterType.IsPointer
                                    || (p.ParameterType.IsByRef && p.ParameterType.GetElementType()!.IsByRefLike)
                                    || p.ParameterType.ContainsGenericParameters)
                || (method.DeclaringType?.ContainsGenericParameters ?? false)
                || (method is MethodInfo info && info.ReturnType.IsByRefLike))
                return;
            // An init accessor called on a harvested object could be MeshOptions.Default's: 1e-300 is a legal deflection, and every
            // other test in this assembly's parallel run that meshed by default then failed, in about one run in four
            if (method is MethodInfo setter && setter.ReturnParameter.GetRequiredCustomModifiers().Contains(typeof(IsExternalInit)))
            {
                if (setter.DeclaringType?.GetConstructor(Type.EmptyTypes) is not { } fresh)
                    return;
                receiver = fresh.Invoke(null);
            }
            Type[] types = parameters.Select(p => p.ParameterType.IsByRef ? p.ParameterType.GetElementType()! : p.ParameterType).ToArray();
            object?[] good = types.Select(t => Good(t).FirstOrDefault()).ToArray();
            Invoke(method, receiver, good, escapes);
            for (int i = 0; i < types.Length; i++)
            {
                foreach (object? bad in Awkward(types[i]))
                {
                    var arguments = (object?[])good.Clone();
                    arguments[i] = bad;
                    Invoke(method, receiver, arguments, escapes);
                }
            }
        }

        private static void Invoke(MethodBase method, object? receiver, object?[] arguments, ConcurrentDictionary<string, string> escapes)
        {
            RuntimeHelpers.EnsureSufficientExecutionStack();
            Exception? thrown = null;
            var call = Task.Run(() =>
            {
                try
                {
                    if (method is ConstructorInfo constructor)
                        constructor.Invoke((object?[])arguments.Clone());
                    else
                        method.Invoke(receiver, (object?[])arguments.Clone());
                }
                catch (TargetInvocationException e)
                {
                    thrown = e.InnerException;
                }
                catch (Exception e) when (e is ArgumentException or TargetParameterCountException or NotSupportedException)
                {
                    // The reflection's own refusal of an argument, not the member's
                }
            });
            if (!call.Wait(TimeSpan.FromSeconds(20)) || thrown is null || thrown.GetType().IsVisible)
                return;
            escapes.TryAdd($"{method.DeclaringType!.FullName}.{method.Name}({string.Join(", ", method.GetParameters().Select(p => p.ParameterType.Name))})",
                           $"{thrown.GetType().Name}: {thrown.Message}");
        }
    }
}
