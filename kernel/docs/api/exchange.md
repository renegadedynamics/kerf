# API review: exchange

*Unit 10D, draft. Namespace `Kerf.Exchange`, in DataExchange (BREP and the shared types in ModelingData). Engine:
`Kerf.Engine.Exchange` and its sub-namespaces: `Step` (STEPControl, STEPCAFControl, the 1,300 entity classes and
their readers), `Framework` (Interface, Transfer, IFSelect, XSControl), `Stl`, `Gltf`, `ThreeMf`, `Obj`, `Brep`,
`Documents` (XdeDoc).*

## Main types

New: one static class per format, `Step`, `Stl`, `Gltf`, `ThreeMf`, `Obj`, `Brep`; `ModelNode` (an assembly tree with
names, colours and placements); `ColorRgba`; `LengthUnit`; `ReadLimits`; options records per format.

```csharp
public static class Step
{
    public static Result<ModelNode> Read(Stream stream, StepReadOptions? options = null,
                                         IProgress<double>? progress = null, CancellationToken cancellationToken = default);
    public static Result<ModelNode> Read(string path, StepReadOptions? options = null, …);   // resolves external references beside it
    public static Result Write(ModelNode model, Stream stream, StepWriteOptions? options = null, …);
    public static Result Write(Shape shape, Stream stream, StepWriteOptions? options = null, …);
}

public static class Stl
{
    public static Result<Face> Read(Stream stream, StlReadOptions? options = null, CancellationToken cancellationToken = default);  // a mesh-only face
    public static Result Write(ShapeMesh mesh, Stream stream, StlWriteOptions? options = null, CancellationToken cancellationToken = default);
}

public static class Gltf   // and ThreeMf.Write, Obj.Read: the same shape
{
    public static Result<ModelNode> Read(Stream stream, GltfReadOptions? options = null, CancellationToken cancellationToken = default);
    public static Result Write(ModelNode model, Stream stream, GltfWriteOptions? options = null, CancellationToken cancellationToken = default);
}

public static class Brep
{
    public static Result<Shape> Read(Stream stream, ReadLimits? limits = null, CancellationToken cancellationToken = default);
    public static Result Write(Shape shape, Stream stream, BrepWriteOptions? options = null);   // text or binary, with or without triangles
}

public sealed record ModelNode
{
    public string? Name { get; init; }
    public ColorRgba? Color { get; init; }                     // null: inherit the parent's
    public Transform3D Placement { get; init; } = Transform3D.Identity;   // relative to the parent
    public Shape? Shape { get; init; }                         // a part; null for an assembly
    public ImmutableArray<ModelNode> Children { get; init; } = [];
    public static ModelNode Part(Shape shape, string? name = null, ColorRgba? color = null);
    public static ModelNode Assembly(IEnumerable<ModelNode> children, string? name = null, Transform3D? placement = null);
    public Shape ToShape();                                     // the parts placed, as one compound
}

public readonly record struct ColorRgba(float R, float G, float B, float A = 1)     // linear RGB, as OCCT keeps colours
{
    public static ColorRgba FromSrgb(double r, double g, double b, double a = 1);
    public (double R, double G, double B, double A) ToSrgb();
}

public sealed record ReadLimits                                 // hostile input: refuse rather than allocate
{
    public static ReadLimits Default { get; } = new();
    public long MaxBytes { get; init; } = 1L << 30;
    public int MaxEntities { get; init; } = 20_000_000;   public int MaxDepth { get; init; } = 1_000;
    public long MaxTriangles { get; init; } = 50_000_000; public int MaxNodes { get; init; } = 1_000_000;
}

public sealed record StepReadOptions { public ReadLimits Limits { get; init; } = ReadLimits.Default; public bool Heal { get; init; } = true;
                                       public bool ProductNames { get; init; } = true; public bool ExternalReferences { get; init; } }
public sealed record StepWriteOptions { public LengthUnit Unit { get; init; } = LengthUnit.Millimetre; public bool WritePcurves { get; init; } = true;
                                        public StepPrecision Precision { get; init; } = StepPrecision.Average; public DateTimeOffset? Timestamp { get; init; }
                                        public StepSchema Schema { get; init; } = StepSchema.Ap214; public bool CleanDuplicates { get; init; } }
public sealed record GltfWriteOptions { public bool Binary { get; init; } = true; public LengthUnit Unit { get; init; } = LengthUnit.Millimetre;
                                        public MeshOptions Mesh { get; init; } = MeshOptions.Default; }
```

- **Readers take streams, limits and cancellation**, as build123d.NET's `Importers` and Bento's `CAD_IO_` functions do
  today; a malformed or over-limit file is a failed `Result` with `exchange.malformed`, `exchange.limit-exceeded`
  (with which limit), `exchange.unsupported-schema`; skipped entities are warnings (`exchange.entity-skipped`) with
  their STEP ids in the message. Assemblies nest at most `ModelNode.MaxNesting`, 64 levels, in a file or in code (decision 21102).
- **Writers take the mesh or tree they write.** STL takes a `ShapeMesh` (meshing.md) rather than meshing a shape in
  place; glTF and 3MF mesh with `options.Mesh` themselves.
- **Units** in the API are explicit (`LengthUnit`): shapes are in millimetres unless a reader's options say otherwise.

## Examples

```csharp
var read = Step.Read(stream, new StepReadOptions { Limits = ReadLimits.Default with { MaxBytes = 200 << 20 } },
                     cancellationToken: cancel);
if (!read.IsSuccess) return CellError.Value(read.Error!.Message);    // Bento's #VALUE!
Shape everything = read.Value.ToShape();
```

```csharp
var model = ModelNode.Assembly([ModelNode.Part(body, "Body", ColorRgba.FromSrgb(0.9, 0.7, 0.1)),
                                ModelNode.Part(lid, "Lid")], name: "Box");
Step.Write(model, file, new StepWriteOptions { Unit = LengthUnit.Millimetre }).GetValueOrThrow();
```

```csharp
Stl.Write(part.Mesh(new MeshOptions { LinearDeflection = 1e-3 }).Value, stream, new StlWriteOptions { Ascii = false });
```

```csharp
Face scanned = Stl.Read(stream).Value;            // one face holding the triangles
```

```csharp
Brep.Write(shape, stream, new BrepWriteOptions { WithTriangles = false });
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `STEPControl_Reader` (`ReadStream`, `TransferRoots`, `OneShape`), `IFSelect_ReturnStatus` | `Step.Read` → `Result<ModelNode>` |
| `STEPCAFControl_Reader` with `XdeDoc_Document` (names, colours, assemblies) | `Step.Read` → `ModelNode` |
| `STEPControl_Writer`, `STEPCAFControl_Writer`, `STEPControl_StepModelType`, `Interface_Static` settings | `Step.Write` with `StepWriteOptions` |
| `StepFile_ReadLimits` | `ReadLimits` (one record for all readers) |
| `StlAPI_Writer`, `RWStl.ReadStream` | `Stl.Write(ShapeMesh)`, `Stl.Read` |
| `RWGltf_CafWriter`, `RWGltf_CafReader` | `Gltf.Write`, `Gltf.Read` |
| `RW3mf_…Writer` | `ThreeMf.Write` |
| `RWObj_CafReader` | `Obj.Read` |
| `BRepTools.Write/Read`, `BinTools.Write/Read` | `Brep.Write`, `Brep.Read` |
| `Quantity_Color`, `Quantity_ColorRGBA`, `Quantity_TypeOfColor` | `ColorRgba` |
| `XdeDoc_LengthUnit`, `UnitsMethods` | `LengthUnit` |
| build123d.NET's `ExchangeNode`, `ExchangeReadLimits`, `StepWriteSettings`, `Extras.ReadStl/WriteGltf…` | `ModelNode`, `ReadLimits`, `StepWriteOptions`, the format classes |

**Disappears from public:** the whole STEP entity model (`StepBasic_…` to `StepVisual_…`, about 1,300 classes), the
`Interface`/`Transfer`/`IFSelect` framework, `XSControl_WorkSession`, `XdeDoc`'s label API, `Interface_Static`'s
string-keyed settings, `Message_ProgressRange`.

## Open points

- `ModelNode` covers parts, assemblies, names, colours and placements, which is all the modelling layer and Bento use.
  Layers, materials, PMI and validation properties stay in the engine's document until someone needs them.
- IGES isn't ported; nothing here names it.
- Whether `Step.Read(path)` belongs in the kernel (file access, external references) or only streams, with the
  modelling layer resolving files. Proposed: both, since external references need a base folder.
