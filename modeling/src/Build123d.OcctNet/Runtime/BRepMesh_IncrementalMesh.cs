using O = global::Occt;

namespace Build123d.Occt;

// OCCT meshes the shape it's given in place. OCCT.NET never writes a frozen shape: it meshes a copy and hands it back
// from Shape() (its decision 489). So the wrapper given here takes the meshed shape, as the native wrapper's shape
// would carry the mesh. build123d.NET meshes only private copies, so nothing else sees the change.
public partial class BRepMesh_IncrementalMesh
{
    public BRepMesh_IncrementalMesh() : this(New(() => new O.BRepMesh_IncrementalMesh())) { }

    public BRepMesh_IncrementalMesh(TopoDS_Shape theShape, double theLinDeflection)
        : this(Mesh(theShape, s => new O.BRepMesh_IncrementalMesh(s, theLinDeflection))) { }

    public BRepMesh_IncrementalMesh(TopoDS_Shape theShape, double theLinDeflection, bool isRelative)
        : this(Mesh(theShape, s => new O.BRepMesh_IncrementalMesh(s, theLinDeflection, isRelative))) { }

    public BRepMesh_IncrementalMesh(TopoDS_Shape theShape, double theLinDeflection, bool isRelative, double theAngDeflection)
        : this(Mesh(theShape, s => new O.BRepMesh_IncrementalMesh(s, theLinDeflection, isRelative, theAngDeflection))) { }

    public BRepMesh_IncrementalMesh(TopoDS_Shape theShape, double theLinDeflection, bool isRelative, double theAngDeflection, bool isInParallel)
        : this(Mesh(theShape, s => new O.BRepMesh_IncrementalMesh(s, theLinDeflection, isRelative, theAngDeflection, isInParallel))) { }

    private static O.BRepMesh_IncrementalMesh New(Func<O.BRepMesh_IncrementalMesh> make)
    {
        try
        {
            return make();
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    private static O.BRepMesh_IncrementalMesh Mesh(TopoDS_Shape shape, Func<O.TopoDS_Shape, O.BRepMesh_IncrementalMesh> make)
    {
        ArgumentNullException.ThrowIfNull(shape);
        var mesher = New(() => make(shape.V));
        shape.V = mesher.Shape();
        return mesher;
    }
}
