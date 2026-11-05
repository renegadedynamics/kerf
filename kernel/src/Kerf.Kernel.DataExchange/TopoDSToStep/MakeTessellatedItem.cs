// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeTessellatedItem.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeTessellatedItem.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>
/// Maps the triangulation of a face to a triangulated_face or a triangulated_surface_set, and the triangulations of a
/// shell's faces to a tessellated_shell.
/// </summary>
internal sealed class MakeTessellatedItem : ExportRoot
{
    private TessellatedItem _tessellatedItem;

    // Get parameters from a TriangulatedFace or TriangulatedSurfaceSet Poly_Triangulation
    private static void InitTriangulation(Triangulation mesh, TCollection_HAsciiString name, CoordinatesList coordinates,
                                          NCollection_HArray1<Xyz> points, NCollection_HArray2<double> normals,
                                          NCollection_HArray1<int> indices, NCollection_HArray2<int> trias,
                                          in Factors localFactors)
    {
        double factor = localFactors.LengthFactor();
        for (int nodeIndex = 1; nodeIndex <= mesh.NodeCount; ++nodeIndex)
        {
            points.SetValue(nodeIndex, mesh.Node(nodeIndex).XYZ() / factor);
        }

        coordinates.Init(name, points);

        if (!mesh.HasNormals)
        {
            // Kerf: OCCT computes the normals into the face's own triangulation; the port into a copy, so the
            // shape written never changes (decision 1252)
            mesh = new Triangulation(mesh);
            global::Kerf.Engine.Meshing.Poly.ComputeNormals(mesh);
        }

        for (int nodeIndex = 1; nodeIndex <= mesh.NodeCount; ++nodeIndex)
        {
            Direction3D norm = mesh.Normal(nodeIndex);
            normals.SetValue(nodeIndex, 1, norm.X);
            normals.SetValue(nodeIndex, 2, norm.Y);
            normals.SetValue(nodeIndex, 3, norm.Z);
        }

        for (int nodeIndex = 1; nodeIndex <= mesh.NodeCount; ++nodeIndex)
        {
            indices.SetValue(nodeIndex, nodeIndex);
        }

        for (int triangleIndex = 1; triangleIndex <= mesh.TriangleCount; ++triangleIndex)
        {
            Triangle1 t = mesh.Triangle(triangleIndex);
            trias.SetValue(triangleIndex, 1, t.Value(1));
            trias.SetValue(triangleIndex, 2, t.Value(2));
            trias.SetValue(triangleIndex, 3, t.Value(3));
        }
    }

    public MakeTessellatedItem()
    {
    }

    public MakeTessellatedItem(in Face face, ExportTool tool, FinderProcess fp,
                                            bool toPreferSurfaceSet, in Factors localFactors,
                                            ProgressRange progress = default)
    {
        Init(face, tool, fp, toPreferSurfaceSet, localFactors, progress);
    }

    public MakeTessellatedItem(in Shell shell, ExportTool tool, FinderProcess fp,
                                            in Factors localFactors, ProgressRange progress = default)
    {
        Init(shell, tool, fp, localFactors, progress);
    }

    /// <summary>Creates a triangulated_face or a triangulated_surface_set from a face.</summary>
    public void Init(in Face face, ExportTool tool, FinderProcess fp, bool toPreferSurfaceSet,
                     in Factors localFactors, ProgressRange progress = default)
    {
        _done = false;
        if (progress.UserBreak())
        {
            return;
        }

        Triangulation mesh = Tool.Triangulation(face, out Placement loc);
        if (mesh is null)
        {
            var errShape = new ShapeMapper(face);
            fp.AddWarning(errShape, " Face not mapped to TessellatedItem");
            return;
        }

        var name = new TCollection_HAsciiString("");
        var coordinates = new CoordinatesList();
        var points = new NCollection_HArray1<Xyz>(1, mesh.NodeCount);
        var normals = new NCollection_HArray2<double>(1, mesh.NodeCount, 1, 3);
        var indices = new NCollection_HArray1<int>(1, mesh.NodeCount);
        var trias = new NCollection_HArray2<int>(1, mesh.TriangleCount, 1, 3);
        InitTriangulation(mesh, name, coordinates, points, normals, indices, trias, localFactors);

        bool hasGeomLink = tool.IsBound(face);
        var geomLink = new FaceOrSurface();
        if (hasGeomLink)
        {
            StepTopologicalRepresentationItem topoItem = tool.Find(face);
            geomLink.SetValue(topoItem);
            var triaFace = new TriangulatedFace();
            triaFace.Init(name, coordinates, mesh.NodeCount, normals, hasGeomLink, geomLink, indices, trias);
            _tessellatedItem = triaFace;
        }
        else if (toPreferSurfaceSet)
        {
            var triaSurfaceSet = new TriangulatedSurfaceSet();
            triaSurfaceSet.Init(name, coordinates, mesh.NodeCount, normals, indices, trias);
            _tessellatedItem = triaSurfaceSet;
        }
        else
        {
            var triaFace = new TriangulatedFace();
            triaFace.Init(name, coordinates, mesh.NodeCount, normals, hasGeomLink, geomLink, indices, trias);
            _tessellatedItem = triaFace;
        }

        _done = true;
    }

    /// <summary>Creates a tessellated_shell from a shell.</summary>
    public void Init(in Shell shell, ExportTool tool, FinderProcess fp, in Factors localFactors,
                     ProgressRange progress = default)
    {
        _done = false;
        _tessellatedItem = null;

        if (progress.UserBreak())
        {
            return;
        }

        var exp = new Explorer();
        int nbFaces = 0;
        for (exp.Init(shell, ShapeType.Face); exp.More(); exp.Next(), ++nbFaces)
        {
        }

        using var ps = new ProgressScope(progress, null, nbFaces);

        var tessFaces = new NCollection_Sequence<TessellatedStructuredItem>();
        for (exp.Init(shell, ShapeType.Face); exp.More() && ps.More(); exp.Next(), ps.Next())
        {
            Face face = ShapeCasts.Face(exp.Current());
            var makeFace = new MakeTessellatedItem(face, tool, fp, false, localFactors, ps.Next());
            if (makeFace.IsDone)
            {
                tessFaces.Append(makeFace.Value() as TessellatedStructuredItem);
            }
        }

        if (tessFaces.IsEmpty())
        {
            return;
        }

        var tessShell = new TessellatedShell();
        var name = new TCollection_HAsciiString("");

        var items = new NCollection_HArray1<TessellatedStructuredItem>(1, tessFaces.Length());
        for (int indx = 1; indx <= tessFaces.Length(); ++indx)
        {
            items.SetValue(indx, tessFaces.Value(indx));
        }

        StepConnectedFaceSet faceSet = null;
        if (tool.IsBound(shell))
        {
            faceSet = tool.Find(shell) as StepConnectedFaceSet;
        }

        bool hasTopoLink = faceSet is not null;
        tessShell.Init(name, items, hasTopoLink, faceSet);

        _tessellatedItem = tessShell;

        // TopoDSToStep::AddResult(theFP, theShell, theTessellatedItem);
        _done = true;
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public TessellatedItem Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeTessellatedItem::Value() - no result");
        return _tessellatedItem;
    }
}
