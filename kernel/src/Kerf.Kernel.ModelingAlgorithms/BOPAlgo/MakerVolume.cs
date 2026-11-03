// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakerVolume.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakerVolume.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakerVolume.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>
/// Builds solids from a set of shapes (faces, shells, solids; edges and vertices become internal parts): the shapes
/// are intersected (General Fuse), their split faces are put into a box enclosing them all, the solids are built from
/// the faces by <see cref="BuilderSolid"/>, and the solid holding the box is removed.
/// </summary>
/// <remarks>
/// With <see cref="SetIntersect"/> false the arguments are not intersected: they must already make closed volumes
/// that share their faces. The result is a solid, or a compound of solids. <see cref="Box"/> and <see cref="Faces"/>
/// return the box and the faces the solids were built from (OCCT's working data; the box is published frozen).
/// </remarks>
public class MakerVolume : BooleanBuilder
{
    protected bool _intersect;
    protected Box _bBox = new();
    protected Solid _sBox = new();
    protected NCollection_List<Shape> _faces = new();
    protected bool _avoidInternalShapes;

    /// <summary>Empty constructor.</summary>
    public MakerVolume()
    {
        _intersect = true;
        _avoidInternalShapes = false;
    }

    /// <summary>Clears the data.</summary>
    public override void Clear()
    {
        base.Clear();
        _intersect = true;
        _bBox = new Box();
        _sBox.Nullify();
        _faces.Clear();
        _avoidInternalShapes = false;
    }

    /// <summary>Sets whether the arguments are intersected (true by default).</summary>
    public void SetIntersect(bool bIntersect) => _intersect = bIntersect;

    /// <summary>Whether the arguments are intersected.</summary>
    public bool IsIntersect => _intersect;

    /// <summary>The solid box enclosing the faces (frozen).</summary>
    public Solid Box()
    {
        _sBox.TShape()?.FreezeGraph();
        return _sBox;
    }

    /// <summary>The solid box, unfrozen, for ported code.</summary>
    internal Solid InternalBox() => _sBox;

    /// <summary>The faces the solids are built from (each face in both orientations, and the box's faces).</summary>
    public NCollection_List<Shape> Faces() => _faces;

    /// <summary>Prevents the addition of internal parts into the result's solids (they are added by default).</summary>
    public void SetAvoidInternalShapes(bool avoidInternal) => _avoidInternalShapes = avoidInternal;

    /// <summary>Whether the internal parts are left out of the result.</summary>
    public bool IsAvoidInternalShapes => _avoidInternalShapes;

    protected override void CheckData()
    {
        if (_arguments.IsEmpty())
        {
            AddError(new AlertTooFewArguments()); // no arguments to process
            return;
        }
        //
        CheckFiller();
    }

    /// <summary>Performs the operation.</summary>
    public override void Perform(ProgressRange range = default)
    {
        using var ps = new ProgressScope(range, "Performing MakeVolume operation", 10);
        double interPart = _intersect ? 9 : 0.5;
        double buildPart = 10.0 - interPart;

        GetReport().Clear();
        //
        if (_entryPoint == 1)
        {
            _paveFiller = null;
        }
        //
        PaveFiller pPF = new();
        //
        if (!_intersect)
        {
            // if there is no need to intersect the arguments, then it is necessary
            // to create the compound of them and use it as one argument
            Compound anArgs = new();
            BrepBuilder bb = new();
            NCollection_List<Shape>.Iterator it = new();
            NCollection_List<Shape> ls = new();
            //
            bb.MakeCompound(ref anArgs);
            it.Initialize(_arguments);
            for (; it.More(); it.Next())
            {
                Shape s = it.Value();
                bb.Add(anArgs, s);
            }
            ls.Append(anArgs);
            //
            pPF.SetArguments(ls);
        }
        else
        {
            pPF.SetArguments(_arguments);
        }
        //
        pPF.SetRunParallel(_runParallel);
        pPF.SetFuzzyValue(_fuzzyValue);
        pPF.SetNonDestructive(_nonDestructive);
        pPF.SetGlue(_glue);
        pPF.SetUseOBB(_useOBB);
        pPF.Perform(ps.Next(interPart));
        //
        _entryPoint = 1;
        PerformInternal(pPF, ps.Next(buildPart));
    }

    protected override void PerformInternal1(PaveFiller filler, ProgressRange range)
    {
        using var ps = new ProgressScope(range, "Building volumes", 100);
        _paveFiller = filler;
        _ds = _paveFiller.PDS();
        _context = _paveFiller.Context();
        //
        // 1. CheckData
        CheckData();
        if (HasErrors)
        {
            return;
        }
        //
        // 2. Prepare
        Prepare();
        if (HasErrors)
        {
            return;
        }
        //
        PISteps steps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        AnalyzeProgress(100.0, steps);

        // 3. Fill Images
        if (_intersect)
        {
            // 3.1. Vertices
            FillImagesVertices(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatVertices)));
            if (HasErrors)
            {
                return;
            }
            // 3.2. Edges
            FillImagesEdges(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatEdges)));
            if (HasErrors)
            {
                return;
            }
            // 3.3. Wires
            FillImagesContainers(ShapeType.Wire, ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatWires)));
            if (HasErrors)
            {
                return;
            }
            // 3.4. Faces
            FillImagesFaces(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatFaces)));
            if (HasErrors)
            {
                return;
            }
        }
        //
        // 4. Collect faces
        CollectFaces();
        if (HasErrors)
        {
            return;
        }
        //
        HashSet<Shape> boxFaces = new(ShapeHasher.Instance);
        NCollection_List<Shape> lsr = new();
        //
        // 5. Create bounding box
        MakeBox(boxFaces);
        //
        // 6. Make volumes
        BuildSolids(lsr, ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_BuildSolids)));
        if (HasErrors)
        {
            return;
        }
        //
        // 7. Treat the result
        RemoveBox(lsr, boxFaces);
        //
        // 8. Fill internal shapes
        FillInternalShapes(lsr);
        //
        // 9. Build Result
        BuildShape(lsr);
        //
        // 10. History
        PrepareHistory(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_FillHistory)));
        if (HasErrors)
        {
            return;
        }
        //
        // 11. Post-treatment
        PostTreat(ps.Next(steps.GetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_PostTreat)));
    }

    /// <summary>The steps of the operation (OCCT's nested enum, after the Builder's).</summary>
    protected new enum BOPAlgo_PIOperation
    {
        PIOperation_BuildSolids = (int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_Last,
        PIOperation_Last,
    }

    protected override void FillPISteps(PISteps steps)
    {
        NbShapes nbShapes = GetNbShapes();
        if (_intersect)
        {
            steps.SetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatVertices, nbShapes.NbVertices());
            steps.SetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatEdges, nbShapes.NbEdges());
            steps.SetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatWires, nbShapes.NbWires());
            steps.SetStep((int)BooleanBuilder.BOPAlgo_PIOperation.PIOperation_TreatFaces, 50 * nbShapes.NbFaces());
        }
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_BuildSolids, 50 * nbShapes.NbFaces());
    }

    /// <summary>Collects all faces.</summary>
    protected void CollectFaces()
    {
        //
        int i, nbShapes;
        NCollection_List<Shape>.Iterator it = new();
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        //
        nbShapes = _ds!.SourceShapeCount;
        for (i = 0; i < nbShapes; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            //
            Box b = si.Box();
            _bBox.Add(b);
            //
            Shape f = si.Shape();
            if (_images.IsBound(f))
            {
                NCollection_List<Shape> lfIm = _images.Find(f);
                it.Initialize(lfIm);
                for (; it.More(); it.Next())
                {
                    Shape fIm = it.Value();
                    if (mFence.Add(fIm))
                    {
                        AddFace(fIm, _faces);
                    }
                }
            }
            else
            {
                AddFace(f, _faces);
            }
        }
    }

    /// <summary>Makes the solid box.</summary>
    protected void MakeBox(HashSet<Shape> boxFaces)
    {
        //
        double xmin, ymin, zmin, xmax, ymax, zmax, ext;
        //
        ext = Math.Sqrt(_bBox.SquareExtent()) * 0.5;
        _bBox.Enlarge(ext);
        _bBox.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);
        //
        Point3D pMin = new(xmin, ymin, zmin), pMax = new(xmax, ymax, zmax);
        //
        _sBox = new MakeBox(pMin, pMax).InternalSolid();
        //
        Explorer exp = new(_sBox, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Shape f = exp.Current();
            _faces.Append(f);
            boxFaces.Add(f);
        }
    }

    /// <summary>Builds the solids.</summary>
    protected void BuildSolids(NCollection_List<Shape> lsr, ProgressRange range)
    {
        BuilderSolid bs = new();
        //
        bs.SetShapes(_faces);
        bs.SetRunParallel(_runParallel);
        bs.SetAvoidInternalShapes(_avoidInternalShapes);
        bs.Perform(range);
        if (bs.HasErrors)
        {
            AddError(new AlertSolidBuilderFailed()); // SolidBuilder failed
            return;
        }
        //
        _report.Merge(bs.GetReport());
        //
        lsr.Assign(bs.Areas());
    }

    /// <summary>Removes the covering box.</summary>
    protected void RemoveBox(NCollection_List<Shape> lsr, HashSet<Shape> boxFaces)
    {
        //
        NCollection_List<Shape>.Iterator it = new();
        Explorer exp = new();
        bool bFound;
        //
        bFound = false;
        it.Initialize(lsr);
        for (; it.More(); it.Next())
        {
            Shape sr = it.Value();
            //
            exp.Init(sr, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Shape f = exp.Current();
                if (boxFaces.Contains(f))
                {
                    bFound = true;
                    lsr.Remove(it);
                    break;
                }
            }
            if (bFound)
            {
                break;
            }
        }
    }

    /// <summary>Builds the result.</summary>
    protected void BuildShape(NCollection_List<Shape> lsr)
    {
        if (lsr.Extent() == 1)
        {
            _shape = lsr.First();
        }
        else
        {
            BrepBuilder bb = new();
            NCollection_List<Shape>.Iterator it = new();
            //
            it.Initialize(lsr);
            for (; it.More(); it.Next())
            {
                Shape sol = it.Value();
                bb.Add(_shape, sol);
            }
        }
    }

    /// <summary>Fills the solids with the internal shapes (the arguments' edges and vertices).</summary>
    protected void FillInternalShapes(NCollection_List<Shape> lsr)
    {
        if (_avoidInternalShapes)
        {
            return;
        }

        // Get all non-compound shapes
        NCollection_List<Shape> lsc = new();
        // Fence map
        HashSet<Shape> mFence = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itLA = new(_ds!.Arguments());
        for (; itLA.More(); itLA.Next())
        {
            AlgoTools.TreatCompound(itLA.Value(), lsc, mFence);
        }

        // Get only edges and vertices from arguments
        NCollection_List<Shape> lve = new();

        itLA.Initialize(lsc);
        for (; itLA.More(); itLA.Next())
        {
            Shape s = itLA.Value();
            ShapeType type = s.ShapeType();
            if (type == ShapeType.Wire)
            {
                for (Iterator it = new(s); it.More(); it.Next())
                {
                    Shape ss = it.Value();
                    if (mFence.Add(ss))
                    {
                        lve.Append(ss);
                    }
                }
            }
            else if (type == ShapeType.Vertex || type == ShapeType.Edge)
            {
                lve.Append(s);
            }
        }

        BuilderTools.FillInternals(lsr, lve, _images, _context!);
    }

    private static void AddFace(in Shape f, NCollection_List<Shape> lf)
    {
        Shape ff = f;
        ff.Orientation(Forward);
        lf.Append(ff);
        ff.Orientation(Reversed);
        lf.Append(ff);
    }
}
