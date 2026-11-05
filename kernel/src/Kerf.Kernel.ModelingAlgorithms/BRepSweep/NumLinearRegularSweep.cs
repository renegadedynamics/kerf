// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_NumLinearRegularSweep.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_NumLinearRegularSweep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>
/// The topological sweep of a generating shape along a directing indexed edge (<see cref="NumShape"/>): the
/// result shape of each pair (generating sub-shape, directing sub-shape) is built on demand from the results of their
/// sub-shapes, with the geometry given by the derived class.
/// </summary>
/// <remarks>
/// Internal (decision 32xx-S1): the shapes it builds are unfrozen and, without copy, it writes into the generating
/// shape's sub-shapes; callers use <see cref="ApiMakePrism"/> and <see cref="ApiMakeRevol"/>.
/// </remarks>
internal abstract class NumLinearRegularSweep
{
    protected readonly SweepBuilder _builder;
    protected readonly Shape _genShape;
    protected readonly NumShape _dirWire;
    protected readonly SweepingTool _genShapeTool;
    protected readonly NumShapeTool _dirShapeTool;
    protected readonly NCollection_Array2<Shape> _shapes;
    protected readonly NCollection_Array2<bool> _builtShapes;
    protected readonly NCollection_Array2<bool> _usedShapes;

    protected NumLinearRegularSweep(SweepBuilder aBuilder, in Shape aGenShape, in NumShape aDirShape)
    {
        _builder = aBuilder;
        _genShape = aGenShape;
        _dirWire = aDirShape;
        _genShapeTool = new SweepingTool(aGenShape);
        _dirShapeTool = new NumShapeTool(aDirShape);
        // Les Tableaux
        _shapes = new NCollection_Array2<Shape>(1, _genShapeTool.ShapeCount, 1, _dirShapeTool.NbShapes());
        _builtShapes = new NCollection_Array2<bool>(1, _genShapeTool.ShapeCount, 1, _dirShapeTool.NbShapes());
        _usedShapes = new NCollection_Array2<bool>(1, _genShapeTool.ShapeCount, 1, _dirShapeTool.NbShapes());
        _builtShapes.Init(false);
        _usedShapes.Init(false);
    }

    /// <summary>Builds the vertex addressed by [aGenV,aDirV], with its geometric part, but without sub-shapes.</summary>
    public abstract Shape MakeEmptyVertex(in Shape aGenV, in NumShape aDirV);

    /// <summary>Builds the edge addressed by [aGenV,aDirE], with its geometric part, but without sub-shapes.</summary>
    public abstract Shape MakeEmptyDirectingEdge(in Shape aGenV, in NumShape aDirE);

    /// <summary>Builds the edge addressed by [aGenE,aDirV], with its geometric part, but without sub-shapes.</summary>
    public abstract Shape MakeEmptyGeneratingEdge(in Shape aGenE, in NumShape aDirV);

    /// <summary>Sets the parameters of the new vertex on the new face (directly in a cap face).</summary>
    public abstract void SetParameters(in Shape aNewFace, ref Shape aNewVertex, in Shape aGenF,
                                       in Shape aGenV, in NumShape aDirV);

    /// <summary>Sets the parameter of the new vertex on the new edge (the edge was built from a vertex).</summary>
    public abstract void SetDirectingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenV,
                                               in NumShape aDirE, in NumShape aDirV);

    /// <summary>Sets the parameter of the new vertex on the new edge (the edge was built from an edge).</summary>
    public abstract void SetGeneratingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenE,
                                                in Shape aGenV, in NumShape aDirV);

    /// <summary>Builds the face addressed by [aGenS,aDirS], with its geometric part, but without sub-shapes.</summary>
    public abstract Shape MakeEmptyFace(in Shape aGenS, in NumShape aDirS);

    /// <summary>Sets the pcurve of the new edge on the new face (a cap face).</summary>
    public abstract void SetPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenF,
                                   in Shape aGenE, in NumShape aDirV, Orientation orien);

    /// <summary>Sets the pcurve of the new edge (built from an edge) on the new face.</summary>
    public abstract void SetGeneratingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                             in NumShape aDirE, in NumShape aDirV, Orientation orien);

    /// <summary>Sets the pcurve of the new edge (built from a vertex) on the new face.</summary>
    public abstract void SetDirectingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                            in Shape aGenV, in NumShape aDirE, Orientation orien);

    /// <summary>The orientation of the shell in the solid generated by the face and the directing edge.</summary>
    public abstract Orientation DirectSolid(in Shape aGenS, in NumShape aDirS);

    /// <summary>True if aNewSubShape (addressed by [aSubGenS,aDirS]) is to be added to aNewShape ([aGenS,aDirS]).</summary>
    public abstract bool GGDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Shape aSubGenS, in NumShape aDirS);

    /// <summary>True if aNewSubShape (addressed by [aGenS,aSubDirS]) is to be added to aNewShape ([aGenS,aDirS]).</summary>
    public abstract bool GDDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in NumShape aDirS, in NumShape aSubDirS);

    /// <summary>In some particular cases the topology of a generated face must be composed of independent closed wires.</summary>
    public abstract bool SeparatedWires(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                        in Shape aSubGenS, in NumShape aDirS);

    /// <summary>In some particular cases the topology of a generated shell must be split in several shells.</summary>
    public virtual Shape SplitShell(in Shape aNewShape)
    {
        Shape comp = new();
        _builder.MakeCompound(ref comp);
        _builder.Add(comp, aNewShape);
        return comp;
    }

    /// <summary>Called to set the continuity of the new edges.</summary>
    public abstract void SetContinuity(in Shape aGenS, in NumShape aDirS);

    /// <summary>True if the shape addressed by [aGenS,aDirS] exists.</summary>
    public abstract bool HasShape(in Shape aGenS, in NumShape aDirS);

    /// <summary>True if aGenS can't be transformed by the sweep.</summary>
    public abstract bool IsInvariant(in Shape aGenS);

    /// <summary>The resulting shape indexed by aDirS and aGenS.</summary>
    public Shape Shape(in Shape aGenS, in NumShape aDirS)
    {
        int iGenS = _genShapeTool.Index(aGenS);
        int iDirS = _dirShapeTool.Index(aDirS);
        if (!_builtShapes[iGenS, iDirS])
        {
            Shape newShape;
            Shape bGenS, cGenS, subGenS, subsubGenS;
            NumShape bDirS, subDirS;
            var it = new SweepingIterator();
            var kt = new NumShapeIterator();
            var lt = new SweepingIterator();
            Orientation or, pr;
            if (_dirShapeTool.Type(aDirS) == ShapeType.Vertex)
            {
                // Ici on construit les "planchers" du Shape.
                ShapeType genSType = _genShapeTool.Type(aGenS);
                switch (genSType)
                {
                    case ShapeType.Vertex:
                        _shapes[iGenS, iDirS] = MakeEmptyVertex(aGenS, aDirS);
                        break;
                    case ShapeType.Edge:
                        _shapes[iGenS, iDirS] = MakeEmptyGeneratingEdge(aGenS, aDirS);
                        break;
                    case ShapeType.Wire:
                        _builder.MakeWire(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Face:
                        _shapes[iGenS, iDirS] = MakeEmptyFace(aGenS, aDirS);
                        break;
                    case ShapeType.Shell:
                        _builder.MakeShell(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Solid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.CompSolid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.Compound:
                        _builder.MakeCompound(ref _shapes[iGenS, iDirS]);
                        break;
                    default:
                        throw new NoSuchObjectException("Unknown Shape");
                }
                bGenS = aGenS;
                _genShapeTool.SetOrientation(ref bGenS, Forward);
                for (it.Init(bGenS); it.More(); it.Next())
                {
                    subGenS = it.Value();
                    or = it.Orientation();
                    if (HasShape(subGenS, aDirS))
                    {
                        newShape = Shape(subGenS, aDirS);
                        int iNewGenS = _genShapeTool.Index(subGenS);
                        int iNewDirS = iDirS;
                        if (GGDShapeIsToAdd(_shapes[iGenS, iDirS], newShape, aGenS, subGenS, aDirS))
                        {
                            // Les "planchers" doivent etre construits par les
                            // fonctions de construcion geometrique identiquement
                            // au shape generateur.
                            // On leur recolle juste une orientation pour etre bien
                            // sur.
                            _builder.Add(_shapes[iGenS, iDirS], newShape, or);
                            _usedShapes[iNewGenS, iNewDirS] = true;
                            ShapeType subGenSType = _genShapeTool.Type(subGenS);
                            if (genSType == ShapeType.Face)
                            {
                                if (subGenSType == ShapeType.Vertex)
                                {
                                    SetParameters(_shapes[iGenS, iDirS], ref newShape, aGenS, subGenS, aDirS);
                                }
                                else if (subGenSType == ShapeType.Edge)
                                {
                                    SetPCurve(_shapes[iGenS, iDirS], ref newShape, aGenS, subGenS, aDirS, or);
                                }
                                else if (subGenSType == ShapeType.Wire)
                                {
                                    var jt = new SweepingIterator();
                                    cGenS = subGenS;
                                    _genShapeTool.SetOrientation(ref cGenS, Forward);
                                    for (jt.Init(cGenS); jt.More(); jt.Next())
                                    {
                                        subsubGenS = jt.Value();
                                        pr = jt.Orientation();
                                        if (HasShape(subsubGenS, aDirS))
                                        {
                                            Shape newsubEdge = Shape(subsubGenS, aDirS);
                                            SetPCurve(_shapes[iGenS, iDirS], ref newsubEdge, aGenS, subsubGenS, aDirS, pr);
                                        }
                                    }
                                }
                            }
                            else if (genSType == ShapeType.Edge)
                            {
                                SetGeneratingParameter(_shapes[iGenS, iDirS], ref newShape, bGenS, subGenS, aDirS);
                            }
                        }
                    }
                }
            }
            else if (_dirShapeTool.Type(aDirS) == ShapeType.Edge)
            {
                // Ici on construit les murs du Shape.
                ShapeType genSType = _genShapeTool.Type(aGenS);
                Shape newWire = new(), newShell = new();
                var wireSeq = new NCollection_Sequence<Shape>();
                bool sepwires = false;
                switch (genSType)
                {
                    case ShapeType.Vertex:
                        _shapes[iGenS, iDirS] = MakeEmptyDirectingEdge(aGenS, aDirS);
                        break;
                    case ShapeType.Edge:
                        // On cree un wire intermediaire qui contient tous les edges
                        // du montant (face) du Shape pour le cas standard, et une
                        // sequence de wires pour les cas merdiques necessitant des
                        // wires independants.
                        _builder.MakeWire(ref newWire);
                        _shapes[iGenS, iDirS] = MakeEmptyFace(aGenS, aDirS);
                        break;
                    case ShapeType.Wire:
                        _builder.MakeShell(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Face:
                        // On cree un shell intermediaire dans lequel on jette toutes
                        // les faces en direct, pour eviter les empilages compliques
                        // de shells et sous shells dans la structure du solide.
                        _builder.MakeShell(ref newShell);
                        _builder.MakeSolid(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Shell:
                        _builder.MakeCompSolid(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Solid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.CompSolid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.Compound:
                        _builder.MakeCompound(ref _shapes[iGenS, iDirS]);
                        break;
                    default:
                        throw new NoSuchObjectException("Unknown Shape");
                }
                bGenS = aGenS;
                _genShapeTool.SetOrientation(ref bGenS, Forward);
                for (it.Init(bGenS); it.More(); it.Next())
                {
                    subGenS = it.Value();
                    if (HasShape(subGenS, aDirS))
                    {
                        newShape = Shape(subGenS, aDirS);
                        int iNewGenS = _genShapeTool.Index(subGenS);
                        int iNewDirS = iDirS;
                        if (GGDShapeIsToAdd(_shapes[iGenS, iDirS], newShape, aGenS, subGenS, aDirS))
                        {
                            ShapeType subGenSType = _genShapeTool.Type(subGenS);
                            if (genSType == ShapeType.Edge)
                            {
                                or = it.Orientation();
                                if (SeparatedWires(_shapes[iGenS, iDirS], newShape, aGenS, subGenS, aDirS))
                                {
                                    sepwires = true;
                                    Shape wi = new();
                                    _builder.MakeWire(ref wi);
                                    _builder.Add(wi, newShape, or);
                                    _usedShapes[iNewGenS, iNewDirS] = true;
                                    wi.Closed(Tool.IsClosed(wi));
                                    wireSeq.Append(wi);
                                }
                                else
                                {
                                    _builder.Add(newWire, newShape, or);
                                    _usedShapes[iNewGenS, iNewDirS] = true;
                                }
                                SetDirectingPCurve(_shapes[iGenS, iDirS], ref newShape, bGenS, subGenS, aDirS, or);
                            }
                            else if (genSType == ShapeType.Wire)
                            {
                                or = it.Orientation();
                                _builder.Add(_shapes[iGenS, iDirS], newShape, or);
                                _usedShapes[iNewGenS, iNewDirS] = true;
                            }
                            else if (genSType == ShapeType.Face)
                            {
                                or = it.Orientation();
                                if (subGenSType == ShapeType.Wire)
                                {
                                    for (lt.Init(newShape); lt.More(); lt.Next())
                                    {
                                        _builder.Add(newShell, lt.Value(), global::Kerf.Engine.Topology.TopAbs.Compose(lt.Orientation(), or));
                                    }
                                }
                                else if (subGenSType == ShapeType.Edge)
                                {
                                    _builder.Add(newShell, newShape, or);
                                    _usedShapes[iNewGenS, iNewDirS] = true;
                                }
                            }
                            else if (genSType == ShapeType.Shell)
                            {
                                or = Forward;
                                _builder.Add(_shapes[iGenS, iDirS], newShape, or);
                                _usedShapes[iNewGenS, iNewDirS] = true;
                            }
                            else if (genSType == ShapeType.Compound)
                            {
                                or = Forward;
                                _builder.Add(_shapes[iGenS, iDirS], newShape, or);
                                _usedShapes[iNewGenS, iNewDirS] = true;
                            }
                            else
                            {
                                or = it.Orientation();
                                _builder.Add(_shapes[iGenS, iDirS], newShape, or);
                                _usedShapes[iNewGenS, iNewDirS] = true;
                            }
                        }
                    }
                }
                bDirS = aDirS;
                for (kt.Init(bDirS); kt.More(); kt.Next())
                {
                    subDirS = kt.Value();
                    if (HasShape(aGenS, subDirS))
                    {
                        newShape = Shape(aGenS, subDirS);
                        int iNewGenS = iGenS;
                        int iNewDirS = _dirShapeTool.Index(subDirS);
                        if (GDDShapeIsToAdd(_shapes[iGenS, iDirS], newShape, aGenS, aDirS, subDirS))
                        {
                            if (genSType == ShapeType.Edge)
                            {
                                or = global::Kerf.Engine.Topology.TopAbs.Reverse(kt.Orientation());
                                _builder.Add(newWire, newShape, or);
                                _usedShapes[iNewGenS, iNewDirS] = true;
                                SetGeneratingPCurve(_shapes[iGenS, iDirS], ref newShape, aGenS, aDirS, subDirS, or);
                            }
                            else if (genSType == ShapeType.Vertex)
                            {
                                or = kt.Orientation();
                                _builder.Add(_shapes[iGenS, iDirS], newShape, or);
                                _usedShapes[iNewGenS, iNewDirS] = true;
                                SetDirectingParameter(_shapes[iGenS, iDirS], ref newShape, aGenS, aDirS, subDirS);
                            }
                            else if (genSType == ShapeType.Face)
                            {
                                or = kt.Orientation();
                                _builder.Add(newShell, newShape, or);
                                _usedShapes[iNewGenS, iNewDirS] = true;
                            }
                        }
                    }
                }
                if (genSType == ShapeType.Edge)
                {
                    if (sepwires)
                    {
                        for (int ij = 1; ij <= wireSeq.Length(); ij++)
                        {
                            _builder.Add(_shapes[iGenS, iDirS], wireSeq.Value(ij));
                        }
                    }
                    else
                    {
                        newWire.Closed(Tool.IsClosed(newWire));
                        _builder.Add(_shapes[iGenS, iDirS], newWire);
                    }
                    _builtShapes[iGenS, iDirS] = true;
                    SetContinuity(aGenS, aDirS);
                }
                if (genSType == ShapeType.Wire)
                {
                    SetContinuity(aGenS, aDirS);
                }
                if (genSType == ShapeType.Face)
                {
                    newShell.Closed(Tool.IsClosed(newShell));
                    Shape temp = SplitShell(newShell);
                    Orientation shellOri = DirectSolid(aGenS, aDirS);
                    lt.Init(temp);
                    if (lt.More())
                    {
                        lt.Next();
                    }
                    if (lt.More())
                    {
                        for (lt.Init(temp); lt.More(); lt.Next())
                        {
                            _builder.Add(_shapes[iGenS, iDirS], lt.Value(), shellOri);
                        }
                    }
                    else
                    {
                        _builder.Add(_shapes[iGenS, iDirS], newShell, shellOri);
                    }
                }
            }
            else if (_dirShapeTool.Type(aDirS) == ShapeType.Wire)
            {
                ShapeType genSType = _genShapeTool.Type(aGenS);
                switch (genSType)
                {
                    case ShapeType.Vertex:
                        _builder.MakeWire(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Edge:
                        _builder.MakeShell(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Wire:
                        _builder.MakeShell(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Face:
                        _builder.MakeCompSolid(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Shell:
                        _builder.MakeCompSolid(ref _shapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Solid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.CompSolid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.Compound:
                        _builder.MakeCompound(ref _shapes[iGenS, iDirS]);
                        break;
                    default:
                        throw new NoSuchObjectException("Unknown Shape");
                }
                bDirS = aDirS;
                for (kt.Init(aDirS); kt.More(); kt.Next())
                {
                    subDirS = kt.Value();
                    if (HasShape(aGenS, subDirS))
                    {
                        int iNewGenS = iGenS;
                        int iNewDirS = _dirShapeTool.Index(subDirS);
                        or = kt.Orientation();
                        newShape = Shape(aGenS, subDirS);
                        _builder.Add(_shapes[iGenS, iDirS], newShape, or);
                        _usedShapes[iNewGenS, iNewDirS] = true;
                    }
                }
            }
            _builtShapes[iGenS, iDirS] = true;
        }
        // Change the "Closed" flag only for Wires and Shells
        if (_shapes[iGenS, iDirS].Type == ShapeType.Wire || _shapes[iGenS, iDirS].Type == ShapeType.Shell)
        {
            // Kerf: frozen. OCCT sets the flag again on each call; a result already published (frozen), or a
            // generating shape's own wire, is written only if the flag changes, and only where a write is taken: after
            // the sweep's working overlay is finished, the caller's wire keeps its flag (the result holds the wire as
            // the sweep left it, which FromSwept gives).
            bool isClosed = Tool.IsClosed(_shapes[iGenS, iDirS]);
            if (_shapes[iGenS, iDirS].ClosedFlag != isClosed && !WorkingOverlay.IsLocked(_shapes[iGenS, iDirS].TShape()!))
            {
                _shapes[iGenS, iDirS].Closed(isClosed);
            }
        }
        return _shapes[iGenS, iDirS];
    }

    /// <summary>The resulting shape generated with aGenS (null if there is none).</summary>
    public Shape Shape(in Shape aGenS)
    {
        if (_genShapeTool.Index(aGenS) != 0 && HasShape(aGenS, _dirWire))
        {
            return Shape(aGenS, _dirWire);
        }
        else
        {
            Shape bidon = new();
            return bidon;
        }
    }

    /// <summary>True if the initial shape aGenS is used in the result shape.</summary>
    public bool IsUsed(in Shape aGenS)
    {
        int iGenS = _genShapeTool.Index(aGenS);
        if (iGenS == 0)
        {
            return false;
        }
        int j;
        bool isBuilt = false;
        bool isUsed = false;
        for (j = 2; j <= _builtShapes.UpperCol(); ++j)
        {
            isBuilt = isBuilt || _builtShapes[iGenS, j];
            isUsed = isUsed || _usedShapes[iGenS, j];
        }
        if (isUsed)
        {
            if (aGenS.Type == ShapeType.Vertex && IsInvariant(aGenS))
            {
                if (_usedShapes[iGenS, 1] || !Closed())
                {
                    return isUsed;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return isUsed;
            }
        }
        //
        if (isBuilt) //&& !IsUsed
        {
            if (!HasShape(aGenS, _dirWire) && !Closed())
            {
                return true;
            }
            else if (aGenS.Type == ShapeType.Vertex && !Closed())
            {
                if (!_builtShapes[iGenS, 1])
                {
                    return true;
                }
            }
        }
        return isUsed;
    }

    /// <summary>True if the shape generated from theS (a sub-shape of the generating shape) is used in the result.</summary>
    public bool GenIsUsed(in Shape aGenS)
    {
        int iGenS = _genShapeTool.Index(aGenS);
        if (iGenS == 0)
        {
            return false;
        }
        if (iGenS == 1)
        {
            return _builtShapes[iGenS, 1];
        }
        else
        {
            return _builtShapes[iGenS, 1] && _usedShapes[iGenS, 1];
        }
    }

    /// <summary>The resulting shape (null if there is none).</summary>
    public Shape Shape()
    {
        if (HasShape(_genShape, _dirWire))
        {
            return Shape(_genShape, _dirWire);
        }
        else
        {
            Shape bidon = new();
            return bidon;
        }
    }

    /// <summary>The resulting shape indexed by the first vertex of myDirWire and myGenShape.</summary>
    public Shape FirstShape()
    {
        Shape result = new();
        if (_dirShapeTool.HasFirstVertex())
        {
            if (HasShape(_genShape, _dirShapeTool.FirstVertex()))
            {
                result = Shape(_genShape, _dirShapeTool.FirstVertex());
            }
        }
        return result;
    }

    /// <summary>The resulting shape indexed by the last vertex of myDirWire and myGenShape.</summary>
    public Shape LastShape()
    {
        Shape result = new();
        if (_dirShapeTool.HasLastVertex())
        {
            if (HasShape(_genShape, _dirShapeTool.LastVertex()))
            {
                result = Shape(_genShape, _dirShapeTool.LastVertex());
            }
        }
        return result;
    }

    /// <summary>The resulting shape indexed by the first vertex of myDirWire and aGenS.</summary>
    public Shape FirstShape(in Shape aGenS)
    {
        Shape result = new();
        if (_dirShapeTool.HasFirstVertex())
        {
            if (HasShape(aGenS, _dirShapeTool.FirstVertex()))
            {
                result = Shape(aGenS, _dirShapeTool.FirstVertex());
            }
        }
        return result;
    }

    /// <summary>The resulting shape indexed by the last vertex of myDirWire and aGenS.</summary>
    public Shape LastShape(in Shape aGenS)
    {
        Shape result = new();
        if (_dirShapeTool.HasLastVertex())
        {
            if (HasShape(aGenS, _dirShapeTool.LastVertex()))
            {
                result = Shape(aGenS, _dirShapeTool.LastVertex());
            }
        }
        return result;
    }

    /// <summary>True if the directing wire is closed.</summary>
    public bool Closed() => _dirWire.Closed();
}
