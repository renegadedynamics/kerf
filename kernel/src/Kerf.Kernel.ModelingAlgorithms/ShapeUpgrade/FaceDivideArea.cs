// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivideArea.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivideArea.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivideArea.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Divides a face into parts of an area no greater than the maximal area, or into a number of parts (splitting by
/// number, optionally with fixed numbers of U and V splits), with <see cref="SplitSurfaceArea"/>; when
/// splitting by area the parts still too large are divided again.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's modifiable references (<see cref="MaxArea"/>, <see cref="NbParts"/>) return
/// <c>ref</c>. The face given is not modified (see <see cref="FaceDivide"/>).
/// </remarks>
internal class FaceDivideArea : FaceDivide
{
    private double _maxArea;
    private int _nbParts;
    private int _unbSplit;
    private int _vnbSplit;
    private bool _isSplittingByNumber;

    /// <summary>Empty constructor (precision 1e-5, no maximal area).</summary>
    public FaceDivideArea()
    {
        _maxArea = Kerf.Engine.Foundation.Precision.Infinite();
        _nbParts = 0;
        _unbSplit = _vnbSplit = -1;
        _isSplittingByNumber = false;
        SetPrecision(1.0e-5);
        SetSplitSurfaceTool(new SplitSurfaceArea());
    }

    /// <summary>Initializes by a face.</summary>
    public FaceDivideArea(in Face f) : this()
    {
        Init(f);
    }

    /// <summary>The maximal area allowed for the faces (-1 when splitting by number).</summary>
    public ref double MaxArea() => ref _maxArea;

    /// <summary>The number of parts expected when splitting by number.</summary>
    public ref int NbParts() => ref _nbParts;

    /// <summary>Sets fixed numbers of splits in U and V (only when splitting by number).</summary>
    public void SetNumbersUVSplits(int nbUsplits, int nbVsplits)
    {
        _unbSplit = nbUsplits;
        _vnbSplit = nbVsplits;
        if (_unbSplit > 0 && _vnbSplit > 0)
        {
            _nbParts = _unbSplit * _vnbSplit;
        }
    }

    /// <summary>
    /// Sets the splitting mode: splitting by number splits the face into about <see cref="NbParts"/> parts close to
    /// squares in 2D.
    /// </summary>
    public void SetSplittingByNumber(bool isSplittingByNumber)
    {
        _isSplittingByNumber = isSplittingByNumber;
    }

    /// <summary>Performs the splitting and computes the resulting shell; true if something was done.</summary>
    public override bool Perform(double area = 0.0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        var gprop = new Properties();

        MassPropertiesBuilder.SurfaceProperties(_face, gprop, Precision());
        double anArea = gprop.Mass();

        int anbParts = 0;
        if (_maxArea == -1)
        {
            anbParts = _nbParts;
            _maxArea = anArea / anbParts;
        }

        if ((anArea - _maxArea) < Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }

        if (anbParts == 0)
        {
            anbParts = RealToInt(Math.Ceiling(anArea / _maxArea));
        }

        var surfTool = GetSplitSurfaceTool() as SplitSurfaceArea;
        if (surfTool is null)
        {
            return false;
        }
        surfTool.NbParts() = anbParts;
        if (_isSplittingByNumber)
        {
            surfTool.SetSplittingIntoSquares(true);
            surfTool.SetNumbersUVSplits(_unbSplit, _vnbSplit);
        }
        if (!base.Perform(anArea))
        {
            return false;
        }

        Shape result = InternalResult();
        if (result.Type == ShapeType.Face)
        {
            return false;
        }
        int status = _status;

        if (!_isSplittingByNumber)
        {
            var expF = new Explorer(result, ShapeType.Face);
            Shape copyRes = result.EmptyCopied();

            bool isModified = false;
            for (; expF.More(); expF.Next())
            {
                Shape sh = Context()!.InternalApply(expF.Current());
                Face face = ShapeCasts.Face(sh);
                Init(face);
                var b = new BrepBuilder();
                if (Perform())
                {
                    isModified = true;
                    Shape res = InternalResult();
                    var expR = new Explorer(res, ShapeType.Face);
                    for (; expR.More(); expR.Next())
                    {
                        b.Add(copyRes, expR.Current());
                    }
                }
                else
                {
                    b.Add(copyRes, face);
                }
            }
            if (isModified)
            {
                if (copyRes.Type == ShapeType.Wire || copyRes.Type == ShapeType.Shell)
                {
                    copyRes.Closed(Tool.IsClosed(copyRes));
                }
                Context()!.Replace(result, copyRes);
            }
        }

        _status |= status;
        _result = Context()!.InternalApply(result);
        return Status(ExtendStatus.Done);
    }
}
