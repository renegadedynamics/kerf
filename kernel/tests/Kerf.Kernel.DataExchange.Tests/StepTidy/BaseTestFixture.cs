// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_BaseTestFixture.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;

namespace Kerf.Kernel.DataExchange.Tests.StepTidy;

/// <summary>A work session on a STEP model, with helpers that add entities to it.</summary>
public abstract class BaseTestFixture
{
    private protected readonly ExchangeWorkSession myWS;

    protected BaseTestFixture()
    {
        myWS = new ExchangeWorkSession();
        StepController.SelectFor(myWS);
        myWS.SetModel(myWS.NormAdaptor()!.NewModel());
    }

    private protected StepCartesianPoint addCartesianPoint(string? theName = null, Xyz? thePoint = null)
    {
        Xyz aPoint = thePoint ?? new Xyz(0.0, 0.0, 0.0);
        var aCartesianPoint = new StepCartesianPoint();
        aCartesianPoint.Init3D(Name(theName), aPoint.X(), aPoint.Y(), aPoint.Z());
        myWS.Model()!.AddWithRefs(aCartesianPoint);
        return aCartesianPoint;
    }

    private protected StepDirection addDirection(string? theName = null, Xyz? theDirection = null)
    {
        Xyz aDir = theDirection ?? new Xyz(0.0, 0.0, 1.0);
        var aDirection = new StepDirection();
        var aDirectionRatios = new NCollection_HArray1<double>(1, 3);
        aDirectionRatios.SetValue(1, aDir.X());
        aDirectionRatios.SetValue(2, aDir.Y());
        aDirectionRatios.SetValue(3, aDir.Z());
        aDirection.Init(Name(theName), aDirectionRatios);
        myWS.Model()!.AddWithRefs(aDirection);
        return aDirection;
    }

    private protected StepVector addVector(string? theName = null, Xyz? theOrientation = null, double aMagnitude = 1.0)
    {
        var aVector = new StepVector();
        aVector.Init(Name(theName), addDirection(null, theOrientation), aMagnitude);
        myWS.Model()!.AddWithRefs(aVector);
        return aVector;
    }

    private protected StepAxis2Placement3d addAxis2Placement3d(string? theName = null, Xyz? theLocation = null, Xyz? theAxis = null, Xyz? theRefDirection = null)
    {
        var aAxis2Placement3d = new StepAxis2Placement3d();
        aAxis2Placement3d.Init(Name(theName), addCartesianPoint(null, theLocation), true, addDirection(null, theAxis), true,
                               addDirection(null, theRefDirection ?? new Xyz(0.0, 1.0, 0.0)));
        myWS.Model()!.AddWithRefs(aAxis2Placement3d);
        return aAxis2Placement3d;
    }

    private protected StepLine addLine(string? theName = null, Xyz? theLocation = null, Xyz? theOrientation = null, double aMagnitude = 1.0)
    {
        var aLine = new StepLine();
        aLine.Init(Name(theName), addCartesianPoint(null, theLocation), addVector(null, theOrientation, aMagnitude));
        myWS.Model()!.AddWithRefs(aLine);
        return aLine;
    }

    private protected StepCircle addCircle(string? theName = null, Xyz? theLocation = null, Xyz? theAxis = null, Xyz? theRefDirection = null, double theRadius = 1.0)
    {
        var aCircle = new StepCircle();
        var aSelector = new StepAxis2Placement();
        aSelector.SetValue(addAxis2Placement3d(null, theLocation, theAxis, theRefDirection));
        aCircle.Init(Name(theName), aSelector, theRadius);
        myWS.Model()!.AddWithRefs(aCircle);
        return aCircle;
    }

    private protected StepPlane addPlane(string? theName = null, Xyz? theLocation = null, Xyz? theAxis = null, Xyz? theRefDirection = null)
    {
        var aPlane = new StepPlane();
        aPlane.Init(Name(theName), addAxis2Placement3d(null, theLocation, theAxis, theRefDirection));
        myWS.Model()!.AddWithRefs(aPlane);
        return aPlane;
    }

    private protected void addToModel(Transient theEntity) => myWS.Model()!.AddWithRefs(theEntity);

    private static TCollection_HAsciiString Name(string? theName)
        => theName is null ? new TCollection_HAsciiString() : new TCollection_HAsciiString(theName);
}
