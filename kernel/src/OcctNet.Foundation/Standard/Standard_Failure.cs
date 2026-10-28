// Port of: src/FoundationClasses/TKernel/Standard/Standard_Failure.hxx
// Port of: src/FoundationClasses/TKernel/Standard/Standard_*.hxx (DEFINE_STANDARD_EXCEPTION)
// Port of: src/FoundationClasses/TKernel/StdFail/StdFail_*.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_VectorWithNullMagnitude.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of OCCT's exceptions. OCCT's class hierarchy is kept, so code can catch a whole family
/// (<see cref="Standard_DomainError"/>) as in C++.
/// </summary>
public class Standard_Failure : Exception
{
    public Standard_Failure() { }

    public Standard_Failure(string? message) : base(message) { }

    public Standard_Failure(string? message, Exception? innerException) : base(message, innerException) { }

    /// <summary>The OCCT name of the exception's type, as <c>DynamicType()->Name()</c> gives it.</summary>
    public string OcctType => GetType().Name;

    /// <summary>The message, or an empty string when there is none, as OCCT's <c>GetMessageString</c>.</summary>
    public string GetMessageString() => base.Message;
}

public class Standard_DomainError : Standard_Failure
{
    public Standard_DomainError() { }
    public Standard_DomainError(string? message) : base(message) { }
}

public class Standard_ConstructionError : Standard_DomainError
{
    public Standard_ConstructionError() { }
    public Standard_ConstructionError(string? message) : base(message) { }
}

public class Standard_DimensionError : Standard_DomainError
{
    public Standard_DimensionError() { }
    public Standard_DimensionError(string? message) : base(message) { }
}

public class Standard_DimensionMismatch : Standard_DimensionError
{
    public Standard_DimensionMismatch() { }
    public Standard_DimensionMismatch(string? message) : base(message) { }
}

public class Standard_ImmutableObject : Standard_DomainError
{
    public Standard_ImmutableObject() { }
    public Standard_ImmutableObject(string? message) : base(message) { }
}

public class Standard_MultiplyDefined : Standard_DomainError
{
    public Standard_MultiplyDefined() { }
    public Standard_MultiplyDefined(string? message) : base(message) { }
}

public class Standard_NoMoreObject : Standard_DomainError
{
    public Standard_NoMoreObject() { }
    public Standard_NoMoreObject(string? message) : base(message) { }
}

public class Standard_NoSuchObject : Standard_DomainError
{
    public Standard_NoSuchObject() { }
    public Standard_NoSuchObject(string? message) : base(message) { }
}

public class Standard_NullObject : Standard_DomainError
{
    public Standard_NullObject() { }
    public Standard_NullObject(string? message) : base(message) { }
}

public class Standard_TypeMismatch : Standard_DomainError
{
    public Standard_TypeMismatch() { }
    public Standard_TypeMismatch(string? message) : base(message) { }
}

public class Standard_RangeError : Standard_DomainError
{
    public Standard_RangeError() { }
    public Standard_RangeError(string? message) : base(message) { }
}

public class Standard_OutOfRange : Standard_RangeError
{
    public Standard_OutOfRange() { }
    public Standard_OutOfRange(string? message) : base(message) { }
}

public class Standard_NegativeValue : Standard_RangeError
{
    public Standard_NegativeValue() { }
    public Standard_NegativeValue(string? message) : base(message) { }
}

public class Standard_NullValue : Standard_RangeError
{
    public Standard_NullValue() { }
    public Standard_NullValue(string? message) : base(message) { }
}

public class Standard_NumericError : Standard_Failure
{
    public Standard_NumericError() { }
    public Standard_NumericError(string? message) : base(message) { }
}

public class Standard_DivideByZero : Standard_NumericError
{
    public Standard_DivideByZero() { }
    public Standard_DivideByZero(string? message) : base(message) { }
}

public class Standard_Overflow : Standard_NumericError
{
    public Standard_Overflow() { }
    public Standard_Overflow(string? message) : base(message) { }
}

public class Standard_Underflow : Standard_NumericError
{
    public Standard_Underflow() { }
    public Standard_Underflow(string? message) : base(message) { }
}

public class Standard_ProgramError : Standard_Failure
{
    public Standard_ProgramError() { }
    public Standard_ProgramError(string? message) : base(message) { }
}

public class Standard_NotImplemented : Standard_ProgramError
{
    public Standard_NotImplemented() { }
    public Standard_NotImplemented(string? message) : base(message) { }
}

public class Standard_AbortiveTransaction : Standard_Failure
{
    public Standard_AbortiveTransaction() { }
    public Standard_AbortiveTransaction(string? message) : base(message) { }
}

public class StdFail_NotDone : Standard_Failure
{
    public StdFail_NotDone() { }
    public StdFail_NotDone(string? message) : base(message) { }
}

public class StdFail_InfiniteSolutions : Standard_Failure
{
    public StdFail_InfiniteSolutions() { }
    public StdFail_InfiniteSolutions(string? message) : base(message) { }
}

public class StdFail_Undefined : Standard_Failure
{
    public StdFail_Undefined() { }
    public StdFail_Undefined(string? message) : base(message) { }
}

public class StdFail_UndefinedDerivative : Standard_DomainError
{
    public StdFail_UndefinedDerivative() { }
    public StdFail_UndefinedDerivative(string? message) : base(message) { }
}

public class StdFail_UndefinedValue : Standard_DomainError
{
    public StdFail_UndefinedValue() { }
    public StdFail_UndefinedValue(string? message) : base(message) { }
}

public class gp_VectorWithNullMagnitude : Standard_DomainError
{
    public gp_VectorWithNullMagnitude() { }
    public gp_VectorWithNullMagnitude(string? message) : base(message) { }
}
